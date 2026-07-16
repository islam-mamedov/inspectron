from __future__ import annotations

import os
import queue
import tempfile
import threading
from dataclasses import dataclass
from pathlib import Path

import rclpy
from inspectron_safety_supervisor.msg import SceneAssessment as SceneAssessmentMessage
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from inspectron.clients.ollama import OllamaVLMClient
from inspectron.clients.openai_compatible import OpenAICompatibleVLMClient
from inspectron.domain import CapturedFrame
from inspectron.site_safety import SceneAssessment
from inspectron.vlm import (
    SITE_SAFETY_RESPONSE_SCHEMA,
    SiteSafetyVLMPerception,
)
from inspectron_perception_bridge.conversion import (
    assessment_to_message,
    failure_assessment,
    make_evidence_id,
    validate_compressed_image,
)


@dataclass(frozen=True, slots=True)
class _FrameJob:
    waypoint: str
    scene_id: str
    evidence_id: str
    view_index: int
    image_suffix: str
    image_data: bytes


@dataclass(frozen=True, slots=True)
class _AnalysisResult:
    assessment: SceneAssessment
    error_message: str | None = None


class _FixtureVLMClient:
    def __init__(self, response_json: str) -> None:
        if not response_json.strip():
            raise ValueError("fixture_response_json cannot be empty for fixture provider")
        self.response_json = response_json

    def generate(
        self,
        *,
        image_path: Path,
        prompt: str,
    ) -> str:
        del image_path
        del prompt
        return self.response_json


class PerceptionBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_bridge")

        self.provider = str(
            self.declare_parameter(
                "provider",
                os.getenv("INSPECTRON_VLM_PROVIDER", "ollama"),
            ).value
        )
        self.base_url = str(
            self.declare_parameter(
                "base_url",
                os.getenv(
                    "INSPECTRON_VLM_BASE_URL",
                    "http://localhost:11434",
                ),
            ).value
        )
        self.model = str(
            self.declare_parameter(
                "model",
                os.getenv("INSPECTRON_VLM_MODEL", "qwen3-vl:8b"),
            ).value
        )
        self.api_key_environment_variable = str(
            self.declare_parameter(
                "api_key_environment_variable",
                "INSPECTRON_VLM_API_KEY",
            ).value
        )
        self.timeout_seconds = float(self.declare_parameter("timeout_seconds", 180.0).value)
        self.max_image_bytes = int(
            self.declare_parameter(
                "max_image_bytes",
                25 * 1024 * 1024,
            ).value
        )
        self.num_ctx = int(self.declare_parameter("num_ctx", 16384).value)
        self.default_waypoint = str(
            self.declare_parameter(
                "default_waypoint",
                "unknown_waypoint",
            ).value
        )
        self.scene_id = str(
            self.declare_parameter(
                "scene_id",
                "live_robot_camera",
            ).value
        )
        self.fixture_response_json = str(
            self.declare_parameter(
                "fixture_response_json",
                "",
            ).value
        )

        self._validate_parameters()

        self._perception = SiteSafetyVLMPerception(self._build_client())

        assessment_qos = QoSProfile(depth=10)
        assessment_qos.reliability = ReliabilityPolicy.RELIABLE

        self._assessment_publisher = self.create_publisher(
            SceneAssessmentMessage,
            "/inspectron/scene_assessment",
            assessment_qos,
        )

        self._image_subscription = self.create_subscription(
            CompressedImage,
            "/inspectron/camera/compressed",
            self._on_image,
            qos_profile_sensor_data,
        )

        self._jobs: queue.Queue[_FrameJob] = queue.Queue(maxsize=1)
        self._results: queue.Queue[_AnalysisResult] = queue.Queue()
        self._stop_event = threading.Event()
        self._sequence = 0

        self._worker = threading.Thread(
            target=self._worker_loop,
            name="inspectron-vlm-worker",
            daemon=True,
        )
        self._worker.start()

        self._result_timer = self.create_timer(
            0.05,
            self._publish_completed_results,
        )

        self.get_logger().info(
            "Perception bridge active: "
            "input=/inspectron/camera/compressed "
            "output=/inspectron/scene_assessment "
            f"provider={self.provider} model={self.model}"
        )

    def _validate_parameters(self) -> None:
        if self.provider not in {
            "ollama",
            "openai",
            "openai_compatible",
            "fixture",
        }:
            raise ValueError("provider must be ollama, openai, openai_compatible, or fixture")

        if not self.model.strip() and self.provider != "fixture":
            raise ValueError("model cannot be empty")

        if self.timeout_seconds <= 0.0:
            raise ValueError("timeout_seconds must be positive")

        if self.max_image_bytes <= 0:
            raise ValueError("max_image_bytes must be positive")

        if self.num_ctx <= 0:
            raise ValueError("num_ctx must be positive")

        if not self.default_waypoint.strip():
            raise ValueError("default_waypoint cannot be empty")

        if not self.scene_id.strip():
            raise ValueError("scene_id cannot be empty")

    def _build_client(self):
        if self.provider == "ollama":
            return OllamaVLMClient(
                base_url=self.base_url,
                model=self.model,
                timeout_seconds=self.timeout_seconds,
                max_image_bytes=self.max_image_bytes,
                num_ctx=self.num_ctx,
                response_schema=SITE_SAFETY_RESPONSE_SCHEMA,
            )

        if self.provider in {"openai", "openai_compatible"}:
            api_key = None

            if self.api_key_environment_variable:
                api_key = os.getenv(self.api_key_environment_variable)

            return OpenAICompatibleVLMClient(
                base_url=self.base_url,
                model=self.model,
                api_key=api_key,
                timeout_seconds=self.timeout_seconds,
                max_image_bytes=self.max_image_bytes,
            )

        return _FixtureVLMClient(self.fixture_response_json)

    def _on_image(self, message: CompressedImage) -> None:
        self._sequence += 1

        waypoint = message.header.frame_id.strip() or self.default_waypoint

        stamp = message.header.stamp

        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        evidence_id = make_evidence_id(
            waypoint=waypoint,
            seconds=stamp.sec,
            nanoseconds=stamp.nanosec,
            sequence=self._sequence,
        )

        try:
            image_data = bytes(message.data)
            image_suffix = validate_compressed_image(
                image_format=message.format,
                data=image_data,
                max_image_bytes=self.max_image_bytes,
            )
        except (TypeError, ValueError) as error:
            self.get_logger().error(
                f"Rejected compressed image evidence_id={evidence_id}: {str(error)[:500]}"
            )
            self._publish_assessment(
                failure_assessment(
                    waypoint=waypoint,
                    evidence_id=evidence_id,
                )
            )
            return

        job = _FrameJob(
            waypoint=waypoint,
            scene_id=self.scene_id,
            evidence_id=evidence_id,
            view_index=self._sequence - 1,
            image_suffix=image_suffix,
            image_data=image_data,
        )

        try:
            self._jobs.put_nowait(job)
        except queue.Full:
            self.get_logger().warning(
                "Dropped camera frame because VLM inference "
                f"is still active: evidence_id={evidence_id}"
            )

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                job = self._jobs.get(timeout=0.1)
            except queue.Empty:
                continue

            image_path: Path | None = None

            try:
                with tempfile.NamedTemporaryFile(
                    suffix=job.image_suffix,
                    prefix="inspectron-",
                    delete=False,
                ) as image_file:
                    image_file.write(job.image_data)
                    image_path = Path(image_file.name)

                frame = CapturedFrame(
                    waypoint=job.waypoint,
                    scene_id=job.scene_id,
                    evidence_id=job.evidence_id,
                    view_index=job.view_index,
                    image_path=str(image_path),
                )

                assessment = self._perception.analyze(frame)
                result = _AnalysisResult(assessment=assessment)
            except Exception as error:
                result = _AnalysisResult(
                    assessment=failure_assessment(
                        waypoint=job.waypoint,
                        evidence_id=job.evidence_id,
                    ),
                    error_message=str(error)[:500],
                )
            finally:
                if image_path is not None:
                    image_path.unlink(missing_ok=True)

                self._jobs.task_done()

            self._results.put(result)

    def _publish_completed_results(self) -> None:
        while True:
            try:
                result = self._results.get_nowait()
            except queue.Empty:
                return

            if result.error_message is not None:
                self.get_logger().error(
                    "VLM inference failed; published fail-closed "
                    f"assessment: {result.error_message}"
                )

            self._publish_assessment(result.assessment)
            self._results.task_done()

    def _publish_assessment(
        self,
        assessment: SceneAssessment,
    ) -> None:
        message = assessment_to_message(
            assessment,
            observed_at=self.get_clock().now().to_msg(),
        )
        self._assessment_publisher.publish(message)

        self.get_logger().info(
            "Published assessment "
            f"evidence_id={message.evidence_id} "
            f"traversability={message.traversability} "
            f"action={message.recommended_action}"
        )

    def destroy_node(self):
        self._stop_event.set()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

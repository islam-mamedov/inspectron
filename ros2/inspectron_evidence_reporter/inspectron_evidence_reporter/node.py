from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path

import rclpy
from inspectron_evidence_msgs.msg import (
    EvidenceCapture,
    ReportStatus,
)
from inspectron_mission_msgs.msg import MissionState
from inspectron_safety_supervisor.msg import (
    PolicyDecision,
    SceneAssessment,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger

from inspectron_evidence_reporter.reporting import (
    MissionReportRecorder,
    create_mission_id,
    ros_time_to_iso,
    utc_now_iso,
)

_RECORDING_STATES = {
    MissionState.STATE_WAITING_FOR_POLICY,
    MissionState.STATE_MOVING,
    MissionState.STATE_PAUSED,
    MissionState.STATE_INSPECTING_CLOSER,
    MissionState.STATE_REROUTING,
    MissionState.STATE_SAFETY_STOPPED,
}

_TERMINAL_OUTCOMES = {
    MissionState.STATE_COMPLETED: "completed",
    MissionState.STATE_ABORTED: "aborted",
    MissionState.STATE_EMERGENCY_STOPPED: ("emergency_stopped"),
}


class EvidenceReporterNode(Node):
    def __init__(self) -> None:
        super().__init__("evidence_reporter")

        self.declare_parameter(
            "output_directory",
            "~/.local/share/inspectron/reports",
        )
        self.declare_parameter(
            "max_image_bytes",
            12 * 1024 * 1024,
        )
        self.declare_parameter("auto_finalize", True)
        self.declare_parameter(
            "write_partial_report",
            True,
        )
        self.declare_parameter("finalize_delay_ms", 250)

        output_value = str(self.get_parameter("output_directory").value).strip()
        self.max_image_bytes = int(self.get_parameter("max_image_bytes").value)
        self.auto_finalize = bool(self.get_parameter("auto_finalize").value)
        self.write_partial_report = bool(self.get_parameter("write_partial_report").value)
        finalize_delay_ms = int(self.get_parameter("finalize_delay_ms").value)

        if not output_value:
            raise ValueError("output_directory cannot be empty")

        if self.max_image_bytes <= 0:
            raise ValueError("max_image_bytes must be positive")

        if finalize_delay_ms < 0:
            raise ValueError("finalize_delay_ms cannot be negative")

        self.output_directory = Path(output_value).expanduser()
        self.finalize_delay_seconds = finalize_delay_ms / 1000.0

        self._recorder: MissionReportRecorder | None = None
        self._last_state_code: int | None = None
        self._pending_outcome: str | None = None
        self._finalize_deadline: float | None = None
        self._recording_errors: list[str] = []
        self._last_report_path = ""
        self._last_evidence_count = 0

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        mission_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_publisher = self.create_publisher(
            ReportStatus,
            "/inspectron/report/status",
            status_qos,
        )

        self.evidence_subscription = self.create_subscription(
            EvidenceCapture,
            "/inspectron/evidence_capture",
            self._on_evidence,
            20,
        )
        self.assessment_subscription = self.create_subscription(
            SceneAssessment,
            "/inspectron/scene_assessment",
            self._on_assessment,
            20,
        )
        self.policy_subscription = self.create_subscription(
            PolicyDecision,
            "/inspectron/policy_decision",
            self._on_policy,
            20,
        )
        self.mission_subscription = self.create_subscription(
            MissionState,
            "/inspectron/mission/state",
            self._on_mission_state,
            mission_qos,
        )

        self.finalize_service = self.create_service(
            Trigger,
            "/inspectron/report/finalize",
            self._on_manual_finalize,
        )

        self.finalize_timer = self.create_timer(
            0.05,
            self._on_finalize_timer,
        )

        self._publish_status(
            ReportStatus.STATUS_IDLE,
            "Evidence reporter is idle",
        )

    def _on_mission_state(
        self,
        message: MissionState,
    ) -> None:
        if self._recorder is not None and self._recorder.finalized:
            if message.state == MissionState.STATE_IDLE:
                self._recorder = None
                self._last_state_code = message.state
                self._pending_outcome = None
                self._finalize_deadline = None
                self._recording_errors.clear()
                self._publish_status(
                    ReportStatus.STATUS_IDLE,
                    "Reporter reset after mission finalization",
                )
                return

            if message.state in _RECORDING_STATES:
                self._recorder = None
                self._recording_errors.clear()

        if self._recorder is None:
            if message.state not in _RECORDING_STATES:
                return

            if not self._start_recorder(message):
                return

        if not self._record_event(
            "mission state",
            lambda recorder: recorder.record_mission_state(
                state=message.state,
                current_waypoint=message.current_waypoint,
                active_goal=message.active_goal,
                waypoint_index=message.waypoint_index,
                waypoint_count=message.waypoint_count,
                last_evidence_id=message.last_evidence_id,
                reason=message.reason,
                motion_authorized=(message.motion_authorized),
                updated_at=_stamp_to_iso(message.updated_at),
            ),
        ):
            return

        self._last_state_code = message.state

        outcome = _TERMINAL_OUTCOMES.get(message.state)

        if outcome is not None and self.auto_finalize:
            self._schedule_finalization(outcome)
        elif message.state == MissionState.STATE_IDLE and self.auto_finalize:
            self._schedule_finalization("reset")

    def _on_evidence(
        self,
        message: EvidenceCapture,
    ) -> None:
        self._record_event(
            "evidence capture",
            lambda recorder: recorder.record_evidence(
                evidence_id=message.evidence_id,
                waypoint=message.waypoint,
                scene_id=message.scene_id,
                view_index=message.view_index,
                image_format=message.image.format,
                image_data=bytes(message.image.data),
                captured_at=_stamp_to_iso(message.image.header.stamp),
            ),
        )

    def _on_assessment(
        self,
        message: SceneAssessment,
    ) -> None:
        self._record_event(
            "scene assessment",
            lambda recorder: recorder.record_assessment(
                evidence_id=message.evidence_id,
                traversability=message.traversability,
                hazards=list(message.hazards),
                recommended_action=(message.recommended_action),
                confidence=message.confidence,
                view_quality=message.view_quality,
                observed_at=_stamp_to_iso(message.observed_at),
            ),
        )

    def _on_policy(
        self,
        message: PolicyDecision,
    ) -> None:
        self._record_event(
            "policy decision",
            lambda recorder: recorder.record_policy_decision(
                evidence_id=message.evidence_id,
                action=message.action,
                reason=message.reason,
                status=message.status,
                model_action_overridden=(message.model_action_overridden),
                source_observed_at=_stamp_to_iso(message.source_observed_at),
                decided_at=_stamp_to_iso(message.decided_at),
            ),
        )

    def _start_recorder(
        self,
        message: MissionState,
    ) -> bool:
        started_at = _stamp_to_iso(message.updated_at)

        for _ in range(4):
            mission_id = create_mission_id()

            try:
                self._recorder = MissionReportRecorder(
                    output_directory=(self.output_directory),
                    mission_id=mission_id,
                    max_image_bytes=(self.max_image_bytes),
                    started_at=started_at,
                )
                break
            except FileExistsError:
                continue
            except Exception as error:
                self._publish_failure(
                    "Could not start evidence report",
                    error,
                )
                return False
        else:
            self._publish_failure(
                "Could not start evidence report",
                RuntimeError("Repeated mission identifier collision"),
            )
            return False

        self._last_state_code = None
        self._pending_outcome = None
        self._finalize_deadline = None
        self._recording_errors.clear()

        self._publish_status(
            ReportStatus.STATUS_RECORDING,
            (f"Recording mission {self._recorder.mission_id}"),
        )
        return True

    def _record_event(
        self,
        description: str,
        operation: Callable[
            [MissionReportRecorder],
            object,
        ],
    ) -> bool:
        recorder = self._recorder

        if recorder is None or recorder.finalized:
            return False

        try:
            operation(recorder)

            if self.write_partial_report:
                recorder.write_partial_report()
        except Exception as error:
            self._recording_errors.append(f"{description}: {type(error).__name__}: {error}")
            self._publish_failure(
                f"Failed to record {description}",
                error,
            )
            return False

        if self._recording_errors:
            self._publish_status(
                ReportStatus.STATUS_FAILED,
                (f"Recording continues after {len(self._recording_errors)} error(s)"),
            )
        else:
            self._publish_status(
                ReportStatus.STATUS_RECORDING,
                f"Recorded {description}",
            )

        return True

    def _schedule_finalization(
        self,
        outcome: str,
    ) -> None:
        self._pending_outcome = outcome
        self._finalize_deadline = time.monotonic() + self.finalize_delay_seconds

        self._publish_status(
            ReportStatus.STATUS_RECORDING,
            (f"Waiting {self.finalize_delay_seconds:.3f}s before finalizing outcome {outcome}"),
        )

    def _on_finalize_timer(self) -> None:
        if (
            self._pending_outcome is None
            or self._finalize_deadline is None
            or time.monotonic() < self._finalize_deadline
        ):
            return

        outcome = self._pending_outcome
        self._pending_outcome = None
        self._finalize_deadline = None
        self._finalize(outcome)

    def _on_manual_finalize(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        recorder = self._recorder

        if recorder is None:
            response.success = False
            response.message = "No active mission report exists"
            return response

        if recorder.finalized:
            response.success = True
            response.message = f"Report already finalized: {self._last_report_path}"
            return response

        self._pending_outcome = None
        self._finalize_deadline = None

        outcome = _TERMINAL_OUTCOMES.get(
            self._last_state_code,
            "manual",
        )
        response.success = self._finalize(outcome)

        if response.success:
            response.message = f"Finalized report: {self._last_report_path}"
        else:
            response.message = "Report finalization failed; inspect /inspectron/report/status"

        return response

    def _finalize(
        self,
        outcome: str,
    ) -> bool:
        recorder = self._recorder

        if recorder is None:
            return False

        try:
            artifacts = recorder.finalize(
                outcome=outcome,
                finalized_at=utc_now_iso(),
            )
        except Exception as error:
            self._publish_failure(
                "Failed to finalize mission report",
                error,
            )
            return False

        self._last_report_path = str(artifacts.json_path)
        self._last_evidence_count = recorder.evidence_count

        if self._recording_errors:
            message = (
                f"Finalized {outcome} report with {len(self._recording_errors)} recording error(s)"
            )
        else:
            message = f"Finalized {outcome} mission report"

        self._publish_status(
            ReportStatus.STATUS_FINALIZED,
            message,
            report_path=self._last_report_path,
        )
        self.get_logger().info(f"{message}: {self._last_report_path}")
        return True

    def _publish_failure(
        self,
        context: str,
        error: Exception,
    ) -> None:
        message = f"{context}: {type(error).__name__}: {str(error)[:500]}"
        self.get_logger().error(message)
        self._publish_status(
            ReportStatus.STATUS_FAILED,
            message,
        )

    def _publish_status(
        self,
        status: int,
        message: str,
        *,
        report_path: str | None = None,
    ) -> None:
        recorder = self._recorder
        status_message = ReportStatus()
        status_message.status = status
        status_message.mission_id = recorder.mission_id if recorder is not None else ""
        status_message.report_path = self._last_report_path if report_path is None else report_path
        status_message.evidence_count = (
            recorder.evidence_count if recorder is not None else self._last_evidence_count
        )
        status_message.message = message
        status_message.updated_at = self.get_clock().now().to_msg()
        self.status_publisher.publish(status_message)

    def destroy_node(self):
        recorder = self._recorder

        if recorder is not None and not recorder.finalized and self.write_partial_report:
            try:
                recorder.write_partial_report()
            except Exception as error:
                self.get_logger().error(f"Failed to write shutdown partial report: {error}")

        return super().destroy_node()


def _stamp_to_iso(stamp) -> str:
    if stamp.sec == 0 and stamp.nanosec == 0:
        return utc_now_iso()

    return ros_time_to_iso(
        stamp.sec,
        stamp.nanosec,
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvidenceReporterNode()

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

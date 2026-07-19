from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from inspectron_evidence_msgs.msg import EvidenceCapture, ReportStatus
from inspectron_mission_msgs.msg import MissionState
from inspectron_safety_supervisor.msg import PolicyDecision, SceneAssessment
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from inspectron_e2e_simulation.scenario_logic import (
    DESIRED_MODE_AUTHORIZED_ONLY,
    CallMissionAbort,
    CallMissionStart,
    CompleteScenario,
    LogNote,
    PublishCameraFrame,
    PublishDesiredVelocity,
    PublishWaypointReached,
    ScenarioDriverCore,
)

TINY_JPEG = b"\xff\xd8\xff\xd9"


class ScenarioSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("scenario_simulator")

        waypoints = list(
            self.declare_parameter(
                "waypoints",
                ["aisle_a", "aisle_b", "aisle_c"],
            ).value
        )
        desired_linear_x = float(self.declare_parameter("desired_linear_x", 0.30).value)
        desired_angular_z = float(self.declare_parameter("desired_angular_z", 0.0).value)
        min_authorized_motion_samples = int(
            self.declare_parameter(
                "min_authorized_motion_samples",
                5,
            ).value
        )
        frame_interval_ms = int(self.declare_parameter("frame_interval_ms", 125).value)
        desired_velocity_mode = str(
            self.declare_parameter(
                "desired_velocity_mode",
                DESIRED_MODE_AUTHORIZED_ONLY,
            ).value
        )
        auto_start = bool(self.declare_parameter("auto_start", False).value)
        abort_on_safety_stop = bool(
            self.declare_parameter(
                "abort_on_safety_stop",
                False,
            ).value
        )
        tick_rate_hz = float(self.declare_parameter("tick_rate_hz", 20.0).value)

        if frame_interval_ms <= 0:
            raise ValueError("frame_interval_ms must be positive")

        if tick_rate_hz <= 0.0:
            raise ValueError("tick_rate_hz must be positive")

        self.core = ScenarioDriverCore(
            waypoints=waypoints,
            desired_linear_x=desired_linear_x,
            desired_angular_z=desired_angular_z,
            min_authorized_motion_samples=min_authorized_motion_samples,
            frame_interval_seconds=frame_interval_ms / 1000.0,
            desired_velocity_mode=desired_velocity_mode,
            auto_start=auto_start,
            abort_on_safety_stop=abort_on_safety_stop,
        )

        self.scenario_complete = False
        self._graph_ready_signaled = False

        transient_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._camera_publisher = self.create_publisher(
            CompressedImage,
            "/inspectron/camera/compressed",
            qos_profile_sensor_data,
        )
        self._desired_publisher = self.create_publisher(
            Twist,
            "/inspectron/desired_cmd_vel",
            10,
        )
        self._waypoint_reached_publisher = self.create_publisher(
            String,
            "/inspectron/mission/waypoint_reached",
            10,
        )

        self._state_subscription = self.create_subscription(
            MissionState,
            "/inspectron/mission/state",
            self._on_mission_state,
            transient_qos,
        )
        self._report_subscription = self.create_subscription(
            ReportStatus,
            "/inspectron/report/status",
            self._on_report_status,
            transient_qos,
        )
        self._policy_subscription = self.create_subscription(
            PolicyDecision,
            "/inspectron/policy_decision",
            self._on_policy,
            transient_qos,
        )
        self._assessment_subscription = self.create_subscription(
            SceneAssessment,
            "/inspectron/scene_assessment",
            self._on_assessment,
            20,
        )
        self._evidence_subscription = self.create_subscription(
            EvidenceCapture,
            "/inspectron/evidence_capture",
            self._on_evidence,
            20,
        )
        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            10,
        )
        self._perception_request_subscription = self.create_subscription(
            Empty,
            "/inspectron/mission/perception_request",
            self._on_perception_request,
            10,
        )

        self._start_client = self.create_client(
            Trigger,
            "/inspectron/mission/start",
        )
        self._abort_client = self.create_client(
            Trigger,
            "/inspectron/mission/abort",
        )

        self._tick_timer = self.create_timer(
            1.0 / tick_rate_hz,
            self._on_tick,
        )

        self.get_logger().info(
            "Scenario simulator active: "
            f"waypoints={len(waypoints)} "
            f"mode={desired_velocity_mode} "
            f"auto_start={auto_start} "
            f"abort_on_safety_stop={abort_on_safety_stop}"
        )

    def _graph_is_ready(self) -> bool:
        core = self.core
        services_ready = True

        if core.auto_start:
            services_ready = self._start_client.service_is_ready()

        if services_ready and core.abort_on_safety_stop:
            services_ready = self._abort_client.service_is_ready()

        return (
            services_ready
            and self._camera_publisher.get_subscription_count() > 0
            and self._desired_publisher.get_subscription_count() > 0
            and self._waypoint_reached_publisher.get_subscription_count() > 0
            and self._state_subscription.get_publisher_count() > 0
            and self._report_subscription.get_publisher_count() > 0
            and self._policy_subscription.get_publisher_count() > 0
            and self._assessment_subscription.get_publisher_count() > 0
            and self._evidence_subscription.get_publisher_count() > 0
            and self._cmd_vel_subscription.get_publisher_count() > 0
        )

    def _on_tick(self) -> None:
        if not self._graph_ready_signaled:
            if not self._graph_is_ready():
                return

            self._graph_ready_signaled = True
            self._execute(self.core.on_graph_ready())
            self.get_logger().info("Simulation graph discovered; scenario driver active")

        self._execute(self.core.on_tick(time.monotonic()))

    def _on_mission_state(self, message: MissionState) -> None:
        self._execute(
            self.core.on_mission_state(
                state=message.state,
                active_goal=message.active_goal,
                motion_authorized=message.motion_authorized,
                last_evidence_id=message.last_evidence_id,
            )
        )

    def _on_report_status(self, message: ReportStatus) -> None:
        self._execute(
            self.core.on_report_status(
                status=message.status,
                report_path=message.report_path,
                message=message.message,
            )
        )

    def _on_policy(self, message: PolicyDecision) -> None:
        self._execute(
            self.core.on_policy_decision(
                action=message.action,
                status=message.status,
                evidence_id=message.evidence_id,
            )
        )

    def _on_assessment(self, message: SceneAssessment) -> None:
        self._execute(self.core.on_assessment(message.evidence_id))

    def _on_evidence(self, message: EvidenceCapture) -> None:
        self._execute(self.core.on_evidence_capture(message.evidence_id))

    def _on_cmd_vel(self, message: Twist) -> None:
        self._execute(
            self.core.on_cmd_vel(
                linear_x=message.linear.x,
                angular_z=message.angular.z,
            )
        )

    def _on_perception_request(self, message: Empty) -> None:
        del message
        self._execute(self.core.on_perception_request())

    def _execute(self, commands: list[object]) -> None:
        for command in commands:
            if isinstance(command, PublishCameraFrame):
                self._publish_camera_frame(command.waypoint)
            elif isinstance(command, PublishDesiredVelocity):
                twist = Twist()
                twist.linear.x = command.linear_x
                twist.angular.z = command.angular_z
                self._desired_publisher.publish(twist)
            elif isinstance(command, PublishWaypointReached):
                self._waypoint_reached_publisher.publish(String(data=command.waypoint))
                self.get_logger().info(f"Reported waypoint reached: {command.waypoint}")
            elif isinstance(command, CallMissionStart):
                self._call_trigger(self._start_client, "start")
            elif isinstance(command, CallMissionAbort):
                self._call_trigger(self._abort_client, "abort")
            elif isinstance(command, LogNote):
                self.get_logger().warning(command.message)
            elif isinstance(command, CompleteScenario):
                self.scenario_complete = True
                self.get_logger().info(f"E2E_SCENARIO_COMPLETE {command.summary}")

    def _publish_camera_frame(self, waypoint: str) -> None:
        message = CompressedImage()
        message.header.frame_id = waypoint
        message.header.stamp = self.get_clock().now().to_msg()
        message.format = "jpeg"
        message.data = TINY_JPEG
        self._camera_publisher.publish(message)

    def _call_trigger(self, client, label: str) -> None:
        if not client.service_is_ready():
            self.get_logger().error(f"Mission {label} service is not available")
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda finished: self._log_trigger_result(label, finished),
        )

    def _log_trigger_result(self, label: str, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Mission {label} call failed: {error}")
            return

        if response.success:
            self.get_logger().info(f"Mission {label} accepted: {response.message}")
        else:
            self.get_logger().error(f"Mission {label} rejected: {response.message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioSimulatorNode()

    try:
        while rclpy.ok() and not node.scenario_complete:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.scenario_complete:
            node.get_logger().info("Scenario driver exiting after completion")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

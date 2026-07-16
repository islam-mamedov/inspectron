from __future__ import annotations

import time

import rclpy
from inspectron_mission_msgs.msg import MissionState as MissionStateMessage
from inspectron_safety_supervisor.msg import PolicyDecision
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from inspectron_mission_orchestrator.state_machine import (
    MissionPhase,
    MissionStateMachine,
    TransitionResult,
)

STATE_TO_MESSAGE = {
    MissionPhase.IDLE: MissionStateMessage.STATE_IDLE,
    MissionPhase.WAITING_FOR_POLICY: (MissionStateMessage.STATE_WAITING_FOR_POLICY),
    MissionPhase.MOVING: MissionStateMessage.STATE_MOVING,
    MissionPhase.PAUSED: MissionStateMessage.STATE_PAUSED,
    MissionPhase.INSPECTING_CLOSER: (MissionStateMessage.STATE_INSPECTING_CLOSER),
    MissionPhase.REROUTING: MissionStateMessage.STATE_REROUTING,
    MissionPhase.SAFETY_STOPPED: (MissionStateMessage.STATE_SAFETY_STOPPED),
    MissionPhase.COMPLETED: MissionStateMessage.STATE_COMPLETED,
    MissionPhase.ABORTED: MissionStateMessage.STATE_ABORTED,
    MissionPhase.EMERGENCY_STOPPED: (MissionStateMessage.STATE_EMERGENCY_STOPPED),
}


class MissionOrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_orchestrator")

        self.declare_parameter(
            "waypoints",
            ["aisle_a", "aisle_b", "aisle_c"],
        )
        self.declare_parameter("policy_timeout_ms", 750)
        self.declare_parameter("reroute_timeout_ms", 5000)
        self.declare_parameter("watchdog_rate_hz", 20.0)

        waypoints = list(self.get_parameter("waypoints").value)
        policy_timeout_ms = int(self.get_parameter("policy_timeout_ms").value)
        reroute_timeout_ms = int(self.get_parameter("reroute_timeout_ms").value)
        watchdog_rate_hz = float(self.get_parameter("watchdog_rate_hz").value)

        if policy_timeout_ms <= 0:
            raise ValueError("policy_timeout_ms must be positive")

        if reroute_timeout_ms <= 0:
            raise ValueError("reroute_timeout_ms must be positive")

        if watchdog_rate_hz <= 0.0:
            raise ValueError("watchdog_rate_hz must be positive")

        self.machine = MissionStateMachine(
            waypoints,
            policy_timeout_seconds=policy_timeout_ms / 1000.0,
            reroute_timeout_seconds=reroute_timeout_ms / 1000.0,
        )

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.state_publisher = self.create_publisher(
            MissionStateMessage,
            "/inspectron/mission/state",
            state_qos,
        )
        self.goal_publisher = self.create_publisher(
            String,
            "/inspectron/mission/waypoint_goal",
            10,
        )
        self.cancel_publisher = self.create_publisher(
            Bool,
            "/inspectron/mission/cancel_motion",
            10,
        )
        self.perception_request_publisher = self.create_publisher(
            Empty,
            "/inspectron/mission/perception_request",
            10,
        )
        self.closer_view_publisher = self.create_publisher(
            Empty,
            "/inspectron/mission/closer_view_request",
            10,
        )
        self.reroute_request_publisher = self.create_publisher(
            String,
            "/inspectron/mission/reroute_request",
            10,
        )

        self.policy_subscription = self.create_subscription(
            PolicyDecision,
            "/inspectron/policy_decision",
            self._on_policy,
            10,
        )
        self.waypoint_reached_subscription = self.create_subscription(
            String,
            "/inspectron/mission/waypoint_reached",
            self._on_waypoint_reached,
            10,
        )
        self.reroute_target_subscription = self.create_subscription(
            String,
            "/inspectron/mission/reroute_target",
            self._on_reroute_target,
            10,
        )
        self.emergency_stop_subscription = self.create_subscription(
            Bool,
            "/inspectron/emergency_stop",
            self._on_emergency_stop,
            10,
        )

        self.start_service = self.create_service(
            Trigger,
            "/inspectron/mission/start",
            self._on_start,
        )
        self.pause_service = self.create_service(
            Trigger,
            "/inspectron/mission/pause",
            self._on_pause,
        )
        self.resume_service = self.create_service(
            Trigger,
            "/inspectron/mission/resume",
            self._on_resume,
        )
        self.abort_service = self.create_service(
            Trigger,
            "/inspectron/mission/abort",
            self._on_abort,
        )
        self.reset_service = self.create_service(
            Trigger,
            "/inspectron/mission/reset",
            self._on_reset,
        )

        self.watchdog_timer = self.create_timer(
            1.0 / watchdog_rate_hz,
            self._on_watchdog,
        )

        self._publish_state()
        self.get_logger().info(f"Mission orchestrator ready with {len(waypoints)} waypoints")

    def _on_start(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        return self._service_response(response, self.machine.start())

    def _on_pause(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        return self._service_response(response, self.machine.pause())

    def _on_resume(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        return self._service_response(
            response,
            self.machine.resume(time.monotonic()),
        )

    def _on_abort(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        return self._service_response(response, self.machine.abort())

    def _on_reset(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        return self._service_response(response, self.machine.reset())

    def _on_policy(self, message: PolicyDecision) -> None:
        result = self.machine.apply_policy(
            action=message.action,
            status=message.status,
            evidence_id=message.evidence_id,
            now=time.monotonic(),
        )
        self._apply_transition(result)

    def _on_waypoint_reached(self, message: String) -> None:
        result = self.machine.waypoint_reached(message.data)
        self._apply_transition(result)

    def _on_reroute_target(self, message: String) -> None:
        result = self.machine.set_reroute_target(message.data)
        self._apply_transition(result)

    def _on_emergency_stop(self, message: Bool) -> None:
        result = self.machine.set_emergency_stop(message.data)
        self._apply_transition(result)

    def _on_watchdog(self) -> None:
        result = self.machine.watchdog(time.monotonic())

        if result is not None:
            self._apply_transition(result)

    def _service_response(
        self,
        response: Trigger.Response,
        result: TransitionResult,
    ) -> Trigger.Response:
        self._apply_transition(result)
        response.success = result.accepted
        response.message = result.message
        return response

    def _apply_transition(
        self,
        result: TransitionResult,
    ) -> None:
        snapshot = self.machine.snapshot

        if result.cancel_motion:
            self.cancel_publisher.publish(Bool(data=True))

        if result.publish_goal and snapshot.active_goal:
            self.goal_publisher.publish(String(data=snapshot.active_goal))

        if result.request_perception:
            self.perception_request_publisher.publish(Empty())

        if result.request_closer_view:
            self.closer_view_publisher.publish(Empty())

        if result.reroute_request is not None:
            self.reroute_request_publisher.publish(String(data=result.reroute_request))

        self._publish_state()

        if result.accepted:
            self.get_logger().info(result.message)
        else:
            self.get_logger().warning(result.message)

    def _publish_state(self) -> None:
        snapshot = self.machine.snapshot

        message = MissionStateMessage()
        message.state = STATE_TO_MESSAGE[snapshot.state]
        message.current_waypoint = snapshot.current_waypoint
        message.active_goal = snapshot.active_goal
        message.waypoint_index = snapshot.waypoint_index
        message.waypoint_count = snapshot.waypoint_count
        message.last_evidence_id = snapshot.last_evidence_id
        message.reason = snapshot.reason
        message.motion_authorized = snapshot.motion_authorized
        message.updated_at = self.get_clock().now().to_msg()

        self.state_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionOrchestratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_publisher.publish(Bool(data=True))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

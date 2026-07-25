from __future__ import annotations

import math
import time

import rclpy
from inspectron_mission_msgs.msg import MissionState as MissionStateMessage
from inspectron_safety_supervisor.msg import PolicyDecision
from rclpy.executors import ExternalShutdownException
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
MAX_REJECTED_FUTURE_POLICY_OBSERVATIONS = 256


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
        self.declare_parameter("state_heartbeat_rate_hz", 4.0)

        waypoints = list(self.get_parameter("waypoints").value)
        policy_timeout_ms = int(self.get_parameter("policy_timeout_ms").value)
        reroute_timeout_ms = int(self.get_parameter("reroute_timeout_ms").value)
        watchdog_rate_hz = float(self.get_parameter("watchdog_rate_hz").value)
        state_heartbeat_rate_hz = float(self.get_parameter("state_heartbeat_rate_hz").value)

        if policy_timeout_ms <= 0:
            raise ValueError("policy_timeout_ms must be positive")

        if reroute_timeout_ms <= 0:
            raise ValueError("reroute_timeout_ms must be positive")

        if not math.isfinite(watchdog_rate_hz) or watchdog_rate_hz <= 0.0:
            raise ValueError("watchdog_rate_hz must be finite and positive")

        if not math.isfinite(state_heartbeat_rate_hz) or state_heartbeat_rate_hz <= 0.0:
            raise ValueError("state_heartbeat_rate_hz must be finite and positive")

        self.machine = MissionStateMachine(
            waypoints,
            policy_timeout_seconds=policy_timeout_ms / 1000.0,
            reroute_timeout_seconds=reroute_timeout_ms / 1000.0,
        )
        self.policy_timeout_seconds = policy_timeout_ms / 1000.0
        self.policy_timeout_ns = policy_timeout_ms * 1_000_000
        self._last_policy_observed_ns = 0
        self._policy_recovery_barrier_ns = 0
        self._rejected_future_policy_observations: set[int] = set()
        self._rejected_future_policy_quarantine_until: float | None = None

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
        self._last_state_message: MissionStateMessage | None = None
        self.state_heartbeat_timer = self.create_timer(
            1.0 / state_heartbeat_rate_hz,
            self._publish_state_heartbeat,
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
        now = time.monotonic()
        source_reference = self._policy_source_reference(
            message,
            monotonic_now=now,
        )
        result = self.machine.apply_policy(
            action=message.action,
            status=(
                message.status if source_reference is not None else PolicyDecision.STATUS_STALE
            ),
            evidence_id=message.evidence_id,
            now=now,
            policy_reference=source_reference,
        )
        self._apply_transition(result)

    def _policy_source_reference(
        self,
        message: PolicyDecision,
        *,
        monotonic_now: float,
    ) -> float | None:
        observed_at = message.source_observed_at
        current_ns = self.get_clock().now().nanoseconds

        if observed_at.sec < 0 or observed_at.nanosec >= 1_000_000_000:
            self._reject_policy_source()
            return None

        observed_ns = observed_at.sec * 1_000_000_000 + observed_at.nanosec
        self._prune_rejected_future_policy_observations(
            current_ns,
            monotonic_now,
        )

        was_rejected_exactly_while_future = observed_ns in self._rejected_future_policy_observations
        future_quarantine_active = self._extend_future_policy_quarantine(
            observed_ns,
            current_ns,
        )
        if was_rejected_exactly_while_future or future_quarantine_active:
            self._reject_cached_future_policy_source()
            return None

        if observed_ns <= 0:
            self._reject_policy_source()
            return None

        if observed_ns > current_ns:
            self._remember_rejected_future_policy_observation(
                observed_ns,
                current_ns,
            )
            self._reject_policy_source()
            return None

        blocked_by_existing_barrier = (
            self._policy_recovery_barrier_ns > 0 and observed_ns <= self._policy_recovery_barrier_ns
        )
        if blocked_by_existing_barrier:
            self._reject_policy_source(
                advance_recovery_barrier=False,
            )
            return None

        age_ns = current_ns - observed_ns

        if age_ns > self.policy_timeout_ns:
            self._reject_policy_source()
            return None

        if observed_ns <= self._last_policy_observed_ns:
            self._reject_policy_source()
            return None

        self._last_policy_observed_ns = observed_ns
        self._policy_recovery_barrier_ns = 0
        self._rejected_future_policy_observations = {
            rejected_ns
            for rejected_ns in (self._rejected_future_policy_observations)
            if rejected_ns > observed_ns
        }
        return monotonic_now - age_ns / 1_000_000_000

    def _prune_rejected_future_policy_observations(
        self,
        current_ns: int,
        monotonic_now: float,
    ) -> None:
        expired_before_ns = current_ns - self.policy_timeout_ns
        retained = {
            observation_ns
            for observation_ns in self._rejected_future_policy_observations
            if (
                observation_ns >= expired_before_ns
                and observation_ns > self._last_policy_observed_ns
            )
        }
        opened_recovery_epoch = len(retained) != len(self._rejected_future_policy_observations)
        self._rejected_future_policy_observations = retained
        if (
            self._rejected_future_policy_quarantine_until is not None
            and monotonic_now > self._rejected_future_policy_quarantine_until
        ):
            self._rejected_future_policy_quarantine_until = None
            opened_recovery_epoch = True
        if opened_recovery_epoch:
            self._policy_recovery_barrier_ns = max(
                self._policy_recovery_barrier_ns,
                current_ns,
            )

    def _remember_rejected_future_policy_observation(
        self,
        observed_ns: int,
        current_ns: int,
        monotonic_now: float | None = None,
    ) -> None:
        if monotonic_now is None:
            monotonic_now = time.monotonic()

        self._prune_rejected_future_policy_observations(
            current_ns,
            monotonic_now,
        )
        if self._rejected_future_policy_quarantine_until is not None:
            return

        self._rejected_future_policy_observations.add(observed_ns)
        if len(self._rejected_future_policy_observations) > MAX_REJECTED_FUTURE_POLICY_OBSERVATIONS:
            self._rejected_future_policy_observations.clear()
            self._rejected_future_policy_quarantine_until = (
                monotonic_now + self.policy_timeout_seconds * 2
            )

    def _extend_future_policy_quarantine(
        self,
        observed_ns: int,
        current_ns: int,
        monotonic_now: float | None = None,
    ) -> bool:
        if self._rejected_future_policy_quarantine_until is None:
            return False

        if observed_ns > current_ns:
            if monotonic_now is None:
                monotonic_now = time.monotonic()
            self._rejected_future_policy_quarantine_until = max(
                self._rejected_future_policy_quarantine_until,
                monotonic_now + self.policy_timeout_seconds * 2,
            )
        return True

    def _reject_cached_future_policy_source(self) -> None:
        self._reject_policy_source(
            advance_recovery_barrier=(self._policy_recovery_barrier_ns == 0),
        )

    def _reject_policy_source(
        self,
        *,
        advance_recovery_barrier: bool = True,
    ) -> None:
        if not advance_recovery_barrier:
            return

        rejection_ns = self.get_clock().now().nanoseconds
        self._policy_recovery_barrier_ns = max(
            self._policy_recovery_barrier_ns,
            rejection_ns,
        )

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

        self._last_state_message = message
        self.state_publisher.publish(message)

    def _publish_state_heartbeat(self) -> None:
        message = self._last_state_message

        if (
            message is not None
            and message.state == MissionStateMessage.STATE_MOVING
            and message.motion_authorized
        ):
            self.state_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionOrchestratorNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cancel_publisher.publish(Bool(data=True))

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

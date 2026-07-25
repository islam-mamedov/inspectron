from __future__ import annotations

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from inspectron_mission_msgs.msg import MissionState
from inspectron_safety_supervisor.msg import PolicyDecision
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger


def generate_test_description():
    orchestrator = launch_ros.actions.Node(
        package="inspectron_mission_orchestrator",
        executable="mission_orchestrator_node",
        name="mission_orchestrator",
        output="screen",
        parameters=[
            {
                "waypoints": ["aisle_a", "aisle_b"],
                "policy_timeout_ms": 1000,
                "reroute_timeout_ms": 2000,
                "watchdog_rate_hz": 20.0,
                "state_heartbeat_rate_hz": 4.0,
            }
        ],
    )

    return launch.LaunchDescription(
        [
            orchestrator,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class MissionOrchestratorGraphTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("mission_orchestrator_graph_test")
        self.states = []

        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.state_subscription = self.node.create_subscription(
            MissionState,
            "/inspectron/mission/state",
            self.states.append,
            state_qos,
        )
        self.policy_publisher = self.node.create_publisher(
            PolicyDecision,
            "/inspectron/policy_decision",
            10,
        )
        self.reached_publisher = self.node.create_publisher(
            String,
            "/inspectron/mission/waypoint_reached",
            10,
        )
        self.start_client = self.node.create_client(
            Trigger,
            "/inspectron/mission/start",
        )
        self.resume_client = self.node.create_client(
            Trigger,
            "/inspectron/mission/resume",
        )
        self.reset_client = self.node.create_client(
            Trigger,
            "/inspectron/mission/reset",
        )

        self.assertTrue(
            self.start_client.wait_for_service(timeout_sec=8.0),
            "mission start service was not discovered",
        )
        self.assertTrue(
            self.resume_client.wait_for_service(timeout_sec=8.0),
            "mission resume service was not discovered",
        )
        self.assertTrue(
            self.reset_client.wait_for_service(timeout_sec=8.0),
            "mission reset service was not discovered",
        )

        self._wait_until(
            lambda: (
                self.policy_publisher.get_subscription_count() > 0
                and self.reached_publisher.get_subscription_count() > 0
                and self.state_subscription.get_publisher_count() > 0
            ),
            timeout_seconds=8.0,
            failure_message=("mission orchestrator graph was not discovered"),
        )
        self._call_trigger(self.reset_client)
        self._wait_for_state(MissionState.STATE_IDLE)
        self.states.clear()

    def tearDown(self):
        self.node.destroy_client(self.reset_client)
        self.node.destroy_client(self.resume_client)
        self.node.destroy_client(self.start_client)
        self.node.destroy_subscription(self.state_subscription)
        self.node.destroy_publisher(self.policy_publisher)
        self.node.destroy_publisher(self.reached_publisher)
        self.node.destroy_node()

    def _wait_until(
        self,
        predicate,
        *,
        timeout_seconds,
        failure_message,
    ):
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if predicate():
                return

        self.fail(failure_message)

    def _wait_for_state(
        self,
        state,
        *,
        active_goal=None,
        timeout_seconds=8.0,
    ):
        self._wait_until(
            lambda: any(
                message.state == state
                and (active_goal is None or message.active_goal == active_goal)
                for message in self.states
            ),
            timeout_seconds=timeout_seconds,
            failure_message=(f"timed out waiting for state {state} with goal {active_goal!r}"),
        )

    def _publish_until_state(
        self,
        publisher,
        message,
        state,
        *,
        active_goal=None,
    ):
        deadline = time.monotonic() + 8.0
        next_publish = 0.0

        while time.monotonic() < deadline:
            now = time.monotonic()

            if now >= next_publish:
                publisher.publish(message)
                next_publish = now + 0.10

            rclpy.spin_once(self.node, timeout_sec=0.03)

            if any(
                item.state == state and (active_goal is None or item.active_goal == active_goal)
                for item in self.states
            ):
                return

        self.fail(f"timed out publishing toward state {state} with goal {active_goal!r}")

    def _publish_once_and_wait_for_state(
        self,
        publisher,
        message,
        state,
        *,
        active_goal=None,
    ):
        state_index = len(self.states)
        publisher.publish(message)
        self._wait_until(
            lambda: any(
                item.state == state and (active_goal is None or item.active_goal == active_goal)
                for item in self.states[state_index:]
            ),
            timeout_seconds=8.0,
            failure_message=(f"timed out waiting for state {state} with goal {active_goal!r}"),
        )

    def _call_trigger(self, client):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=5.0,
        )
        self.assertTrue(future.done())
        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.success)
        return response

    def _start_mission(self):
        self._call_trigger(self.start_client)
        self._wait_for_state(
            MissionState.STATE_WAITING_FOR_POLICY,
            active_goal="aisle_a",
        )

    def _reset_mission(self):
        self._call_trigger(self.reset_client)
        self._wait_for_state(MissionState.STATE_IDLE)
        self.states.clear()

    def _policy(
        self,
        evidence_id,
        *,
        action=PolicyDecision.ACTION_PROCEED,
        source_age_seconds=0.0,
        source_time_ns=None,
        source_stamp=None,
    ):
        message = PolicyDecision()
        message.action = action
        message.reason = PolicyDecision.REASON_CLEAR_PATH
        message.status = PolicyDecision.STATUS_VALID
        message.model_action_overridden = False
        message.evidence_id = evidence_id
        if source_stamp is not None:
            (
                message.source_observed_at.sec,
                message.source_observed_at.nanosec,
            ) = source_stamp
        elif source_time_ns is None:
            source_time_ns = self.node.get_clock().now().nanoseconds - int(
                source_age_seconds * 1_000_000_000
            )
            message.source_observed_at.sec = source_time_ns // 1_000_000_000
            message.source_observed_at.nanosec = source_time_ns % 1_000_000_000
        elif source_time_ns > 0:
            message.source_observed_at.sec = source_time_ns // 1_000_000_000
            message.source_observed_at.nanosec = source_time_ns % 1_000_000_000
        return message

    def test_completes_mission_through_ros_graph(self):
        self._start_mission()

        self._publish_once_and_wait_for_state(
            self.policy_publisher,
            self._policy("aisle_a-1.000000000-000001"),
            MissionState.STATE_MOVING,
            active_goal="aisle_a",
        )

        self._wait_until(
            lambda: any(
                sum(
                    1
                    for candidate in self.states
                    if candidate.state == MissionState.STATE_MOVING
                    and candidate.active_goal == "aisle_a"
                    and candidate.motion_authorized
                    and candidate.updated_at.sec == message.updated_at.sec
                    and candidate.updated_at.nanosec == message.updated_at.nanosec
                )
                >= 2
                for message in self.states
                if message.state == MissionState.STATE_MOVING
                and message.active_goal == "aisle_a"
                and message.motion_authorized
            ),
            timeout_seconds=3.0,
            failure_message="authorized MOVING state was not heartbeated",
        )

        self._publish_until_state(
            self.reached_publisher,
            String(data="aisle_a"),
            MissionState.STATE_WAITING_FOR_POLICY,
            active_goal="aisle_b",
        )

        self._publish_once_and_wait_for_state(
            self.policy_publisher,
            self._policy("aisle_b-2.000000000-000002"),
            MissionState.STATE_MOVING,
            active_goal="aisle_b",
        )

        self._publish_until_state(
            self.reached_publisher,
            String(data="aisle_b"),
            MissionState.STATE_COMPLETED,
        )

        completed = [
            message for message in self.states if message.state == MissionState.STATE_COMPLETED
        ][-1]

        self.assertEqual(completed.waypoint_index, 2)
        self.assertEqual(completed.waypoint_count, 2)
        self.assertFalse(completed.motion_authorized)

    def test_policy_source_time_fails_closed_and_keeps_original_deadline(
        self,
    ):
        settle_deadline = time.monotonic() + 0.65
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.03)

        self._start_mission()
        partially_aged = self._policy(
            "aisle_a-partial-000001",
            source_age_seconds=0.55,
        )
        self._publish_once_and_wait_for_state(
            self.policy_publisher,
            partially_aged,
            MissionState.STATE_MOVING,
            active_goal="aisle_a",
        )

        moving_at = time.monotonic()
        self._wait_for_state(
            MissionState.STATE_SAFETY_STOPPED,
            active_goal="aisle_a",
            timeout_seconds=1.0,
        )
        elapsed = time.monotonic() - moving_at
        self.assertLess(
            elapsed,
            0.75,
            "partially aged policy received a new watchdog lifetime",
        )
        self._reset_mission()

        self._start_mission()
        pre_stop_source_ns = self.node.get_clock().now().nanoseconds
        self._publish_once_and_wait_for_state(
            self.policy_publisher,
            self._policy(
                "aisle_a-barrier-missing-000002",
                source_time_ns=0,
            ),
            MissionState.STATE_SAFETY_STOPPED,
            active_goal="aisle_a",
        )
        post_stop_source_ns = self.node.get_clock().now().nanoseconds

        state_index = len(self.states)
        self.policy_publisher.publish(
            self._policy(
                "aisle_a-barrier-older-000003",
                source_time_ns=pre_stop_source_ns,
            )
        )
        self._wait_until(
            lambda: any(
                message.state == MissionState.STATE_SAFETY_STOPPED
                and message.reason == "Policy decision is invalid or stale"
                for message in self.states[state_index:]
            ),
            timeout_seconds=3.0,
            failure_message="pre-stop in-flight policy was not rejected",
        )

        state_index = len(self.states)
        self.policy_publisher.publish(
            self._policy(
                "aisle_a-barrier-recovery-000004",
                source_time_ns=post_stop_source_ns,
            )
        )
        self._wait_until(
            lambda: any(
                message.state == MissionState.STATE_SAFETY_STOPPED
                and message.reason == "Fresh motion policy stored; explicit resume is required"
                for message in self.states[state_index:]
            ),
            timeout_seconds=3.0,
            failure_message=("post-stop observation was blocked by a later older callback"),
        )

        state_index = len(self.states)
        self._call_trigger(self.resume_client)
        self._wait_until(
            lambda: any(
                message.state == MissionState.STATE_MOVING and message.motion_authorized
                for message in self.states[state_index:]
            ),
            timeout_seconds=3.0,
            failure_message="post-stop observation did not restore resumable authority",
        )
        self._reset_mission()

        rejected_sources = (
            (
                "expired",
                self._policy(
                    "aisle_a-expired-000007",
                    source_age_seconds=1.2,
                ),
            ),
            (
                "negative",
                self._policy(
                    "aisle_a-negative-000005",
                    source_stamp=(-1, 0),
                ),
            ),
            (
                "malformed",
                self._policy(
                    "aisle_a-malformed-000006",
                    source_stamp=(1, 1_000_000_000),
                ),
            ),
        )

        for description, policy in rejected_sources:
            with self.subTest(description=description):
                self._start_mission()
                self._publish_once_and_wait_for_state(
                    self.policy_publisher,
                    policy,
                    MissionState.STATE_SAFETY_STOPPED,
                    active_goal="aisle_a",
                )
                self.assertFalse(
                    [
                        state
                        for state in self.states
                        if state.state == MissionState.STATE_MOVING and state.motion_authorized
                    ],
                    f"{description} policy source authorized motion",
                )
                self._reset_mission()

        self._start_mission()
        future_source_ns = self.node.get_clock().now().nanoseconds + 1_000_000_000
        future_policy = self._policy(
            "aisle_a-future-000008",
            source_time_ns=future_source_ns,
        )
        state_index = len(self.states)
        self.policy_publisher.publish(future_policy)
        first_transition = []

        def find_first_future_transition():
            first_transition[:] = [
                message
                for message in self.states[state_index:]
                if message.state
                in (
                    MissionState.STATE_MOVING,
                    MissionState.STATE_SAFETY_STOPPED,
                )
            ][:1]
            return bool(first_transition)

        self._wait_until(
            find_first_future_transition,
            timeout_seconds=3.0,
            failure_message="future policy produced no mission transition",
        )
        self.assertEqual(
            first_transition[0].state,
            MissionState.STATE_SAFETY_STOPPED,
            "future policy authorized motion before its source time",
        )
        self.assertEqual(
            first_transition[0].reason,
            "Policy decision is invalid or stale",
        )
        self.assertLess(
            self.node.get_clock().now().nanoseconds,
            future_source_ns,
            "future policy was not evaluated until after ROS clock catch-up",
        )
        self._wait_until(
            lambda: self.node.get_clock().now().nanoseconds > future_source_ns,
            timeout_seconds=2.0,
            failure_message="ROS clock did not catch up to the future policy",
        )
        state_index = len(self.states)
        self.policy_publisher.publish(future_policy)
        self._wait_until(
            lambda: len(self.states) > state_index,
            timeout_seconds=3.0,
            failure_message="future policy replay produced no mission transition",
        )
        replay_state = self.states[state_index]
        self.assertEqual(
            replay_state.state,
            MissionState.STATE_SAFETY_STOPPED,
            "future policy replay left the safety-stopped state",
        )
        self.assertEqual(
            replay_state.reason,
            "Policy decision is invalid or stale",
            "future policy replay became fresh authority after clock catch-up",
        )
        self._reset_mission()

        self._start_mission()
        self._publish_once_and_wait_for_state(
            self.policy_publisher,
            self._policy("aisle_a-fresh-000009"),
            MissionState.STATE_MOVING,
            active_goal="aisle_a",
        )

        stop_policy = self._policy(
            "aisle_a-stop-000010",
            action=PolicyDecision.ACTION_STOP,
        )
        stop_source_ns = (
            stop_policy.source_observed_at.sec * 1_000_000_000
            + stop_policy.source_observed_at.nanosec
        )
        self._publish_once_and_wait_for_state(
            self.policy_publisher,
            stop_policy,
            MissionState.STATE_SAFETY_STOPPED,
            active_goal="aisle_a",
        )

        for description, source_time_ns in (
            ("duplicate", stop_source_ns),
            ("older", stop_source_ns - 100_000_000),
        ):
            with self.subTest(description=description):
                state_index = len(self.states)
                self.policy_publisher.publish(
                    self._policy(
                        f"aisle_a-{description}-000011",
                        source_time_ns=source_time_ns,
                    )
                )
                self._wait_until(
                    lambda marker=state_index: any(
                        message.state == MissionState.STATE_SAFETY_STOPPED
                        and message.reason == "Policy decision is invalid or stale"
                        for message in self.states[marker:]
                    ),
                    timeout_seconds=3.0,
                    failure_message=(f"{description} policy observation was not rejected"),
                )

        resume_state_index = len(self.states)
        self._call_trigger(self.resume_client)
        self._wait_until(
            lambda: any(
                message.state == MissionState.STATE_WAITING_FOR_POLICY
                and message.active_goal == "aisle_a"
                for message in self.states[resume_state_index:]
            ),
            timeout_seconds=3.0,
            failure_message=(
                "resume reused replayed authority instead of waiting for a fresh policy"
            ),
        )
        self.assertFalse(
            [
                message
                for message in self.states[resume_state_index:]
                if message.state == MissionState.STATE_MOVING and message.motion_authorized
            ],
            "replayed policy was cached as resumable authority",
        )

        self._publish_once_and_wait_for_state(
            self.policy_publisher,
            self._policy("aisle_a-recovered-000012"),
            MissionState.STATE_MOVING,
            active_goal="aisle_a",
        )


@launch_testing.post_shutdown_test()
class MissionOrchestratorShutdownTest(unittest.TestCase):
    def test_orchestrator_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0],
            process="mission_orchestrator_node",
        )


if __name__ == "__main__":
    unittest.main()

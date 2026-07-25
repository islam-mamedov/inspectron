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

        self.assertTrue(
            self.start_client.wait_for_service(timeout_sec=8.0),
            "mission start service was not discovered",
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

    def tearDown(self):
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

    @staticmethod
    def _policy(evidence_id):
        message = PolicyDecision()
        message.action = PolicyDecision.ACTION_PROCEED
        message.reason = PolicyDecision.REASON_CLEAR_PATH
        message.status = PolicyDecision.STATUS_VALID
        message.model_action_overridden = False
        message.evidence_id = evidence_id
        return message

    def test_completes_mission_through_ros_graph(self):
        future = self.start_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=5.0,
        )

        self.assertTrue(future.done())
        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.success)

        self._wait_for_state(
            MissionState.STATE_WAITING_FOR_POLICY,
            active_goal="aisle_a",
        )

        self._publish_until_state(
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

        self._publish_until_state(
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

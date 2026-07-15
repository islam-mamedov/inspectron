import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist
from inspectron_safety_supervisor.msg import PolicyDecision
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def generate_test_description():
    controller = launch_ros.actions.Node(
        package="inspectron_motion_controller",
        executable="motion_controller_node",
        name="motion_controller",
        output="screen",
        parameters=[
            {
                "policy_timeout_ms": 300,
                "command_timeout_ms": 500,
                "slow_scale": 0.35,
                "max_linear_speed": 0.40,
                "max_angular_speed": 0.80,
                "control_rate_hz": 20.0,
            }
        ],
    )

    return launch.LaunchDescription(
        [
            controller,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class MotionControllerGraphTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("motion_controller_graph_test")
        self.outputs = []

        policy_qos = QoSProfile(depth=1)
        policy_qos.reliability = ReliabilityPolicy.RELIABLE
        policy_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.policy_publisher = self.node.create_publisher(
            PolicyDecision,
            "/inspectron/policy_decision",
            policy_qos,
        )
        self.command_publisher = self.node.create_publisher(
            Twist,
            "/inspectron/desired_cmd_vel",
            10,
        )
        self.output_subscription = self.node.create_subscription(
            Twist,
            "/cmd_vel",
            self.outputs.append,
            10,
        )

        self._wait_until(
            lambda: (
                self.policy_publisher.get_subscription_count() > 0
                and self.command_publisher.get_subscription_count() > 0
            ),
            timeout_seconds=5.0,
            failure_message="controller subscriptions were not discovered",
        )

    def tearDown(self):
        self.node.destroy_subscription(self.output_subscription)
        self.node.destroy_publisher(self.command_publisher)
        self.node.destroy_publisher(self.policy_publisher)
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

    def _wait_for_output(self, predicate, description):
        start_index = len(self.outputs)
        matched = []

        def find_match():
            matched[:] = [command for command in self.outputs[start_index:] if predicate(command)]
            return bool(matched)

        self._wait_until(
            find_match,
            timeout_seconds=5.0,
            failure_message=f"timed out waiting for {description}",
        )
        return matched[-1]

    def _publish_policy(self, action, status=PolicyDecision.STATUS_VALID):
        decision = PolicyDecision()
        decision.status = status
        decision.action = action
        decision.evidence_id = "motion_graph_test"
        self.policy_publisher.publish(decision)

    def _publish_command(self, linear_x=0.20, angular_z=0.40):
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.command_publisher.publish(command)

    @staticmethod
    def _is_zero(command):
        return abs(command.linear.x) < 1e-9 and abs(command.angular.z) < 1e-9

    @staticmethod
    def _matches(command, linear_x, angular_z):
        return abs(command.linear.x - linear_x) < 1e-9 and abs(command.angular.z - angular_z) < 1e-9

    def test_policy_gates_motion_and_watchdogs_fail_closed(self):
        self._wait_for_output(
            self._is_zero,
            "startup zero command",
        )

        self._publish_command()
        self._publish_policy(PolicyDecision.ACTION_PROCEED)

        self._wait_for_output(
            lambda command: self._matches(command, 0.20, 0.40),
            "proceed command",
        )

        self._publish_command()
        self._publish_policy(PolicyDecision.ACTION_SLOW_DOWN)

        self._wait_for_output(
            lambda command: self._matches(command, 0.07, 0.14),
            "scaled slow command",
        )

        self._publish_command()
        self._publish_policy(PolicyDecision.ACTION_STOP)

        self._wait_for_output(
            self._is_zero,
            "policy stop command",
        )

        self._publish_command()
        self._publish_policy(255)

        self._wait_for_output(
            self._is_zero,
            "unknown-action stop command",
        )

        self._publish_command()
        self._publish_policy(
            PolicyDecision.ACTION_PROCEED,
            PolicyDecision.STATUS_INVALID,
        )

        self._wait_for_output(
            self._is_zero,
            "invalid-policy stop command",
        )

        self._publish_command()
        self._publish_policy(PolicyDecision.ACTION_PROCEED)

        self._wait_for_output(
            lambda command: self._matches(command, 0.20, 0.40),
            "motion before policy timeout",
        )

        policy_timeout_deadline = time.monotonic() + 1.0
        policy_timeout_observed = False

        while time.monotonic() < policy_timeout_deadline:
            self._publish_command()
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if self.outputs and self._is_zero(self.outputs[-1]):
                policy_timeout_observed = True
                break

        self.assertTrue(
            policy_timeout_observed,
            "policy watchdog did not publish zero velocity",
        )

        self._publish_command()
        self._publish_policy(PolicyDecision.ACTION_PROCEED)

        self._wait_for_output(
            lambda command: self._matches(command, 0.20, 0.40),
            "motion before command timeout",
        )

        command_timeout_deadline = time.monotonic() + 1.2
        command_timeout_observed = False

        while time.monotonic() < command_timeout_deadline:
            self._publish_policy(PolicyDecision.ACTION_PROCEED)
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if self.outputs and self._is_zero(self.outputs[-1]):
                command_timeout_observed = True
                break

        self.assertTrue(
            command_timeout_observed,
            "command watchdog did not publish zero velocity",
        )

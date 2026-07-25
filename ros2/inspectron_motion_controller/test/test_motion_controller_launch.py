import os
import signal
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from geometry_msgs.msg import Twist
from inspectron_mission_msgs.msg import MissionState
from inspectron_safety_supervisor.msg import PolicyDecision
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

PREFIX = "/test/motion_controller_authority"


def generate_test_description():
    controller = launch_ros.actions.Node(
        package="inspectron_motion_controller",
        executable="motion_controller_node",
        name="motion_controller",
        output="screen",
        remappings=[
            ("/inspectron/policy_decision", f"{PREFIX}/inspectron/policy_decision"),
            ("/inspectron/mission/state", f"{PREFIX}/inspectron/mission/state"),
            ("/inspectron/desired_cmd_vel", f"{PREFIX}/inspectron/desired_cmd_vel"),
            ("/cmd_vel", f"{PREFIX}/cmd_vel"),
        ],
        parameters=[
            {
                "policy_timeout_ms": 600,
                "command_timeout_ms": 500,
                "mission_state_timeout_ms": 1000,
                "slow_scale": 0.35,
                "max_linear_speed": 0.40,
                "max_angular_speed": 0.80,
                "control_rate_hz": 20.0,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                controller,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"controller": controller},
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
            f"{PREFIX}/inspectron/policy_decision",
            policy_qos,
        )
        self.state_publisher = self.node.create_publisher(
            MissionState,
            f"{PREFIX}/inspectron/mission/state",
            policy_qos,
        )
        self.command_publisher = self.node.create_publisher(
            Twist,
            f"{PREFIX}/inspectron/desired_cmd_vel",
            10,
        )
        self.output_subscription = self.node.create_subscription(
            Twist,
            f"{PREFIX}/cmd_vel",
            self.outputs.append,
            10,
        )

        self._wait_until(
            lambda: (
                self.policy_publisher.get_subscription_count() > 0
                and self.state_publisher.get_subscription_count() > 0
                and self.command_publisher.get_subscription_count() > 0
            ),
            timeout_seconds=5.0,
            failure_message="controller subscriptions were not discovered",
        )
        self._assert_live_authority_subscriptions_are_volatile()

    def tearDown(self):
        for _ in range(3):
            self._publish_state(
                MissionState.STATE_IDLE,
                motion_authorized=False,
                evidence_id="",
                active_goal="",
            )
            self._publish_policy(
                PolicyDecision.ACTION_STOP,
                evidence_id="",
            )
            self._publish_command(0.0, 0.0)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.node.destroy_subscription(self.output_subscription)
        self.node.destroy_publisher(self.command_publisher)
        self.node.destroy_publisher(self.state_publisher)
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

    def _assert_live_authority_subscriptions_are_volatile(self):
        for topic in (
            f"{PREFIX}/inspectron/policy_decision",
            f"{PREFIX}/inspectron/mission/state",
        ):
            endpoints = self.node.get_subscriptions_info_by_topic(topic)
            self.assertEqual(
                len(endpoints),
                1,
                f"expected one controller subscription on {topic}",
            )
            self.assertEqual(
                endpoints[0].qos_profile.durability,
                DurabilityPolicy.VOLATILE,
                f"controller authority subscription on {topic} accepts durable history",
            )

    def _publish_policy(
        self,
        action,
        status=PolicyDecision.STATUS_VALID,
        *,
        evidence_id="aisle_a-1.000000000-000001",
        source_age_seconds=0.0,
        source_time_ns=None,
        source_stamp=None,
    ):
        decision = PolicyDecision()
        decision.status = status
        decision.action = action
        decision.evidence_id = evidence_id
        if source_stamp is not None:
            (
                decision.source_observed_at.sec,
                decision.source_observed_at.nanosec,
            ) = source_stamp
        elif source_time_ns is None:
            source_time_ns = self.node.get_clock().now().nanoseconds - int(
                source_age_seconds * 1_000_000_000
            )
            decision.source_observed_at.sec = source_time_ns // 1_000_000_000
            decision.source_observed_at.nanosec = source_time_ns % 1_000_000_000
        elif source_time_ns > 0:
            decision.source_observed_at.sec = source_time_ns // 1_000_000_000
            decision.source_observed_at.nanosec = source_time_ns % 1_000_000_000
        self.policy_publisher.publish(decision)

    def _publish_state(
        self,
        state=MissionState.STATE_MOVING,
        *,
        motion_authorized=True,
        evidence_id="aisle_a-1.000000000-000001",
        active_goal="aisle_a",
    ):
        message = MissionState()
        message.state = state
        message.motion_authorized = motion_authorized
        message.last_evidence_id = evidence_id
        message.active_goal = active_goal
        self.state_publisher.publish(message)

    def _publish_command(self, linear_x=0.20, angular_z=0.40):
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.command_publisher.publish(command)

    def _publish_channels(self, channels, evidence_id):
        if "mission" in channels:
            self._publish_state(evidence_id=evidence_id)
        if "command" in channels:
            self._publish_command()
        if "policy" in channels:
            self._publish_policy(
                PolicyDecision.ACTION_PROCEED,
                evidence_id=evidence_id,
            )

    def _wait_for_authorized_motion(self, evidence_id, description):
        start_index = len(self.outputs)
        deadline = time.monotonic() + 5.0

        while time.monotonic() < deadline:
            self._publish_channels(
                {"policy", "mission", "command"},
                evidence_id,
            )
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if any(self._matches(command, 0.20, 0.40) for command in self.outputs[start_index:]):
                return

        self.fail(f"timed out waiting for {description}")

    def _queue_aged_channel(
        self,
        controller,
        channel,
        evidence_id,
        *,
        age_seconds,
    ):
        controller_pid = controller.process_details["pid"]
        os.kill(controller_pid, signal.SIGSTOP)
        try:
            self._publish_channels({channel}, evidence_id)
            age_deadline = time.monotonic() + age_seconds

            while time.monotonic() < age_deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)

            output_marker = len(self.outputs)
        finally:
            os.kill(controller_pid, signal.SIGCONT)

        return output_marker

    def _publish_fresh_other_channels(self, delayed_channel, evidence_id):
        self._publish_channels(
            {"policy", "mission", "command"} - {delayed_channel},
            evidence_id,
        )

    def _assert_expired_channel_fails_closed(
        self,
        controller,
        channel,
        evidence_id,
    ):
        self._wait_for_authorized_motion(
            evidence_id,
            f"motion before delaying {channel}",
        )
        output_marker = self._queue_aged_channel(
            controller,
            channel,
            evidence_id,
            age_seconds=1.2,
        )
        observation_deadline = time.monotonic() + 0.75

        while time.monotonic() < observation_deadline:
            self._publish_fresh_other_channels(channel, evidence_id)
            rclpy.spin_once(self.node, timeout_sec=0.02)

        denied_outputs = self.outputs[output_marker:]
        self.assertGreaterEqual(
            len(denied_outputs),
            8,
            f"controller produced too few samples after delayed {channel}",
        )
        self.assertTrue(
            all(self._is_zero(command) for command in denied_outputs),
            f"expired delayed {channel} revived motion",
        )

    def _assert_partially_aged_channel_keeps_deadline(
        self,
        controller,
        channel,
        evidence_id,
        *,
        age_seconds,
        expiry_deadline_seconds,
    ):
        self._wait_for_authorized_motion(
            evidence_id,
            f"motion before partially delaying {channel}",
        )
        output_marker = self._queue_aged_channel(
            controller,
            channel,
            evidence_id,
            age_seconds=age_seconds,
        )
        expiry_deadline = time.monotonic() + expiry_deadline_seconds
        scan_index = output_marker
        motion_samples = 0
        zero_after_motion = False

        while time.monotonic() < expiry_deadline:
            self._publish_fresh_other_channels(channel, evidence_id)
            rclpy.spin_once(self.node, timeout_sec=0.02)

            for command in self.outputs[scan_index:]:
                if self._matches(command, 0.20, 0.40):
                    motion_samples += 1
                elif motion_samples >= 2 and self._is_zero(command):
                    zero_after_motion = True
                    break

            scan_index = len(self.outputs)
            if zero_after_motion:
                break

        self.assertGreaterEqual(
            motion_samples,
            2,
            f"partially aged {channel} was not admitted before its original deadline",
        )
        self.assertTrue(
            zero_after_motion,
            f"partially aged {channel} received a new watchdog lifetime",
        )

    @staticmethod
    def _is_zero(command):
        return abs(command.linear.x) < 1e-9 and abs(command.angular.z) < 1e-9

    @staticmethod
    def _matches(command, linear_x, angular_z):
        return abs(command.linear.x - linear_x) < 1e-9 and abs(command.angular.z - angular_z) < 1e-9

    def _assert_zero_while_inputs_stay_fresh(
        self,
        description,
        *,
        evidence_id="aisle_a-1.000000000-000001",
        duration_seconds=0.35,
    ):
        self._wait_for_output(
            self._is_zero,
            f"{description} initial zero command",
        )
        start_index = len(self.outputs)
        deadline = time.monotonic() + duration_seconds

        while time.monotonic() < deadline:
            self._publish_command()
            self._publish_policy(
                PolicyDecision.ACTION_PROCEED,
                evidence_id=evidence_id,
            )
            rclpy.spin_once(self.node, timeout_sec=0.05)

        denied_outputs = self.outputs[start_index:]
        self.assertGreaterEqual(
            len(denied_outputs),
            3,
            f"controller produced too few samples during {description}",
        )
        self.assertTrue(
            all(self._is_zero(command) for command in denied_outputs),
            f"controller allowed motion during {description}",
        )

    def _assert_zero_while_command_stays_fresh(
        self,
        description,
        *,
        duration_seconds=0.35,
    ):
        start_index = len(self.outputs)
        deadline = time.monotonic() + duration_seconds

        while time.monotonic() < deadline:
            self._publish_command()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        denied_outputs = self.outputs[start_index:]
        self.assertGreaterEqual(
            len(denied_outputs),
            3,
            f"controller produced too few samples during {description}",
        )
        self.assertTrue(
            all(self._is_zero(command) for command in denied_outputs),
            f"controller replayed cached authority during {description}",
        )

    def test_policy_gates_motion_and_watchdogs_fail_closed(self):
        self._wait_for_output(
            self._is_zero,
            "startup zero command",
        )

        self._assert_zero_while_command_stays_fresh("controller startup")

        self._publish_command()
        self._publish_policy(PolicyDecision.ACTION_PROCEED)
        self._assert_zero_while_inputs_stay_fresh("missing mission state")

        self._publish_state(
            MissionState.STATE_MOVING,
            motion_authorized=False,
        )
        self._assert_zero_while_inputs_stay_fresh("revoked mission authority")

        self._publish_state(evidence_id="aisle_a-1.100000000-000002")
        self._assert_zero_while_inputs_stay_fresh("mismatched mission evidence")

        self._publish_state(evidence_id="")
        self._assert_zero_while_inputs_stay_fresh(
            "empty correlated evidence",
            evidence_id="",
        )

        self._publish_state(active_goal="")
        self._assert_zero_while_inputs_stay_fresh("missing active goal")

        self._publish_state()
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

        self._publish_state()
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

        self._publish_state()
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

        non_authorizing_states = (
            MissionState.STATE_IDLE,
            MissionState.STATE_WAITING_FOR_POLICY,
            MissionState.STATE_PAUSED,
            MissionState.STATE_INSPECTING_CLOSER,
            MissionState.STATE_REROUTING,
            MissionState.STATE_SAFETY_STOPPED,
            MissionState.STATE_COMPLETED,
            MissionState.STATE_ABORTED,
            MissionState.STATE_EMERGENCY_STOPPED,
            255,
        )

        for state in non_authorizing_states:
            self._publish_state(
                state,
                motion_authorized=True,
            )
            self._assert_zero_while_inputs_stay_fresh(f"mission state {state}")

        self._publish_state(
            MissionState.STATE_MOVING,
            motion_authorized=True,
            evidence_id="aisle_a-2.000000000-000003",
        )
        self._assert_zero_while_inputs_stay_fresh(
            "new mission evidence with old policy",
        )

        self._publish_command()
        self._publish_policy(
            PolicyDecision.ACTION_PROCEED,
            evidence_id="aisle_a-2.000000000-000003",
        )
        self._wait_for_output(
            lambda command: self._matches(command, 0.20, 0.40),
            "motion after matching mission evidence",
        )

        mission_timeout_deadline = time.monotonic() + 1.5
        mission_timeout_observed = False

        while time.monotonic() < mission_timeout_deadline:
            self._publish_command()
            self._publish_policy(
                PolicyDecision.ACTION_PROCEED,
                evidence_id="aisle_a-2.000000000-000003",
            )
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if self.outputs and self._is_zero(self.outputs[-1]):
                mission_timeout_observed = True
                break

        self.assertTrue(
            mission_timeout_observed,
            "mission-state watchdog did not publish zero velocity",
        )

    def test_delayed_inputs_fail_closed_and_keep_original_deadlines(self, controller):
        for index, channel in enumerate(("policy", "mission", "command"), start=4):
            self._assert_expired_channel_fails_closed(
                controller,
                channel,
                f"aisle_a-{index}.000000000-00000{index}",
            )

        partial_cases = (
            ("policy", 0.30, 0.45),
            ("mission", 0.50, 0.75),
            ("command", 0.25, 0.38),
        )
        for index, (channel, age_seconds, expiry_deadline_seconds) in enumerate(
            partial_cases,
            start=7,
        ):
            self._assert_partially_aged_channel_keeps_deadline(
                controller,
                channel,
                f"aisle_a-{index}.000000000-00000{index}",
                age_seconds=age_seconds,
                expiry_deadline_seconds=expiry_deadline_seconds,
            )

    def test_policy_observation_time_fails_closed_and_keeps_deadline(
        self,
    ):
        settle_deadline = time.monotonic() + 0.35
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.03)

        partial_evidence_id = "aisle_a-15.000000000-000015"
        source_time_ns = self.node.get_clock().now().nanoseconds - 200_000_000
        start_index = len(self.outputs)
        self._publish_state(evidence_id=partial_evidence_id)
        self._publish_command()
        self._publish_policy(
            PolicyDecision.ACTION_PROCEED,
            evidence_id=partial_evidence_id,
            source_time_ns=source_time_ns,
        )
        authorization_deadline = time.monotonic() + 0.20

        while time.monotonic() < authorization_deadline:
            self._publish_state(evidence_id=partial_evidence_id)
            self._publish_command()
            rclpy.spin_once(self.node, timeout_sec=0.03)

            if any(self._matches(command, 0.20, 0.40) for command in self.outputs[start_index:]):
                break
        else:
            self.fail("partially aged observation was not admitted before its original deadline")

        moving_at = time.monotonic()
        zero_observed = False
        expiry_deadline = moving_at + 0.55

        while time.monotonic() < expiry_deadline:
            self._publish_state(evidence_id=partial_evidence_id)
            self._publish_command()
            rclpy.spin_once(self.node, timeout_sec=0.03)

            if self.outputs and self._is_zero(self.outputs[-1]):
                zero_observed = True
                break

        self.assertTrue(
            zero_observed,
            "partially aged observation received a new policy lifetime",
        )
        self.assertLess(
            time.monotonic() - moving_at,
            0.48,
            "policy expired later than its source-observation deadline",
        )

        barrier_evidence_id = "aisle_a-16.000000000-000016"
        self._wait_for_authorized_motion(
            barrier_evidence_id,
            "motion before missing-time recovery barrier",
        )
        pre_stop_source_ns = self.node.get_clock().now().nanoseconds
        output_marker = len(self.outputs)
        self._publish_state(evidence_id=barrier_evidence_id)
        self._publish_command()
        self._publish_policy(
            PolicyDecision.ACTION_PROCEED,
            evidence_id=barrier_evidence_id,
            source_time_ns=0,
        )
        rejection_deadline = time.monotonic() + 0.5
        while time.monotonic() < rejection_deadline:
            self._publish_state(evidence_id=barrier_evidence_id)
            self._publish_command()
            rclpy.spin_once(self.node, timeout_sec=0.03)

            if any(self._is_zero(command) for command in self.outputs[output_marker:]):
                break
        else:
            self.fail("missing observation time did not revoke active motion")

        post_stop_source_ns = self.node.get_clock().now().nanoseconds

        output_marker = len(self.outputs)
        older_deadline = time.monotonic() + 0.15
        while time.monotonic() < older_deadline:
            self._publish_state(evidence_id=barrier_evidence_id)
            self._publish_command()
            self._publish_policy(
                PolicyDecision.ACTION_PROCEED,
                evidence_id=barrier_evidence_id,
                source_time_ns=pre_stop_source_ns,
            )
            rclpy.spin_once(self.node, timeout_sec=0.03)

        self.assertTrue(
            self.outputs[output_marker:],
            "pre-stop in-flight policy produced no controller output",
        )
        self.assertTrue(
            all(self._is_zero(command) for command in self.outputs[output_marker:]),
            "pre-stop in-flight policy restored motion",
        )

        output_marker = len(self.outputs)
        self._publish_state(evidence_id=barrier_evidence_id)
        self._publish_command()
        self._publish_policy(
            PolicyDecision.ACTION_PROCEED,
            evidence_id=barrier_evidence_id,
            source_time_ns=post_stop_source_ns,
        )
        recovery_deadline = time.monotonic() + 0.35
        while time.monotonic() < recovery_deadline:
            self._publish_state(evidence_id=barrier_evidence_id)
            self._publish_command()
            rclpy.spin_once(self.node, timeout_sec=0.03)

            if any(self._matches(command, 0.20, 0.40) for command in self.outputs[output_marker:]):
                break
        else:
            self.fail("post-stop observation was blocked by a later older callback")

        future_source_ns = self.node.get_clock().now().nanoseconds + 200_000_000
        rejected_sources = (
            ("future", {"source_time_ns": future_source_ns}),
            ("negative", {"source_stamp": (-1, 0)}),
            (
                "malformed",
                {"source_stamp": (1, 1_000_000_000)},
            ),
            ("expired", {"source_age_seconds": 0.8}),
        )

        for index, (description, source_arguments) in enumerate(
            rejected_sources,
            start=20,
        ):
            with self.subTest(description=description):
                evidence_id = f"aisle_a-{index}.000000000-0000{index}"
                start_index = len(self.outputs)
                deadline = time.monotonic() + 0.4

                while time.monotonic() < deadline:
                    self._publish_state(evidence_id=evidence_id)
                    self._publish_command()
                    self._publish_policy(
                        PolicyDecision.ACTION_PROCEED,
                        evidence_id=evidence_id,
                        **source_arguments,
                    )
                    rclpy.spin_once(self.node, timeout_sec=0.04)

                denied_outputs = self.outputs[start_index:]
                self.assertGreaterEqual(
                    len(denied_outputs),
                    3,
                    f"too few samples for {description} observation time",
                )
                self.assertTrue(
                    all(self._is_zero(command) for command in denied_outputs),
                    f"{description} observation time authorized motion",
                )
                if description == "future":
                    self.assertGreater(
                        self.node.get_clock().now().nanoseconds,
                        future_source_ns,
                        "future replay test ended before ROS clock catch-up",
                    )

        self._wait_for_authorized_motion(
            "aisle_a-24.000000000-000024",
            "fresh recovery after temporal rejection",
        )

        stop_evidence_id = "aisle_a-25.000000000-000025"
        stop_source_ns = self.node.get_clock().now().nanoseconds
        self._publish_state(evidence_id=stop_evidence_id)
        self._publish_command()
        self._publish_policy(
            PolicyDecision.ACTION_STOP,
            evidence_id=stop_evidence_id,
            source_time_ns=stop_source_ns,
        )
        self._wait_for_output(
            self._is_zero,
            "newer policy stop before replay attempts",
        )

        replay_cases = (
            ("duplicate", stop_source_ns),
            ("older", stop_source_ns - 100_000_000),
        )
        for index, (description, replay_source_ns) in enumerate(
            replay_cases,
            start=26,
        ):
            evidence_id = f"aisle_a-{index}.000000000-0000{index}"
            output_marker = len(self.outputs)
            deadline = time.monotonic() + 0.35

            while time.monotonic() < deadline:
                self._publish_state(evidence_id=evidence_id)
                self._publish_command()
                self._publish_policy(
                    PolicyDecision.ACTION_PROCEED,
                    evidence_id=evidence_id,
                    source_time_ns=replay_source_ns,
                )
                rclpy.spin_once(self.node, timeout_sec=0.03)

            denied_outputs = self.outputs[output_marker:]
            self.assertGreaterEqual(len(denied_outputs), 3)
            self.assertTrue(
                all(self._is_zero(command) for command in denied_outputs),
                f"{description} policy observation revived motion",
            )

        self._wait_for_authorized_motion(
            "aisle_a-28.000000000-000028",
            "fresh recovery after policy replay rejection",
        )

import os
import signal
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from builtin_interfaces.msg import Time
from inspectron_safety_supervisor.msg import PolicyDecision, SceneAssessment
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

ASSESSMENT_TIMEOUT_SECONDS = 1.0


def generate_test_description():
    supervisor = launch_ros.actions.Node(
        package="inspectron_safety_supervisor",
        executable="safety_supervisor_node",
        name="safety_supervisor",
        output="screen",
        parameters=[{"assessment_timeout_ms": int(ASSESSMENT_TIMEOUT_SECONDS * 1000)}],
    )

    return (
        launch.LaunchDescription(
            [
                supervisor,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"supervisor": supervisor},
    )


class SafetySupervisorGraphTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("safety_supervisor_graph_test")
        self.decisions = []

        decision_qos = QoSProfile(depth=1)
        decision_qos.reliability = ReliabilityPolicy.RELIABLE
        decision_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.node.create_publisher(
            SceneAssessment,
            "/inspectron/scene_assessment",
            10,
        )
        self.subscription = self.node.create_subscription(
            PolicyDecision,
            "/inspectron/policy_decision",
            self.decisions.append,
            decision_qos,
        )

        self._wait_until(
            lambda: self.publisher.get_subscription_count() > 0,
            timeout_seconds=5.0,
            failure_message="supervisor subscription was not discovered",
        )

    def tearDown(self):
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_publisher(self.publisher)
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

    def _wait_for_decision(self, predicate, description):
        matched = []

        def find_match():
            matched[:] = [decision for decision in self.decisions if predicate(decision)]
            return bool(matched)

        self._wait_until(
            find_match,
            timeout_seconds=5.0,
            failure_message=f"timed out waiting for {description}",
        )
        return matched[-1]

    def _stamp_offset(self, seconds):
        stamp_ns = self.node.get_clock().now().nanoseconds + int(seconds * 1_000_000_000)
        return Time(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )

    @staticmethod
    def _clear_assessment(evidence_id, observed_at):
        message = SceneAssessment()
        message.traversability = SceneAssessment.TRAVERSABILITY_CLEAR
        message.hazards = []
        message.recommended_action = SceneAssessment.ACTION_PROCEED
        message.confidence = 0.95
        message.view_quality = 0.90
        message.observed_at = observed_at
        message.evidence_id = evidence_id
        return message

    @staticmethod
    def _same_stamp(left, right):
        return left.sec == right.sec and left.nanosec == right.nanosec

    def test_valid_invalid_and_stale_assessments_fail_closed(self):
        clear = self._clear_assessment(
            "graph_clear_001",
            self.node.get_clock().now().to_msg(),
        )
        self.publisher.publish(clear)

        clear_decision = self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == clear.evidence_id
                and decision.status == PolicyDecision.STATUS_VALID
            ),
            "valid clear-scene decision",
        )
        self.assertEqual(clear_decision.action, PolicyDecision.ACTION_PROCEED)
        self.assertEqual(clear_decision.reason, PolicyDecision.REASON_CLEAR_PATH)
        self.assertFalse(clear_decision.model_action_overridden)
        self.assertTrue(
            self._same_stamp(
                clear_decision.source_observed_at,
                clear.observed_at,
            )
        )

        invalid = SceneAssessment()
        invalid.traversability = 255
        invalid.hazards = []
        invalid.recommended_action = SceneAssessment.ACTION_PROCEED
        invalid.confidence = 0.95
        invalid.view_quality = 0.90
        invalid.observed_at = self.node.get_clock().now().to_msg()
        invalid.evidence_id = "graph_invalid_001"
        self.publisher.publish(invalid)

        invalid_decision = self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == invalid.evidence_id
                and decision.status == PolicyDecision.STATUS_INVALID
            ),
            "invalid-input stop decision",
        )
        self.assertEqual(invalid_decision.action, PolicyDecision.ACTION_STOP)
        self.assertEqual(
            invalid_decision.reason,
            PolicyDecision.REASON_INVALID_ASSESSMENT,
        )
        self.assertTrue(invalid_decision.model_action_overridden)
        self.assertTrue(
            self._same_stamp(
                invalid_decision.source_observed_at,
                invalid.observed_at,
            )
        )

        stale_decision = self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == invalid.evidence_id
                and decision.status == PolicyDecision.STATUS_STALE
            ),
            "watchdog stop decision",
        )
        self.assertEqual(stale_decision.action, PolicyDecision.ACTION_STOP)
        self.assertEqual(
            stale_decision.reason,
            PolicyDecision.REASON_INVALID_ASSESSMENT,
        )

    def test_zero_future_and_expired_observations_stop_immediately(self):
        temporal_cases = (
            ("expired", self._stamp_offset(-2.0)),
            ("zero", Time()),
            ("negative", Time(sec=-1, nanosec=0)),
            ("malformed", Time(sec=1, nanosec=1_000_000_000)),
        )

        for label, observed_at in temporal_cases:
            evidence_id = f"graph_temporal_{label}"
            assessment = self._clear_assessment(evidence_id, observed_at)
            published_at = time.monotonic()
            self.publisher.publish(assessment)

            decision = self._wait_for_decision(
                lambda candidate, expected_evidence_id=evidence_id: (
                    candidate.evidence_id == expected_evidence_id
                ),
                f"{label} observation stop decision",
            )

            self.assertLess(
                time.monotonic() - published_at,
                ASSESSMENT_TIMEOUT_SECONDS * 0.85,
                f"{label} observation waited for the watchdog instead of stopping immediately",
            )
            self.assertEqual(decision.status, PolicyDecision.STATUS_STALE)
            self.assertEqual(decision.action, PolicyDecision.ACTION_STOP)
            self.assertEqual(
                decision.reason,
                PolicyDecision.REASON_INVALID_ASSESSMENT,
            )
            self.assertTrue(
                self._same_stamp(decision.source_observed_at, observed_at),
            )

        future = self._clear_assessment(
            "graph_temporal_future",
            self._stamp_offset(1.0),
        )
        future_ns = future.observed_at.sec * 1_000_000_000 + future.observed_at.nanosec
        decision_index = len(self.decisions)
        self.publisher.publish(future)
        self._wait_until(
            lambda: any(
                decision.evidence_id == future.evidence_id
                for decision in self.decisions[decision_index:]
            ),
            timeout_seconds=3.0,
            failure_message="future observation produced no decision",
        )
        first_future_decision = next(
            decision
            for decision in self.decisions[decision_index:]
            if decision.evidence_id == future.evidence_id
        )
        self.assertEqual(
            first_future_decision.status,
            PolicyDecision.STATUS_STALE,
            "future observation was accepted before its source time",
        )
        self.assertEqual(
            first_future_decision.action,
            PolicyDecision.ACTION_STOP,
        )
        self.assertLess(
            self.node.get_clock().now().nanoseconds,
            future_ns,
            "future observation was not evaluated until after ROS clock catch-up",
        )

        self._wait_until(
            lambda: self.node.get_clock().now().nanoseconds > future_ns,
            timeout_seconds=2.0,
            failure_message="ROS clock did not catch up to the future observation",
        )
        decision_index = len(self.decisions)
        self.publisher.publish(future)
        self._wait_until(
            lambda: any(
                decision.evidence_id == future.evidence_id
                for decision in self.decisions[decision_index:]
            ),
            timeout_seconds=3.0,
            failure_message="future observation replay produced no decision",
        )
        first_replay_decision = next(
            decision
            for decision in self.decisions[decision_index:]
            if decision.evidence_id == future.evidence_id
        )
        self.assertEqual(
            first_replay_decision.status,
            PolicyDecision.STATUS_STALE,
            "future observation replay became valid after clock catch-up",
        )
        self.assertEqual(
            first_replay_decision.action,
            PolicyDecision.ACTION_STOP,
        )

        recovery = self._clear_assessment(
            "graph_temporal_future_recovery",
            self.node.get_clock().now().to_msg(),
        )
        self.publisher.publish(recovery)
        recovered = self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == recovery.evidence_id
                and decision.status == PolicyDecision.STATUS_VALID
            ),
            "post-future fresh recovery",
        )
        self.assertEqual(recovered.action, PolicyDecision.ACTION_PROCEED)

    def test_partially_aged_observation_keeps_only_remaining_lifetime(self):
        settle_deadline = time.monotonic() + 0.65
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.03)

        evidence_id = "graph_partial_age"
        assessment = self._clear_assessment(
            evidence_id,
            self._stamp_offset(-0.55),
        )
        published_at = time.monotonic()
        self.publisher.publish(assessment)

        valid_decision = self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == evidence_id
                and decision.status == PolicyDecision.STATUS_VALID
            ),
            "partially aged valid decision",
        )
        self.assertEqual(valid_decision.action, PolicyDecision.ACTION_PROCEED)

        stale_decision = self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == evidence_id
                and decision.status == PolicyDecision.STATUS_STALE
            ),
            "partially aged watchdog decision",
        )
        self.assertLess(
            time.monotonic() - published_at,
            ASSESSMENT_TIMEOUT_SECONDS * 0.90,
            "partially aged assessment received a new full watchdog lifetime",
        )
        self.assertTrue(
            self._same_stamp(
                stale_decision.source_observed_at,
                assessment.observed_at,
            )
        )

        recovery = self._clear_assessment(
            "graph_partial_age_recovery",
            self.node.get_clock().now().to_msg(),
        )
        self.publisher.publish(recovery)
        recovered_decision = self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == recovery.evidence_id
                and decision.status == PolicyDecision.STATUS_VALID
            ),
            "fresh recovery decision",
        )
        self.assertEqual(recovered_decision.action, PolicyDecision.ACTION_PROCEED)

    def test_older_or_duplicate_observation_cannot_revive_proceed(self):
        newer_stamp = self.node.get_clock().now().to_msg()
        newer_invalid = SceneAssessment()
        newer_invalid.traversability = 255
        newer_invalid.recommended_action = SceneAssessment.ACTION_PROCEED
        newer_invalid.confidence = 0.95
        newer_invalid.view_quality = 0.90
        newer_invalid.observed_at = newer_stamp
        newer_invalid.evidence_id = "graph_order_newer_invalid"
        self.publisher.publish(newer_invalid)

        self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == newer_invalid.evidence_id
                and decision.status == PolicyDecision.STATUS_INVALID
                and decision.action == PolicyDecision.ACTION_STOP
            ),
            "newer invalid stop decision",
        )

        newer_ns = newer_stamp.sec * 1_000_000_000 + newer_stamp.nanosec
        temporal_cases = (
            ("duplicate", newer_stamp),
            (
                "older",
                Time(
                    sec=(newer_ns - 100_000_000) // 1_000_000_000,
                    nanosec=(newer_ns - 100_000_000) % 1_000_000_000,
                ),
            ),
        )

        for label, observed_at in temporal_cases:
            evidence_id = f"graph_order_{label}_clear"
            self.publisher.publish(self._clear_assessment(evidence_id, observed_at))
            decision = self._wait_for_decision(
                lambda candidate, expected=evidence_id: candidate.evidence_id == expected,
                f"{label} observation stop decision",
            )
            self.assertEqual(decision.status, PolicyDecision.STATUS_STALE)
            self.assertEqual(decision.action, PolicyDecision.ACTION_STOP)
            self.assertFalse(
                [
                    candidate
                    for candidate in self.decisions
                    if candidate.evidence_id == evidence_id
                    and candidate.status == PolicyDecision.STATUS_VALID
                ],
                f"{label} clear assessment revived proceed",
            )

        recovery = self._clear_assessment(
            "graph_order_fresh_recovery",
            self.node.get_clock().now().to_msg(),
        )
        self.publisher.publish(recovery)
        recovered_decision = self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == recovery.evidence_id
                and decision.status == PolicyDecision.STATUS_VALID
            ),
            "fresh ordered recovery decision",
        )
        self.assertEqual(recovered_decision.action, PolicyDecision.ACTION_PROCEED)

    def test_temporal_rejection_requires_post_stop_observation(self):
        in_flight_stamp = self.node.get_clock().now().to_msg()
        missing_time = self._clear_assessment(
            "graph_barrier_missing_time",
            Time(),
        )
        self.publisher.publish(missing_time)
        self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == missing_time.evidence_id
                and decision.status == PolicyDecision.STATUS_STALE
                and decision.action == PolicyDecision.ACTION_STOP
            ),
            "missing-time stop decision",
        )
        post_stop_stamp = self.node.get_clock().now().to_msg()

        older_clear = self._clear_assessment(
            "graph_barrier_older_clear",
            in_flight_stamp,
        )
        self.publisher.publish(older_clear)
        rejected_older = self._wait_for_decision(
            lambda decision: decision.evidence_id == older_clear.evidence_id,
            "pre-stop in-flight observation rejection",
        )
        self.assertEqual(
            rejected_older.status,
            PolicyDecision.STATUS_STALE,
        )
        self.assertEqual(
            rejected_older.action,
            PolicyDecision.ACTION_STOP,
        )
        self.assertFalse(
            [
                decision
                for decision in self.decisions
                if decision.evidence_id == older_clear.evidence_id
                and decision.status == PolicyDecision.STATUS_VALID
            ],
            "pre-stop in-flight observation revived proceed",
        )

        recovery = self._clear_assessment(
            "graph_barrier_fresh_recovery",
            post_stop_stamp,
        )
        self.publisher.publish(recovery)
        recovered = self._wait_for_decision(
            lambda decision: (
                decision.evidence_id == recovery.evidence_id
                and decision.status == PolicyDecision.STATUS_VALID
            ),
            "post-stop fresh recovery",
        )
        self.assertEqual(recovered.action, PolicyDecision.ACTION_PROCEED)

    def test_expired_dds_metadata_stops_fresh_observation(self, supervisor):
        evidence_id = "graph_expired_dds"
        supervisor_pid = supervisor.process_details["pid"]
        os.kill(supervisor_pid, signal.SIGSTOP)

        try:
            assessment = self._clear_assessment(
                evidence_id,
                self._stamp_offset(ASSESSMENT_TIMEOUT_SECONDS),
            )
            self.publisher.publish(assessment)

            deadline = time.monotonic() + ASSESSMENT_TIMEOUT_SECONDS * 1.2
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.02)
        finally:
            os.kill(supervisor_pid, signal.SIGCONT)

        decision = self._wait_for_decision(
            lambda candidate: candidate.evidence_id == evidence_id,
            "expired DDS metadata stop decision",
        )
        self.assertEqual(decision.status, PolicyDecision.STATUS_STALE)
        self.assertEqual(decision.action, PolicyDecision.ACTION_STOP)
        self.assertTrue(
            self._same_stamp(
                decision.source_observed_at,
                assessment.observed_at,
            )
        )

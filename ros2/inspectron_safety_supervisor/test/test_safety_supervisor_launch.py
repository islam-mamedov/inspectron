import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from inspectron_safety_supervisor.msg import PolicyDecision, SceneAssessment
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def generate_test_description():
    supervisor = launch_ros.actions.Node(
        package="inspectron_safety_supervisor",
        executable="safety_supervisor_node",
        name="safety_supervisor",
        output="screen",
        parameters=[{"assessment_timeout_ms": 500}],
    )

    return launch.LaunchDescription(
        [
            supervisor,
            launch_testing.actions.ReadyToTest(),
        ]
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

    def test_valid_invalid_and_stale_assessments_fail_closed(self):
        clear = SceneAssessment()
        clear.traversability = SceneAssessment.TRAVERSABILITY_CLEAR
        clear.hazards = []
        clear.recommended_action = SceneAssessment.ACTION_PROCEED
        clear.confidence = 0.95
        clear.view_quality = 0.90
        clear.observed_at = self.node.get_clock().now().to_msg()
        clear.evidence_id = "graph_clear_001"
        self.publisher.publish(clear)

        clear_decision = self._wait_for_decision(
            lambda decision: decision.evidence_id == clear.evidence_id
            and decision.status == PolicyDecision.STATUS_VALID,
            "valid clear-scene decision",
        )
        self.assertEqual(clear_decision.action, PolicyDecision.ACTION_PROCEED)
        self.assertEqual(clear_decision.reason, PolicyDecision.REASON_CLEAR_PATH)
        self.assertFalse(clear_decision.model_action_overridden)

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
            lambda decision: decision.evidence_id == invalid.evidence_id
            and decision.status == PolicyDecision.STATUS_INVALID,
            "invalid-input stop decision",
        )
        self.assertEqual(invalid_decision.action, PolicyDecision.ACTION_STOP)
        self.assertEqual(
            invalid_decision.reason,
            PolicyDecision.REASON_INVALID_ASSESSMENT,
        )
        self.assertTrue(invalid_decision.model_action_overridden)

        stale_decision = self._wait_for_decision(
            lambda decision: decision.evidence_id == invalid.evidence_id
            and decision.status == PolicyDecision.STATUS_STALE,
            "watchdog stop decision",
        )
        self.assertEqual(stale_decision.action, PolicyDecision.ACTION_STOP)
        self.assertEqual(
            stale_decision.reason,
            PolicyDecision.REASON_INVALID_ASSESSMENT,
        )

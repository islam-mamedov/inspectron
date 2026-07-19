from __future__ import annotations

import json
import time
import unittest
from hashlib import sha256
from pathlib import Path
from tempfile import mkdtemp

import launch
import launch_ros.actions
import launch_testing.actions
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
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

REPORT_ROOT = Path(mkdtemp(prefix="inspectron-report-launch-"))
JPEG = b"\xff\xd8\xff\xd9"
EVIDENCE_ID = "aisle_a-1.000000000-000001"
STATUS_TOPIC = "/test/evidence_reporter/report/status"
MISSION_TOPIC = "/test/evidence_reporter/mission/state"
EVIDENCE_TOPIC = "/test/evidence_reporter/evidence_capture"
ASSESSMENT_TOPIC = "/test/evidence_reporter/scene_assessment"
POLICY_TOPIC = "/test/evidence_reporter/policy_decision"
FINALIZE_SERVICE = "/test/evidence_reporter/report/finalize"


def generate_test_description():
    reporter = launch_ros.actions.Node(
        package="inspectron_evidence_reporter",
        executable="evidence_reporter_node",
        name="evidence_reporter",
        output="screen",
        parameters=[
            {
                "output_directory": str(REPORT_ROOT),
                "max_image_bytes": 1024,
                "auto_finalize": True,
                "write_partial_report": True,
                "finalize_delay_ms": 100,
            }
        ],
        remappings=[
            ("/inspectron/report/status", STATUS_TOPIC),
            ("/inspectron/mission/state", MISSION_TOPIC),
            ("/inspectron/evidence_capture", EVIDENCE_TOPIC),
            ("/inspectron/scene_assessment", ASSESSMENT_TOPIC),
            ("/inspectron/policy_decision", POLICY_TOPIC),
            ("/inspectron/report/finalize", FINALIZE_SERVICE),
        ],
    )

    return launch.LaunchDescription(
        [
            reporter,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class EvidenceReporterGraphTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("evidence_reporter_graph_test")
        self.statuses = []

        transient_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_subscription = self.node.create_subscription(
            ReportStatus,
            STATUS_TOPIC,
            self.statuses.append,
            transient_qos,
        )
        self.mission_publisher = self.node.create_publisher(
            MissionState,
            MISSION_TOPIC,
            transient_qos,
        )
        self.evidence_publisher = self.node.create_publisher(
            EvidenceCapture,
            EVIDENCE_TOPIC,
            10,
        )
        self.assessment_publisher = self.node.create_publisher(
            SceneAssessment,
            ASSESSMENT_TOPIC,
            10,
        )
        self.policy_publisher = self.node.create_publisher(
            PolicyDecision,
            POLICY_TOPIC,
            10,
        )

        self._wait_until(
            lambda: (
                self.status_subscription.get_publisher_count() > 0
                and self.mission_publisher.get_subscription_count() > 0
                and self.evidence_publisher.get_subscription_count() > 0
                and self.assessment_publisher.get_subscription_count() > 0
                and self.policy_publisher.get_subscription_count() > 0
            ),
            timeout_seconds=8.0,
            failure_message=("evidence reporter graph was not discovered"),
        )

    def tearDown(self):
        self.node.destroy_subscription(self.status_subscription)
        self.node.destroy_publisher(self.mission_publisher)
        self.node.destroy_publisher(self.evidence_publisher)
        self.node.destroy_publisher(self.assessment_publisher)
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
            rclpy.spin_once(
                self.node,
                timeout_sec=0.05,
            )

            if predicate():
                return

        self.fail(failure_message)

    def _publish_and_wait(
        self,
        publisher,
        message,
        predicate,
        description,
    ):
        start_index = len(self.statuses)
        publisher.publish(message)

        matched = []

        def status_received():
            matched.extend(status for status in self.statuses[start_index:] if predicate(status))
            return bool(matched)

        self._wait_until(
            status_received,
            timeout_seconds=8.0,
            failure_message=(f"timed out waiting for {description}"),
        )

        return matched[-1]

    def test_writes_complete_verified_mission_report(self):
        started = MissionState()
        started.state = MissionState.STATE_WAITING_FOR_POLICY
        started.current_waypoint = "aisle_a"
        started.active_goal = "aisle_a"
        started.waypoint_index = 0
        started.waypoint_count = 1
        started.last_evidence_id = ""
        started.reason = "Waiting for policy"
        started.motion_authorized = False
        started.updated_at = self.node.get_clock().now().to_msg()

        recording_status = self._publish_and_wait(
            self.mission_publisher,
            started,
            lambda status: (
                status.status == ReportStatus.STATUS_RECORDING
                and status.message == "Recorded mission state"
            ),
            "mission recording startup",
        )

        self.assertTrue(recording_status.mission_id)

        evidence = EvidenceCapture()
        evidence.evidence_id = EVIDENCE_ID
        evidence.waypoint = "aisle_a"
        evidence.scene_id = "warehouse"
        evidence.view_index = 0
        evidence.image.header.stamp = self.node.get_clock().now().to_msg()
        evidence.image.header.frame_id = "aisle_a"
        evidence.image.format = "jpeg"
        evidence.image.data = JPEG

        evidence_status = self._publish_and_wait(
            self.evidence_publisher,
            evidence,
            lambda status: (
                status.status == ReportStatus.STATUS_RECORDING
                and status.message == "Recorded evidence capture"
                and status.evidence_count == 1
            ),
            "evidence capture recording",
        )

        self.assertEqual(
            evidence_status.mission_id,
            recording_status.mission_id,
        )

        assessment = SceneAssessment()
        assessment.traversability = SceneAssessment.TRAVERSABILITY_CLEAR
        assessment.hazards = []
        assessment.recommended_action = SceneAssessment.ACTION_PROCEED
        assessment.confidence = 0.95
        assessment.view_quality = 0.90
        assessment.observed_at = self.node.get_clock().now().to_msg()
        assessment.evidence_id = EVIDENCE_ID

        self._publish_and_wait(
            self.assessment_publisher,
            assessment,
            lambda status: (
                status.status == ReportStatus.STATUS_RECORDING
                and status.message == "Recorded scene assessment"
            ),
            "scene assessment recording",
        )

        policy = PolicyDecision()
        policy.action = PolicyDecision.ACTION_PROCEED
        policy.reason = PolicyDecision.REASON_CLEAR_PATH
        policy.status = PolicyDecision.STATUS_VALID
        policy.model_action_overridden = False
        policy.source_observed_at = assessment.observed_at
        policy.decided_at = self.node.get_clock().now().to_msg()
        policy.evidence_id = EVIDENCE_ID

        self._publish_and_wait(
            self.policy_publisher,
            policy,
            lambda status: (
                status.status == ReportStatus.STATUS_RECORDING
                and status.message == "Recorded policy decision"
            ),
            "policy decision recording",
        )

        completed = MissionState()
        completed.state = MissionState.STATE_COMPLETED
        completed.current_waypoint = "aisle_a"
        completed.active_goal = ""
        completed.waypoint_index = 1
        completed.waypoint_count = 1
        completed.last_evidence_id = EVIDENCE_ID
        completed.reason = "All required waypoints were completed"
        completed.motion_authorized = False
        completed.updated_at = self.node.get_clock().now().to_msg()

        finalized_status = self._publish_and_wait(
            self.mission_publisher,
            completed,
            lambda status: (
                status.status == ReportStatus.STATUS_FINALIZED and bool(status.report_path)
            ),
            "automatic report finalization",
        )

        report_path = Path(finalized_status.report_path)
        markdown_path = report_path.with_name("report.md")

        self.assertTrue(report_path.is_file())
        self.assertTrue(markdown_path.is_file())
        self.assertEqual(
            report_path.parent.parent,
            REPORT_ROOT,
        )
        self.assertEqual(
            finalized_status.evidence_count,
            1,
        )

        report = json.loads(report_path.read_text(encoding="utf-8"))

        self.assertEqual(
            report["mission_id"],
            recording_status.mission_id,
        )
        self.assertEqual(
            report["outcome"],
            "completed",
        )
        self.assertEqual(report["coverage"], 1.0)
        self.assertEqual(
            report["counts"],
            {
                "evidence": 1,
                "assessments": 1,
                "policy_decisions": 1,
                "mission_states": 2,
            },
        )
        self.assertEqual(
            report["unmatched"],
            {
                "assessment_without_evidence": [],
                "evidence_without_assessment": [],
            },
        )
        self.assertTrue(report["integrity"]["verified"])
        self.assertEqual(
            report["integrity"]["verified_evidence_files"],
            1,
        )
        self.assertEqual(
            report["evidence"][0]["sha256"],
            sha256(JPEG).hexdigest(),
        )
        self.assertEqual(
            report["assessments"][0]["recommended_action"],
            "proceed",
        )
        self.assertEqual(
            report["policy_decisions"][0]["status"],
            "valid",
        )

        image_path = report_path.parent / report["evidence"][0]["image_path"]

        self.assertEqual(
            image_path.read_bytes(),
            JPEG,
        )
        self.assertIn(
            "# Inspectron Mission Report",
            markdown_path.read_text(encoding="utf-8"),
        )


if __name__ == "__main__":
    unittest.main()

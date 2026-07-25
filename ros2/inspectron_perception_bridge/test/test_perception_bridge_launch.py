from __future__ import annotations

import json
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from builtin_interfaces.msg import Time
from inspectron_evidence_msgs.msg import EvidenceCapture
from inspectron_safety_supervisor.msg import SceneAssessment
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

CAMERA_TOPIC = "/test/perception_bridge/camera/compressed"
ASSESSMENT_TOPIC = "/test/perception_bridge/scene_assessment"
EVIDENCE_TOPIC = "/test/perception_bridge/evidence_capture"


def generate_test_description():
    fixture_response = json.dumps(
        {
            "traversability": "clear",
            "hazards": [],
            "recommended_action": "proceed",
            "confidence": 0.95,
            "view_quality": 0.90,
        }
    )
    fixture_responses_by_waypoint = json.dumps(
        {
            "blocked_aisle": {
                "traversability": "blocked",
                "hazards": ["debris"],
                "recommended_action": "reroute",
                "confidence": 0.94,
                "view_quality": 0.91,
            },
            "inference_failure": {
                "unexpected": "schema",
            },
        }
    )

    bridge = launch_ros.actions.Node(
        package="inspectron_perception_bridge",
        executable="perception_bridge_node",
        name="perception_bridge",
        output="screen",
        parameters=[
            {
                "provider": "fixture",
                "model": "fixture-model",
                "fixture_response_json": fixture_response,
                "fixture_responses_by_waypoint_json": (fixture_responses_by_waypoint),
                "default_waypoint": "test_waypoint",
                "scene_id": "test_scene",
                "timeout_seconds": 1.0,
                "max_image_bytes": 1024,
            }
        ],
        remappings=[
            ("/inspectron/camera/compressed", CAMERA_TOPIC),
            ("/inspectron/scene_assessment", ASSESSMENT_TOPIC),
            ("/inspectron/evidence_capture", EVIDENCE_TOPIC),
        ],
    )

    return launch.LaunchDescription(
        [
            bridge,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class PerceptionBridgeGraphTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("perception_bridge_graph_test")
        self.assessments = []
        self.evidence_captures = []

        self.image_publisher = self.node.create_publisher(
            CompressedImage,
            CAMERA_TOPIC,
            qos_profile_sensor_data,
        )
        self.assessment_subscription = self.node.create_subscription(
            SceneAssessment,
            ASSESSMENT_TOPIC,
            self.assessments.append,
            10,
        )
        self.evidence_subscription = self.node.create_subscription(
            EvidenceCapture,
            EVIDENCE_TOPIC,
            self.evidence_captures.append,
            10,
        )

        self._wait_until(
            lambda: (
                self.image_publisher.get_subscription_count() > 0
                and self.assessment_subscription.get_publisher_count() > 0
                and self.evidence_subscription.get_publisher_count() > 0
            ),
            timeout_seconds=5.0,
            failure_message=("perception bridge graph was not discovered"),
        )

    def tearDown(self):
        self.node.destroy_subscription(self.assessment_subscription)
        self.node.destroy_subscription(self.evidence_subscription)
        self.node.destroy_publisher(self.image_publisher)
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

    def _publish_until_assessment(
        self,
        *,
        image,
        predicate,
        description,
    ):
        start_index = len(self.assessments)
        deadline = time.monotonic() + 8.0
        next_publish = 0.0

        while time.monotonic() < deadline:
            now = time.monotonic()

            if now >= next_publish:
                self.image_publisher.publish(image)
                next_publish = now + 0.10

            rclpy.spin_once(self.node, timeout_sec=0.02)

            matched = [
                assessment for assessment in self.assessments[start_index:] if predicate(assessment)
            ]

            if matched:
                return matched[-1]

        self.fail(f"timed out waiting for {description}")

    @staticmethod
    def _stamp_tuple(stamp):
        return stamp.sec, stamp.nanosec

    def _matching_capture(self, assessment):
        self._wait_until(
            lambda: any(
                capture.evidence_id == assessment.evidence_id for capture in self.evidence_captures
            ),
            timeout_seconds=5.0,
            failure_message=("matching evidence capture was not published"),
        )

        return next(
            capture
            for capture in self.evidence_captures
            if capture.evidence_id == assessment.evidence_id
        )

    def test_valid_and_invalid_images_publish_safe_assessments(
        self,
    ):
        valid_image = CompressedImage()
        valid_image.header.frame_id = "aisle_a"
        valid_image.header.stamp = Time(sec=10, nanosec=20)
        valid_image.format = "jpeg"
        valid_image.data = b"\xff\xd8\xff\xd9"

        valid_assessment = self._publish_until_assessment(
            image=valid_image,
            predicate=lambda assessment: (
                assessment.traversability == SceneAssessment.TRAVERSABILITY_CLEAR
            ),
            description="valid fixture assessment",
        )

        self.assertEqual(
            valid_assessment.recommended_action,
            SceneAssessment.ACTION_PROCEED,
        )
        self.assertEqual(valid_assessment.confidence, 0.95)
        self.assertEqual(valid_assessment.view_quality, 0.90)
        self.assertTrue(valid_assessment.evidence_id.startswith("aisle_a-"))
        self.assertEqual(
            self._stamp_tuple(valid_assessment.observed_at),
            self._stamp_tuple(valid_image.header.stamp),
        )

        valid_capture = self._matching_capture(valid_assessment)

        self.assertEqual(valid_capture.waypoint, "aisle_a")
        self.assertEqual(valid_capture.scene_id, "test_scene")
        self.assertEqual(
            valid_capture.image.header.frame_id,
            "aisle_a",
        )
        self.assertEqual(valid_capture.image.format, "jpeg")
        self.assertEqual(
            bytes(valid_capture.image.data),
            b"\xff\xd8\xff\xd9",
        )
        self.assertEqual(
            self._stamp_tuple(valid_capture.image.header.stamp),
            self._stamp_tuple(valid_assessment.observed_at),
        )

        invalid_image = CompressedImage()
        invalid_image.header.frame_id = "aisle_b"
        invalid_image.header.stamp = Time(sec=30, nanosec=40)
        invalid_image.format = "jpeg"
        invalid_image.data = b"not-a-jpeg"

        invalid_assessment = self._publish_until_assessment(
            image=invalid_image,
            predicate=lambda assessment: (
                assessment.evidence_id.startswith("aisle_b-")
                and assessment.traversability == SceneAssessment.TRAVERSABILITY_UNKNOWN
            ),
            description="fail-closed invalid-image assessment",
        )

        self.assertEqual(
            invalid_assessment.recommended_action,
            SceneAssessment.ACTION_INSPECT_CLOSER,
        )
        self.assertEqual(invalid_assessment.confidence, 0.0)
        self.assertEqual(invalid_assessment.view_quality, 0.0)
        self.assertTrue(invalid_assessment.evidence_id.startswith("aisle_b-"))
        self.assertEqual(
            self._stamp_tuple(invalid_assessment.observed_at),
            self._stamp_tuple(invalid_image.header.stamp),
        )

        blocked_image = CompressedImage()
        blocked_image.header.frame_id = "blocked_aisle"
        blocked_image.header.stamp = Time(sec=50, nanosec=60)
        blocked_image.format = "jpeg"
        blocked_image.data = b"\xff\xd8\xff\xd9"

        blocked_assessment = self._publish_until_assessment(
            image=blocked_image,
            predicate=lambda assessment: (
                assessment.recommended_action == SceneAssessment.ACTION_REROUTE
            ),
            description="waypoint-specific fixture assessment",
        )

        self.assertEqual(
            blocked_assessment.traversability,
            SceneAssessment.TRAVERSABILITY_BLOCKED,
        )
        self.assertEqual(
            blocked_assessment.recommended_action,
            SceneAssessment.ACTION_REROUTE,
        )
        self.assertEqual(
            list(blocked_assessment.hazards),
            [SceneAssessment.HAZARD_DEBRIS],
        )
        self.assertTrue(blocked_assessment.evidence_id.startswith("blocked_aisle-"))
        self.assertEqual(
            self._stamp_tuple(blocked_assessment.observed_at),
            self._stamp_tuple(blocked_image.header.stamp),
        )

        failed_image = CompressedImage()
        failed_image.header.frame_id = "inference_failure"
        failed_image.header.stamp = Time(sec=70, nanosec=80)
        failed_image.format = "jpeg"
        failed_image.data = b"\xff\xd8\xff\xd9"

        failed_assessment = self._publish_until_assessment(
            image=failed_image,
            predicate=lambda assessment: (
                assessment.evidence_id.startswith("inference_failure-")
                and assessment.traversability == SceneAssessment.TRAVERSABILITY_UNKNOWN
            ),
            description="fail-closed inference assessment",
        )

        self.assertEqual(
            failed_assessment.recommended_action,
            SceneAssessment.ACTION_INSPECT_CLOSER,
        )
        self.assertEqual(
            self._stamp_tuple(failed_assessment.observed_at),
            self._stamp_tuple(failed_image.header.stamp),
        )

        failed_capture = self._matching_capture(failed_assessment)
        self.assertEqual(
            self._stamp_tuple(failed_capture.image.header.stamp),
            self._stamp_tuple(failed_assessment.observed_at),
        )

    def test_zero_camera_stamp_is_preserved_for_fail_closed_admission(
        self,
    ):
        unstamped_image = CompressedImage()
        unstamped_image.header.frame_id = "unstamped_aisle"
        unstamped_image.format = "jpeg"
        unstamped_image.data = b"\xff\xd8\xff\xd9"

        assessment = self._publish_until_assessment(
            image=unstamped_image,
            predicate=lambda message: message.evidence_id.startswith("unstamped_aisle-"),
            description="assessment with missing camera observation time",
        )

        self.assertEqual(
            self._stamp_tuple(assessment.observed_at),
            (0, 0),
        )

        capture = self._matching_capture(assessment)
        self.assertEqual(
            self._stamp_tuple(capture.image.header.stamp),
            self._stamp_tuple(assessment.observed_at),
        )

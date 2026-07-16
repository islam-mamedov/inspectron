from __future__ import annotations

import json
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from inspectron_safety_supervisor.msg import SceneAssessment
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


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
                "default_waypoint": "test_waypoint",
                "scene_id": "test_scene",
                "timeout_seconds": 1.0,
                "max_image_bytes": 1024,
            }
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

        self.image_publisher = self.node.create_publisher(
            CompressedImage,
            "/inspectron/camera/compressed",
            qos_profile_sensor_data,
        )
        self.assessment_subscription = self.node.create_subscription(
            SceneAssessment,
            "/inspectron/scene_assessment",
            self.assessments.append,
            10,
        )

        self._wait_until(
            lambda: (
                self.image_publisher.get_subscription_count() > 0
                and self.assessment_subscription.get_publisher_count() > 0
            ),
            timeout_seconds=5.0,
            failure_message=("perception bridge graph was not discovered"),
        )

    def tearDown(self):
        self.node.destroy_subscription(self.assessment_subscription)
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

    def test_valid_and_invalid_images_publish_safe_assessments(
        self,
    ):
        valid_image = CompressedImage()
        valid_image.header.frame_id = "aisle_a"
        valid_image.header.stamp = self.node.get_clock().now().to_msg()
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

        invalid_image = CompressedImage()
        invalid_image.header.frame_id = "aisle_b"
        invalid_image.header.stamp = self.node.get_clock().now().to_msg()
        invalid_image.format = "jpeg"
        invalid_image.data = b"not-a-jpeg"

        invalid_assessment = self._publish_until_assessment(
            image=invalid_image,
            predicate=lambda assessment: (
                assessment.traversability == SceneAssessment.TRAVERSABILITY_UNKNOWN
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

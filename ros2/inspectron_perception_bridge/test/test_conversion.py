from __future__ import annotations

import unittest

from builtin_interfaces.msg import Time
from inspectron_perception_bridge.conversion import (
    assessment_to_message,
    failure_assessment,
    make_evidence_id,
    validate_compressed_image,
)
from inspectron_safety_supervisor.msg import SceneAssessment as SceneAssessmentMessage

from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
)


class AssessmentConversionTest(unittest.TestCase):
    def test_converts_complete_assessment(self):
        assessment = SceneAssessment(
            waypoint="aisle_a",
            evidence_id="frame-001",
            traversability=Traversability.RESTRICTED,
            hazards=frozenset(
                {
                    HazardType.DEBRIS,
                    HazardType.LIQUID_SPILL,
                }
            ),
            recommended_action=RecommendedAction.SLOW_DOWN,
            confidence=0.82,
            view_quality=0.91,
        )

        message = assessment_to_message(
            assessment,
            observed_at=Time(sec=10, nanosec=20),
        )

        self.assertEqual(
            message.traversability,
            SceneAssessmentMessage.TRAVERSABILITY_RESTRICTED,
        )
        self.assertEqual(
            list(message.hazards),
            [
                SceneAssessmentMessage.HAZARD_DEBRIS,
                SceneAssessmentMessage.HAZARD_LIQUID_SPILL,
            ],
        )
        self.assertEqual(
            message.recommended_action,
            SceneAssessmentMessage.ACTION_SLOW_DOWN,
        )
        self.assertEqual(message.confidence, 0.82)
        self.assertEqual(message.view_quality, 0.91)
        self.assertEqual(message.observed_at.sec, 10)
        self.assertEqual(message.observed_at.nanosec, 20)
        self.assertEqual(message.evidence_id, "frame-001")

    def test_failure_assessment_is_fail_closed(self):
        assessment = failure_assessment(
            waypoint="aisle_b",
            evidence_id="failed-frame",
        )

        self.assertEqual(
            assessment.traversability,
            Traversability.UNKNOWN,
        )
        self.assertEqual(
            assessment.recommended_action,
            RecommendedAction.INSPECT_CLOSER,
        )
        self.assertEqual(assessment.confidence, 0.0)
        self.assertEqual(assessment.view_quality, 0.0)
        self.assertFalse(assessment.hazards)

    def test_accepts_jpeg_and_png_signatures(self):
        self.assertEqual(
            validate_compressed_image(
                image_format="jpeg",
                data=b"\xff\xd8\xff\xd9",
                max_image_bytes=100,
            ),
            ".jpg",
        )
        self.assertEqual(
            validate_compressed_image(
                image_format="rgb8; png compressed",
                data=b"\x89PNG\r\n\x1a\ncontent",
                max_image_bytes=100,
            ),
            ".png",
        )

    def test_rejects_invalid_signature(self):
        with self.assertRaisesRegex(ValueError, "JPEG signature"):
            validate_compressed_image(
                image_format="jpeg",
                data=b"not-a-jpeg",
                max_image_bytes=100,
            )

    def test_rejects_oversized_image(self):
        with self.assertRaisesRegex(ValueError, "exceeds"):
            validate_compressed_image(
                image_format="jpeg",
                data=b"\xff\xd8\xff\xd9",
                max_image_bytes=3,
            )

    def test_creates_sanitized_evidence_id(self):
        evidence_id = make_evidence_id(
            waypoint="aisle A/camera",
            seconds=12,
            nanoseconds=34,
            sequence=5,
        )

        self.assertEqual(
            evidence_id,
            "aisle_A_camera-12.000000034-000005",
        )


if __name__ == "__main__":
    unittest.main()

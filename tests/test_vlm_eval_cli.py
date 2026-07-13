from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
)
from inspectron.vlm_eval_cli import (
    SafetyEvaluationSample,
    evaluate_samples,
)


class FakeSafetyPerception:
    def __init__(
        self,
        assessments: dict[str, SceneAssessment],
    ) -> None:
        self.assessments = assessments

    def analyze(
        self,
        frame: object,
    ) -> SceneAssessment:
        return self.assessments[frame.evidence_id]


class SafetyEvaluationTests(unittest.TestCase):
    def test_policy_reduces_unsafe_motion(self) -> None:
        samples = [
            SafetyEvaluationSample(
                sample_id="human_1",
                image="human.jpg",
                traversability=(Traversability.RESTRICTED),
                hazards=frozenset({HazardType.HUMAN_IN_PATH}),
                expected_action=(RecommendedAction.STOP),
            ),
            SafetyEvaluationSample(
                sample_id="clear_1",
                image="clear.jpg",
                traversability=Traversability.CLEAR,
                hazards=frozenset(),
                expected_action=(RecommendedAction.PROCEED),
            ),
        ]

        assessments = {
            "human_1": SceneAssessment(
                waypoint="evaluation_0000",
                evidence_id="human_1",
                traversability=Traversability.CLEAR,
                hazards=frozenset({HazardType.HUMAN_IN_PATH}),
                recommended_action=(RecommendedAction.PROCEED),
                confidence=0.90,
                view_quality=0.90,
            ),
            "clear_1": SceneAssessment(
                waypoint="evaluation_0001",
                evidence_id="clear_1",
                traversability=Traversability.CLEAR,
                hazards=frozenset(),
                recommended_action=(RecommendedAction.PROCEED),
                confidence=0.90,
                view_quality=0.90,
            ),
        }

        clock_values = iter([0.0, 1.0, 1.0, 3.0])

        with TemporaryDirectory() as directory:
            data_root = Path(directory)

            report = evaluate_samples(
                samples=samples,
                data_root=data_root,
                perception=FakeSafetyPerception(assessments),
                model_name="fake-vlm",
                clock=lambda: next(clock_values),
            )

        self.assertEqual(
            report["traversability_accuracy"],
            0.5,
        )
        self.assertEqual(
            report["hazard_micro_f1"],
            1.0,
        )
        self.assertEqual(
            report["model_action_accuracy"],
            0.5,
        )
        self.assertEqual(
            report["enforced_action_accuracy"],
            1.0,
        )
        self.assertEqual(
            report["policy_override_rate"],
            0.5,
        )
        self.assertEqual(
            report["model_unsafe_motion_count"],
            1,
        )
        self.assertEqual(
            report["enforced_unsafe_motion_count"],
            0,
        )
        self.assertEqual(
            report["mean_latency_seconds"],
            1.5,
        )

    def test_manifest_rejects_unsafe_action(
        self,
    ) -> None:
        from inspectron.vlm_eval_cli import (
            load_manifest,
        )

        with TemporaryDirectory() as directory:
            manifest = Path(directory) / "manifest.json"

            manifest.write_text(
                """
                [
                  {
                    "id": "blocked_1",
                    "image": "blocked.jpg",
                    "traversability": "blocked",
                    "hazards": ["debris"],
                    "expected_action": "proceed"
                  }
                ]
                """,
                encoding="utf-8",
            )

            with self.assertRaises(ValueError):
                load_manifest(manifest)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import json
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


class PartiallyFailingSafetyPerception(FakeSafetyPerception):
    def __init__(
        self,
        assessments: dict[str, SceneAssessment],
        failing_ids: frozenset[str],
    ) -> None:
        super().__init__(assessments)
        self.failing_ids = failing_ids

    def analyze(
        self,
        frame: object,
    ) -> SceneAssessment:
        if frame.evidence_id in self.failing_ids:
            raise RuntimeError(f"Inference failed for {frame.evidence_id}")

        return super().analyze(frame)


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

    def test_reports_per_hazard_metrics_for_missing_and_extra_predictions(
        self,
    ) -> None:
        samples = [
            SafetyEvaluationSample(
                sample_id="debris_with_extra",
                image="debris_with_extra.jpg",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset({HazardType.DEBRIS}),
                expected_action=RecommendedAction.SLOW_DOWN,
            ),
            SafetyEvaluationSample(
                sample_id="debris_missed",
                image="debris_missed.jpg",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset({HazardType.DEBRIS}),
                expected_action=RecommendedAction.SLOW_DOWN,
            ),
        ]

        assessments = {
            "debris_with_extra": SceneAssessment(
                waypoint="evaluation_0000",
                evidence_id="debris_with_extra",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset(
                    {
                        HazardType.DEBRIS,
                        HazardType.LIQUID_SPILL,
                    }
                ),
                recommended_action=RecommendedAction.SLOW_DOWN,
                confidence=0.90,
                view_quality=0.90,
            ),
            "debris_missed": SceneAssessment(
                waypoint="evaluation_0001",
                evidence_id="debris_missed",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset(),
                recommended_action=RecommendedAction.SLOW_DOWN,
                confidence=0.90,
                view_quality=0.90,
            ),
        }

        clock_values = iter([0.0, 1.0, 1.0, 2.0])

        report = evaluate_samples(
            samples=samples,
            data_root=Path("/tmp"),
            perception=FakeSafetyPerception(assessments),
            model_name="fake-vlm",
            clock=lambda: next(clock_values),
        )

        metrics = report["hazard_per_class"]

        self.assertEqual(
            set(metrics),
            {hazard.value for hazard in HazardType},
        )
        self.assertEqual(
            metrics["debris"],
            {
                "support": 2,
                "true_positives": 1,
                "false_positives": 0,
                "false_negatives": 1,
                "precision": 1.0,
                "recall": 0.5,
                "f1": 2 / 3,
            },
        )
        self.assertEqual(
            metrics["liquid_spill"],
            {
                "support": 0,
                "true_positives": 0,
                "false_positives": 1,
                "false_negatives": 0,
                "precision": 0.0,
                "recall": 0.0,
                "f1": 0.0,
            },
        )
        self.assertEqual(
            metrics["human_in_path"],
            {
                "support": 0,
                "true_positives": 0,
                "false_positives": 0,
                "false_negatives": 0,
                "precision": 0.0,
                "recall": 0.0,
                "f1": 0.0,
            },
        )

        # Strict serialization rejects NaN and Infinity.
        json.dumps(report, allow_nan=False)

    def test_per_hazard_counts_failures_and_conserves_micro_counts(
        self,
    ) -> None:
        samples = [
            SafetyEvaluationSample(
                sample_id="multi_failure",
                image="multi_failure.jpg",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset(
                    {
                        HazardType.DEBRIS,
                        HazardType.LIQUID_SPILL,
                    }
                ),
                expected_action=RecommendedAction.SLOW_DOWN,
            ),
            SafetyEvaluationSample(
                sample_id="debris_success",
                image="debris_success.jpg",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset({HazardType.DEBRIS}),
                expected_action=RecommendedAction.SLOW_DOWN,
            ),
        ]

        assessments = {
            "debris_success": SceneAssessment(
                waypoint="evaluation_0001",
                evidence_id="debris_success",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset(
                    {
                        HazardType.DEBRIS,
                        HazardType.FIRE_OR_SMOKE,
                    }
                ),
                recommended_action=RecommendedAction.STOP,
                confidence=0.90,
                view_quality=0.90,
            ),
        }

        clock_values = iter([0.0, 1.0, 1.0, 2.0])

        report = evaluate_samples(
            samples=samples,
            data_root=Path("/tmp"),
            perception=PartiallyFailingSafetyPerception(
                assessments=assessments,
                failing_ids=frozenset({"multi_failure"}),
            ),
            model_name="partially-failing-vlm",
            clock=lambda: next(clock_values),
        )

        metrics = report["hazard_per_class"]

        # The failed sample contributes one false negative per expected hazard.
        self.assertEqual(metrics["debris"]["false_negatives"], 1)
        self.assertEqual(metrics["debris"]["true_positives"], 1)
        self.assertEqual(metrics["liquid_spill"]["false_negatives"], 1)
        self.assertEqual(metrics["fire_or_smoke"]["false_positives"], 1)

        # Counts must be plain integers so JSON renders numbers, not booleans.
        for per_class in metrics.values():
            for field in (
                "support",
                "true_positives",
                "false_positives",
                "false_negatives",
            ):
                self.assertIs(type(per_class[field]), int)

        # Summing per-class counts must reproduce the aggregate micro counts.
        summed = {
            field: sum(per_class[field] for per_class in metrics.values())
            for field in ("true_positives", "false_positives", "false_negatives")
        }

        self.assertEqual(summed["true_positives"], 1)
        self.assertEqual(summed["false_positives"], 1)
        self.assertEqual(summed["false_negatives"], 2)

        micro_precision = summed["true_positives"] / (
            summed["true_positives"] + summed["false_positives"]
        )
        micro_recall = summed["true_positives"] / (
            summed["true_positives"] + summed["false_negatives"]
        )

        self.assertEqual(report["hazard_micro_precision"], micro_precision)
        self.assertEqual(report["hazard_micro_recall"], micro_recall)

    def test_per_hazard_metrics_when_every_sample_fails(self) -> None:
        samples = [
            SafetyEvaluationSample(
                sample_id="debris_failure",
                image="debris_failure.jpg",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset({HazardType.DEBRIS}),
                expected_action=RecommendedAction.SLOW_DOWN,
            ),
        ]

        clock_values = iter([0.0, 1.0])

        report = evaluate_samples(
            samples=samples,
            data_root=Path("/tmp"),
            perception=PartiallyFailingSafetyPerception(
                assessments={},
                failing_ids=frozenset({"debris_failure"}),
            ),
            model_name="failing-vlm",
            clock=lambda: next(clock_values),
        )

        debris = report["hazard_per_class"]["debris"]

        self.assertEqual(debris["support"], 1)
        self.assertEqual(debris["false_negatives"], 1)
        self.assertEqual(debris["precision"], 0.0)
        self.assertEqual(debris["recall"], 0.0)
        self.assertEqual(debris["f1"], 0.0)

        self.assertEqual(report["hazard_micro_f1"], 0.0)
        self.assertEqual(
            report["traversability_confusion_matrix"]["restricted"]["error"],
            1,
        )

        json.dumps(report, allow_nan=False)

    def test_reports_traversability_confusion_matrix_including_errors(
        self,
    ) -> None:
        samples = [
            SafetyEvaluationSample(
                sample_id="clear_as_restricted",
                image="clear.jpg",
                traversability=Traversability.CLEAR,
                hazards=frozenset(),
                expected_action=RecommendedAction.PROCEED,
            ),
            SafetyEvaluationSample(
                sample_id="blocked_correct",
                image="blocked.jpg",
                traversability=Traversability.BLOCKED,
                hazards=frozenset({HazardType.DEBRIS}),
                expected_action=RecommendedAction.REROUTE,
            ),
            SafetyEvaluationSample(
                sample_id="unknown_failure",
                image="unknown.jpg",
                traversability=Traversability.UNKNOWN,
                hazards=frozenset(),
                expected_action=RecommendedAction.INSPECT_CLOSER,
            ),
        ]

        assessments = {
            "clear_as_restricted": SceneAssessment(
                waypoint="evaluation_0000",
                evidence_id="clear_as_restricted",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset(),
                recommended_action=RecommendedAction.SLOW_DOWN,
                confidence=0.90,
                view_quality=0.90,
            ),
            "blocked_correct": SceneAssessment(
                waypoint="evaluation_0001",
                evidence_id="blocked_correct",
                traversability=Traversability.BLOCKED,
                hazards=frozenset({HazardType.DEBRIS}),
                recommended_action=RecommendedAction.REROUTE,
                confidence=0.90,
                view_quality=0.90,
            ),
        }

        clock_values = iter([0.0, 1.0, 1.0, 2.0, 2.0, 3.0])

        report = evaluate_samples(
            samples=samples,
            data_root=Path("/tmp"),
            perception=PartiallyFailingSafetyPerception(
                assessments=assessments,
                failing_ids=frozenset({"unknown_failure"}),
            ),
            model_name="partially-failing-vlm",
            clock=lambda: next(clock_values),
        )

        matrix = report["traversability_confusion_matrix"]
        expected_labels = {value.value for value in Traversability}
        predicted_labels = {*expected_labels, "error"}

        self.assertEqual(set(matrix), expected_labels)

        for row in matrix.values():
            self.assertEqual(set(row), predicted_labels)

        self.assertEqual(matrix["clear"]["restricted"], 1)
        self.assertEqual(matrix["blocked"]["blocked"], 1)
        self.assertEqual(matrix["unknown"]["error"], 1)
        self.assertEqual(
            sum(sum(row.values()) for row in matrix.values()),
            len(samples),
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

    def test_manifest_rejects_duplicate_sample_ids(
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
                    "id": "clear_1",
                    "image": "clear_a.jpg",
                    "traversability": "clear",
                    "hazards": [],
                    "expected_action": "proceed"
                  },
                  {
                    "id": "clear_1",
                    "image": "clear_b.jpg",
                    "traversability": "clear",
                    "hazards": [],
                    "expected_action": "proceed"
                  }
                ]
                """,
                encoding="utf-8",
            )

            with self.assertRaisesRegex(
                ValueError,
                "Duplicate sample id",
            ):
                load_manifest(manifest)


if __name__ == "__main__":
    unittest.main()

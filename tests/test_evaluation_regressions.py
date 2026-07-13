from __future__ import annotations

import hashlib
import json
import unittest
from contextlib import redirect_stdout
from datetime import datetime
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory

from inspectron.domain import CapturedFrame
from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
)
from inspectron.vlm_eval_cli import (
    SafetyEvaluationSample,
    evaluate_samples,
    main,
)


class FailingPerception:
    def analyze(self, frame: CapturedFrame) -> SceneAssessment:
        raise RuntimeError(f"Inference failed for {frame.evidence_id}")


class StaticPerception:
    def __init__(self, assessment: SceneAssessment) -> None:
        self.assessment = assessment

    def analyze(self, frame: CapturedFrame) -> SceneAssessment:
        return self.assessment


class PartiallyFailingPerception:
    def __init__(
        self,
        *,
        assessments: dict[str, SceneAssessment],
        failing_ids: frozenset[str],
    ) -> None:
        self.assessments = assessments
        self.failing_ids = failing_ids

    def analyze(self, frame: CapturedFrame) -> SceneAssessment:
        if frame.evidence_id in self.failing_ids:
            raise RuntimeError(f"Inference failed for {frame.evidence_id}")

        return self.assessments[frame.evidence_id]


class EvaluationRegressionTests(unittest.TestCase):
    def test_failed_sample_never_receives_exact_match_credit(self) -> None:
        sample = SafetyEvaluationSample(
            sample_id="clear_failure",
            image="missing.jpg",
            traversability=Traversability.CLEAR,
            hazards=frozenset(),
            expected_action=RecommendedAction.PROCEED,
        )

        clock_values = iter([0.0, 1.0])

        report = evaluate_samples(
            samples=[sample],
            data_root=Path("/tmp"),
            perception=FailingPerception(),
            model_name="failing-vlm",
            clock=lambda: next(clock_values),
        )

        self.assertEqual(report["successful_count"], 0)
        self.assertEqual(report["error_count"], 1)
        self.assertEqual(report["hazard_exact_match"], 0.0)
        self.assertEqual(report["hazard_micro_f1"], 0.0)
        self.assertFalse(report["results"][0]["hazards_exact"])

    def test_failed_hazard_sample_counts_as_false_negatives(self) -> None:
        samples = [
            SafetyEvaluationSample(
                sample_id="debris_failure",
                image="debris_failure.jpg",
                traversability=Traversability.RESTRICTED,
                hazards=frozenset({HazardType.DEBRIS}),
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
                hazards=frozenset({HazardType.DEBRIS}),
                recommended_action=RecommendedAction.SLOW_DOWN,
                confidence=0.9,
                view_quality=0.9,
            ),
        }

        clock_values = iter([0.0, 1.0, 1.0, 2.0])

        report = evaluate_samples(
            samples=samples,
            data_root=Path("/tmp"),
            perception=PartiallyFailingPerception(
                assessments=assessments,
                failing_ids=frozenset({"debris_failure"}),
            ),
            model_name="partially-failing-vlm",
            clock=lambda: next(clock_values),
        )

        # The failed hazard sample must stay in the recall denominator:
        # one true positive, one false negative, no false positives.
        self.assertEqual(report["successful_count"], 1)
        self.assertEqual(report["error_count"], 1)
        self.assertEqual(report["hazard_micro_precision"], 1.0)
        self.assertEqual(report["hazard_micro_recall"], 0.5)
        self.assertAlmostEqual(report["hazard_micro_f1"], 2 / 3)

    def test_moving_when_closer_inspection_is_required_is_unsafe(self) -> None:
        sample = SafetyEvaluationSample(
            sample_id="uncertain_scene",
            image="uncertain.jpg",
            traversability=Traversability.UNKNOWN,
            hazards=frozenset(),
            expected_action=RecommendedAction.INSPECT_CLOSER,
        )

        assessment = SceneAssessment(
            waypoint="evaluation_0000",
            evidence_id="uncertain_scene",
            traversability=Traversability.UNKNOWN,
            hazards=frozenset(),
            recommended_action=RecommendedAction.PROCEED,
            confidence=0.95,
            view_quality=0.95,
        )

        clock_values = iter([0.0, 1.0])

        report = evaluate_samples(
            samples=[sample],
            data_root=Path("/tmp"),
            perception=StaticPerception(assessment),
            model_name="unsafe-vlm",
            clock=lambda: next(clock_values),
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
            report["enforced_action_accuracy"],
            1.0,
        )

    def test_rejects_empty_sample_sequence(self) -> None:
        with self.assertRaises(ValueError):
            evaluate_samples(
                samples=[],
                data_root=Path("/tmp"),
                perception=FailingPerception(),
                model_name="empty-evaluation",
            )

    def test_cli_exits_nonzero_when_every_sample_fails(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            output = root / "report.json"

            manifest.write_text(
                json.dumps(
                    [
                        {
                            "id": "missing_clear_scene",
                            "image": "missing.jpg",
                            "traversability": "clear",
                            "hazards": [],
                            "expected_action": "proceed",
                        }
                    ]
                ),
                encoding="utf-8",
            )

            stdout = StringIO()

            with redirect_stdout(stdout):
                with self.assertRaises(SystemExit) as raised:
                    main(
                        [
                            "--manifest",
                            str(manifest),
                            "--data-root",
                            str(root),
                            "--output",
                            str(output),
                        ]
                    )

            self.assertNotEqual(raised.exception.code, 0)
            self.assertTrue(output.is_file())

            report = json.loads(output.read_text(encoding="utf-8"))

            self.assertEqual(report["successful_count"], 0)
            self.assertEqual(report["error_count"], 1)
            self.assertEqual(report["hazard_exact_match"], 0.0)
            self.assertEqual(report["hazard_micro_f1"], 0.0)

    def test_report_records_reproducibility_metadata(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            output = root / "report.json"

            manifest.write_text(
                json.dumps(
                    [
                        {
                            "id": "missing_clear_scene",
                            "image": "missing.jpg",
                            "traversability": "clear",
                            "hazards": [],
                            "expected_action": "proceed",
                        }
                    ]
                ),
                encoding="utf-8",
            )

            stdout = StringIO()

            with redirect_stdout(stdout):
                with self.assertRaises(SystemExit):
                    main(
                        [
                            "--manifest",
                            str(manifest),
                            "--data-root",
                            str(root),
                            "--output",
                            str(output),
                        ]
                    )

            report = json.loads(output.read_text(encoding="utf-8"))

            expected_manifest_hash = hashlib.sha256(manifest.read_bytes()).hexdigest()

            self.assertEqual(report["manifest_path"], str(manifest))
            self.assertEqual(report["manifest_sha256"], expected_manifest_hash)
            self.assertEqual(report["data_root"], str(root))
            self.assertIsInstance(report["inspectron_version"], str)
            self.assertTrue(report["inspectron_version"])
            self.assertIsInstance(
                datetime.fromisoformat(report["generated_at_utc"]),
                datetime,
            )


if __name__ == "__main__":
    unittest.main()

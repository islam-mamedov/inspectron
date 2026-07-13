from __future__ import annotations

import json
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory

from inspectron.domain import CapturedFrame
from inspectron.site_safety import (
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


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import json
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory

from inspectron.domain import CapturedFrame
from inspectron.repeated_evaluation import build_repeated_report
from inspectron.site_safety import (
    RecommendedAction,
    SceneAssessment,
    Traversability,
)
from inspectron.vlm_eval_cli import (
    SafetyEvaluationSample,
    build_parser,
    evaluate_repeated_samples,
    main,
)

DEFAULT_METRICS: dict[str, float | int] = {
    "traversability_accuracy": 1.0,
    "hazard_exact_match": 1.0,
    "hazard_micro_precision": 1.0,
    "hazard_micro_recall": 1.0,
    "hazard_micro_f1": 1.0,
    "model_action_accuracy": 1.0,
    "enforced_action_accuracy": 1.0,
    "policy_override_rate": 0.0,
    "model_unsafe_motion_count": 0,
    "enforced_unsafe_motion_count": 0,
    "mean_latency_seconds": 10.0,
}


class CountingPerception:
    def __init__(self) -> None:
        self.call_count = 0

    def analyze(self, frame: CapturedFrame) -> SceneAssessment:
        self.call_count += 1

        return SceneAssessment(
            waypoint=frame.waypoint,
            evidence_id=frame.evidence_id,
            traversability=Traversability.CLEAR,
            hazards=frozenset(),
            recommended_action=RecommendedAction.PROCEED,
            confidence=0.95,
            view_quality=0.90,
        )


def _evaluation_sample() -> SafetyEvaluationSample:
    return SafetyEvaluationSample(
        sample_id="clear_001",
        image="clear/clear_001.jpg",
        traversability=Traversability.CLEAR,
        hazards=frozenset(),
        expected_action=RecommendedAction.PROCEED,
    )


def _result(
    *,
    sample_id: str,
    image: str,
    traversability: str | None,
    hazards: list[str],
    model_action: str | None,
    enforced_action: str | None,
    error: str | None = None,
) -> dict[str, object]:
    return {
        "id": sample_id,
        "image": image,
        "predicted_traversability": traversability,
        "predicted_hazards": hazards,
        "model_action": model_action,
        "enforced_action": enforced_action,
        "error": error,
    }


def _run(
    results: list[dict[str, object]],
    **metric_overrides: float | int,
) -> dict[str, object]:
    metrics = {
        **DEFAULT_METRICS,
        **metric_overrides,
    }

    successful_count = sum(1 for result in results if result["error"] is None)
    error_count = len(results) - successful_count

    return {
        "model": "qwen3-vl:8b",
        "sample_count": len(results),
        "successful_count": successful_count,
        "error_count": error_count,
        **metrics,
        "results": results,
    }


def _clear_result() -> dict[str, object]:
    return _result(
        sample_id="clear_001",
        image="clear/clear_001.jpg",
        traversability="clear",
        hazards=[],
        model_action="proceed",
        enforced_action="proceed",
    )


def _debris_result(
    *,
    traversability: str = "restricted",
    model_action: str = "slow_down",
    enforced_action: str = "slow_down",
    hazards: list[str] | None = None,
) -> dict[str, object]:
    return _result(
        sample_id="debris_001",
        image="debris/debris_001.jpg",
        traversability=traversability,
        hazards=hazards or ["debris", "liquid_spill"],
        model_action=model_action,
        enforced_action=enforced_action,
    )


class RepeatedEvaluationTests(unittest.TestCase):
    def test_one_run_preserves_original_report_schema(self) -> None:
        perception = CountingPerception()
        clock_values = iter([0.0, 1.0])

        report = evaluate_repeated_samples(
            samples=[_evaluation_sample()],
            data_root=Path("/tmp"),
            perception=perception,
            model_name="fake-vlm",
            run_count=1,
            clock=lambda: next(clock_values),
        )

        self.assertEqual(perception.call_count, 1)
        self.assertEqual(report["sample_count"], 1)
        self.assertEqual(report["successful_count"], 1)
        self.assertEqual(report["mean_latency_seconds"], 1.0)
        self.assertIn("hazard_per_class", report)
        self.assertIn("traversability_confusion_matrix", report)
        self.assertNotIn("run_count", report)
        self.assertNotIn("metric_summary", report)
        self.assertNotIn("runs", report)

    def test_three_runs_execute_every_sample_and_aggregate(self) -> None:
        perception = CountingPerception()
        clock_values = iter(
            [
                0.0,
                1.0,
                1.0,
                2.0,
                2.0,
                3.0,
            ]
        )

        report = evaluate_repeated_samples(
            samples=[_evaluation_sample()],
            data_root=Path("/tmp"),
            perception=perception,
            model_name="fake-vlm",
            run_count=3,
            clock=lambda: next(clock_values),
        )

        self.assertEqual(perception.call_count, 3)
        self.assertEqual(report["run_count"], 3)
        self.assertEqual(report["sample_count"], 1)
        self.assertEqual(report["total_evaluations"], 3)
        self.assertEqual(report["successful_count"], 3)
        self.assertEqual(report["error_count"], 0)
        self.assertEqual(report["mean_prediction_agreement"], 1.0)
        self.assertEqual(report["fully_stable_scene_count"], 1)
        self.assertEqual(len(report["runs"]), 3)
        self.assertEqual(
            [run["run_index"] for run in report["runs"]],
            [1, 2, 3],
        )
        self.assertEqual(
            report["metric_summary"]["mean_latency_seconds"],
            {
                "mean": 1.0,
                "population_std": 0.0,
                "min": 1.0,
                "max": 1.0,
            },
        )

        json.dumps(report, allow_nan=False)

    def test_repeated_evaluator_rejects_invalid_run_count(self) -> None:
        for invalid_run_count in (0, -1, True):
            with self.subTest(run_count=invalid_run_count):
                with self.assertRaises(ValueError):
                    evaluate_repeated_samples(
                        samples=[_evaluation_sample()],
                        data_root=Path("/tmp"),
                        perception=CountingPerception(),
                        model_name="fake-vlm",
                        run_count=invalid_run_count,
                    )

    def test_cli_rejects_non_positive_runs(self) -> None:
        parser = build_parser()

        with redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit):
                parser.parse_args(
                    [
                        "--manifest",
                        "manifest.json",
                        "--data-root",
                        "benchmarks/data",
                        "--runs",
                        "0",
                    ]
                )

    def test_aggregates_metrics_and_scene_agreement(self) -> None:
        run_one = _run(
            [_clear_result(), _debris_result()],
            traversability_accuracy=1.0,
            mean_latency_seconds=10.0,
        )
        run_two = _run(
            [
                _clear_result(),
                _debris_result(
                    traversability="blocked",
                    model_action="stop",
                    enforced_action="stop",
                ),
            ],
            traversability_accuracy=0.5,
            mean_latency_seconds=14.0,
        )
        run_three = _run(
            [
                _clear_result(),
                _debris_result(
                    traversability="blocked",
                    model_action="stop",
                    enforced_action="stop",
                ),
            ],
            traversability_accuracy=0.5,
            mean_latency_seconds=12.0,
        )

        report = build_repeated_report(
            model_name="qwen3-vl:8b",
            run_reports=[run_one, run_two, run_three],
        )

        self.assertEqual(report["model"], "qwen3-vl:8b")
        self.assertEqual(report["run_count"], 3)
        self.assertEqual(report["sample_count"], 2)
        self.assertEqual(report["total_evaluations"], 6)
        self.assertEqual(report["successful_count"], 6)
        self.assertEqual(report["error_count"], 0)
        self.assertEqual(report["fully_stable_scene_count"], 1)
        self.assertAlmostEqual(
            report["mean_prediction_agreement"],
            5 / 6,
        )

        metric_summary = report["metric_summary"]
        traversability = metric_summary["traversability_accuracy"]

        self.assertAlmostEqual(
            traversability["mean"],
            2 / 3,
        )
        self.assertAlmostEqual(
            traversability["population_std"],
            0.23570226039551584,
        )
        self.assertEqual(traversability["min"], 0.5)
        self.assertEqual(traversability["max"], 1.0)

        latency = metric_summary["mean_latency_seconds"]

        self.assertEqual(latency["mean"], 12.0)
        self.assertAlmostEqual(
            latency["population_std"],
            1.632993161855452,
        )
        self.assertEqual(latency["min"], 10.0)
        self.assertEqual(latency["max"], 14.0)

        scenes = report["scene_prediction_agreement"]

        self.assertEqual(scenes[0]["id"], "clear_001")
        self.assertEqual(scenes[0]["agreement_rate"], 1.0)
        self.assertTrue(scenes[0]["all_runs_agree"])
        self.assertEqual(scenes[0]["unique_prediction_count"], 1)

        self.assertEqual(scenes[1]["id"], "debris_001")
        self.assertAlmostEqual(scenes[1]["agreement_rate"], 2 / 3)
        self.assertFalse(scenes[1]["all_runs_agree"])
        self.assertEqual(scenes[1]["unique_prediction_count"], 2)

    def test_hazard_order_does_not_create_false_disagreement(
        self,
    ) -> None:
        run_one = _run(
            [
                _debris_result(
                    hazards=["debris", "liquid_spill"],
                )
            ]
        )
        run_two = _run(
            [
                _debris_result(
                    hazards=["liquid_spill", "debris"],
                )
            ]
        )

        report = build_repeated_report(
            model_name="qwen3-vl:8b",
            run_reports=[run_one, run_two],
        )

        scene = report["scene_prediction_agreement"][0]

        self.assertEqual(scene["agreement_rate"], 1.0)
        self.assertTrue(scene["all_runs_agree"])
        self.assertEqual(scene["unique_prediction_count"], 1)

        variant = scene["prediction_variants"][0]

        self.assertEqual(
            variant["predicted_hazards"],
            ["debris", "liquid_spill"],
        )
        self.assertEqual(variant["count"], 2)
        self.assertEqual(variant["rate"], 1.0)

    def test_failure_is_an_agreement_state_without_error_text(
        self,
    ) -> None:
        first_failure = _result(
            sample_id="fire_001",
            image="fire_or_smoke/fire_001.jpg",
            traversability=None,
            hazards=[],
            model_action=None,
            enforced_action=None,
            error="request timed out after 30 seconds",
        )
        second_failure = _result(
            sample_id="fire_001",
            image="fire_or_smoke/fire_001.jpg",
            traversability=None,
            hazards=[],
            model_action=None,
            enforced_action=None,
            error="connection reset by peer",
        )

        report = build_repeated_report(
            model_name="qwen3-vl:8b",
            run_reports=[
                _run([first_failure]),
                _run([second_failure]),
            ],
        )

        self.assertEqual(report["successful_count"], 0)
        self.assertEqual(report["error_count"], 2)

        scene = report["scene_prediction_agreement"][0]

        self.assertEqual(scene["successful_count"], 0)
        self.assertEqual(scene["agreement_rate"], 1.0)
        self.assertTrue(scene["all_runs_agree"])

        variant = scene["prediction_variants"][0]

        self.assertTrue(variant["failed"])
        self.assertNotIn("error", variant)

        serialized_variants = json.dumps(
            scene["prediction_variants"],
            sort_keys=True,
        )

        self.assertNotIn("timed out", serialized_variants)
        self.assertNotIn("connection reset", serialized_variants)

    def test_single_run_has_zero_population_standard_deviation(
        self,
    ) -> None:
        report = build_repeated_report(
            model_name="qwen3-vl:8b",
            run_reports=[_run([_clear_result()])],
        )

        self.assertEqual(report["run_count"], 1)
        self.assertEqual(report["mean_prediction_agreement"], 1.0)
        self.assertEqual(report["fully_stable_scene_count"], 1)

        for summary in report["metric_summary"].values():
            self.assertEqual(summary["population_std"], 0.0)
            self.assertEqual(summary["mean"], summary["min"])
            self.assertEqual(summary["mean"], summary["max"])

        json.dumps(report, allow_nan=False)

    def test_cli_writes_repeated_report_and_exits_nonzero_when_all_runs_fail(
        self,
    ) -> None:
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
                            "--runs",
                            "2",
                            "--output",
                            str(output),
                        ]
                    )

            self.assertNotEqual(raised.exception.code, 0)
            self.assertTrue(output.is_file())

            report = json.loads(
                output.read_text(encoding="utf-8"),
                parse_constant=self.fail,
            )

            self.assertEqual(report["run_count"], 2)
            self.assertEqual(report["sample_count"], 1)
            self.assertEqual(report["total_evaluations"], 2)
            self.assertEqual(report["successful_count"], 0)
            self.assertEqual(report["error_count"], 2)
            self.assertEqual(report["mean_prediction_agreement"], 1.0)
            self.assertEqual(report["fully_stable_scene_count"], 1)
            self.assertIn("metric_summary", report)
            self.assertIn("manifest_sha256", report)
            self.assertIn("generated_at_utc", report)
            self.assertEqual(
                [run["run_index"] for run in report["runs"]],
                [1, 2],
            )

            scene = report["scene_prediction_agreement"][0]

            self.assertEqual(scene["successful_count"], 0)
            self.assertTrue(scene["prediction_variants"][0]["failed"])

            summary = json.loads(
                stdout.getvalue(),
                parse_constant=self.fail,
            )

            self.assertEqual(summary["run_count"], 2)
            self.assertEqual(summary["successful_count"], 0)
            self.assertIn("metric_summary", summary)

    def test_rejects_empty_run_collection(self) -> None:
        with self.assertRaisesRegex(
            ValueError,
            "At least one run report",
        ):
            build_repeated_report(
                model_name="qwen3-vl:8b",
                run_reports=[],
            )

    def test_rejects_model_mismatch(self) -> None:
        mismatched_run = _run([_clear_result()])
        mismatched_run["model"] = "different-model"

        with self.assertRaisesRegex(
            ValueError,
            "model does not match",
        ):
            build_repeated_report(
                model_name="qwen3-vl:8b",
                run_reports=[mismatched_run],
            )

    def test_rejects_different_sample_order(self) -> None:
        run_one = _run([_clear_result(), _debris_result()])
        run_two = _run([_debris_result(), _clear_result()])

        with self.assertRaisesRegex(
            ValueError,
            "same samples in the same order",
        ):
            build_repeated_report(
                model_name="qwen3-vl:8b",
                run_reports=[run_one, run_two],
            )

    def test_rejects_inconsistent_sample_count(self) -> None:
        invalid_run = _run([_clear_result()])
        invalid_run["sample_count"] = 2

        with self.assertRaisesRegex(
            ValueError,
            "invalid sample count",
        ):
            build_repeated_report(
                model_name="qwen3-vl:8b",
                run_reports=[invalid_run],
            )

    def test_rejects_non_numeric_metric(self) -> None:
        invalid_run = _run([_clear_result()])
        invalid_run["hazard_micro_f1"] = "perfect"

        with self.assertRaisesRegex(
            ValueError,
            "invalid metric hazard_micro_f1",
        ):
            build_repeated_report(
                model_name="qwen3-vl:8b",
                run_reports=[invalid_run],
            )

    def test_rejects_non_finite_metric(self) -> None:
        for non_finite_value in (
            float("nan"),
            float("inf"),
            float("-inf"),
        ):
            with self.subTest(value=non_finite_value):
                invalid_run = _run([_clear_result()])
                invalid_run["mean_latency_seconds"] = non_finite_value

                with self.assertRaisesRegex(
                    ValueError,
                    "invalid metric mean_latency_seconds",
                ):
                    build_repeated_report(
                        model_name="qwen3-vl:8b",
                        run_reports=[invalid_run],
                    )

    def test_generated_run_index_cannot_be_overridden(self) -> None:
        run = _run([_clear_result()])
        run["run_index"] = 999

        report = build_repeated_report(
            model_name="qwen3-vl:8b",
            run_reports=[run],
        )

        self.assertEqual(report["runs"][0]["run_index"], 1)


if __name__ == "__main__":
    unittest.main()

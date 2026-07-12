from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from inspectron.domain import (
    DefectType,
    Observation,
)
from inspectron.vlm_eval_cli import (
    EvaluationSample,
    evaluate_samples,
    load_manifest,
)


class FakePerception:
    def __init__(
        self,
        predictions: dict[str, DefectType],
    ) -> None:
        self.predictions = predictions

    def analyze(self, frame: object) -> Observation:
        sample_id = frame.evidence_id
        prediction = self.predictions[sample_id]

        return Observation(
            waypoint=frame.waypoint,
            asset_id=frame.asset_id,
            evidence_id=frame.evidence_id,
            predicted_defect=prediction,
            confidence=0.90,
            view_quality=0.80,
        )


class VLMEvaluationTests(unittest.TestCase):
    def test_calculates_batch_metrics(self) -> None:
        samples = [
            EvaluationSample(
                sample_id="crack_1",
                image="crack.jpg",
                expected=DefectType.CRACK,
            ),
            EvaluationSample(
                sample_id="clean_1",
                image="clean.jpg",
                expected=DefectType.NONE,
            ),
        ]

        perception = FakePerception(
            {
                "crack_1": DefectType.CRACK,
                "clean_1": DefectType.CRACK,
            }
        )

        clock_values = iter(
            [
                0.0,
                1.0,
                1.0,
                3.0,
            ]
        )

        with TemporaryDirectory() as directory:
            data_root = Path(directory)

            (data_root / "crack.jpg").write_bytes(b"crack-image")
            (data_root / "clean.jpg").write_bytes(b"clean-image")

            report = evaluate_samples(
                samples=samples,
                data_root=data_root,
                perception=perception,
                model_name="fake-vlm",
                clock=lambda: next(clock_values),
            )

        self.assertEqual(report["sample_count"], 2)
        self.assertEqual(report["successful_count"], 2)
        self.assertEqual(report["correct_count"], 1)
        self.assertEqual(report["accuracy"], 0.5)
        self.assertEqual(
            report["mean_latency_seconds"],
            1.5,
        )

        per_class = report["per_class"]

        self.assertAlmostEqual(
            per_class["crack"]["precision"],
            0.5,
        )
        self.assertEqual(
            per_class["none"]["recall"],
            0.0,
        )
        self.assertAlmostEqual(
            report["macro_f1"],
            1 / 3,
        )

    def test_rejects_unknown_manifest_label(self) -> None:
        with TemporaryDirectory() as directory:
            manifest_path = Path(directory) / "manifest.json"

            manifest_path.write_text(
                json.dumps(
                    [
                        {
                            "id": "sample_1",
                            "image": "sample.jpg",
                            "expected": "water_damage",
                        }
                    ]
                ),
                encoding="utf-8",
            )

            with self.assertRaises(ValueError):
                load_manifest(manifest_path)


if __name__ == "__main__":
    unittest.main()

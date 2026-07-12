from __future__ import annotations

import argparse
import json
import os
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

from inspectron.clients.ollama import OllamaVLMClient
from inspectron.domain import CapturedFrame, DefectType
from inspectron.ports import PerceptionPort
from inspectron.vlm import VLMPerception


@dataclass(frozen=True, slots=True)
class EvaluationSample:
    sample_id: str
    image: str
    expected: DefectType


def load_manifest(path: Path) -> list[EvaluationSample]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"Manifest is not valid JSON: {path}") from error

    if not isinstance(payload, list):
        raise ValueError("Manifest must contain a JSON array")

    samples: list[EvaluationSample] = []

    for index, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"Manifest item {index} must be an object")

        try:
            sample_id = item["id"]
            image = item["image"]
            expected_value = item["expected"]
        except KeyError as error:
            raise ValueError(f"Manifest item {index} is missing field: {error.args[0]}") from error

        if not isinstance(sample_id, str) or not sample_id:
            raise ValueError(f"Manifest item {index} has an invalid id")

        if not isinstance(image, str) or not image:
            raise ValueError(f"Manifest item {index} has an invalid image")

        image_path = Path(image)

        if image_path.is_absolute() or ".." in image_path.parts:
            raise ValueError(f"Manifest item {index} must use a safe relative image path")

        try:
            expected = DefectType(expected_value)
        except ValueError as error:
            raise ValueError(
                f"Manifest item {index} has an unsupported label: {expected_value}"
            ) from error

        samples.append(
            EvaluationSample(
                sample_id=sample_id,
                image=image,
                expected=expected,
            )
        )

    if not samples:
        raise ValueError("Manifest cannot be empty")

    return samples


def evaluate_samples(
    *,
    samples: Sequence[EvaluationSample],
    data_root: Path,
    perception: PerceptionPort,
    model_name: str,
    clock: Callable[[], float] = perf_counter,
) -> dict[str, object]:
    results: list[dict[str, object]] = []
    latencies: list[float] = []

    for index, sample in enumerate(samples):
        image_path = data_root / sample.image

        frame = CapturedFrame(
            waypoint=f"evaluation_{index:04d}",
            asset_id=sample.sample_id,
            evidence_id=sample.sample_id,
            image_path=str(image_path),
        )

        started_at = clock()

        try:
            observation = perception.analyze(frame)
        except Exception as error:
            latency = clock() - started_at
            latencies.append(latency)

            results.append(
                {
                    "id": sample.sample_id,
                    "image": sample.image,
                    "expected": sample.expected.value,
                    "predicted": None,
                    "correct": False,
                    "confidence": None,
                    "view_quality": None,
                    "latency_seconds": latency,
                    "error": (f"{type(error).__name__}: {error}"),
                }
            )

            continue

        latency = clock() - started_at
        latencies.append(latency)

        predicted = observation.predicted_defect
        correct = predicted is sample.expected

        results.append(
            {
                "id": sample.sample_id,
                "image": sample.image,
                "expected": sample.expected.value,
                "predicted": predicted.value,
                "correct": correct,
                "confidence": observation.confidence,
                "view_quality": observation.view_quality,
                "latency_seconds": latency,
                "error": None,
            }
        )

    correct_count = sum(bool(result["correct"]) for result in results)

    successful_count = sum(result["error"] is None for result in results)

    accuracy = correct_count / len(results)
    per_class = _calculate_per_class_metrics(results)

    macro_f1 = sum(metrics["f1"] for metrics in per_class.values()) / len(per_class)

    mean_latency = sum(latencies) / len(latencies)

    return {
        "model": model_name,
        "sample_count": len(results),
        "successful_count": successful_count,
        "error_count": len(results) - successful_count,
        "correct_count": correct_count,
        "accuracy": accuracy,
        "macro_f1": macro_f1,
        "mean_latency_seconds": mean_latency,
        "per_class": per_class,
        "results": results,
    }


def _calculate_per_class_metrics(
    results: Sequence[dict[str, object]],
) -> dict[str, dict[str, float | int]]:
    labels = sorted({str(result["expected"]) for result in results})

    metrics: dict[str, dict[str, float | int]] = {}

    for label in labels:
        true_positives = sum(
            result["expected"] == label and result["predicted"] == label for result in results
        )

        false_positives = sum(
            result["expected"] != label and result["predicted"] == label for result in results
        )

        false_negatives = sum(
            result["expected"] == label and result["predicted"] != label for result in results
        )

        precision_denominator = true_positives + false_positives
        recall_denominator = true_positives + false_negatives

        precision = true_positives / precision_denominator if precision_denominator else 0.0

        recall = true_positives / recall_denominator if recall_denominator else 0.0

        f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0

        metrics[label] = {
            "support": recall_denominator,
            "true_positives": true_positives,
            "false_positives": false_positives,
            "false_negatives": false_negatives,
            "precision": precision,
            "recall": recall,
            "f1": f1,
        }

    return metrics


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate a VLM on labeled images.",
    )

    parser.add_argument(
        "--manifest",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--data-root",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--base-url",
        default=os.getenv(
            "INSPECTRON_VLM_BASE_URL",
            "http://localhost:11434",
        ),
    )
    parser.add_argument(
        "--model",
        default=os.getenv(
            "INSPECTRON_VLM_MODEL",
            "qwen3-vl:8b",
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/metrics/vlm_smoke.json"),
    )

    return parser


def main(
    argv: Sequence[str] | None = None,
) -> None:
    arguments = build_parser().parse_args(argv)

    samples = load_manifest(arguments.manifest)

    client = OllamaVLMClient(
        base_url=arguments.base_url,
        model=arguments.model,
    )

    perception = VLMPerception(client)

    report = evaluate_samples(
        samples=samples,
        data_root=arguments.data_root,
        perception=perception,
        model_name=arguments.model,
    )

    arguments.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    arguments.output.write_text(
        json.dumps(report, indent=2),
        encoding="utf-8",
    )

    summary = {
        "model": report["model"],
        "sample_count": report["sample_count"],
        "successful_count": report["successful_count"],
        "accuracy": report["accuracy"],
        "macro_f1": report["macro_f1"],
        "mean_latency_seconds": report["mean_latency_seconds"],
        "output": str(arguments.output),
    }

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

from __future__ import annotations

import argparse
import hashlib
import json
import os
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from time import perf_counter
from typing import Protocol

from inspectron.clients.ollama import OllamaVLMClient
from inspectron.domain import CapturedFrame
from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
    find_consistency_violations,
    resolve_safe_action,
)
from inspectron.vlm import (
    SITE_SAFETY_RESPONSE_SCHEMA,
    SiteSafetyVLMPerception,
)


@dataclass(frozen=True, slots=True)
class SafetyEvaluationSample:
    sample_id: str
    image: str
    traversability: Traversability
    hazards: frozenset[HazardType]
    expected_action: RecommendedAction


class SafetyPerception(Protocol):
    def analyze(
        self,
        frame: CapturedFrame,
    ) -> SceneAssessment: ...


def load_manifest(
    path: Path,
) -> list[SafetyEvaluationSample]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"Manifest is not valid JSON: {path}") from error

    if not isinstance(payload, list):
        raise ValueError("Manifest must contain a JSON array")

    samples: list[SafetyEvaluationSample] = []
    seen_ids: set[str] = set()
    seen_images: set[str] = set()

    for index, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"Manifest item {index} must be an object")

        required = {
            "id",
            "image",
            "traversability",
            "hazards",
            "expected_action",
        }

        missing = required - item.keys()

        if missing:
            raise ValueError(f"Manifest item {index} is missing: {sorted(missing)}")

        sample_id = item["id"]
        image = item["image"]
        raw_hazards = item["hazards"]

        if not isinstance(sample_id, str) or not sample_id:
            raise ValueError(f"Manifest item {index} has an invalid id")

        if not isinstance(image, str) or not image:
            raise ValueError(f"Manifest item {index} has an invalid image")

        image_path = Path(image)

        if image_path.is_absolute() or ".." in image_path.parts:
            raise ValueError(f"Manifest item {index} must use a safe relative image path")

        if sample_id in seen_ids:
            raise ValueError(f"Duplicate sample id: {sample_id}")

        if image in seen_images:
            raise ValueError(f"Duplicate image path: {image}")

        seen_ids.add(sample_id)
        seen_images.add(image)

        if not isinstance(raw_hazards, list):
            raise ValueError(f"Manifest item {index} hazards must be an array")

        if not all(isinstance(value, str) for value in raw_hazards):
            raise ValueError(f"Manifest item {index} hazards must contain strings")

        if len(raw_hazards) != len(set(raw_hazards)):
            raise ValueError(f"Manifest item {index} contains duplicate hazards")

        try:
            traversability = Traversability(item["traversability"])
            hazards = frozenset(HazardType(value) for value in raw_hazards)
            expected_action = RecommendedAction(item["expected_action"])
        except ValueError as error:
            raise ValueError(f"Manifest item {index} contains an unsupported label") from error

        if traversability is Traversability.CLEAR and hazards:
            raise ValueError(f"Manifest item {index} cannot be clear while containing hazards")

        expected_assessment = SceneAssessment(
            waypoint=sample_id,
            evidence_id=sample_id,
            traversability=traversability,
            hazards=hazards,
            recommended_action=expected_action,
            confidence=1.0,
            view_quality=1.0,
        )

        resolved_action = resolve_safe_action(expected_assessment)

        if expected_action is not resolved_action:
            raise ValueError(
                f"Manifest item {index} expected action must be {resolved_action.value}"
            )

        samples.append(
            SafetyEvaluationSample(
                sample_id=sample_id,
                image=image,
                traversability=traversability,
                hazards=hazards,
                expected_action=expected_action,
            )
        )

    if not samples:
        raise ValueError("Manifest cannot be empty")

    return samples


def evaluate_samples(
    *,
    samples: Sequence[SafetyEvaluationSample],
    data_root: Path,
    perception: SafetyPerception,
    model_name: str,
    clock: Callable[[], float] = perf_counter,
) -> dict[str, object]:
    if not samples:
        raise ValueError("At least one evaluation sample is required")

    results: list[dict[str, object]] = []
    latencies: list[float] = []

    for index, sample in enumerate(samples):
        frame = CapturedFrame(
            waypoint=f"evaluation_{index:04d}",
            scene_id=sample.sample_id,
            evidence_id=sample.sample_id,
            image_path=str(data_root / sample.image),
        )

        started_at = clock()

        try:
            assessment = perception.analyze(frame)
        except (OSError, RuntimeError, ValueError) as error:
            latency = clock() - started_at
            latencies.append(latency)

            results.append(
                _error_result(
                    sample=sample,
                    latency=latency,
                    error=error,
                )
            )
            continue

        latency = clock() - started_at
        latencies.append(latency)

        enforced_action = resolve_safe_action(assessment)

        violations = find_consistency_violations(assessment)

        results.append(
            {
                "id": sample.sample_id,
                "image": sample.image,
                "expected_traversability": (sample.traversability.value),
                "predicted_traversability": (assessment.traversability.value),
                "expected_hazards": sorted(hazard.value for hazard in sample.hazards),
                "predicted_hazards": sorted(hazard.value for hazard in assessment.hazards),
                "expected_action": (sample.expected_action.value),
                "model_action": (assessment.recommended_action.value),
                "enforced_action": (enforced_action.value),
                "traversability_correct": (assessment.traversability is sample.traversability),
                "hazards_exact": (assessment.hazards == sample.hazards),
                "model_action_correct": (assessment.recommended_action is sample.expected_action),
                "enforced_action_correct": (enforced_action is sample.expected_action),
                "policy_overrode_model": (enforced_action is not assessment.recommended_action),
                "consistency_violations": list(violations),
                "confidence": assessment.confidence,
                "view_quality": assessment.view_quality,
                "latency_seconds": latency,
                "error": None,
            }
        )

    return _build_report(
        model_name=model_name,
        results=results,
        latencies=latencies,
    )


def _error_result(
    *,
    sample: SafetyEvaluationSample,
    latency: float,
    error: Exception,
) -> dict[str, object]:
    return {
        "id": sample.sample_id,
        "image": sample.image,
        "expected_traversability": (sample.traversability.value),
        "predicted_traversability": None,
        "expected_hazards": sorted(hazard.value for hazard in sample.hazards),
        "predicted_hazards": [],
        "expected_action": (sample.expected_action.value),
        "model_action": None,
        "enforced_action": None,
        "traversability_correct": False,
        "hazards_exact": False,
        "model_action_correct": False,
        "enforced_action_correct": False,
        "policy_overrode_model": False,
        "consistency_violations": [],
        "confidence": None,
        "view_quality": None,
        "latency_seconds": latency,
        "error": f"{type(error).__name__}: {error}",
    }


def _build_report(
    *,
    model_name: str,
    results: Sequence[dict[str, object]],
    latencies: Sequence[float],
) -> dict[str, object]:
    sample_count = len(results)

    successful_count = sum(result["error"] is None for result in results)

    traversability_accuracy = (
        sum(bool(result["traversability_correct"]) for result in results) / sample_count
    )

    hazard_exact_match = sum(bool(result["hazards_exact"]) for result in results) / sample_count

    model_action_accuracy = (
        sum(bool(result["model_action_correct"]) for result in results) / sample_count
    )

    enforced_action_accuracy = (
        sum(bool(result["enforced_action_correct"]) for result in results) / sample_count
    )

    policy_override_count = sum(bool(result["policy_overrode_model"]) for result in results)

    hazard_metrics = _hazard_metrics(results)

    model_unsafe_motion_count = _unsafe_motion_count(
        results,
        action_field="model_action",
    )

    enforced_unsafe_motion_count = _unsafe_motion_count(
        results,
        action_field="enforced_action",
    )

    return {
        "model": model_name,
        "sample_count": sample_count,
        "successful_count": successful_count,
        "error_count": (sample_count - successful_count),
        "traversability_accuracy": (traversability_accuracy),
        "hazard_exact_match": hazard_exact_match,
        "hazard_micro_precision": (hazard_metrics["precision"]),
        "hazard_micro_recall": (hazard_metrics["recall"]),
        "hazard_micro_f1": hazard_metrics["f1"],
        "model_action_accuracy": (model_action_accuracy),
        "enforced_action_accuracy": (enforced_action_accuracy),
        "policy_override_count": (policy_override_count),
        "policy_override_rate": (policy_override_count / sample_count),
        "model_unsafe_motion_count": (model_unsafe_motion_count),
        "enforced_unsafe_motion_count": (enforced_unsafe_motion_count),
        "mean_latency_seconds": (sum(latencies) / len(latencies)),
        "results": list(results),
    }


def _hazard_metrics(
    results: Sequence[dict[str, object]],
) -> dict[str, float | int]:
    if not any(result["error"] is None for result in results):
        return {
            "true_positives": 0,
            "false_positives": 0,
            "false_negatives": 0,
            "precision": 0.0,
            "recall": 0.0,
            "f1": 0.0,
        }

    true_positives = 0
    false_positives = 0
    false_negatives = 0

    # Failed samples predict no hazards, so their expected hazards
    # count as false negatives instead of leaving the denominator.
    for result in results:
        expected = set(result["expected_hazards"])
        predicted = set(result["predicted_hazards"])

        true_positives += len(expected & predicted)
        false_positives += len(predicted - expected)
        false_negatives += len(expected - predicted)

    precision_denominator = true_positives + false_positives
    recall_denominator = true_positives + false_negatives

    precision = (
        true_positives / precision_denominator
        if precision_denominator
        else float(false_negatives == 0)
    )

    recall = true_positives / recall_denominator if recall_denominator else 1.0

    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0

    return {
        "true_positives": true_positives,
        "false_positives": false_positives,
        "false_negatives": false_negatives,
        "precision": precision,
        "recall": recall,
        "f1": f1,
    }


def _unsafe_motion_count(
    results: Sequence[dict[str, object]],
    *,
    action_field: str,
) -> int:
    must_not_move = {
        RecommendedAction.STOP.value,
        RecommendedAction.REROUTE.value,
        RecommendedAction.INSPECT_CLOSER.value,
    }

    permits_motion = {
        RecommendedAction.PROCEED.value,
        RecommendedAction.SLOW_DOWN.value,
    }

    return sum(
        result["expected_action"] in must_not_move and result[action_field] in permits_motion
        for result in results
    )


def _package_version() -> str:
    try:
        return version("inspectron")
    except PackageNotFoundError:
        return "unknown"


def _run_metadata(
    *,
    manifest_path: Path,
    data_root: Path,
) -> dict[str, str]:
    manifest_sha256 = hashlib.sha256(manifest_path.read_bytes()).hexdigest()

    return {
        "generated_at_utc": datetime.now(UTC).isoformat(timespec="seconds"),
        "inspectron_version": _package_version(),
        "manifest_path": str(manifest_path),
        "manifest_sha256": manifest_sha256,
        "data_root": str(data_root),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=("Evaluate embodied VLM site-safety assessments."),
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
        default=Path("artifacts/metrics/site_safety_smoke.json"),
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
        response_schema=SITE_SAFETY_RESPONSE_SCHEMA,
    )

    perception = SiteSafetyVLMPerception(client)

    report = evaluate_samples(
        samples=samples,
        data_root=arguments.data_root,
        perception=perception,
        model_name=arguments.model,
    )

    report = {
        **_run_metadata(
            manifest_path=arguments.manifest,
            data_root=arguments.data_root,
        ),
        **report,
    }

    arguments.output.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    arguments.output.write_text(
        json.dumps(report, indent=2),
        encoding="utf-8",
    )

    summary_keys = (
        "model",
        "sample_count",
        "successful_count",
        "traversability_accuracy",
        "hazard_exact_match",
        "hazard_micro_f1",
        "model_action_accuracy",
        "enforced_action_accuracy",
        "policy_override_rate",
        "model_unsafe_motion_count",
        "enforced_unsafe_motion_count",
        "mean_latency_seconds",
    )

    summary = {key: report[key] for key in summary_keys}

    summary["output"] = str(arguments.output)

    print(json.dumps(summary, indent=2))

    if report["successful_count"] == 0:
        raise SystemExit("Evaluation failed: no samples completed successfully")


if __name__ == "__main__":
    main()

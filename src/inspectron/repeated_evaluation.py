from __future__ import annotations

import json
from collections import Counter
from collections.abc import Sequence
from math import isfinite
from statistics import fmean, pstdev
from typing import TypeAlias

REPEATED_METRIC_FIELDS = (
    "traversability_accuracy",
    "hazard_exact_match",
    "hazard_micro_precision",
    "hazard_micro_recall",
    "hazard_micro_f1",
    "model_action_accuracy",
    "enforced_action_accuracy",
    "policy_override_rate",
    "model_unsafe_motion_count",
    "enforced_unsafe_motion_count",
    "mean_latency_seconds",
)

PredictionSignature: TypeAlias = tuple[
    str | None,
    tuple[str, ...],
    str | None,
    str | None,
    bool,
]


def build_repeated_report(
    *,
    model_name: str,
    run_reports: Sequence[dict[str, object]],
) -> dict[str, object]:
    """Aggregate multiple evaluations over the same ordered sample set."""

    if not run_reports:
        raise ValueError("At least one run report is required")

    normalized_runs = [
        _validated_run(
            report,
            index=index,
            model_name=model_name,
        )
        for index, report in enumerate(run_reports, start=1)
    ]

    reference_identity = _sample_identity(normalized_runs[0]["results"])

    for run in normalized_runs[1:]:
        if _sample_identity(run["results"]) != reference_identity:
            raise ValueError(
                "Repeated evaluation runs must contain the same samples in the same order"
            )

    sample_count = len(reference_identity)
    run_count = len(normalized_runs)

    metric_summary = {
        field: _summarize_metric(
            reports=normalized_runs,
            field=field,
        )
        for field in REPEATED_METRIC_FIELDS
    }

    scene_prediction_agreement = _scene_prediction_agreement(normalized_runs)

    agreement_values = [float(scene["agreement_rate"]) for scene in scene_prediction_agreement]

    successful_count = sum(int(run["successful_count"]) for run in normalized_runs)
    error_count = sum(int(run["error_count"]) for run in normalized_runs)

    return {
        "model": model_name,
        "run_count": run_count,
        "sample_count": sample_count,
        "total_evaluations": run_count * sample_count,
        "successful_count": successful_count,
        "error_count": error_count,
        "mean_prediction_agreement": (fmean(agreement_values) if agreement_values else 0.0),
        "fully_stable_scene_count": sum(
            bool(scene["all_runs_agree"]) for scene in scene_prediction_agreement
        ),
        "metric_summary": metric_summary,
        "scene_prediction_agreement": scene_prediction_agreement,
        "runs": normalized_runs,
    }


def _validated_run(
    report: dict[str, object],
    *,
    index: int,
    model_name: str,
) -> dict[str, object]:
    if report.get("model") != model_name:
        raise ValueError(f"Run {index} model does not match {model_name!r}")

    results = report.get("results")

    if not isinstance(results, list) or not all(isinstance(result, dict) for result in results):
        raise ValueError(f"Run {index} has invalid results")

    sample_count = report.get("sample_count")

    if (
        isinstance(sample_count, bool)
        or not isinstance(sample_count, int)
        or sample_count != len(results)
    ):
        raise ValueError(f"Run {index} has an invalid sample count")

    successful_count = report.get("successful_count")
    error_count = report.get("error_count")

    for name, value in (
        ("successful_count", successful_count),
        ("error_count", error_count),
    ):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"Run {index} has an invalid {name}")

    if successful_count + error_count != sample_count:
        raise ValueError(f"Run {index} success and error counts do not equal its sample count")

    for field in REPEATED_METRIC_FIELDS:
        value = report.get(field)

        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not isfinite(float(value))
        ):
            raise ValueError(f"Run {index} has an invalid metric {field}")

    return {
        **report,
        "run_index": index,
        "results": results,
    }


def _sample_identity(
    results: object,
) -> tuple[tuple[str, str], ...]:
    if not isinstance(results, list):
        raise ValueError("Run results must be a list")

    identity: list[tuple[str, str]] = []

    for result in results:
        if not isinstance(result, dict):
            raise ValueError("Run result must be an object")

        sample_id = result.get("id")
        image = result.get("image")

        if not isinstance(sample_id, str) or not isinstance(
            image,
            str,
        ):
            raise ValueError("Run results must contain string id and image fields")

        identity.append((sample_id, image))

    return tuple(identity)


def _summarize_metric(
    *,
    reports: Sequence[dict[str, object]],
    field: str,
) -> dict[str, float]:
    values = [float(report[field]) for report in reports]

    return {
        "mean": fmean(values),
        "population_std": pstdev(values),
        "min": min(values),
        "max": max(values),
    }


def _scene_prediction_agreement(
    reports: Sequence[dict[str, object]],
) -> list[dict[str, object]]:
    results_by_run = [report["results"] for report in reports]

    first_results = results_by_run[0]
    run_count = len(results_by_run)
    agreement: list[dict[str, object]] = []

    for sample_index, first_result in enumerate(first_results):
        signatures = [_prediction_signature(results[sample_index]) for results in results_by_run]

        counts = Counter(signatures)
        modal_count = max(counts.values())

        variants = []

        for signature, count in sorted(
            counts.items(),
            key=lambda item: json.dumps(
                _signature_payload(item[0]),
                sort_keys=True,
            ),
        ):
            variants.append(
                {
                    **_signature_payload(signature),
                    "count": count,
                    "rate": count / run_count,
                }
            )

        agreement.append(
            {
                "id": first_result["id"],
                "image": first_result["image"],
                "run_count": run_count,
                "successful_count": sum(
                    result["error"] is None
                    for result in (results[sample_index] for results in results_by_run)
                ),
                "agreement_rate": modal_count / run_count,
                "all_runs_agree": len(counts) == 1,
                "unique_prediction_count": len(counts),
                "prediction_variants": variants,
            }
        )

    return agreement


def _prediction_signature(
    result: dict[str, object],
) -> PredictionSignature:
    raw_hazards = result.get("predicted_hazards")

    if not isinstance(raw_hazards, list) or not all(
        isinstance(hazard, str) for hazard in raw_hazards
    ):
        raise ValueError("Predicted hazards must be an array of strings")

    predicted_traversability = result.get("predicted_traversability")
    model_action = result.get("model_action")
    enforced_action = result.get("enforced_action")

    for field_name, value in (
        (
            "predicted_traversability",
            predicted_traversability,
        ),
        ("model_action", model_action),
        ("enforced_action", enforced_action),
    ):
        if value is not None and not isinstance(value, str):
            raise ValueError(f"{field_name} must be a string or null")

    return (
        predicted_traversability,
        tuple(sorted(raw_hazards)),
        model_action,
        enforced_action,
        result.get("error") is not None,
    )


def _signature_payload(
    signature: PredictionSignature,
) -> dict[str, object]:
    (
        predicted_traversability,
        predicted_hazards,
        model_action,
        enforced_action,
        failed,
    ) = signature

    return {
        "predicted_traversability": predicted_traversability,
        "predicted_hazards": list(predicted_hazards),
        "model_action": model_action,
        "enforced_action": enforced_action,
        "failed": failed,
    }

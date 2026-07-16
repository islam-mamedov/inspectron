from __future__ import annotations

import re

from builtin_interfaces.msg import Time
from inspectron_safety_supervisor.msg import SceneAssessment as SceneAssessmentMessage

from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
)

_TRAVERSABILITY_TO_MESSAGE = {
    Traversability.CLEAR: SceneAssessmentMessage.TRAVERSABILITY_CLEAR,
    Traversability.RESTRICTED: SceneAssessmentMessage.TRAVERSABILITY_RESTRICTED,
    Traversability.BLOCKED: SceneAssessmentMessage.TRAVERSABILITY_BLOCKED,
    Traversability.UNKNOWN: SceneAssessmentMessage.TRAVERSABILITY_UNKNOWN,
}

_HAZARD_TO_MESSAGE = {
    HazardType.HUMAN_IN_PATH: SceneAssessmentMessage.HAZARD_HUMAN_IN_PATH,
    HazardType.DEBRIS: SceneAssessmentMessage.HAZARD_DEBRIS,
    HazardType.LIQUID_SPILL: SceneAssessmentMessage.HAZARD_LIQUID_SPILL,
    HazardType.OPEN_EDGE: SceneAssessmentMessage.HAZARD_OPEN_EDGE,
    HazardType.FIRE_OR_SMOKE: SceneAssessmentMessage.HAZARD_FIRE_OR_SMOKE,
    HazardType.UNSTABLE_LOAD: SceneAssessmentMessage.HAZARD_UNSTABLE_LOAD,
}

_ACTION_TO_MESSAGE = {
    RecommendedAction.PROCEED: SceneAssessmentMessage.ACTION_PROCEED,
    RecommendedAction.SLOW_DOWN: SceneAssessmentMessage.ACTION_SLOW_DOWN,
    RecommendedAction.STOP: SceneAssessmentMessage.ACTION_STOP,
    RecommendedAction.REROUTE: SceneAssessmentMessage.ACTION_REROUTE,
    RecommendedAction.INSPECT_CLOSER: SceneAssessmentMessage.ACTION_INSPECT_CLOSER,
}


def assessment_to_message(
    assessment: SceneAssessment,
    *,
    observed_at: Time,
) -> SceneAssessmentMessage:
    message = SceneAssessmentMessage()
    message.traversability = _TRAVERSABILITY_TO_MESSAGE[assessment.traversability]
    message.hazards = sorted(_HAZARD_TO_MESSAGE[hazard] for hazard in assessment.hazards)
    message.recommended_action = _ACTION_TO_MESSAGE[assessment.recommended_action]
    message.confidence = assessment.confidence
    message.view_quality = assessment.view_quality
    message.observed_at = observed_at
    message.evidence_id = assessment.evidence_id
    return message


def failure_assessment(
    *,
    waypoint: str,
    evidence_id: str,
) -> SceneAssessment:
    return SceneAssessment(
        waypoint=waypoint,
        evidence_id=evidence_id,
        traversability=Traversability.UNKNOWN,
        hazards=frozenset(),
        recommended_action=RecommendedAction.INSPECT_CLOSER,
        confidence=0.0,
        view_quality=0.0,
    )


def validate_compressed_image(
    *,
    image_format: str,
    data: bytes,
    max_image_bytes: int,
) -> str:
    if max_image_bytes <= 0:
        raise ValueError("max_image_bytes must be positive")

    if not data:
        raise ValueError("compressed image is empty")

    if len(data) > max_image_bytes:
        raise ValueError(f"compressed image exceeds {max_image_bytes} byte limit")

    normalized_format = image_format.lower()

    if "jpeg" in normalized_format or "jpg" in normalized_format:
        if not data.startswith(b"\xff\xd8\xff"):
            raise ValueError("compressed image does not have a JPEG signature")
        return ".jpg"

    if "png" in normalized_format:
        if not data.startswith(b"\x89PNG\r\n\x1a\n"):
            raise ValueError("compressed image does not have a PNG signature")
        return ".png"

    raise ValueError(f"unsupported compressed image format: {image_format}")


def make_evidence_id(
    *,
    waypoint: str,
    seconds: int,
    nanoseconds: int,
    sequence: int,
) -> str:
    safe_waypoint = re.sub(
        r"[^A-Za-z0-9_.-]+",
        "_",
        waypoint,
    ).strip("_")

    if not safe_waypoint:
        safe_waypoint = "unknown_waypoint"

    return f"{safe_waypoint}-{seconds}.{nanoseconds:09d}-{sequence:06d}"

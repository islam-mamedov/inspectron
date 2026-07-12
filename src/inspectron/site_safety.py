from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class Traversability(StrEnum):
    CLEAR = "clear"
    RESTRICTED = "restricted"
    BLOCKED = "blocked"
    UNKNOWN = "unknown"


class HazardType(StrEnum):
    HUMAN_IN_PATH = "human_in_path"
    DEBRIS = "debris"
    LIQUID_SPILL = "liquid_spill"
    OPEN_EDGE = "open_edge"
    FIRE_OR_SMOKE = "fire_or_smoke"
    UNSTABLE_LOAD = "unstable_load"


class RecommendedAction(StrEnum):
    PROCEED = "proceed"
    SLOW_DOWN = "slow_down"
    STOP = "stop"
    REROUTE = "reroute"
    INSPECT_CLOSER = "inspect_closer"


CRITICAL_HAZARDS = frozenset(
    {
        HazardType.HUMAN_IN_PATH,
        HazardType.OPEN_EDGE,
        HazardType.FIRE_OR_SMOKE,
        HazardType.UNSTABLE_LOAD,
    }
)


@dataclass(frozen=True, slots=True)
class SceneAssessment:
    waypoint: str
    evidence_id: str
    traversability: Traversability
    hazards: frozenset[HazardType]
    recommended_action: RecommendedAction
    confidence: float
    view_quality: float

    def __post_init__(self) -> None:
        for field_name, value in (
            ("confidence", self.confidence),
            ("view_quality", self.view_quality),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{field_name} must be between 0 and 1")


def resolve_safe_action(
    assessment: SceneAssessment,
    *,
    confidence_threshold: float = 0.70,
    quality_threshold: float = 0.60,
) -> RecommendedAction:
    """Apply deterministic safety rules to a VLM assessment."""

    if assessment.hazards & CRITICAL_HAZARDS:
        return RecommendedAction.STOP

    if assessment.traversability is Traversability.BLOCKED:
        return RecommendedAction.REROUTE

    evidence_is_weak = (
        assessment.confidence < confidence_threshold
        or assessment.view_quality < quality_threshold
        or assessment.traversability is Traversability.UNKNOWN
    )

    if evidence_is_weak:
        return RecommendedAction.INSPECT_CLOSER

    if assessment.traversability is Traversability.RESTRICTED or assessment.hazards:
        return RecommendedAction.SLOW_DOWN

    return RecommendedAction.PROCEED


def find_consistency_violations(
    assessment: SceneAssessment,
) -> tuple[str, ...]:
    """Identify contradictions in a model-generated assessment."""

    violations: list[str] = []

    if assessment.traversability is Traversability.CLEAR and assessment.hazards:
        violations.append("clear_traversability_with_reported_hazards")

    safe_action = resolve_safe_action(assessment)

    if assessment.recommended_action is not safe_action:
        violations.append("model_action_disagrees_with_safety_policy")

    return tuple(violations)

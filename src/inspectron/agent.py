from __future__ import annotations

from collections.abc import Sequence

from inspectron.domain import (
    Action,
    ActionKind,
    DefectType,
    Finding,
    Observation,
)


class InspectionAgent:
    """Deterministic baseline inspection planner.

    A VLM planner will later use the same observation/action interface.
    """

    def __init__(
        self,
        required_waypoints: Sequence[str],
        *,
        confidence_threshold: float = 0.75,
        quality_threshold: float = 0.60,
        max_views_per_waypoint: int = 2,
    ) -> None:
        if not required_waypoints:
            raise ValueError("At least one waypoint is required")

        self.required_waypoints = tuple(required_waypoints)
        self.confidence_threshold = confidence_threshold
        self.quality_threshold = quality_threshold
        self.max_views_per_waypoint = max_views_per_waypoint

        self.visited_waypoints: set[str] = set()
        self.findings: dict[tuple[str, DefectType], Finding] = {}

    def observe(self, observation: Observation) -> None:
        self.visited_waypoints.add(observation.waypoint)

        supported = (
            observation.predicted_defect
            not in {DefectType.NONE, DefectType.UNKNOWN}
            and observation.confidence >= self.confidence_threshold
            and observation.view_quality >= self.quality_threshold
        )

        if not supported:
            return

        key = (observation.asset_id, observation.predicted_defect)
        existing = self.findings.get(key)

        evidence_ids = (observation.evidence_id,)
        previous_confidence = 0.0

        if existing is not None:
            evidence_ids = tuple(
                dict.fromkeys(
                    (*existing.evidence_ids, observation.evidence_id)
                )
            )
            previous_confidence = existing.confidence

        self.findings[key] = Finding(
            asset_id=observation.asset_id,
            defect_type=observation.predicted_defect,
            confidence=max(observation.confidence, previous_confidence),
            evidence_ids=evidence_ids,
        )

    def choose_action(self, observation: Observation) -> Action:
        evidence_is_weak = (
            observation.confidence < self.confidence_threshold
            or observation.view_quality < self.quality_threshold
        )

        another_view_is_available = (
            observation.view_index + 1 < self.max_views_per_waypoint
        )

        if evidence_is_weak and another_view_is_available:
            return Action(
                kind=ActionKind.INSPECT,
                target=observation.waypoint,
                reason="Evidence quality or confidence is below threshold",
            )

        for waypoint in self.required_waypoints:
            if waypoint not in self.visited_waypoints:
                return Action(
                    kind=ActionKind.MOVE,
                    target=waypoint,
                    reason="Required waypoint has not been inspected",
                )

        return Action(
            kind=ActionKind.REPORT,
            reason="All required waypoints have been inspected",
        )
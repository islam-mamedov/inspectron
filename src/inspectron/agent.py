from __future__ import annotations

from collections.abc import Sequence

from inspectron.domain import Action, ActionKind
from inspectron.site_safety import (
    RecommendedAction,
    SceneAssessment,
    resolve_safe_action,
)


class SiteSafetyAgent:
    """Converts scene assessments into safe robot actions."""

    def __init__(
        self,
        required_waypoints: Sequence[str],
        *,
        max_views_per_waypoint: int = 2,
        slow_speed_scale: float = 0.35,
        reroute_speed_scale: float = 0.50,
    ) -> None:
        if not required_waypoints:
            raise ValueError("At least one waypoint is required")

        if max_views_per_waypoint < 1:
            raise ValueError("max_views_per_waypoint must be positive")

        self.required_waypoints = tuple(required_waypoints)
        self.max_views_per_waypoint = max_views_per_waypoint
        self.slow_speed_scale = slow_speed_scale
        self.reroute_speed_scale = reroute_speed_scale

        self.visited_waypoints: set[str] = set()
        self.view_counts: dict[str, int] = {}
        self.assessments: list[SceneAssessment] = []
        self.policy_override_count = 0

    def observe(
        self,
        assessment: SceneAssessment,
    ) -> None:
        self.visited_waypoints.add(assessment.waypoint)

        self.view_counts[assessment.waypoint] = (
            self.view_counts.get(
                assessment.waypoint,
                0,
            )
            + 1
        )

        self.assessments.append(assessment)

    def choose_action(
        self,
        assessment: SceneAssessment,
    ) -> Action:
        safe_action = resolve_safe_action(assessment)

        if safe_action is not assessment.recommended_action:
            self.policy_override_count += 1

        if safe_action is RecommendedAction.STOP:
            return Action(
                kind=ActionKind.STOP,
                reason=("Safety policy requires an immediate stop"),
            )

        if safe_action is RecommendedAction.INSPECT_CLOSER:
            view_count = self.view_counts.get(
                assessment.waypoint,
                0,
            )

            if view_count < self.max_views_per_waypoint:
                return Action(
                    kind=ActionKind.INSPECT,
                    target=assessment.waypoint,
                    reason=("Evidence is insufficient; capture another view"),
                )

            return Action(
                kind=ActionKind.STOP,
                reason=("Evidence remains insufficient after maximum inspection views"),
            )

        next_waypoint = self._next_waypoint()

        if next_waypoint is None:
            return Action(
                kind=ActionKind.REPORT,
                reason=("All required waypoints have been assessed"),
            )

        if safe_action is RecommendedAction.REROUTE:
            return Action(
                kind=ActionKind.MOVE,
                target=next_waypoint,
                reason=("Current route is blocked; move through an alternate waypoint"),
                speed_scale=(self.reroute_speed_scale),
            )

        if safe_action is RecommendedAction.SLOW_DOWN:
            return Action(
                kind=ActionKind.MOVE,
                target=next_waypoint,
                reason=("Restricted scene; proceed at reduced speed"),
                speed_scale=self.slow_speed_scale,
            )

        return Action(
            kind=ActionKind.MOVE,
            target=next_waypoint,
            reason="Scene is clear; proceed",
            speed_scale=1.0,
        )

    def _next_waypoint(self) -> str | None:
        for waypoint in self.required_waypoints:
            if waypoint not in self.visited_waypoints:
                return waypoint

        return None

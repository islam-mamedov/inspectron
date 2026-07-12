from __future__ import annotations

from collections.abc import Collection

from inspectron.domain import Action, ActionKind, SafetyDecision


class SafetyGate:
    """Validates every proposed action before robot execution."""

    def __init__(self, allowed_waypoints: Collection[str]) -> None:
        self.allowed_waypoints = frozenset(allowed_waypoints)

    def validate(
        self,
        action: Action,
        *,
        current_waypoint: str,
        visited_waypoints: Collection[str],
        blocked_waypoints: Collection[str],
    ) -> SafetyDecision:
        if action.kind is ActionKind.MOVE:
            if action.target is None:
                return SafetyDecision(
                    allowed=False,
                    reason="Move action requires a target",
                )

            if action.speed_scale is None:
                return SafetyDecision(
                    allowed=False,
                    reason="Move action requires an explicit speed scale",
                )

            if action.target not in self.allowed_waypoints:
                return SafetyDecision(
                    allowed=False,
                    reason=f"Waypoint outside mission: {action.target}",
                )

            if action.target in blocked_waypoints:
                return SafetyDecision(
                    allowed=False,
                    reason=f"Waypoint is blocked: {action.target}",
                )

            return SafetyDecision(
                allowed=True,
                reason="Move target and speed are allowed",
            )

        if action.kind is ActionKind.INSPECT:
            if action.target != current_waypoint:
                return SafetyDecision(
                    allowed=False,
                    reason="Inspection target must be the current waypoint",
                )

            return SafetyDecision(
                allowed=True,
                reason="Inspection target is valid",
            )

        if action.kind is ActionKind.REPORT:
            missing = self.allowed_waypoints - set(visited_waypoints)

            if missing:
                return SafetyDecision(
                    allowed=False,
                    reason=f"Unvisited waypoints: {sorted(missing)}",
                )

            return SafetyDecision(
                allowed=True,
                reason="Inspection coverage is complete",
            )

        if action.kind is ActionKind.STOP:
            return SafetyDecision(
                allowed=True,
                reason="Emergency stop is always allowed",
            )

        return SafetyDecision(
            allowed=False,
            reason=f"Unsupported action: {action.kind}",
        )

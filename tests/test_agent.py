from __future__ import annotations

import unittest

from inspectron.agent import (
    SiteSafetyAgent,
)
from inspectron.domain import ActionKind
from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
)


def scene(
    *,
    waypoint: str = "aisle_a",
    traversability: Traversability,
    hazards: frozenset[HazardType] = frozenset(),
    action: RecommendedAction,
    confidence: float = 0.90,
    view_quality: float = 0.90,
) -> SceneAssessment:
    return SceneAssessment(
        waypoint=waypoint,
        evidence_id=f"{waypoint}_frame",
        traversability=traversability,
        hazards=hazards,
        recommended_action=action,
        confidence=confidence,
        view_quality=view_quality,
    )


class SiteSafetyAgentTests(unittest.TestCase):
    def test_human_in_path_forces_stop(self) -> None:
        agent = SiteSafetyAgent(["aisle_a", "aisle_b"])

        assessment = scene(
            traversability=Traversability.CLEAR,
            hazards=frozenset({HazardType.HUMAN_IN_PATH}),
            action=RecommendedAction.PROCEED,
        )

        agent.observe(assessment)
        command = agent.choose_action(assessment)

        self.assertEqual(
            command.kind,
            ActionKind.STOP,
        )
        self.assertEqual(
            agent.policy_override_count,
            1,
        )

    def test_weak_evidence_requests_second_view(
        self,
    ) -> None:
        agent = SiteSafetyAgent(
            ["aisle_a"],
            max_views_per_waypoint=2,
        )

        assessment = scene(
            traversability=Traversability.UNKNOWN,
            action=(RecommendedAction.INSPECT_CLOSER),
            confidence=0.40,
            view_quality=0.35,
        )

        agent.observe(assessment)

        first_command = agent.choose_action(assessment)

        self.assertEqual(
            first_command.kind,
            ActionKind.INSPECT,
        )

        agent.observe(assessment)

        second_command = agent.choose_action(assessment)

        self.assertEqual(
            second_command.kind,
            ActionKind.STOP,
        )

    def test_restricted_path_reduces_speed(
        self,
    ) -> None:
        agent = SiteSafetyAgent(["aisle_a", "aisle_b"])

        assessment = scene(
            traversability=(Traversability.RESTRICTED),
            hazards=frozenset({HazardType.DEBRIS}),
            action=RecommendedAction.SLOW_DOWN,
        )

        agent.observe(assessment)
        command = agent.choose_action(assessment)

        self.assertEqual(
            command.kind,
            ActionKind.MOVE,
        )
        self.assertEqual(
            command.target,
            "aisle_b",
        )
        self.assertEqual(
            command.speed_scale,
            0.35,
        )

    def test_blocked_route_selects_alternative(
        self,
    ) -> None:
        agent = SiteSafetyAgent(["aisle_a", "aisle_b"])

        assessment = scene(
            traversability=Traversability.BLOCKED,
            hazards=frozenset({HazardType.DEBRIS}),
            action=RecommendedAction.REROUTE,
        )

        agent.observe(assessment)
        command = agent.choose_action(assessment)

        self.assertEqual(
            command.kind,
            ActionKind.MOVE,
        )
        self.assertEqual(
            command.target,
            "aisle_b",
        )
        self.assertEqual(
            command.speed_scale,
            0.50,
        )

    def test_reports_after_final_safe_waypoint(
        self,
    ) -> None:
        agent = SiteSafetyAgent(["aisle_a"])

        assessment = scene(
            traversability=Traversability.CLEAR,
            action=RecommendedAction.PROCEED,
        )

        agent.observe(assessment)
        command = agent.choose_action(assessment)

        self.assertEqual(
            command.kind,
            ActionKind.REPORT,
        )


if __name__ == "__main__":
    unittest.main()

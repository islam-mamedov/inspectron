from __future__ import annotations

import unittest

from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
    find_consistency_violations,
    resolve_safe_action,
)


def assessment(
    *,
    traversability: Traversability,
    hazards: frozenset[HazardType] = frozenset(),
    action: RecommendedAction = RecommendedAction.PROCEED,
    confidence: float = 0.90,
    view_quality: float = 0.90,
) -> SceneAssessment:
    return SceneAssessment(
        waypoint="warehouse_a",
        evidence_id="frame_001",
        traversability=traversability,
        hazards=hazards,
        recommended_action=action,
        confidence=confidence,
        view_quality=view_quality,
    )


class SiteSafetyTests(unittest.TestCase):
    def test_critical_hazard_forces_stop(self) -> None:
        scene = assessment(
            traversability=Traversability.CLEAR,
            hazards=frozenset({HazardType.HUMAN_IN_PATH}),
        )

        self.assertEqual(
            resolve_safe_action(scene),
            RecommendedAction.STOP,
        )

    def test_blocked_path_forces_reroute(self) -> None:
        scene = assessment(
            traversability=Traversability.BLOCKED,
        )

        self.assertEqual(
            resolve_safe_action(scene),
            RecommendedAction.REROUTE,
        )

    def test_weak_evidence_requests_closer_view(self) -> None:
        scene = assessment(
            traversability=Traversability.UNKNOWN,
            confidence=0.45,
            view_quality=0.40,
        )

        self.assertEqual(
            resolve_safe_action(scene),
            RecommendedAction.INSPECT_CLOSER,
        )

    def test_clear_safe_scene_allows_progress(self) -> None:
        scene = assessment(
            traversability=Traversability.CLEAR,
        )

        self.assertEqual(
            resolve_safe_action(scene),
            RecommendedAction.PROCEED,
        )

    def test_detects_model_policy_disagreement(self) -> None:
        scene = assessment(
            traversability=Traversability.BLOCKED,
            action=RecommendedAction.PROCEED,
        )

        self.assertIn(
            "model_action_disagrees_with_safety_policy",
            find_consistency_violations(scene),
        )


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import unittest

from inspectron.agent import (
    SiteSafetyAgent,
)
from inspectron.domain import (
    ActionKind,
    CapturedFrame,
)
from inspectron.mission import (
    MissionStatus,
    run_site_safety_mission,
)
from inspectron.safety import SafetyGate
from inspectron.simulation import (
    MockSafetyPerception,
    MockSafetyRobot,
    SafetyScenario,
    baseline_site_safety_scenario,
)
from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
)


class SiteSafetyMissionTests(unittest.TestCase):
    def test_completes_safe_adaptive_mission(
        self,
    ) -> None:
        scenario = baseline_site_safety_scenario()

        robot = MockSafetyRobot(scenario)

        result = run_site_safety_mission(
            agent=SiteSafetyAgent(scenario.required_waypoints),
            robot=robot,
            perception=MockSafetyPerception(scenario.assessments),
            safety_gate=SafetyGate(scenario.required_waypoints),
        )

        self.assertEqual(
            result.status,
            MissionStatus.COMPLETED,
        )
        self.assertEqual(result.coverage, 1.0)
        self.assertTrue(robot.stopped)
        self.assertEqual(
            robot.motion_log,
            [
                ("aisle_b", 1.0),
                ("aisle_c", 0.35),
            ],
        )
        self.assertEqual(
            robot.inspection_count,
            1,
        )
        self.assertEqual(
            [action.kind for action in result.action_trace],
            [
                ActionKind.MOVE,
                ActionKind.MOVE,
                ActionKind.INSPECT,
                ActionKind.REPORT,
            ],
        )

    def test_human_hazard_stops_mission(
        self,
    ) -> None:
        scenario = SafetyScenario(
            name="human_stop",
            frames={
                "aisle_a": (
                    CapturedFrame(
                        waypoint="aisle_a",
                        scene_id="scene_a",
                        evidence_id="human_0",
                    ),
                ),
                "aisle_b": (
                    CapturedFrame(
                        waypoint="aisle_b",
                        scene_id="scene_b",
                        evidence_id="clear_0",
                    ),
                ),
            },
            assessments={
                "human_0": SceneAssessment(
                    waypoint="aisle_a",
                    evidence_id="human_0",
                    traversability=(Traversability.CLEAR),
                    hazards=frozenset({HazardType.HUMAN_IN_PATH}),
                    recommended_action=(RecommendedAction.PROCEED),
                    confidence=0.95,
                    view_quality=0.90,
                ),
            },
        )

        robot = MockSafetyRobot(scenario)

        result = run_site_safety_mission(
            agent=SiteSafetyAgent(scenario.required_waypoints),
            robot=robot,
            perception=MockSafetyPerception(scenario.assessments),
            safety_gate=SafetyGate(scenario.required_waypoints),
        )

        self.assertEqual(
            result.status,
            MissionStatus.SAFETY_STOP,
        )
        self.assertEqual(
            result.action_trace[-1].kind,
            ActionKind.STOP,
        )
        self.assertEqual(
            result.policy_override_count,
            1,
        )
        self.assertEqual(result.coverage, 0.5)
        self.assertTrue(robot.stopped)


if __name__ == "__main__":
    unittest.main()

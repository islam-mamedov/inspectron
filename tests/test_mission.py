import unittest
from dataclasses import replace

from inspectron.agent import InspectionAgent
from inspectron.domain import ActionKind
from inspectron.mission import run_mission
from inspectron.safety import SafetyGate
from inspectron.simulation import (
    MockPerception,
    MockRobot,
    baseline_scenario,
)


class MissionTests(unittest.TestCase):
    def test_robot_and_perception_are_separate_components(self) -> None:
        scenario = baseline_scenario()
        robot = MockRobot(scenario)
        perception = MockPerception(scenario.predictions)

        result = run_mission(
            agent=InspectionAgent(scenario.required_waypoints),
            robot=robot,
            perception=perception,
            safety_gate=SafetyGate(scenario.required_waypoints),
        )

        self.assertTrue(result.completed)
        self.assertTrue(robot.stopped)

        self.assertEqual(
            perception.analyzed_evidence_ids,
            [
                "image_a_0",
                "image_b_0",
                "image_c_0",
                "image_c_1",
            ],
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

    def test_safety_gate_prevents_move_to_blocked_waypoint(self) -> None:
        scenario = replace(
            baseline_scenario(),
            blocked_waypoints=frozenset({"bay_b"}),
        )

        robot = MockRobot(scenario)

        result = run_mission(
            agent=InspectionAgent(scenario.required_waypoints),
            robot=robot,
            perception=MockPerception(scenario.predictions),
            safety_gate=SafetyGate(scenario.required_waypoints),
        )

        self.assertFalse(result.completed)
        self.assertTrue(robot.stopped)
        self.assertEqual(len(result.safety_violations), 1)
        self.assertIn("blocked", result.safety_violations[0])


if __name__ == "__main__":
    unittest.main()

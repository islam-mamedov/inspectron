from __future__ import annotations

import unittest

from inspectron.agent import SiteSafetyAgent
from inspectron.domain import Action, ActionKind, CapturedFrame
from inspectron.mission import MissionStatus, run_site_safety_mission
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


class AlwaysFailingPerception:
    def analyze(self, frame: CapturedFrame) -> SceneAssessment:
        raise RuntimeError(f"Perception unavailable for {frame.evidence_id}")


class FailAfterFirstPerception:
    def __init__(self, assessments: dict[str, SceneAssessment]) -> None:
        self.delegate = MockSafetyPerception(assessments)
        self.call_count = 0

    def analyze(self, frame: CapturedFrame) -> SceneAssessment:
        self.call_count += 1

        if self.call_count > 1:
            raise RuntimeError("Perception failed after robot movement")

        return self.delegate.analyze(frame)


class MismatchedPerception:
    def analyze(self, frame: CapturedFrame) -> SceneAssessment:
        return SceneAssessment(
            waypoint="forged_waypoint",
            evidence_id=frame.evidence_id,
            traversability=Traversability.CLEAR,
            hazards=frozenset(),
            recommended_action=RecommendedAction.PROCEED,
            confidence=0.95,
            view_quality=0.95,
        )


class SafetyRegressionTests(unittest.TestCase):
    def test_rejects_move_without_target(self) -> None:
        gate = SafetyGate(["aisle_a", "aisle_b"])

        decision = gate.validate(
            Action(
                kind=ActionKind.MOVE,
                speed_scale=0.50,
            ),
            current_waypoint="aisle_a",
            visited_waypoints={"aisle_a"},
            blocked_waypoints=set(),
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(
            decision.reason,
            "Move action requires a target",
        )

    def test_rejects_move_without_speed(self) -> None:
        gate = SafetyGate(["aisle_a", "aisle_b"])

        decision = gate.validate(
            Action(
                kind=ActionKind.MOVE,
                target="aisle_b",
            ),
            current_waypoint="aisle_a",
            visited_waypoints={"aisle_a"},
            blocked_waypoints=set(),
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(
            decision.reason,
            "Move action requires an explicit speed scale",
        )

    def test_blocked_final_waypoint_stops(self) -> None:
        scenario = SafetyScenario(
            name="blocked_final_waypoint",
            frames={
                "aisle_a": (
                    CapturedFrame(
                        waypoint="aisle_a",
                        scene_id="blocked_scene",
                        evidence_id="blocked_0",
                    ),
                ),
            },
            assessments={
                "blocked_0": SceneAssessment(
                    waypoint="aisle_a",
                    evidence_id="blocked_0",
                    traversability=Traversability.BLOCKED,
                    hazards=frozenset({HazardType.DEBRIS}),
                    recommended_action=RecommendedAction.REROUTE,
                    confidence=0.95,
                    view_quality=0.95,
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
        self.assertTrue(robot.stopped)

    def test_initial_perception_failure_stops_robot(self) -> None:
        scenario = baseline_site_safety_scenario()
        robot = MockSafetyRobot(scenario)

        result = run_site_safety_mission(
            agent=SiteSafetyAgent(scenario.required_waypoints),
            robot=robot,
            perception=AlwaysFailingPerception(),
            safety_gate=SafetyGate(scenario.required_waypoints),
        )

        self.assertEqual(
            result.status,
            MissionStatus.RUNTIME_FAILURE,
        )
        self.assertIn(
            "Perception unavailable",
            result.failure_reason or "",
        )
        self.assertTrue(robot.stopped)
        self.assertEqual(result.coverage, 0.0)

    def test_perception_failure_after_move_stops_robot(self) -> None:
        scenario = baseline_site_safety_scenario()
        robot = MockSafetyRobot(scenario)

        result = run_site_safety_mission(
            agent=SiteSafetyAgent(scenario.required_waypoints),
            robot=robot,
            perception=FailAfterFirstPerception(scenario.assessments),
            safety_gate=SafetyGate(scenario.required_waypoints),
        )

        self.assertEqual(
            result.status,
            MissionStatus.RUNTIME_FAILURE,
        )
        self.assertEqual(
            robot.motion_log,
            [("aisle_b", 1.0)],
        )
        self.assertIn(
            "failed after robot movement",
            result.failure_reason or "",
        )
        self.assertTrue(robot.stopped)

    def test_rejects_mismatched_perception_metadata(self) -> None:
        scenario = baseline_site_safety_scenario()
        robot = MockSafetyRobot(scenario)

        result = run_site_safety_mission(
            agent=SiteSafetyAgent(scenario.required_waypoints),
            robot=robot,
            perception=MismatchedPerception(),
            safety_gate=SafetyGate(scenario.required_waypoints),
        )

        self.assertEqual(
            result.status,
            MissionStatus.RUNTIME_FAILURE,
        )
        self.assertIn(
            "does not match the captured frame",
            result.failure_reason or "",
        )
        self.assertEqual(result.coverage, 0.0)
        self.assertTrue(robot.stopped)

    def test_reusing_agent_starts_with_clean_state(self) -> None:
        scenario = baseline_site_safety_scenario()
        agent = SiteSafetyAgent(scenario.required_waypoints)

        results = []
        motion_logs = []

        for _ in range(2):
            robot = MockSafetyRobot(scenario)

            result = run_site_safety_mission(
                agent=agent,
                robot=robot,
                perception=MockSafetyPerception(scenario.assessments),
                safety_gate=SafetyGate(scenario.required_waypoints),
            )

            results.append(result)
            motion_logs.append(list(robot.motion_log))

        self.assertEqual(
            [result.status for result in results],
            [
                MissionStatus.COMPLETED,
                MissionStatus.COMPLETED,
            ],
        )
        self.assertEqual(
            motion_logs,
            [
                [
                    ("aisle_b", 1.0),
                    ("aisle_c", 0.35),
                ],
                [
                    ("aisle_b", 1.0),
                    ("aisle_c", 0.35),
                ],
            ],
        )
        self.assertEqual(
            [len(result.assessments) for result in results],
            [4, 4],
        )


if __name__ == "__main__":
    unittest.main()

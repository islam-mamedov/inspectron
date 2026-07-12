import unittest

from inspectron.domain import Action, ActionKind
from inspectron.safety import SafetyGate


class SafetyGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.gate = SafetyGate(["bay_a", "bay_b"])

    def test_rejects_waypoint_outside_mission(self) -> None:
        decision = self.gate.validate(
            Action(ActionKind.MOVE, target="loading_dock"),
            current_waypoint="bay_a",
            visited_waypoints={"bay_a"},
            blocked_waypoints=set(),
        )

        self.assertFalse(decision.allowed)

    def test_rejects_early_report(self) -> None:
        decision = self.gate.validate(
            Action(ActionKind.REPORT),
            current_waypoint="bay_a",
            visited_waypoints={"bay_a"},
            blocked_waypoints=set(),
        )

        self.assertFalse(decision.allowed)
        self.assertIn("bay_b", decision.reason)

    def test_stop_is_always_allowed(self) -> None:
        decision = self.gate.validate(
            Action(ActionKind.STOP),
            current_waypoint="bay_a",
            visited_waypoints=set(),
            blocked_waypoints={"bay_a", "bay_b"},
        )

        self.assertTrue(decision.allowed)


if __name__ == "__main__":
    unittest.main()
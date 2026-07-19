from __future__ import annotations

import json
import unittest

from inspectron_e2e_simulation.scenario_logic import (
    DESIRED_MODE_ALWAYS,
    DESIRED_MODE_AUTHORIZED_ONLY,
)
from inspectron_e2e_simulation.scenarios import (
    HAZARD_STOP_SCENARIO,
    SAFE_MISSION_SCENARIO,
    scenario_definition,
)

from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    Traversability,
)

REQUIRED_FIELDS = {
    "traversability",
    "hazards",
    "recommended_action",
    "confidence",
    "view_quality",
}


class ScenarioDefinitionTest(unittest.TestCase):
    def _parsed_fixture(self, name):
        scenario = scenario_definition(name)
        payload = json.loads(scenario.fixture_response_json)

        self.assertEqual(set(payload), REQUIRED_FIELDS)
        Traversability(payload["traversability"])
        RecommendedAction(payload["recommended_action"])

        for hazard in payload["hazards"]:
            HazardType(hazard)

        self.assertTrue(0.0 <= payload["confidence"] <= 1.0)
        self.assertTrue(0.0 <= payload["view_quality"] <= 1.0)
        return scenario, payload

    def test_safe_scenario_produces_clear_proceed_fixture(self):
        scenario, payload = self._parsed_fixture(SAFE_MISSION_SCENARIO)
        self.assertEqual(payload["traversability"], "clear")
        self.assertEqual(payload["hazards"], [])
        self.assertEqual(payload["recommended_action"], "proceed")
        self.assertGreaterEqual(payload["confidence"], 0.70)
        self.assertGreaterEqual(payload["view_quality"], 0.60)
        self.assertEqual(
            scenario.desired_velocity_mode,
            DESIRED_MODE_AUTHORIZED_ONLY,
        )
        self.assertFalse(scenario.abort_on_safety_stop)

    def test_hazard_scenario_produces_critical_stop_fixture(self):
        scenario, payload = self._parsed_fixture(HAZARD_STOP_SCENARIO)
        self.assertIn("human_in_path", payload["hazards"])
        self.assertEqual(payload["recommended_action"], "stop")
        # Confidence and view quality must clear the supervisor thresholds so
        # the stop is attributed to the critical hazard, not weak evidence.
        self.assertGreaterEqual(payload["confidence"], 0.70)
        self.assertGreaterEqual(payload["view_quality"], 0.60)
        self.assertEqual(scenario.desired_velocity_mode, DESIRED_MODE_ALWAYS)
        self.assertTrue(scenario.abort_on_safety_stop)

    def test_scenarios_share_waypoints_with_unique_names(self):
        safe = scenario_definition(SAFE_MISSION_SCENARIO)
        hazard = scenario_definition(HAZARD_STOP_SCENARIO)
        self.assertEqual(safe.waypoints, hazard.waypoints)
        self.assertEqual(len(set(safe.waypoints)), len(safe.waypoints))

    def test_unknown_scenario_rejected(self):
        with self.assertRaises(ValueError):
            scenario_definition("unknown")


if __name__ == "__main__":
    unittest.main()

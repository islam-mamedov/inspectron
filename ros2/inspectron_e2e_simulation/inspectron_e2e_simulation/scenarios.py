from __future__ import annotations

import json
from dataclasses import dataclass

from inspectron_e2e_simulation.scenario_logic import (
    DESIRED_MODE_ALWAYS,
    DESIRED_MODE_AUTHORIZED_ONLY,
)

SAFE_MISSION_SCENARIO = "safe_mission"
HAZARD_STOP_SCENARIO = "hazard_stop"

DEFAULT_WAYPOINTS = ("aisle_a", "aisle_b", "aisle_c")

SAFE_FIXTURE_RESPONSE = {
    "traversability": "clear",
    "hazards": [],
    "recommended_action": "proceed",
    "confidence": 0.95,
    "view_quality": 0.90,
}

HAZARD_FIXTURE_RESPONSE = {
    "traversability": "restricted",
    "hazards": ["human_in_path"],
    "recommended_action": "stop",
    "confidence": 0.93,
    "view_quality": 0.88,
}


@dataclass(frozen=True, slots=True)
class ScenarioDefinition:
    name: str
    waypoints: tuple[str, ...]
    fixture_response_json: str
    desired_velocity_mode: str
    abort_on_safety_stop: bool
    scene_id: str


def scenario_definition(name: str) -> ScenarioDefinition:
    if name == SAFE_MISSION_SCENARIO:
        return ScenarioDefinition(
            name=name,
            waypoints=DEFAULT_WAYPOINTS,
            fixture_response_json=json.dumps(SAFE_FIXTURE_RESPONSE),
            desired_velocity_mode=DESIRED_MODE_AUTHORIZED_ONLY,
            abort_on_safety_stop=False,
            scene_id="e2e_safe_mission",
        )

    if name == HAZARD_STOP_SCENARIO:
        return ScenarioDefinition(
            name=name,
            waypoints=DEFAULT_WAYPOINTS,
            fixture_response_json=json.dumps(HAZARD_FIXTURE_RESPONSE),
            desired_velocity_mode=DESIRED_MODE_ALWAYS,
            abort_on_safety_stop=True,
            scene_id="e2e_hazard_stop",
        )

    raise ValueError(f"Unknown scenario: {name}")

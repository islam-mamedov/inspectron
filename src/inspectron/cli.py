from __future__ import annotations

import json

from inspectron.agent import SiteSafetyAgent
from inspectron.mission import (
    run_site_safety_mission,
)
from inspectron.safety import SafetyGate
from inspectron.simulation import (
    MockSafetyPerception,
    MockSafetyRobot,
    baseline_site_safety_scenario,
)
from inspectron.site_safety import (
    resolve_safe_action,
)


def main() -> None:
    scenario = baseline_site_safety_scenario()

    robot = MockSafetyRobot(scenario)

    result = run_site_safety_mission(
        agent=SiteSafetyAgent(scenario.required_waypoints),
        robot=robot,
        perception=MockSafetyPerception(scenario.assessments),
        safety_gate=SafetyGate(scenario.required_waypoints),
    )

    output = {
        "scenario": scenario.name,
        "status": result.status.value,
        "coverage": result.coverage,
        "policy_override_count": result.policy_override_count,
        "safety_violations": result.safety_violations,
        "failure_reason": result.failure_reason,
        "assessments": [
            {
                "waypoint": assessment.waypoint,
                "evidence_id": (assessment.evidence_id),
                "traversability": (assessment.traversability.value),
                "hazards": sorted(hazard.value for hazard in assessment.hazards),
                "model_action": (assessment.recommended_action.value),
                "enforced_action": (resolve_safe_action(assessment).value),
                "confidence": (assessment.confidence),
                "view_quality": (assessment.view_quality),
            }
            for assessment in result.assessments
        ],
        "action_trace": [
            {
                "action": action.kind.value,
                "target": action.target,
                "speed_scale": (action.speed_scale),
                "reason": action.reason,
            }
            for action in result.action_trace
        ],
        "motion_log": [
            {
                "target": target,
                "speed_scale": speed_scale,
            }
            for target, speed_scale in robot.motion_log
        ],
    }

    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()

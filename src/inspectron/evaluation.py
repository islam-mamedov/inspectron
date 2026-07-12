from __future__ import annotations

from dataclasses import asdict, dataclass

from inspectron.agent import InspectionAgent
from inspectron.domain import DefectType, EpisodeResult
from inspectron.mission import run_mission
from inspectron.safety import SafetyGate
from inspectron.simulation import (
    InspectionScenario,
    MockPerception,
    MockRobot,
)


@dataclass(frozen=True, slots=True)
class EpisodeMetrics:
    precision: float
    recall: float
    coverage: float
    steps: int
    safety_violations: int
    completed: bool

    def to_dict(self) -> dict[str, float | int | bool]:
        return asdict(self)


def run_episode(
    scenario: InspectionScenario,
    *,
    max_steps: int = 20,
) -> EpisodeResult:
    agent = InspectionAgent(scenario.required_waypoints)
    robot = MockRobot(scenario)
    perception = MockPerception(scenario.predictions)
    safety_gate = SafetyGate(scenario.required_waypoints)

    return run_mission(
        agent=agent,
        robot=robot,
        perception=perception,
        safety_gate=safety_gate,
        max_steps=max_steps,
    )


def score_episode(
    result: EpisodeResult,
    ground_truth: dict[str, DefectType],
) -> EpisodeMetrics:
    predicted = {(finding.asset_id, finding.defect_type) for finding in result.findings}

    expected = set(ground_truth.items())
    true_positives = len(predicted & expected)

    precision = true_positives / len(predicted) if predicted else float(not expected)

    recall = true_positives / len(expected) if expected else 1.0

    return EpisodeMetrics(
        precision=precision,
        recall=recall,
        coverage=result.coverage,
        steps=len(result.action_trace),
        safety_violations=len(result.safety_violations),
        completed=result.completed,
    )

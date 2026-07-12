from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Protocol

from inspectron.agent import (
    SiteSafetyAgent,
)
from inspectron.domain import (
    Action,
    ActionKind,
    CapturedFrame,
)
from inspectron.safety import SafetyGate
from inspectron.site_safety import SceneAssessment


class MissionStatus(StrEnum):
    COMPLETED = "completed"
    SAFETY_STOP = "safety_stop"
    VALIDATION_FAILURE = "validation_failure"
    STEP_LIMIT = "step_limit"


class SafetyRobotPort(Protocol):
    @property
    def current_waypoint(self) -> str: ...

    @property
    def blocked_waypoints(self) -> frozenset[str]: ...

    def reset(self) -> CapturedFrame: ...

    def move(
        self,
        target: str,
        *,
        speed_scale: float,
    ) -> CapturedFrame: ...

    def inspect(self) -> CapturedFrame: ...

    def stop(self) -> None: ...


class SafetyPerceptionPort(Protocol):
    def analyze(
        self,
        frame: CapturedFrame,
    ) -> SceneAssessment: ...


@dataclass(slots=True)
class SafetyMissionResult:
    status: MissionStatus
    assessments: list[SceneAssessment]
    action_trace: list[Action]
    visited_waypoints: set[str]
    required_waypoints: set[str]
    safety_violations: list[str] = field(default_factory=list)
    policy_override_count: int = 0

    @property
    def coverage(self) -> float:
        if not self.required_waypoints:
            return 1.0

        visited = self.visited_waypoints & self.required_waypoints

        return len(visited) / len(self.required_waypoints)


def run_site_safety_mission(
    *,
    agent: SiteSafetyAgent,
    robot: SafetyRobotPort,
    perception: SafetyPerceptionPort,
    safety_gate: SafetyGate,
    max_steps: int = 20,
) -> SafetyMissionResult:
    if max_steps < 1:
        raise ValueError("max_steps must be positive")

    frame = robot.reset()
    assessment = perception.analyze(frame)

    action_trace: list[Action] = []
    safety_violations: list[str] = []
    status = MissionStatus.STEP_LIMIT

    for _ in range(max_steps):
        agent.observe(assessment)
        action = agent.choose_action(assessment)
        action_trace.append(action)

        decision = safety_gate.validate(
            action,
            current_waypoint=(robot.current_waypoint),
            visited_waypoints=(agent.visited_waypoints),
            blocked_waypoints=(robot.blocked_waypoints),
        )

        if not decision.allowed:
            safety_violations.append(decision.reason)
            robot.stop()
            status = MissionStatus.VALIDATION_FAILURE
            break

        if action.kind is ActionKind.STOP:
            robot.stop()
            status = MissionStatus.SAFETY_STOP
            break

        if action.kind is ActionKind.REPORT:
            robot.stop()
            status = MissionStatus.COMPLETED
            break

        if action.kind is ActionKind.MOVE:
            if action.target is None:
                raise RuntimeError("Validated move action has no target")

            frame = robot.move(
                action.target,
                speed_scale=(action.speed_scale or 1.0),
            )

        elif action.kind is ActionKind.INSPECT:
            frame = robot.inspect()

        else:
            raise RuntimeError(f"Unsupported action: {action.kind}")

        assessment = perception.analyze(frame)
    else:
        robot.stop()

    return SafetyMissionResult(
        status=status,
        assessments=list(agent.assessments),
        action_trace=action_trace,
        visited_waypoints=set(agent.visited_waypoints),
        required_waypoints=set(agent.required_waypoints),
        safety_violations=safety_violations,
        policy_override_count=(agent.policy_override_count),
    )

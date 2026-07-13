from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Protocol

from inspectron.agent import SiteSafetyAgent
from inspectron.domain import Action, ActionKind, CapturedFrame
from inspectron.safety import SafetyGate
from inspectron.site_safety import SceneAssessment


class MissionStatus(StrEnum):
    COMPLETED = "completed"
    SAFETY_STOP = "safety_stop"
    VALIDATION_FAILURE = "validation_failure"
    RUNTIME_FAILURE = "runtime_failure"
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
    def analyze(self, frame: CapturedFrame) -> SceneAssessment: ...


@dataclass(slots=True)
class SafetyMissionResult:
    status: MissionStatus
    assessments: list[SceneAssessment]
    action_trace: list[Action]
    visited_waypoints: set[str]
    required_waypoints: set[str]
    safety_violations: list[str] = field(default_factory=list)
    policy_override_count: int = 0
    failure_reason: str | None = None

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

    agent.reset()

    action_trace: list[Action] = []
    safety_violations: list[str] = []
    status = MissionStatus.STEP_LIMIT
    failure_reason: str | None = None

    try:
        frame = robot.reset()
        assessment = perception.analyze(frame)
        _validate_assessment(frame, assessment)

        for _ in range(max_steps):
            agent.observe(assessment)
            action = agent.choose_action(assessment)
            action_trace.append(action)

            decision = safety_gate.validate(
                action,
                current_waypoint=robot.current_waypoint,
                visited_waypoints=agent.visited_waypoints,
                blocked_waypoints=robot.blocked_waypoints,
            )

            if not decision.allowed:
                safety_violations.append(decision.reason)
                status = MissionStatus.VALIDATION_FAILURE
                break

            if action.kind is ActionKind.STOP:
                status = MissionStatus.SAFETY_STOP
                break

            if action.kind is ActionKind.REPORT:
                status = MissionStatus.COMPLETED
                break

            if action.kind is ActionKind.MOVE:
                if action.target is None:
                    raise RuntimeError("Validated move action has no target")

                if action.speed_scale is None:
                    raise RuntimeError("Validated move action has no speed scale")

                frame = robot.move(
                    action.target,
                    speed_scale=action.speed_scale,
                )

            elif action.kind is ActionKind.INSPECT:
                frame = robot.inspect()

            else:
                raise RuntimeError(f"Unsupported action: {action.kind}")

            assessment = perception.analyze(frame)
            _validate_assessment(frame, assessment)

    except Exception as error:
        status = MissionStatus.RUNTIME_FAILURE
        failure_reason = f"{type(error).__name__}: {error}"

    finally:
        try:
            robot.stop()
        except Exception as error:
            stop_failure = f"{type(error).__name__}: {error}"

            if failure_reason is None:
                failure_reason = f"Robot stop failed: {stop_failure}"
            else:
                failure_reason = f"{failure_reason}; robot stop also failed: {stop_failure}"

            status = MissionStatus.RUNTIME_FAILURE

    return SafetyMissionResult(
        status=status,
        assessments=list(agent.assessments),
        action_trace=action_trace,
        visited_waypoints=set(agent.visited_waypoints),
        required_waypoints=set(agent.required_waypoints),
        safety_violations=safety_violations,
        policy_override_count=agent.policy_override_count,
        failure_reason=failure_reason,
    )


def _validate_assessment(
    frame: CapturedFrame,
    assessment: SceneAssessment,
) -> None:
    if assessment.waypoint != frame.waypoint:
        raise ValueError(
            "Assessment waypoint does not match the captured frame: "
            f"{assessment.waypoint!r} != {frame.waypoint!r}"
        )

    if assessment.evidence_id != frame.evidence_id:
        raise ValueError(
            "Assessment evidence ID does not match the captured frame: "
            f"{assessment.evidence_id!r} != {frame.evidence_id!r}"
        )

from __future__ import annotations

from inspectron.agent import InspectionAgent
from inspectron.domain import ActionKind, EpisodeResult
from inspectron.ports import PerceptionPort, RobotPort
from inspectron.safety import SafetyGate


def run_mission(
    *,
    agent: InspectionAgent,
    robot: RobotPort,
    perception: PerceptionPort,
    safety_gate: SafetyGate,
    max_steps: int = 20,
) -> EpisodeResult:
    if max_steps < 1:
        raise ValueError("max_steps must be positive")

    frame = robot.reset()
    observation = perception.analyze(frame)

    action_trace = []
    safety_violations = []
    completed = False

    for _ in range(max_steps):
        agent.observe(observation)
        action = agent.choose_action(observation)
        action_trace.append(action)

        decision = safety_gate.validate(
            action,
            current_waypoint=robot.current_waypoint,
            visited_waypoints=agent.visited_waypoints,
            blocked_waypoints=robot.blocked_waypoints,
        )

        if not decision.allowed:
            safety_violations.append(decision.reason)
            robot.stop()
            break

        if action.kind is ActionKind.REPORT:
            completed = True
            robot.stop()
            break

        if action.kind is ActionKind.STOP:
            robot.stop()
            break

        if action.kind is ActionKind.MOVE:
            if action.target is None:
                raise RuntimeError("Validated move action has no target")

            frame = robot.move(action.target)

        elif action.kind is ActionKind.INSPECT:
            frame = robot.inspect()

        else:
            raise RuntimeError(f"Unsupported action: {action.kind}")

        observation = perception.analyze(frame)
    else:
        robot.stop()

    return EpisodeResult(
        findings=list(agent.findings.values()),
        visited_waypoints=agent.visited_waypoints,
        required_waypoints=set(agent.required_waypoints),
        action_trace=action_trace,
        safety_violations=safety_violations,
        completed=completed,
    )

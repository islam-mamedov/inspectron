from __future__ import annotations

import math
import re
from dataclasses import dataclass
from enum import Enum

STATE_IDLE = 0
STATE_WAITING_FOR_POLICY = 1
STATE_MOVING = 2
STATE_PAUSED = 3
STATE_INSPECTING_CLOSER = 4
STATE_REROUTING = 5
STATE_SAFETY_STOPPED = 6
STATE_COMPLETED = 7
STATE_ABORTED = 8
STATE_EMERGENCY_STOPPED = 9

POLICY_ACTION_PROCEED = 0
POLICY_ACTION_SLOW_DOWN = 1
POLICY_STATUS_VALID = 0

REPORT_STATUS_RECORDING = 1
REPORT_STATUS_FINALIZED = 2
REPORT_STATUS_FAILED = 3

DESIRED_MODE_AUTHORIZED_ONLY = "authorized_only"
DESIRED_MODE_ALWAYS = "always"

MISSION_STATE_NAMES = {
    STATE_IDLE: "idle",
    STATE_WAITING_FOR_POLICY: "waiting_for_policy",
    STATE_MOVING: "moving",
    STATE_PAUSED: "paused",
    STATE_INSPECTING_CLOSER: "inspecting_closer",
    STATE_REROUTING: "rerouting",
    STATE_SAFETY_STOPPED: "safety_stopped",
    STATE_COMPLETED: "completed",
    STATE_ABORTED: "aborted",
    STATE_EMERGENCY_STOPPED: "emergency_stopped",
}

_STREAMING_MISSION_STATES = {STATE_WAITING_FOR_POLICY, STATE_MOVING}
_TERMINAL_MISSION_STATES = {STATE_COMPLETED, STATE_ABORTED, STATE_EMERGENCY_STOPPED}
_VELOCITY_EPSILON = 1e-9
_PRIME_RETRY_SECONDS = 0.5


def sanitize_waypoint(waypoint: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", waypoint).strip("_")
    return safe or "unknown_waypoint"


def evidence_matches_waypoint(evidence_id: str, waypoint: str) -> bool:
    return evidence_id.startswith(f"{sanitize_waypoint(waypoint)}-")


class GoalPhase(Enum):
    STREAMING = "streaming"
    DRAINING = "draining"
    AWAITING_STATE_CONFIRMATION = "awaiting_state_confirmation"
    REPORTED = "reported"


@dataclass(frozen=True, slots=True)
class PublishCameraFrame:
    waypoint: str


@dataclass(frozen=True, slots=True)
class PublishDesiredVelocity:
    linear_x: float
    angular_z: float


@dataclass(frozen=True, slots=True)
class PublishWaypointReached:
    waypoint: str


@dataclass(frozen=True, slots=True)
class CallMissionStart:
    pass


@dataclass(frozen=True, slots=True)
class CallMissionAbort:
    pass


@dataclass(frozen=True, slots=True)
class LogNote:
    message: str


@dataclass(frozen=True, slots=True)
class CompleteScenario:
    summary: str


@dataclass(slots=True)
class CommandVelocityStats:
    total_samples: int = 0
    nonzero_samples: int = 0
    nonzero_before_authorizing_policy: int = 0
    max_abs_linear_x: float = 0.0
    trailing_zero_samples: int = 0


@dataclass(slots=True)
class _OutstandingFrame:
    waypoint: str
    evidence_id: str | None = None
    assessment_id: str | None = None


class ScenarioDriverCore:
    def __init__(
        self,
        *,
        waypoints: list[str] | tuple[str, ...],
        desired_linear_x: float,
        desired_angular_z: float,
        min_authorized_motion_samples: int,
        frame_interval_seconds: float,
        desired_velocity_mode: str,
        auto_start: bool,
        abort_on_safety_stop: bool,
    ) -> None:
        normalized = tuple(str(waypoint).strip() for waypoint in waypoints)

        if not normalized:
            raise ValueError("At least one waypoint is required")

        if any(not waypoint for waypoint in normalized):
            raise ValueError("Waypoints cannot be empty")

        if len(set(normalized)) != len(normalized):
            raise ValueError("Waypoints must be unique")

        sanitized = [sanitize_waypoint(waypoint) for waypoint in normalized]

        if len(set(sanitized)) != len(sanitized):
            raise ValueError("Waypoints must remain unique after sanitization")

        for first in sanitized:
            for second in sanitized:
                if first != second and second.startswith(f"{first}-"):
                    raise ValueError("Waypoint names cannot nest as evidence prefixes")

        if not math.isfinite(desired_linear_x) or not math.isfinite(desired_angular_z):
            raise ValueError("Desired velocities must be finite")

        if abs(desired_linear_x) <= _VELOCITY_EPSILON:
            raise ValueError("desired_linear_x must be non-zero")

        if min_authorized_motion_samples < 1:
            raise ValueError("min_authorized_motion_samples must be at least 1")

        if not math.isfinite(frame_interval_seconds) or frame_interval_seconds <= 0.0:
            raise ValueError("frame_interval_seconds must be positive")

        if desired_velocity_mode not in {
            DESIRED_MODE_AUTHORIZED_ONLY,
            DESIRED_MODE_ALWAYS,
        }:
            raise ValueError("desired_velocity_mode must be authorized_only or always")

        self.waypoints = normalized
        self.desired_linear_x = float(desired_linear_x)
        self.desired_angular_z = float(desired_angular_z)
        self.min_authorized_motion_samples = int(min_authorized_motion_samples)
        self.frame_interval_seconds = float(frame_interval_seconds)
        self.desired_velocity_mode = desired_velocity_mode
        self.auto_start = bool(auto_start)
        self.abort_on_safety_stop = bool(abort_on_safety_stop)

        self.cmd_vel_stats = CommandVelocityStats()

        self._graph_ready = False
        self._outstanding: _OutstandingFrame | None = None
        self._next_frame_time = 0.0
        self._start_gate_open = False
        self._start_emitted = False
        self._abort_emitted = False
        self._complete_emitted = False
        self._authorizing_policy_seen = False
        self._reporter_recording = False
        self._report_finalized = False
        self._report_path = ""
        self._terminal_state: int | None = None
        self._mission_state = STATE_IDLE
        self._active_goal = ""
        self._motion_authorized = False
        self._state_last_evidence = ""
        self._goal_phase: dict[str, GoalPhase] = {}
        self._goal_last_evidence: dict[str, str] = {}
        self._authorized_motion_samples: dict[str, int] = {}

    @property
    def start_gate_open(self) -> bool:
        return self._start_gate_open

    @property
    def authorizing_policy_seen(self) -> bool:
        return self._authorizing_policy_seen

    @property
    def report_finalized(self) -> bool:
        return self._report_finalized

    @property
    def report_path(self) -> str:
        return self._report_path

    @property
    def terminal_state(self) -> int | None:
        return self._terminal_state

    def goal_phase(self, waypoint: str) -> GoalPhase | None:
        return self._goal_phase.get(waypoint)

    def on_graph_ready(self) -> list[object]:
        self._graph_ready = True
        return []

    def on_tick(self, now: float) -> list[object]:
        commands: list[object] = []

        desired = self._desired_velocity_command()

        if desired is not None:
            commands.append(desired)

        frame_waypoint = self._next_frame_waypoint()

        if frame_waypoint is not None and now >= self._next_frame_time:
            self._outstanding = _OutstandingFrame(waypoint=frame_waypoint)

            if self._start_gate_open:
                self._next_frame_time = now + self.frame_interval_seconds
            else:
                self._next_frame_time = now + max(
                    self.frame_interval_seconds,
                    _PRIME_RETRY_SECONDS,
                )

            commands.append(PublishCameraFrame(waypoint=frame_waypoint))

        return commands

    def on_perception_request(self) -> list[object]:
        self._next_frame_time = 0.0
        return []

    def on_evidence_capture(self, evidence_id: str) -> list[object]:
        outstanding = self._outstanding

        if outstanding is not None and evidence_matches_waypoint(
            evidence_id,
            outstanding.waypoint,
        ):
            outstanding.evidence_id = evidence_id
            return self._resolve_outstanding_if_complete()

        return []

    def on_assessment(self, evidence_id: str) -> list[object]:
        outstanding = self._outstanding

        if outstanding is not None and evidence_matches_waypoint(
            evidence_id,
            outstanding.waypoint,
        ):
            outstanding.assessment_id = evidence_id
            return self._resolve_outstanding_if_complete()

        return []

    def on_policy_decision(
        self,
        *,
        action: int,
        status: int,
        evidence_id: str,
    ) -> list[object]:
        commands: list[object] = []

        if status == POLICY_STATUS_VALID and action in {
            POLICY_ACTION_PROCEED,
            POLICY_ACTION_SLOW_DOWN,
        }:
            self._authorizing_policy_seen = True

        if not self._start_gate_open and evidence_matches_waypoint(
            evidence_id,
            self.waypoints[0],
        ):
            self._start_gate_open = True

            if self.auto_start and not self._start_emitted:
                self._start_emitted = True
                commands.append(CallMissionStart())
            else:
                commands.append(
                    LogNote("Prime decision observed; ready for operator mission start")
                )

        return commands

    def on_mission_state(
        self,
        *,
        state: int,
        active_goal: str,
        motion_authorized: bool,
        last_evidence_id: str,
    ) -> list[object]:
        self._mission_state = state
        self._active_goal = active_goal
        self._motion_authorized = motion_authorized
        self._state_last_evidence = last_evidence_id

        commands: list[object] = []

        if active_goal and active_goal not in self._goal_phase:
            self._goal_phase[active_goal] = GoalPhase.STREAMING
            self._authorized_motion_samples[active_goal] = 0

        if state in _TERMINAL_MISSION_STATES:
            self._terminal_state = state

        if state == STATE_SAFETY_STOPPED and self.abort_on_safety_stop and not self._abort_emitted:
            self._abort_emitted = True
            commands.append(CallMissionAbort())

        commands.extend(self._advance_goal_protocol())
        commands.extend(self._completion_commands())
        return commands

    def on_report_status(
        self,
        *,
        status: int,
        report_path: str,
        message: str,
    ) -> list[object]:
        commands: list[object] = []

        if status == REPORT_STATUS_RECORDING:
            self._reporter_recording = True
        elif status == REPORT_STATUS_FINALIZED:
            self._report_finalized = True

            if report_path:
                self._report_path = report_path
        elif status == REPORT_STATUS_FAILED:
            commands.append(LogNote(f"Evidence reporter failure: {message}"))

        commands.extend(self._completion_commands())
        return commands

    def on_cmd_vel(self, *, linear_x: float, angular_z: float) -> list[object]:
        stats = self.cmd_vel_stats
        stats.total_samples += 1

        nonzero = abs(linear_x) > _VELOCITY_EPSILON or abs(angular_z) > _VELOCITY_EPSILON

        if not nonzero:
            stats.trailing_zero_samples += 1
            return []

        stats.nonzero_samples += 1
        stats.trailing_zero_samples = 0
        stats.max_abs_linear_x = max(stats.max_abs_linear_x, abs(linear_x))

        if not self._authorizing_policy_seen:
            stats.nonzero_before_authorizing_policy += 1

        goal = self._active_goal

        if (
            goal
            and self._mission_state == STATE_MOVING
            and self._motion_authorized
            and self._goal_phase.get(goal) is GoalPhase.STREAMING
        ):
            samples = self._authorized_motion_samples.get(goal, 0) + 1
            self._authorized_motion_samples[goal] = samples

            if samples >= self.min_authorized_motion_samples:
                self._goal_phase[goal] = GoalPhase.DRAINING
                return self._advance_goal_protocol()

        return []

    def summary_text(self) -> str:
        stats = self.cmd_vel_stats

        if self._terminal_state is None:
            terminal = "none"
        else:
            terminal = MISSION_STATE_NAMES.get(
                self._terminal_state,
                str(self._terminal_state),
            )

        return (
            "scenario summary: "
            f"terminal_state={terminal} "
            f"report_finalized={self._report_finalized} "
            f"report_path={self._report_path or 'none'} "
            f"cmd_vel_samples={stats.total_samples} "
            f"nonzero_cmd_vel_samples={stats.nonzero_samples} "
            f"nonzero_before_authorizing_policy={stats.nonzero_before_authorizing_policy} "
            f"max_abs_linear_x={stats.max_abs_linear_x:.3f} "
            f"trailing_zero_cmd_vel_samples={stats.trailing_zero_samples}"
        )

    def _desired_velocity_command(self) -> PublishDesiredVelocity | None:
        if self.desired_velocity_mode == DESIRED_MODE_ALWAYS:
            return PublishDesiredVelocity(
                linear_x=self.desired_linear_x,
                angular_z=self.desired_angular_z,
            )

        if self._mission_state == STATE_MOVING and self._motion_authorized:
            return PublishDesiredVelocity(
                linear_x=self.desired_linear_x,
                angular_z=self.desired_angular_z,
            )

        return None

    def _next_frame_waypoint(self) -> str | None:
        if not self._graph_ready or self._outstanding is not None:
            return None

        if not self._start_gate_open:
            if self._mission_state == STATE_IDLE:
                return self.waypoints[0]

            return None

        if not self._reporter_recording:
            return None

        if self._mission_state not in _STREAMING_MISSION_STATES:
            return None

        goal = self._active_goal

        if not goal:
            return None

        if self._goal_phase.get(goal) is not GoalPhase.STREAMING:
            return None

        return goal

    def _resolve_outstanding_if_complete(self) -> list[object]:
        outstanding = self._outstanding

        if (
            outstanding is None
            or outstanding.evidence_id is None
            or outstanding.assessment_id is None
        ):
            return []

        commands: list[object] = []

        if outstanding.evidence_id != outstanding.assessment_id:
            commands.append(
                LogNote(
                    "Evidence and assessment identifiers diverged: "
                    f"{outstanding.evidence_id} vs {outstanding.assessment_id}"
                )
            )

        self._goal_last_evidence[outstanding.waypoint] = outstanding.assessment_id
        self._outstanding = None

        commands.extend(self._advance_goal_protocol())
        return commands

    def _advance_goal_protocol(self) -> list[object]:
        goal = self._active_goal

        if not goal:
            return []

        phase = self._goal_phase.get(goal)

        if phase is GoalPhase.DRAINING and self._outstanding is None:
            self._goal_phase[goal] = GoalPhase.AWAITING_STATE_CONFIRMATION
            phase = GoalPhase.AWAITING_STATE_CONFIRMATION

        if phase is not GoalPhase.AWAITING_STATE_CONFIRMATION:
            return []

        target = self._goal_last_evidence.get(goal, "")

        if not target:
            return []

        if self._mission_state != STATE_MOVING:
            return []

        if self._state_last_evidence != target:
            return []

        self._goal_phase[goal] = GoalPhase.REPORTED
        return [PublishWaypointReached(waypoint=goal)]

    def _completion_commands(self) -> list[object]:
        if not self.auto_start or self._complete_emitted:
            return []

        if self._terminal_state is None or not self._report_finalized:
            return []

        self._complete_emitted = True
        return [CompleteScenario(summary=self.summary_text())]

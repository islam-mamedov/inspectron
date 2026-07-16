from __future__ import annotations

import re
from dataclasses import dataclass
from enum import IntEnum

ACTION_PROCEED = 0
ACTION_SLOW_DOWN = 1
ACTION_STOP = 2
ACTION_REROUTE = 3
ACTION_INSPECT_CLOSER = 4

STATUS_VALID = 0
STATUS_INVALID = 1
STATUS_STALE = 2


class MissionPhase(IntEnum):
    IDLE = 0
    WAITING_FOR_POLICY = 1
    MOVING = 2
    PAUSED = 3
    INSPECTING_CLOSER = 4
    REROUTING = 5
    SAFETY_STOPPED = 6
    COMPLETED = 7
    ABORTED = 8
    EMERGENCY_STOPPED = 9


@dataclass(frozen=True, slots=True)
class MissionSnapshot:
    state: MissionPhase
    current_waypoint: str
    active_goal: str
    waypoint_index: int
    waypoint_count: int
    last_evidence_id: str
    reason: str
    motion_authorized: bool


@dataclass(frozen=True, slots=True)
class TransitionResult:
    accepted: bool
    message: str
    publish_goal: bool = False
    cancel_motion: bool = False
    request_perception: bool = False
    request_closer_view: bool = False
    reroute_request: str | None = None


class MissionStateMachine:
    def __init__(
        self,
        waypoints: list[str] | tuple[str, ...],
        *,
        policy_timeout_seconds: float,
        reroute_timeout_seconds: float,
    ) -> None:
        normalized = tuple(waypoint.strip() for waypoint in waypoints)

        if not normalized:
            raise ValueError("At least one waypoint is required")

        if any(not waypoint for waypoint in normalized):
            raise ValueError("Waypoints cannot be empty")

        if len(set(normalized)) != len(normalized):
            raise ValueError("Waypoints must be unique")

        if policy_timeout_seconds <= 0.0:
            raise ValueError("policy_timeout_seconds must be positive")

        if reroute_timeout_seconds <= 0.0:
            raise ValueError("reroute_timeout_seconds must be positive")

        self.waypoints = normalized
        self.policy_timeout_seconds = policy_timeout_seconds
        self.reroute_timeout_seconds = reroute_timeout_seconds

        self._state = MissionPhase.IDLE
        self._waypoint_index = 0
        self._active_goal = ""
        self._last_evidence_id = ""
        self._reason = "Mission is idle"
        self._motion_authorized = False
        self._last_policy_at: float | None = None
        self._latest_policy_allows_motion = False
        self._reroute_started_at: float | None = None
        self._emergency_stop_active = False

    @property
    def snapshot(self) -> MissionSnapshot:
        if self._waypoint_index < len(self.waypoints):
            current_waypoint = self.waypoints[self._waypoint_index]
        else:
            current_waypoint = self.waypoints[-1]

        return MissionSnapshot(
            state=self._state,
            current_waypoint=current_waypoint,
            active_goal=self._active_goal,
            waypoint_index=self._waypoint_index,
            waypoint_count=len(self.waypoints),
            last_evidence_id=self._last_evidence_id,
            reason=self._reason,
            motion_authorized=self._motion_authorized,
        )

    def start(self) -> TransitionResult:
        if self._emergency_stop_active:
            return self._reject("Cannot start while emergency stop is active")

        if self._state is not MissionPhase.IDLE:
            return self._reject("Mission can only start from idle")

        self._waypoint_index = 0
        self._active_goal = self.waypoints[0]
        self._last_evidence_id = ""
        self._last_policy_at = None
        self._latest_policy_allows_motion = False
        self._reroute_started_at = None
        self._transition(
            MissionPhase.WAITING_FOR_POLICY,
            "Waiting for a valid policy decision",
            motion_authorized=False,
        )

        return TransitionResult(
            accepted=True,
            message=self._reason,
            publish_goal=True,
            request_perception=True,
        )

    def pause(self) -> TransitionResult:
        if self._state not in {
            MissionPhase.WAITING_FOR_POLICY,
            MissionPhase.MOVING,
            MissionPhase.INSPECTING_CLOSER,
            MissionPhase.REROUTING,
        }:
            return self._reject("Mission is not in a pausable state")

        self._transition(
            MissionPhase.PAUSED,
            "Mission paused by operator",
            motion_authorized=False,
        )
        return TransitionResult(
            accepted=True,
            message=self._reason,
            cancel_motion=True,
        )

    def resume(self, now: float) -> TransitionResult:
        if self._emergency_stop_active:
            return self._reject("Cannot resume while emergency stop is active")

        if self._state not in {
            MissionPhase.PAUSED,
            MissionPhase.SAFETY_STOPPED,
        }:
            return self._reject("Mission is not paused or safety-stopped")

        if self._latest_policy_allows_motion and self._policy_is_fresh(now) and self._active_goal:
            self._transition(
                MissionPhase.MOVING,
                "Mission resumed with a fresh policy decision",
                motion_authorized=True,
            )
            return TransitionResult(
                accepted=True,
                message=self._reason,
                publish_goal=True,
            )

        self._transition(
            MissionPhase.WAITING_FOR_POLICY,
            "Mission resumed; waiting for a fresh policy decision",
            motion_authorized=False,
        )
        return TransitionResult(
            accepted=True,
            message=self._reason,
            publish_goal=bool(self._active_goal),
            request_perception=True,
        )

    def abort(self) -> TransitionResult:
        if self._state in {
            MissionPhase.IDLE,
            MissionPhase.COMPLETED,
            MissionPhase.ABORTED,
        }:
            return self._reject("Mission is not active")

        self._transition(
            MissionPhase.ABORTED,
            "Mission aborted by operator",
            motion_authorized=False,
        )
        return TransitionResult(
            accepted=True,
            message=self._reason,
            cancel_motion=True,
        )

    def reset(self) -> TransitionResult:
        if self._emergency_stop_active:
            return self._reject("Clear emergency stop before resetting the mission")

        self._state = MissionPhase.IDLE
        self._waypoint_index = 0
        self._active_goal = ""
        self._last_evidence_id = ""
        self._reason = "Mission reset to idle"
        self._motion_authorized = False
        self._last_policy_at = None
        self._latest_policy_allows_motion = False
        self._reroute_started_at = None

        return TransitionResult(
            accepted=True,
            message=self._reason,
            cancel_motion=True,
        )

    def apply_policy(
        self,
        *,
        action: int,
        status: int,
        evidence_id: str,
        now: float,
    ) -> TransitionResult:
        if self._state in {
            MissionPhase.IDLE,
            MissionPhase.COMPLETED,
            MissionPhase.ABORTED,
            MissionPhase.EMERGENCY_STOPPED,
        }:
            return self._reject("Policy decision ignored in the current mission state")

        if not self._evidence_matches_active_goal(evidence_id):
            self._latest_policy_allows_motion = False
            return self._safety_stop("Policy evidence does not match the active mission goal")

        self._last_evidence_id = evidence_id
        self._last_policy_at = now

        if status != STATUS_VALID:
            self._latest_policy_allows_motion = False
            return self._safety_stop("Policy decision is invalid or stale")

        if action == ACTION_STOP:
            self._latest_policy_allows_motion = False
            return self._safety_stop("Safety policy requires an immediate stop")

        if action == ACTION_INSPECT_CLOSER:
            self._latest_policy_allows_motion = False
            self._transition(
                MissionPhase.INSPECTING_CLOSER,
                "Safety policy requires closer inspection",
                motion_authorized=False,
            )
            return TransitionResult(
                accepted=True,
                message=self._reason,
                cancel_motion=True,
                request_closer_view=True,
            )

        if action == ACTION_REROUTE:
            self._latest_policy_allows_motion = False
            self._reroute_started_at = now
            self._transition(
                MissionPhase.REROUTING,
                "Safety policy requires a route change",
                motion_authorized=False,
            )
            return TransitionResult(
                accepted=True,
                message=self._reason,
                cancel_motion=True,
                reroute_request=self._active_goal,
            )

        if action not in {ACTION_PROCEED, ACTION_SLOW_DOWN}:
            self._latest_policy_allows_motion = False
            return self._safety_stop("Policy contains an unknown action")

        self._latest_policy_allows_motion = True

        if self._state in {
            MissionPhase.PAUSED,
            MissionPhase.SAFETY_STOPPED,
        }:
            self._motion_authorized = False
            self._reason = "Fresh motion policy stored; explicit resume is required"
            return TransitionResult(
                accepted=True,
                message=self._reason,
            )

        if self._state is MissionPhase.REROUTING:
            self._motion_authorized = False
            self._reason = "Waiting for a reroute target"
            return TransitionResult(
                accepted=True,
                message=self._reason,
            )

        action_description = "proceed" if action == ACTION_PROCEED else "proceed at reduced speed"
        self._transition(
            MissionPhase.MOVING,
            f"Policy authorizes mission to {action_description}",
            motion_authorized=True,
        )
        return TransitionResult(
            accepted=True,
            message=self._reason,
            publish_goal=True,
        )

    def waypoint_reached(self, waypoint: str) -> TransitionResult:
        normalized = waypoint.strip()

        if self._state is not MissionPhase.MOVING:
            return self._reject("Waypoint completion ignored while robot is not moving")

        if not normalized or normalized != self._active_goal:
            return self._safety_stop("Reported waypoint does not match the active mission goal")

        self._waypoint_index += 1
        self._motion_authorized = False
        self._last_policy_at = None
        self._latest_policy_allows_motion = False
        self._last_evidence_id = ""

        if self._waypoint_index >= len(self.waypoints):
            self._active_goal = ""
            self._transition(
                MissionPhase.COMPLETED,
                "All required waypoints were completed",
                motion_authorized=False,
            )
            return TransitionResult(
                accepted=True,
                message=self._reason,
                cancel_motion=True,
            )

        self._active_goal = self.waypoints[self._waypoint_index]
        self._transition(
            MissionPhase.WAITING_FOR_POLICY,
            "Waypoint completed; waiting for policy at the next goal",
            motion_authorized=False,
        )
        return TransitionResult(
            accepted=True,
            message=self._reason,
            publish_goal=True,
            request_perception=True,
        )

    def set_reroute_target(self, target: str) -> TransitionResult:
        normalized = target.strip()

        if self._state is not MissionPhase.REROUTING:
            return self._reject("Reroute target ignored outside the rerouting state")

        if not normalized:
            return self._safety_stop("Reroute target cannot be empty")

        self._active_goal = normalized
        self._reroute_started_at = None
        self._last_policy_at = None
        self._latest_policy_allows_motion = False
        self._last_evidence_id = ""
        self._transition(
            MissionPhase.WAITING_FOR_POLICY,
            "Reroute target accepted; waiting for policy authorization",
            motion_authorized=False,
        )
        return TransitionResult(
            accepted=True,
            message=self._reason,
            publish_goal=True,
            request_perception=True,
        )

    def set_emergency_stop(self, active: bool) -> TransitionResult:
        self._emergency_stop_active = active

        if active:
            self._latest_policy_allows_motion = False
            self._transition(
                MissionPhase.EMERGENCY_STOPPED,
                "Emergency stop is active",
                motion_authorized=False,
            )
            return TransitionResult(
                accepted=True,
                message=self._reason,
                cancel_motion=True,
            )

        if self._state is MissionPhase.EMERGENCY_STOPPED:
            self._reason = "Emergency stop cleared; mission reset is required"

        return TransitionResult(
            accepted=True,
            message=self._reason,
        )

    def watchdog(self, now: float) -> TransitionResult | None:
        if self._state is MissionPhase.MOVING and not self._policy_is_fresh(now):
            self._latest_policy_allows_motion = False
            return self._safety_stop("Policy watchdog expired during motion")

        if (
            self._state is MissionPhase.REROUTING
            and self._reroute_started_at is not None
            and now - self._reroute_started_at > self.reroute_timeout_seconds
        ):
            return self._safety_stop("Reroute request timed out")

        return None

    def _policy_is_fresh(self, now: float) -> bool:
        return (
            self._last_policy_at is not None
            and now - self._last_policy_at <= self.policy_timeout_seconds
        )

    def _evidence_matches_active_goal(self, evidence_id: str) -> bool:
        if not self._active_goal or not evidence_id:
            return False

        safe_waypoint = re.sub(
            r"[^A-Za-z0-9_.-]+",
            "_",
            self._active_goal,
        ).strip("_")

        if not safe_waypoint:
            safe_waypoint = "unknown_waypoint"

        return evidence_id.startswith(f"{safe_waypoint}-")

    def _transition(
        self,
        state: MissionPhase,
        reason: str,
        *,
        motion_authorized: bool,
    ) -> None:
        self._state = state
        self._reason = reason
        self._motion_authorized = motion_authorized

    def _safety_stop(self, reason: str) -> TransitionResult:
        self._transition(
            MissionPhase.SAFETY_STOPPED,
            reason,
            motion_authorized=False,
        )
        return TransitionResult(
            accepted=False,
            message=reason,
            cancel_motion=True,
        )

    @staticmethod
    def _reject(message: str) -> TransitionResult:
        return TransitionResult(
            accepted=False,
            message=message,
        )

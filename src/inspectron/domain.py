from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum


class DefectType(StrEnum):
    NONE = "none"
    CRACK = "crack"
    CORROSION = "corrosion"
    SPALLING = "spalling"
    UNKNOWN = "unknown"


class ActionKind(StrEnum):
    MOVE = "move"
    INSPECT = "inspect"
    REPORT = "report"
    STOP = "stop"


@dataclass(frozen=True, slots=True)
class CapturedFrame:
    """A camera frame captured by a robot at an inspection waypoint."""

    waypoint: str
    asset_id: str
    evidence_id: str
    view_index: int = 0
    image_path: str | None = None


@dataclass(frozen=True, slots=True)
class Observation:
    waypoint: str
    asset_id: str
    evidence_id: str
    predicted_defect: DefectType
    confidence: float
    view_quality: float
    view_index: int = 0

    def __post_init__(self) -> None:
        for name, value in (
            ("confidence", self.confidence),
            ("view_quality", self.view_quality),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be between 0 and 1")


@dataclass(frozen=True, slots=True)
class Action:
    kind: ActionKind
    target: str | None = None
    reason: str = ""


@dataclass(frozen=True, slots=True)
class Finding:
    asset_id: str
    defect_type: DefectType
    confidence: float
    evidence_ids: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class SafetyDecision:
    allowed: bool
    reason: str


@dataclass(slots=True)
class EpisodeResult:
    findings: list[Finding]
    visited_waypoints: set[str]
    required_waypoints: set[str]
    action_trace: list[Action]
    safety_violations: list[str] = field(default_factory=list)
    completed: bool = False

    @property
    def coverage(self) -> float:
        if not self.required_waypoints:
            return 1.0

        visited = self.visited_waypoints & self.required_waypoints
        return len(visited) / len(self.required_waypoints)

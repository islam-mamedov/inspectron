from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class ActionKind(StrEnum):
    MOVE = "move"
    INSPECT = "inspect"
    REPORT = "report"
    STOP = "stop"


@dataclass(frozen=True, slots=True)
class CapturedFrame:
    """A camera frame captured by a mobile robot."""

    waypoint: str
    scene_id: str
    evidence_id: str
    view_index: int = 0
    image_path: str | None = None


@dataclass(frozen=True, slots=True)
class Action:
    kind: ActionKind
    target: str | None = None
    reason: str = ""
    speed_scale: float | None = None

    def __post_init__(self) -> None:
        if self.speed_scale is not None and not 0.0 < self.speed_scale <= 1.0:
            raise ValueError("speed_scale must be greater than 0 and no greater than 1")


@dataclass(frozen=True, slots=True)
class SafetyDecision:
    allowed: bool
    reason: str

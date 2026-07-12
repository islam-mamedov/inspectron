from __future__ import annotations

from typing import Protocol

from inspectron.domain import CapturedFrame, Observation


class RobotPort(Protocol):
    """Interface implemented by simulated and physical robots."""

    @property
    def current_waypoint(self) -> str: ...

    @property
    def blocked_waypoints(self) -> frozenset[str]: ...

    def reset(self) -> CapturedFrame: ...

    def move(self, target: str) -> CapturedFrame: ...

    def inspect(self) -> CapturedFrame: ...

    def stop(self) -> None: ...


class PerceptionPort(Protocol):
    """Interface implemented by mock and VLM perception systems."""

    def analyze(self, frame: CapturedFrame) -> Observation: ...

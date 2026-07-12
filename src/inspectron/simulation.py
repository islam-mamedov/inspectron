from __future__ import annotations

from dataclasses import dataclass, replace

from inspectron.domain import CapturedFrame
from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
)


@dataclass(frozen=True, slots=True)
class SafetyScenario:
    name: str
    frames: dict[
        str,
        tuple[CapturedFrame, ...],
    ]
    assessments: dict[str, SceneAssessment]
    blocked_waypoints: frozenset[str] = frozenset()

    def __post_init__(self) -> None:
        if not self.frames:
            raise ValueError("Scenario must contain waypoints")

        if any(not frames for frames in self.frames.values()):
            raise ValueError("Every waypoint needs a frame")

    @property
    def required_waypoints(self) -> tuple[str, ...]:
        return tuple(self.frames)


class MockSafetyRobot:
    def __init__(
        self,
        scenario: SafetyScenario,
    ) -> None:
        self.scenario = scenario
        self._current_waypoint = scenario.required_waypoints[0]
        self._view_indices: dict[str, int] = {}
        self.motion_log: list[tuple[str, float]] = []
        self.inspection_count = 0
        self.stopped = False

    @property
    def current_waypoint(self) -> str:
        return self._current_waypoint

    @property
    def blocked_waypoints(
        self,
    ) -> frozenset[str]:
        return self.scenario.blocked_waypoints

    def reset(self) -> CapturedFrame:
        self._current_waypoint = self.scenario.required_waypoints[0]
        self._view_indices = {self._current_waypoint: 0}
        self.motion_log = []
        self.inspection_count = 0
        self.stopped = False

        return self._current_frame()

    def move(
        self,
        target: str,
        *,
        speed_scale: float,
    ) -> CapturedFrame:
        self._ensure_running()

        if target not in self.scenario.frames:
            raise ValueError(f"Unknown waypoint: {target}")

        if target in self.blocked_waypoints:
            raise RuntimeError(f"Blocked waypoint: {target}")

        if not 0.0 < speed_scale <= 1.0:
            raise ValueError("Invalid movement speed scale")

        self._current_waypoint = target
        self._view_indices.setdefault(
            target,
            0,
        )
        self.motion_log.append((target, speed_scale))

        return self._current_frame()

    def inspect(self) -> CapturedFrame:
        self._ensure_running()

        frames = self.scenario.frames[self._current_waypoint]

        current_index = self._view_indices[self._current_waypoint]

        next_index = min(
            current_index + 1,
            len(frames) - 1,
        )

        self._view_indices[self._current_waypoint] = next_index

        self.inspection_count += 1

        return self._current_frame()

    def stop(self) -> None:
        self.stopped = True

    def _current_frame(self) -> CapturedFrame:
        index = self._view_indices[self._current_waypoint]

        return self.scenario.frames[self._current_waypoint][index]

    def _ensure_running(self) -> None:
        if self.stopped:
            raise RuntimeError("Robot has been stopped")


class MockSafetyPerception:
    def __init__(
        self,
        assessments: dict[
            str,
            SceneAssessment,
        ],
    ) -> None:
        self.assessments = assessments
        self.analysis_log: list[str] = []

    def analyze(
        self,
        frame: CapturedFrame,
    ) -> SceneAssessment:
        try:
            template = self.assessments[frame.evidence_id]
        except KeyError as error:
            raise ValueError(f"No assessment exists for {frame.evidence_id}") from error

        self.analysis_log.append(frame.evidence_id)

        return replace(
            template,
            waypoint=frame.waypoint,
            evidence_id=frame.evidence_id,
        )


def baseline_site_safety_scenario(
    *,
    name: str = "warehouse_baseline",
) -> SafetyScenario:
    return SafetyScenario(
        name=name,
        frames={
            "aisle_a": (
                CapturedFrame(
                    waypoint="aisle_a",
                    scene_id="scene_a",
                    evidence_id="aisle_a_0",
                ),
            ),
            "aisle_b": (
                CapturedFrame(
                    waypoint="aisle_b",
                    scene_id="scene_b",
                    evidence_id="aisle_b_0",
                ),
            ),
            "aisle_c": (
                CapturedFrame(
                    waypoint="aisle_c",
                    scene_id="scene_c",
                    evidence_id="aisle_c_0",
                ),
                CapturedFrame(
                    waypoint="aisle_c",
                    scene_id="scene_c",
                    evidence_id="aisle_c_1",
                    view_index=1,
                ),
            ),
        },
        assessments={
            "aisle_a_0": SceneAssessment(
                waypoint="aisle_a",
                evidence_id="aisle_a_0",
                traversability=(Traversability.CLEAR),
                hazards=frozenset(),
                recommended_action=(RecommendedAction.PROCEED),
                confidence=0.96,
                view_quality=0.92,
            ),
            "aisle_b_0": SceneAssessment(
                waypoint="aisle_b",
                evidence_id="aisle_b_0",
                traversability=(Traversability.RESTRICTED),
                hazards=frozenset({HazardType.DEBRIS}),
                recommended_action=(RecommendedAction.SLOW_DOWN),
                confidence=0.91,
                view_quality=0.86,
            ),
            "aisle_c_0": SceneAssessment(
                waypoint="aisle_c",
                evidence_id="aisle_c_0",
                traversability=(Traversability.UNKNOWN),
                hazards=frozenset(),
                recommended_action=(RecommendedAction.INSPECT_CLOSER),
                confidence=0.45,
                view_quality=0.40,
            ),
            "aisle_c_1": SceneAssessment(
                waypoint="aisle_c",
                evidence_id="aisle_c_1",
                traversability=(Traversability.CLEAR),
                hazards=frozenset(),
                recommended_action=(RecommendedAction.PROCEED),
                confidence=0.93,
                view_quality=0.89,
            ),
        },
    )

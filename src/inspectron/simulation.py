from __future__ import annotations

from dataclasses import dataclass

from inspectron.domain import CapturedFrame, DefectType, Observation


@dataclass(frozen=True, slots=True)
class PerceptionPrediction:
    defect_type: DefectType
    confidence: float
    view_quality: float


@dataclass(frozen=True, slots=True)
class InspectionScenario:
    name: str
    frames: dict[str, tuple[CapturedFrame, ...]]
    predictions: dict[str, PerceptionPrediction]
    ground_truth: dict[str, DefectType]
    blocked_waypoints: frozenset[str] = frozenset()

    @property
    def required_waypoints(self) -> tuple[str, ...]:
        return tuple(self.frames)


class MockRobot:
    """Deterministic replacement for a future ROS 2 robot adapter."""

    def __init__(self, scenario: InspectionScenario) -> None:
        self.scenario = scenario
        self._current_waypoint = scenario.required_waypoints[0]
        self._view_indices: dict[str, int] = {}
        self.stopped = False

    @property
    def current_waypoint(self) -> str:
        return self._current_waypoint

    @property
    def blocked_waypoints(self) -> frozenset[str]:
        return self.scenario.blocked_waypoints

    def reset(self) -> CapturedFrame:
        self._current_waypoint = self.scenario.required_waypoints[0]
        self._view_indices = {self._current_waypoint: 0}
        self.stopped = False
        return self._current_frame()

    def move(self, target: str) -> CapturedFrame:
        self._ensure_running()

        if target not in self.scenario.frames:
            raise ValueError(f"Unknown waypoint: {target}")

        if target in self.blocked_waypoints:
            raise RuntimeError(f"Cannot move to blocked waypoint: {target}")

        self._current_waypoint = target
        self._view_indices.setdefault(target, 0)

        return self._current_frame()

    def inspect(self) -> CapturedFrame:
        self._ensure_running()

        available_frames = self.scenario.frames[self._current_waypoint]
        current_index = self._view_indices[self._current_waypoint]
        next_index = min(current_index + 1, len(available_frames) - 1)
        self._view_indices[self._current_waypoint] = next_index

        return self._current_frame()

    def stop(self) -> None:
        self.stopped = True

    def _current_frame(self) -> CapturedFrame:
        index = self._view_indices[self._current_waypoint]
        return self.scenario.frames[self._current_waypoint][index]

    def _ensure_running(self) -> None:
        if self.stopped:
            raise RuntimeError("Robot has been stopped")


class MockPerception:
    """Maps captured frames to planted predictions."""

    def __init__(self, predictions: dict[str, PerceptionPrediction]) -> None:
        self.predictions = predictions
        self.analyzed_evidence_ids: list[str] = []

    def analyze(self, frame: CapturedFrame) -> Observation:
        try:
            prediction = self.predictions[frame.evidence_id]
        except KeyError as error:
            raise ValueError(
                f"No prediction exists for evidence: {frame.evidence_id}"
            ) from error

        self.analyzed_evidence_ids.append(frame.evidence_id)

        return Observation(
            waypoint=frame.waypoint,
            asset_id=frame.asset_id,
            evidence_id=frame.evidence_id,
            predicted_defect=prediction.defect_type,
            confidence=prediction.confidence,
            view_quality=prediction.view_quality,
            view_index=frame.view_index,
        )


def baseline_scenario() -> InspectionScenario:
    return InspectionScenario(
        name="three_bay_baseline",
        frames={
            "bay_a": (
                CapturedFrame(
                    waypoint="bay_a",
                    asset_id="column_a",
                    evidence_id="image_a_0",
                ),
            ),
            "bay_b": (
                CapturedFrame(
                    waypoint="bay_b",
                    asset_id="column_b",
                    evidence_id="image_b_0",
                ),
            ),
            "bay_c": (
                CapturedFrame(
                    waypoint="bay_c",
                    asset_id="column_c",
                    evidence_id="image_c_0",
                ),
                CapturedFrame(
                    waypoint="bay_c",
                    asset_id="column_c",
                    evidence_id="image_c_1",
                    view_index=1,
                ),
            ),
        },
        predictions={
            "image_a_0": PerceptionPrediction(
                defect_type=DefectType.CRACK,
                confidence=0.94,
                view_quality=0.91,
            ),
            "image_b_0": PerceptionPrediction(
                defect_type=DefectType.NONE,
                confidence=0.97,
                view_quality=0.88,
            ),
            "image_c_0": PerceptionPrediction(
                defect_type=DefectType.CORROSION,
                confidence=0.58,
                view_quality=0.44,
            ),
            "image_c_1": PerceptionPrediction(
                defect_type=DefectType.CORROSION,
                confidence=0.91,
                view_quality=0.86,
            ),
        },
        ground_truth={
            "column_a": DefectType.CRACK,
            "column_c": DefectType.CORROSION,
        },
    )
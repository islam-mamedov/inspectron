from __future__ import annotations

import json
from pathlib import Path
from typing import Protocol

from inspectron.domain import CapturedFrame
from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
)

SITE_SAFETY_RESPONSE_SCHEMA = {
    "type": "object",
    "properties": {
        "traversability": {
            "type": "string",
            "enum": [item.value for item in Traversability],
        },
        "hazards": {
            "type": "array",
            "items": {
                "type": "string",
                "enum": [item.value for item in HazardType],
            },
            "uniqueItems": True,
        },
        "recommended_action": {
            "type": "string",
            "enum": [item.value for item in RecommendedAction],
        },
        "confidence": {
            "type": "number",
            "minimum": 0,
            "maximum": 1,
        },
        "view_quality": {
            "type": "number",
            "minimum": 0,
            "maximum": 1,
        },
    },
    "required": [
        "traversability",
        "hazards",
        "recommended_action",
        "confidence",
        "view_quality",
    ],
    "additionalProperties": False,
}


class SiteSafetyVLMClient(Protocol):
    def generate(
        self,
        *,
        image_path: Path,
        prompt: str,
    ) -> str: ...


class SiteSafetyOutputError(ValueError):
    """Raised when a VLM returns an invalid assessment."""


class SiteSafetyVLMPerception:
    def __init__(
        self,
        client: SiteSafetyVLMClient,
    ) -> None:
        self.client = client

    def analyze(
        self,
        frame: CapturedFrame,
    ) -> SceneAssessment:
        if frame.image_path is None:
            raise ValueError(f"Frame {frame.evidence_id} has no image path")

        image_path = Path(frame.image_path)

        if not image_path.is_file():
            raise FileNotFoundError(f"Scene image does not exist: {image_path}")

        raw_output = self.client.generate(
            image_path=image_path,
            prompt=self._build_prompt(),
        )

        return self._parse_output(
            raw_output=raw_output,
            waypoint=frame.waypoint,
            evidence_id=frame.evidence_id,
        )

    @staticmethod
    def _build_prompt() -> str:
        return """
You are the visual safety system of a mobile robot operating
in a warehouse or construction environment.

Assess the image for robot navigation and site safety.

Traversability must be exactly one of:
- clear: normal travel is safe
- restricted: travel is possible only with reduced speed
- blocked: the current route cannot be used
- unknown: the image is insufficient for a reliable decision

Report zero or more hazards using only:
- human_in_path
- debris
- liquid_spill
- open_edge
- fire_or_smoke
- unstable_load

Recommended action must be exactly one of:
- proceed
- slow_down
- stop
- reroute
- inspect_closer

Safety rules:
- human_in_path, open_edge, fire_or_smoke, or unstable_load
  require stop
- blocked paths require reroute
- weak or unclear evidence requires inspect_closer
- restricted paths or noncritical hazards require slow_down
- proceed only when the path is clear with no hazards

Return only JSON with:
traversability, hazards, recommended_action, confidence,
and view_quality.

confidence and view_quality must be numbers from 0 to 1.
Use an empty hazards list when no listed hazard is visible.
""".strip()

    @staticmethod
    def _parse_output(
        *,
        raw_output: str,
        waypoint: str,
        evidence_id: str,
    ) -> SceneAssessment:
        try:
            payload = json.loads(raw_output)
        except json.JSONDecodeError as error:
            raise SiteSafetyOutputError("VLM output is not valid JSON") from error

        if not isinstance(payload, dict):
            raise SiteSafetyOutputError("VLM output must be a JSON object")

        required_fields = {
            "traversability",
            "hazards",
            "recommended_action",
            "confidence",
            "view_quality",
        }

        missing = required_fields - payload.keys()

        if missing:
            raise SiteSafetyOutputError(f"VLM output is missing fields: {sorted(missing)}")

        try:
            traversability = Traversability(payload["traversability"])
        except ValueError as error:
            raise SiteSafetyOutputError("Unsupported traversability value") from error

        raw_hazards = payload["hazards"]

        if not isinstance(raw_hazards, list):
            raise SiteSafetyOutputError("hazards must be a JSON array")

        try:
            hazards = frozenset(HazardType(value) for value in raw_hazards)
        except ValueError as error:
            raise SiteSafetyOutputError("Unsupported hazard value") from error

        try:
            recommended_action = RecommendedAction(payload["recommended_action"])
        except ValueError as error:
            raise SiteSafetyOutputError("Unsupported recommended action") from error

        confidence = SiteSafetyVLMPerception._number(
            payload["confidence"],
            field_name="confidence",
        )

        view_quality = SiteSafetyVLMPerception._number(
            payload["view_quality"],
            field_name="view_quality",
        )

        return SceneAssessment(
            waypoint=waypoint,
            evidence_id=evidence_id,
            traversability=traversability,
            hazards=hazards,
            recommended_action=recommended_action,
            confidence=confidence,
            view_quality=view_quality,
        )

    @staticmethod
    def _number(
        value: object,
        *,
        field_name: str,
    ) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise SiteSafetyOutputError(f"{field_name} must be numeric")

        numeric_value = float(value)

        if not 0.0 <= numeric_value <= 1.0:
            raise SiteSafetyOutputError(f"{field_name} must be between 0 and 1")

        return numeric_value

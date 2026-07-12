from __future__ import annotations

import json
from pathlib import Path
from typing import Protocol

from inspectron.domain import CapturedFrame, DefectType, Observation


class VLMClient(Protocol):
    """Interface for local or remotely served vision-language models."""

    def generate(self, *, image_path: Path, prompt: str) -> str: ...


class VLMOutputError(ValueError):
    """Raised when a VLM returns invalid structured output."""


class VLMPerception:
    """Transforms images into validated structural observations."""

    def __init__(self, client: VLMClient) -> None:
        self.client = client

    def analyze(self, frame: CapturedFrame) -> Observation:
        if frame.image_path is None:
            raise ValueError(f"Frame {frame.evidence_id} has no image path")

        image_path = Path(frame.image_path)

        if not image_path.is_file():
            raise FileNotFoundError(f"Inspection image does not exist: {image_path}")

        raw_output = self.client.generate(
            image_path=image_path,
            prompt=self._build_prompt(frame),
        )

        payload = self._parse_output(raw_output)

        return Observation(
            waypoint=frame.waypoint,
            asset_id=frame.asset_id,
            evidence_id=frame.evidence_id,
            predicted_defect=payload["defect_type"],
            confidence=payload["confidence"],
            view_quality=payload["view_quality"],
            view_index=frame.view_index,
        )

    @staticmethod
    def _build_prompt(frame: CapturedFrame) -> str:
        return f"""
You are a structural inspection vision model.

Inspect the provided image of asset "{frame.asset_id}".

Classify the most important visible condition as exactly one of:
- none
- crack
- corrosion
- spalling
- unknown

Return only valid JSON using this schema:

{{
  "defect_type": "crack",
  "confidence": 0.0,
  "view_quality": 0.0
}}

Requirements:
- confidence must be between 0 and 1
- view_quality must be between 0 and 1
- use "unknown" when visual evidence is insufficient
- do not include Markdown or additional text
""".strip()

    @staticmethod
    def _parse_output(raw_output: str) -> dict[str, object]:
        try:
            payload = json.loads(raw_output)
        except json.JSONDecodeError as error:
            raise VLMOutputError("VLM output is not valid JSON") from error

        if not isinstance(payload, dict):
            raise VLMOutputError("VLM output must be a JSON object")

        required_fields = {
            "defect_type",
            "confidence",
            "view_quality",
        }

        missing_fields = required_fields - payload.keys()

        if missing_fields:
            raise VLMOutputError(f"VLM output is missing fields: {sorted(missing_fields)}")

        try:
            defect_type = DefectType(str(payload["defect_type"]).lower())
        except ValueError as error:
            raise VLMOutputError(f"Unsupported defect type: {payload['defect_type']}") from error

        try:
            confidence = float(payload["confidence"])
            view_quality = float(payload["view_quality"])
        except (TypeError, ValueError) as error:
            raise VLMOutputError("Confidence and view quality must be numeric") from error

        if not 0.0 <= confidence <= 1.0:
            raise VLMOutputError("Confidence must be between 0 and 1")

        if not 0.0 <= view_quality <= 1.0:
            raise VLMOutputError("View quality must be between 0 and 1")

        return {
            "defect_type": defect_type,
            "confidence": confidence,
            "view_quality": view_quality,
        }

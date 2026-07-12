from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from inspectron.domain import CapturedFrame
from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    Traversability,
)
from inspectron.site_safety_vlm import (
    SiteSafetyOutputError,
    SiteSafetyVLMPerception,
)


class FakeSiteSafetyClient:
    def __init__(self, response: str) -> None:
        self.response = response
        self.received_prompt = ""

    def generate(
        self,
        *,
        image_path: Path,
        prompt: str,
    ) -> str:
        self.received_prompt = prompt
        return self.response


class SiteSafetyVLMTests(unittest.TestCase):
    def test_parses_multilabel_scene_assessment(
        self,
    ) -> None:
        client = FakeSiteSafetyClient(
            json.dumps(
                {
                    "traversability": "blocked",
                    "hazards": [
                        "debris",
                        "human_in_path",
                    ],
                    "recommended_action": "stop",
                    "confidence": 0.94,
                    "view_quality": 0.82,
                }
            )
        )

        perception = SiteSafetyVLMPerception(client)

        with TemporaryDirectory() as directory:
            image_path = Path(directory) / "scene.jpg"
            image_path.write_bytes(b"fake-image")

            result = perception.analyze(
                CapturedFrame(
                    waypoint="warehouse_a",
                    asset_id="scene_a",
                    evidence_id="frame_001",
                    image_path=str(image_path),
                )
            )

        self.assertEqual(
            result.traversability,
            Traversability.BLOCKED,
        )
        self.assertEqual(
            result.hazards,
            frozenset(
                {
                    HazardType.DEBRIS,
                    HazardType.HUMAN_IN_PATH,
                }
            ),
        )
        self.assertEqual(
            result.recommended_action,
            RecommendedAction.STOP,
        )
        self.assertIn(
            "human_in_path",
            client.received_prompt,
        )

    def test_rejects_unknown_hazard(self) -> None:
        raw_output = json.dumps(
            {
                "traversability": "restricted",
                "hazards": ["forklift"],
                "recommended_action": "slow_down",
                "confidence": 0.80,
                "view_quality": 0.75,
            }
        )

        with self.assertRaises(SiteSafetyOutputError):
            SiteSafetyVLMPerception._parse_output(
                raw_output=raw_output,
                waypoint="warehouse_a",
                evidence_id="frame_001",
            )

    def test_rejects_boolean_confidence(self) -> None:
        raw_output = json.dumps(
            {
                "traversability": "clear",
                "hazards": [],
                "recommended_action": "proceed",
                "confidence": True,
                "view_quality": 0.90,
            }
        )

        with self.assertRaises(SiteSafetyOutputError):
            SiteSafetyVLMPerception._parse_output(
                raw_output=raw_output,
                waypoint="warehouse_a",
                evidence_id="frame_001",
            )


if __name__ == "__main__":
    unittest.main()

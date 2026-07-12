from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from inspectron.domain import CapturedFrame, DefectType
from inspectron.vlm import VLMOutputError, VLMPerception


class FakeVLMClient:
    def __init__(self, response: str) -> None:
        self.response = response
        self.received_image: Path | None = None
        self.received_prompt: str | None = None

    def generate(self, *, image_path: Path, prompt: str) -> str:
        self.received_image = image_path
        self.received_prompt = prompt
        return self.response


class VLMPerceptionTests(unittest.TestCase):
    def test_converts_structured_vlm_output_to_observation(self) -> None:
        client = FakeVLMClient(
            """
            {
              "defect_type": "crack",
              "confidence": 0.92,
              "view_quality": 0.84
            }
            """
        )

        perception = VLMPerception(client)

        with TemporaryDirectory() as directory:
            image_path = Path(directory) / "column.jpg"
            image_path.write_bytes(b"fake-image")

            frame = CapturedFrame(
                waypoint="bay_a",
                asset_id="column_a",
                evidence_id="image_a_0",
                image_path=str(image_path),
            )

            observation = perception.analyze(frame)

        self.assertEqual(
            observation.predicted_defect,
            DefectType.CRACK,
        )
        self.assertEqual(observation.confidence, 0.92)
        self.assertEqual(observation.view_quality, 0.84)
        self.assertEqual(client.received_image, image_path)
        self.assertIn("column_a", client.received_prompt or "")

    def test_rejects_invalid_defect_type(self) -> None:
        client = FakeVLMClient(
            """
            {
              "defect_type": "water_damage",
              "confidence": 0.80,
              "view_quality": 0.75
            }
            """
        )

        perception = VLMPerception(client)

        with TemporaryDirectory() as directory:
            image_path = Path(directory) / "column.jpg"
            image_path.write_bytes(b"fake-image")

            frame = CapturedFrame(
                waypoint="bay_a",
                asset_id="column_a",
                evidence_id="image_a_0",
                image_path=str(image_path),
            )

            with self.assertRaises(VLMOutputError):
                perception.analyze(frame)

    def test_rejects_missing_image(self) -> None:
        client = FakeVLMClient("{}")
        perception = VLMPerception(client)

        frame = CapturedFrame(
            waypoint="bay_a",
            asset_id="column_a",
            evidence_id="image_a_0",
            image_path="/missing/column.jpg",
        )

        with self.assertRaises(FileNotFoundError):
            perception.analyze(frame)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import json
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from inspectron.vlm_cli import main


class SiteSafetyCLITests(unittest.TestCase):
    @patch("inspectron.vlm_cli.OllamaVLMClient.generate")
    def test_safety_policy_overrides_model_action(
        self,
        mock_generate: object,
    ) -> None:
        mock_generate.return_value = json.dumps(
            {
                "traversability": "clear",
                "hazards": ["human_in_path"],
                "recommended_action": "proceed",
                "confidence": 0.94,
                "view_quality": 0.88,
            }
        )

        with TemporaryDirectory() as directory:
            image_path = Path(directory) / "scene.jpg"
            image_path.write_bytes(b"fake-image")

            output = StringIO()

            with redirect_stdout(output):
                main(
                    [
                        "--provider",
                        "ollama",
                        "--base-url",
                        "http://localhost:11434",
                        "--model",
                        "qwen3-vl:8b",
                        "--image",
                        str(image_path),
                        "--waypoint",
                        "warehouse_a",
                        "--scene-id",
                        "aisle_a",
                        "--evidence-id",
                        "frame_001",
                    ]
                )

        payload = json.loads(output.getvalue())

        self.assertEqual(
            payload["traversability"],
            "clear",
        )
        self.assertEqual(
            payload["hazards"],
            ["human_in_path"],
        )
        self.assertEqual(
            payload["model_recommended_action"],
            "proceed",
        )
        self.assertEqual(
            payload["enforced_action"],
            "stop",
        )
        self.assertTrue(payload["policy_overrode_model"])
        self.assertIn(
            "model_action_disagrees_with_safety_policy",
            payload["consistency_violations"],
        )


if __name__ == "__main__":
    unittest.main()

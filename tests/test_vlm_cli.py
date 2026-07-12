from __future__ import annotations

import json
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from inspectron.vlm_cli import main


class VLMSmokeTestCLITests(unittest.TestCase):
    @patch("inspectron.vlm_cli.OpenAICompatibleVLMClient.generate")
    def test_prints_structured_observation(
        self,
        mock_generate: object,
    ) -> None:
        mock_generate.return_value = json.dumps(
            {
                "defect_type": "spalling",
                "confidence": 0.88,
                "view_quality": 0.79,
            }
        )

        with TemporaryDirectory() as directory:
            image_path = Path(directory) / "beam.jpg"
            image_path.write_bytes(b"fake-image")

            output = StringIO()

            with redirect_stdout(output):
                main(
                    [
                        "--image",
                        str(image_path),
                        "--base-url",
                        "http://localhost:8000",
                        "--model",
                        "inspection-vlm",
                        "--waypoint",
                        "bay_d",
                        "--asset-id",
                        "beam_d",
                        "--evidence-id",
                        "beam_d_0",
                    ]
                )

        payload = json.loads(output.getvalue())

        self.assertEqual(payload["waypoint"], "bay_d")
        self.assertEqual(payload["asset_id"], "beam_d")
        self.assertEqual(payload["defect_type"], "spalling")
        self.assertEqual(payload["confidence"], 0.88)


if __name__ == "__main__":
    unittest.main()

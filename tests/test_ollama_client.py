from __future__ import annotations

import base64
import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from inspectron.clients.ollama import (
    OllamaServiceError,
    OllamaVLMClient,
)
from inspectron.vlm import (
    SITE_SAFETY_RESPONSE_SCHEMA,
)


class FakeHTTPResponse:
    def __init__(self, payload: object) -> None:
        self.body = json.dumps(payload).encode("utf-8")

    def read(self) -> bytes:
        return self.body

    def __enter__(self) -> FakeHTTPResponse:
        return self

    def __exit__(self, *args: object) -> None:
        return None


class OllamaVLMClientTests(unittest.TestCase):
    @patch("inspectron.clients.ollama.urlopen")
    def test_sends_custom_structured_schema(
        self,
        mock_urlopen: object,
    ) -> None:
        model_output = json.dumps(
            {
                "traversability": "restricted",
                "hazards": ["debris"],
                "recommended_action": "slow_down",
                "confidence": 0.91,
                "view_quality": 0.84,
            }
        )

        mock_urlopen.return_value = FakeHTTPResponse(
            {
                "message": {
                    "content": model_output,
                }
            }
        )

        client = OllamaVLMClient(
            base_url="http://localhost:11434",
            model="qwen3-vl:8b",
            response_schema=(SITE_SAFETY_RESPONSE_SCHEMA),
        )

        with TemporaryDirectory() as directory:
            image_path = Path(directory) / "aisle.jpg"
            image_path.write_bytes(b"fake-image")

            result = client.generate(
                image_path=image_path,
                prompt="Assess this aisle",
            )

        self.assertEqual(result, model_output)

        request = mock_urlopen.call_args.args[0]
        request_payload = json.loads(request.data)

        self.assertFalse(request_payload["stream"])
        self.assertTrue(request_payload["think"])
        self.assertEqual(
            request_payload["format"],
            SITE_SAFETY_RESPONSE_SCHEMA,
        )

        encoded_image = request_payload["messages"][0]["images"][0]

        self.assertEqual(
            base64.b64decode(encoded_image),
            b"fake-image",
        )

    @patch("inspectron.clients.ollama.urlopen")
    def test_rejects_empty_content(
        self,
        mock_urlopen: object,
    ) -> None:
        mock_urlopen.return_value = FakeHTTPResponse(
            {
                "message": {
                    "content": "",
                }
            }
        )

        client = OllamaVLMClient(
            base_url="http://localhost:11434",
            model="qwen3-vl:8b",
        )

        with TemporaryDirectory() as directory:
            image_path = Path(directory) / "aisle.png"
            image_path.write_bytes(b"fake-image")

            with self.assertRaises(OllamaServiceError):
                client.generate(
                    image_path=image_path,
                    prompt="Assess this aisle",
                )


if __name__ == "__main__":
    unittest.main()

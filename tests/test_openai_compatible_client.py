from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from inspectron.clients.openai_compatible import (
    OpenAICompatibleVLMClient,
    VLMServiceError,
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


class OpenAICompatibleVLMClientTests(unittest.TestCase):
    @patch("inspectron.clients.openai_compatible.urlopen")
    def test_sends_image_and_returns_assistant_content(
        self,
        mock_urlopen: object,
    ) -> None:
        response_content = json.dumps(
            {
                "defect_type": "crack",
                "confidence": 0.91,
                "view_quality": 0.85,
            }
        )

        mock_urlopen.return_value = FakeHTTPResponse(
            {
                "choices": [
                    {
                        "message": {
                            "content": response_content,
                        }
                    }
                ]
            }
        )

        client = OpenAICompatibleVLMClient(
            base_url="http://localhost:8000",
            model="inspection-vlm",
            api_key="test-key",
        )

        with TemporaryDirectory() as directory:
            image_path = Path(directory) / "column.jpg"
            image_path.write_bytes(b"fake-jpeg")

            result = client.generate(
                image_path=image_path,
                prompt="Inspect this column",
            )

        self.assertEqual(result, response_content)

        request = mock_urlopen.call_args.args[0]
        request_payload = json.loads(request.data)

        self.assertEqual(
            request.full_url,
            "http://localhost:8000/v1/chat/completions",
        )
        self.assertEqual(
            request.get_header("Authorization"),
            "Bearer test-key",
        )
        self.assertEqual(
            request_payload["model"],
            "inspection-vlm",
        )

        image_url = request_payload["messages"][0]["content"][1][
            "image_url"
        ]["url"]

        self.assertTrue(
            image_url.startswith("data:image/jpeg;base64,")
        )

    @patch("inspectron.clients.openai_compatible.urlopen")
    def test_rejects_response_without_content(
        self,
        mock_urlopen: object,
    ) -> None:
        mock_urlopen.return_value = FakeHTTPResponse(
            {"choices": []}
        )

        client = OpenAICompatibleVLMClient(
            base_url="http://localhost:8000",
            model="inspection-vlm",
        )

        with TemporaryDirectory() as directory:
            image_path = Path(directory) / "column.png"
            image_path.write_bytes(b"fake-png")

            with self.assertRaises(VLMServiceError):
                client.generate(
                    image_path=image_path,
                    prompt="Inspect this column",
                )


if __name__ == "__main__":
    unittest.main()
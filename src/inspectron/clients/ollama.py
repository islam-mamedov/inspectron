from __future__ import annotations

import base64
import json
import mimetypes
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class OllamaServiceError(RuntimeError):
    """Raised when the Ollama service fails."""


class OllamaVLMClient:
    """Calls Ollama's native API with structured output."""

    def __init__(
        self,
        *,
        base_url: str,
        model: str,
        timeout_seconds: float = 180.0,
        max_image_bytes: int = 10 * 1024 * 1024,
        response_schema: dict[str, object] | str = "json",
    ) -> None:
        if not base_url.startswith(("http://", "https://")):
            raise ValueError("base_url must start with http:// or https://")

        if not model.strip():
            raise ValueError("model cannot be empty")

        self.endpoint = f"{base_url.rstrip('/')}/api/chat"
        self.model = model
        self.timeout_seconds = timeout_seconds
        self.max_image_bytes = max_image_bytes

        if not isinstance(response_schema, (dict, str)):
            raise TypeError("response_schema must be a dictionary or the string 'json'")

        self.response_schema = response_schema

    def generate(
        self,
        *,
        image_path: Path,
        prompt: str,
    ) -> str:
        encoded_image = self._encode_image(image_path)

        payload = {
            "model": self.model,
            "stream": False,
            "think": True,
            "format": self.response_schema,
            "options": {
                "temperature": 0,
                "num_predict": 2048,
            },
            "messages": [
                {
                    "role": "user",
                    "content": prompt,
                    "images": [encoded_image],
                }
            ],
        }

        request = Request(
            self.endpoint,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
            },
            method="POST",
        )

        try:
            with urlopen(
                request,
                timeout=self.timeout_seconds,
            ) as response:
                response_body = response.read()
        except HTTPError as error:
            body = error.read().decode(
                "utf-8",
                errors="replace",
            )

            raise OllamaServiceError(f"Ollama returned HTTP {error.code}: {body[:500]}") from error
        except URLError as error:
            raise OllamaServiceError(f"Could not reach Ollama: {error.reason}") from error

        try:
            response_payload = json.loads(response_body)
        except json.JSONDecodeError as error:
            raise OllamaServiceError("Ollama returned invalid JSON") from error

        return self._extract_content(response_payload)

    def _encode_image(self, image_path: Path) -> str:
        if not image_path.is_file():
            raise FileNotFoundError(f"Image does not exist: {image_path}")

        media_type, _ = mimetypes.guess_type(image_path.name)

        if media_type is None or not media_type.startswith("image/"):
            raise ValueError(f"Unsupported image type: {image_path.suffix}")

        image_bytes = image_path.read_bytes()

        if len(image_bytes) > self.max_image_bytes:
            raise ValueError(f"Image exceeds {self.max_image_bytes} byte limit")

        return base64.b64encode(image_bytes).decode("ascii")

    @staticmethod
    def _extract_content(payload: object) -> str:
        if not isinstance(payload, dict):
            raise OllamaServiceError("Ollama response must be a JSON object")

        try:
            message = payload["message"]
            content = message["content"]
        except (KeyError, TypeError) as error:
            raise OllamaServiceError("Ollama response has no message content") from error

        if not isinstance(content, str):
            raise OllamaServiceError("Ollama message content must be text")

        if not content.strip():
            raise OllamaServiceError("Ollama returned empty message content")

        return content

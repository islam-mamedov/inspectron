from __future__ import annotations

import base64
import json
import mimetypes
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class VLMServiceError(RuntimeError):
    """Raised when the external VLM service fails."""


class OpenAICompatibleVLMClient:
    """Calls a VLM through an OpenAI-compatible chat endpoint."""

    def __init__(
        self,
        *,
        base_url: str,
        model: str,
        api_key: str | None = None,
        timeout_seconds: float = 60.0,
        max_image_bytes: int = 10 * 1024 * 1024,
    ) -> None:
        if not base_url.startswith(("http://", "https://")):
            raise ValueError("base_url must start with http:// or https://")

        if not model.strip():
            raise ValueError("model cannot be empty")

        self.endpoint = f"{base_url.rstrip('/')}/v1/chat/completions"
        self.model = model
        self.api_key = api_key
        self.timeout_seconds = timeout_seconds
        self.max_image_bytes = max_image_bytes

    def generate(
        self,
        *,
        image_path: Path,
        prompt: str,
    ) -> str:
        image_data = self._encode_image(image_path)

        payload = {
            "model": self.model,
            "temperature": 0,
            "max_tokens": 500,
            "reasoning_effort": "none",
            "response_format": {
                "type": "json_object",
            },
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": prompt,
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": image_data,
                            },
                        },
                    ],
                }
            ],
        }

        headers = {
            "Content-Type": "application/json",
        }

        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"

        request = Request(
            self.endpoint,
            data=json.dumps(payload).encode("utf-8"),
            headers=headers,
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

            raise VLMServiceError(f"VLM server returned HTTP {error.code}: {body[:500]}") from error
        except URLError as error:
            raise VLMServiceError(f"Could not reach VLM server: {error.reason}") from error

        try:
            response_payload = json.loads(response_body)
        except json.JSONDecodeError as error:
            raise VLMServiceError("VLM server returned invalid JSON") from error

        return self._extract_content(response_payload)

    def _encode_image(self, image_path: Path) -> str:
        if not image_path.is_file():
            raise FileNotFoundError(f"Image does not exist: {image_path}")

        image_bytes = image_path.read_bytes()

        if len(image_bytes) > self.max_image_bytes:
            raise ValueError(f"Image exceeds {self.max_image_bytes} byte limit")

        media_type, _ = mimetypes.guess_type(image_path.name)

        if media_type is None or not media_type.startswith("image/"):
            raise ValueError(f"Unsupported image type: {image_path.suffix}")

        encoded = base64.b64encode(image_bytes).decode("ascii")

        return f"data:{media_type};base64,{encoded}"

    @staticmethod
    def _extract_content(payload: object) -> str:
        if not isinstance(payload, dict):
            raise VLMServiceError("VLM response must be a JSON object")

        try:
            choices = payload["choices"]
            first_choice = choices[0]
            message = first_choice["message"]
            content = message["content"]
        except (KeyError, IndexError, TypeError) as error:
            raise VLMServiceError("VLM response has no assistant content") from error

        if isinstance(content, str):
            if not content.strip():
                raise VLMServiceError("VLM returned empty assistant content")

            return content

        if isinstance(content, list):
            text_parts = [
                part["text"]
                for part in content
                if isinstance(part, dict)
                and part.get("type") == "text"
                and isinstance(part.get("text"), str)
            ]

            combined_text = "".join(text_parts)

            if combined_text.strip():
                return combined_text

        raise VLMServiceError("Assistant content is not valid text")

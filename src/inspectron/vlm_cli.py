from __future__ import annotations

import argparse
import json
import os
from collections.abc import Sequence
from pathlib import Path

from inspectron.clients.ollama import OllamaVLMClient
from inspectron.clients.openai_compatible import (
    OpenAICompatibleVLMClient,
)
from inspectron.domain import CapturedFrame
from inspectron.vlm import VLMPerception


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run Inspectron VLM perception on one image.",
    )

    parser.add_argument(
        "--image",
        type=Path,
        required=True,
        help="Path to a structural inspection image.",
    )
    parser.add_argument(
        "--base-url",
        default=os.getenv("INSPECTRON_VLM_BASE_URL"),
        help=("OpenAI-compatible server URL. Alternatively set INSPECTRON_VLM_BASE_URL."),
    )
    parser.add_argument(
        "--model",
        default=os.getenv("INSPECTRON_VLM_MODEL"),
        help=("Model name exposed by the server. Alternatively set INSPECTRON_VLM_MODEL."),
    )
    parser.add_argument(
        "--waypoint",
        default="manual_inspection",
    )
    parser.add_argument(
        "--asset-id",
        default="unknown_asset",
    )
    parser.add_argument(
        "--evidence-id",
        default="manual_image_0",
    )

    parser.add_argument(
        "--provider",
        choices=("openai", "ollama"),
        default="openai",
        help="Inference API provider.",
    )

    return parser


def main(argv: Sequence[str] | None = None) -> None:
    parser = build_parser()
    arguments = parser.parse_args(argv)

    if not arguments.base_url:
        parser.error("--base-url or INSPECTRON_VLM_BASE_URL is required")

    if not arguments.model:
        parser.error("--model or INSPECTRON_VLM_MODEL is required")

    if arguments.provider == "ollama":
        client = OllamaVLMClient(
            base_url=arguments.base_url,
            model=arguments.model,
        )
    else:
        client = OpenAICompatibleVLMClient(
            base_url=arguments.base_url,
            model=arguments.model,
            api_key=os.getenv("INSPECTRON_VLM_API_KEY"),
        )

    perception = VLMPerception(client)

    frame = CapturedFrame(
        waypoint=arguments.waypoint,
        asset_id=arguments.asset_id,
        evidence_id=arguments.evidence_id,
        image_path=str(arguments.image),
    )

    observation = perception.analyze(frame)

    output = {
        "waypoint": observation.waypoint,
        "asset_id": observation.asset_id,
        "evidence_id": observation.evidence_id,
        "defect_type": observation.predicted_defect.value,
        "confidence": observation.confidence,
        "view_quality": observation.view_quality,
        "view_index": observation.view_index,
    }

    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()

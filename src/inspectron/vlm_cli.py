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
from inspectron.site_safety import (
    find_consistency_violations,
    resolve_safe_action,
)
from inspectron.vlm import (
    SITE_SAFETY_RESPONSE_SCHEMA,
    SiteSafetyVLMPerception,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=("Run an embodied site-safety assessment on one image."),
    )

    parser.add_argument(
        "--image",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--provider",
        choices=("ollama", "openai"),
        default=os.getenv(
            "INSPECTRON_VLM_PROVIDER",
            "ollama",
        ),
    )
    parser.add_argument(
        "--base-url",
        default=os.getenv(
            "INSPECTRON_VLM_BASE_URL",
            "http://localhost:11434",
        ),
    )
    parser.add_argument(
        "--model",
        default=os.getenv(
            "INSPECTRON_VLM_MODEL",
            "qwen3-vl:8b",
        ),
    )
    parser.add_argument(
        "--waypoint",
        default="manual_site_inspection",
    )
    parser.add_argument(
        "--scene-id",
        default="unknown_scene",
    )
    parser.add_argument(
        "--evidence-id",
        default="manual_frame_0",
    )

    return parser


def main(
    argv: Sequence[str] | None = None,
) -> None:
    arguments = build_parser().parse_args(argv)

    if arguments.provider == "ollama":
        client = OllamaVLMClient(
            base_url=arguments.base_url,
            model=arguments.model,
            response_schema=SITE_SAFETY_RESPONSE_SCHEMA,
        )
    else:
        client = OpenAICompatibleVLMClient(
            base_url=arguments.base_url,
            model=arguments.model,
            api_key=os.getenv("INSPECTRON_VLM_API_KEY"),
        )

    perception = SiteSafetyVLMPerception(client)

    frame = CapturedFrame(
        waypoint=arguments.waypoint,
        scene_id=arguments.scene_id,
        evidence_id=arguments.evidence_id,
        image_path=str(arguments.image),
    )

    assessment = perception.analyze(frame)
    enforced_action = resolve_safe_action(assessment)

    violations = find_consistency_violations(assessment)

    output = {
        "waypoint": assessment.waypoint,
        "scene_id": arguments.scene_id,
        "evidence_id": assessment.evidence_id,
        "traversability": (assessment.traversability.value),
        "hazards": sorted(hazard.value for hazard in assessment.hazards),
        "model_recommended_action": (assessment.recommended_action.value),
        "enforced_action": enforced_action.value,
        "policy_overrode_model": (enforced_action is not assessment.recommended_action),
        "consistency_violations": list(violations),
        "confidence": assessment.confidence,
        "view_quality": assessment.view_quality,
    }

    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()

from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections import Counter
from collections.abc import Sequence
from pathlib import Path
from urllib.parse import urlparse

from inspectron.site_safety import (
    HazardType,
    RecommendedAction,
    SceneAssessment,
    Traversability,
    resolve_safe_action,
)

REQUIRED_FIELDS = frozenset(
    {
        "id",
        "image",
        "traversability",
        "hazards",
        "expected_action",
        "split",
        "source_dataset",
        "source_id",
        "source_page_url",
        "author",
        "license_name",
        "license_url",
        "image_sha256",
    }
)

ALLOWED_LICENSES = frozenset(
    {
        "CC BY 2.0",
        "CC BY 4.0",
        "CC BY-SA 4.0",
        "CC0 1.0",
        "Public Domain",
    }
)

SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


def _load_manifest(path: Path) -> list[dict[str, object]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"Manifest is not valid JSON: {path}") from error

    if not isinstance(payload, list):
        raise ValueError("Manifest must contain a JSON array")

    if not payload:
        raise ValueError("Manifest cannot be empty")

    for index, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"Manifest item {index} must be an object")

    return payload


def _required_string(
    item: dict[str, object],
    field: str,
    index: int,
) -> str:
    value = item[field]

    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Manifest item {index} has an invalid {field}")

    return value.strip()


def _validate_https_url(
    value: str,
    field: str,
    index: int,
) -> None:
    parsed = urlparse(value)

    if parsed.scheme != "https" or not parsed.netloc:
        raise ValueError(f"Manifest item {index} field {field} must be an HTTPS URL")


def _safe_relative_path(
    value: str,
    index: int,
) -> Path:
    path = Path(value)

    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"Manifest item {index} must use a safe relative image path")

    return path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as image_file:
        for chunk in iter(lambda: image_file.read(1024 * 1024), b""):
            digest.update(chunk)

    return digest.hexdigest()


def _validate_labels(
    item: dict[str, object],
    index: int,
) -> tuple[Traversability, frozenset[HazardType], RecommendedAction]:
    raw_hazards = item["hazards"]

    if not isinstance(raw_hazards, list):
        raise ValueError(f"Manifest item {index} hazards must be an array")

    if not all(isinstance(value, str) for value in raw_hazards):
        raise ValueError(f"Manifest item {index} hazards must contain strings")

    if len(raw_hazards) != len(set(raw_hazards)):
        raise ValueError(f"Manifest item {index} contains duplicate hazards")

    try:
        traversability = Traversability(item["traversability"])
        hazards = frozenset(HazardType(value) for value in raw_hazards)
        expected_action = RecommendedAction(item["expected_action"])
    except ValueError as error:
        raise ValueError(f"Manifest item {index} contains an unsupported label") from error

    if traversability is Traversability.CLEAR and hazards:
        raise ValueError(f"Manifest item {index} cannot be clear while containing hazards")

    expected_assessment = SceneAssessment(
        waypoint=f"benchmark_{index:04d}",
        evidence_id=_required_string(item, "id", index),
        traversability=traversability,
        hazards=hazards,
        recommended_action=expected_action,
        confidence=1.0,
        view_quality=1.0,
    )

    policy_action = resolve_safe_action(expected_assessment)

    if expected_action is not policy_action:
        raise ValueError(f"Manifest item {index} expected_action must be {policy_action.value}")

    return traversability, hazards, expected_action


def validate_manifest(
    manifest_path: Path,
    *,
    data_root: Path | None,
    verify_files: bool = True,
) -> dict[str, object]:
    records = _load_manifest(manifest_path)

    if verify_files and data_root is None:
        raise ValueError("data_root is required when file verification is enabled")

    resolved_data_root = data_root.resolve() if data_root else None

    seen_ids: set[str] = set()
    seen_images: set[str] = set()
    seen_sources: set[tuple[str, str]] = set()
    seen_hashes: set[str] = set()

    traversability_counts: Counter[str] = Counter()
    hazard_counts: Counter[str] = Counter()
    action_counts: Counter[str] = Counter()
    source_counts: Counter[str] = Counter()
    license_counts: Counter[str] = Counter()
    verified_files = 0

    for index, item in enumerate(records):
        missing = REQUIRED_FIELDS - item.keys()

        if missing:
            raise ValueError(f"Manifest item {index} is missing: {sorted(missing)}")

        sample_id = _required_string(item, "id", index)
        image = _required_string(item, "image", index)
        split = _required_string(item, "split", index)
        source_dataset = _required_string(item, "source_dataset", index)
        source_id = _required_string(item, "source_id", index)
        source_page_url = _required_string(
            item,
            "source_page_url",
            index,
        )
        _required_string(item, "author", index)
        license_name = _required_string(item, "license_name", index)
        license_url = _required_string(item, "license_url", index)
        expected_hash = _required_string(item, "image_sha256", index)

        if split != "benchmark":
            raise ValueError(f"Manifest item {index} split must be benchmark")

        if license_name not in ALLOWED_LICENSES:
            raise ValueError(f"Manifest item {index} uses unsupported license {license_name!r}")

        _validate_https_url(source_page_url, "source_page_url", index)
        _validate_https_url(license_url, "license_url", index)

        if not SHA256_PATTERN.fullmatch(expected_hash):
            raise ValueError(f"Manifest item {index} has an invalid image_sha256")

        relative_image = _safe_relative_path(image, index)
        source_key = (source_dataset, source_id)

        if sample_id in seen_ids:
            raise ValueError(f"Duplicate sample id: {sample_id}")

        if image in seen_images:
            raise ValueError(f"Duplicate image path: {image}")

        if source_key in seen_sources:
            raise ValueError(f"Duplicate source image: {source_dataset}/{source_id}")

        if expected_hash in seen_hashes:
            raise ValueError(f"Duplicate image content hash: {expected_hash}")

        traversability, hazards, expected_action = _validate_labels(
            item,
            index,
        )

        if verify_files:
            assert resolved_data_root is not None

            try:
                image_path = (resolved_data_root / relative_image).resolve(strict=True)
            except FileNotFoundError as error:
                raise ValueError(f"Manifest item {index} image does not exist: {image}") from error

            if not image_path.is_relative_to(resolved_data_root):
                raise ValueError(f"Manifest item {index} resolves outside data_root")

            if not image_path.is_file():
                raise ValueError(f"Manifest item {index} image is not a file")

            actual_hash = _sha256(image_path)

            if actual_hash != expected_hash:
                raise ValueError(f"Manifest item {index} SHA-256 mismatch for {image}")

            verified_files += 1

        seen_ids.add(sample_id)
        seen_images.add(image)
        seen_sources.add(source_key)
        seen_hashes.add(expected_hash)

        traversability_counts[traversability.value] += 1
        action_counts[expected_action.value] += 1
        source_counts[source_dataset] += 1
        license_counts[license_name] += 1

        for hazard in hazards:
            hazard_counts[hazard.value] += 1

    return {
        "sample_count": len(records),
        "verified_files": verified_files,
        "traversability_counts": dict(sorted(traversability_counts.items())),
        "hazard_counts": dict(sorted(hazard_counts.items())),
        "action_counts": dict(sorted(action_counts.items())),
        "source_counts": dict(sorted(source_counts.items())),
        "license_counts": dict(sorted(license_counts.items())),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate Inspectron benchmark data and provenance."
    )
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--data-root", type=Path)
    parser.add_argument(
        "--metadata-only",
        action="store_true",
        help="Validate metadata without checking local image files.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    parser = build_parser()
    arguments = parser.parse_args(argv)

    if not arguments.metadata_only and arguments.data_root is None:
        parser.error("--data-root is required unless --metadata-only is used")

    report = validate_manifest(
        arguments.manifest,
        data_root=arguments.data_root,
        verify_files=not arguments.metadata_only,
    )

    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

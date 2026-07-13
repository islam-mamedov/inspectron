from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any
from urllib.parse import urlparse
from urllib.request import HTTPRedirectHandler, Request, build_opener

ALLOWED_CONTENT_TYPES = frozenset(
    {
        "image/jpeg",
        "image/png",
        "image/webp",
    }
)

SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
DEFAULT_MAX_IMAGE_BYTES = 25 * 1024 * 1024
CHUNK_SIZE = 1024 * 1024
USER_AGENT = "Inspectron-Benchmark/0.1"


def _required_string(
    item: dict[str, object],
    field: str,
    index: int,
) -> str:
    value = item.get(field)

    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Manifest item {index} has an invalid {field}")

    return value.strip()


def _safe_relative_path(
    value: str,
    index: int,
) -> Path:
    path = Path(value)

    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"Manifest item {index} must use a safe relative image path")

    return path


def _validate_https_url(
    value: str,
    *,
    context: str,
) -> None:
    parsed = urlparse(value)

    if parsed.scheme != "https" or not parsed.netloc:
        raise ValueError(f"{context} must be an HTTPS URL")


class _HTTPSOnlyRedirectHandler(HTTPRedirectHandler):
    """Refuse redirects leaving HTTPS before the follow-up request is sent."""

    def redirect_request(
        self,
        req: Request,
        fp: Any,
        code: int,
        msg: str,
        headers: Any,
        newurl: str,
    ) -> Request | None:
        if urlparse(newurl).scheme != "https":
            raise ValueError(f"Refusing redirect to non-HTTPS URL: {newurl}")

        return super().redirect_request(req, fp, code, msg, headers, newurl)


_HTTPS_ONLY_OPENER = build_opener(_HTTPSOnlyRedirectHandler)


def _open_https_url(
    request: Request,
    *,
    timeout: float,
) -> Any:
    return _HTTPS_ONLY_OPENER.open(request, timeout=timeout)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()

    with path.open("rb") as input_file:
        for chunk in iter(lambda: input_file.read(CHUNK_SIZE), b""):
            digest.update(chunk)

    return digest.hexdigest()


def _looks_like_image(path: Path) -> bool:
    with path.open("rb") as input_file:
        header = input_file.read(12)

    is_jpeg = header.startswith(b"\xff\xd8\xff")
    is_png = header.startswith(b"\x89PNG\r\n\x1a\n")
    is_webp = len(header) >= 12 and header.startswith(b"RIFF") and header[8:12] == b"WEBP"

    return is_jpeg or is_png or is_webp


def load_download_records(
    manifest_path: Path,
) -> list[dict[str, str]]:
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"Manifest is not valid JSON: {manifest_path}") from error

    if not isinstance(payload, list):
        raise ValueError("Manifest must contain a JSON array")

    if not payload:
        raise ValueError("Manifest cannot be empty")

    records: list[dict[str, str]] = []
    seen_images: set[str] = set()

    for index, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"Manifest item {index} must be an object")

        sample_id = _required_string(item, "id", index)
        image = _required_string(item, "image", index)
        download_url = _required_string(
            item,
            "download_url",
            index,
        )
        expected_hash = _required_string(
            item,
            "image_sha256",
            index,
        )

        _safe_relative_path(image, index)
        _validate_https_url(
            download_url,
            context=f"Manifest item {index} download_url",
        )

        if not SHA256_PATTERN.fullmatch(expected_hash):
            raise ValueError(f"Manifest item {index} has an invalid image_sha256")

        if image in seen_images:
            raise ValueError(f"Duplicate image path: {image}")

        seen_images.add(image)

        records.append(
            {
                "id": sample_id,
                "image": image,
                "download_url": download_url,
                "image_sha256": expected_hash,
            }
        )

    return records


def _target_path(
    *,
    data_root: Path,
    relative_image: str,
) -> Path:
    resolved_root = data_root.resolve()
    target = (resolved_root / relative_image).resolve(strict=False)

    if not target.is_relative_to(resolved_root):
        raise ValueError(f"Image path resolves outside data root: {relative_image}")

    target.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    return target


def _download_record(
    *,
    record: dict[str, str],
    data_root: Path,
    max_image_bytes: int,
    timeout_seconds: float,
    opener: Callable[..., Any],
) -> tuple[str, int]:
    target = _target_path(
        data_root=data_root,
        relative_image=record["image"],
    )
    expected_hash = record["image_sha256"]

    if target.exists():
        if not target.is_file():
            raise ValueError(f"Existing image path is not a file: {target}")

        actual_hash = _sha256(target)

        if actual_hash != expected_hash:
            raise ValueError(f"Existing file hash mismatch: {record['image']}")

        if not _looks_like_image(target):
            raise ValueError(f"Existing file is not a supported image: {record['image']}")

        return "verified_existing", target.stat().st_size

    request = Request(
        record["download_url"],
        headers={"User-Agent": USER_AGENT},
    )
    partial = target.with_name(f".{target.name}.part")
    partial.unlink(missing_ok=True)

    response = None

    try:
        response = opener(
            request,
            timeout=timeout_seconds,
        )

        final_url = response.geturl()
        _validate_https_url(
            final_url,
            context=f"Final download URL for {record['image']}",
        )

        raw_content_type = response.headers.get("Content-Type") or ""
        content_type = raw_content_type.split(";", 1)[0].strip().lower()

        if content_type not in ALLOWED_CONTENT_TYPES:
            raise ValueError(f"Unsupported response content type: {content_type or 'missing'}")

        raw_content_length = response.headers.get("Content-Length")

        if raw_content_length is not None:
            try:
                content_length = int(raw_content_length)
            except ValueError as error:
                raise ValueError("Response has an invalid Content-Length") from error

            if content_length > max_image_bytes:
                raise ValueError(f"Image exceeds {max_image_bytes} bytes")

        digest = hashlib.sha256()
        downloaded_bytes = 0

        with partial.open("wb") as output_file:
            while True:
                chunk = response.read(CHUNK_SIZE)

                if not chunk:
                    break

                downloaded_bytes += len(chunk)

                if downloaded_bytes > max_image_bytes:
                    raise ValueError(f"Image exceeds {max_image_bytes} bytes")

                digest.update(chunk)
                output_file.write(chunk)

        if downloaded_bytes == 0:
            raise ValueError("Downloaded image is empty")

        if not _looks_like_image(partial):
            raise ValueError("Downloaded file is not a supported image")

        actual_hash = digest.hexdigest()

        if actual_hash != expected_hash:
            raise ValueError(
                f"Downloaded SHA-256 mismatch for "
                f"{record['image']}: expected {expected_hash}, "
                f"received {actual_hash}"
            )

        partial.replace(target)
        return "downloaded", downloaded_bytes

    except Exception:
        partial.unlink(missing_ok=True)
        raise

    finally:
        if response is not None:
            response.close()


def download_manifest(
    *,
    manifest_path: Path,
    data_root: Path,
    max_image_bytes: int = DEFAULT_MAX_IMAGE_BYTES,
    timeout_seconds: float = 30.0,
    opener: Callable[..., Any] = _open_https_url,
) -> dict[str, object]:
    if max_image_bytes <= 0:
        raise ValueError("max_image_bytes must be positive")

    if timeout_seconds <= 0:
        raise ValueError("timeout_seconds must be positive")

    records = load_download_records(manifest_path)

    downloaded_count = 0
    verified_existing_count = 0
    total_bytes = 0
    results: list[dict[str, object]] = []

    for record in records:
        status, image_bytes = _download_record(
            record=record,
            data_root=data_root,
            max_image_bytes=max_image_bytes,
            timeout_seconds=timeout_seconds,
            opener=opener,
        )

        if status == "downloaded":
            downloaded_count += 1
        else:
            verified_existing_count += 1

        total_bytes += image_bytes

        results.append(
            {
                "id": record["id"],
                "image": record["image"],
                "status": status,
                "bytes": image_bytes,
                "sha256": record["image_sha256"],
            }
        )

    return {
        "sample_count": len(records),
        "downloaded_count": downloaded_count,
        "verified_existing_count": (verified_existing_count),
        "total_bytes": total_bytes,
        "results": results,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=("Download and verify Inspectron benchmark images.")
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--data-root",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--max-image-mb",
        type=float,
        default=25.0,
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=30.0,
    )
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    arguments = build_parser().parse_args(argv)

    max_image_bytes = int(arguments.max_image_mb * 1024 * 1024)

    report = download_manifest(
        manifest_path=arguments.manifest,
        data_root=arguments.data_root,
        max_image_bytes=max_image_bytes,
        timeout_seconds=arguments.timeout_seconds,
    )

    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

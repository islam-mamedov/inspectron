from __future__ import annotations

import hashlib
import importlib.util
import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType
from typing import Any


def _load_downloader() -> ModuleType:
    script_path = (
        Path(__file__).resolve().parents[1] / "benchmarks" / "scripts" / "download_dataset.py"
    )

    specification = importlib.util.spec_from_file_location(
        "inspectron_dataset_downloader",
        script_path,
    )

    if specification is None or specification.loader is None:
        raise RuntimeError("Unable to load dataset downloader")

    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


DOWNLOADER = _load_downloader()
JPEG_BYTES = b"\xff\xd8\xff\xe0inspectron-test-image"


class FakeResponse:
    def __init__(
        self,
        *,
        data: bytes,
        url: str = "https://example.org/image.jpg",
        content_type: str = "image/jpeg",
    ) -> None:
        self.data = data
        self.url = url
        self.offset = 0
        self.closed = False
        self.headers = {
            "Content-Type": content_type,
            "Content-Length": str(len(data)),
        }

    def read(self, size: int = -1) -> bytes:
        if self.offset >= len(self.data):
            return b""

        if size < 0:
            size = len(self.data) - self.offset

        chunk = self.data[self.offset : self.offset + size]
        self.offset += len(chunk)
        return chunk

    def geturl(self) -> str:
        return self.url

    def close(self) -> None:
        self.closed = True


def _record(
    image_hash: str,
    *,
    download_url: str = "https://example.org/image.jpg",
) -> dict[str, object]:
    return {
        "id": "sample_001",
        "image": "clear/sample_001.jpg",
        "download_url": download_url,
        "image_sha256": image_hash,
    }


def _write_manifest(
    path: Path,
    record: dict[str, object],
) -> None:
    path.write_text(
        json.dumps([record]),
        encoding="utf-8",
    )


class DatasetDownloadTests(unittest.TestCase):
    def test_downloads_and_verifies_image(self) -> None:
        expected_hash = hashlib.sha256(JPEG_BYTES).hexdigest()

        with TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            data_root = root / "data"
            _write_manifest(
                manifest,
                _record(expected_hash),
            )

            response = FakeResponse(data=JPEG_BYTES)

            def opener(
                request: Any,
                *,
                timeout: float,
            ) -> FakeResponse:
                self.assertEqual(
                    request.full_url,
                    "https://example.org/image.jpg",
                )
                self.assertEqual(timeout, 5.0)
                return response

            report = DOWNLOADER.download_manifest(
                manifest_path=manifest,
                data_root=data_root,
                timeout_seconds=5.0,
                opener=opener,
            )

            downloaded = data_root / "clear" / "sample_001.jpg"

            self.assertEqual(
                downloaded.read_bytes(),
                JPEG_BYTES,
            )
            self.assertEqual(
                report["downloaded_count"],
                1,
            )
            self.assertEqual(
                report["verified_existing_count"],
                0,
            )
            self.assertTrue(response.closed)

    def test_existing_valid_image_skips_network(self) -> None:
        expected_hash = hashlib.sha256(JPEG_BYTES).hexdigest()

        with TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            data_root = root / "data"
            image = data_root / "clear" / "sample_001.jpg"
            image.parent.mkdir(parents=True)
            image.write_bytes(JPEG_BYTES)

            _write_manifest(
                manifest,
                _record(expected_hash),
            )

            def opener(*args: Any, **kwargs: Any) -> Any:
                raise AssertionError("Network must not be called")

            report = DOWNLOADER.download_manifest(
                manifest_path=manifest,
                data_root=data_root,
                opener=opener,
            )

            self.assertEqual(
                report["downloaded_count"],
                0,
            )
            self.assertEqual(
                report["verified_existing_count"],
                1,
            )

    def test_rejects_hash_mismatch(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            data_root = root / "data"

            _write_manifest(
                manifest,
                _record("0" * 64),
            )

            response = FakeResponse(data=JPEG_BYTES)

            with self.assertRaisesRegex(
                ValueError,
                "SHA-256 mismatch",
            ):
                DOWNLOADER.download_manifest(
                    manifest_path=manifest,
                    data_root=data_root,
                    opener=lambda *args, **kwargs: response,
                )

            target = data_root / "clear" / "sample_001.jpg"
            partial = data_root / "clear" / ".sample_001.jpg.part"

            self.assertFalse(target.exists())
            self.assertFalse(partial.exists())

    def test_rejects_non_https_download(self) -> None:
        expected_hash = hashlib.sha256(JPEG_BYTES).hexdigest()

        with TemporaryDirectory() as directory:
            manifest = Path(directory) / "manifest.json"
            _write_manifest(
                manifest,
                _record(
                    expected_hash,
                    download_url=("http://example.org/image.jpg"),
                ),
            )

            with self.assertRaisesRegex(
                ValueError,
                "HTTPS",
            ):
                DOWNLOADER.load_download_records(manifest)

    def test_rejects_symlink_escape_before_creating_directories(self) -> None:
        expected_hash = hashlib.sha256(JPEG_BYTES).hexdigest()

        with TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            data_root = root / "data"
            data_root.mkdir()

            outside = root / "outside"
            outside.mkdir()
            (data_root / "clear").symlink_to(
                outside,
                target_is_directory=True,
            )

            _write_manifest(
                manifest,
                _record(expected_hash),
            )

            def opener(*args: Any, **kwargs: Any) -> Any:
                raise AssertionError("Network must not be called")

            with self.assertRaisesRegex(
                ValueError,
                "outside data root",
            ):
                DOWNLOADER.download_manifest(
                    manifest_path=manifest,
                    data_root=data_root,
                    opener=opener,
                )

            self.assertEqual(list(outside.iterdir()), [])

    def test_redirect_handler_refuses_non_https_redirect(self) -> None:
        handler = DOWNLOADER._HTTPSOnlyRedirectHandler()
        request = DOWNLOADER.Request("https://example.org/image.jpg")

        with self.assertRaisesRegex(ValueError, "non-HTTPS"):
            handler.redirect_request(
                request,
                None,
                302,
                "Found",
                {},
                "http://example.org/image.jpg",
            )

    def test_redirect_handler_allows_https_redirect(self) -> None:
        handler = DOWNLOADER._HTTPSOnlyRedirectHandler()
        request = DOWNLOADER.Request("https://example.org/image.jpg")

        redirected = handler.redirect_request(
            request,
            None,
            302,
            "Found",
            {},
            "https://cdn.example.org/image.jpg",
        )

        self.assertIsNotNone(redirected)
        self.assertEqual(
            redirected.full_url,
            "https://cdn.example.org/image.jpg",
        )

    def test_rejects_response_that_ends_on_non_https_url(self) -> None:
        expected_hash = hashlib.sha256(JPEG_BYTES).hexdigest()

        with TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            data_root = root / "data"

            _write_manifest(
                manifest,
                _record(expected_hash),
            )

            response = FakeResponse(
                data=JPEG_BYTES,
                url="http://example.org/image.jpg",
            )

            with self.assertRaisesRegex(
                ValueError,
                "HTTPS",
            ):
                DOWNLOADER.download_manifest(
                    manifest_path=manifest,
                    data_root=data_root,
                    opener=lambda *args, **kwargs: response,
                )

            self.assertFalse((data_root / "clear" / "sample_001.jpg").exists())

    def test_rejects_non_image_response(self) -> None:
        expected_hash = hashlib.sha256(JPEG_BYTES).hexdigest()

        with TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"

            _write_manifest(
                manifest,
                _record(expected_hash),
            )

            response = FakeResponse(
                data=b"<html>not an image</html>",
                content_type="text/html",
            )

            with self.assertRaisesRegex(
                ValueError,
                "content type",
            ):
                DOWNLOADER.download_manifest(
                    manifest_path=manifest,
                    data_root=root / "data",
                    opener=lambda *args, **kwargs: response,
                )


if __name__ == "__main__":
    unittest.main()

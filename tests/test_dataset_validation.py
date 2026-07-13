from __future__ import annotations

import hashlib
import importlib.util
import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType


def _load_validator() -> ModuleType:
    script_path = (
        Path(__file__).resolve().parents[1] / "benchmarks" / "scripts" / "validate_dataset.py"
    )

    specification = importlib.util.spec_from_file_location(
        "inspectron_dataset_validator",
        script_path,
    )

    if specification is None or specification.loader is None:
        raise RuntimeError("Unable to load dataset validator")

    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


VALIDATOR = _load_validator()


def _sample(image_hash: str) -> dict[str, object]:
    return {
        "id": "clear_001",
        "image": "clear_001.jpg",
        "traversability": "clear",
        "hazards": [],
        "expected_action": "proceed",
        "split": "benchmark",
        "source_dataset": "Open Images",
        "source_id": "example-image-id",
        "source_page_url": "https://example.org/source-image",
        "author": "Example Author",
        "license_name": "CC BY 2.0",
        "license_url": "https://creativecommons.org/licenses/by/2.0/",
        "image_sha256": image_hash,
    }


class DatasetValidationTests(unittest.TestCase):
    def test_validates_image_and_provenance(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            data_root = root / "data"
            data_root.mkdir()

            image_bytes = b"inspectron-real-image"
            image_path = data_root / "clear_001.jpg"
            image_path.write_bytes(image_bytes)

            image_hash = hashlib.sha256(image_bytes).hexdigest()
            manifest = root / "manifest.json"
            manifest.write_text(
                json.dumps([_sample(image_hash)]),
                encoding="utf-8",
            )

            report = VALIDATOR.validate_manifest(
                manifest,
                data_root=data_root,
            )

            self.assertEqual(report["sample_count"], 1)
            self.assertEqual(report["verified_files"], 1)
            self.assertEqual(
                report["traversability_counts"],
                {"clear": 1},
            )

    def test_rejects_modified_image(self) -> None:
        with TemporaryDirectory() as directory:
            root = Path(directory)
            data_root = root / "data"
            data_root.mkdir()

            image_path = data_root / "clear_001.jpg"
            image_path.write_bytes(b"modified-image")

            manifest = root / "manifest.json"
            manifest.write_text(
                json.dumps([_sample("0" * 64)]),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                VALIDATOR.validate_manifest(
                    manifest,
                    data_root=data_root,
                )

    def test_rejects_missing_provenance(self) -> None:
        sample = _sample("0" * 64)
        del sample["author"]

        with TemporaryDirectory() as directory:
            manifest = Path(directory) / "manifest.json"
            manifest.write_text(
                json.dumps([sample]),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "author"):
                VALIDATOR.validate_manifest(
                    manifest,
                    data_root=None,
                    verify_files=False,
                )

    def test_rejects_action_that_violates_policy(self) -> None:
        sample = _sample("0" * 64)
        sample.update(
            {
                "traversability": "restricted",
                "hazards": ["human_in_path"],
                "expected_action": "proceed",
            }
        )

        with TemporaryDirectory() as directory:
            manifest = Path(directory) / "manifest.json"
            manifest.write_text(
                json.dumps([sample]),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(
                ValueError,
                "expected_action must be stop",
            ):
                VALIDATOR.validate_manifest(
                    manifest,
                    data_root=None,
                    verify_files=False,
                )


if __name__ == "__main__":
    unittest.main()

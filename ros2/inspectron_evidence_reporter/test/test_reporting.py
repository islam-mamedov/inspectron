from __future__ import annotations

import json
import stat
import unittest
from datetime import UTC, datetime
from hashlib import sha256
from pathlib import Path
from tempfile import TemporaryDirectory

from inspectron_evidence_reporter.reporting import (
    MissionReportRecorder,
    create_mission_id,
    ros_time_to_iso,
)

JPEG = b"\xff\xd8\xff\xd9"


def make_recorder(
    directory: str,
    *,
    mission_id: str = "mission-test-001",
) -> MissionReportRecorder:
    return MissionReportRecorder(
        output_directory=Path(directory),
        mission_id=mission_id,
        max_image_bytes=1024,
        started_at="2026-07-17T00:00:00.000000Z",
    )


class MissionReportRecorderTests(unittest.TestCase):
    def test_creates_deterministic_identifiers_and_timestamps(self):
        timestamp = datetime(
            2026,
            1,
            2,
            3,
            4,
            5,
            6789,
            tzinfo=UTC,
        )

        mission_id = create_mission_id(
            timestamp=timestamp,
            token="run 01",
        )

        self.assertEqual(
            mission_id,
            "mission-20260102T030405.006789Z-run-01",
        )
        self.assertEqual(
            ros_time_to_iso(0, 1),
            "1970-01-01T00:00:00.000000001Z",
        )

        with self.assertRaisesRegex(
            ValueError,
            "timezone-aware",
        ):
            create_mission_id(
                timestamp=datetime(2026, 1, 2),
                token="test",
            )

    def test_records_and_finalizes_complete_report(self):
        with TemporaryDirectory() as directory:
            recorder = make_recorder(directory)

            evidence = recorder.record_evidence(
                evidence_id="aisle_a-1.000000000-000001",
                waypoint="aisle_a",
                scene_id="warehouse",
                view_index=0,
                image_format="jpeg",
                image_data=JPEG,
                captured_at="2026-07-17T00:00:01.000000000Z",
            )
            recorder.record_assessment(
                evidence_id="aisle_a-1.000000000-000001",
                traversability=1,
                hazards=[1, 2],
                recommended_action=1,
                confidence=0.85,
                view_quality=0.90,
                observed_at="2026-07-17T00:00:01.100000000Z",
            )
            recorder.record_policy_decision(
                evidence_id="aisle_a-1.000000000-000001",
                action=1,
                reason=4,
                status=0,
                model_action_overridden=False,
                source_observed_at=("2026-07-17T00:00:01.100000000Z"),
                decided_at="2026-07-17T00:00:01.200000000Z",
            )
            recorder.record_mission_state(
                state=7,
                current_waypoint="aisle_a",
                active_goal="",
                waypoint_index=1,
                waypoint_count=1,
                last_evidence_id=("aisle_a-1.000000000-000001"),
                reason="All required waypoints were completed",
                motion_authorized=False,
                updated_at="2026-07-17T00:00:02.000000000Z",
            )

            partial_path = recorder.write_partial_report()
            partial = json.loads(partial_path.read_text(encoding="utf-8"))

            self.assertEqual(partial["outcome"], "recording")
            self.assertFalse(partial["integrity"]["verified"])

            artifacts = recorder.finalize(
                outcome="completed",
                finalized_at=("2026-07-17T00:00:03.000000000Z"),
            )

            report = json.loads(artifacts.json_path.read_text(encoding="utf-8"))
            markdown = artifacts.markdown_path.read_text(encoding="utf-8")

            self.assertTrue(recorder.finalized)
            self.assertEqual(recorder.evidence_count, 1)
            self.assertEqual(report["outcome"], "completed")
            self.assertEqual(report["coverage"], 1.0)
            self.assertEqual(
                report["counts"],
                {
                    "evidence": 1,
                    "assessments": 1,
                    "policy_decisions": 1,
                    "mission_states": 1,
                },
            )
            self.assertTrue(report["integrity"]["verified"])
            self.assertEqual(
                report["integrity"]["verified_evidence_files"],
                1,
            )
            self.assertEqual(
                report["unmatched"],
                {
                    "assessment_without_evidence": [],
                    "evidence_without_assessment": [],
                },
            )
            self.assertEqual(
                report["evidence"][0]["sha256"],
                sha256(JPEG).hexdigest(),
            )
            self.assertEqual(
                report["assessments"][0]["hazards"],
                ["debris", "liquid_spill"],
            )
            self.assertIn(
                "# Inspectron Mission Report",
                markdown,
            )
            self.assertIn(
                "aisle_a-1.000000000-000001",
                markdown,
            )
            self.assertFalse(partial_path.exists())

            image_path = recorder.mission_directory / str(evidence["image_path"])

            for path in (
                image_path,
                artifacts.json_path,
                artifacts.markdown_path,
            ):
                mode = stat.S_IMODE(path.stat().st_mode)
                self.assertEqual(mode & 0o222, 0)

            with self.assertRaisesRegex(
                RuntimeError,
                "immutable",
            ):
                recorder.record_mission_state(
                    state=8,
                    current_waypoint="aisle_a",
                    active_goal="",
                    waypoint_index=1,
                    waypoint_count=1,
                    last_evidence_id="",
                    reason="late mutation",
                    motion_authorized=False,
                    updated_at=("2026-07-17T00:00:04.000000000Z"),
                )

    def test_duplicate_evidence_is_idempotent(self):
        with TemporaryDirectory() as directory:
            recorder = make_recorder(directory)

            arguments = {
                "evidence_id": "frame-001",
                "waypoint": "aisle_a",
                "scene_id": "warehouse",
                "view_index": 0,
                "image_format": "jpeg",
                "image_data": JPEG,
                "captured_at": ("2026-07-17T00:00:01.000000000Z"),
            }

            first = recorder.record_evidence(**arguments)
            second = recorder.record_evidence(**arguments)

            self.assertEqual(first, second)
            self.assertEqual(recorder.evidence_count, 1)
            self.assertEqual(
                len(list(recorder.evidence_directory.iterdir())),
                1,
            )

            conflicting = {
                **arguments,
                "image_data": b"\xff\xd8\xff\xda",
            }

            with self.assertRaisesRegex(
                ValueError,
                "Conflicting evidence",
            ):
                recorder.record_evidence(**conflicting)

    def test_checksum_tampering_blocks_finalization(self):
        with TemporaryDirectory() as directory:
            recorder = make_recorder(directory)

            record = recorder.record_evidence(
                evidence_id="frame-001",
                waypoint="aisle_a",
                scene_id="warehouse",
                view_index=0,
                image_format="jpeg",
                image_data=JPEG,
                captured_at="2026-07-17T00:00:01.000000000Z",
            )

            image_path = recorder.mission_directory / str(record["image_path"])
            image_path.chmod(0o644)
            image_path.write_bytes(b"\xff\xd8\xff\xda")

            with self.assertRaisesRegex(
                RuntimeError,
                "checksum changed",
            ):
                recorder.finalize(
                    outcome="completed",
                    finalized_at=("2026-07-17T00:00:03.000000000Z"),
                )

            self.assertFalse(recorder.report_json_path.exists())
            self.assertFalse(recorder.report_markdown_path.exists())

    def test_rejects_invalid_inputs(self):
        with TemporaryDirectory() as directory:
            with self.assertRaisesRegex(
                ValueError,
                "mission_id",
            ):
                make_recorder(
                    directory,
                    mission_id="../unsafe",
                )

            recorder = make_recorder(directory)

            with self.assertRaisesRegex(
                ValueError,
                "JPEG signature",
            ):
                recorder.record_evidence(
                    evidence_id="frame-001",
                    waypoint="aisle_a",
                    scene_id="warehouse",
                    view_index=0,
                    image_format="jpeg",
                    image_data=b"invalid",
                    captured_at=("2026-07-17T00:00:01.000000000Z"),
                )

            with self.assertRaisesRegex(
                ValueError,
                "confidence",
            ):
                recorder.record_assessment(
                    evidence_id="frame-001",
                    traversability=0,
                    hazards=[],
                    recommended_action=0,
                    confidence=float("nan"),
                    view_quality=0.9,
                    observed_at=("2026-07-17T00:00:01.000000000Z"),
                )

            with self.assertRaisesRegex(
                ValueError,
                "cannot exceed",
            ):
                recorder.record_mission_state(
                    state=2,
                    current_waypoint="aisle_a",
                    active_goal="aisle_a",
                    waypoint_index=2,
                    waypoint_count=1,
                    last_evidence_id="",
                    reason="invalid progress",
                    motion_authorized=True,
                    updated_at=("2026-07-17T00:00:01.000000000Z"),
                )

    def test_reports_unmatched_evidence_and_assessments(self):
        with TemporaryDirectory() as directory:
            recorder = make_recorder(directory)

            recorder.record_evidence(
                evidence_id="evidence-only",
                waypoint="aisle_a",
                scene_id="warehouse",
                view_index=0,
                image_format="jpeg",
                image_data=JPEG,
                captured_at="2026-07-17T00:00:01.000000000Z",
            )
            recorder.record_assessment(
                evidence_id="assessment-only",
                traversability=0,
                hazards=[],
                recommended_action=0,
                confidence=0.9,
                view_quality=0.9,
                observed_at="2026-07-17T00:00:02.000000000Z",
            )

            artifacts = recorder.finalize(
                outcome="aborted",
                finalized_at=("2026-07-17T00:00:03.000000000Z"),
            )
            report = json.loads(artifacts.json_path.read_text(encoding="utf-8"))

            self.assertEqual(
                report["unmatched"]["evidence_without_assessment"],
                ["evidence-only"],
            )
            self.assertEqual(
                report["unmatched"]["assessment_without_evidence"],
                ["assessment-only"],
            )


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import json
import math
import os
import re
import secrets
from dataclasses import dataclass
from datetime import UTC, datetime
from hashlib import sha256
from pathlib import Path

TRAVERSABILITY_LABELS = {
    0: "clear",
    1: "restricted",
    2: "blocked",
    3: "unknown",
}

HAZARD_LABELS = {
    0: "human_in_path",
    1: "debris",
    2: "liquid_spill",
    3: "open_edge",
    4: "fire_or_smoke",
    5: "unstable_load",
}

ACTION_LABELS = {
    0: "proceed",
    1: "slow_down",
    2: "stop",
    3: "reroute",
    4: "inspect_closer",
}

POLICY_REASON_LABELS = {
    0: "invalid_assessment",
    1: "critical_hazard",
    2: "blocked_path",
    3: "weak_evidence",
    4: "restricted_path",
    5: "clear_path",
}

POLICY_STATUS_LABELS = {
    0: "valid",
    1: "invalid",
    2: "stale",
}

MISSION_STATE_LABELS = {
    0: "idle",
    1: "waiting_for_policy",
    2: "moving",
    3: "paused",
    4: "inspecting_closer",
    5: "rerouting",
    6: "safety_stopped",
    7: "completed",
    8: "aborted",
    9: "emergency_stopped",
}

_MISSION_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}")


@dataclass(frozen=True, slots=True)
class ReportArtifacts:
    json_path: Path
    markdown_path: Path


def utc_now_iso() -> str:
    return datetime.now(UTC).isoformat(timespec="microseconds").replace("+00:00", "Z")


def ros_time_to_iso(
    seconds: int,
    nanoseconds: int,
) -> str:
    if isinstance(seconds, bool) or not isinstance(seconds, int):
        raise ValueError("seconds must be an integer")

    if (
        isinstance(nanoseconds, bool)
        or not isinstance(nanoseconds, int)
        or not 0 <= nanoseconds < 1_000_000_000
    ):
        raise ValueError("nanoseconds must be between 0 and 999999999")

    timestamp = datetime.fromtimestamp(seconds, UTC)

    return timestamp.strftime("%Y-%m-%dT%H:%M:%S") + f".{nanoseconds:09d}Z"


def create_mission_id(
    *,
    timestamp: datetime | None = None,
    token: str | None = None,
) -> str:
    moment = timestamp or datetime.now(UTC)

    if moment.tzinfo is None:
        raise ValueError("Mission timestamp must be timezone-aware")

    normalized = moment.astimezone(UTC)
    suffix = token or secrets.token_hex(4)
    safe_suffix = re.sub(
        r"[^A-Za-z0-9_.-]+",
        "-",
        suffix,
    ).strip(".-")

    if not safe_suffix:
        raise ValueError("Mission token cannot be empty")

    return "mission-" + normalized.strftime("%Y%m%dT%H%M%S.%fZ") + "-" + safe_suffix[:32]


class MissionReportRecorder:
    def __init__(
        self,
        *,
        output_directory: Path,
        mission_id: str,
        max_image_bytes: int,
        started_at: str,
    ) -> None:
        if not _MISSION_ID_PATTERN.fullmatch(mission_id):
            raise ValueError("mission_id contains unsupported characters")

        if (
            isinstance(max_image_bytes, bool)
            or not isinstance(max_image_bytes, int)
            or max_image_bytes <= 0
        ):
            raise ValueError("max_image_bytes must be a positive integer")

        self.mission_id = mission_id
        self.max_image_bytes = max_image_bytes
        self.started_at = _required_text(
            "started_at",
            started_at,
        )

        root = output_directory.expanduser()
        root.mkdir(parents=True, exist_ok=True)

        self.mission_directory = root / mission_id
        self.mission_directory.mkdir(
            parents=False,
            exist_ok=False,
        )

        self.evidence_directory = self.mission_directory / "evidence"
        self.evidence_directory.mkdir()

        self.partial_report_path = self.mission_directory / "report.partial.json"
        self.report_json_path = self.mission_directory / "report.json"
        self.report_markdown_path = self.mission_directory / "report.md"

        self._evidence: dict[str, dict[str, object]] = {}
        self._assessments: dict[
            str,
            dict[str, object],
        ] = {}
        self._policy_decisions: list[dict[str, object]] = []
        self._mission_states: list[dict[str, object]] = []
        self._finalized = False
        self._artifacts: ReportArtifacts | None = None

    @property
    def finalized(self) -> bool:
        return self._finalized

    @property
    def evidence_count(self) -> int:
        return len(self._evidence)

    def record_evidence(
        self,
        *,
        evidence_id: str,
        waypoint: str,
        scene_id: str,
        view_index: int,
        image_format: str,
        image_data: bytes,
        captured_at: str,
    ) -> dict[str, object]:
        self._ensure_mutable()

        evidence_id = _required_text(
            "evidence_id",
            evidence_id,
        )
        waypoint = _required_text("waypoint", waypoint)
        scene_id = _required_text("scene_id", scene_id)
        image_format = _required_text(
            "image_format",
            image_format,
        )
        captured_at = _required_text(
            "captured_at",
            captured_at,
        )

        if isinstance(view_index, bool) or not isinstance(view_index, int) or view_index < 0:
            raise ValueError("view_index must be a non-negative integer")

        try:
            payload = bytes(image_data)
        except (TypeError, ValueError) as error:
            raise ValueError("image_data must be byte-compatible") from error

        suffix = _image_suffix(
            image_format=image_format,
            data=payload,
            max_image_bytes=self.max_image_bytes,
        )
        digest = sha256(payload).hexdigest()
        filename = _evidence_filename(
            evidence_id,
            suffix,
        )
        relative_path = f"evidence/{filename}"

        record: dict[str, object] = {
            "evidence_id": evidence_id,
            "waypoint": waypoint,
            "scene_id": scene_id,
            "view_index": view_index,
            "image_format": image_format,
            "image_path": relative_path,
            "sha256": digest,
            "byte_count": len(payload),
            "captured_at": captured_at,
        }

        existing = self._evidence.get(evidence_id)

        if existing is not None:
            if existing == record:
                return dict(existing)

            raise ValueError("Conflicting evidence reused the same evidence ID")

        image_path = self.mission_directory / relative_path

        try:
            with image_path.open("xb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
        except FileExistsError as error:
            raise RuntimeError(
                "Evidence file already exists and will not be overwritten"
            ) from error

        image_path.chmod(0o444)
        self._evidence[evidence_id] = record

        return dict(record)

    def record_assessment(
        self,
        *,
        evidence_id: str,
        traversability: int,
        hazards: list[int] | tuple[int, ...],
        recommended_action: int,
        confidence: float,
        view_quality: float,
        observed_at: str,
    ) -> dict[str, object]:
        self._ensure_mutable()

        evidence_id = _required_text(
            "evidence_id",
            evidence_id,
        )

        hazard_codes = [_uint8("hazard", hazard) for hazard in hazards]
        traversability_code = _uint8(
            "traversability",
            traversability,
        )
        action_code = _uint8(
            "recommended_action",
            recommended_action,
        )

        record: dict[str, object] = {
            "evidence_id": evidence_id,
            "traversability_code": traversability_code,
            "traversability": _label(
                TRAVERSABILITY_LABELS,
                traversability_code,
            ),
            "hazard_codes": hazard_codes,
            "hazards": [_label(HAZARD_LABELS, code) for code in hazard_codes],
            "recommended_action_code": action_code,
            "recommended_action": _label(
                ACTION_LABELS,
                action_code,
            ),
            "confidence": _unit_interval(
                "confidence",
                confidence,
            ),
            "view_quality": _unit_interval(
                "view_quality",
                view_quality,
            ),
            "observed_at": _required_text(
                "observed_at",
                observed_at,
            ),
        }

        existing = self._assessments.get(evidence_id)

        if existing is not None:
            if existing == record:
                return dict(existing)

            raise ValueError("Conflicting assessment reused the same evidence ID")

        self._assessments[evidence_id] = record

        return dict(record)

    def record_policy_decision(
        self,
        *,
        evidence_id: str,
        action: int,
        reason: int,
        status: int,
        model_action_overridden: bool,
        source_observed_at: str,
        decided_at: str,
    ) -> dict[str, object]:
        self._ensure_mutable()

        action_code = _uint8("action", action)
        reason_code = _uint8("reason", reason)
        status_code = _uint8("status", status)

        if not isinstance(model_action_overridden, bool):
            raise ValueError("model_action_overridden must be a boolean")

        record: dict[str, object] = {
            "evidence_id": _optional_text(
                "evidence_id",
                evidence_id,
            ),
            "action_code": action_code,
            "action": _label(
                ACTION_LABELS,
                action_code,
            ),
            "reason_code": reason_code,
            "reason": _label(
                POLICY_REASON_LABELS,
                reason_code,
            ),
            "status_code": status_code,
            "status": _label(
                POLICY_STATUS_LABELS,
                status_code,
            ),
            "model_action_overridden": (model_action_overridden),
            "source_observed_at": _required_text(
                "source_observed_at",
                source_observed_at,
            ),
            "decided_at": _required_text(
                "decided_at",
                decided_at,
            ),
        }

        if self._policy_decisions and self._policy_decisions[-1] == record:
            return dict(record)

        self._policy_decisions.append(record)

        return dict(record)

    def record_mission_state(
        self,
        *,
        state: int,
        current_waypoint: str,
        active_goal: str,
        waypoint_index: int,
        waypoint_count: int,
        last_evidence_id: str,
        reason: str,
        motion_authorized: bool,
        updated_at: str,
    ) -> dict[str, object]:
        self._ensure_mutable()

        state_code = _uint8("state", state)

        for name, value in (
            ("waypoint_index", waypoint_index),
            ("waypoint_count", waypoint_count),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")

        if waypoint_index > waypoint_count:
            raise ValueError("waypoint_index cannot exceed waypoint_count")

        if not isinstance(motion_authorized, bool):
            raise ValueError("motion_authorized must be a boolean")

        record: dict[str, object] = {
            "state_code": state_code,
            "state": _label(
                MISSION_STATE_LABELS,
                state_code,
            ),
            "current_waypoint": _optional_text(
                "current_waypoint",
                current_waypoint,
            ),
            "active_goal": _optional_text(
                "active_goal",
                active_goal,
            ),
            "waypoint_index": waypoint_index,
            "waypoint_count": waypoint_count,
            "last_evidence_id": _optional_text(
                "last_evidence_id",
                last_evidence_id,
            ),
            "reason": _optional_text(
                "reason",
                reason,
            ),
            "motion_authorized": motion_authorized,
            "updated_at": _required_text(
                "updated_at",
                updated_at,
            ),
        }

        if self._mission_states and self._mission_states[-1] == record:
            return dict(record)

        self._mission_states.append(record)

        return dict(record)

    def verify_evidence_files(self) -> None:
        for record in self._evidence.values():
            image_path = self.mission_directory / str(record["image_path"])

            try:
                payload = image_path.read_bytes()
            except OSError as error:
                raise RuntimeError(f"Evidence file cannot be read: {image_path}") from error

            if len(payload) != record["byte_count"]:
                raise RuntimeError(
                    f"Evidence byte count changed after capture: {record['evidence_id']}"
                )

            if sha256(payload).hexdigest() != record["sha256"]:
                raise RuntimeError(
                    f"Evidence checksum changed after capture: {record['evidence_id']}"
                )

    def write_partial_report(self) -> Path:
        self._ensure_mutable()

        payload = self._build_report(
            outcome="recording",
            finalized_at=None,
            integrity_verified=False,
        )
        _atomic_replace_text(
            self.partial_report_path,
            _json_text(payload),
        )

        return self.partial_report_path

    def finalize(
        self,
        *,
        outcome: str,
        finalized_at: str,
    ) -> ReportArtifacts:
        if self._finalized:
            if self._artifacts is None:
                raise RuntimeError("Finalized recorder has no report artifacts")

            return self._artifacts

        outcome = _required_text("outcome", outcome)
        finalized_at = _required_text(
            "finalized_at",
            finalized_at,
        )

        self.verify_evidence_files()

        payload = self._build_report(
            outcome=outcome,
            finalized_at=finalized_at,
            integrity_verified=True,
        )

        _write_immutable_text(
            self.report_json_path,
            _json_text(payload),
        )
        _write_immutable_text(
            self.report_markdown_path,
            _markdown_text(payload),
        )

        self.partial_report_path.unlink(
            missing_ok=True,
        )

        self.report_json_path.chmod(0o444)
        self.report_markdown_path.chmod(0o444)

        self._artifacts = ReportArtifacts(
            json_path=self.report_json_path,
            markdown_path=self.report_markdown_path,
        )
        self._finalized = True

        return self._artifacts

    def _build_report(
        self,
        *,
        outcome: str,
        finalized_at: str | None,
        integrity_verified: bool,
    ) -> dict[str, object]:
        evidence_ids = set(self._evidence)
        assessment_ids = set(self._assessments)

        latest_state = self._mission_states[-1] if self._mission_states else None

        coverage = 0.0

        if latest_state is not None:
            waypoint_count = int(latest_state["waypoint_count"])
            waypoint_index = int(latest_state["waypoint_index"])

            if waypoint_count > 0:
                coverage = (
                    min(
                        waypoint_index,
                        waypoint_count,
                    )
                    / waypoint_count
                )

        return {
            "schema_version": 1,
            "mission_id": self.mission_id,
            "started_at": self.started_at,
            "finalized_at": finalized_at,
            "outcome": outcome,
            "coverage": coverage,
            "counts": {
                "evidence": len(self._evidence),
                "assessments": len(self._assessments),
                "policy_decisions": len(self._policy_decisions),
                "mission_states": len(self._mission_states),
            },
            "integrity": {
                "algorithm": "sha256",
                "verified": integrity_verified,
                "verified_evidence_files": (len(self._evidence) if integrity_verified else 0),
            },
            "unmatched": {
                "evidence_without_assessment": sorted(evidence_ids - assessment_ids),
                "assessment_without_evidence": sorted(assessment_ids - evidence_ids),
            },
            "evidence": [dict(record) for record in self._evidence.values()],
            "assessments": [dict(record) for record in self._assessments.values()],
            "policy_decisions": [dict(record) for record in self._policy_decisions],
            "mission_states": [dict(record) for record in self._mission_states],
        }

    def _ensure_mutable(self) -> None:
        if self._finalized:
            raise RuntimeError("Finalized mission reports are immutable")


def _required_text(
    name: str,
    value: str,
) -> str:
    normalized = _optional_text(name, value)

    if not normalized:
        raise ValueError(f"{name} cannot be empty")

    return normalized


def _optional_text(
    name: str,
    value: str,
) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{name} must be a string")

    if "\x00" in value:
        raise ValueError(f"{name} cannot contain null characters")

    return value.strip()


def _uint8(
    name: str,
    value: int,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255:
        raise ValueError(f"{name} must be an unsigned 8-bit integer")

    return value


def _unit_interval(
    name: str,
    value: float,
) -> float:
    if isinstance(value, bool) or not isinstance(
        value,
        int | float,
    ):
        raise ValueError(f"{name} must be numeric")

    normalized = float(value)

    if not math.isfinite(normalized) or not 0.0 <= normalized <= 1.0:
        raise ValueError(f"{name} must be finite and between 0 and 1")

    return normalized


def _label(
    labels: dict[int, str],
    value: int,
) -> str:
    return labels.get(value, f"unknown_{value}")


def _image_suffix(
    *,
    image_format: str,
    data: bytes,
    max_image_bytes: int,
) -> str:
    if not data:
        raise ValueError("Evidence image cannot be empty")

    if len(data) > max_image_bytes:
        raise ValueError("Evidence image exceeds the configured size limit")

    normalized = image_format.lower()

    if "jpeg" in normalized or "jpg" in normalized:
        if not data.startswith(b"\xff\xd8\xff"):
            raise ValueError("Evidence image has an invalid JPEG signature")

        return ".jpg"

    if "png" in normalized:
        if not data.startswith(b"\x89PNG\r\n\x1a\n"):
            raise ValueError("Evidence image has an invalid PNG signature")

        return ".png"

    raise ValueError(f"Unsupported evidence image format: {image_format}")


def _evidence_filename(
    evidence_id: str,
    suffix: str,
) -> str:
    safe_identifier = re.sub(
        r"[^A-Za-z0-9_.-]+",
        "_",
        evidence_id,
    ).strip("._-")

    if not safe_identifier:
        safe_identifier = "evidence"

    identifier_digest = sha256(evidence_id.encode("utf-8")).hexdigest()[:12]

    return f"{safe_identifier[:80]}-{identifier_digest}{suffix}"


def _json_text(
    payload: dict[str, object],
) -> str:
    return (
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    )


def _markdown_text(
    payload: dict[str, object],
) -> str:
    counts = payload["counts"]
    integrity = payload["integrity"]

    lines = [
        "# Inspectron Mission Report",
        "",
        f"- Mission ID: `{payload['mission_id']}`",
        f"- Outcome: `{payload['outcome']}`",
        f"- Started: `{payload['started_at']}`",
        f"- Finalized: `{payload['finalized_at']}`",
        f"- Coverage: `{float(payload['coverage']):.3f}`",
        (f"- Evidence integrity: `{integrity['verified_evidence_files']}` SHA-256 verified files"),
        "",
        "## Summary",
        "",
        "| Evidence | Assessments | Policy decisions | Mission states |",
        "|---:|---:|---:|---:|",
        (
            f"| {counts['evidence']} "
            f"| {counts['assessments']} "
            f"| {counts['policy_decisions']} "
            f"| {counts['mission_states']} |"
        ),
        "",
        "## Evidence",
        "",
        "| Evidence ID | Waypoint | Bytes | SHA-256 | File |",
        "|---|---|---:|---|---|",
    ]

    evidence = payload["evidence"]

    if evidence:
        for record in evidence:
            lines.append(
                "| "
                + _markdown_cell(record["evidence_id"])
                + " | "
                + _markdown_cell(record["waypoint"])
                + " | "
                + str(record["byte_count"])
                + " | `"
                + str(record["sha256"])
                + "` | [image]("
                + str(record["image_path"])
                + ") |"
            )
    else:
        lines.append("| None |  | 0 |  |  |")

    lines.extend(
        [
            "",
            "## Assessments",
            "",
            (
                "| Evidence ID | Traversability | Hazards "
                "| Model action | Confidence | View quality |"
            ),
            "|---|---|---|---|---:|---:|",
        ]
    )

    assessments = payload["assessments"]

    if assessments:
        for record in assessments:
            hazards = ", ".join(record["hazards"]) or "none"
            lines.append(
                "| "
                + _markdown_cell(record["evidence_id"])
                + " | "
                + _markdown_cell(record["traversability"])
                + " | "
                + _markdown_cell(hazards)
                + " | "
                + _markdown_cell(record["recommended_action"])
                + " | "
                + f"{float(record['confidence']):.3f}"
                + " | "
                + f"{float(record['view_quality']):.3f}"
                + " |"
            )
    else:
        lines.append("| None |  |  |  |  |  |")

    lines.extend(
        [
            "",
            "## Policy decisions",
            "",
            ("| Evidence ID | Action | Status | Reason | Overridden | Decided at |"),
            "|---|---|---|---|---|---|",
        ]
    )

    decisions = payload["policy_decisions"]

    if decisions:
        for record in decisions:
            lines.append(
                "| "
                + _markdown_cell(record["evidence_id"])
                + " | "
                + _markdown_cell(record["action"])
                + " | "
                + _markdown_cell(record["status"])
                + " | "
                + _markdown_cell(record["reason"])
                + " | "
                + str(record["model_action_overridden"])
                + " | "
                + _markdown_cell(record["decided_at"])
                + " |"
            )
    else:
        lines.append("| None |  |  |  |  |  |")

    lines.extend(
        [
            "",
            "## Mission state history",
            "",
            ("| Updated at | State | Active goal | Motion authorized | Reason |"),
            "|---|---|---|---|---|",
        ]
    )

    states = payload["mission_states"]

    if states:
        for record in states:
            lines.append(
                "| "
                + _markdown_cell(record["updated_at"])
                + " | "
                + _markdown_cell(record["state"])
                + " | "
                + _markdown_cell(record["active_goal"])
                + " | "
                + str(record["motion_authorized"])
                + " | "
                + _markdown_cell(record["reason"])
                + " |"
            )
    else:
        lines.append("| None |  |  |  |  |")

    unmatched = payload["unmatched"]

    lines.extend(
        [
            "",
            "## Unmatched records",
            "",
            (
                "- Evidence without assessment: "
                + _markdown_list(unmatched["evidence_without_assessment"])
            ),
            (
                "- Assessment without evidence: "
                + _markdown_list(unmatched["assessment_without_evidence"])
            ),
            "",
        ]
    )

    return "\n".join(lines)


def _markdown_cell(value: object) -> str:
    return str(value).replace("|", r"\|").replace("\n", " ")


def _markdown_list(values: list[str]) -> str:
    if not values:
        return "none"

    return ", ".join(f"`{value}`" for value in values)


def _atomic_replace_text(
    path: Path,
    content: str,
) -> None:
    temporary = _write_temporary(path, content)

    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _write_immutable_text(
    path: Path,
    content: str,
) -> None:
    if path.exists():
        if path.read_text(encoding="utf-8") == content:
            return

        raise FileExistsError(f"Immutable report already exists: {path}")

    temporary = _write_temporary(path, content)

    try:
        try:
            os.link(temporary, path)
        except FileExistsError:
            if path.read_text(encoding="utf-8") != content:
                raise
    finally:
        temporary.unlink(missing_ok=True)


def _write_temporary(
    path: Path,
    content: str,
) -> Path:
    temporary = path.with_name(f".{path.name}.{secrets.token_hex(8)}.tmp")

    with temporary.open(
        "x",
        encoding="utf-8",
        newline="\n",
    ) as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())

    return temporary

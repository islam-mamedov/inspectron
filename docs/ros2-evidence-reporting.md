# ROS 2 Evidence Capture and Mission Reporting

The ROS 2 evidence layer contains two packages:

- `inspectron_evidence_msgs` defines evidence-capture and report-status
  interfaces.
- `inspectron_evidence_reporter` stores accepted camera evidence and produces
  final mission reports.

The perception bridge publishes each accepted compressed image with the same
evidence ID used by its scene assessment. The reporter correlates that capture
with assessments, safety-policy decisions, and mission-state history.

## Input topics

| Topic | Type |
|---|---|
| `/inspectron/evidence_capture` | `inspectron_evidence_msgs/msg/EvidenceCapture` |
| `/inspectron/scene_assessment` | `inspectron_safety_supervisor/msg/SceneAssessment` |
| `/inspectron/policy_decision` | `inspectron_safety_supervisor/msg/PolicyDecision` |
| `/inspectron/mission/state` | `inspectron_mission_msgs/msg/MissionState` |

The mission-state subscription uses transient-local durability so the reporter
can receive the latest state when it starts.

## Output topic

| Topic | Type |
|---|---|
| `/inspectron/report/status` | `inspectron_evidence_msgs/msg/ReportStatus` |

The status contains the mission ID, report path, evidence count, status message,
and update time.

| Status | Meaning |
|---|---|
| `STATUS_IDLE` | No mission report is active |
| `STATUS_RECORDING` | Mission events are being recorded |
| `STATUS_FINALIZED` | Immutable final artifacts were written |
| `STATUS_FAILED` | Recording or finalization encountered an error |

## Manual finalization

The `/inspectron/report/finalize` service uses `std_srvs/srv/Trigger`.

It immediately finalizes the active report. If the report was already
finalized, the service returns its existing path. The request fails when no
mission report exists.

## Report lifecycle

Recording starts when the reporter receives an active mission state, including
waiting, moving, paused, closer-inspection, rerouting, or safety-stopped states.

While a mission is active, the reporter records:

- accepted compressed evidence images and their metadata;
- scene assessments;
- safety-policy decisions;
- mission-state transitions.

When partial reports are enabled, `report.partial.json` is atomically refreshed
after each successfully recorded event.

Completed, aborted, and emergency-stopped missions are automatically finalized
after the configured delay. Returning an active report to idle finalizes it
with a `reset` outcome. A safety-stopped mission remains open because it can
still be explicitly resumed or reset.

The delay allows assessment or policy messages already in transit to reach the
report before it becomes immutable.

## Report contents

Each mission is stored beneath the configured output directory:

```text
<output_directory>/<mission_id>/
├── evidence/
│   └── <safe-evidence-id>-<id-digest>.jpg
├── report.partial.json
├── report.json
└── report.md
```

PNG evidence uses the `.png` suffix. The partial report exists only during
recording and is removed after successful finalization.

The JSON report includes:

- schema version, mission ID, timestamps, outcome, and waypoint coverage;
- counts for evidence, assessments, policy decisions, and mission states;
- evidence paths, byte counts, metadata, and SHA-256 checksums;
- readable assessment and policy labels alongside their numeric codes;
- complete mission-state history;
- unmatched evidence and assessment identifiers;
- final evidence-integrity verification results.

The Markdown report provides a human-readable summary with links to stored
evidence images.

## Integrity and error handling

The recorder accepts JPEG and PNG evidence within the configured size limit and
validates each image signature before storage.

Evidence files are created without overwriting existing files and made
read-only after capture. Repeating identical evidence or assessment data is
idempotent. Conflicting data that reuses an evidence ID is rejected.

Before finalization, every image is checked against its recorded byte count and
SHA-256 digest. Missing or modified evidence blocks finalization. Final JSON and
Markdown artifacts are also created without overwriting existing files and are
made read-only.

Invalid events publish `STATUS_FAILED`. Recording can continue so later valid
events are not lost. Unmatched evidence and assessments are listed rather than
silently omitted.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `output_directory` | `~/.local/share/inspectron/reports` | Mission artifact root |
| `max_image_bytes` | `12582912` | Maximum evidence-image size |
| `auto_finalize` | `true` | Finalize from terminal mission states |
| `write_partial_report` | `true` | Maintain an atomic recording report |
| `finalize_delay_ms` | `250` | Delay before automatic finalization |

Empty output paths, non-positive image limits, and negative finalization delays
cause startup to fail.

## Validation

The implementation includes:

- mission-ID and timestamp tests;
- complete JSON, Markdown, and image-output tests;
- duplicate and conflicting evidence-ID tests;
- invalid image and numeric-field tests;
- checksum-tampering tests that block finalization;
- unmatched evidence and assessment tests;
- ROS graph integration that finalizes a complete mission;
- perception integration that verifies matching evidence captures.

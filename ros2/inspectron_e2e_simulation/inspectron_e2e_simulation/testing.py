from __future__ import annotations

import json
import time
from hashlib import sha256
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from inspectron_evidence_msgs.msg import EvidenceCapture, ReportStatus
from inspectron_mission_msgs.msg import MissionState
from inspectron_safety_supervisor.msg import PolicyDecision, SceneAssessment
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from inspectron_e2e_simulation.pipeline import remapped_name
from inspectron_e2e_simulation.scenario_logic import sanitize_waypoint

_VELOCITY_EPSILON = 1e-9


def is_authorizing(policy: PolicyDecision) -> bool:
    return policy.status == PolicyDecision.STATUS_VALID and policy.action in (
        PolicyDecision.ACTION_PROCEED,
        PolicyDecision.ACTION_SLOW_DOWN,
    )


def is_nonzero_twist(twist: Twist) -> bool:
    return abs(twist.linear.x) > _VELOCITY_EPSILON or abs(twist.angular.z) > _VELOCITY_EPSILON


class PipelineTestHarness:
    def __init__(self, node_name: str, prefix: str) -> None:
        self.node = rclpy.create_node(node_name)
        self.prefix = prefix

        self.states: list[MissionState] = []
        self.policies: list[PolicyDecision] = []
        self.assessments: list[SceneAssessment] = []
        self.evidence: list[EvidenceCapture] = []
        self.cmd_vels: list[Twist] = []
        self.report_statuses: list[ReportStatus] = []
        self.event_log: list[tuple[str, object]] = []

        transient = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.state_subscription = self.node.create_subscription(
            MissionState,
            self._name("/inspectron/mission/state"),
            self._on_state,
            transient,
        )
        self.policy_subscription = self.node.create_subscription(
            PolicyDecision,
            self._name("/inspectron/policy_decision"),
            self._on_policy,
            transient,
        )
        self.report_subscription = self.node.create_subscription(
            ReportStatus,
            self._name("/inspectron/report/status"),
            self._on_report_status,
            transient,
        )
        self.assessment_subscription = self.node.create_subscription(
            SceneAssessment,
            self._name("/inspectron/scene_assessment"),
            self._on_assessment,
            volatile,
        )
        self.evidence_subscription = self.node.create_subscription(
            EvidenceCapture,
            self._name("/inspectron/evidence_capture"),
            self._on_evidence,
            volatile,
        )
        self.cmd_vel_subscription = self.node.create_subscription(
            Twist,
            self._name("/cmd_vel"),
            self._on_cmd_vel,
            volatile,
        )

        self.start_client = self.node.create_client(
            Trigger,
            self._name("/inspectron/mission/start"),
        )
        self.abort_client = self.node.create_client(
            Trigger,
            self._name("/inspectron/mission/abort"),
        )

    def _name(self, name: str) -> str:
        return remapped_name(name, self.prefix)

    def _on_state(self, message: MissionState) -> None:
        self.states.append(message)
        self.event_log.append(("state", message))

    def _on_policy(self, message: PolicyDecision) -> None:
        self.policies.append(message)
        self.event_log.append(("policy", message))

    def _on_report_status(self, message: ReportStatus) -> None:
        self.report_statuses.append(message)
        self.event_log.append(("report", message))

    def _on_assessment(self, message: SceneAssessment) -> None:
        self.assessments.append(message)
        self.event_log.append(("assessment", message))

    def _on_evidence(self, message: EvidenceCapture) -> None:
        self.evidence.append(message)
        self.event_log.append(("evidence", message))

    def _on_cmd_vel(self, message: Twist) -> None:
        self.cmd_vels.append(message)
        self.event_log.append(("cmd_vel", message))

    def destroy(self) -> None:
        self.node.destroy_client(self.start_client)
        self.node.destroy_client(self.abort_client)
        self.node.destroy_subscription(self.state_subscription)
        self.node.destroy_subscription(self.policy_subscription)
        self.node.destroy_subscription(self.report_subscription)
        self.node.destroy_subscription(self.assessment_subscription)
        self.node.destroy_subscription(self.evidence_subscription)
        self.node.destroy_subscription(self.cmd_vel_subscription)
        self.node.destroy_node()

    def wait_until(
        self,
        predicate,
        *,
        timeout_seconds,
        failure_message,
        test_case,
    ):
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if predicate():
                return

        test_case.fail(failure_message)

    def wait_for_graph(self, *, test_case, timeout_seconds=30.0):
        self.wait_until(
            lambda: (
                self.start_client.service_is_ready()
                and self.abort_client.service_is_ready()
                and self.state_subscription.get_publisher_count() > 0
                and self.policy_subscription.get_publisher_count() > 0
                and self.report_subscription.get_publisher_count() > 0
                and self.assessment_subscription.get_publisher_count() > 0
                and self.evidence_subscription.get_publisher_count() > 0
                and self.cmd_vel_subscription.get_publisher_count() > 0
            ),
            timeout_seconds=timeout_seconds,
            failure_message="simulation pipeline graph was not discovered",
            test_case=test_case,
        )

    def call_trigger(self, client, *, label, test_case, timeout_seconds=10.0):
        test_case.assertTrue(
            client.wait_for_service(timeout_sec=timeout_seconds),
            f"{label} service was not available",
        )

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=timeout_seconds,
        )

        test_case.assertTrue(future.done(), f"{label} call did not complete")
        response = future.result()
        test_case.assertIsNotNone(response, f"{label} returned no response")
        return response

    def first_event_index(self, predicate):
        for index, (kind, message) in enumerate(self.event_log):
            if predicate(kind, message):
                return index

        return None

    def nonzero_cmd_count(self) -> int:
        return sum(1 for twist in self.cmd_vels if is_nonzero_twist(twist))

    def goal_progression(self) -> list[str]:
        ordered: list[str] = []

        for state in self.states:
            if state.state not in (
                MissionState.STATE_WAITING_FOR_POLICY,
                MissionState.STATE_MOVING,
            ):
                continue

            if state.active_goal and state.active_goal not in ordered:
                ordered.append(state.active_goal)

        return ordered


def assert_read_only(test_case, path: Path) -> None:
    mode = path.stat().st_mode
    test_case.assertEqual(mode & 0o222, 0, f"{path} is writable")


def load_report(test_case, report_root: Path):
    mission_dirs = [path for path in report_root.iterdir() if path.is_dir()]
    test_case.assertEqual(
        len(mission_dirs),
        1,
        f"expected exactly one mission directory in {report_root}",
    )

    mission_dir = mission_dirs[0]
    report_json = mission_dir / "report.json"
    report_md = mission_dir / "report.md"
    partial = mission_dir / "report.partial.json"

    test_case.assertTrue(report_json.is_file(), "report.json is missing")
    test_case.assertTrue(report_md.is_file(), "report.md is missing")
    test_case.assertFalse(
        partial.exists(),
        "report.partial.json still exists after finalization",
    )
    assert_read_only(test_case, report_json)
    assert_read_only(test_case, report_md)

    payload = json.loads(report_json.read_text(encoding="utf-8"))
    test_case.assertEqual(payload["mission_id"], mission_dir.name)
    return payload, mission_dir


def verify_report_integrity(test_case, payload, mission_dir: Path) -> None:
    integrity = payload["integrity"]
    counts = payload["counts"]
    evidence_records = payload["evidence"]

    test_case.assertEqual(integrity["algorithm"], "sha256")
    test_case.assertTrue(integrity["verified"], "report integrity is not verified")
    test_case.assertEqual(
        integrity["verified_evidence_files"],
        counts["evidence"],
    )
    test_case.assertEqual(len(evidence_records), counts["evidence"])
    test_case.assertEqual(
        payload["unmatched"]["evidence_without_assessment"],
        [],
    )
    test_case.assertEqual(
        payload["unmatched"]["assessment_without_evidence"],
        [],
    )

    for record in evidence_records:
        image_path = mission_dir / str(record["image_path"])
        test_case.assertTrue(
            image_path.is_file(),
            f"evidence file missing: {image_path}",
        )
        assert_read_only(test_case, image_path)

        data = image_path.read_bytes()
        test_case.assertEqual(len(data), record["byte_count"])
        test_case.assertEqual(sha256(data).hexdigest(), record["sha256"])
        test_case.assertTrue(
            str(record["evidence_id"]).startswith(f"{sanitize_waypoint(str(record['waypoint']))}-"),
            "evidence identifier does not carry its waypoint prefix",
        )

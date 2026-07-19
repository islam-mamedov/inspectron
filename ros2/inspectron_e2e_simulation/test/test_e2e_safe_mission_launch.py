from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import mkdtemp

import launch
import launch_testing.actions
import launch_testing.asserts
import rclpy
from inspectron_e2e_simulation.pipeline import create_pipeline_nodes
from inspectron_e2e_simulation.scenarios import (
    SAFE_MISSION_SCENARIO,
    scenario_definition,
)
from inspectron_e2e_simulation.testing import (
    PipelineTestHarness,
    is_authorizing,
    is_nonzero_twist,
    load_report,
    verify_report_integrity,
)
from inspectron_evidence_msgs.msg import ReportStatus
from inspectron_mission_msgs.msg import MissionState
from inspectron_safety_supervisor.msg import PolicyDecision

PREFIX = "/test/e2e_safe"
REPORT_ROOT = Path(mkdtemp(prefix="inspectron-e2e-safe-"))
WAYPOINTS = ("aisle_a", "aisle_b", "aisle_c")


def generate_test_description():
    scenario = scenario_definition(SAFE_MISSION_SCENARIO)

    nodes = create_pipeline_nodes(
        waypoints=list(scenario.waypoints),
        fixture_response_json=scenario.fixture_response_json,
        report_directory=str(REPORT_ROOT),
        scene_id=scenario.scene_id,
        desired_velocity_mode=scenario.desired_velocity_mode,
        auto_start=False,
        abort_on_safety_stop=False,
        prefix=PREFIX,
        node_name_suffix="e2e_safe",
    )

    return launch.LaunchDescription(
        [
            *nodes,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class SafeMissionEndToEndTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = PipelineTestHarness("e2e_safe_mission_test", PREFIX)
        self.harness.wait_for_graph(test_case=self)

    def tearDown(self):
        self.harness.destroy()

    def test_safe_mission_completes_with_verified_report(self):
        harness = self.harness

        harness.wait_until(
            lambda: any(policy.evidence_id.startswith("aisle_a-") for policy in harness.policies),
            timeout_seconds=30.0,
            failure_message="priming decision never flowed through the pipeline",
            test_case=self,
        )

        harness.wait_until(
            lambda: len(harness.cmd_vels) >= 3,
            timeout_seconds=15.0,
            failure_message="motion controller published no cmd_vel before start",
            test_case=self,
        )
        self.assertEqual(
            harness.nonzero_cmd_count(),
            0,
            "cmd_vel was non-zero before the mission started",
        )

        response = harness.call_trigger(
            harness.start_client,
            label="mission start",
            test_case=self,
        )
        self.assertTrue(response.success, f"mission start rejected: {response.message}")

        harness.wait_until(
            lambda: any(state.state == MissionState.STATE_COMPLETED for state in harness.states),
            timeout_seconds=90.0,
            failure_message="mission never reached COMPLETED",
            test_case=self,
        )

        completed = [
            state for state in harness.states if state.state == MissionState.STATE_COMPLETED
        ][-1]
        self.assertEqual(completed.waypoint_index, len(WAYPOINTS))
        self.assertEqual(completed.waypoint_count, len(WAYPOINTS))
        self.assertFalse(completed.motion_authorized)

        self.assertEqual(harness.goal_progression(), list(WAYPOINTS))

        for state in harness.states:
            self.assertNotIn(
                state.state,
                (
                    MissionState.STATE_SAFETY_STOPPED,
                    MissionState.STATE_ABORTED,
                    MissionState.STATE_EMERGENCY_STOPPED,
                ),
                f"unsafe mission state observed: {state.state} ({state.reason})",
            )

        self.assertGreater(
            harness.nonzero_cmd_count(),
            0,
            "authorized motion never produced a non-zero cmd_vel",
        )

        first_nonzero = harness.first_event_index(
            lambda kind, message: kind == "cmd_vel" and is_nonzero_twist(message)
        )
        first_authorizing_policy = harness.first_event_index(
            lambda kind, message: kind == "policy" and is_authorizing(message)
        )
        first_authorized_state = harness.first_event_index(
            lambda kind, message: kind == "state" and message.motion_authorized
        )
        self.assertIsNotNone(first_nonzero)
        self.assertIsNotNone(first_authorizing_policy)
        self.assertIsNotNone(first_authorized_state)
        self.assertLess(
            first_authorizing_policy,
            first_nonzero,
            "non-zero cmd_vel appeared before any valid authorizing policy",
        )
        self.assertLess(
            first_authorized_state,
            first_nonzero,
            "non-zero cmd_vel appeared before the mission authorized motion",
        )

        for twist in harness.cmd_vels:
            self.assertLessEqual(abs(twist.linear.x), 0.40 + 1e-6)
            self.assertLessEqual(abs(twist.angular.z), 0.80 + 1e-6)

        harness.wait_until(
            lambda: (
                len(harness.assessments) > 0
                and {item.evidence_id for item in harness.assessments}
                == {item.evidence_id for item in harness.evidence}
            ),
            timeout_seconds=15.0,
            failure_message="assessment and evidence identifiers never converged",
            test_case=self,
        )

        for waypoint in WAYPOINTS:
            self.assertTrue(
                any(item.evidence_id.startswith(f"{waypoint}-") for item in harness.evidence),
                f"no evidence captured for waypoint {waypoint}",
            )

        for policy in harness.policies:
            if policy.evidence_id:
                self.assertEqual(policy.status, PolicyDecision.STATUS_VALID)
                self.assertEqual(policy.action, PolicyDecision.ACTION_PROCEED)

        harness.wait_until(
            lambda: (
                len(harness.cmd_vels) >= 5
                and all(not is_nonzero_twist(twist) for twist in harness.cmd_vels[-5:])
            ),
            timeout_seconds=15.0,
            failure_message="cmd_vel did not settle to zero after completion",
            test_case=self,
        )

        harness.wait_until(
            lambda: any(
                status.status == ReportStatus.STATUS_FINALIZED for status in harness.report_statuses
            ),
            timeout_seconds=30.0,
            failure_message="evidence reporter never finalized the report",
            test_case=self,
        )

        final_status = [
            status
            for status in harness.report_statuses
            if status.status == ReportStatus.STATUS_FINALIZED
        ][-1]
        self.assertIn("completed", final_status.message)
        self.assertNotIn("error", final_status.message)
        self.assertGreaterEqual(final_status.evidence_count, len(WAYPOINTS))

        payload, mission_dir = load_report(self, REPORT_ROOT)
        self.assertEqual(payload["outcome"], "completed")
        self.assertEqual(payload["coverage"], 1.0)
        self.assertEqual(final_status.mission_id, payload["mission_id"])
        self.assertEqual(
            final_status.report_path,
            str(mission_dir / "report.json"),
        )

        verify_report_integrity(self, payload, mission_dir)

        counts = payload["counts"]
        self.assertGreaterEqual(counts["evidence"], len(WAYPOINTS))
        self.assertEqual(counts["assessments"], counts["evidence"])

        recorded_waypoints = {record["waypoint"] for record in payload["evidence"]}
        self.assertEqual(recorded_waypoints, set(WAYPOINTS))

        for record in payload["assessments"]:
            self.assertEqual(record["traversability"], "clear")
            self.assertEqual(record["recommended_action"], "proceed")

        self.assertGreater(len(payload["policy_decisions"]), 0)

        for record in payload["policy_decisions"]:
            self.assertEqual(record["status"], "valid")
            self.assertEqual(record["action"], "proceed")

        state_labels = [record["state"] for record in payload["mission_states"]]
        for label in ("waiting_for_policy", "moving", "completed"):
            self.assertIn(label, state_labels)
        self.assertNotIn("safety_stopped", state_labels)

        last_state = payload["mission_states"][-1]
        self.assertEqual(last_state["state"], "completed")
        self.assertFalse(last_state["motion_authorized"])


@launch_testing.post_shutdown_test()
class SafeMissionAfterShutdownTest(unittest.TestCase):
    def test_scenario_simulator_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0],
            process="scenario_simulator_node",
        )


if __name__ == "__main__":
    unittest.main()

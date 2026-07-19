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
    HAZARD_STOP_SCENARIO,
    scenario_definition,
)
from inspectron_e2e_simulation.testing import (
    PipelineTestHarness,
    load_report,
    verify_report_integrity,
)
from inspectron_evidence_msgs.msg import ReportStatus
from inspectron_mission_msgs.msg import MissionState
from inspectron_safety_supervisor.msg import PolicyDecision

PREFIX = "/test/e2e_hazard"
REPORT_ROOT = Path(mkdtemp(prefix="inspectron-e2e-hazard-"))


def generate_test_description():
    scenario = scenario_definition(HAZARD_STOP_SCENARIO)

    nodes = create_pipeline_nodes(
        waypoints=list(scenario.waypoints),
        fixture_response_json=scenario.fixture_response_json,
        report_directory=str(REPORT_ROOT),
        scene_id=scenario.scene_id,
        desired_velocity_mode=scenario.desired_velocity_mode,
        auto_start=False,
        abort_on_safety_stop=False,
        prefix=PREFIX,
        node_name_suffix="e2e_hazard",
    )

    return launch.LaunchDescription(
        [
            *nodes,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class HazardStopEndToEndTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = PipelineTestHarness("e2e_hazard_stop_test", PREFIX)
        self.harness.wait_for_graph(test_case=self)

    def tearDown(self):
        self.harness.destroy()

    def test_critical_hazard_blocks_all_motion_and_preserves_evidence(self):
        harness = self.harness

        harness.wait_until(
            lambda: any(
                policy.evidence_id.startswith("aisle_a-")
                and policy.action == PolicyDecision.ACTION_STOP
                and policy.status == PolicyDecision.STATUS_VALID
                and policy.reason == PolicyDecision.REASON_CRITICAL_HAZARD
                for policy in harness.policies
            ),
            timeout_seconds=30.0,
            failure_message="critical-hazard stop decision never flowed through",
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
            "cmd_vel was non-zero while desired velocity streamed without authorization",
        )

        response = harness.call_trigger(
            harness.start_client,
            label="mission start",
            test_case=self,
        )
        self.assertTrue(response.success, f"mission start rejected: {response.message}")

        harness.wait_until(
            lambda: any(
                state.state == MissionState.STATE_SAFETY_STOPPED for state in harness.states
            ),
            timeout_seconds=30.0,
            failure_message="mission never entered SAFETY_STOPPED",
            test_case=self,
        )

        samples_target = len(harness.cmd_vels) + 20
        harness.wait_until(
            lambda: len(harness.cmd_vels) >= samples_target,
            timeout_seconds=15.0,
            failure_message="motion controller stopped publishing cmd_vel",
            test_case=self,
        )
        self.assertEqual(
            harness.nonzero_cmd_count(),
            0,
            "a non-zero cmd_vel escaped during the hazard scenario",
        )

        for state in harness.states:
            self.assertFalse(
                state.motion_authorized,
                f"mission authorized motion in state {state.state}",
            )
            self.assertNotEqual(
                state.state,
                MissionState.STATE_MOVING,
                "mission entered MOVING during a critical hazard",
            )

        for policy in harness.policies:
            if policy.evidence_id:
                self.assertEqual(policy.status, PolicyDecision.STATUS_VALID)
                self.assertEqual(policy.action, PolicyDecision.ACTION_STOP)
                self.assertEqual(policy.reason, PolicyDecision.REASON_CRITICAL_HAZARD)

        harness.wait_until(
            lambda: (
                len(harness.evidence) > 0
                and {item.evidence_id for item in harness.assessments}
                == {item.evidence_id for item in harness.evidence}
            ),
            timeout_seconds=15.0,
            failure_message="assessment and evidence identifiers never converged",
            test_case=self,
        )

        response = harness.call_trigger(
            harness.abort_client,
            label="mission abort",
            test_case=self,
        )
        self.assertTrue(response.success, f"mission abort rejected: {response.message}")

        harness.wait_until(
            lambda: any(state.state == MissionState.STATE_ABORTED for state in harness.states),
            timeout_seconds=15.0,
            failure_message="mission never entered ABORTED after operator abort",
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
        self.assertIn("aborted", final_status.message)
        self.assertNotIn("error", final_status.message)
        self.assertGreaterEqual(final_status.evidence_count, 1)

        payload, mission_dir = load_report(self, REPORT_ROOT)
        self.assertEqual(payload["outcome"], "aborted")
        self.assertEqual(payload["coverage"], 0.0)

        verify_report_integrity(self, payload, mission_dir)

        self.assertGreaterEqual(payload["counts"]["evidence"], 1)
        self.assertGreater(len(payload["assessments"]), 0)

        for record in payload["assessments"]:
            self.assertIn("human_in_path", record["hazards"])

        self.assertGreater(len(payload["policy_decisions"]), 0)

        for record in payload["policy_decisions"]:
            self.assertEqual(record["status"], "valid")
            self.assertEqual(record["action"], "stop")
            self.assertEqual(record["reason"], "critical_hazard")

        state_labels = [record["state"] for record in payload["mission_states"]]
        for label in ("waiting_for_policy", "safety_stopped", "aborted"):
            self.assertIn(label, state_labels)
        self.assertNotIn("moving", state_labels)

        for record in payload["mission_states"]:
            self.assertFalse(record["motion_authorized"])

        self.assertEqual(harness.nonzero_cmd_count(), 0)


@launch_testing.post_shutdown_test()
class HazardStopAfterShutdownTest(unittest.TestCase):
    def test_scenario_simulator_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0],
            process="scenario_simulator_node",
        )


if __name__ == "__main__":
    unittest.main()

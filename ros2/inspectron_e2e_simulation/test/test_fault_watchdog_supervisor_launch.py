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
    is_nonzero_twist,
    load_report,
    verify_report_integrity,
)
from inspectron_evidence_msgs.msg import ReportStatus
from inspectron_mission_msgs.msg import MissionState
from inspectron_safety_supervisor.msg import PolicyDecision

PREFIX = "/test/e2e_fault_b"
REPORT_ROOT = Path(mkdtemp(prefix="inspectron-fault-b-"))
STALL_GOAL = "aisle_b"


def generate_test_description():
    scenario = scenario_definition(SAFE_MISSION_SCENARIO)

    nodes = create_pipeline_nodes(
        waypoints=list(scenario.waypoints),
        fixture_response_json=scenario.fixture_response_json,
        report_directory=str(REPORT_ROOT),
        scene_id="e2e_fault_supervisor_staleness",
        desired_velocity_mode=scenario.desired_velocity_mode,
        auto_start=False,
        abort_on_safety_stop=False,
        prefix=PREFIX,
        node_name_suffix="e2e_fault_b",
        orchestrator_policy_timeout_ms=4000,
        supervisor_assessment_timeout_ms=1500,
        simulator_extra_parameters={"fault_camera_stall_at_goal_index": 1},
    )

    return launch.LaunchDescription(
        [
            *nodes,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class SupervisorStalenessFaultTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = PipelineTestHarness("e2e_fault_b_test", PREFIX)
        self.harness.wait_for_graph(test_case=self)

    def tearDown(self):
        self.harness.destroy()

    def test_supervisor_staleness_stops_mission_on_camera_stall(self):
        harness = self.harness

        harness.wait_until(
            lambda: any(policy.evidence_id.startswith("aisle_a-") for policy in harness.policies),
            timeout_seconds=30.0,
            failure_message="priming decision never flowed through the pipeline",
            test_case=self,
        )

        response = harness.call_trigger(
            harness.start_client,
            label="mission start",
            test_case=self,
        )
        self.assertTrue(response.success, f"mission start rejected: {response.message}")

        harness.wait_until(
            lambda: any(
                state.state == MissionState.STATE_MOVING
                and state.active_goal == STALL_GOAL
                and state.motion_authorized
                for state in harness.states
            ),
            timeout_seconds=30.0,
            failure_message="mission never started moving toward the stall goal",
            test_case=self,
        )

        harness.wait_until(
            lambda: any(
                policy.status == PolicyDecision.STATUS_STALE
                and policy.action == PolicyDecision.ACTION_STOP
                and policy.evidence_id.startswith(f"{STALL_GOAL}-")
                for policy in harness.policies
            ),
            timeout_seconds=20.0,
            failure_message="supervisor never published a stale stop decision",
            test_case=self,
        )

        harness.wait_until(
            lambda: any(
                state.state == MissionState.STATE_SAFETY_STOPPED for state in harness.states
            ),
            timeout_seconds=15.0,
            failure_message="mission never SAFETY_STOPPED after the stale decision",
            test_case=self,
        )

        stopped = [
            state for state in harness.states if state.state == MissionState.STATE_SAFETY_STOPPED
        ][0]
        self.assertIn(
            "invalid or stale",
            stopped.reason,
            f"unexpected stop reason: {stopped.reason}",
        )
        self.assertEqual(stopped.active_goal, STALL_GOAL)

        harness.wait_until(
            lambda: (
                len(harness.cmd_vels) >= 5
                and all(not is_nonzero_twist(twist) for twist in harness.cmd_vels[-5:])
            ),
            timeout_seconds=15.0,
            failure_message="cmd_vel did not settle to zero after the safety stop",
            test_case=self,
        )

        stop_index = next(
            index
            for index, state in enumerate(harness.states)
            if state.state == MissionState.STATE_SAFETY_STOPPED
        )

        for state in harness.states[stop_index:]:
            self.assertFalse(
                state.motion_authorized,
                "motion was authorized after the safety stop",
            )
            self.assertNotEqual(
                state.state,
                MissionState.STATE_MOVING,
                "mission re-entered MOVING after the safety stop",
            )

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

        payload, mission_dir = load_report(self, REPORT_ROOT)
        self.assertEqual(payload["outcome"], "aborted")
        self.assertAlmostEqual(payload["coverage"], 1.0 / 3.0, places=3)

        verify_report_integrity(self, payload, mission_dir)

        stale_records = [
            record for record in payload["policy_decisions"] if record["status"] == "stale"
        ]
        self.assertGreater(
            len(stale_records),
            0,
            "no stale policy decision was preserved in the report",
        )

        for record in stale_records:
            self.assertEqual(record["action"], "stop")

        stop_records = [
            record for record in payload["mission_states"] if record["state"] == "safety_stopped"
        ]
        self.assertGreater(len(stop_records), 0)
        self.assertIn("invalid or stale", stop_records[0]["reason"])

        last_state = payload["mission_states"][-1]
        self.assertEqual(last_state["state"], "aborted")
        self.assertFalse(last_state["motion_authorized"])


@launch_testing.post_shutdown_test()
class SupervisorStalenessAfterShutdownTest(unittest.TestCase):
    def test_scenario_simulator_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0],
            process="scenario_simulator_node",
        )


if __name__ == "__main__":
    unittest.main()

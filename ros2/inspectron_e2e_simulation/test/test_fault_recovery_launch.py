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

PREFIX = "/test/e2e_fault_c"
REPORT_ROOT = Path(mkdtemp(prefix="inspectron-fault-c-"))
WAYPOINTS = ("aisle_a", "aisle_b", "aisle_c")
STALL_GOAL = "aisle_b"
MALFORMED_GOAL = "aisle_c"


def generate_test_description():
    scenario = scenario_definition(SAFE_MISSION_SCENARIO)

    nodes = create_pipeline_nodes(
        waypoints=list(scenario.waypoints),
        fixture_response_json=scenario.fixture_response_json,
        report_directory=str(REPORT_ROOT),
        scene_id="e2e_fault_recovery",
        desired_velocity_mode=scenario.desired_velocity_mode,
        auto_start=False,
        abort_on_safety_stop=False,
        prefix=PREFIX,
        node_name_suffix="e2e_fault_c",
        simulator_extra_parameters={
            "fault_desired_stall_at_goal_index": 1,
            "fault_desired_stall_ticks": 30,
            "fault_malformed_frame_at_goal_index": 2,
        },
    )

    return launch.LaunchDescription(
        [
            *nodes,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class RecoveryFaultTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = PipelineTestHarness("e2e_fault_c_test", PREFIX)
        self.harness.wait_for_graph(test_case=self)

    def tearDown(self):
        self.harness.destroy()

    def _stall_goal_motion_runs(self):
        current_state = None
        profile = []

        for kind, message in self.harness.event_log:
            if kind == "state":
                current_state = message
            elif kind == "cmd_vel" and (
                current_state is not None
                and current_state.state == MissionState.STATE_MOVING
                and current_state.active_goal == STALL_GOAL
                and current_state.motion_authorized
            ):
                profile.append(is_nonzero_twist(message))

        runs: list[list] = []

        for sample in profile:
            if runs and runs[-1][0] == sample:
                runs[-1][1] += 1
            else:
                runs.append([sample, 1])

        return runs

    def test_mission_recovers_from_command_stall_and_malformed_frame(self):
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
            lambda: any(state.state == MissionState.STATE_COMPLETED for state in harness.states),
            timeout_seconds=90.0,
            failure_message="mission never reached COMPLETED despite recoverable faults",
            test_case=self,
        )

        for state in harness.states:
            self.assertNotIn(
                state.state,
                (
                    MissionState.STATE_SAFETY_STOPPED,
                    MissionState.STATE_ABORTED,
                    MissionState.STATE_EMERGENCY_STOPPED,
                ),
                f"recoverable fault escalated to an unsafe state ({state.reason})",
            )

        completed = [
            state for state in harness.states if state.state == MissionState.STATE_COMPLETED
        ][-1]
        self.assertEqual(completed.waypoint_index, len(WAYPOINTS))

        runs = self._stall_goal_motion_runs()
        sandwich = any(
            runs[index][0] is True
            and runs[index + 1][0] is False
            and runs[index + 1][1] >= 10
            and runs[index + 2][0] is True
            for index in range(len(runs) - 2)
        )
        self.assertTrue(
            sandwich,
            "no motion/zero/motion sandwich during the desired-velocity stall; "
            f"observed runs: {runs}",
        )

        self.assertTrue(
            any(
                state.state == MissionState.STATE_INSPECTING_CLOSER
                and state.active_goal == MALFORMED_GOAL
                for state in harness.states
            ),
            "mission never entered INSPECTING_CLOSER for the malformed frame",
        )

        inspect_policies = [
            policy
            for policy in harness.policies
            if policy.action == PolicyDecision.ACTION_INSPECT_CLOSER
        ]
        self.assertGreater(
            len(inspect_policies),
            0,
            "supervisor never demanded closer inspection",
        )

        for policy in inspect_policies:
            self.assertEqual(policy.status, PolicyDecision.STATUS_VALID)
            self.assertEqual(policy.reason, PolicyDecision.REASON_WEAK_EVIDENCE)
            self.assertTrue(policy.evidence_id.startswith(f"{MALFORMED_GOAL}-"))

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

        payload, mission_dir = load_report(self, REPORT_ROOT)
        self.assertEqual(payload["outcome"], "completed")
        self.assertEqual(payload["coverage"], 1.0)

        verify_report_integrity(
            self,
            payload,
            mission_dir,
            allowed_assessments_without_evidence=1,
        )

        orphaned = payload["unmatched"]["assessment_without_evidence"]
        self.assertTrue(
            orphaned[0].startswith(f"{MALFORMED_GOAL}-"),
            f"unexpected orphaned assessment: {orphaned}",
        )

        zero_confidence = [
            record for record in payload["assessments"] if record["confidence"] == 0.0
        ]
        self.assertEqual(len(zero_confidence), 1)
        self.assertEqual(zero_confidence[0]["recommended_action"], "inspect_closer")

        for record in payload["policy_decisions"]:
            self.assertNotEqual(record["action"], "stop")
            self.assertNotEqual(record["status"], "stale")

        inspect_records = [
            record for record in payload["policy_decisions"] if record["action"] == "inspect_closer"
        ]
        self.assertGreater(len(inspect_records), 0)

        for record in inspect_records:
            self.assertEqual(record["reason"], "weak_evidence")
            self.assertEqual(record["status"], "valid")

        state_labels = [record["state"] for record in payload["mission_states"]]

        for label in ("waiting_for_policy", "moving", "inspecting_closer", "completed"):
            self.assertIn(label, state_labels)

        self.assertNotIn("safety_stopped", state_labels)

        last_state = payload["mission_states"][-1]
        self.assertEqual(last_state["state"], "completed")
        self.assertFalse(last_state["motion_authorized"])


@launch_testing.post_shutdown_test()
class RecoveryFaultAfterShutdownTest(unittest.TestCase):
    def test_scenario_simulator_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0],
            process="scenario_simulator_node",
        )


if __name__ == "__main__":
    unittest.main()

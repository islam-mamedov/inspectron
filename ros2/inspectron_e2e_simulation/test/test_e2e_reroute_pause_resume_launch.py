from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import mkdtemp

import launch
import launch_testing.actions
import launch_testing.asserts
import rclpy
from inspectron_e2e_simulation.pipeline import create_pipeline_nodes
from inspectron_e2e_simulation.scenarios import SAFE_FIXTURE_RESPONSE
from inspectron_e2e_simulation.testing import (
    PipelineTestHarness,
    is_authorizing,
    is_nonzero_twist,
    load_report,
    verify_report_integrity,
)
from inspectron_evidence_msgs.msg import ReportStatus
from inspectron_mission_msgs.msg import MissionState

PREFIX = "/test/e2e_reroute_pause_resume"
REPORT_ROOT = Path(mkdtemp(prefix="inspectron-e2e-reroute-pause-"))
WAYPOINTS = ("aisle_a", "aisle_b", "aisle_c")
BLOCKED_WAYPOINT = "aisle_b"
REROUTE_TARGET = "aisle_b_detour"

BLOCKED_FIXTURE_RESPONSE = {
    "traversability": "blocked",
    "hazards": ["debris"],
    "recommended_action": "reroute",
    "confidence": 0.94,
    "view_quality": 0.91,
}


def generate_test_description():
    nodes = create_pipeline_nodes(
        waypoints=list(WAYPOINTS),
        fixture_response_json=json.dumps(SAFE_FIXTURE_RESPONSE),
        fixture_responses_by_waypoint_json=json.dumps({BLOCKED_WAYPOINT: BLOCKED_FIXTURE_RESPONSE}),
        report_directory=str(REPORT_ROOT),
        scene_id="e2e_reroute_pause_resume",
        desired_velocity_mode="authorized_only",
        auto_start=False,
        abort_on_safety_stop=False,
        prefix=PREFIX,
        node_name_suffix="e2e_reroute_pause_resume",
        simulator_extra_parameters={
            "min_authorized_motion_samples": 50,
        },
    )

    return launch.LaunchDescription(
        [
            *nodes,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class ReroutePauseResumeEndToEndTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = PipelineTestHarness(
            "e2e_reroute_pause_resume_test",
            PREFIX,
        )
        self.harness.wait_for_graph(test_case=self)

    def tearDown(self):
        self.harness.destroy()

    def test_pause_requires_fresh_policy_then_blocked_goal_reroutes(self):
        harness = self.harness

        harness.wait_until(
            lambda: any(policy.evidence_id.startswith("aisle_a-") for policy in harness.policies),
            timeout_seconds=30.0,
            failure_message="priming decision never flowed through the pipeline",
            test_case=self,
        )

        start_response = harness.call_trigger(
            harness.start_client,
            label="mission start",
            test_case=self,
        )
        self.assertTrue(
            start_response.success,
            f"mission start rejected: {start_response.message}",
        )

        harness.wait_until(
            lambda: (
                any(
                    state.state == MissionState.STATE_MOVING
                    and state.active_goal == "aisle_a"
                    and state.motion_authorized
                    for state in harness.states
                )
                and harness.nonzero_cmd_count() > 0
            ),
            timeout_seconds=30.0,
            failure_message="mission never began authorized motion toward aisle_a",
            test_case=self,
        )

        pause_state_marker = len(harness.states)
        pause_response = harness.call_trigger(
            harness.pause_client,
            label="mission pause",
            test_case=self,
        )
        self.assertTrue(
            pause_response.success,
            f"mission pause rejected: {pause_response.message}",
        )

        harness.wait_until(
            lambda: any(
                state.state == MissionState.STATE_PAUSED
                for state in harness.states[pause_state_marker:]
            ),
            timeout_seconds=10.0,
            failure_message="mission never entered PAUSED",
            test_case=self,
        )
        paused_state = next(
            state
            for state in harness.states[pause_state_marker:]
            if state.state == MissionState.STATE_PAUSED
        )
        self.assertFalse(paused_state.motion_authorized)

        paused_cmd_marker = len(harness.cmd_vels)
        harness.wait_until(
            lambda: (
                len(harness.cmd_vels) >= paused_cmd_marker + 5
                and all(not is_nonzero_twist(twist) for twist in harness.cmd_vels[-5:])
            ),
            timeout_seconds=10.0,
            failure_message="cmd_vel did not remain zero while paused",
            test_case=self,
        )

        resume_state_marker = len(harness.states)
        resume_policy_marker = len(harness.policies)
        resume_response = harness.call_trigger(
            harness.resume_client,
            label="mission resume",
            test_case=self,
        )
        self.assertTrue(
            resume_response.success,
            f"mission resume rejected: {resume_response.message}",
        )
        self.assertIn("fresh policy", resume_response.message)

        harness.wait_until(
            lambda: any(
                state.state == MissionState.STATE_WAITING_FOR_POLICY
                and state.active_goal == "aisle_a"
                and not state.motion_authorized
                for state in harness.states[resume_state_marker:]
            ),
            timeout_seconds=10.0,
            failure_message="resume did not wait for a fresh policy",
            test_case=self,
        )

        harness.wait_until(
            lambda: any(
                is_authorizing(policy)
                and policy.evidence_id.startswith("aisle_a-")
                and policy.evidence_id != paused_state.last_evidence_id
                for policy in harness.policies[resume_policy_marker:]
            ),
            timeout_seconds=20.0,
            failure_message="no fresh authorizing policy arrived after resume",
            test_case=self,
        )

        harness.wait_until(
            lambda: any(
                state.state == MissionState.STATE_MOVING
                and state.active_goal == "aisle_a"
                and state.motion_authorized
                for state in harness.states[resume_state_marker:]
            ),
            timeout_seconds=20.0,
            failure_message="mission did not resume authorized motion",
            test_case=self,
        )

        harness.wait_until(
            lambda: (
                any(request.data == BLOCKED_WAYPOINT for request in harness.reroute_requests)
                and any(
                    state.state == MissionState.STATE_REROUTING
                    and state.active_goal == BLOCKED_WAYPOINT
                    for state in harness.states
                )
            ),
            timeout_seconds=60.0,
            failure_message="blocked aisle never produced a reroute request",
            test_case=self,
        )

        reroute_cmd_marker = len(harness.cmd_vels)
        harness.wait_until(
            lambda: (
                len(harness.cmd_vels) >= reroute_cmd_marker + 5
                and all(not is_nonzero_twist(twist) for twist in harness.cmd_vels[-5:])
            ),
            timeout_seconds=10.0,
            failure_message="cmd_vel did not remain zero while rerouting",
            test_case=self,
        )

        harness.publish_reroute_target(REROUTE_TARGET)

        harness.wait_until(
            lambda: any(
                state.active_goal == REROUTE_TARGET
                and state.state
                in (
                    MissionState.STATE_WAITING_FOR_POLICY,
                    MissionState.STATE_MOVING,
                )
                for state in harness.states
            ),
            timeout_seconds=20.0,
            failure_message="orchestrator did not accept the reroute target",
            test_case=self,
        )

        harness.wait_until(
            lambda: any(state.state == MissionState.STATE_COMPLETED for state in harness.states),
            timeout_seconds=90.0,
            failure_message="rerouted mission never reached COMPLETED",
            test_case=self,
        )

        completed = [
            state for state in harness.states if state.state == MissionState.STATE_COMPLETED
        ][-1]
        self.assertEqual(completed.waypoint_index, len(WAYPOINTS))
        self.assertEqual(completed.waypoint_count, len(WAYPOINTS))
        self.assertFalse(completed.motion_authorized)
        self.assertEqual(
            harness.goal_progression(),
            ["aisle_a", BLOCKED_WAYPOINT, REROUTE_TARGET, "aisle_c"],
        )

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
        verify_report_integrity(self, payload, mission_dir)

        recorded_waypoints = {record["waypoint"] for record in payload["evidence"]}
        self.assertTrue(
            {"aisle_a", BLOCKED_WAYPOINT, REROUTE_TARGET, "aisle_c"} <= recorded_waypoints
        )

        state_labels = [record["state"] for record in payload["mission_states"]]
        self.assertIn("paused", state_labels)
        self.assertIn("rerouting", state_labels)
        self.assertNotIn("safety_stopped", state_labels)


@launch_testing.post_shutdown_test()
class ReroutePauseResumeAfterShutdownTest(unittest.TestCase):
    def test_scenario_simulator_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0],
            process="scenario_simulator_node",
        )


if __name__ == "__main__":
    unittest.main()

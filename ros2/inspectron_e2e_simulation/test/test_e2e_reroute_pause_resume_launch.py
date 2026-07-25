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
from inspectron_safety_supervisor.msg import PolicyDecision

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
        desired_velocity_mode="always",
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

    def _assert_sustained_zero_velocity(self, description, sample_count=8):
        harness = self.harness
        harness.wait_until(
            lambda: harness.cmd_vels and not is_nonzero_twist(harness.cmd_vels[-1]),
            timeout_seconds=10.0,
            failure_message=f"cmd_vel did not stop during {description}",
            test_case=self,
        )

        output_marker = len(harness.cmd_vels)
        desired_marker = len(harness.desired_cmd_vels)
        harness.wait_until(
            lambda: (
                len(harness.cmd_vels) >= output_marker + sample_count
                and len(harness.desired_cmd_vels) >= desired_marker + sample_count
                and all(not is_nonzero_twist(twist) for twist in harness.cmd_vels[output_marker:])
                and all(
                    is_nonzero_twist(twist) for twist in harness.desired_cmd_vels[desired_marker:]
                )
            ),
            timeout_seconds=10.0,
            failure_message=(
                "nonzero desired velocity did not remain fresh and fully blocked "
                f"during {description}"
            ),
            test_case=self,
        )

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

        self._assert_sustained_zero_velocity("PAUSED")

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
        resumed_policy = next(
            policy
            for policy in harness.policies[resume_policy_marker:]
            if is_authorizing(policy)
            and policy.evidence_id.startswith("aisle_a-")
            and policy.evidence_id != paused_state.last_evidence_id
        )

        harness.wait_until(
            lambda: any(
                state.state == MissionState.STATE_MOVING
                and state.active_goal == "aisle_a"
                and state.motion_authorized
                and state.last_evidence_id == resumed_policy.evidence_id
                for state in harness.states[resume_state_marker:]
            ),
            timeout_seconds=20.0,
            failure_message="mission did not resume authorized motion",
            test_case=self,
        )

        harness.wait_until(
            lambda: any(
                policy.status == PolicyDecision.STATUS_VALID
                and policy.action == PolicyDecision.ACTION_REROUTE
                and policy.evidence_id.startswith(f"{BLOCKED_WAYPOINT}-")
                for policy in harness.policies
            ),
            timeout_seconds=60.0,
            failure_message="blocked aisle never produced a reroute policy",
            test_case=self,
        )
        reroute_policy = next(
            policy
            for policy in harness.policies
            if policy.status == PolicyDecision.STATUS_VALID
            and policy.action == PolicyDecision.ACTION_REROUTE
            and policy.evidence_id.startswith(f"{BLOCKED_WAYPOINT}-")
        )
        harness.wait_until(
            lambda: (
                any(request.data == BLOCKED_WAYPOINT for request in harness.reroute_requests)
                and any(
                    state.state == MissionState.STATE_REROUTING
                    and state.active_goal == BLOCKED_WAYPOINT
                    and state.last_evidence_id == reroute_policy.evidence_id
                    for state in harness.states
                )
            ),
            timeout_seconds=10.0,
            failure_message="reroute state did not preserve the blocking policy evidence",
            test_case=self,
        )

        self._assert_sustained_zero_velocity("REROUTING")

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
        self._assert_sustained_zero_velocity("COMPLETED")
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

        correlated_evidence_ids = {
            resumed_policy.evidence_id,
            reroute_policy.evidence_id,
        }
        self.assertLessEqual(
            correlated_evidence_ids,
            {item.evidence_id for item in harness.evidence},
        )
        self.assertLessEqual(
            correlated_evidence_ids,
            {item.evidence_id for item in harness.assessments},
        )
        self.assertLessEqual(
            correlated_evidence_ids,
            {item.evidence_id for item in harness.policies},
        )
        self.assertLessEqual(
            correlated_evidence_ids,
            {state.last_evidence_id for state in harness.states if state.last_evidence_id},
        )
        for section in (
            "evidence",
            "assessments",
            "policy_decisions",
            "mission_states",
        ):
            self.assertLessEqual(
                correlated_evidence_ids,
                {
                    record["evidence_id"]
                    if section != "mission_states"
                    else record["last_evidence_id"]
                    for record in payload[section]
                },
                f"report lost cross-topic evidence correlation in {section}",
            )

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

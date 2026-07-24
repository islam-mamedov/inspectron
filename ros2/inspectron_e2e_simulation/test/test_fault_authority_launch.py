from __future__ import annotations

import time
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

PREFIX = "/test/e2e_fault_d"
REPORT_ROOT = Path(mkdtemp(prefix="inspectron-fault-d-"))
WRONG_GOAL_TARGET = "aisle_b"
FORGED_TARGET = "aisle_c"


def generate_test_description():
    scenario = scenario_definition(SAFE_MISSION_SCENARIO)

    nodes = create_pipeline_nodes(
        waypoints=list(scenario.waypoints),
        fixture_response_json=scenario.fixture_response_json,
        report_directory=str(REPORT_ROOT),
        scene_id="e2e_fault_authority",
        desired_velocity_mode=scenario.desired_velocity_mode,
        auto_start=False,
        abort_on_safety_stop=False,
        prefix=PREFIX,
        node_name_suffix="e2e_fault_d",
        simulator_extra_parameters={
            "fault_wrong_goal_frame_at_goal_index": 1,
            "fault_forged_waypoint_at_goal_index": 2,
        },
    )

    return launch.LaunchDescription(
        [
            *nodes,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class AuthorityFaultTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = PipelineTestHarness("e2e_fault_d_test", PREFIX)
        self.harness.wait_for_graph(test_case=self)

    def tearDown(self):
        self.harness.destroy()

    def _wait_prime_decision(self, marker):
        harness = self.harness
        harness.wait_until(
            lambda: any(
                policy.evidence_id.startswith("aisle_a-") for policy in harness.policies[marker:]
            ),
            timeout_seconds=20.0,
            failure_message="no fresh priming decision before mission start",
            test_case=self,
        )

    def _start_mission(self):
        response = self.harness.call_trigger(
            self.harness.start_client,
            label="mission start",
            test_case=self,
        )
        self.assertTrue(response.success, f"mission start rejected: {response.message}")

    def _wait_state_in_segment(self, marker, predicate, *, timeout_seconds, failure_message):
        harness = self.harness
        harness.wait_until(
            lambda: any(predicate(state) for state in harness.states[marker:]),
            timeout_seconds=timeout_seconds,
            failure_message=failure_message,
            test_case=self,
        )
        return next(state for state in harness.states[marker:] if predicate(state))

    def _wait_zero_tail(self):
        harness = self.harness
        harness.wait_until(
            lambda: (
                len(harness.cmd_vels) >= 5
                and all(not is_nonzero_twist(twist) for twist in harness.cmd_vels[-5:])
            ),
            timeout_seconds=15.0,
            failure_message="cmd_vel did not settle to zero",
            test_case=self,
        )

    def _abort_and_finalize(self, marker, expected_finalized):
        harness = self.harness

        response = harness.call_trigger(
            harness.abort_client,
            label="mission abort",
            test_case=self,
        )
        self.assertTrue(response.success, f"mission abort rejected: {response.message}")

        self._wait_state_in_segment(
            marker,
            lambda state: state.state == MissionState.STATE_ABORTED,
            timeout_seconds=15.0,
            failure_message="mission never entered ABORTED after operator abort",
        )
        self._wait_finalized_count(expected_finalized)

    def _wait_finalized_count(self, expected_finalized):
        harness = self.harness
        harness.wait_until(
            lambda: (
                sum(
                    1
                    for status in harness.report_statuses
                    if status.status == ReportStatus.STATUS_FINALIZED
                )
                >= expected_finalized
            ),
            timeout_seconds=30.0,
            failure_message=(f"reporter never finalized report {expected_finalized}"),
            test_case=self,
        )

    def _reset_and_reprime(self):
        harness = self.harness
        marker = len(harness.states)
        prime_marker = len(harness.policies)

        response = harness.call_trigger(
            harness.reset_client,
            label="mission reset",
            test_case=self,
        )
        self.assertTrue(response.success, f"mission reset rejected: {response.message}")

        self._wait_state_in_segment(
            marker,
            lambda state: state.state == MissionState.STATE_IDLE,
            timeout_seconds=15.0,
            failure_message="mission never returned to IDLE after reset",
        )
        self._wait_prime_decision(prime_marker)

    def test_authority_faults_across_sequential_missions(self):
        harness = self.harness

        self._wait_prime_decision(0)

        # Mission 1: a frame for the wrong goal must safety-stop the mission.
        self._start_mission()

        stopped = self._wait_state_in_segment(
            0,
            lambda state: state.state == MissionState.STATE_SAFETY_STOPPED,
            timeout_seconds=30.0,
            failure_message="wrong-goal evidence never produced a safety stop",
        )
        self.assertIn("Policy evidence does not match", stopped.reason)
        self.assertEqual(stopped.active_goal, WRONG_GOAL_TARGET)

        harness.wait_until(
            lambda: any(
                policy.evidence_id.startswith("intruder_zone-") for policy in harness.policies
            ),
            timeout_seconds=10.0,
            failure_message="the wrong-goal decision never flowed through the supervisor",
            test_case=self,
        )

        for state in harness.states:
            if state.active_goal == WRONG_GOAL_TARGET:
                self.assertNotEqual(
                    state.state,
                    MissionState.STATE_MOVING,
                    "mission moved toward a goal with mismatched evidence",
                )

        self._wait_zero_tail()
        self._abort_and_finalize(0, expected_finalized=1)

        payload, mission_dir = load_report(self, REPORT_ROOT, expected_missions=1)
        self.assertEqual(payload["outcome"], "aborted")
        verify_report_integrity(self, payload, mission_dir)
        stop_reasons = [
            record["reason"]
            for record in payload["mission_states"]
            if record["state"] == "safety_stopped"
        ]
        self.assertTrue(
            any("Policy evidence does not match" in reason for reason in stop_reasons),
            f"report lost the mismatch reason: {stop_reasons}",
        )

        self._reset_and_reprime()

        # Mission 2: a forged waypoint_reached must safety-stop the mission.
        mission_two = len(harness.states)
        self._start_mission()

        stopped = self._wait_state_in_segment(
            mission_two,
            lambda state: state.state == MissionState.STATE_SAFETY_STOPPED,
            timeout_seconds=45.0,
            failure_message="forged waypoint never produced a safety stop",
        )
        self.assertIn("Reported waypoint does not match", stopped.reason)
        self.assertEqual(stopped.active_goal, FORGED_TARGET)

        self._wait_zero_tail()
        self._abort_and_finalize(mission_two, expected_finalized=2)

        payload, mission_dir = load_report(self, REPORT_ROOT, expected_missions=2)
        self.assertEqual(payload["outcome"], "aborted")
        verify_report_integrity(self, payload, mission_dir)
        stop_reasons = [
            record["reason"]
            for record in payload["mission_states"]
            if record["state"] == "safety_stopped"
        ]
        self.assertTrue(
            any("Reported waypoint does not match" in reason for reason in stop_reasons),
            f"report lost the forged-waypoint reason: {stop_reasons}",
        )

        self._reset_and_reprime()

        # Mission 3: emergency stop mid-motion, operator recovery contract.
        mission_three = len(harness.states)
        nonzero_before = harness.nonzero_cmd_count()
        self._start_mission()

        self._wait_state_in_segment(
            mission_three,
            lambda state: (
                state.state == MissionState.STATE_MOVING
                and state.active_goal == "aisle_a"
                and state.motion_authorized
            ),
            timeout_seconds=30.0,
            failure_message="mission three never started moving",
        )

        harness.wait_until(
            lambda: harness.nonzero_cmd_count() > nonzero_before,
            timeout_seconds=15.0,
            failure_message="no motion observed before the emergency stop",
            test_case=self,
        )

        harness.wait_until(
            lambda: harness.emergency_stop_publisher.get_subscription_count() > 0,
            timeout_seconds=10.0,
            failure_message="emergency stop topic has no subscriber",
            test_case=self,
        )

        deadline = time.monotonic() + 15.0
        next_publish = 0.0

        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                harness.publish_emergency_stop(True)
                next_publish = time.monotonic() + 0.25

            rclpy.spin_once(harness.node, timeout_sec=0.05)

            if any(
                state.state == MissionState.STATE_EMERGENCY_STOPPED
                for state in harness.states[mission_three:]
            ):
                break
        else:
            self.fail("mission never entered EMERGENCY_STOPPED")

        self._wait_zero_tail()

        emergency_index = next(
            index
            for index, state in enumerate(harness.states)
            if index >= mission_three and state.state == MissionState.STATE_EMERGENCY_STOPPED
        )

        for state in harness.states[emergency_index:]:
            self.assertFalse(
                state.motion_authorized,
                "motion authorized after the emergency stop",
            )

        response = harness.call_trigger(
            harness.reset_client,
            label="mission reset during emergency stop",
            test_case=self,
        )
        self.assertFalse(
            response.success,
            "reset was accepted while the emergency stop was active",
        )
        self.assertIn("emergency", response.message.lower())

        self._wait_finalized_count(3)

        payload, mission_dir = load_report(self, REPORT_ROOT, expected_missions=3)
        self.assertEqual(payload["outcome"], "emergency_stopped")
        verify_report_integrity(self, payload, mission_dir)

        state_labels = [record["state"] for record in payload["mission_states"]]
        self.assertIn("moving", state_labels)
        self.assertIn("emergency_stopped", state_labels)

        last_state = payload["mission_states"][-1]
        self.assertEqual(last_state["state"], "emergency_stopped")
        self.assertFalse(last_state["motion_authorized"])

        reset_accepted = False

        for _ in range(10):
            harness.publish_emergency_stop(False)

            for _ in range(4):
                rclpy.spin_once(harness.node, timeout_sec=0.05)

            response = harness.call_trigger(
                harness.reset_client,
                label="mission reset after clearing emergency stop",
                test_case=self,
            )

            if response.success:
                reset_accepted = True
                break

        self.assertTrue(
            reset_accepted,
            "reset never succeeded after clearing the emergency stop",
        )

        self._wait_state_in_segment(
            emergency_index,
            lambda state: state.state == MissionState.STATE_IDLE,
            timeout_seconds=15.0,
            failure_message="mission never returned to IDLE after recovery",
        )


@launch_testing.post_shutdown_test()
class AuthorityFaultAfterShutdownTest(unittest.TestCase):
    def test_scenario_simulator_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0],
            process="scenario_simulator_node",
        )


if __name__ == "__main__":
    unittest.main()

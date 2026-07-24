from __future__ import annotations

import unittest

from inspectron_e2e_simulation.scenario_logic import (
    DESIRED_MODE_ALWAYS,
    DESIRED_MODE_AUTHORIZED_ONLY,
    STATE_ABORTED,
    STATE_MOVING,
    STATE_SAFETY_STOPPED,
    STATE_WAITING_FOR_POLICY,
    CallMissionAbort,
    CallMissionStart,
    CompleteScenario,
    GoalPhase,
    PublishCameraFrame,
    PublishDesiredVelocity,
    PublishWaypointReached,
    ScenarioDriverCore,
    evidence_matches_waypoint,
    sanitize_waypoint,
)

REPORT_RECORDING = 1
REPORT_FINALIZED = 2


def make_core(**overrides):
    parameters = dict(
        waypoints=("aisle_a", "aisle_b"),
        desired_linear_x=0.3,
        desired_angular_z=0.0,
        min_authorized_motion_samples=3,
        frame_interval_seconds=0.1,
        desired_velocity_mode=DESIRED_MODE_AUTHORIZED_ONLY,
        auto_start=True,
        abort_on_safety_stop=False,
    )
    parameters.update(overrides)
    return ScenarioDriverCore(**parameters)


def commands_of_type(commands, command_type):
    return [command for command in commands if isinstance(command, command_type)]


def evidence_id_for(waypoint, sequence):
    return f"{waypoint}-{sequence}.000000000-{sequence:06d}"


def resolve_frame(core, waypoint, sequence):
    evidence_id = evidence_id_for(waypoint, sequence)
    commands = list(core.on_evidence_capture(evidence_id))
    commands.extend(core.on_assessment(evidence_id))
    return evidence_id, commands


def prime(core, *, now=0.0):
    core.on_graph_ready()
    frames = commands_of_type(core.on_tick(now), PublishCameraFrame)
    assert len(frames) == 1
    return frames[0]


def prime_and_start(core):
    frame = prime(core)
    evidence_id, _ = resolve_frame(core, frame.waypoint, 1)
    return core.on_policy_decision(action=0, status=0, evidence_id=evidence_id)


def begin_recorded_waiting(core, goal):
    core.on_report_status(status=REPORT_RECORDING, report_path="", message="recording")
    core.on_mission_state(
        state=STATE_WAITING_FOR_POLICY,
        active_goal=goal,
        motion_authorized=False,
        last_evidence_id="",
    )


class ValidationTest(unittest.TestCase):
    def test_rejects_empty_waypoint_list(self):
        with self.assertRaises(ValueError):
            make_core(waypoints=())

    def test_rejects_blank_waypoint(self):
        with self.assertRaises(ValueError):
            make_core(waypoints=("aisle_a", "  "))

    def test_rejects_duplicate_waypoints(self):
        with self.assertRaises(ValueError):
            make_core(waypoints=("aisle_a", "aisle_a"))

    def test_rejects_nesting_evidence_prefixes(self):
        with self.assertRaises(ValueError):
            make_core(waypoints=("dock", "dock-2"))

    def test_rejects_zero_desired_linear_velocity(self):
        with self.assertRaises(ValueError):
            make_core(desired_linear_x=0.0)

    def test_rejects_unknown_desired_mode(self):
        with self.assertRaises(ValueError):
            make_core(desired_velocity_mode="sometimes")

    def test_rejects_non_positive_frame_interval(self):
        with self.assertRaises(ValueError):
            make_core(frame_interval_seconds=0.0)

    def test_rejects_non_positive_motion_samples(self):
        with self.assertRaises(ValueError):
            make_core(min_authorized_motion_samples=0)

    def test_sanitize_matches_production_prefixing(self):
        self.assertEqual(sanitize_waypoint("dock bay 2"), "dock_bay_2")
        self.assertTrue(evidence_matches_waypoint("dock_bay_2-1.000000000-000001", "dock bay 2"))


class PrimingTest(unittest.TestCase):
    def test_no_frame_before_graph_ready(self):
        core = make_core()
        self.assertEqual(commands_of_type(core.on_tick(0.0), PublishCameraFrame), [])

    def test_priming_frame_uses_first_waypoint(self):
        core = make_core()
        frame = prime(core)
        self.assertEqual(frame.waypoint, "aisle_a")

    def test_missing_prime_evidence_is_retried_then_lockstep_resumes(self):
        core = make_core()
        prime(core, now=0.0)
        self.assertEqual(commands_of_type(core.on_tick(0.2), PublishCameraFrame), [])
        frames = commands_of_type(core.on_tick(0.6), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].waypoint, "aisle_a")

        core.on_evidence_capture(evidence_id_for("aisle_a", 1))
        self.assertEqual(commands_of_type(core.on_tick(1.2), PublishCameraFrame), [])

        core.on_assessment(evidence_id_for("aisle_a", 1))
        frames = commands_of_type(core.on_tick(1.3), PublishCameraFrame)
        self.assertEqual(len(frames), 1)

    def test_prime_retry_is_paced(self):
        core = make_core()
        prime(core, now=0.0)
        resolve_frame(core, "aisle_a", 1)
        self.assertEqual(commands_of_type(core.on_tick(0.2), PublishCameraFrame), [])
        frames = commands_of_type(core.on_tick(0.6), PublishCameraFrame)
        self.assertEqual(len(frames), 1)

    def test_auto_start_emitted_once_after_prime_decision(self):
        core = make_core()
        commands = prime_and_start(core)
        self.assertEqual(len(commands_of_type(commands, CallMissionStart)), 1)
        self.assertTrue(core.start_gate_open)

        repeat = core.on_policy_decision(
            action=0,
            status=0,
            evidence_id=evidence_id_for("aisle_a", 2),
        )
        self.assertEqual(commands_of_type(repeat, CallMissionStart), [])

    def test_manual_mode_never_emits_start(self):
        core = make_core(auto_start=False)
        commands = prime_and_start(core)
        self.assertEqual(commands_of_type(commands, CallMissionStart), [])
        self.assertTrue(core.start_gate_open)

    def test_no_frames_between_gate_open_and_mission_start(self):
        core = make_core()
        prime_and_start(core)
        self.assertEqual(commands_of_type(core.on_tick(20.0), PublishCameraFrame), [])


class StreamingGateTest(unittest.TestCase):
    def test_streaming_requires_reporter_recording(self):
        core = make_core()
        prime_and_start(core)
        core.on_mission_state(
            state=STATE_WAITING_FOR_POLICY,
            active_goal="aisle_a",
            motion_authorized=False,
            last_evidence_id="",
        )
        self.assertEqual(commands_of_type(core.on_tick(30.0), PublishCameraFrame), [])

        core.on_report_status(status=REPORT_RECORDING, report_path="", message="recording")
        frames = commands_of_type(core.on_tick(31.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].waypoint, "aisle_a")

    def test_in_mission_lockstep_blocks_frames_until_resolution(self):
        core = make_core()
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")

        frames = commands_of_type(core.on_tick(35.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(commands_of_type(core.on_tick(36.0), PublishCameraFrame), [])

        core.on_evidence_capture(evidence_id_for("aisle_a", 2))
        self.assertEqual(commands_of_type(core.on_tick(37.0), PublishCameraFrame), [])

        core.on_assessment(evidence_id_for("aisle_a", 2))
        frames = commands_of_type(core.on_tick(38.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)

    def test_streaming_stops_in_safety_stopped(self):
        core = make_core()
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        core.on_mission_state(
            state=STATE_SAFETY_STOPPED,
            active_goal="aisle_a",
            motion_authorized=False,
            last_evidence_id="",
        )
        self.assertEqual(commands_of_type(core.on_tick(40.0), PublishCameraFrame), [])

    def test_perception_request_resets_frame_pacing(self):
        core = make_core()
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")

        frames = commands_of_type(core.on_tick(50.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        resolve_frame(core, "aisle_a", 2)

        self.assertEqual(commands_of_type(core.on_tick(50.05), PublishCameraFrame), [])
        core.on_perception_request()
        frames = commands_of_type(core.on_tick(50.06), PublishCameraFrame)
        self.assertEqual(len(frames), 1)


class WaypointProtocolTest(unittest.TestCase):
    def authorize_goal(self, core, goal, sequence):
        begin_recorded_waiting(core, goal)
        frames = commands_of_type(core.on_tick(100.0 + sequence), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].waypoint, goal)

        evidence_id, _ = resolve_frame(core, goal, sequence)
        core.on_policy_decision(action=0, status=0, evidence_id=evidence_id)
        core.on_mission_state(
            state=STATE_MOVING,
            active_goal=goal,
            motion_authorized=True,
            last_evidence_id=evidence_id,
        )
        return evidence_id

    def test_waypoint_reached_after_motion_drain_and_state_match(self):
        core = make_core()
        prime_and_start(core)
        self.authorize_goal(core, "aisle_a", 2)

        commands = []
        for _ in range(3):
            commands = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)

        reached = commands_of_type(commands, PublishWaypointReached)
        self.assertEqual(len(reached), 1)
        self.assertEqual(reached[0].waypoint, "aisle_a")
        self.assertIs(core.goal_phase("aisle_a"), GoalPhase.REPORTED)

        extra = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)
        self.assertEqual(commands_of_type(extra, PublishWaypointReached), [])

    def test_waypoint_not_reached_without_enough_motion(self):
        core = make_core()
        prime_and_start(core)
        self.authorize_goal(core, "aisle_a", 2)

        commands = list(core.on_cmd_vel(linear_x=0.3, angular_z=0.0))
        commands.extend(core.on_cmd_vel(linear_x=0.3, angular_z=0.0))
        self.assertEqual(commands_of_type(commands, PublishWaypointReached), [])
        self.assertIs(core.goal_phase("aisle_a"), GoalPhase.STREAMING)

    def test_zero_cmd_vel_does_not_count_as_motion(self):
        core = make_core()
        prime_and_start(core)
        self.authorize_goal(core, "aisle_a", 2)

        for _ in range(10):
            core.on_cmd_vel(linear_x=0.0, angular_z=0.0)

        self.assertIs(core.goal_phase("aisle_a"), GoalPhase.STREAMING)

    def test_waypoint_waits_for_in_flight_frame_and_state_update(self):
        core = make_core()
        prime_and_start(core)
        first_evidence = self.authorize_goal(core, "aisle_a", 2)

        frames = commands_of_type(core.on_tick(200.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)

        commands = []
        for _ in range(3):
            commands = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)

        self.assertEqual(commands_of_type(commands, PublishWaypointReached), [])
        self.assertIs(core.goal_phase("aisle_a"), GoalPhase.DRAINING)

        second_evidence, resolve_commands = resolve_frame(core, "aisle_a", 3)
        self.assertEqual(
            commands_of_type(resolve_commands, PublishWaypointReached),
            [],
        )
        self.assertIs(
            core.goal_phase("aisle_a"),
            GoalPhase.AWAITING_STATE_CONFIRMATION,
        )

        stale_state = core.on_mission_state(
            state=STATE_MOVING,
            active_goal="aisle_a",
            motion_authorized=True,
            last_evidence_id=first_evidence,
        )
        self.assertEqual(commands_of_type(stale_state, PublishWaypointReached), [])

        fresh_state = core.on_mission_state(
            state=STATE_MOVING,
            active_goal="aisle_a",
            motion_authorized=True,
            last_evidence_id=second_evidence,
        )
        reached = commands_of_type(fresh_state, PublishWaypointReached)
        self.assertEqual(len(reached), 1)

    def test_next_goal_streams_with_new_frame_id(self):
        core = make_core()
        prime_and_start(core)
        self.authorize_goal(core, "aisle_a", 2)

        for _ in range(3):
            core.on_cmd_vel(linear_x=0.3, angular_z=0.0)

        core.on_mission_state(
            state=STATE_WAITING_FOR_POLICY,
            active_goal="aisle_b",
            motion_authorized=False,
            last_evidence_id="",
        )
        frames = commands_of_type(core.on_tick(300.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].waypoint, "aisle_b")


class DesiredVelocityTest(unittest.TestCase):
    def test_authorized_only_mode_waits_for_authorization(self):
        core = make_core()
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.assertEqual(
            commands_of_type(core.on_tick(400.0), PublishDesiredVelocity),
            [],
        )

        core.on_mission_state(
            state=STATE_MOVING,
            active_goal="aisle_a",
            motion_authorized=True,
            last_evidence_id="",
        )
        desired = commands_of_type(core.on_tick(401.0), PublishDesiredVelocity)
        self.assertEqual(len(desired), 1)
        self.assertAlmostEqual(desired[0].linear_x, 0.3)

    def test_always_mode_streams_before_start(self):
        core = make_core(desired_velocity_mode=DESIRED_MODE_ALWAYS)
        desired = commands_of_type(core.on_tick(0.0), PublishDesiredVelocity)
        self.assertEqual(len(desired), 1)


class HazardBehaviorTest(unittest.TestCase):
    def test_abort_emitted_once_when_configured(self):
        core = make_core(
            desired_velocity_mode=DESIRED_MODE_ALWAYS,
            abort_on_safety_stop=True,
        )
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")

        commands = core.on_mission_state(
            state=STATE_SAFETY_STOPPED,
            active_goal="aisle_a",
            motion_authorized=False,
            last_evidence_id="",
        )
        self.assertEqual(len(commands_of_type(commands, CallMissionAbort)), 1)

        repeat = core.on_mission_state(
            state=STATE_SAFETY_STOPPED,
            active_goal="aisle_a",
            motion_authorized=False,
            last_evidence_id="",
        )
        self.assertEqual(commands_of_type(repeat, CallMissionAbort), [])

    def test_abort_not_emitted_when_disabled(self):
        core = make_core()
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")

        commands = core.on_mission_state(
            state=STATE_SAFETY_STOPPED,
            active_goal="aisle_a",
            motion_authorized=False,
            last_evidence_id="",
        )
        self.assertEqual(commands_of_type(commands, CallMissionAbort), [])

    def test_stop_policy_does_not_open_authorization(self):
        core = make_core(desired_velocity_mode=DESIRED_MODE_ALWAYS)
        prime(core)
        evidence_id, _ = resolve_frame(core, "aisle_a", 1)
        core.on_policy_decision(action=2, status=0, evidence_id=evidence_id)
        self.assertFalse(core.authorizing_policy_seen)
        self.assertTrue(core.start_gate_open)


class CommandVelocityStatsTest(unittest.TestCase):
    def test_nonzero_before_authorizing_policy_is_counted(self):
        core = make_core(desired_velocity_mode=DESIRED_MODE_ALWAYS)
        core.on_cmd_vel(linear_x=0.2, angular_z=0.0)
        self.assertEqual(core.cmd_vel_stats.nonzero_before_authorizing_policy, 1)

        core.on_policy_decision(
            action=0,
            status=0,
            evidence_id=evidence_id_for("aisle_a", 1),
        )
        core.on_cmd_vel(linear_x=0.2, angular_z=0.0)
        self.assertEqual(core.cmd_vel_stats.nonzero_before_authorizing_policy, 1)
        self.assertEqual(core.cmd_vel_stats.nonzero_samples, 2)

    def test_trailing_zero_streak_resets_on_motion(self):
        core = make_core()
        core.on_cmd_vel(linear_x=0.0, angular_z=0.0)
        core.on_cmd_vel(linear_x=0.1, angular_z=0.0)
        core.on_cmd_vel(linear_x=0.0, angular_z=0.0)
        core.on_cmd_vel(linear_x=0.0, angular_z=0.0)
        self.assertEqual(core.cmd_vel_stats.trailing_zero_samples, 2)
        self.assertEqual(core.cmd_vel_stats.total_samples, 4)


class CompletionTest(unittest.TestCase):
    def test_completion_requires_terminal_state_and_finalized_report(self):
        core = make_core()
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")

        commands = core.on_mission_state(
            state=STATE_ABORTED,
            active_goal="",
            motion_authorized=False,
            last_evidence_id="",
        )
        self.assertEqual(commands_of_type(commands, CompleteScenario), [])

        commands = core.on_report_status(
            status=REPORT_FINALIZED,
            report_path="/tmp/report.json",
            message="finalized",
        )
        complete = commands_of_type(commands, CompleteScenario)
        self.assertEqual(len(complete), 1)
        self.assertIn("terminal_state=aborted", complete[0].summary)
        self.assertIn("report_finalized=True", complete[0].summary)
        self.assertEqual(core.report_path, "/tmp/report.json")

        repeat = core.on_report_status(
            status=REPORT_FINALIZED,
            report_path="/tmp/report.json",
            message="finalized",
        )
        self.assertEqual(commands_of_type(repeat, CompleteScenario), [])

    def test_manual_mode_does_not_emit_completion(self):
        core = make_core(auto_start=False)
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        core.on_mission_state(
            state=STATE_ABORTED,
            active_goal="",
            motion_authorized=False,
            last_evidence_id="",
        )
        commands = core.on_report_status(
            status=REPORT_FINALIZED,
            report_path="/tmp/report.json",
            message="finalized",
        )
        self.assertEqual(commands_of_type(commands, CompleteScenario), [])


if __name__ == "__main__":
    unittest.main()

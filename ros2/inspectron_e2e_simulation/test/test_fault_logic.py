from __future__ import annotations

import unittest

from inspectron_e2e_simulation.scenario_logic import (
    DESIRED_MODE_AUTHORIZED_ONLY,
    STATE_IDLE,
    STATE_INSPECTING_CLOSER,
    STATE_MOVING,
    STATE_WAITING_FOR_POLICY,
    WRONG_GOAL_WAYPOINT,
    FaultPlan,
    GoalPhase,
    LogNote,
    PublishCameraFrame,
    PublishDesiredVelocity,
    PublishWaypointReached,
    ScenarioDriverCore,
)

REPORT_RECORDING = 1


def make_core(**overrides):
    parameters = dict(
        waypoints=("aisle_a", "aisle_b", "aisle_c"),
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


def notes_containing(commands, text):
    return [note for note in commands_of_type(commands, LogNote) if text in note.message]


def evidence_id_for(waypoint, sequence):
    return f"{waypoint}-{sequence}.000000000-{sequence:06d}"


def resolve_frame(core, waypoint, sequence):
    evidence_id = evidence_id_for(waypoint, sequence)
    commands = list(core.on_evidence_capture(evidence_id))
    commands.extend(core.on_assessment(evidence_id))
    return evidence_id, commands


def prime_and_start(core):
    core.on_graph_ready()
    frames = commands_of_type(core.on_tick(0.0), PublishCameraFrame)
    assert len(frames) == 1
    prime_frame = frames[0]
    evidence_id, _ = resolve_frame(core, prime_frame.waypoint, 1)
    core.on_policy_decision(action=0, status=0, evidence_id=evidence_id)
    return prime_frame


def begin_recorded_waiting(core, goal):
    core.on_report_status(status=REPORT_RECORDING, report_path="", message="recording")
    core.on_mission_state(
        state=STATE_WAITING_FOR_POLICY,
        active_goal=goal,
        motion_authorized=False,
        last_evidence_id="",
    )


class FaultCoreTestCase(unittest.TestCase):
    def authorize_goal(self, core, goal, sequence, *, now):
        frames = commands_of_type(core.on_tick(now), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].waypoint, goal)
        self.assertFalse(frames[0].malformed)

        evidence_id, _ = resolve_frame(core, goal, sequence)
        core.on_policy_decision(action=0, status=0, evidence_id=evidence_id)
        state_commands = core.on_mission_state(
            state=STATE_MOVING,
            active_goal=goal,
            motion_authorized=True,
            last_evidence_id=evidence_id,
        )
        return evidence_id, state_commands

    def complete_goal(self, core, goal, sequence, *, now):
        self.authorize_goal(core, goal, sequence, now=now)

        reached = []

        for _ in range(3):
            commands = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)
            reached.extend(commands_of_type(commands, PublishWaypointReached))

        self.assertEqual(len(reached), 1)
        self.assertEqual(reached[0].waypoint, goal)


class FaultPlanValidationTest(unittest.TestCase):
    def test_rejects_out_of_range_goal_index(self):
        with self.assertRaises(ValueError):
            make_core(fault_plan=FaultPlan(camera_stall_at_goal_index=3))

    def test_rejects_negative_non_disabled_goal_index(self):
        with self.assertRaises(ValueError):
            make_core(fault_plan=FaultPlan(malformed_frame_at_goal_index=-2))

    def test_rejects_two_faults_on_one_goal(self):
        with self.assertRaises(ValueError):
            make_core(
                fault_plan=FaultPlan(
                    camera_stall_at_goal_index=1,
                    malformed_frame_at_goal_index=1,
                )
            )

    def test_rejects_desired_stall_without_ticks(self):
        with self.assertRaises(ValueError):
            make_core(fault_plan=FaultPlan(desired_stall_at_goal_index=1))

    def test_rejects_stall_ticks_without_goal(self):
        with self.assertRaises(ValueError):
            make_core(fault_plan=FaultPlan(desired_stall_ticks=5))

    def test_rejects_wrong_goal_fault_when_waypoint_collides(self):
        with self.assertRaises(ValueError):
            make_core(
                waypoints=("aisle_a", WRONG_GOAL_WAYPOINT),
                fault_plan=FaultPlan(wrong_goal_frame_at_goal_index=0),
            )

    def test_default_plan_has_no_enabled_faults(self):
        core = make_core()
        self.assertEqual(core.fault_plan.enabled_goal_indexes(), [])


class CameraStallFaultTest(FaultCoreTestCase):
    def test_stall_halts_goal_after_authorization(self):
        core = make_core(fault_plan=FaultPlan(camera_stall_at_goal_index=1))
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)

        begin_recorded_waiting(core, "aisle_b")
        _, state_commands = self.authorize_goal(core, "aisle_b", 3, now=20.0)

        self.assertEqual(len(notes_containing(state_commands, "camera stalled")), 1)
        self.assertIs(core.goal_phase("aisle_b"), GoalPhase.FAULT_HALTED)

        self.assertEqual(commands_of_type(core.on_tick(30.0), PublishCameraFrame), [])

        for _ in range(10):
            commands = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)
            self.assertEqual(commands_of_type(commands, PublishWaypointReached), [])

        self.assertIs(core.goal_phase("aisle_b"), GoalPhase.FAULT_HALTED)
        self.assertIn("camera_stall", core.summary_text())

    def test_earlier_goal_unaffected_by_stall_target(self):
        core = make_core(fault_plan=FaultPlan(camera_stall_at_goal_index=1))
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)


class MalformedFrameFaultTest(FaultCoreTestCase):
    def test_first_mission_frame_at_goal_is_malformed_once(self):
        core = make_core(fault_plan=FaultPlan(malformed_frame_at_goal_index=1))
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)

        begin_recorded_waiting(core, "aisle_b")
        frames = commands_of_type(core.on_tick(20.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertTrue(frames[0].malformed)
        self.assertEqual(frames[0].waypoint, "aisle_b")

        core.on_assessment(evidence_id_for("aisle_b", 3))

        follow_up = commands_of_type(core.on_tick(21.0), PublishCameraFrame)
        self.assertEqual(len(follow_up), 1)
        self.assertFalse(follow_up[0].malformed)
        self.assertEqual(follow_up[0].waypoint, "aisle_b")
        self.assertIn("malformed_frame", core.summary_text())

    def test_malformed_resolves_on_assessment_alone(self):
        core = make_core(fault_plan=FaultPlan(malformed_frame_at_goal_index=1))
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)

        begin_recorded_waiting(core, "aisle_b")
        commands_of_type(core.on_tick(20.0), PublishCameraFrame)

        self.assertEqual(commands_of_type(core.on_tick(21.0), PublishCameraFrame), [])
        core.on_assessment(evidence_id_for("aisle_b", 3))
        frames = commands_of_type(core.on_tick(22.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)

    def test_priming_frame_is_never_malformed(self):
        core = make_core(fault_plan=FaultPlan(malformed_frame_at_goal_index=0))
        prime_frame = prime_and_start(core)
        self.assertFalse(prime_frame.malformed)

        begin_recorded_waiting(core, "aisle_a")
        frames = commands_of_type(core.on_tick(10.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertTrue(frames[0].malformed)

    def test_streaming_continues_in_inspecting_closer(self):
        core = make_core(fault_plan=FaultPlan(malformed_frame_at_goal_index=1))
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)

        begin_recorded_waiting(core, "aisle_b")
        commands_of_type(core.on_tick(20.0), PublishCameraFrame)
        core.on_assessment(evidence_id_for("aisle_b", 3))

        core.on_mission_state(
            state=STATE_INSPECTING_CLOSER,
            active_goal="aisle_b",
            motion_authorized=False,
            last_evidence_id=evidence_id_for("aisle_b", 3),
        )
        frames = commands_of_type(core.on_tick(21.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertFalse(frames[0].malformed)


class WrongGoalFrameFaultTest(FaultCoreTestCase):
    def test_first_frame_at_goal_carries_wrong_waypoint_once(self):
        core = make_core(fault_plan=FaultPlan(wrong_goal_frame_at_goal_index=1))
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)

        begin_recorded_waiting(core, "aisle_b")
        frames = commands_of_type(core.on_tick(20.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].waypoint, WRONG_GOAL_WAYPOINT)
        self.assertFalse(frames[0].malformed)

        resolve_frame(core, WRONG_GOAL_WAYPOINT, 3)

        follow_up = commands_of_type(core.on_tick(21.0), PublishCameraFrame)
        self.assertEqual(len(follow_up), 1)
        self.assertEqual(follow_up[0].waypoint, "aisle_b")
        self.assertIn("wrong_goal_frame", core.summary_text())


class DesiredStallFaultTest(FaultCoreTestCase):
    def test_stall_engages_suppresses_and_releases(self):
        core = make_core(
            fault_plan=FaultPlan(
                desired_stall_at_goal_index=1,
                desired_stall_ticks=4,
            )
        )
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)

        begin_recorded_waiting(core, "aisle_b")
        self.authorize_goal(core, "aisle_b", 3, now=20.0)

        first = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)
        self.assertEqual(notes_containing(first, "desired velocity stalled"), [])
        self.assertFalse(core.desired_stall_active)

        second = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)
        self.assertEqual(len(notes_containing(second, "desired velocity stalled")), 1)
        self.assertTrue(core.desired_stall_active)

        for _ in range(6):
            commands = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)
            self.assertEqual(commands_of_type(commands, PublishWaypointReached), [])

        for tick in range(3):
            commands = core.on_tick(30.0 + tick)
            self.assertEqual(commands_of_type(commands, PublishDesiredVelocity), [])
            self.assertTrue(core.desired_stall_active)

        release = core.on_tick(33.0)
        self.assertEqual(commands_of_type(release, PublishDesiredVelocity), [])
        self.assertEqual(len(notes_containing(release, "FAULT released")), 1)
        self.assertFalse(core.desired_stall_active)

        resumed = core.on_tick(34.0)
        self.assertEqual(len(commands_of_type(resumed, PublishDesiredVelocity)), 1)

        # A frame emitted during the stall is still in flight, so the drain
        # must hold until it resolves and the orchestrator confirms it.
        commands = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)
        self.assertEqual(commands_of_type(commands, PublishWaypointReached), [])

        stall_frame_evidence, _ = resolve_frame(core, "aisle_b", 4)
        state_commands = core.on_mission_state(
            state=STATE_MOVING,
            active_goal="aisle_b",
            motion_authorized=True,
            last_evidence_id=stall_frame_evidence,
        )
        reached = commands_of_type(state_commands, PublishWaypointReached)
        self.assertEqual(len(reached), 1)
        self.assertEqual(reached[0].waypoint, "aisle_b")
        self.assertIn("desired_stall", core.summary_text())


class ForgedWaypointFaultTest(FaultCoreTestCase):
    def test_forged_waypoint_emitted_once_on_authorization(self):
        core = make_core(fault_plan=FaultPlan(forged_waypoint_at_goal_index=1))
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)

        begin_recorded_waiting(core, "aisle_b")
        _, state_commands = self.authorize_goal(core, "aisle_b", 3, now=20.0)

        forged = commands_of_type(state_commands, PublishWaypointReached)
        self.assertEqual(len(forged), 1)
        self.assertEqual(forged[0].waypoint, WRONG_GOAL_WAYPOINT)
        self.assertIs(core.goal_phase("aisle_b"), GoalPhase.FAULT_HALTED)

        for _ in range(10):
            commands = core.on_cmd_vel(linear_x=0.3, angular_z=0.0)
            self.assertEqual(commands_of_type(commands, PublishWaypointReached), [])

        self.assertEqual(commands_of_type(core.on_tick(30.0), PublishCameraFrame), [])
        self.assertIn("forged_waypoint", core.summary_text())


class DefaultBehaviorRegressionTest(FaultCoreTestCase):
    def test_disabled_faults_leave_protocol_unchanged(self):
        core = make_core()
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)
        self.assertIn("injected_faults=none", core.summary_text())


class MissionResetTest(FaultCoreTestCase):
    def test_idle_reset_closes_gate_and_reprimes_for_next_mission(self):
        core = make_core()
        prime_and_start(core)
        begin_recorded_waiting(core, "aisle_a")
        self.complete_goal(core, "aisle_a", 2, now=10.0)
        self.assertIs(core.goal_phase("aisle_a"), GoalPhase.REPORTED)
        self.assertTrue(core.start_gate_open)

        core.on_mission_state(
            state=STATE_IDLE,
            active_goal="",
            motion_authorized=False,
            last_evidence_id="",
        )
        self.assertIsNone(core.goal_phase("aisle_a"))
        self.assertFalse(core.start_gate_open)

        frames = commands_of_type(core.on_tick(50.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].waypoint, "aisle_a")

        evidence_id, _ = resolve_frame(core, "aisle_a", 9)
        core.on_policy_decision(action=0, status=0, evidence_id=evidence_id)
        self.assertTrue(core.start_gate_open)

        begin_recorded_waiting(core, "aisle_a")
        frames = commands_of_type(core.on_tick(60.0), PublishCameraFrame)
        self.assertEqual(len(frames), 1)


if __name__ == "__main__":
    unittest.main()

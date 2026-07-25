from __future__ import annotations

import unittest

from inspectron_mission_orchestrator.state_machine import (
    ACTION_INSPECT_CLOSER,
    ACTION_PROCEED,
    ACTION_REROUTE,
    ACTION_STOP,
    STATUS_STALE,
    STATUS_VALID,
    MissionPhase,
    MissionStateMachine,
)


class MissionStateMachineTests(unittest.TestCase):
    def setUp(self):
        self.machine = MissionStateMachine(
            ["aisle_a", "aisle_b"],
            policy_timeout_seconds=0.75,
            reroute_timeout_seconds=2.0,
        )

    def test_completes_two_waypoint_mission(self):
        start = self.machine.start()

        self.assertTrue(start.accepted)
        self.assertTrue(start.publish_goal)
        self.assertTrue(start.request_perception)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.WAITING_FOR_POLICY,
        )
        self.assertEqual(
            self.machine.snapshot.active_goal,
            "aisle_a",
        )

        proceed = self.machine.apply_policy(
            action=ACTION_PROCEED,
            status=STATUS_VALID,
            evidence_id="aisle_a-1.000000000-000001",
            now=1.0,
        )

        self.assertTrue(proceed.accepted)
        self.assertTrue(proceed.publish_goal)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.MOVING,
        )
        self.assertTrue(self.machine.snapshot.motion_authorized)

        first_reached = self.machine.waypoint_reached("aisle_a")

        self.assertTrue(first_reached.accepted)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.WAITING_FOR_POLICY,
        )
        self.assertEqual(
            self.machine.snapshot.active_goal,
            "aisle_b",
        )

        self.machine.apply_policy(
            action=ACTION_PROCEED,
            status=STATUS_VALID,
            evidence_id="aisle_b-2.000000000-000002",
            now=2.0,
        )
        second_reached = self.machine.waypoint_reached("aisle_b")

        self.assertTrue(second_reached.accepted)
        self.assertTrue(second_reached.cancel_motion)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.COMPLETED,
        )
        self.assertEqual(
            self.machine.snapshot.waypoint_index,
            2,
        )
        self.assertFalse(self.machine.snapshot.motion_authorized)

    def test_mismatched_evidence_fails_closed(self):
        self.machine.start()

        result = self.machine.apply_policy(
            action=ACTION_PROCEED,
            status=STATUS_VALID,
            evidence_id="wrong_waypoint-1.000000000-000001",
            now=1.0,
        )

        self.assertFalse(result.accepted)
        self.assertTrue(result.cancel_motion)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.SAFETY_STOPPED,
        )
        self.assertFalse(self.machine.snapshot.motion_authorized)

    def test_invalid_or_stale_policy_fails_closed(self):
        self.machine.start()

        result = self.machine.apply_policy(
            action=ACTION_PROCEED,
            status=STATUS_STALE,
            evidence_id="aisle_a-1.000000000-000001",
            now=1.0,
        )

        self.assertFalse(result.accepted)
        self.assertTrue(result.cancel_motion)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.SAFETY_STOPPED,
        )

    def test_stop_requires_fresh_policy_and_resume(self):
        self.machine.start()

        stop_result = self.machine.apply_policy(
            action=ACTION_STOP,
            status=STATUS_VALID,
            evidence_id="aisle_a-1.000000000-000001",
            now=1.0,
        )

        self.assertFalse(stop_result.accepted)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.SAFETY_STOPPED,
        )

        stored_policy = self.machine.apply_policy(
            action=ACTION_PROCEED,
            status=STATUS_VALID,
            evidence_id="aisle_a-1.100000000-000002",
            now=1.1,
        )

        self.assertTrue(stored_policy.accepted)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.SAFETY_STOPPED,
        )
        self.assertFalse(self.machine.snapshot.motion_authorized)

        resumed = self.machine.resume(1.2)

        self.assertTrue(resumed.accepted)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.MOVING,
        )
        self.assertTrue(self.machine.snapshot.motion_authorized)

    def test_pause_invalidates_policy_before_resume(self):
        self.machine.start()
        self.machine.apply_policy(
            action=ACTION_PROCEED,
            status=STATUS_VALID,
            evidence_id="aisle_a-1.000000000-000001",
            now=1.0,
        )

        paused = self.machine.pause()

        self.assertTrue(paused.accepted)
        self.assertTrue(paused.cancel_motion)
        self.assertEqual(self.machine.snapshot.state, MissionPhase.PAUSED)
        self.assertFalse(self.machine.snapshot.motion_authorized)

        resumed = self.machine.resume(1.1)

        self.assertTrue(resumed.accepted)
        self.assertTrue(resumed.publish_goal)
        self.assertTrue(resumed.request_perception)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.WAITING_FOR_POLICY,
        )
        self.assertFalse(self.machine.snapshot.motion_authorized)

        fresh_policy = self.machine.apply_policy(
            action=ACTION_PROCEED,
            status=STATUS_VALID,
            evidence_id="aisle_a-1.200000000-000002",
            now=1.2,
        )

        self.assertTrue(fresh_policy.accepted)
        self.assertEqual(self.machine.snapshot.state, MissionPhase.MOVING)
        self.assertTrue(self.machine.snapshot.motion_authorized)

    def test_inspect_and_reroute_are_fail_closed(self):
        self.machine.start()

        inspect_result = self.machine.apply_policy(
            action=ACTION_INSPECT_CLOSER,
            status=STATUS_VALID,
            evidence_id="aisle_a-1.000000000-000001",
            now=1.0,
        )

        self.assertTrue(inspect_result.request_closer_view)
        self.assertTrue(inspect_result.cancel_motion)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.INSPECTING_CLOSER,
        )

        reroute_result = self.machine.apply_policy(
            action=ACTION_REROUTE,
            status=STATUS_VALID,
            evidence_id="aisle_a-1.100000000-000002",
            now=1.1,
        )

        self.assertEqual(
            reroute_result.reroute_request,
            "aisle_a",
        )
        self.assertTrue(reroute_result.cancel_motion)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.REROUTING,
        )

        target_result = self.machine.set_reroute_target("aisle_a_alt")

        self.assertTrue(target_result.accepted)
        self.assertTrue(target_result.publish_goal)
        self.assertTrue(target_result.request_perception)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.WAITING_FOR_POLICY,
        )
        self.assertEqual(
            self.machine.snapshot.active_goal,
            "aisle_a_alt",
        )

    def test_policy_age_does_not_shorten_reroute_response_window(self):
        machine = MissionStateMachine(
            ["aisle_a"],
            policy_timeout_seconds=2.0,
            reroute_timeout_seconds=0.5,
        )
        machine.start()

        reroute = machine.apply_policy(
            action=ACTION_REROUTE,
            status=STATUS_VALID,
            evidence_id="aisle_a-9.200000000-000001",
            now=10.0,
            policy_reference=9.2,
        )

        self.assertTrue(reroute.accepted)
        self.assertEqual(machine.snapshot.state, MissionPhase.REROUTING)
        self.assertIsNone(machine.watchdog(10.49))

        timeout = machine.watchdog(10.51)
        self.assertIsNotNone(timeout)
        self.assertTrue(timeout.cancel_motion)
        self.assertEqual(
            machine.snapshot.state,
            MissionPhase.SAFETY_STOPPED,
        )

    def test_watchdog_and_emergency_stop(self):
        self.machine.start()
        self.machine.apply_policy(
            action=ACTION_PROCEED,
            status=STATUS_VALID,
            evidence_id="aisle_a-1.000000000-000001",
            now=1.0,
        )

        watchdog_result = self.machine.watchdog(2.0)

        self.assertIsNotNone(watchdog_result)
        self.assertTrue(watchdog_result.cancel_motion)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.SAFETY_STOPPED,
        )

        emergency_result = self.machine.set_emergency_stop(True)

        self.assertTrue(emergency_result.cancel_motion)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.EMERGENCY_STOPPED,
        )

        self.machine.set_emergency_stop(False)

        reset_result = self.machine.reset()

        self.assertTrue(reset_result.accepted)
        self.assertEqual(
            self.machine.snapshot.state,
            MissionPhase.IDLE,
        )


if __name__ == "__main__":
    unittest.main()

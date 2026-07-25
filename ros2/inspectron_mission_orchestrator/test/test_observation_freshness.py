from __future__ import annotations

from inspectron_mission_orchestrator.node import (
    MAX_REJECTED_FUTURE_POLICY_OBSERVATIONS,
    MissionOrchestratorNode,
)


class _FreshnessState:
    _prune_rejected_future_policy_observations = (
        MissionOrchestratorNode._prune_rejected_future_policy_observations
    )
    _remember_rejected_future_policy_observation = (
        MissionOrchestratorNode._remember_rejected_future_policy_observation
    )
    _extend_future_policy_quarantine = MissionOrchestratorNode._extend_future_policy_quarantine
    _reject_cached_future_policy_source = (
        MissionOrchestratorNode._reject_cached_future_policy_source
    )
    _reject_policy_source = MissionOrchestratorNode._reject_policy_source

    def __init__(self):
        self.policy_timeout_seconds = 1.0
        self.policy_timeout_ns = 1_000_000_000
        self._last_policy_observed_ns = 0
        self._policy_recovery_barrier_ns = 0
        self._rejected_future_policy_observations: set[int] = set()
        self._rejected_future_policy_quarantine_until: float | None = None


def test_future_rejection_state_is_bounded_and_pruned_safely():
    state = _FreshnessState()
    current_ns = 10_000_000_000
    monotonic_now = 100.0
    first_future_ns = current_ns + 1

    for offset in range(MAX_REJECTED_FUTURE_POLICY_OBSERVATIONS + 1):
        state._remember_rejected_future_policy_observation(
            first_future_ns + offset,
            current_ns,
            monotonic_now,
        )

    assert not state._rejected_future_policy_observations
    assert (
        state._rejected_future_policy_quarantine_until
        == monotonic_now + state.policy_timeout_seconds * 2
    )

    state._prune_rejected_future_policy_observations(
        current_ns,
        monotonic_now + state.policy_timeout_seconds * 2,
    )
    assert (
        state._rejected_future_policy_quarantine_until
        == monotonic_now + state.policy_timeout_seconds * 2
    )

    state._prune_rejected_future_policy_observations(
        current_ns + 1,
        monotonic_now + state.policy_timeout_seconds * 2 + 1e-6,
    )
    assert state._rejected_future_policy_quarantine_until is None
    assert state._policy_recovery_barrier_ns == current_ns + 1


def test_admitted_high_water_clears_covered_future_rejections():
    state = _FreshnessState()
    current_ns = 20_000_000_000
    rejected_future_ns = current_ns + 100_000_000
    state._remember_rejected_future_policy_observation(
        rejected_future_ns,
        current_ns,
        200.0,
    )

    state._last_policy_observed_ns = rejected_future_ns + 1
    state._prune_rejected_future_policy_observations(
        current_ns,
        200.0,
    )

    assert not state._rejected_future_policy_observations
    assert state._policy_recovery_barrier_ns == current_ns


def test_active_quarantine_extends_only_for_future_inputs():
    state = _FreshnessState()
    current_ns = 30_000_000_000
    state._rejected_future_policy_quarantine_until = 302.0

    assert state._extend_future_policy_quarantine(
        current_ns,
        current_ns,
        301.0,
    )
    assert state._rejected_future_policy_quarantine_until == 302.0

    assert state._extend_future_policy_quarantine(
        current_ns + 1,
        current_ns,
        301.0,
    )
    assert state._rejected_future_policy_quarantine_until == 303.0


def test_cached_future_replay_does_not_move_an_open_recovery_barrier():
    state = _FreshnessState()
    state._policy_recovery_barrier_ns = 40_000_000_000

    state._reject_cached_future_policy_source()

    assert state._policy_recovery_barrier_ns == 40_000_000_000

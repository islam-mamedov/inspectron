# ROS 2 Mission Orchestrator

The ROS 2 mission layer contains two packages:

- `inspectron_mission_msgs` defines the mission-state interface.
- `inspectron_mission_orchestrator` coordinates waypoint progress, perception,
  safety-policy decisions, rerouting, and operator controls.

The orchestrator does not directly publish robot velocity. The fail-closed
motion controller remains responsible for `/cmd_vel`.

## Mission states

| State | Meaning |
|---|---|
| `IDLE` | No mission is active |
| `WAITING_FOR_POLICY` | Waiting for valid evidence and a safety decision |
| `MOVING` | Motion is authorized toward the active goal |
| `PAUSED` | Paused by an operator |
| `INSPECTING_CLOSER` | Additional perception evidence is required |
| `REROUTING` | Waiting for an alternate route target |
| `SAFETY_STOPPED` | Safety policy or watchdog stopped the mission |
| `COMPLETED` | All required waypoints were completed |
| `ABORTED` | Mission was aborted by an operator |
| `EMERGENCY_STOPPED` | Emergency-stop input is active or latched |

## Input topics

| Topic | Type |
|---|---|
| `/inspectron/policy_decision` | `inspectron_safety_supervisor/msg/PolicyDecision` |
| `/inspectron/mission/waypoint_reached` | `std_msgs/msg/String` |
| `/inspectron/mission/reroute_target` | `std_msgs/msg/String` |
| `/inspectron/emergency_stop` | `std_msgs/msg/Bool` |

## Output topics

| Topic | Type |
|---|---|
| `/inspectron/mission/state` | `inspectron_mission_msgs/msg/MissionState` |
| `/inspectron/mission/waypoint_goal` | `std_msgs/msg/String` |
| `/inspectron/mission/cancel_motion` | `std_msgs/msg/Bool` |
| `/inspectron/mission/perception_request` | `std_msgs/msg/Empty` |
| `/inspectron/mission/closer_view_request` | `std_msgs/msg/Empty` |
| `/inspectron/mission/reroute_request` | `std_msgs/msg/String` |

The mission-state topic uses transient-local durability so a newly connected
observer receives the latest state. While motion is authorized, the
orchestrator republishes the same `MOVING` state as a heartbeat. Its
`updated_at` remains the logical transition time, so evidence reports
de-duplicate the heartbeat while the motion controller can still detect an
orchestrator failure with its steady-clock watchdog.

## Operator services

All operator services use `std_srvs/srv/Trigger`.

| Service | Behavior |
|---|---|
| `/inspectron/mission/start` | Start from the first configured waypoint |
| `/inspectron/mission/pause` | Cancel motion and pause the mission |
| `/inspectron/mission/resume` | Resume only under a fresh safe policy |
| `/inspectron/mission/abort` | Cancel motion and abort the mission |
| `/inspectron/mission/reset` | Return a stopped mission to idle |

## Fail-closed behavior

Motion authorization requires a valid and fresh `ACTION_PROCEED` or
`ACTION_SLOW_DOWN` decision whose evidence ID matches the active waypoint.
Freshness is measured from the policy's `source_observed_at` camera-observation
time, not from callback arrival. Missing, future-dated, or expired source times
are treated as stale decisions. The source age is deducted from the
steady-clock policy watchdog, so delayed delivery never grants an observation
a second full authorization lifetime. Camera, supervisor, and orchestrator
must share a synchronized ROS clock domain; missing time or clock skew fails
closed.

Policy observation times must advance strictly. Duplicate or regressing
decisions trigger a safety stop. A new temporal fault opens a recovery barrier
at the orchestrator's local rejection time; a well-formed decision already on
or before an active barrier is rejected without moving it again, allowing a
legitimate post-stop observation to recover after older queued results drain.

Rejected future timestamps are retained in a bounded 256-entry replay cache,
so replaying one after clock catch-up remains stale. Overflow starts a
fail-closed quarantine for twice `policy_timeout_ms`; future inputs extend it,
and release opens a new local barrier. This is bounded operational protection.
Permanent replay identity across arbitrary future intervals or process
restarts requires a publisher session and monotonic source sequence in the
message protocol.

The orchestrator cancels motion when:

- the safety supervisor requests stop, reroute, or closer inspection;
- a policy is invalid, stale, unknown, or associated with another waypoint;
- the policy watchdog expires during motion;
- a reported completed waypoint differs from the active goal;
- reroute selection times out;
- an operator pauses or aborts the mission;
- emergency stop becomes active.

A safety-stopped mission never automatically restarts. It requires a fresh
motion-authorizing policy and an explicit resume request.

Pausing invalidates the previous motion policy. Resume republishes the active
goal, requests new perception, and remains in `WAITING_FOR_POLICY` until a new
matching decision authorizes motion.

After emergency stop is cleared, the mission remains latched and must be reset
before another mission can start.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `waypoints` | `aisle_a`, `aisle_b`, `aisle_c` | Ordered required mission goals |
| `policy_timeout_ms` | `750` | Maximum policy age while moving |
| `reroute_timeout_ms` | `5000` | Maximum wait for a reroute target |
| `watchdog_rate_hz` | `20.0` | Mission watchdog frequency |
| `state_heartbeat_rate_hz` | `4.0` | Authorized-motion state heartbeat frequency |

Invalid parameters cause startup to fail.

## Validation

The package includes:

- pure state-machine transition tests;
- evidence and waypoint mismatch tests;
- policy-watchdog tests;
- emergency-stop latching tests;
- inspection and reroute tests;
- pause-policy invalidation and resume tests;
- a ROS graph test that completes a two-waypoint mission;
- a full-pipeline reroute and pause/resume launch test;
- clean SIGINT and external-shutdown process-exit validation.

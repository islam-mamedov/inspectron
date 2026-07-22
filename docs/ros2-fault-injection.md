# ROS 2 Fault Injection

This suite extends `ros2/inspectron_e2e_simulation` (see
`docs/ros2-end-to-end-simulation.md` for the base architecture) with systematic
fault injection through the complete, unmodified production pipeline. Faults
are injected only at the simulator's boundary - the camera stream, the desired
velocity stream, and the waypoint-reached report - never by patching production
nodes or bypassing the perception bridge or safety supervisor.

## Fault matrix

| Fault | Injection | Expected fail-closed response | Test file |
| --- | --- | --- | --- |
| Camera stall (perception loss mid-motion) | simulator stops framing at the target goal | orchestrator policy watchdog -> `SAFETY_STOPPED` ("Policy watchdog expired during motion"), `/cmd_vel` -> 0 | `test_fault_watchdog_orchestrator_launch.py` |
| Camera stall, supervisor-first timeouts | same stall, inverted timeout configuration | supervisor `STATUS_STALE` STOP decision -> `SAFETY_STOPPED` ("Policy decision is invalid or stale"); stale decision preserved in the report | `test_fault_watchdog_supervisor_launch.py` |
| Desired-velocity stall (navigation dropout) | simulator suppresses `desired_cmd_vel` for 30 ticks mid-motion | controller command watchdog zeroes `/cmd_vel` within 250 ms while the mission stays `MOVING`; motion resumes; mission completes | `test_fault_recovery_launch.py` |
| Malformed camera frame | one frame with an invalid JPEG payload | bridge rejects it (no evidence capture), publishes a fail-closed assessment -> `INSPECT_CLOSER`/`weak_evidence` -> `INSPECTING_CLOSER` -> recovery -> `COMPLETED`; report shows exactly one assessment without evidence | `test_fault_recovery_launch.py` |
| Wrong-goal evidence | one valid frame tagged with a different waypoint (`intruder_zone`) | orchestrator evidence-prefix check -> `SAFETY_STOPPED` ("Policy evidence does not match the active mission goal"); mission never moves toward that goal | `test_fault_authority_launch.py` |
| Forged waypoint report | `waypoint_reached` published for a non-active goal mid-motion | `SAFETY_STOPPED` ("Reported waypoint does not match the active mission goal") | `test_fault_authority_launch.py` |
| Emergency stop | operator publishes `/inspectron/emergency_stop` mid-motion | `EMERGENCY_STOPPED`, zero velocity, reset rejected while active, report finalized with outcome `emergency_stopped`; clear + reset -> `IDLE` | `test_fault_authority_launch.py` |

The recovery test proves that recoverable faults (command dropout, one bad
frame) do **not** escalate: the mission ends `COMPLETED` with coverage 1.0 and
zero stop or stale decisions in its report. The other tests prove that
non-recoverable faults always end in a stopped state with zero residual
velocity and a fully preserved evidence trail.

## Fault plan semantics

Faults are simulator parameters (`fault_*`), grouped into a validated
`FaultPlan` in `scenario_logic.py`:

- each fault targets one goal **index** and fires **once per simulator
  process** (one-shot), which is what lets the authority test consume
  different faults across sequential missions in one launched graph;
- at most one fault may target a given goal index;
- the desired stall requires a positive tick count;
- mission waypoints may not collide with the `intruder_zone` fault waypoint;
- the priming frame is never faulted, so the pre-start pipeline check stays
  trustworthy.

`FAULT_HALTED` is a dedicated per-goal phase: once a stall or forgery fires,
the simulator freezes that goal's framing and completion handshake so the
production watchdogs - not the simulator - decide the outcome.

## Sequential missions in one graph

The authority test runs three missions in a single launched pipeline. Two
contracts make this deterministic:

1. **Finalize before reset.** The evidence reporter only starts a new recorder
   after the previous report is finalized, so each mission is aborted (or
   terminally stopped) and finalized before the operator reset.
2. **Re-prime after reset.** When the mission returns to `IDLE`, the simulator
   clears its per-goal bookkeeping, closes its start gate, and forgets the
   reporter's recording state. It then re-primes exactly like a fresh launch,
   and the test waits for a fresh first-waypoint decision before starting the
   next mission. This guarantees no camera frame ever straddles a report's
   recording boundary (which would orphan an assessment) and that the
   supervisor's staleness clock is fresh at every start.

## Timeout configurations

| Test | Orchestrator `policy_timeout_ms` | Supervisor `assessment_timeout_ms` | Why |
| --- | --- | --- | --- |
| watchdog (orchestrator) | 1000 | 4000 | orchestrator watchdog must win by 3 s |
| watchdog (supervisor) | 4000 | 1500 | supervisor staleness must win by 2.5 s |
| recovery, authority | 1500 | 3000 | standard simulation values |

The multi-second separations make the winning watchdog deterministic under CI
scheduling jitter. As everywhere in the simulation, timeouts are only widened
or reordered - never disabled - and every fail-closed path remains active.

## Running the suite in Docker

From the repository root (the package-selection command from
`docs/ros2-end-to-end-simulation.md` runs these tests automatically since they
live in the same package):

```bash
docker run --rm -v "$PWD":/workspace/inspectron ros:jazzy-ros-base bash -c '
set -eo pipefail
source /opt/ros/jazzy/setup.bash
apt-get update -qq
apt-get install -y -qq --no-install-recommends \
  ros-jazzy-ament-cmake-gtest ros-jazzy-ament-cmake-pytest \
  ros-jazzy-launch-testing-ament-cmake >/dev/null
mkdir -p /tmp/inspectron_ros_ws && cd /tmp/inspectron_ros_ws
colcon build \
  --base-paths /workspace/inspectron/ros2 \
  --packages-up-to inspectron_e2e_simulation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DINSPECTRON_ROS_WARNINGS_AS_ERRORS=ON
colcon test \
  --base-paths /workspace/inspectron/ros2 \
  --packages-select inspectron_e2e_simulation \
  --event-handlers console_direct+
colcon test-result --verbose
'
```

Expected: `100% tests passed, 0 tests failed out of 9` - three pytest suites
plus six launch tests (the two end-to-end scenarios and the four fault suites),
finishing in roughly twenty seconds of test time.

## Limitations

- Faults are injected at the simulator boundary; in-node fault injection
  (memory pressure, thread stalls inside production nodes) is out of scope.
- Reroute and pause/resume mission paths are not yet exercised; a blocked-path
  fixture scenario would be the natural vehicle for reroute testing.
- The demonstration launch (`e2e_simulation.launch.py`) intentionally exposes
  only the `safe_mission` and `hazard_stop` scenarios; fault runs are
  test-driven because their assertions are what give them meaning.
- CI runs these tests via the existing ROS 2 Jazzy job; no workflow changes
  were needed because the package was already registered.

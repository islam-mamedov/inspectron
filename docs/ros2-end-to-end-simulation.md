# ROS 2 End-to-End Simulation

`ros2/inspectron_e2e_simulation` launches and verifies the complete Inspectron
pipeline with no robot, no camera, no GPU, no API keys, no internet access, and
no Ollama server:


All production nodes run unmodified. The only simulation-specific node is the
scenario simulator; perception uses the bridge's deterministic `fixture`
provider, so the VLM response comes from deterministic default and
waypoint-specific JSON documents and the whole run is offline and repeatable.

## Nodes and message flow

| Node | Package | Role in the simulation |
| --- | --- | --- |
| `scenario_simulator` | `inspectron_e2e_simulation` | camera + navigation + operator driver |
| `perception_bridge` | `inspectron_perception_bridge` | real bridge, `provider=fixture` |
| `safety_supervisor` | `inspectron_safety_supervisor` | real C++ policy enforcement |
| `mission_orchestrator` | `inspectron_mission_orchestrator` | real mission state machine |
| `motion_controller` | `inspectron_motion_controller` | real fail-closed velocity gate |
| `evidence_reporter` | `inspectron_evidence_reporter` | real evidence + report recorder |


After priming, the simulator publishes camera frames in **lockstep**: at most
one frame is in flight, and the next frame is published only after the previous
frame's evidence capture *and* scene assessment were both observed. Before the
first evidence acknowledgment, the simulator retries the priming frame every
500 ms so DDS discovery cannot strand the scenario if the first camera sample
is lost. The bridge's job queue (`maxsize=1`) silently drops frames while
inference is active; evidence acknowledgment restores lockstep before another
retry can be emitted. Each accepted frame therefore yields exactly one evidence
capture and one assessment with the same evidence ID. Each frame carries the
active mission goal in `header.frame_id`, which the bridge turns into the
`{waypoint}-{stamp}-{sequence}` evidence ID that the orchestrator later checks
against its active goal.

## Mission priming

The supervisor latches a fail-closed `STATUS_STALE` stop decision if no
assessment arrives within `assessment_timeout_ms`, and that stop would abort a
freshly started mission because its evidence ID cannot match the active goal.
The simulator therefore publishes one **priming frame** for the first waypoint
as soon as the graph is discovered, before the mission starts. The resulting
decision proves the full camera-to-policy chain works, opens the simulator's
start gate, and resets the supervisor's staleness clock. The reporter is not
recording while the mission is idle, so priming traffic never appears in the
final report.

## Scenarios

| | `safe_mission` | `hazard_stop` |
| --- | --- | --- |
| Fixture response | `clear`, no hazards, `proceed`, 0.95 / 0.90 | `restricted`, `human_in_path`, `stop`, 0.93 / 0.88 |
| Expected policy | `PROCEED`, `VALID`, `clear_path` | `STOP`, `VALID`, `critical_hazard` |
| Desired velocity | published only while motion is authorized | published continuously (adversarial) |
| Mission outcome | `COMPLETED`, coverage 1.0 | `SAFETY_STOPPED`, then operator abort, coverage 0.0 |
| Report outcome | `completed` | `aborted` |

The hazard fixture keeps confidence and view quality above the supervisor
thresholds on purpose: the stop must be attributed to the critical hazard, not
to weak evidence. Its continuous desired-velocity stream proves the motion
controller blocks an actively requesting navigation stack, not merely a silent
one.

The test-only adaptive scenario streams desired velocity continuously, starts
with the safe fixture, pauses during authorized motion, and proves
`/cmd_vel` remains zero despite the adversarial input. Resume must obtain a new
matching policy and mission authorization before motion returns. At `aisle_b`,
a waypoint-specific blocked/debris fixture produces `REROUTE`; the test holds
the system in `REROUTING`, verifies sustained zero motion, and then publishes
the alternate target `aisle_b_detour`. The completed goal sequence is
`aisle_a, aisle_b, aisle_b_detour, aisle_c`, and the finalized report contains
evidence for both the blocked route and its detour.

## How the simulator decides a waypoint was reached

For the active goal, in order:

1. The mission must report `MOVING` with `motion_authorized` for that goal.
2. At least `min_authorized_motion_samples` (default 5) non-zero `/cmd_vel`
   samples must be observed while authorized - motion was actually granted by
   the real controller, not assumed.
3. Frame streaming for the goal stops, and the in-flight frame (if any) must
   resolve (evidence + assessment observed).
4. The simulator waits until `MissionState.last_evidence_id` equals the final
   evidence ID of that goal, which proves the orchestrator has consumed the
   final policy decision for the goal.
5. `waypoint_reached` is published exactly once.

Steps 3-4 exist because the orchestrator safety-stops on any policy whose
evidence ID does not match the active goal. Without the drain-and-confirm
handshake, a decision for goal N could arrive after the orchestrator advanced
to goal N+1 and trip that check. The handshake makes the transition
deterministic without weakening the production check.

## Safety invariants demonstrated

1. Default output is zero velocity - both tests observe only zero `/cmd_vel`
   before the mission starts, while the controller publishes at 20 Hz.
2. Missing policy never authorizes motion - pre-start phase of both tests.
3. Invalid or stale policy never authorizes motion - controller gates on
   `STATUS_VALID` plus freshness; the hazard test observes zero motion under a
   continuous desired-velocity stream.
4. A policy for the wrong evidence or waypoint never authorizes mission
   progress - orchestrator evidence-prefix check, kept strict; the simulator
   handshake works within it.
5. Critical hazard results in stop - hazard test: `STOP`/`critical_hazard`
   decision, `SAFETY_STOPPED` mission state, zero motion.
6. Mission completion requires all waypoints - safe test asserts in-order
   goals `aisle_a, aisle_b, aisle_c` and `waypoint_index == waypoint_count`.
7. Terminal states leave no residual velocity - safe test waits for a trailing
   window of zero `/cmd_vel` after `COMPLETED`; hazard test observes all-zero
   throughout.
8. Evidence IDs stay correlated across capture, assessment, policy, mission
   state, and report records - asserted at the graph level and again inside
   the finalized report.
9. Reports cannot claim verified integrity unless evidence passes byte-count
   and SHA-256 checks - the tests independently recompute SHA-256 for every
   stored file and assert `integrity.verified`.
10. Pause invalidates pre-pause authorization - resume remains stopped until a
    new matching evidence/policy pair reaches the orchestrator.
11. Reroute is fail-closed - the blocked goal produces a reroute request and
    motion remains zero until an alternate target is explicitly supplied and
    separately authorized.
12. Mission authority is enforced at the controller boundary - fresh
    `PROCEED` policy and desired velocity still produce zero output while the
    mission is paused, rerouting, or complete, and the controller requires the
    mission and policy evidence IDs to match.

## Report artifacts

Each mission produces `report_directory/mission-<UTC>-<token>/` containing:

- `evidence/*.jpg` - immutable (0444) evidence images, one per accepted frame;
- `report.partial.json` - live progress file, removed at finalization;
- `report.json` - immutable final report: counts, coverage, unmatched-record
  lists, SHA-256 per evidence file, assessments, policy decisions, mission
  state history;
- `report.md` - immutable human-readable summary of the same data.

## Build and test in Docker

Run from the repository root on the host:

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

Expected: `100% tests passed, 0 tests failed out of 10` (three pytest suites
and seven launch tests), and a final summary with 0 errors, 0 failures, 0
skipped.

## Run the demonstration

```bash
docker run --rm \
  -v "$PWD":/workspace/inspectron \
  -v /tmp/inspectron-e2e-demo:/demo_reports \
  ros:jazzy-ros-base bash -c '
set -eo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /tmp/inspectron_ros_ws && cd /tmp/inspectron_ros_ws
colcon build \
  --base-paths /workspace/inspectron/ros2 \
  --packages-up-to inspectron_e2e_simulation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DINSPECTRON_ROS_WARNINGS_AS_ERRORS=ON
source install/setup.bash
ros2 launch inspectron_e2e_simulation e2e_simulation.launch.py \
  scenario:=safe_mission report_directory:=/demo_reports/safe
ros2 launch inspectron_e2e_simulation e2e_simulation.launch.py \
  scenario:=hazard_stop report_directory:=/demo_reports/hazard
'
```

Report artifacts land in `/tmp/inspectron-e2e-demo/` on the host. In demo mode
the simulator starts the mission itself through the operator start service,
aborts on safety stop in the hazard scenario, prints a summary, and exits;
launch then shuts everything down, so each command terminates on its own.

Expected success output (timestamps and IDs vary):

```
[scenario_simulator_node-6] [INFO] [...] [scenario_simulator]: Reported waypoint reached: aisle_a
[scenario_simulator_node-6] [INFO] [...] [scenario_simulator]: Reported waypoint reached: aisle_b
[scenario_simulator_node-6] [INFO] [...] [scenario_simulator]: Reported waypoint reached: aisle_c
[scenario_simulator_node-6] [INFO] [...] [scenario_simulator]: E2E_SCENARIO_COMPLETE scenario summary:
    terminal_state=completed report_finalized=True report_path=.../safe/mission-.../report.json
    cmd_vel_samples=27 nonzero_cmd_vel_samples=21 nonzero_before_authorizing_policy=0
    max_abs_linear_x=0.300 trailing_zero_cmd_vel_samples=1
```

and for the hazard scenario:

```
[mission_orchestrator_node-3] [WARN] [...] [mission_orchestrator]: Safety policy requires an immediate stop
[scenario_simulator_node-6] [INFO] [...] [scenario_simulator]: E2E_SCENARIO_COMPLETE scenario summary:
    terminal_state=aborted report_finalized=True report_path=.../hazard/mission-.../report.json
    cmd_vel_samples=11 nonzero_cmd_vel_samples=0 nonzero_before_authorizing_policy=0
    max_abs_linear_x=0.000 trailing_zero_cmd_vel_samples=11
```

`nonzero_before_authorizing_policy=0` is the key line in both: no motion
command ever preceded a valid authorizing policy.

## Timing parameters used by the scenarios

| Parameter | Production default | Scenario value | Reason |
| --- | --- | --- | --- |
| supervisor `assessment_timeout_ms` | 1500 | 3000 | headroom for CI scheduling jitter between priming and mission start |
| orchestrator `policy_timeout_ms` | 750 | 1500 | lockstep decisions arrive every ~100-200 ms; larger margin for CI stalls |
| controller `policy_timeout_ms` | 750 | 1500 | same margin as the orchestrator |
| controller `command_timeout_ms` | 250 | 250 | unchanged; simulator ticks at 20 Hz |

These are configuration values within each node's declared parameters. All
fail-closed behavior (missing, stale, invalid, hazard) is unchanged; timeouts
were widened, never disabled.

## Test isolation

Launch tests from different packages run concurrently under `colcon test`, and
several existing tests use unremapped production topic names. All simulation
launch tests therefore remap **every** production topic and service to a
per-test prefix (including `/test/e2e_safe/...`,
`/test/e2e_reroute_pause_resume/...`, and the fault-test prefixes) and add
per-scenario node name suffixes, so the simulation cannot consume from or
publish into any other test, in either direction.

## Clean shutdown

The mission orchestrator handles both `KeyboardInterrupt` and
`ExternalShutdownException`. It publishes the courtesy `cancel_motion` message
only while the ROS context remains valid, then destroys the node and shuts down
the context safely. Its launch test verifies a clean process exit with code 0.

## Limitations and the simulation/hardware boundary

- There is no physics: no pose, odometry, or kinematics. Waypoint arrival is a
  protocol decision made by the simulator after observing authorized motion,
  not a spatial event. Hardware integration replaces the simulator's camera,
  desired-velocity, and `waypoint_reached` roles with real sensors and
  navigation.
- Camera frames are 4-byte valid JPEG stubs and the VLM is a fixture; visual
  content is never interpreted. Real-model behavior is covered by the separate
  VLM benchmark suite, not by this simulation.
- The demonstration scenarios exercise the proceed and critical-hazard-stop
  paths. Reroute and pause/resume are test-driven because operator timing and
  assertions give those paths meaning. Watchdog timeouts, stale decisions,
  malformed frames, wrong-goal evidence, forged waypoint reports, command
  stalls, and emergency-stop recovery are covered by the fault-injection
  suite; see `docs/ros2-fault-injection.md`.
- Everything runs on one host with local DDS; network transport effects are
  out of scope.
- Repeating a waypoint name within one mission (for example a reroute back to
  an already visited target) is not supported by the simulator's per-goal
  bookkeeping.

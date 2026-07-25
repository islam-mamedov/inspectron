# ROS 2 Fail-Closed Motion Controller

The `inspectron_motion_controller` package converts an upstream desired
velocity into a safe robot command according to the latest decision from the
Inspectron safety supervisor and the mission authority granted by the
orchestrator.

## Topic graph

| Direction | Topic | Type |
|---|---|---|
| Input | `/inspectron/desired_cmd_vel` | `geometry_msgs/msg/Twist` |
| Input | `/inspectron/policy_decision` | `inspectron_safety_supervisor/msg/PolicyDecision` |
| Input | `/inspectron/mission/state` | `inspectron_mission_msgs/msg/MissionState` |
| Output | `/cmd_vel` | `geometry_msgs/msg/Twist` |

## Policy behavior

| Decision | Controller behavior |
|---|---|
| `ACTION_PROCEED` with valid, fresh, mission-authorized inputs | Clamp and forward the desired command |
| `ACTION_SLOW_DOWN` with valid, fresh, mission-authorized inputs | Clamp and scale the desired command |
| `ACTION_STOP` | Publish zero velocity |
| `ACTION_REROUTE` | Publish zero velocity |
| `ACTION_INSPECT_CLOSER` | Publish zero velocity |
| Invalid or stale decision | Publish zero velocity |
| Unknown action or status | Publish zero velocity |
| Missing decision | Publish zero velocity |
| Stale desired command | Publish zero velocity |
| Non-finite command | Publish zero velocity |
| Missing mission state | Publish zero velocity |
| Stale mission state | Publish zero velocity |
| Mission state other than `MOVING` | Publish zero velocity |
| `motion_authorized == false` | Publish zero velocity |
| Missing active goal or evidence ID | Publish zero velocity |
| Mission and policy evidence IDs differ | Publish zero velocity |

The controller uses `std::chrono::steady_clock` for all watchdogs so system
clock changes cannot make stale commands appear fresh.

The safety supervisor and orchestrator publish durable state for auditing and
late-joining observers, but the controller deliberately uses reliable,
volatile subscriptions for both authority inputs. Cached DDS samples therefore
cannot be mistaken for live authority when the controller first joins the
graph. The controller also validates DDS publication and receipt timestamps
before admitting policy, mission-state, or desired-command samples. Missing,
future-dated, or already-expired metadata actively revokes that input instead
of refreshing its watchdog. This prevents queued reliable samples from
reviving motion after a transport or executor interruption. An admitted
sample's publication/receipt age is deducted from its steady-clock lifetime;
delivery never grants a second full watchdog period.

Policy decisions carry the camera observation time in
`source_observed_at`. The controller validates that timestamp independently
and uses the oldest of the observation, DDS publication, and DDS receipt ages
as the policy's steady-clock reference. A freshly republished decision
therefore cannot turn old visual evidence into fresh actuator authority.

Policy observation times must advance strictly. A duplicate or regressing
source time revokes policy authority. A new temporal fault opens a recovery
barrier at the controller's local rejection time; well-formed decisions already
on or before that barrier remain stopped without moving it again, so queued
pre-stop messages cannot starve a post-stop recovery.

The controller retains rejected future timestamps in a bounded 256-entry
replay cache. Cache overflow starts a fail-closed quarantine for twice
`policy_timeout_ms`; future inputs extend it, and release opens a new local
barrier. This bounded window prevents clock-catch-up replay without allowing
unbounded memory growth. Permanent replay identity across arbitrary future
intervals or process restarts requires a publisher session and monotonic source
sequence at the protocol layer.

DDS publication/receipt timestamps use system time, while
`source_observed_at` uses the ROS clock domain shared with the camera and
controller. Those clocks must be synchronized across hosts; missing metadata
or clock skew fails closed. Once a sample is admitted, the controller uses a
steady-clock watchdog so later clock changes cannot extend its lifetime.

The controller starts without authority and requires all of the following on
every control tick:

1. the latest mission-state heartbeat is fresh;
2. the latest mission state is `MOVING`;
3. `motion_authorized` is true;
4. the active goal and mission evidence ID are non-empty;
5. the mission evidence ID matches the latest policy decision.

Policy and mission-state callbacks may arrive in either order. Correlating
their evidence IDs prevents a new policy from inheriting old mission authority
and prevents a new mission authorization from using an unrelated policy.
This barrier relies on the upstream evidence contract: each evidence ID is
non-empty, immutable, and unique to one observation.
The orchestrator refreshes an authorized `MOVING` state while motion remains
active. If those heartbeats stop, the mission-state watchdog revokes authority.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `policy_timeout_ms` | `750` | Maximum age of the latest policy decision |
| `command_timeout_ms` | `250` | Maximum age of the desired velocity |
| `mission_state_timeout_ms` | `1000` | Maximum age of the latest mission-authority heartbeat |
| `slow_scale` | `0.35` | Scale applied to slow-down commands |
| `max_linear_speed` | `0.40` | Absolute linear velocity limit |
| `max_angular_speed` | `0.80` | Absolute angular velocity limit |
| `control_rate_hz` | `20.0` | Safe-command publication frequency |

Invalid parameter values cause node startup to fail rather than silently
weakening the safety policy.

`mission_state_timeout_ms` must be longer than the orchestrator heartbeat
period (`1000 / state_heartbeat_rate_hz`) with enough scheduling and transport
margin. The defaults use a 1000 ms timeout and a 250 ms heartbeat period.
Overly tight tuning fails closed by publishing zero velocity.

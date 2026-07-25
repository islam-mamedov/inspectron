# ROS 2 Safety Supervisor

## Purpose

`inspectron_safety_supervisor` places the deterministic C++ policy behind a ROS
2 message boundary. A perception process publishes a structured scene
assessment. The supervisor validates it, invokes the independently tested C++
policy, and publishes the only action that downstream robot control is allowed
to consider.

The VLM does not publish velocity commands and this node does not trust the
model-recommended action. Malformed input and stale perception both produce an
explicit `stop` decision.

## Topics

Default input:

```text
/inspectron/scene_assessment
```

Message type:

```text
inspectron_safety_supervisor/msg/SceneAssessment
```

Default output:

```text
/inspectron/policy_decision
```

Message type:

```text
inspectron_safety_supervisor/msg/PolicyDecision
```

The decision records the enforced action, stable reason, model-policy override
flag, input-health status, evidence identifier, source observation time, and
decision time. The output publisher is reliable and transient-local so a late
subscriber receives the most recent safety state.

Before admitting an assessment, the supervisor validates three independent
timestamps against `assessment_timeout_ms`:

- `SceneAssessment.observed_at`, using the node's ROS clock;
- the DDS publication timestamp, using the system clock;
- the DDS receipt timestamp, using the system clock.

Zero, malformed, future-dated, expired, or unavailable temporal metadata
produces an immediate `STATUS_STALE` `stop` decision and does not refresh the
watchdog. For an admitted assessment, the oldest of those three ages is
deducted from its steady-clock lifetime. Delivery therefore never grants an
already-aged observation a second full watchdog period. Subsequent ROS or
system clock changes cannot extend the admitted lease.

Admitted observation times must also advance strictly. A duplicate or older
assessment produces `STATUS_STALE` `stop` and cannot overwrite a newer
fail-closed result when asynchronous inference completes out of order.
A new temporal fault opens a recovery barrier at the local rejection time;
recovery then requires an observation captured after that barrier. A
well-formed sample already on or before an active barrier is rejected without
moving the barrier again, so a slow queue of pre-stop inference results cannot
chase and starve a legitimate post-stop observation.

Future-dated observations are also held in a bounded replay cache. Replaying
the same timestamp after the ROS clock catches up remains fail-closed. More
than 256 distinct future timestamps starts a fail-closed quarantine for twice
`assessment_timeout_ms`; future inputs extend it, and its release opens a new
local recovery barrier. This is bounded operational replay defense, not
permanent message identity: durable replay protection across an arbitrary
future interval or process restart requires a publisher session identifier
and monotonic source sequence in the message protocol.

## Fail-closed behavior

The supervisor publishes `stop` when:

- the C++ policy detects an invalid probability or enum value;
- a critical hazard is reported;
- an assessment has missing, future, or expired temporal metadata;
- no assessment arrives before the watchdog timeout;
- an established assessment stream becomes stale.

Startup rejects empty topic names, non-positive watchdog timeouts, and policy
thresholds outside `[0, 1]`. A downstream motion controller must also default to
stop when the decision topic is unavailable; process death cannot be made safe
by an in-process callback.

The supervisor cannot stop for a hazard that perception completely misses. The
42-scene benchmark exposed this enforcement boundary, so the ROS 2 adapter does
not present deterministic policy as a replacement for perception quality.

Publishers must set `observed_at` to the source observation time and preserve
that value through inference and transport. Publisher and supervisor system
clocks must be synchronized for DDS publication-age validation; camera/ROS
clocks must likewise share the supervisor's ROS time domain. Clock skew and
missing timestamp support fail closed.

The Inspectron perception bridge preserves the camera acquisition timestamp
through asynchronous inference. Any replacement perception publisher must
honor the same contract; stamping inference completion time would conceal the
age this lease is designed to bound.

## Parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `input_topic` | `/inspectron/scene_assessment` | Structured perception topic |
| `decision_topic` | `/inspectron/policy_decision` | Enforced decision topic |
| `confidence_threshold` | `0.70` | Minimum accepted model confidence |
| `view_quality_threshold` | `0.60` | Minimum accepted image quality |
| `assessment_timeout_ms` | `1500` | Maximum temporal-metadata age and time without an admitted assessment |

## Build in ROS 2 Jazzy

The package targets ROS 2 Jazzy and C++20. From an Ubuntu ROS 2 environment:

```bash
mkdir -p /tmp/inspectron_ros_ws
cd /tmp/inspectron_ros_ws

source /opt/ros/jazzy/setup.bash

colcon build \
  --base-paths /path/to/inspectron/ros2 \
  --packages-select inspectron_safety_supervisor \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DINSPECTRON_ROS_WARNINGS_AS_ERRORS=ON

colcon test \
  --base-paths /path/to/inspectron/ros2 \
  --packages-select inspectron_safety_supervisor
colcon test-result --verbose
```

Run the node after sourcing the workspace:

```bash
source install/setup.bash
ros2 launch inspectron_safety_supervisor safety_supervisor.launch.py
```

## First integration scope

This milestone provides:

- typed ROS 2 assessment and decision messages;
- direct reuse of the C++ safety core;
- reliable, transient-local decisions;
- invalid-input rejection;
- source-observation and DDS metadata freshness admission;
- a steady-clock perception watchdog;
- message conversion tests;
- a graph-level launch test covering valid, invalid, and stale input;
- launch and parameter configuration.

GitHub Actions builds and tests the package inside the ROS 2 Jazzy container, in
addition to the existing Python and standalone C++ jobs. The next milestone is
a simulated robot controller that consumes the enforced decision.

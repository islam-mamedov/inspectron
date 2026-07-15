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

## Fail-closed behavior

The supervisor publishes `stop` when:

- the C++ policy detects an invalid probability or enum value;
- a critical hazard is reported;
- no assessment arrives before the watchdog timeout;
- an established assessment stream becomes stale.

Startup rejects empty topic names, non-positive watchdog timeouts, and policy
thresholds outside `[0, 1]`. A downstream motion controller must also default to
stop when the decision topic is unavailable; process death cannot be made safe
by an in-process callback.

The supervisor cannot stop for a hazard that perception completely misses. The
42-scene benchmark exposed this enforcement boundary, so the ROS 2 adapter does
not present deterministic policy as a replacement for perception quality.

## Parameters

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `input_topic` | `/inspectron/scene_assessment` | Structured perception topic |
| `decision_topic` | `/inspectron/policy_decision` | Enforced decision topic |
| `confidence_threshold` | `0.70` | Minimum accepted model confidence |
| `view_quality_threshold` | `0.60` | Minimum accepted image quality |
| `assessment_timeout_ms` | `1500` | Maximum time without an assessment |

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
- a steady-clock perception watchdog;
- message conversion tests;
- a graph-level launch test covering valid, invalid, and stale input;
- launch and parameter configuration.

GitHub Actions builds and tests the package inside the ROS 2 Jazzy container, in
addition to the existing Python and standalone C++ jobs. The next milestone is
a simulated robot controller that consumes the enforced decision.

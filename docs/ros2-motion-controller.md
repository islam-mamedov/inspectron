# ROS 2 Fail-Closed Motion Controller

The `inspectron_motion_controller` package converts an upstream desired
velocity into a safe robot command according to the latest decision from the
Inspectron safety supervisor.

## Topic graph

| Direction | Topic | Type |
|---|---|---|
| Input | `/inspectron/desired_cmd_vel` | `geometry_msgs/msg/Twist` |
| Input | `/inspectron/policy_decision` | `inspectron_safety_supervisor/msg/PolicyDecision` |
| Output | `/cmd_vel` | `geometry_msgs/msg/Twist` |

## Policy behavior

| Decision | Controller behavior |
|---|---|
| `ACTION_PROCEED` with valid, fresh inputs | Clamp and forward the desired command |
| `ACTION_SLOW_DOWN` with valid, fresh inputs | Clamp and scale the desired command |
| `ACTION_STOP` | Publish zero velocity |
| `ACTION_REROUTE` | Publish zero velocity |
| `ACTION_INSPECT_CLOSER` | Publish zero velocity |
| Invalid or stale decision | Publish zero velocity |
| Unknown action or status | Publish zero velocity |
| Missing decision | Publish zero velocity |
| Stale desired command | Publish zero velocity |
| Non-finite command | Publish zero velocity |

The controller uses `std::chrono::steady_clock` for both watchdogs so system
clock changes cannot make stale commands appear fresh.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `policy_timeout_ms` | `750` | Maximum age of the latest policy decision |
| `command_timeout_ms` | `250` | Maximum age of the desired velocity |
| `slow_scale` | `0.35` | Scale applied to slow-down commands |
| `max_linear_speed` | `0.40` | Absolute linear velocity limit |
| `max_angular_speed` | `0.80` | Absolute angular velocity limit |
| `control_rate_hz` | `20.0` | Safe-command publication frequency |

Invalid parameter values cause node startup to fail rather than silently
weakening the safety policy.

# Inspectron Architecture

## 1. Objective

Inspectron is an embodied Vision-Language Model system for autonomous site-safety assessment.

The system combines:

- multimodal scene understanding;
- a structured perception contract;
- deterministic safety reasoning;
- closed-loop agent behavior;
- robot-action validation;
- fail-safe mission termination;
- measurable safety evaluation.

The central design rule is that learned model output is advisory. Deterministic components retain authority over robot movement.

## 2. System Boundary

Inspectron currently accepts camera images and simulated robot state. It produces validated mission actions and an auditable assessment trace.

Current inputs:

- camera-frame path;
- waypoint and scene identifiers;
- required mission waypoints;
- robot position and blocked-waypoint state.

Current outputs:

- structured scene assessments;
- model-recommended actions;
- policy-enforced actions;
- robot action trace;
- safety-policy overrides;
- mission coverage and termination status;
- controlled runtime-failure information;
- evaluation and latency metrics.

Physical robot drivers, mapping, localization, and geometric path planning are outside the current implementation.

## 3. Data Flow

```mermaid
sequenceDiagram
    participant R as Robot adapter
    participant M as Mission orchestrator
    participant V as VLM perception
    participant P as Safety policy
    participant A as Site-safety agent
    participant G as Execution gate

    R->>M: CapturedFrame
    M->>V: Analyze image
    V-->>M: SceneAssessment
    M->>A: Observe assessment
    A->>P: Resolve safe action
    P-->>A: Enforced recommendation
    A->>G: Proposed robot action
    G-->>M: Allow or reject

    alt action allowed
        M->>R: Move, inspect, stop, or report
    else action rejected
        M->>R: Stop
    end

    opt runtime failure
        M->>R: Attempt fail-safe stop
    end
```

## 4. Core Components

### Scene-safety domain

`site_safety.py` defines:

- `Traversability`
- `HazardType`
- `RecommendedAction`
- `SceneAssessment`
- `resolve_safe_action`
- `find_consistency_violations`

This module contains no model or robot dependencies. It is the deterministic safety-policy core.

### VLM perception

`vlm.py` converts a `CapturedFrame` into a `SceneAssessment`.

Responsibilities:

- construct the multimodal site-safety prompt;
- require structured JSON output;
- validate supported traversability values;
- validate multi-label hazards;
- validate the recommended action;
- reject missing, malformed, or out-of-range fields.

The Ollama adapter additionally sends a JSON Schema as the structured output format.

### Site-safety agent

`agent.py` maintains mission state and converts assessments into robot actions.

It tracks:

- visited waypoints;
- views captured per waypoint;
- assessment history;
- policy override count.

Agent behaviors include:

- resetting state before every mission;
- rejecting assessments outside the mission;
- immediate stop for critical hazards;
- another inspection for weak evidence;
- stop after the maximum number of uncertain views;
- reduced-speed movement through restricted scenes;
- selection of another required waypoint after a blocked route;
- stopping when a route is blocked and no alternative remains;
- reporting only after required coverage.

### Mission orchestrator

`mission.py` implements the closed-loop mission:

```text
capture → perceive → validate metadata → observe → decide → validate action → execute → repeat
```

The mission terminates with one of:

- `completed`
- `safety_stop`
- `validation_failure`
- `runtime_failure`
- `step_limit`

Robot and perception implementations are injected through protocols, keeping mission logic independent of Ollama, ROS 2, or the simulator.

The mission validates that the waypoint and evidence identifiers returned by perception match the captured frame. This prevents a faulty perception adapter from claiming that the robot assessed a location it never visited.

### Execution safety gate

`safety.py` validates proposed robot actions immediately before execution.

It enforces these invariants:

- movement requires a target;
- movement requires an explicit validated speed scale;
- movement targets must belong to the mission;
- blocked waypoints cannot be entered;
- inspection can occur only at the current waypoint;
- reporting requires complete waypoint coverage;
- stopping is always permitted.

This gate protects the robot boundary even if an earlier component proposes an invalid action.

### Simulation adapters

`simulation.py` supplies deterministic robot and perception adapters.

The baseline warehouse scenario exercises:

- clear-path travel;
- reduced-speed travel around debris;
- uncertainty-driven reinspection;
- successful mission completion.

Regression scenarios additionally verify:

- immediate stop for critical hazards;
- stop when no reroute remains;
- controlled handling of initial perception failures;
- controlled handling of failures after robot movement;
- rejection of mismatched perception metadata;
- rejection of movement without a target or speed;
- clean agent state when an agent is reused.

Simulation provides repeatable integration tests without requiring robot hardware.

## 5. Structured Perception Contract

A scene assessment contains:

```json
{
  "traversability": "clear | restricted | blocked | unknown",
  "hazards": [
    "human_in_path | debris | liquid_spill | open_edge | fire_or_smoke | unstable_load"
  ],
  "recommended_action": "proceed | slow_down | stop | reroute | inspect_closer",
  "confidence": 0.0,
  "view_quality": 0.0
}
```

`confidence` and `view_quality` must be numeric values between `0` and `1`.

The parser rejects unsupported labels instead of silently converting them.

## 6. Deterministic Safety Policy

Safety rules are applied in priority order:

| Priority | Condition | Enforced action |
|---|---|---|
| 1 | Human in path, open edge, fire or smoke, or unstable load | `stop` |
| 2 | Traversability is blocked | `reroute` |
| 3 | Confidence below 0.70, view quality below 0.60, or unknown traversability | `inspect_closer` |
| 4 | Restricted traversability or another reported hazard | `slow_down` |
| 5 | Clear scene with no hazards | `proceed` |

The policy result takes precedence over the model recommendation.

Both values are preserved so evaluations can measure:

- model-policy disagreement;
- override frequency;
- unsafe motion before enforcement;
- unsafe motion after enforcement.

This creates a hybrid learned-and-symbolic architecture: the VLM interprets the scene while explicit rules own safety decisions.

## 7. Active Perception

A single image may be blurred, occluded, dark, or ambiguous.

When evidence is weak, the agent requests another observation instead of proceeding. The current default permits two views per waypoint. If uncertainty remains after the view limit, the mission stops.

This behavior turns perception into a closed-loop process rather than a one-shot classifier.

## 8. Motion Behavior

Policy recommendations are converted into robot actions:

| Enforced recommendation | Robot behavior |
|---|---|
| `proceed` | Move at full speed scale |
| `slow_down` | Move at speed scale `0.35` |
| `inspect_closer` | Capture another view |
| `reroute` | Select another required waypoint at speed scale `0.50` |
| `stop` | Stop the mission |

Every MOVE action must contain both:

- a target waypoint;
- an explicit speed scale between `0` and `1`.

A missing speed is never converted to full speed.

The present rerouting behavior is waypoint-based. A future navigation adapter will delegate geometric route selection to a robot navigation stack.

If the current route is blocked and no unvisited waypoint remains, the agent stops instead of incorrectly reporting successful completion.

## 9. Evaluation

The batch evaluator compares predictions with a labeled JSON manifest.

Reported metrics include:

- traversability accuracy;
- exact hazard-set match;
- hazard micro precision, recall, and F1;
- model action accuracy;
- enforced action accuracy;
- policy override count and rate;
- unsafe-motion counts;
- mean inference latency;
- per-sample errors.

Unsafe motion is currently counted when the expected action is `stop` or `reroute`, but the evaluated action would permit forward motion.

The most important comparison is:

```text
model unsafe motion count → enforced unsafe motion count
```

This measures whether the deterministic layer reduces unsafe model decisions.

Support for treating an incorrect movement recommendation during `inspect_closer` as unsafe motion is part of the next evaluation-hardening milestone.

## 10. Failure Handling

Implemented validation includes:

- invalid image paths;
- unsupported image types;
- oversized images;
- unreachable inference services;
- malformed service responses;
- invalid VLM JSON;
- missing schema fields;
- unsupported labels;
- invalid confidence ranges;
- mismatched frame and assessment metadata;
- illegal robot actions;
- movement without an explicit speed;
- incomplete mission coverage.

The mission runner converts unexpected exceptions into a controlled `runtime_failure` result. The result includes a `failure_reason` containing the exception type and message.

The robot stop operation is attempted from a `finally` block on every mission termination path, including:

- successful completion;
- a safety-policy stop;
- an action-validation failure;
- a perception failure;
- a robot-adapter failure;
- reaching the mission step limit.

If the stop operation also fails, its error is appended to `failure_reason`, and the mission remains in the `runtime_failure` state.

## 11. Mission State Isolation

A `SiteSafetyAgent` may be reused, but state from one mission must never influence another.

Before every mission, the orchestrator resets:

- visited waypoints;
- captured-view counts;
- assessment history;
- policy override count.

This prevents a second mission from incorrectly reporting completion based on waypoints visited during an earlier run.

Mission results copy their assessment and waypoint collections so later agent resets do not mutate historical results.

## 12. Proposed ROS 2 Boundary

The Python system should retain:

- VLM inference;
- prompt and schema management;
- scene-assessment parsing;
- experiment configuration;
- evaluation and observability.

A future ROS 2 integration should provide adapters for:

- camera frames;
- localization and waypoint state;
- navigation goals;
- velocity limits;
- emergency stop;
- assessment and action telemetry.

The mission orchestrator can continue using the existing protocols while concrete ROS 2 adapters replace the simulated components.

## 13. Planned C++ Component

C++ is not required for the current VLM prototype.

The best future C++ component is the final ROS 2 safety supervisor because it sits at the performance-sensitive robot-control boundary. It should:

- consume proposed actions;
- enforce waypoint and velocity constraints;
- reject stale perception results;
- apply timeouts;
- issue emergency stops;
- publish auditable safety decisions.

The VLM and experimental agent logic should remain in Python unless profiling demonstrates a concrete performance requirement.

## 14. Trust Boundaries

The components have different authority levels:

```text
VLM perception        → may describe and recommend
Safety policy         → may override recommendations
Agent                 → may propose mission actions
Execution safety gate → may approve or reject actions
Robot controller      → executes only approved actions
```

No unvalidated VLM output crosses directly into the robot-control interface.

Runtime errors also cannot silently bypass mission termination: the orchestrator records the failure and attempts to stop the robot.

## 15. Current Non-Goals

The current version does not claim:

- physical robot deployment;
- safety certification;
- a production-scale benchmark;
- confidence calibration;
- online mapping or localization;
- geometric path optimization;
- distributed model training;
- real-time edge performance.

These are explicit future milestones rather than hidden assumptions.
# C++ Safety Supervisor

## Objective

The C++ safety supervisor ports Inspectron's deterministic policy out of the
Python perception process. Learned perception can recommend an action, but the
supervisor independently resolves the action that downstream robot integration
is allowed to execute.

This first milestone is deliberately a pure C++ library. It has no ROS 2,
networking, model-runtime, or JSON dependency, so the safety rules can be built,
tested, and audited in isolation. A later ROS 2 adapter will translate messages
into `SceneAssessment` values and publish the resulting `PolicyDecision`.

## Enforcement order

Rules are evaluated in this order:

1. Invalid or non-finite input fails closed to `stop`.
2. A critical hazard forces `stop`.
3. A blocked noncritical path requires `reroute`.
4. Weak evidence or unknown traversability requires `inspect_closer`.
5. A restricted path or noncritical hazard requires `slow_down`.
6. A clear, hazard-free scene may `proceed`.

Critical hazards are:

- `human_in_path`
- `open_edge`
- `fire_or_smoke`
- `unstable_load`

`debris` and `liquid_spill` are noncritical hazards unless traversability or
evidence quality requires a more conservative result.

## Python parity

The valid-input decision order mirrors `inspectron.site_safety.resolve_safe_action`.
The C++ boundary adds explicit validation for non-finite values, invalid enum
values, and invalid policy thresholds. These cases cannot be represented by a
valid Python `SceneAssessment`, and C++ resolves them to `stop` rather than
continuing with undefined or malformed state.

Each decision includes:

- the enforced action;
- a stable machine-readable reason;
- whether the model recommendation was overridden.

The C++ implementation does not claim to recover hazards that perception never
reports. The 42-scene benchmark demonstrated that enforcement cannot contain a
complete perception miss. This boundary remains explicit rather than being
hidden by the port.

## Build and test

Requirements:

- CMake 3.20 or newer
- a C++20 compiler
- network access during the first configuration so CMake can fetch pinned
  GoogleTest v1.15.2

From the repository root:

```bash
cmake -S cpp -B build/cpp \
  -DCMAKE_BUILD_TYPE=Release \
  -DINSPECTRON_WARNINGS_AS_ERRORS=ON

cmake --build build/cpp --parallel
ctest --test-dir build/cpp --output-on-failure
```

To rebuild without tests:

```bash
cmake -S cpp -B build/cpp-release \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF

cmake --build build/cpp-release --parallel
```

## Test coverage

The GoogleTest suite covers:

- every critical hazard;
- policy precedence;
- blocked, restricted, unknown, and clear traversability;
- confidence and view-quality thresholds;
- noncritical hazards;
- model-policy disagreements;
- schema-compatible string values;
- NaN, infinity, out-of-range probabilities, invalid thresholds, and invalid
  enums failing closed.

## Next milestone

The ROS 2 adapter will:

1. subscribe to a structured scene-assessment topic;
2. reject missing or unknown schema values;
3. call this library without duplicating policy logic;
4. publish an auditable policy decision;
5. issue `stop` when parsing, timing, or node-health checks fail.

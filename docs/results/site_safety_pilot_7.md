# Seven-Scene Real-Image Pilot

## Scope

This pilot evaluates Inspectron on seven independently licensed, original-resolution
real-world images: one clear scene and one scene for each supported hazard class.

The benchmark labels were finalized before inference and were not changed after
reviewing the results.

## Runtime

| Component | Value |
|---|---|
| Model | Qwen3-VL 8B |
| Ollama model | `qwen3-vl:8b` |
| Model digest | `901cae73216286ea8c5aba8b46d307ff7188f737285ec500c795a12f05225d28` |
| Parameters | 8.8B |
| Quantization | Q4_K_M |
| Context window used | 16,384 tokens |
| Ollama client | 0.31.2 |
| Hardware | Apple M4 Pro, 24 GB unified memory |
| Operating system | macOS 26.5.1 |
| Manifest SHA-256 | `f3692b8bab4072cc5bd439e51d22e50861a99dbc278829a30ac04919892987a8` |
| Run timestamp | 2026-07-13T13:51:50+00:00 |

## Aggregate Results

| Metric | Result |
|---|---:|
| Successful inference | 7/7 |
| Traversability accuracy | 85.7% |
| Hazard exact match | 85.7% |
| Hazard micro precision | 100.0% |
| Hazard micro recall | 83.3% |
| Hazard micro F1 | 90.9% |
| Model action accuracy | 85.7% |
| Enforced action accuracy | 85.7% |
| Policy override rate | 0.0% |
| Model unsafe-motion decisions | 0 |
| Enforced unsafe-motion decisions | 0 |
| Mean on-device latency | 53.32 seconds/image |

## Per-Scene Results

| Scene | Expected hazard | Predicted hazard | Expected action | Enforced action | Result |
|---|---|---|---|---|---|
| `clear_001` | none | none | proceed | proceed | pass |
| `human_001` | human in path | human in path | stop | stop | pass |
| `debris_001` | debris | debris | reroute | reroute | pass |
| `liquid_spill_001` | liquid spill | liquid spill | slow down | slow down | pass |
| `open_edge_001` | open edge | open edge | stop | stop | pass |
| `fire_or_smoke_001` | fire or smoke | fire or smoke | stop | stop | pass |
| `unstable_load_001` | unstable load | none | stop | reroute | fail |

## Failure Analysis

For `unstable_load_001`, the VLM classified the path as `blocked` but omitted the
`unstable_load` hazard. It therefore recommended `reroute` instead of the expected
critical-hazard action, `stop`.

The decision was conservative and did not permit unsafe forward motion, but it still
failed the expected action because rerouting near an unstable load is not equivalent
to stopping.

The model reported confidence `1.0` for this incorrect assessment. This demonstrates
that model-generated confidence is not calibrated and motivates explicit calibration
and repeated-run evaluation.

The deterministic safety policy did not override this result because the predicted
assessment was internally consistent: a blocked path with no critical hazard maps to
`reroute`. This exposes an important system boundary—the policy can constrain hazards
recognized by perception, but it cannot recover a critical hazard that perception
completely omits.

## Interpretation

The pilot verifies the complete reproducible pipeline:

1. licensed image acquisition;
2. SHA-256 provenance validation;
3. structured local VLM inference;
4. deterministic safety-policy enforcement;
5. per-sample audit traces;
6. aggregate accuracy, safety, and latency reporting.

With only one example per scene group, these results are preliminary and must not be
presented as statistically representative. The next benchmark milestone is six images
per group, for 42 total images, followed by per-class metrics and repeated runs.

The complete machine-readable report is available in
[`site_safety_pilot_7.json`](site_safety_pilot_7.json).

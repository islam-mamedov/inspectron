# 42-Image Real-World Site-Safety Benchmark Results

Full evaluation of the frozen 42-scene benchmark: one single pass followed by a
three-run repeated evaluation. All labels were frozen at commit `cd4e48d`
before any inference on the completed set.

## Run configuration

| Item | Value |
|---|---|
| Model | `qwen3-vl:8b` (Q4_K_M, Ollama digest `901cae7321…`) |
| Hardware | Apple M4 Pro, 24 GB unified memory, Ollama 0.31.2 |
| Decoding | temperature 0, `num_ctx` 16384, `num_predict` 8192 |
| Manifest SHA-256 | `766720c5b5d22b9ef413738bb0d34413baa90fc377854896a2530ff47269fbaf` |
| Repeated run generated | 2026-07-14T16:01:54+00:00 |
| Mean latency | 54.5 s/image (run means 54.33–54.76, population std 0.18) |

### Disclosed configuration fix before the tracked runs

The first single pass failed on 4 of 42 scenes with empty model output after
80–130 s. Root cause: `num_predict` was 2048 and Qwen3-VL's thinking tokens
share that budget, so hard scenes exhausted it before emitting JSON. The
budget was raised to 8192 (commit `c8759e3`) and the truncated-run artifact
was retained locally. This is a capacity fix of the same class as the earlier
`num_ctx` fix; no prompt, model, or label was changed. All four previously
truncated scenes then produced correct enforced actions.

## Aggregate results (identical in all three runs)

| Metric | Result |
|---|---:|
| Successful inference | 126/126 across 3 runs |
| Enforced action accuracy | 90.5% (38/42) |
| Model action accuracy | 90.5% (38/42) |
| Hazard micro precision / recall / F1 | 90.5% / 80.9% / 85.4% |
| Hazard exact-set match | 73.8% (31/42) |
| Traversability accuracy | 64.3% (27/42) |
| Policy override rate | 0.0% |
| Unsafe motions (model / enforced) | 2 / 2 |
| Mean prediction agreement | 100.0% |
| Fully stable scenes | 42/42 |

Three runs at temperature 0 produced byte-identical joint predictions for
every scene: every metric has zero variance and only latency moves
(±0.18 s). On this stack, greedy decoding is reproducible in practice, which
makes the failures below *reliably* wrong rather than sampling noise.

## Per-class hazard detection

| Hazard | Support | Precision | Recall | F1 |
|---|---:|---:|---:|---:|
| liquid_spill | 7 | 1.00 | 1.00 | 1.00 |
| fire_or_smoke | 6 | 1.00 | 1.00 | 1.00 |
| debris | 8 | 0.70 | 0.88 | 0.78 |
| human_in_path | 14 | 0.91 | 0.71 | 0.80 |
| open_edge | 6 | 1.00 | 0.67 | 0.80 |
| unstable_load | 6 | 1.00 | 0.67 | 0.80 |

![Per-class hazard detection](per_class_hazard_metrics.svg)

## Traversability confusion (expected → predicted)

| | clear | restricted | blocked | error |
|---|---:|---:|---:|---:|
| **clear** | 6 | 0 | 0 | 0 |
| **restricted** | 2 | 13 | 12 | 0 |
| **blocked** | 0 | 1 | 8 | 0 |
| **unknown** | 0 | 0 | 0 | 0 |

The dominant confusion (restricted → blocked, 12 scenes) is conservative:
it can only tighten the enforced action. The two restricted → clear errors
are the dangerous direction, and they are exactly the two unsafe motions.

## Confidence calibration

Accuracy of the enforced action, binned by the model's self-reported
confidence (42 scenes; predictions identical across runs):

| Reported confidence | Scenes | Correct | Accuracy |
|---|---:|---:|---:|
| 0.90–0.94 | 7 | 6 | 85.7% |
| 0.95–0.99 | 26 | 26 | 100.0% |
| 1.00 | 9 | 6 | 66.7% |

Maximal confidence is the *least* reliable bin: three of the four failures
were reported at confidence 1.0, and wrong answers average higher confidence
(0.975) than correct ones (0.950). Model-reported confidence is not usable
as a safety signal without calibration, which is why the deterministic
policy layer treats it only as a weak-evidence trigger, never as permission.

![Confidence calibration](confidence_calibration.svg)

## Failures (stable across all three runs)

| Scene | Ground truth | Prediction | Confidence | Unsafe? |
|---|---|---|---:|---|
| open_edge_004 (rail platform, Maputo) | restricted / open_edge / stop | clear / — / proceed | 1.0 | **yes** |
| open_edge_006 (rail platform, Eluru) | restricted / open_edge / stop | restricted / debris / slow_down | 0.9 | **yes** |
| unstable_load_001 (overloaded truck) | restricted / unstable_load / stop | blocked / — / reroute | 1.0 | no |
| debris_002 (cluttered warehouse aisle) | restricted / debris / slow_down | clear / — / proceed | 1.0 | no |

Findings:

1. **The model does not recognize railway platform edges.** Both platform
   scenes failed — one perceived as a completely clear corridor at
   confidence 1.0 directly toward an unguarded track-level drop. The
   harbour-quay drop (open_edge_005) and all street-level openings
   (manhole, road collapse, sinkhole works) were detected. The failure is
   specific to platform geometry, plausibly because platforms are benign
   in typical training imagery.
2. **The policy never fired, and that is the central result.** Override
   rate 0.0 means the model was internally self-consistent on all 42
   scenes; when it omits a hazard entirely, output-side enforcement is
   structurally blind. Enforcement bounds *inconsistency*, not *ignorance* —
   residual risk after this architecture is a perception-recall problem.
3. **unstable_load_001 reproduces the pilot failure exactly** (third
   consecutive identical result across five total runs), confirming a
   stable perception gap rather than decoding noise.
4. **human_in_path recall (0.71) fails on people near equipment**: the
   model omitted firefighters in three fire scenes and pedestrians beside
   vehicles in two others, while detecting every human in the six scenes
   where a person is the primary subject. It appears to treat "people who
   belong to the scene" as scenery. Actions were unaffected only because
   fire already forces stop.
5. **unstable_load_003 is right for the wrong reason**: it stops, but
   because it saw the worker, not the load. Action-level metrics alone
   would hide this; the exact-set metric catches it.
6. **debris precision (0.70)** reflects over-predicting debris on wet or
   cluttered-but-labeled scenes (both extra predictions were on liquid
   spills, action-harmless).

## Limitations

- 42 scenes, one image per scene; group-level rates carry wide intervals
  and no claim of statistical representativeness is made.
- Single imagery source (Wikimedia Commons) and photographer-selected
  viewpoints; several groups skew geographically.
- Determinism is claimed only for this exact stack (model build, Ollama
  0.31.2, Metal backend, temperature 0), not for the model in general.
- Ground truth is single-annotator with a documented rulebook and frozen
  labels; annotation notes in `benchmarks/DATASET.md` record every
  judgment call.

Machine-readable report:
[`site_safety_42_repeated_3.json`](site_safety_42_repeated_3.json).
The seven-scene pilot that preceded this benchmark is documented in
[`site_safety_pilot_7.md`](site_safety_pilot_7.md).

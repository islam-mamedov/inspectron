# Inspectron Real-Image Site-Safety Benchmark

## Purpose

This benchmark evaluates whether Inspectron can transform real site images into:

- structured multi-label hazard assessments;
- traversability classifications;
- safe navigation recommendations;
- deterministic policy-enforced actions.

The benchmark is designed for evaluation only. Images in this benchmark must not be
used for prompt tuning, model selection, fine-tuning, or threshold selection.

## Initial scope

Version 0.1 targets 42 real images divided into seven primary scene groups:

| Primary scene group | Target images |
|---|---:|
| Clear path | 6 |
| Human in robot path | 6 |
| Debris | 6 |
| Liquid spill | 6 |
| Open edge | 6 |
| Fire or smoke | 6 |
| Unstable load | 6 |
| **Total** | **42** |

Images may contain multiple hazards. The primary scene group is used only to keep
the dataset reasonably balanced.

This is a portfolio-scale smoke benchmark, not a statistically representative
industrial safety dataset.

## Label taxonomy

### Traversability

- `clear`: the planned robot corridor is visibly safe and unobstructed.
- `restricted`: motion may continue only at reduced speed.
- `blocked`: the planned route cannot be used safely.
- `unknown`: the image does not provide sufficient evidence.

### Hazards

- `human_in_path`: a person occupies or is entering the robot travel corridor.
- `debris`: loose material or objects interfere with safe movement.
- `liquid_spill`: visible liquid may reduce traction or indicate leakage.
- `open_edge`: an exposed drop, excavation, shaft, or unprotected platform edge.
- `fire_or_smoke`: visible flame or smoke.
- `unstable_load`: a load appears unsecured, leaning, suspended, or at risk of falling.

A person visible outside the robot corridor is not automatically labeled
`human_in_path`.

### Expected actions

Expected actions are derived from Inspectron's deterministic safety policy:

1. Critical hazards force `stop`.
2. Blocked paths require `reroute`.
3. Unknown or insufficient evidence requires `inspect_closer`.
4. Restricted paths and noncritical hazards require `slow_down`.
5. Clear scenes permit `proceed`.

The benchmark does not manually override the deterministic policy.

## Data sources

### Open Images

Open Images is the preferred source for clear paths, people, debris, spills,
edges, and unstable loads.

Its annotations are distributed under CC BY 4.0. Images are listed as CC BY 2.0,
but the Open Images documentation states that the license of every selected image
should be verified individually.

- Dataset: https://storage.googleapis.com/openimages/web/index.html
- Downloads: https://storage.googleapis.com/openimages/web/download_v7.html
- License guidance: https://github.com/openimages/dataset/blob/master/READMEV3.md

For every selected Open Images item, record:

- Open Images ID;
- original author;
- original landing page;
- image license and license URL;
- local SHA-256 digest.

### Fire Recognition Image Dataset

Original, non-augmented images may be used for the `fire_or_smoke` group.

- DOI: https://doi.org/10.17632/7jk6xh7h6w.1
- License: CC BY 4.0

Augmented copies must not be included because correlated transformations would
inflate the apparent benchmark size.

### Wikimedia Commons

Wikimedia Commons may be used when a suitable Open Images scene is unavailable.

Each file page must be reviewed separately. Record the creator, file page, exact
license, license URL, and SHA-256 digest.

- Reuse guidance:
  https://commons.wikimedia.org/wiki/Commons:Reusing_content_outside_Wikimedia/en

## Provenance requirements

Every manifest record must contain:

- a unique benchmark sample ID;
- a safe relative local image path;
- ground-truth traversability;
- zero or more hazard labels;
- the policy-derived expected action;
- benchmark split;
- source dataset;
- source image ID;
- source landing page;
- original author;
- exact license name;
- license URL;
- SHA-256 digest of the downloaded image.

Records missing any required provenance field are rejected.

## Image storage

Downloaded images are stored under:

```text
benchmarks/data/

## Reconstructing local benchmark images

From the repository root, run:

```bash
PYTHONPATH=src python3 benchmarks/scripts/download_dataset.py \
  --manifest benchmarks/manifests/site_safety_real.json \
  --data-root benchmarks/data
```

The downloader:

- accepts HTTPS sources only;
- limits the maximum response size;
- accepts JPEG, PNG, and WebP images;
- rejects non-image responses;
- prevents paths from escaping the data directory;
- verifies every image against its tracked SHA-256 digest;
- skips network access when an existing image is already valid.

After acquisition, validate the complete benchmark:

```bash
PYTHONPATH=src python3 benchmarks/scripts/validate_dataset.py \
  --manifest benchmarks/manifests/site_safety_real.json \
  --data-root benchmarks/data
```
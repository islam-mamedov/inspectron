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
- a direct HTTPS download URL for byte-identical reacquisition;
- original author;
- exact license name;
- license URL;
- SHA-256 digest of the downloaded image.

Records missing any required provenance field are rejected.

Prefer original-resolution upload URLs over server-generated thumbnail URLs.
Thumbnail renditions (for example Wikimedia `/thumb/...` URLs) may be
re-rendered when the hosting platform changes its image scaler, which would
break the pinned SHA-256 digest and make the sample unrecoverable.

## Image storage

Downloaded images are stored under:

```text
benchmarks/data/
```

Everything beneath `benchmarks/data/` except `.gitkeep` is ignored by Git.
Third-party images are never committed; they are reconstructed from the
manifest with the downloader below.

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

## Candidate review log

Candidates that were downloaded, reviewed, and rejected are recorded here so
they are not re-acquired or silently relabeled later. Rejected files may stay
locally under `benchmarks/data/candidates/`; they are never added to the
manifest and never committed.

### Rejected: `candidates/clear_candidate_001.jpg`

- Source: Wikimedia Commons, `File:Empty warehouse (4589222821).jpg`
- Source page: https://commons.wikimedia.org/wiki/File:Empty_warehouse_(4589222821).jpg
- Author: nick fullerton; license: CC BY 2.0
- SHA-256: `86432631686ba473e463806e18b3e660e69f02704f0df62ad28547bebcb30e99`
- Originally acquired as a clear-path candidate.

Rejection rationale:

1. It is not a clear-path scene. The floor of the abandoned warehouse carries
   scattered loose debris across the travel corridor, so `clear` is ruled out
   by the taxonomy.
2. Relabeling it is not defensible either, because independent annotators
   would not converge on one ground truth. Traversability is a coin flip
   between `restricted` (the visible foreground corridor is passable at
   reduced speed) and `unknown` (large parts of the corridor are in darkness
   and cannot be assessed), and those labels map to different policy actions
   (`slow_down` versus `inspect_closer`).
3. Leaning pallets against columns and a partially detached fixture hanging
   from the ceiling make `unstable_load` a plausible but not clearly correct
   additional label. Because `unstable_load` is a critical hazard, including
   or excluding it flips the expected action across the stop boundary
   (`stop` versus a non-stop action), which is the most safety-critical part
   of the label.
4. A benchmark sample whose expected action could defensibly be `slow_down`,
   `inspect_closer`, or `stop` measures the annotator's choice rather than
   model competence, so it fails the reliability bar for ground truth.

### Rejected: `candidates/clear_cand_004.jpg`

- Source: Wikimedia Commons, `File:Hochregallager.jpg`
- Author: Heinrich Taxis GmbH + Co. KG; license: CC BY-SA 4.0
- SHA-256: `ed755a0ae77093ccfc00b7c65b27d05cb19b67791d2d3d630ede2c37acdbba61`
- Acquired as a clear-path candidate for the 42-image expansion.

Rejection rationale: a forklift with a pallet occupies the full width of the
very-narrow aisle at mid-distance, so the scene is not clear. The honest label
would be `blocked` with no listed hazard (a vehicle is not in the hazard
taxonomy), which fits none of the seven scene groups, and the obstruction is
barely visible in the dim far field, failing the evidence-quality bar.

### Rejected: `candidates/human_cand_002.jpg`

- Source: Wikimedia Commons, `File:Defense.gov photo essay 080605-F-3798Y-294.jpg`
- Author: Tech Sgt. Cohen A. Young; license: Public Domain (US DoD)
- SHA-256: `37be998e614a98d7d1410dbab1043de866674afc7b06e5c7e24ea189cb11fcf6`
- Acquired as a human-in-path candidate for the 42-image expansion.

Rejection rationale: night-vision intensifier imagery (monochrome green
phosphor, heavy blur) is a different sensor modality from the RGB robot
camera this benchmark represents, so the sample would not measure the
deployed perception path.

### Rejected: `candidates/load_cand_004.jpg`

- Source: Wikimedia Commons, `File:Slidell after Katrina - overloaded pick up truck.jpg`
- Author: Steve Wilson; license: CC BY 2.0
- SHA-256: `24044c413769a7242908271d2a7b9c0609bb1203806038253d726ffab489c79e`
- Acquired as an unstable-load candidate for the 42-image expansion.

Rejection rationale: the photograph shows a hurricane-collapsed house resting
on a crushed pickup, viewed side-on with no coherent robot travel corridor.
The unstable element is building fabric, which this dataset's precedent
(see `clear_candidate_001`) does not treat as an `unstable_load`, and the
corridor-dependent labels cannot be assigned reliably.

### Rejected: `candidates/spill_cand_001.jpg`

- Source: not recoverable (no Wikimedia Commons SHA-1 match; the file was
  most likely acquired as a server-rendered thumbnail during the pilot
  session, so its bytes do not match any original upload)
- SHA-256: `269b98f3e18d4467d603b74dcefcace44d71d87ebd2ef69cb22843b467959419`
- Acquired as a liquid-spill candidate during the pilot session.

Rejection rationale: steep top-down close-up of a floor being squeegeed
toward a drain, with no robot travel corridor in frame; additionally the
original source, author, and license cannot be verified, which independently
fails the provenance requirements.

### Rejected: `candidates/edge_cand_002.jpg`

- Source: Wikimedia Commons, `File:SEWER - Open Manhole - New Orleans September 2020.jpg`
- Author: Bart Everson; license: CC BY 2.0
- SHA-256: `aba1f83324c0b9941ddd78e1d1b7d809687174ecad60abbaaf80ca2f619b020b`

Rejection rationale: straight-down close-up of the same open manhole already
accepted as `open_edge_001`, photographed by the same author at the same
event; no corridor context and a near-duplicate scene.

### Rejected: `candidates/edge_cand_003.jpg`

- Source: Wikimedia Commons, `File:Hard-manhole-open-01ASD.jpg`
- Author: Asurnipal; license: CC BY-SA 4.0
- SHA-256: `4863639435cedc08890e0344345392e908c0a4b7277e90b1fb4229cd90c0b47f`

Rejection rationale: downward close-up of an open manhole without usable
travel-corridor geometry for an embodied navigation frame.

### Reserve: `candidates/fire_cand_002.jpg`

- Source: Wikimedia Commons, `File:Structure Fire in Union, Mississippi 04.jpg`
- Author: Ktkvtsh; license: CC BY 4.0
- SHA-256: `f4caaf482aa81f885f26280416e05c0fffd7716ee662189337b0adb8a50d7b45`

Held in reserve, not accepted: a different composition of the same fire
already represented by `fire_or_smoke_001`, with firefighters and hose lines
in the road corridor. Same-event near-duplicates weaken scene diversity, so
this file may be used only if six independent fire scenes cannot be sourced.

### Rejected: `candidates/spill_cand_003.jpg`

- Source: Wikimedia Commons, `File:633rd LRS, CES Airmen participate in fuel spill exercise 140722-F-YC840-020.jpg`
- Author: Senior Airman Aubrey White; license: Public Domain (US Air Force)
- SHA-256: `c9032a7d87913cac432db6eee8fb4c06a37f24a4938c2e9be9fce971ee244a69`

Rejection rationale: despite the title, the frame is a portrait of an airman
in a proximity suit beside a fire truck; no spill and no travel corridor are
visible.

### Rejected: `candidates/edge_cand_005.jpg`

- Source: Wikimedia Commons, `File:Sinkhole in Inverness, Florida.jpg`
- Author: The Eloquent Peasant; license: CC0 1.0
- SHA-256: `b199ee10474107d34109d88d926f1f2bcd71f5b7c229e6e9ad456520854b2973`

Rejection rationale: the collapse is a settling depression with narrow
surface cracks rather than a discrete exposed drop, so annotators could
defensibly assign either `open_edge` (forcing `stop`) or no listed hazard
(yielding `slow_down`). Ground truth that flips the stop boundary on a
judgment call fails the reliability bar.

### Rejected: `candidates/edge_cand_007.jpg`

- Source: Wikimedia Commons, `File:Site of the 2024 Kuala Lumpur sinkhole 07.jpg`
- Author: Ridiculopathy; license: CC0 1.0
- SHA-256: `946e5a61a4db014492a4756e16c65e2e316a0a08406c83bd1619d256ef254440`

Rejection rationale: near-duplicate of the same barricaded worksite accepted
as `human_003` (same event, author, and composition elements).

### Rejected: `candidates/fire_cand_003.jpg`

- Source: Wikimedia Commons, `File:BMW Car Fire (1623624284).jpg`
- Author: Tony Webster; license: CC BY 2.0
- SHA-256: `b1a698aee8aec175dfe034efee5a2cf0b9cbd1bec009866127bac6fa200d936f`

Rejection rationale: extinguished aftermath with a charred engine bay and no
visible flame or smoke, so the scene contains no `fire_or_smoke` evidence
under the written definition.

### Rejected: `candidates/fire_cand_004.jpg`

- Source: Wikimedia Commons, `File:Mustang car fire at CVS on Key West Highway in North Potomac MD July 12 2012 (7575647972).jpg`
- Author: Mark Taylor; license: CC BY 2.0
- SHA-256: `9951bdcdb220e26996b198f1e18b9ce43589cde3d14b1174cdbb797d5fb4db5d`

Rejection rationale: the fire is extinguished under foam with only trace
wisps, making `fire_or_smoke` contestable, while a firefighter and hose
lines occupy the near corridor and stack further label ambiguity.

## Label notes for accepted samples

Judgment calls on accepted records are logged here so annotation decisions
stay auditable and consistent.

- `unstable_load_002` (`File:India-Truck-Overload.jpg`): `open_edge` was
  considered for the mountain roadside and excluded — the visible left edge
  shows shadowed rock face and stacked sacks, not an exposed drop adjacent
  to the corridor.
- `debris_003` (`File:Flood eroded road.jpg`): `open_edge` was considered
  for the flood-eroded gully and excluded — the erosion is shallow uneven
  terrain, not a discrete exposed drop, excavation, or platform edge under
  the written definition.
- `debris_004` (`File:Lincoln St, Wellington flood aftermath, 20 Apr 2026.jpg`):
  `liquid_spill` was considered for the rain-wet pavement and excluded — the
  surface shows ambient wetness rather than a distinct pooled or flowing
  liquid, and treating rain-wet ground as a spill would mislabel every wet
  outdoor scene. The distant hi-vis worker near the traffic cones was not
  labeled `human_in_path` because they are far outside the near corridor.
- `open_edge_002` (`File:Sinkhole, Bolebrooke Road, Bexhill.jpg`): the figure
  at the far right stands behind the site fencing, outside the corridor, so
  `human_in_path` was not labeled.
- `human_003` (`File:Site of the 2024 Kuala Lumpur sinkhole 06.jpg`): the
  sinkhole itself is hidden behind water-filled barriers, so `open_edge` was
  not labeled (no visible drop); the excavator is machinery, which the
  taxonomy does not treat as an `unstable_load`.
- `fire_or_smoke_002` (`File:Staged car fire 5.JPG`): a controlled training
  burn — a real photograph of real combustion, not synthetic imagery. The
  firefighters on the hose line stand in the approach corridor and are
  labeled `human_in_path`; the crowd behind the caution tape is outside the
  corridor and is not.
# Tuniu step 1: camera-to-map fixes on real drone photos (results and evidence)

Date: 2026-10-03. Site: Tuniu River, Toufen, Miaoli, Taiwan. Everything below is MEASURED unless marked
SIMULATED or INFERENCE. Detailed French report by the step-1 agent: `data/processed/x_tuniu/report.md`
(git-ignored data folder; regenerate with the commands at the end).

## Bottom line

- **No originally pre-registered method passes the Level-1 criteria robustly** (≥ 30 % of test photos
  accepted, 0 accepted fixes wrong by > 10 m, ≤ 1 % of off-map negatives accepted).
- **The pre-registered XFeat + ZNCC consensus gate works**: about 30 % of photos accepted with 1–2 wrong
  fixes out of ~1,290 over 19 seeds, versus 65 wrong fixes for XFeat alone.
- **Where it fails is forest**: 96 % of photos are fixed when trees cover < 25 % of the view, 12 % when they
  cover > 75 %. 79 % of the viewed ground on this flight is tree cover (Taiwan: 76 %).

## Setup

- Photos: DJI Phantom 4 RTK survey, 2019-04-11, 271 photos, 100 m above take-off, camera 30° from nadir,
  RTK fixed (MRK log). GNSS is cut after photo 46; test = photos 47–271 (225).
- Estimator inputs after the cut: photo, time, DJI fused attitude (gimbal angles + boresight calibrated on
  photos 1–46), SIMULATED barometer (RTK altitude + measured noise model), SIMULATED prior = RTK + uniform
  ±40 m per axis, Copernicus GLO-30 DEM. RTK after the cut is used only to generate the simulated inputs,
  to place negatives and to score.
- Map: OpenAerialMap orthophoto of 2019-12-12 (3.5 cm, CC BY 4.0, 8 months after the flight), resampled to
  0.1–1 m/px; its 2 m georeferencing offset is calibrated on photos 1–46.
- Negatives: each photo is also matched against 3 map windows ≥ 300 m from the truth (675 per seed).
- Protocol: `data/processed/x_tuniu/preregistration.md`, written before any test-photo result; thresholds
  are fitted on photos 1–46 only.

## Results over 20 seeds (main map, 225 test photos and 675 negatives per seed)

| Method | Accepted (mean, min–max) | Wrong > 10 m (pooled) | Negatives accepted | Seeds passing |
|---|---|---|---|---|
| XFeat, inlier gate, 0.25 m | 36.1 % (22.2–51.1) | **65** in 14/20 seeds (median 17 m, max 314 m) | 2 / 13,500 | 3 / 20 |
| ZNCC + quad ≥ 3, 1 m, flat ground | 27.6 % (25.3–29.8) | 8 / 1,244 (10.3–11.8 m, one 24 m) | 0 / 13,500 | 0 / 20 |
| ZNCC + quad ≥ 3, 1 m, terrain-lifted (EXPLORATORY) | 32.5 % (30.2–34.2) | 3 / 1,461 | 1 / 13,500 | 17 / 20 |
| ALIKED + LightGlue, 0.5 m (seed 0 only) | 66.2 % | 4 | 0 / 675 | fails |

Median error of accepted fixes: about 2.5 m (p90 about 4.5 m).

## Pre-registered consensus gate (seeds 1–19)

Rule committed in `docs/research/tuniu-consensus-prereg.md` (commit `8e64a50`, 2026-10-03 17:04 Taipei;
the file text says "~17:10", the git timestamp is authoritative), before the XFeat seed 1–19 results were
read: accept iff XFeat and ZNCC fixes agree within 4 m; no fitted threshold. Evaluation:
`experiments/x_consensus_tuniu.py`.

| Resolution | Accepted (mean, min–max) | Wrong > 10 m | Negatives accepted | Median / p90 / max error | Seeds passing all three criteria |
|---|---|---|---|---|---|
| 0.5 m | 30.2 % (28.0–34.7) | 1 / 1,290 (≤ 0.37 %, 95 %) | 4 / 12,825 | 2.56 / 4.32 / 17.8 m | 7 / 19 |
| 0.25 m | 30.1 % (27.6–32.9) | 2 / 1,288 (≤ 0.49 %) | 1 / 12,825 | 2.44 / 4.03 / 12.6 m | 9 / 19 |

Pre-registered success criterion (wrong rate ≤ 0.205 % of the terrain-lifted ZNCC rule and mean accepted
≥ 30 %): **met at both resolutions**. Seeds that fail do so mainly by falling just under 30 % accepted.

## Input ablations (seed 0, ZNCC + quad, 1 m)

| Change | Accepted |
|---|---|
| No rectification (camera assumed straight down) | 5.3 %, and all 12 accepted fixes wrong (~53 m) |
| DJI attitude + 3° / + 7° heading error | 27.1 % / 18.7 % |
| Constant 100 m height (no barometer) | 21.8 % |
| Barometer − DEM (default) | 27.6 % |
| RTK height − DEM (oracle) | 29.3 % |
| Flat ground at take-off height | 18.7 % |
| Terrain-lifted ground | 33.8 % |

The ground under the flight is 57–107 m below the drone, not 100 m.

## Where it fails: forest and turns

Share of tree cover (ESA WorldCover 2021, 10 m) in each photo's viewed ground (10–100 m ahead, ±60 m),
against the share of seeds in which the photo is accepted (terrain-lifted ZNCC + quad, 20 seeds):

| Tree cover in view | Photos | Mean acceptance | Never accepted |
|---|---|---|---|
| 0–25 % | 14 | 96 % | 0 |
| 25–50 % | 26 | 77 % | 0 |
| 50–75 % | 38 | 57 % | 3 |
| 75–100 % | 147 | 12 % | 102 |

105 of 225 photos are never accepted in any seed: 87 on straight legs (dense canopy, checked visually on
photo 185) and 18 in turns. Fixes come in clusters; the longest gap per flight is about 60 s / 380 m
(p90 gap 25 s / 177 m). Over such a gap the position comes from dead reckoning only.

## Speed from consecutive photos (stage 4, 20 seeds, 0.25 m)

Pre-registered: median speed error 0.66 m/s (9.6 %), p90 40 %. Exploratory (shared ground plane or
terrain-lifted, 5 seeds): 0.37 m/s (5.3 %). INFERENCE: across a 400 m forest gap this is 20–40 m of drift,
close to the ±45 m search window, so photo-only odometry is not enough; an IMU or VIO is needed.

## Latency (Mac, 1 thread, NOT Jetson)

Decode + rectify + match per photo: ZNCC 0.08 / 0.14 / 0.45 s and XFeat 0.08 / 0.13 / 0.33 s at
1 / 0.5 / 0.25 m/px; ALIKED-LightGlue 0.44 / 2.0 / 5.8 s; RoMa 48 s per pair.

## Evidence anyone can check

1. Independent audit (`experiments/x_audit_tuniu.py`, written separately from the pipeline): re-parses the
   raw DJI MRK log, recomputes every error (max difference 0.000 m), reproduces all 153 summary rows,
   checks that no RTK after the cut reaches the estimator, and computes the chance baseline (prior alone:
   median error 31.9 m, 4.9 % of photos within 10 m). Result: ALL PASS.
2. Visual checkerboards (`experiments/x_gallery_tuniu.py`): rectified photo squares alternating with map
   squares at the fix, at the true position and at the prior, for a random sample fixed in advance
   (8 accepted, 4 rejected, every wrong fix). Kept local: the photo licence is unknown.
3. Pre-registration files and the consensus rule commit timestamp.

## Limits

One small site (~0.4 × 0.3 km), one flight, daylight, 20 MP stabilised camera, drone-made map. Simulated
prior drawn around the truth (a real flight's prior comes from dead reckoning). DJI attitude is
GNSS-aided. The terrain-lifted result is exploratory. Not run: RoMa and DISK accuracy, AdHoP refinement,
PnP with a DSM, 0.1 m on the 2021 map.

## Next

1. OpenDroneMap 3D map from the September 2019 survey flight (in progress), then the same tests with its
   terrain model.
2. Full GNSS-free flight (closed loop, prior from the estimator).
3. Forest: wider view (150 m) with the 3D map, strips of ~5 photos, camera-derived 3D shape against the
   survey DSM.
4. Confirmation on another site with everything frozen.

## Reproduce

```bash
.venv/bin/python experiments/x1_tuniu_export.py          # needs the 2019-04-11 photos (ODM community dataset)
.venv/bin/python experiments/x2_tuniu_map.py fetch && .venv/bin/python experiments/x2_tuniu_map.py coverage
.venv/bin/python experiments/x2_tuniu_map.py resample && .venv/bin/python experiments/x2_tuniu_map.py calibrate
.venv/bin/python experiments/x3_tuniu_fix.py run --split precut --res 1.0 --maps main --methods zncc
.venv/bin/python experiments/x3_tuniu_fix.py run --split test --res 1.0 --maps main --methods zncc
.venv/bin/python experiments/x3_tuniu_fix.py summarize --tag main
.venv/bin/python experiments/x_audit_tuniu.py
.venv/bin/python experiments/x_consensus_tuniu.py        # after the "cons" and "seedsx" runs
```

# Pre-registration: Tuniu Level 2, closed-loop GNSS-free replay

Written 2026-10-03 ~18:40 Taipei, BEFORE any run of the closed loop on test photos 47–271. Committed to git
so the timestamp is verifiable (the git commit time is authoritative). Code at commit time:
`experiments/x5_tuniu_closed_loop.py`. Labels: MEASURED / SIMULATED / INFERENCE.

## Question

Level 1 (step 1) scored single-photo fixes around a SIMULATED prior (RTK ± 40 m). Level 2 asks: after the
GNSS cut, can a navigation filter that builds its own prior (photo odometry + map fixes) stay locked on the
real Tuniu flight (DJI Phantom 4 RTK, 2019-04-11) for the remaining 225 photos (630 s, 4.0 km)?

## What the design saw (honest disclosure)

- Designed and tuned on PRE-CUT photos 1–46 only (`calibrate` + closed-loop smoke runs on frames 2–46,
  seed 3). Two design changes came from those pre-cut runs: (1) odometry is no longer gated against the
  previous displacement (a correct odometry after a turn was rejected and the fallback went stale);
  (2) the fallback uses the camera heading instead of the last displacement vector (pre-cut turn: error
  peak 45 m → 24 m).
- Known from step 1 (already published): fixes fail over forest (longest Level-1 gap ≈ 60 s / 380 m),
  photo odometry at test has heavy tails (stage 4). No x5 output on any test photo exists at commit time.

## Inputs to the estimator after the cut

| Input | Source | Label |
|---|---|---|
| State at the cut | RTK position of photo 46 (last pre-cut) and the RTK displacement 45→46 (initial speed) | MEASURED (allowed) |
| Photos + times | 2019-04-11 photos, MRK timestamps | MEASURED |
| Heading | DJI gimbal yaw + pre-cut boresight (−0.5°); pitch/roll + boresight | MEASURED |
| Heading drift (`drift` only) | per-seed constant N(0, 2°) + random walk 0.1°/√s, starting at photo 46, added to the DJI yaw for rectification, odometry and fallback | SIMULATED |
| Barometer | `x_tuniu_geo.simulated_baro`, seed = `seed_of("baro", seed)` (same as step 1) | SIMULATED |
| Terrain | Copernicus GLO-30 (ellipsoidal), swappable (`--terrain`) | MEASURED |
| Map | OAM 2019-12-12 (`main`) at 0.5 m/px; georeferencing offset from step 1, pre-cut (de = +1.97 m, dn = −0.25 m), swappable (`--map`, `--map-offset`) | MEASURED |

RTK after the cut: only to generate the simulated barometer and to score, in the parent process. The worker
that runs the filter receives no RTK after the cut.

## Filter (EKF, state = east, north, heading error b)

1. Each photo is rectified once to a north-up 0.5 m/px ground patch anchored at the camera nadir
   (`x_tuniu_geo.rectify`, ground ≤ 100 m ahead). Height above ground = simulated baro − terrain under the
   FILTER's provisional estimate (previous estimate + fallback displacement). Ground: `dem_prior` (flat plane)
   or `dem_lifted` (each patch point lifted to the terrain), as in step 1.
2. Odometry: nadir displacement D between the previous and current patch (`x4_tuniu_speed.displacement`,
   ZNCC, sub-pixel). Accepted iff a match exists, |D|/dt ≤ 16 m/s and ZNCC peak ≥ 0.5 (pre-cut: every bad
   pair ≤ 0.28, every good pair ≥ 0.72).
3. Fallback when odometry is not accepted: last accepted speed along the mean camera heading of the pair.
4. Predict: p ← p + Rot(−b)·D; covariance with process noise σ² per axis (below) and b random walk 0.1°/√s;
   b prior N(0, 2°) in both heading settings (the filter does not know which setting it is in).
5. A pair is a "turn" if the DJI gimbal yaw changes by more than 10° between the two photos.
6. Absolute fix: pre-registered agreement rule from step 1 (`docs/research/tuniu-consensus-prereg.md`):
   ZNCC (yaw ±4°, scale ±6 %) and XFeat (2048 keypoints, MAGSAC + plausibility) both return a nadir fix in
   the same map window and agree within 4.0 m; position = ZNCC fix − map offset.
7. Search window: square centred on the FILTER prediction, half-size = max(45 m, 3·σ_max) capped at 120 m,
   σ_max = sqrt of the largest eigenvalue of the predicted position covariance.
8. Innovation gate: chi-square 2 dof, 99 % (NIS ≤ 9.21). A gated-out fix is logged, not used.

Noise model (PRE-CUT only, `data/processed/x_tuniu_l2/calibration_main.json`, seed-0 baro, RTK pose as
estimate):

| Parameter | dem_prior | dem_lifted | How |
|---|---|---|---|
| Odometry σ per photo per axis | 1.17 m | 0.87 m | sd that reproduces the drift accumulated over 20 consecutive pre-cut photos (errors are correlated); floor 0.5 m; turns: max(this, RMS of pre-cut turn residuals) |
| Fallback σ, straight / turn | 1.06 / 7.68 m | 1.06 / 7.68 m | RMS of the fallback rule's pre-cut residuals |
| Fix σ per axis | 1.38 m | 1.78 m | 1.4826·MAD of pre-cut agreed-fix residuals (25 / 28 fixes), floor 1.0 m |
| Initial position σ | 1 m | 1 m | RTK at the cut |

## Run matrix

- Seeds 0–19 (baro and drift). Heading {`dji`, `drift`} × ground {`dem_prior`, `dem_lifted`}, closed loop.
- Baseline: dead reckoning only (same filter, no fixes), same seeds and configs.
- Order: `dji` + `dem_lifted` loop and its dead-reckoning baseline first, then the rest. Max 2 worker
  processes while the OpenDroneMap container runs.

## Metrics (per run and pooled over seeds; test photos 47–271)

- Position error = horizontal distance between the filter estimate after the photo's update and the RTK
  antenna position. Median, p90, max, final. % of test photos (≈ % of time, one photo every 2.8 s) with
  error < 10 m and < 25 m.
- Fixes: agreed, accepted, gated out; accepted fixes wrong by > 10 m.
- Loss of lock at photo k: distance between the window centre (filter prediction) and RTK > the half-window
  used at k (Euclidean; a Chebyshev variant, true nadir outside the square, is also reported). Events =
  runs of consecutive loss-of-lock photos.
- Longest interval without an accepted fix (s, including from the cut to the first fix and from the last fix
  to the end).
- Comparison with dead reckoning: same seed and config, median / max / final error.
- Figure: error vs time for seeds 0–3, `dji` + `dem_lifted`, with the dead-reckoning error, the
  half-window, accepted fixes, and the photos never fixed in step 1 (forest gaps) shaded. No photo pixels.

## Pass criteria (`dji` + `dem_lifted`, closed loop)

1. PRIMARY: no loss of lock (zero loss-of-lock photos) in ≥ 18 of 20 seeds.
2. SECONDARY: pooled median position error ≤ 10 m, and 0 accepted fixes wrong by > 10 m (all seeds).

The other three configs are reported with the same metrics, without a pass claim. The criteria proposed by
the parent are kept unchanged.

## Not claimed

One flight, one small site, DJI GNSS-aided attitude, simulated barometer and heading drift, map 8 months after
the flight. A pass shows the loop closes on this flight; confirmation needs another flight with everything
frozen.

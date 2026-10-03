# Tuniu Level 2: closed-loop GNSS-free replay (results)

Date: 2026-10-03. Site: Tuniu River, Toufen, Miaoli, Taiwan. Labels: MEASURED (real photos, real DJI
attitude, real RTK for scoring), SIMULATED (barometer, heading drift), INFERENCE (interpretation).
Pre-registration: `docs/research/tuniu-level2-prereg.md`, commit `ac18f70`, 2026-10-03 18:39:13 +08:00,
pushed before any test-photo run (the first run file, `runs/main/loop_dji_dem_lifted_s0.csv`, was written at
18:40:35 after a 43 s run). All numbers below come from `data/processed/x_tuniu_l2/` (git-ignored;
regenerate with the commands at the end).

## Bottom line

- **Pre-registered config (`dji` heading + terrain-lifted ground) PASSES both criteria.** No loss of lock in
  20/20 seeds (criterion: ≥ 18). Median position error 3.3 m (criterion: ≤ 10 m). 0 accepted fixes wrong by
  more than 10 m, out of 1,646 accepted.
- **Dead reckoning alone on the same photos: median 54 m, final 163 m.** Map fixes cut the median error by
  about 16× and keep the final error at 2.4 m.
- **The margin is thin.** Worst case: the error reaches 92 % of the search half-window (seed with the
  smallest margin: 3.5 m). With the simulated heading drift it reaches 99 % (margin 0.3 m). All the
  near-misses come right after turns.
- **The flat-ground variant (`dem_prior`) loses lock in 20/20 seeds.** Cause: its odometry drifts faster than
  its pre-cut noise model says. The filter then becomes too sure of itself, and the 99 % gate rejects correct
  fixes (372 rejected, median error 2.6 m).

## What was tested

The real 2019-04-11 flight (DJI Phantom 4 RTK, 271 photos, about 100 m above take-off). GNSS is cut after
photo 46. Photos 47–271 are replayed without GNSS: 225 photos, 630 s, 4.0 km. Each photo's search window is
centred on the filter's own estimate, never on the truth. RTK after the cut is used only to score and to
generate the simulated barometer, and the worker running the filter never receives it.

## How it works (5 lines)

1. Every photo is flattened into a north-up 0.5 m/px ground image. Inputs: DJI camera angles, simulated
   barometer height minus terrain height under the filter's estimate, Copernicus 30 m terrain.
2. Consecutive photos are matched (ZNCC) to measure how far the drone moved. When that fails (turns), the
   filter assumes the last speed along the camera heading.
3. Each photo is also matched against the OpenAerialMap 2019-12 orthophoto inside a window centred on the
   filter estimate: half-size max(45 m, 3σ), capped at 120 m.
4. A fix is used only if ZNCC and XFeat agree within 4 m (the step-1 rule) and it passes a 99 % chi-square
   gate.
5. A Kalman filter (east, north, heading error) merges the two. All noise values are fitted on photos 1–46
   only (`calibration_main.json`).

## Results (20 seeds per row; 225 test photos per seed)

| Heading | Ground | Mode | Seeds | Median / p90 / max error (m) | Final (median) | < 10 m | < 25 m | Fixes accepted (gated) | Wrong > 10 m | Seeds without loss of lock | Longest no-fix gap (median / max) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **dji** | **dem_lifted** | **closed loop** | 20 | **3.3 / 17.3 / 42.5** | 2.4 | 76 % | 94 % | 1646 (19) | **0** | **20 / 20** | 59 / 81 s |
| dji | dem_lifted | dead reckoning | 20 | 53.7 / 139.3 / 182.4 | 163.4 | 0 % | 15 % | – | – | – (no window used) | 630 s |
| dji | dem_prior | closed loop | 20 | 26.6 / 157.1 / 220.7 | 177.5 | 35 % | 49 % | 538 (372) | 3 | 0 / 20 | 263 / 431 s |
| dji | dem_prior | dead reckoning | 20 | 109.9 / 201.1 / 284.9 | 215.7 | 0 % | 11 % | – | – | – (no window used) | 630 s |
| drift | dem_lifted | closed loop | 20 | 3.2 / 19.5 / 44.7 | 1.6 | 76 % | 93 % | 1603 (47) | 3 | 20 / 20 | 59 / 95 s |
| drift | dem_lifted | dead reckoning | 20 | 52.5 / 138.1 / 190.0 | 164.1 | 0 % | 13 % | – | – | – (no window used) | 630 s |
| drift | dem_prior | closed loop | 20 | 28.7 / 156.9 / 238.1 | 177.7 | 34 % | 48 % | 545 (410) | 2 | 0 / 20 | 263 / 431 s |
| drift | dem_prior | dead reckoning | 20 | 117.2 / 187.3 / 281.3 | 204.2 | 0 % | 10 % | – | – | – (no window used) | 630 s |

How to read it: error = distance between the filter estimate and RTK after each photo (MEASURED photos,
SIMULATED barometer and drift). "< 10 m" = share of test photos, which is also the share of time (one photo
every 2.8 s). Source: `summary_main_pooled.csv` (pooled), `summary_main_runs.csv` (per run),
`summary_main.json` (verdict), `runs/main/*.csv` (per photo).

Extra numbers for the pre-registered config (`dji` + `dem_lifted`, closed loop):

- Per-seed median error 2.7–4.5 m; per-seed max error 26.5–42.5 m. 75–88 fixes accepted per seed (33–39 % of
  photos). Median error of accepted fixes 2.4 m, max 9.1 m.
- Lock margin (half-window minus window-centre error): smallest 3.5 m. 2 seeds went above 90 % of the
  half-window.
- The 19 gated-out fixes were all correct (error ≤ 7.8 m). The gate removed no wrong fix here; it only threw
  away good ones when the filter was over-confident after a turn.
- Odometry accepted for 82 % of photo pairs: 89 % on straight legs, 41 % in turns.
- 205 of the 267 photos with error > 25 m are in a turn or within 3 photos after one. Turns are 15 % of
  photos; median error is 14.5 m in turns vs 2.7 m on straight legs.
- Filter consistency: mean normalised squared prediction error 1.49 (1.0 = consistent). The filter is
  slightly over-confident.

With the simulated heading drift (`drift`, bias up to 7.3°), the filter estimates the bias to 0.65° (median
final error). Lock holds in 20/20 seeds, but 3 fixes wrong by 10.5–16.9 m were accepted and the tightest
margin is 0.3 m.

Figure: `data/processed/x_tuniu_l2/fig_error_vs_time_main.png`. Seeds 0–3, `dji` + `dem_lifted`, log scale.
Blue = closed loop, orange = dead reckoning, dashed = search half-window, ticks = accepted fixes, green bands =
photos never fixed by the step-1 rule (forest). The error climbs through each forest gap or turn, then drops
back to 1–3 m at the first accepted fix.

## What it means

- INFERENCE: on this flight, photo odometry plus agreed map fixes closes the loop without GNSS. The step-1
  fixes still work when the prior comes from the filter instead of from the truth. The forest gaps (up to
  81 s with no fix) are bridged.
- INFERENCE: the weak point is the turns, not the forest. Photo odometry fails there (camera rotating, little
  overlap), the fallback (last speed along the heading) is off by 10–20 m per turn, and the error reaches
  30–45 m against a 45 m window. An IMU or a better turn model is the obvious fix. A larger minimum window
  would also help, at the cost of more false-match risk.
- INFERENCE: modelling the terrain matters for odometry, not only for fixes. With a flat ground plane, the
  odometry drift seen after the cut is larger than the 20-photo pre-cut drift (1.17 m per photo per axis).
  The filter becomes over-confident and its own gate rejects good fixes. Correct fixes gated out, not wrong
  fixes accepted, is what loses lock.
- The chi-square gate never saved the filter from a wrong fix in the passing configs. It only rejected good
  ones. The step-1 agreement rule did the real filtering (8 wrong fixes out of about 4,300 accepted over all
  configs, all ≤ 17.3 m).

## Limits

- One flight, one small site (~0.4 × 0.3 km), daylight. The noise model comes from 46 pre-cut photos: 3
  windows of 20 photos for odometry drift, 4 turn pairs for the fallback.
- The DJI attitude is GNSS-aided (`dji`). The `drift` setting simulates a heading error but not
  pitch/roll errors.
- The barometer and heading drift are SIMULATED. The map is OAM 2019-12, 8 months after the flight, with its
  offset calibrated on pre-cut photos.
- Design used pre-cut closed-loop smoke runs (disclosed in the pre-registration). Step-1 test results (forest
  gaps, odometry tails) were already known.
- After the pre-registration commit the code changed only outside the estimator: a crash fix in the progress
  print for dead-reckoning runs, the `prepare-map` command, the docstring, and the report script.
  `run_sequence`, the filter and the calibration are unchanged.
- Not done: OpenDroneMap orthophoto/DSM runs (products not yet available; the CLI is ready). No IMU. No
  real-time timing (Mac, not Jetson). No other site.

## Reproduce

```bash
.venv/bin/python experiments/x5_tuniu_closed_loop.py calibrate                       # pre-cut only
.venv/bin/python experiments/x5_tuniu_closed_loop.py run --mode loop dr --heading dji drift \
    --ground dem_lifted dem_prior --seeds $(seq 0 19) --workers 2                      # resumes from runs/main/
.venv/bin/python experiments/x5_tuniu_l2_report.py summarize
.venv/bin/python experiments/x5_tuniu_l2_report.py figure --seeds 0 1 2 3
```

OpenDroneMap swap (when `odm_orthophoto.tif` and `odm_dem/dsm.tif` exist; set `--terrain-dz` so that DSM
heights are ellipsoidal):

```bash
.venv/bin/python experiments/x5_tuniu_closed_loop.py prepare-map <odm_orthophoto.tif> data/processed/x_tuniu_l2/maps/odm_0.5m.tif
.venv/bin/python experiments/x5_tuniu_closed_loop.py calibrate --map data/processed/x_tuniu_l2/maps/odm_0.5m.tif --terrain <dsm.tif> --terrain-dz <dz> --tag odm
.venv/bin/python experiments/x5_tuniu_closed_loop.py run --map data/processed/x_tuniu_l2/maps/odm_0.5m.tif --map-offset calib \
    --terrain <dsm.tif> --terrain-dz <dz> --calib-tag odm --tag odm --mode loop dr
.venv/bin/python experiments/x5_tuniu_l2_report.py summarize --tag odm
```

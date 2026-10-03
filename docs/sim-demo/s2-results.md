# S2: Alessandro's ESKF, alone and fused with our camera-to-map fixes (SIMULATED flight)

Every number below is in a file of this folder (paths in brackets). Both cameras and 5 seeds were run.

## Result in one line

On the same simulated flight, adding our map fixes to Alessandro's ESKF changes the error after the GNSS cut as
follows (median over 5 seeds):

| Camera | ESKF alone, median / max | ESKF + our fixes, median / max |
|---|---|---|
| Ideal | 56 m / 587 m | 1.7 m / 6.6 m |
| Realistic (Dustin's camera model) | 169 m / 1,946 m | 1.9 m / 11.0 m |

No wrong fix was accepted. His 99 % gate rejected 0–2 correct fixes per run.

## What was run

- Flight: `recordings/ilhan_wufeng_south_80m` (Gazebo, Wufeng south route, 80 m, 8 m/s, 5,112 m).
  IMU 100 Hz, barometer 50 Hz, down camera 5 Hz (512 px).
- GNSS cut: after 450 m of true distance flown (image 482, t = 96.4 s), the same rule and image grid as S1.
  Scored from the cut to the end of the route: 4,540 m flown.
- **Caveat, pre-cut GNSS is SIMULATED.** The recorded `gnss.csv` stops at t = 6.8 s, so before the cut GNSS is
  truth + N(0, 1.5 m) per horizontal axis at 1 Hz. Seed 0 uses exactly S1's series; seeds 1–4 draw new noise.
  Truth is never used after the cut. The one exception is roll/pitch inside the map matcher (AHRS stand-in, same as S1).
- Estimator: `vio/estimation/eskf.py`, unchanged; it is identical to the ale-simulation copy. The offline driver
  `experiments/s2_sim_eskf_fusion.py` reproduces his live sim adapter:
  - start from a settled 0.5 s IMU window (yaw 0; the drone spawns facing east);
  - IMU prediction;
  - barometer update every 20 IMU samples, with his noise model;
  - GNSS before the cut: horizontal position (gate 0.999) plus 5 s finite-difference velocity.
- After the cut: down-camera flow velocity, as in his Mid-Air design (`eskf_runner._flow_update`, parameters from
  `vio/configs/midair_eskf.yaml`). Height above ground is learned from flow during the last 10 s before the cut:
  82.9 m learned vs 78.7 m true for seed 0. After that, the barometer carries it.
- Configurations:
  - **A**: ESKF alone after the cut.
  - **B**: A + our fixes at 1 Hz, i.e. every 5th image, 608 attempts.
    - Fixes come from `s1_sim_map_fix.fix_at`: ZNCC + XFeat consensus within 4 m on the 2018 0.5 m map.
    - They are computed live: the ESKF's own estimate centres the search window, and its own sigma, heading and
      altitude are passed in.
    - Each fix is injected as a horizontal position update. The math is the same as `update_position`, restricted
      to x/y, following Dan's `on_rf` pattern.
    - Fix covariance = S1's pre-cut calibration: 1.24 m per axis (ideal camera), 1.29 m (realistic camera).
    - Gate: his 99 % chi-square gate (2 dof: 9.21).
- Randomness: the filter and the matcher are deterministic. The only random input is the pre-cut GNSS noise, so
  there are 5 seeds of it.

## Frames

| Item | Convention |
|---|---|
| World | ENU metres from the route origin, the same frame as truth and S1. Gravity = (0, 0, −9.80665). His Mid-Air runner uses NED with gravity (0, 0, +g); his sim adapter uses ENU like here. The ESKF class works in either frame. |
| Body | FLU (`imu.csv`); gyro in body axes |
| Down camera | OpenCV optical axes. R_bc = [[0,−1,0],[−1,0,0],[0,0,−1]] (`gazebo_down_optical_to_flu`, `meta.json`): x = body right, y = body back, z = down. Flow ground-plane normal = world down = (0, 0, −1) in ENU. |
| Heading passed to the matcher | Compass bearing of body +x, clockwise from north: atan2(R[0,0], R[1,0]) |

Sanity check (B, seed 0): flow agrees with the fused velocity to 0.4° in direction (median). Measured speed is 6 %
high, which matches the height being learned 5 % too high.

## Results (scored on S1's 1 Hz image grid, horizontal error)

### Realistic camera: ESKF configurations, median over 5 seeds
[`summary_eskf_median_over_seeds_realistic.csv`, `summary_eskf_per_run_realistic.csv`, `runs/realistic/`]

The realistic camera is Dustin's `RealisticCamera`: clouds, haze, vignetting, blur, noise, JPEG. It is applied to
both the flow front end and the fixes.

| Config | Median (m) | p90 (m) | Max (m) | Final (m) | NEES (horizontal) mean / % above 99 % bound | Heading error, median |
|---|---|---|---|---|---|---|
| A: ESKF alone | 168.6 | 1,475 | 1,946 | 1,946 | 65 / 48 % | 25.2° |
| B: ESKF + our fixes | 1.9 | 3.9 | 11.0 | 4.1 | 6.5 / 23 % | 0.6° |

Per seed (median / max / final, m):

| Seed | A | B |
|---|---|---|
| 0 | 243 / 2,627 / 2,627 | 1.9 / 10.6 / 4.1 |
| 1 | 281 / 2,928 / 2,928 | 1.9 / 15.8 / 4.1 |
| 2 | 148 / 1,572 / 1,572 | 1.9 / 11.8 / 4.2 |
| 3 | 98 / 903 / 889 | 1.9 / 11.0 / 4.1 |
| 4 | 169 / 1,946 / 1,946 | 1.9 / 10.8 / 4.1 |

Fixes in B (realistic camera):
- 608 attempts; 147–164 had no consensus; 444–461 were accepted; 0–1 were rejected (NIS 10.4 and 15.3, both
  correct fixes).
- 0 wrong fixes. Fix error: 1.7–1.8 m median, 2.9 m p90, 6.1 m max.
- Longest stretch without an accepted fix: 136–143 m.

What changes with the realistic camera:
- Flow: 2,973 of 3,036 updates accepted. The others were rejected for low inlier ratio (42) or too few tracks (20).
- Without fixes, the ESKF heading drifts: 9–45° median error per seed against 4–25° on the ideal camera.
  - The flow update also corrects attitude (its Jacobian has an attitude term), so degraded flow pulls the heading.
  - A becomes overconfident: NEES 12–179.

### Ideal camera: ESKF configurations, median over 5 seeds
[`summary_eskf_median_over_seeds_ideal.csv`, `summary_eskf_per_run_ideal.csv`]

| Config | Median (m) | p90 (m) | Max (m) | Final (m) | NEES (horizontal) mean / % above 99 % bound | Heading error, median |
|---|---|---|---|---|---|---|
| A: ESKF alone | 55.9 | 429 | 587 | 587 | 3.2 / 10 % | 5.7° |
| B: ESKF + our fixes | 1.7 | 3.1 | 6.6 | 4.0 | 6.7 / 24 % | 0.61° |

Per seed (median / max / final, m):

| Seed | A | B |
|---|---|---|
| 0 | 40 / 108 / 56 | 1.8 / 6.9 / 4.0 |
| 1 | 96 / 415 / 379 | 1.7 / 6.6 / 4.0 |
| 2 | 56 / 769 / 769 | 1.7 / 6.6 / 4.0 |
| 3 | 184 / 1,991 / 1,991 | 1.7 / 6.1 / 4.0 |
| 4 | 39 / 587 / 587 | 1.8 / 6.9 / 4.1 |

### Fixes in B, ideal camera, range over seeds
[`runs/ideal/B_s*_fixes.csv`]

| Attempts | No consensus | Accepted | Rejected by gate | Wrong (> 10 m) | Wrong accepted | Correct rejected |
|---|---|---|---|---|---|---|
| 608 | 99–103 | 503–507 | 1–2 | 0 | 0 | 1–2 |

Details:
- All rejected fixes were correct (seed 0: fix error 4.6 m). Their NIS was 9.5–11.3, just above the 9.21 gate.
- Fix error against truth: 1.6 m median, 2.8–3.1 m p90, 4.8–6.2 m max.
- The search window stayed at its 45 m minimum, because the ESKF's sigma stayed small after the first fix.

### NIS and NEES (measured)

- **Before the cut**, GNSS position NIS averages 1.9–4.4 (expected value 2). The ESKF is mildly overconfident
  before the cut.
- **A, after the cut**: NEES depends on the seed.
  - Ideal camera: from 0.6 (seed 0, conservative) to 40 (seed 3, overconfident). In seed 3 the heading drifted to
    79° error and the final error reached 2 km.
  - Realistic camera: 12–179.
  - Only the pre-cut GNSS noise differs between seeds. A's long-range result is therefore very sensitive to the
    state it inherits at the cut.
- **B, accepted fixes**: NIS median 0.18–0.21 for both cameras (χ²(2) median is 1.39). Fixes look conservative
  relative to the filter.
- **B, after the cut**: horizontal NEES averages 6.4–7.5, and 21–26 % of samples are above the 99 % bound, for
  both cameras.
  - The fused ESKF is overconfident even though it is accurate.
  - Our reading: fix errors are correlated from one second to the next (median 1.6–1.8 m against a 1.24–1.29 m
    sd), while the filter treats 1 Hz fixes as independent.
- The exploratory variant (inflated process noise) was **not run**. The brief allowed it only if B rejected most
  correct fixes, and B rejected at most 2 per run.

### Joint table, same recording
[`joint_table_<camera>_s0.csv` (seed 0), `joint_table_<camera>_median_over_seeds.csv`]

Median over seeds 0–4. S1 rows are SimDemoMinimal's final runs (`outputs/s1_sim/runs/<camera>/loop_s*.csv`,
Dustin's per-frame files), read through `s1_sim_report.series`.

**Ideal camera**

| Estimator | Median (m) | p90 (m) | Max (m) | Final (m) | Accepted fixes | Longest gap without a fix (m) |
|---|---|---|---|---|---|---|
| A: Alessandro's ESKF alone | 55.9 | 429 | 587 | 587 | – | – |
| B: Alessandro's ESKF + our fixes | 1.7 | 3.1 | 6.6 | 4.0 | 506 | 106 |
| S1, ours: odometry + fixes, EKF with scale state | 1.7 | 2.9 | 5.2 | 4.5 | 511 | 121 |
| S1, ours, 3-state EKF (no scale state) | 1.7 | 3.0 | 6.0 | 4.4 | 504 | 114 |
| S1, dead reckoning (same odometry, no fixes) | 28.1 | 80.7 | 84.0 | 84.0 | – | – |
| S1, Dustin's navigator, 2018 map | 18.2 | 48.8 | 68.1 | 14.3 | 10 | 804 |
| S1, Dustin's navigator, camera alone | 69.3 | 111 | 128 | 122 | – | – |

**Realistic camera**

| Estimator | Median (m) | p90 (m) | Max (m) | Final (m) | Accepted fixes | Longest gap without a fix (m) |
|---|---|---|---|---|---|---|
| A: Alessandro's ESKF alone | 168.6 | 1,475 | 1,946 | 1,946 | – | – |
| B: Alessandro's ESKF + our fixes | 1.9 | 3.9 | 11.0 | 4.1 | 451 | 143 |
| S1, ours: odometry + fixes, EKF with scale state | 1.8 | 3.2 | 6.4 | 3.9 | 459 | 151 |
| S1, ours, 3-state EKF (no scale state) | 1.9 | 3.3 | 6.3 | 4.3 | 457 | 148 |
| S1, dead reckoning (same odometry, no fixes) | 26.9 | 84.9 | 88.3 | 88.3 | – | – |
| S1, Dustin's navigator, 2018 map | 26.4 | 56.9 | 86.0 | 76.2 | 10 | 806 |
| S1, Dustin's navigator, camera alone | 71.0 | 120 | 146 | 144 | – | – |

Figures: `fig_error_vs_distance_ideal_s0.png` and `fig_error_vs_distance_realistic_s0.png` show error vs
distance flown since the cut for every estimator, seed 0.

## What this does and does not show

- **Shown, same data:**
  - Our fixes fused into Alessandro's ESKF bring it from hundreds of metres to under 7 m (ideal camera) and under
    16 m (realistic camera) everywhere.
  - His gate accepts them; no wrong fix entered.
  - With the same fixes, his ESKF and our EKF have the same median error (1.7 m ideal, 1.8–1.9 m realistic). The
    fixes dominate.
  - His ESKF has a higher worst case: max 6.6 vs 5.2 m on the ideal camera, 11.0 vs 6.4 m on the realistic one.
    - Every B maximum occurs in a gap between fixes, 30–129 m after the last accepted fix.
    - S1 has gaps of similar length (longest 121–151 m) but lower maxima.
    - So on this flight, between fixes, his IMU + flow prediction drifts faster than our odometry.
- **Not shown:**
  - That his ESKF alone is worse or better than our odometry dead reckoning. The heading inputs differ: S1 uses
    truth heading plus simulated drift, while A integrates the raw simulated gyro.
  - Anything about the real world. This is one simulated flight over the same orthophoto the map is made from.
- **Seen in A:** the estimate jumps by 10–48 m when the drone slows at route reversals (ideal camera, seed 0).
  - Cause: the flow noise scales with speed, so near-hover flow updates are tight, and the position–velocity
    correlation moves the position.
  - This is visible in the figure; we did not tune it.

## Reproduce

```
PY=/Users/ilhan.neuville/dev/hackathon/TaipeiDrift/.venv/bin/python
$PY experiments/s2_sim_eskf_fusion.py run --config A B --seeds 0 1 2 3 4 --workers 3 --camera ideal       # ~2.5 min per B run
$PY experiments/s2_sim_eskf_fusion.py run --config A --seeds 0 --camera realistic                         # builds the flow cache first
$PY experiments/s2_sim_eskf_fusion.py run --config B A --seeds 0 1 2 3 4 --workers 3 --camera realistic
$PY experiments/s2_sim_report.py --camera ideal --seed 0
$PY experiments/s2_sim_report.py --camera realistic --seed 0
```

Logs: `run_ideal.log`, `run_realistic.log`. Flow pairs cache: `cache/flow_pairs_<recording>_<camera>.pkl`.

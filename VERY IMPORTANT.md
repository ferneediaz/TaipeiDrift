# VERY IMPORTANT — Metric Horizontal Velocity Findings

## Bottom line

The downward optical-flow + range path is implemented as an **experimental, disabled-by-default** ESKF measurement. The current Gazebo results do **not** justify enabling it for GNSS-denied navigation: the scheduled denial run diverged substantially, and the measured flow updates were sparse and often inaccurate. Keep GNSS as the primary source and do not describe this trial as a successful navigation solution.

## What was added

- A downward camera optical-flow tracker with forward/backward feature checks, homography RANSAC, camera calibration, attitude de-rotation, and metric scaling from the co-located downward range sensor.
- A `/range/down` Gazebo-to-ROS bridge and configurable range limits/noise in the quad model/launch.
- Optional ESKF horizontal camera-velocity updates, disabled by default and decimated to reduce use of temporally correlated image pairs. The adapter does not subscribe to ground truth; ground truth is used only by the logger and post-run evaluation.
- Flow/range diagnostics, state covariance logging, a metric-velocity summary script, and focused tests.

## Measured scheduled GNSS-denial trial

Run: `outputs/sim_runs/metric_velocity/OF2_gnss_denied_fixed`

- World: city; built-in demo route; 512 px camera; metric flow enabled; flow update every fifth image pair.
- Configured GNSS cutoff: 20 s. Observed denial: 20.462 s.
- Logged duration: 156.4 s.
- Position error: 0.82 m at cutoff; 1.62 m at +5 s; 3.01 m at +10 s; 4.73 m at +20 s; 12.33 m at +30 s; 426.45 m at run end (maximum 643.88 m).
- Flow: 2,245 estimates; 1,345 ground-truth-scored; 266 update attempts; 88 accepted and 178 rejected. The remaining correlated frame-pair updates were decimated. Across scored flow estimates, median vector error was 4.45 m/s (p90 7.58 m/s). Accepted updates had median error 0.72 m/s (p90 3.26 m/s); rejected updates had median error 5.30 m/s.
- Frequent flow rejection causes included too few tracks (290), low RANSAC support (525), and Mahalanobis gating (178); 34 range discontinuities were detected.
- Range: 3,575 valid scans, with 83.10 m observed local-surface dynamic range. Against the post-run city-geometry evaluation, absolute range error was 0.152 m median and 0.260 m p90. This comparison is evaluation-only; truth is not fed into the estimator.

The run-end error is materially worse than the early post-cutoff values; quote the horizon-specific values rather than suggesting a stable 156-second result. Range accuracy alone did not make the optical-flow velocity estimate reliable in this urban scene.

## Controls and interpretation

The forward-only always-GNSS control `outputs/sim_runs/metric_velocity/OF3_ablation_A_forward_only` completed with 3.23 m final position error (6.21 m maximum) over 331.7 s. It is **not** a matched GNSS-denial control, so it cannot establish that optical flow caused the denial-run divergence. An earlier GNSS-on flow trial also diverged, but these results are diagnostic rather than a controlled causal comparison.

Prior flow trials showed similarly poor behavior: enabling updates every image pair resulted in severe divergence; decimating to every fifth pair did not establish a reliable remedy. Do not tune process noise or claim a benefit from these runs. A controlled, matched-seed ablation and improved robust flow/range association would be needed before reconsidering fusion by default.

## Validation and repository state

- Focused tests: `44 passed` (`sim/tests/test_metric_flow.py`, `sim/tests/test_gnss_velocity_fit.py`, `vio/tests/test_eskf.py`).
- Python compilation and `git diff --check` passed.
- Simulation outputs are retained locally under `outputs/sim_runs/metric_velocity/` and are not included in this source commit; large generated data should be shared separately if needed.
- The new metric-flow phase must remain opt-in (`metric_flow:=false` by default) pending better results.

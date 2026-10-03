# GNSS velocity phase

Both runs use the city route, 512 px cameras, forward rotation and direction enabled, and light rosbag recording. The fit is robust GLS over an 8 s window, with at least six fixes spanning six seconds. Fits use GNSS position covariance, reject at most one large normalized residual, and use non-overlapping windows.

## Results

| Experiment | Duration | Position error | GNSS velocity fits | Fit error vs GT |
|---|---:|---:|---:|---:|
| V1, GNSS always on | 150.9 s | final 3.24 m; max 6.30 m | 18; 11 accepted, 7 ESKF rejected | vector median 0.88 m/s, p90 5.98 m/s; speed median 0.60 m/s, p90 2.93 m/s |
| V2, cutoff at 20 s | 124.8 s | final 259.41 m (horizontal 259.39 m, vertical 3.49 m) | 2; both accepted before cutoff | vector median 0.194 m/s, p90 0.245 m/s |

V1 stayed bounded with GNSS on. The large fit-error tail occurs on changing-speed and turning sections: a constant-velocity fit over seven seconds represents average motion across the window, which may differ from instantaneous velocity. Those updates were gated seven times; this fit should not be treated as reliable during every maneuver.

At low horizontal speed, the V1 fit windows with GT horizontal speed below 0.5 m/s had median GT speed 0.058 m/s and median fitted speed 0.296 m/s (p90 0.332 m/s). In the saved old D run, the two-point endpoint method implied 0.89 and 0.91 m/s at t=7 and t=12 while GT was stationary. At t=18 in V2, the regression fit was `[-0.0085, 0.0166, 3.1099]` m/s versus GT `[0.0574, 0.0002, 2.9995]` m/s: vector error 0.130 m/s, speed error 0.111 m/s, direction error 1.27°. The prior D run's t=17 endpoint estimate had 1.33 m/s vector error and 1.09 m/s speed error. The t=18 fit used 8 fixes over 7 s and covariance `diag(0.0536, 0.0536, 0.2143) (m/s)^2`, derived from the GLS slope covariance.

## Denial velocity decomposition

The V2 cutoff indicator first became false at t=20.3 s; the gate was configured for 20 s from the first raw fix (0.002 s), and its first dropped 1 Hz fix was t=21 s. At the first unavailable row, position error was 3.96 m (0.48 m horizontal, 3.93 m vertical); velocity vector error was 0.428 m/s, horizontal velocity error 0.350 m/s, speed error 0.271 m/s, and direction error 6.05°.

| After cutoff | 3D position error | Velocity vector error | Speed error | Direction error | Direction-only equivalent |
|---:|---:|---:|---:|---:|---:|
| +5 s | 4.28 m | 0.72 m/s | 0.34 m/s | 11.60° | 0.61 m/s |
| +10 s | 7.99 m | 1.16 m/s | 0.36 m/s | 19.94° | 1.04 m/s |
| +20 s | 28.13 m | 3.23 m/s | 2.49 m/s | 42.34° | 1.35 m/s |
| +30 s | 31.66 m | 2.03 m/s | 1.98 m/s | 3.06° | 0.41 m/s |
| Final, +104.5 s | 259.41 m | 6.25 m/s | 6.24 m/s | 2.08° | 0.28 m/s |

Across 1,046 post-cutoff samples, speed error exceeded the direction-only equivalent in 70.2%; their median magnitudes were 3.78 m/s and 1.06 m/s. Direction error is more important during the first 5–10 s, but speed magnitude dominates the later drift in this run. Position drift grows with the sustained velocity magnitude error. The saved previous D run was better at matched +20 s and +30 s (13.23 m and 25.64 m versus 28.13 m and 31.66 m here), so the velocity fit did not improve denial navigation overall in this single stochastic run.

## Down camera and range

The downward camera publishes `/camera/down/image_raw` and `/camera/down/camera_info` at 25 Hz. V2 CameraInfo is 512×512 with `fx=fy=255.9991`, `cx=cy=256`. Its SDF pose is a +90° pitch; optical +Z points down. There is no downward range sensor in the model. The barometer measures pressure/altitude and was not used as ground range.

The simulator and VIO test suites passed: 176 tests, 3 existing numerical-correlation warnings. Both simulation processes and rosbag recorders were stopped cleanly. V1 and V2 output folders contain the trajectories, estimator/GNSS/velocity diagnostics, metadata, logs, and light bags. `baseline_reference.json` preserves the old D run's metrics and endpoint-velocity measurements. The first V1 startup attempt had a logger syntax error; its partial bag is preserved in `V1_logger_startup_failure_partial/` and is excluded from the results above.

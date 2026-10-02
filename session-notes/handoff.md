# Handoff

Last updated: Saturday 3 October 2026, 01:00.

## Resume here

Written on Saturday at 01:00, right before the chat was compacted. Dustin wants to work until 03:00 or 04:00, and he decided that the assistant writes all the code.

**Where we are.** Branch `alto-navigator` is checked out. It is `main` plus a merge of `origin/mid-air-baseline-fix` plus the new files below. Goal for tonight: result 2 as shared code inside the `baseline/` package, reproducing `experiments/h_alto_end_to_end.py`.

**Numbers the shared code has to reproduce** (ALTO validation section, GNSS cut after 300 m):

| Run | Median | Worst | End |
|---|---|---|---|
| Camera only | 472.4 m | 657.0 m | 608.2 m |
| Fix every 100 m, 7 nearest images | 25.5 m | 50.3 m | 26.3 m |
| Fix every 300 m, 7 nearest images | 30.6 m | 83.1 m | 15.7 m |
| Fix every 400 m, 7 nearest images (the cliff) | 285.1 m | 897.9 m | 897.9 m |
| Fix every 1,000 m, 7 nearest, score at least 0.33 | 472.4 m | 657.0 m | 608.2 m |
| Fix every 1,000 m, search sized by uncertainty, score at least 0.33 | 56.3 m | 278.1 m | 10.0 m |

**Done and checked** (both flights load, 53 existing tests pass):

- `baseline/src/data/camera_flight.py`: `CameraFlight`, `ReferenceMap`, `prepare`. Positions are (north, east) in metres from the first frame.
- `baseline/src/data/alto.py`: `load_alto_flight`, reads `Val.zip` without unpacking.
- `baseline/src/data/synthetic_camera.py`: `make_synthetic_camera_flight`, a small generated flight for tests (160 pixel images, zoom 0.85, rotation 15 degrees).

**Still to write, in this order:**

1. `src/estimation/image_motion.py`: `image_shift(previous, current)`, the median optical flow in pixels (Farneback with 0.5, 4, 21, 3, 7, 1.5 on frames reduced to about 250 pixels, median over the centre, scaled back to full-size pixels), and `shifts_for_flight(flight, cache_path)`.
2. `src/estimation/map_matching.py`: `make_template(frame, zoom, angle, keep=0.8)` and `match(frame, reference_map, candidates, zooms, angles)`, returning score, position, zoom, angle and reference index. Position is the reference position plus (-(cy - H/2), (cx - H/2)) times metres per pixel, as (north, east).
3. `src/estimation/navigator_core.py`: `fit_motion_matrix(shifts, steps)` by least squares so that steps is about shifts @ A; `blend(estimate, variance, fix, fix_variance)` with gain = variance / (variance + fix_variance); `fix_decision(score, distance, allowed, min_score)` returning use and a reason (OK, LOW_SCORE, DISAGREES_WITH_ESTIMATE); `status(sigma)` giving TRACKING, DEGRADED or LOST.
4. `src/estimation/camera_navigator.py`: the loop of the experiment.
   - Calibration while GNSS works: the motion matrix from the first 300 m; three test fixes at a third, two thirds and the end of that stretch, with zoom 0.60 to 1.00 in steps of 0.05 and angles -10 to 35 in steps of 5, on the 7 reference images nearest to the true position. Their medians give zoom0, angle0 and the fix offset.
   - Dead reckoning: step = shift @ A0 times (zoom / zoom0).
   - Fix every N metres of estimated travel. Fixed search: 7 nearest images, zoom within 0.10 of the last zoom in steps of 0.05 clipped to 0.5 to 1.1, angles angle0 - 5, angle0, angle0 + 5. Sized search: all images within max(60 m, 3 sigma), and zoom 0.60 to 1.10 when more than 400 m have passed since the last fix.
   - Uncertainty: predicted variance = variance + (0.10 times distance since the last fix) squared, starting at 3 m squared. A fix has 15 m. It is used if it lies within 3 sigma of sqrt(predicted variance + 15 squared) and its score is high enough. Then the estimate moves by the gain, the variance becomes (1 - gain) times the predicted variance, the scale becomes new zoom / zoom0, and the distance since the last fix is reset.
5. `src/evaluation/navigation_metrics.py`: median, 90 percent, worst and end error; fixes used and rejected; used but wrong by more than 50 m; rejected although within 30 m.
6. `src/visualization/navigator_plot.py`: path and error over distance, like `docs/figures/alto_end_to_end.png`.
7. `scripts/run_alto_navigator.py` and `configs/alto_navigator.yaml`.
8. Tests on the synthetic flight: `test_navigator_core.py`, `test_map_matching.py`, `test_camera_navigator.py`. A regression test on ALTO that is skipped when the data is missing and checks the table above.
9. Then, if time is left: the status logic, degraded images (blur, darkness, haze), and the held-out test on the training section.
10. Update `baseline/README.md`, the findings and the plan, then push.

**The ALTO training section** was downloading at 00:47 as `data/raw/alto/Unconfirmed 150932.crdownload`. When the browser finishes, the file should be named `Train.zip` in that folder. `load_alto_flight(AltoConfig(section="Train"))` then reads it.

**Working rules from Dustin:** pull, then commit, then push, and say what the pull brought in. No assistant attribution in commits. Plain language in documents, no "not X but Y" phrasing, no middle dots.

## Current objective

The team stays with Challenge 2 (decided Friday night). Saturday morning: agree roles, merge the open branches, record the phone walk, and move the ALTO experiment into shared code.

## State

- Challenge 2, navigation without GNSS. Team name Taipei Drift, repository github.com/dwn97/TaipeiDrift (private). Six members have write access: dwn97, alessandrodipiano, ferneediaz (Dan), IlhanTech, FelixZukunft, rychardsandreireyes-rgb. The last two have not pushed anything yet.
- We build software, not a drone. Target, set with the mentor: low-cost drones, with about 500 dollars as an orientation and not a hard cap. The one thing: position fixes from the drone's own camera against free aerial images, with a check that rejects a wrong fix.
- `docs/PLAN.md` is the plan (three results, roles, timeline, slide story, next steps, mentor feedback). `docs/findings.md` holds every measurement. `docs/landscape.md` holds the research on Raptor and VNS01.
- Data is on Dustin's laptop under `data/raw/` (not committed). `data/README.md` says how to get it.
  - Mid-Air: all 38 archives, complete and verified (9.8 GB).
  - ALTO: the validation section (1.73 GB). The training section (9.93 GB) is not downloaded.
- Experiments on `main`, scripts `e` to `m` in `experiments/`. Headline numbers:
  - Mid-Air IMU-only drift: 485 m after 78 s in the median, 50 m after 36 s.
  - ALTO end to end: camera only 472 m median error, 26 to 31 m with a fix every 100 to 300 m.
  - One fix costs 285 ms on one laptop core, 20 ms at 125 pixels, with the same fix error.

## Branches on GitHub

| Branch | Owner | Content | State |
|---|---|---|---|
| `mid-air-baseline` | Alessandro | Package `baseline/`: Mid-Air loader, IMU dead reckoning, metrics, plots, 50 tests | Uses the textbook gyroscope rule, which is wrong on Mid-Air |
| `mid-air-baseline-fix` | pushed from Dustin's laptop | The same plus one commit: gyroscope rule, scipy fix, 3 tests | 53 tests pass. Median 491 m on the 30 sunny flights. Waiting for Alessandro to confirm |
| `simulations` | Dan | Gazebo in Docker, drone with camera, IMU, barometer, GNSS; worlds `terrain` and `islands` | Pull request 1 is open, without description or review. No recording or export code yet |
| `docs/denseuav-critical-review` | Ilhan | A review of the DenseUAV idea with a proposal for verified fixes | One document, 547 lines |

Merge test on Saturday 00:35: each branch merges into `main` cleanly. Merged one after the other, the only conflict is in `.gitignore` (the baseline and the simulator both add lines at the end). After merging all three, 53 tests pass.

## Notes for building on the team's code

- The baseline's `Trajectory` type requires accelerometer, gyroscope and attitude for every sample. ALTO has none of these, so the ALTO loader needs its own type or the fields have to become optional. The metrics only need a timestamp and a position, so they can be reused.
- The baseline imports its code as `src.*` with `baseline/` on the path. New code for ALTO fits next to it: `src/data/alto.py`, `src/estimation/`, `src/evaluation/`.
- The simulator publishes `/imu/data`, `/air_pressure`, `/gps/fix`, `/ground_truth/odom` and `/camera/down/image_raw`. Its IMU is in the drone's own axes. The image has rosbag with mcap storage installed, but nothing records yet. Its frames are East, North, Up.
- The simulator's gyroscope noise bounds are below what Mid-Air contains (median 0.014 to 0.021 rad/s per sample).
- Ilhan's review specifies states (ACQUIRING, TRACKING, DEGRADED, LOST), rejection reasons and metrics. It proposes SIFT keypoints as the verifier; on ALTO keypoints fail and brightness matching works (`docs/findings.md`, 3.2).
- ALTO orientation: x forward, y right, z down. The camera frame needs a rotation of 90 degrees minus the heading (`experiments/m_alto_orientation.py`).

## Open issues

- Roles are not assigned. Targets for the integrity check are not agreed.
- Results 2 and 3 exist only as experiment scripts.
- ALTO results are tuned and reported on the same 4.6 km section. The training section has to be downloaded in a browser.
- Turns are not handled on ALTO.
- The phone walk has no owner. It needs daylight.
- Nothing has been timed on a small board.
- `docs/PLAN.md` contains a mentor table and `research/` contains other authors' papers. Remove both before the repository is made public. `docs/brief.md` paraphrases the members-only challenge page.
- Mid-Air is licensed for non-commercial use.
- The formulas in `docs/data.md` and in the review use `\[ ... \]`, which GitHub does not render.
- The native uv is at `~/.local/bin/uv` on Dustin's laptop; the `uv` on the shell path is still the Intel build.

## Next exact step

Saturday morning, in this order: Alessandro confirms `mid-air-baseline-fix`; merge the three branches into `main`; the team fills the roles table; then `experiments/h_alto_end_to_end.py` is turned into shared code for result 2.

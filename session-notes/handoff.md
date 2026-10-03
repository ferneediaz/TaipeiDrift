# Handoff

Last updated: Saturday 3 October 2026, 08:35.

## Resume here

**Where we are.** Branch `alto-navigator` holds the camera navigator as shared code inside `baseline/` (commit d29b300). It is `main` plus the Mid-Air gyroscope fix plus the navigator. It reproduces all seven numbers of `experiments/h_alto_end_to_end.py` exactly; 91 tests pass. Run it with `python baseline/scripts/run_alto_navigator.py`. How it works and the results table are in `baseline/README.md`.

**Rules from Dustin for the repo.** Never push to `main`; work on a branch and push only that branch, by name. Pull, then commit, then push, and say what the pull brought in. No assistant attribution in commits. Documents for the team also go on the working branch now; the team merges into `main`.

**Next, in this order:**

1. **Held-out test on the training section** (build step 2). Blocked: the download of `Train.zip` stopped at 01:44 at 10.25 of about 10.66 GB (`data/raw/alto/Unconfirmed 150932.crdownload`, downloaded with Brave). Dustin has to press Resume in Brave's download list. The missing tail holds the three position files (query, reference, matches), which the archive stores last; all camera frames and the main reference images are already in the file. When the download is complete it is called `Train.zip`, and `python baseline/scripts/run_alto_navigator.py --section Train` runs the test with the settings untouched.
2. **What to expect on the training section,** from a look at the images: 28.5 km and 10,436 frames, six times the validation section; fields, long dark forest, villages, a town; about 16 percent of frames with very low contrast, in stretches of up to roughly 760 m; and probably heading changes, which the fixed angle learned before the jam does not follow.
3. **Wrong-fix numbers and a stronger check** (build step 4). The reading suggests checks that need no tuned threshold: agreement of consecutive frames (Tomahawk) or of crops of one frame (UASTHN). See `docs/reading-notes.md`. Build only after the held-out test shows whether the 0.33 threshold holds.
4. Then the rest of the build order in `docs/PLAN.md`: limits with darkened and blurred frames, the demo view, turns, the phone walk.

**Found last night:**

- The navigator now states its own uncertainty and a status (tracking, degraded, lost). In the runs that work, the error stays within 3 times the stated uncertainty in 97 to 100 percent of frames; in the run that breaks (400 m between fixes, no check) only in 24 percent.
- A limit, kept as a test: with a map whose coordinates are 200 m off, the first five fixes are rejected, but after 600 m without a fix the allowed distance has grown past 200 m and a confident wrong fix is believed. The distance check alone cannot catch a wrong place inside the stated uncertainty.
- Right after each fix the error grows again by 10 to 15 percent of the distance flown. That drift is mostly systematic (scale and direction), so two consecutive fixes could re-fit the motion matrix in flight. Not built.

**Saturday morning (09:30):** frame agreement tested as a replacement for the score threshold and dropped: frames 14 m apart see the same ground and agree on the same wrong place (findings 3.6). The option stays in the code, switched off. Tomahawk is not a GNSS-denied design (its camera fixes followed GPS or terrain-matching updates); the reading notes now say so. Findings and plan are updated on this branch: sections 2.6 (Alessandro's visual-inertial odometry), 3.6, 3.7, and the build-order state.

**Reading:** about 15 papers read, notes in `docs/reading-notes.md`.

## Current objective

The team stays with Challenge 2. Saturday: agree roles, decide how the branches reach `main`, record the phone walk in daylight, run the held-out test, then the next build steps.

## State

- Challenge 2, navigation without GNSS. Team name Taipei Drift, repository github.com/dwn97/TaipeiDrift (private). Six members have write access: dwn97, alessandrodipiano, ferneediaz (Dan), IlhanTech, FelixZukunft, rychardsandreireyes-rgb.
- We build software, not a drone. Target, set with the mentor: low-cost drones, with about 500 dollars as an orientation and not a hard cap. The one thing: position fixes from the drone's own camera against free aerial images, with a check that rejects a wrong fix.
- `docs/PLAN.md` is the plan (three results, roles, timeline, build order, slide story, next steps). `docs/findings.md` holds every measurement, `docs/landscape.md` the research on Raptor and VNS01, `docs/reading-notes.md` the papers.
- Data is on Dustin's laptop under `data/raw/` (not committed). `data/README.md` says how to get it.
  - Mid-Air: all 38 archives, complete and verified (9.8 GB).
  - ALTO: the validation section (1.73 GB) complete; the training section stopped at 96 percent.

## Branches on GitHub

| Branch | Owner | Content | State |
|---|---|---|---|
| `main` | | Plan, findings, product research, experiment scripts | Felix pushed `TRN/` directly to `main` on Saturday 08:29 (3a9f96d): a terrain-navigation plan, data inspection scripts, candidate routes. Not reviewed yet |
| `alto-navigator` | Dustin and the assistant | `main` + the Mid-Air fix + the camera navigator | 91 tests pass |
| `mid-air-baseline` | Alessandro | Package `baseline/`: Mid-Air loader, IMU dead reckoning, metrics, plots | Textbook gyroscope rule, wrong on Mid-Air |
| `mid-air-baseline-fix` | pushed from Dustin's laptop | The same plus the gyroscope fix, the scipy fix and 3 tests | Alessandro has made his own version of the fix in `mid-air-vio` |
| `mid-air-vio` | Alessandro, new on Saturday 08:14 | A `vio/` package, 53 files: error-state Kalman filter, optical flow, feature tracking, tests on Mid-Air | Not reviewed yet |
| `simulations` | Dan | Gazebo in Docker, drone with camera, IMU, barometer, GNSS; worlds `terrain` and `islands` | Pull request 1 open, no description or review |
| `docs/denseuav-critical-review` | Ilhan | A review of the DenseUAV idea with a proposal for verified fixes | One document |

## Notes for building on the team's code

- The camera navigator lives next to Alessandro's code in `baseline/`: `src/data/camera_flight.py`, `alto.py`, `synthetic_camera.py`; `src/estimation/image_motion.py`, `map_matching.py`, `navigator_core.py`, `camera_navigator.py`; `src/evaluation/navigation_metrics.py`; `src/visualization/navigator_plot.py`.
- Positions in the camera code are (north, east) in metres from the first frame.
- The simulator publishes `/imu/data`, `/air_pressure`, `/gps/fix`, `/ground_truth/odom` and `/camera/down/image_raw`. Nothing records flights yet. Its frames are East, North, Up.
- Ilhan's review specifies states, rejection reasons and metrics. It proposes keypoints as the verifier; on ALTO and in the papers, keypoints fail between camera and map.
- ALTO orientation: x forward, y right, z down. The camera frame needs a rotation of 90 degrees minus the heading (`experiments/m_alto_orientation.py`).

## Open issues

- Roles are not assigned. Targets for the integrity check are not agreed.
- The ALTO results are tuned and reported on the same 4.6 km section until the training section is complete.
- Turns are not handled.
- The phone walk has no owner. It needs daylight.
- Nothing has been timed on a small board.
- Two lines of work on Mid-Air now exist (`mid-air-baseline-fix` and Alessandro's `mid-air-vio`); the team has to decide which one `main` gets.
- The team now works on four separate approaches: the camera navigator on ALTO, visual-inertial odometry on Mid-Air (Alessandro), terrain navigation (Felix, `TRN/`), and the simulator (Dan). The mentor's advice was to pick one thing; the team should decide what the one thing is and how the others support it.
- `docs/PLAN.md` contains a mentor table and `research/` contains other authors' papers. Remove both before the repository is made public. `docs/brief.md` paraphrases the members-only challenge page.
- Mid-Air is licensed for non-commercial use.

## Next exact step

Press Resume on the `Train.zip` download, then run `python baseline/scripts/run_alto_navigator.py --section Train` and write the result into the findings.

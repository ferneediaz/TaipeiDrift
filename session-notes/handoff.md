# Handoff

Last updated: Saturday 3 October 2026, 00:55.

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

## Overnight research, Saturday 3 October 00:00-09:30 (branch `research/offline-nav-evidence`)

- Start with `docs/research/overnight-synthesis.md`. Details per track in `docs/research/`; open questions in `questions.md`.
- Most important result: on ALTO Round 2 Train (37.4 km, never used for tuning, pre-registered, parameters frozen) the chain of `h_alto_end_to_end.py` gives a median of 94 m per 4.6 km section with a fix every 300 m (Val: 31 m). Treat the Val numbers as tuned on the test set.
- `data/raw/alto/Val.zip` and `UAV_Round2_Train.zip` are on Ilhan's laptop, fetched with per-file Dropbox links (method in `docs/research/datasets-replay-sim.md`).
- New experiment scripts use prefixes `n_` to `v` in `experiments/`; outputs in `data/processed/`. Optional dependencies: `uv sync --extra research`.
- Context for Alessandro's VIO work: `docs/research/contesto-alessandro.md` (Italian) and `docs/research/context-alessandro-en.md` (English).
- Context for Dustin's ALTO navigator, including the Round 1 Train position files that unblock his held-out test: `docs/research/context-dustin.md`.
- The ALTO scripts on this branch (`t_alto_heldout.py`, `r_alto_matchers.py`) still search the reference images centred on the true path. Rerun them with the one-map search of `alto-navigator` before quoting their numbers.

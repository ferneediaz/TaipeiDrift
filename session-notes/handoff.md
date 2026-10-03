# Handoff

Last updated: Saturday 3 October 2026, 14:20. Code freeze Sunday 10:00, demo 13:00.

## Resume here

**Where the work lives (read this first).**

- **Work on branch `integration`** in **`~/Projects/DefenseHackathon-sim`** (a git worktree outside iCloud). It holds every team branch (ours, Alessandro's `mid-air-vio`, `main` with Felix's TRN and Dan's simulator, Ilhan's patch and research, the DenseUAV review); 297 tests pass. It is up for merging into `main` as pull request 2 (https://github.com/dwn97/TaipeiDrift/pull/2); the team merges it, never us. `alto-navigator` was fast-forwarded to the same commit (90b863a) so old links still show the current docs; keep pushing both, or only `integration`.
- `~/Projects/DefenseHackathon` (branch `alto-navigator`) is the older checkout; `~/Desktop/DefenseHackathon` is stale (iCloud).
- Python: `~/.venvs/defensehackathon/bin/python` (`.venv` links to it). Run from outside the iCloud folder.
- Data: `~/Desktop/DefenseHackathon/data/raw.nosync` and `processed.nosync`, linked as `data/raw` and `data/processed` in both checkouts. Simulator recordings in `recordings/` (ignored by git), outputs in `outputs/`.
- **The working doc: `docs/status-saturday.md`** (link for the team: https://github.com/dwn97/TaipeiDrift/blob/integration/docs/status-saturday.md). Phone test plan: `docs/phone-sun-test.md`. Competitors: `docs/landscape.md`.

**Rules from Dustin.** Never push to `main`; work on a branch and push it by name. Pull, then commit, then push, and say what the pull brought in. No assistant attribution in commits or pull requests. Plain language, no middle dot, no "not X but Y". Explain math from small numbers. Communicate clearly: say where we stand, what changed and why; stop at checkpoints when asked. Check the clock with `date` before writing times.

**Simulator demo: done so far.**

1. The container `taipeidrift-sim` runs from `~/Projects/DefenseHackathon-sim/sim` (`docker compose up -d`), mounting the worktree at `/ws/TaipeiDrift`. Start: `docker compose exec -d sim bash -ic "cd /ws/TaipeiDrift && ros2 launch sim/launch/sim.launch.py cam_res:=512 gui:=false world:=terrain > /tmp/sim.log 2>&1"`.
2. Ground: `python sim/scripts/make_ground.py --aerial --fill-empty` (the 2020 photo, empty parts filled with the procedural landscape); trees off (`make_trees.py --count 0`).
3. Route: `python sim/scripts/plan_route.py` writes `sim/scenarios/wufeng_corridor.json` (4.8 km along the corridor both images cover, 100 m, 10 m/s, out north, 180-degree turn, back south).
4. Recorded: `recordings/wufeng_corridor_100m` (recorder `sim/nodes/recorder.py --image-rate-hz 5 --cam-res 512 --duration-s 660`, then `python3 sim/scripts/route_flight.py` in the container). GNSS stays on in the recording; the jam is applied by distance (450 m).
5. Loader `baseline/src/data/sim_replay.py`; runs `baseline/scripts/run_sim_navigator.py` (config `baseline/configs/sim_navigator.yaml`, `motion_fit: rotation_scale`): camera alone 76 m median, 2018 map 26.5 m, fresh 2020 map 17.3 m, no wrong fix, bound held 100 percent. Video: `python baseline/scripts/make_replay.py sim --seed 3`.
6. Findings: the drone pitches 9.5 degrees in cruise, up to 16 in turns; the fixed down camera then looks 16 to 22 m behind; corrected with the true attitude the fix error is 2 to 5 m. Image of the down camera: top = forward, right = body right; frames are turned north up by the heading (`north_up`), zoom = height / 128 for the 0.5 m map.

**Next: the fused navigator** (Alessandro's ESKF plus our fixes, on the same recording). Plan:

- Reuse `vio/estimation/eskf.py` (`ESKF`, ENU world: gravity (0, 0, -9.81), `gyroscope_frame="body"`, IMU body FLU from `imu.csv`), start state from the truth at the jam (as his runner does at the cutoff).
- Updates: barometer height (his `update_altitude`, noise from his `BaroUpdateConfig`); a compass heading (truth yaw plus 4-degree offset and 1-degree noise, a new yaw update, H on the attitude error's world-up axis); down-camera velocity with his `camera_velocity_from_flow` (needs feature tracks and `R_bc` for the sim camera; derive it from `meta.json` `T_body_cam`, and note his `_flow_update` assumes NED down = (0, 0, 1), so pass world down = (0, 0, -1) for ENU).
- Map fix every 300 m: search centre = ESKF position plus where the camera looks (ray through the image centre, from the ESKF attitude and the barometer height); radius 3 sigma from the ESKF covariance, 60 to 600 m; zoom from the height; the fix minus that look offset is the drone's position, 15 m, through `ESKF.update` with the 99 percent gate and our confirmation of large jumps.
- Compare four runs on the same flight: camera alone, ESKF without fixes, fused, fused with the 2020 map. Alessandro's caveat: his filter's stated uncertainty was 2 to 5 times too small on Mid-Air; check NEES on the simulated flight before trusting its gate.
- Dan's new recorder (`sim/nodes/record_midair.py`) writes Mid-Air format; an alternative path to run Alessandro's own pipeline unchanged (down camera only; the simulated drone has no forward camera).

**Also left** (status doc section 7): computing time per fix and per frame and map storage per square kilometre; the README for the submission; slides with Dustin; team decisions on `main`.

## State, in short

- **ALTO validation (USA, real):** camera alone 472 m median; with map fixes every 300 m, 31 m. The search no longer uses the true path (findings 3.8).
- **UAV-VisLoc (China, real photos and a 2.5 to 5 years older map; dead reckoning simulated between photos):** development flight 03: wrong fixes used from about 12 to 0 to 2 with the new checks, median 28 m. Held out, run once: flight 04, 675 m without fixes to 60 m with them, but 18 to 32 wrong fixes still pass; flight 01, the ground changed since the map, fixes do not help (findings 3.9).
- **Limits (findings 3.10):** robust to 1/64 of the light; a dangerous failure (camera loses track, estimate stands still and stays sure of itself) closed by flying on at cruising speed; blur of about 4 m remains a blind spot.
- **Demo clips:** `outputs/replay/alto_val.mp4`, `outputs/replay/visloc_04_confirm_body_seed2.mp4`, stills next to them.
- **The mentor's phone sun compass:** `baseline/scripts/phone_sun_compass.py` is ready; photos go into `data/raw/phone_sun/chessboard/` and `sun/` (steps in the status doc). Alessandro has his own sun detector on `mid-air-vio`; use his.
- 134 tests pass on `alto-navigator`.

## Branches on GitHub

| Branch | Owner | Content | State |
|---|---|---|---|
| `main` | | Plan, findings up to Friday, experiments; Felix's `TRN/` ("Final TRN", baa4326) | Not reviewed by us |
| `alto-navigator` | Dustin and the assistant | The camera navigator, UAV-VisLoc, heading sensors, checks, limits, replay; the working doc | Pushed, 134 tests |
| `sim-demo` | Dustin and the assistant | Dan's simulator plus Ilhan's patch | Pushed 3e9458a; the demo work goes here |
| `simulations` | Dan | ROS 2 and Gazebo simulator | Last push Friday 22:19; pull request 1 open |
| `mid-air-vio` | Alessandro | Error-state Kalman filter on Mid-Air, benchmark, gyro-bias filter, sun detector | 13 to 33 m after 83 s on held-out flights; see status doc 5a |
| `research/offline-nav-evidence` | Ilhan | Overnight research (in French): ALTO Round 2 held-out test, quarters rule, Taiwan coverage, simulator patch | His ALTO numbers used the old search; asked to rerun |
| `docs/denseuav-critical-review` | Ilhan | Review of the DenseUAV idea | |
| `mid-air-baseline`, `mid-air-baseline-fix` | Alessandro, Dustin | IMU baseline on Mid-Air | Superseded by `mid-air-vio` |

## Open issues

- Which branches reach `main`, and the one story for the slides: team decisions.
- `docs/PLAN.md` contains a mentor table and `research/` contains other authors' papers; remove both before the repository is made public. Five more papers of Dustin's sit untracked in the old Desktop copy's `research/`, on purpose.
- Mid-Air is licensed for non-commercial use; UAV-VisLoc states no licence.
- Nothing has been timed on a small board.
- `docs/brief.md` paraphrases the members-only challenge page; check before the repository is made public (Ilhan).
- The formulas in `docs/data.md` and in the review use `\[ ... \]`, which GitHub does not render.
- The native uv is at `~/.local/bin/uv` on Dustin's laptop; the `uv` on the shell path is still the Intel build.

## Overnight research, Saturday 3 October 00:00-09:30 (branch `research/offline-nav-evidence`)

- Start with `docs/research/overnight-synthesis.md`. Details per track in `docs/research/`; open questions in `questions.md`.
- Most important result, rerun leak-free at 13:00 with our navigator (`experiments/w_dustin_heldout.py`): on ALTO Round 2 Train (37.4 km, never used for tuning, pre-registered, parameters frozen) the median of the section medians is 131 m with a fix every 300 m (Val: 31 m). Treat the Val numbers as tuned on the test set.
- `data/raw/alto/Val.zip` and `UAV_Round2_Train.zip` are on Ilhan's laptop, fetched with per-file Dropbox links (method in `docs/research/datasets-replay-sim.md`).
- New experiment scripts use prefixes `n_` to `v` in `experiments/`; outputs in `data/processed/`. Optional dependencies: `uv sync --extra research`.
- Context for Alessandro's VIO work: `docs/research/contesto-alessandro.md` (Italian) and `docs/research/context-alessandro-en.md` (English).
- Context for Dustin's ALTO navigator, including the Round 1 Train position files that unblock his held-out test: `docs/research/context-dustin.md`.
- The ALTO scripts on this branch (`t_alto_heldout.py`, `r_alto_matchers.py`) still search the reference images centred on the true path. Rerun them with the one-map search of `alto-navigator` before quoting their numbers.

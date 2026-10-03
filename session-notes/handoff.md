# Handoff

Last updated: Saturday 3 October 2026, 14:45. Code freeze Sunday 10:00, demo 13:00.

## Resume here

**Where the work lives (read this first).**

- **Work on branch `integration`** in **`~/Projects/DefenseHackathon-sim`** (a git worktree outside iCloud). It holds every team branch (ours, Alessandro's `mid-air-vio`, `main` with Felix's TRN and Dan's simulator, Ilhan's patch and research, the DenseUAV review); 297 tests pass. It is up for merging into `main` as pull request 2 (https://github.com/dwn97/TaipeiDrift/pull/2); the team merges it, never us. `alto-navigator` was fast-forwarded to the same commit (90b863a) so old links still show the current docs; keep pushing both, or only `integration`.
- `~/Projects/DefenseHackathon` (branch `alto-navigator`) is the older checkout; `~/Desktop/DefenseHackathon` is stale (iCloud).
- Python: `~/.venvs/defensehackathon/bin/python` (`.venv` links to it). Run from outside the iCloud folder.
- Data: `~/Desktop/DefenseHackathon/data/raw.nosync` and `processed.nosync`, linked as `data/raw` and `data/processed` in both checkouts. Simulator recordings in `recordings/` (ignored by git), outputs in `outputs/`.
- **The working doc: `docs/status-saturday.md`** (link for the team: https://github.com/dwn97/TaipeiDrift/blob/integration/docs/status-saturday.md). Phone test plan: `docs/phone-sun-test.md`. Competitors: `docs/landscape.md`.

**Rules from Dustin.** Never push to `main`; work on a branch and push it by name. Pull, then commit, then push, and say what the pull brought in. No assistant attribution in commits or pull requests. Plain language, no middle dot, no "not X but Y". Explain math from small numbers. Communicate clearly: say where we stand, what changed and why; stop at checkpoints when asked. Check the clock with `date` before writing times.

**Right now (14:45, before a context compaction).**

- **Flight 2 is recording** in the simulator: `recordings/wufeng_south_80m` (route `sim/scenarios/wufeng_south_80m.json`: south first, 80 m, 8 m/s, 5.1 km; the recorder runs 900 s). It is the honest test of the simulator settings, which were adapted on flight 1. When it has landed (`/tmp/flight.log` in the container ends with `done:`), run it with the settings frozen at save point 1: `python baseline/scripts/run_sim_navigator.py --recording recordings/wufeng_south_80m --route sim/scenarios/wufeng_south_80m.json`, and compare with flight 1 (camera alone 75.9 m, 2018 map 26.5 m, ideal 2020 map 17.3 m, seed 3). Report it plainly, good or bad. The first attempt stalled at the south U-turn (drone spinning on the spot); `route_flight.py` now stops and turns on the spot, with a stall guard.
- **Save point check running** (`python scripts/check_save_point.py`) to confirm the new zoom-edge error does not fire on ALTO or UAV-VisLoc. Flight 1 was already rerun: numbers identical.
- Commit 99f0a14 holds those fixes; save point 1 is tag `sp1-merged-camera-navigator` (f3a755f).
- The draft README for the jury is in the session scratchpad: `/private/tmp/claude-501/-Users-dustin-Desktop-DefenseHackathon/caaf3f37-cab1-430f-8ff7-5d98b7e5a250/scratchpad/README_draft.md` (Felix's TRN removed at Dustin's request). It goes in with save point 2.
- **Dustin decided:** leave Felix's TRN out of our story, README and checks (the folder stays in the repo). Concentrate on the simulation and test whether it works.

**The risks from the review (14:40) and what we do** (an independent review of the repo, checked):

- Dustin, today: download the organisers' suggested dataset (brief: IMU, speed, heading, reference position; link on the members-only page) into `data/raw/organiser/`, then we run a baseline and a correction on it; merge pull request 2; back up `outputs/replay` off the laptop; decide whether the repo goes public (then remove the two arXiv PDFs in `research/`, `docs/brief.md`, the mentor table in `docs/PLAN.md`, and Ilhan's DeepSeek line at `docs/research/overnight-synthesis.md:173`).
- Claude, save point 2 (after the flight-2 test): the README with every headline number as median, 90 percent and worst, labelled real or simulated; UAV-VisLoc steps in `data/README.md`; `sim/README.md` still says `git checkout simulations`; `docs/findings.md` still says 134 tests; final figures into `docs/figures/`; credits (OpenAerialMap CC BY 4.0, Google Earth through UAV-VisLoc) in README and video titles; hedge the NLSC free-maps claim (offline rights unconfirmed); the sun-sensor model gets a fixed per-flight error like the compass (now only random noise, which flatters it), rerun the UAV-VisLoc sun runs and report both; re-time the widest fix (11 zooms after 400 m without a fix, about 4 to 5 s, not 2 s), labelled Intel emulation; call the 2020 map an ideal bound (it is the ground itself); one consistency pass in the status doc (27 vs 41 m for flight 03, the wrong-fix threshold 25 vs 50 m, blur 8 and 16 m also let wrong fixes through).
- Accepted and said in the pitch: wrong fixes on look-alike ground (flight 04, whose 90 percent and worst values are worse with fixes), no gain on changed ground (flight 01), GNSS needed before the jam, no autopilot link (a MAVLink GPS_INPUT adapter only if time is left), Alessandro's VIO numbers rerunnable only with his Mid-Air downloads.

**After save point 2: the fused navigator, timeboxed to 20:00** (if it is not measurably better by then, the demo stays on the last save point). Its heading source on our down-camera-only drone is a simulated sun sensor, the micro sensor of Fan, Peng and Gao 2016 (Rev. Sci. Instrum. 87, 075003, checked from the PDF): 0.1 degree, field of view plus or minus 65 degrees (no reading when the sun is outside it, for example lower than 25 degrees or the drone banking away), 25 Hz, 35 g, 200 mW; plus a per-flight bias; with the compass as the fallback. The update goes into Alessandro's ESKF as `update_sun_direction` (residual s_meas minus R_hat transposed s_world, H on the attitude error = R_hat transposed [s_world]x; his version lives in `vio/estimation/gyro_bias_kf.py`). His README warns that the down camera alone loses the heading (NEES up to 226): the sun or compass update is what makes fusion possible here.

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

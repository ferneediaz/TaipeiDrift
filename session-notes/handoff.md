# Handoff

Last updated: Saturday 3 October 2026, 13:05. Code freeze Sunday 10:00, demo 13:00.

## Resume here

**Where the work lives (moved today, read this first).**

- Working copy: **`~/Projects/DefenseHackathon`**, branch `alto-navigator`, a fresh clone outside iCloud. The old `~/Desktop/DefenseHackathon` syncs to iCloud; with the disk 98 percent full macOS moved its files to iCloud (Python and git stalled). It is stale; do not work there.
- Simulator work: **`~/Projects/DefenseHackathon-sim`**, a git worktree of the same repository on branch **`sim-demo`** (Dan's `simulations` plus Ilhan's patch, commit 3e9458a).
- Python: `~/.venvs/defensehackathon/bin/python` (`.venv` in the working copy links to it). Run from outside the iCloud folder; imports from inside it hang.
- Data: stays in `~/Desktop/DefenseHackathon/data/raw.nosync` and `processed.nosync` (iCloud skips `.nosync`); `data/raw` and `data/processed` in the working copy link there. The worktree needs the same two links.
- **The one working doc for the team: `docs/status-saturday.md`** (what we did, results, done and left with owners). Measurements: `docs/findings.md` sections 3.8 to 3.10.

**Rules from Dustin.** Never push to `main`; work on a branch and push it by name. Pull, then commit, then push, and say what the pull brought in. No assistant attribution in commits. Plain language, no middle dot, no "not X but Y". Explain math from small numbers. Communicate clearly: say where we stand, what changed and why, before diving into work; stop at checkpoints when asked.

**Next: the simulator demo** (Dustin's priority after the compaction). Goal: one simulated flight with IMU, camera, barometer and a real map, where our navigator runs (and later Alessandro's filter with our fixes).

1. Docker image `taipeidrift-sim` is **built** (4.79 GB) and the container `taipeidrift-sim` was started from `~/Projects/DefenseHackathon-sim/sim` (`docker compose up -d`). It mounts the worktree at `/ws/TaipeiDrift`.
2. Ground = the real Wufeng 2020 aerial image: in the worktree, link `data/raw` to the `.nosync` folder, then `~/.venvs/defensehackathon/bin/python sim/scripts/make_ground.py --aerial` (needs rasterio; the image is `data/raw/aerial/wufeng_2020-03-23_x4.tif`). The plane is centred on the world origin, north up, at the image's true size. Consider fewer 3D trees (`make_trees.py --count`), since the photo already shows trees.
3. Coordinates. Both images are EPSG:3826 (TWD97, true metres).
   - 2020: 14,891 x 19,896 px at 0.143 m, bounds W 216099.2, E 218228.1, S 2659835.4, N 2662679.7. Its centre (E 217163.65, N 2661257.55) is the simulator's origin (x east, y north).
   - 2018 (our on-board map): 10,851 x 14,820 px at 0.195 m, bounds W 216121.2, E 218233.8, S 2659765.7, N 2662651.0. In simulator metres its top-left corner is north +1393.45, east -1042.45. Load it as a `GroundMap`, resampled to about 0.5 m per pixel.
   - Only 33 percent of the rectangle holds imagery: a motorway corridor 300 to 600 m wide. The flight must follow it; get the centreline from the coverage mask.
4. Flight: Ilhan's `sim/scripts/t_scenario.py` (velocity commands, altitude and heading hold from the simulator's truth) adapted: climb to about 120 m (90-degree camera: 240 m footprint, 0.47 m per pixel at 512 px), about 10 m/s along the corridor with its turns, GNSS cut after about 300 m (`gnss_cut_s`), `cam_res:=512 gui:=false` (real-time factor about 0.94). Record with `sim/nodes/recorder.py` (format `taipeidrift-replay/1`: CSVs imu, baro, gnss, images, truth). Known bug from Ilhan: the GNSS cut is written in simulation time, not in the recording's `t_s`.
5. A loader from the replay format to `CameraFlight` (frames, truth north/east, timestamps, heading from the truth quaternion plus compass noise as the sensor), the 2018 image as `ground_map`, then the navigator (`search: area`, `camera_motion_floor: 0.3`, `confirm_jumps`), then `make_replay.py` for the video. Run the navigator twice: with the 2018 image (two years old) and with the 2020 image (the ground itself, a fresh map as Ukraine's Eagle Eyes has). The difference is the price of an old map (`docs/landscape.md`, Eagle Eyes section).
6. Later, if time allows: Alessandro's ESKF on the same recording, with our fixes as position updates (his `ESKF.update(r, H, R, gate_prob)`; a fix is a north/east measurement with 15 m accuracy). His filter's uncertainty is 2 to 5 times too small and he flagged a missing propagation term; both matter before fusing (status doc 5a).

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

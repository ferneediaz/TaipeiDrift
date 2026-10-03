# Prompt: a deep review of the TaipeiDrift repository (for Claude Fable)

You are reviewing a hackathon repository the night before its code freeze. Find bugs and gaps that would
change a reported number, mislead the jury, or stop the jury from running the code. You have no context
beyond this prompt and the repository; read before you judge.

## The project

- **Event:** Taiwan Defense Tech Hackathon 2026 (NTU, Taipei). Challenge 2: drone navigation without GNSS.
  Code freeze Sunday 4 October 10:00 (Taipei time); a 3-minute demo at 13:00, 1 to 2 minutes of questions.
- **The brief requires:** a dead-reckoning baseline, at least one correction source or fusion method,
  plots of estimated against reference trajectory and of error over time, and the limits when sensors
  fail or noise rises. **It scores:** reduction in positioning error, technical validity, noise tolerance,
  computing and integration requirements, deployment feasibility, plus the user, deployment and scaling.
- **The solution:** a camera navigator. After GNSS is lost, it dead-reckons from optical flow of a
  downward camera (frames turned north up by a heading sensor), and every 300 m it fixes its position by
  correlating the camera frame with an older aerial photo (ZNCC with CLAHE, search radius 3 sigma,
  60 to 600 m). Fixes pass three checks (score, distance to the estimate, confirmation of large jumps). It
  states an uncertainty (sigma) and a status every frame. Heading: a simulated sun sensor (default) or a
  compass. Evidence: real data (ALTO, UAV-VisLoc) and a Gazebo simulation over Wufeng, Taichung, with
  the real 2020 aerial photo as the ground and the 2018 photo as the map.

## Where things are

- Repository: `~/Projects/DefenseHackathon-sim` (a git worktree, branch `integration`; `main` is the same
  code at save point 4, tag `sp4-sun-heading`). Python: `~/.venvs/defensehackathon/bin/python`. The shell
  is zsh.
- Read first: `docs/status-saturday.md` (section 2 explains the system step by step),
  `docs/simulation-results.md`, `docs/findings.md`, `session-notes/handoff.md`.
- The navigator: `baseline/src/estimation/camera_navigator.py`, `navigator_core.py`, `map_matching.py`,
  `image_motion.py`. Sensors: `baseline/src/sensors/` (`heading.py`, `sun_sensor.py`, `dead_reckoning.py`).
  Data adapters: `baseline/src/data/` (`sim_replay.py`, `camera_model.py`, `uav_visloc.py`, `alto.py`,
  `camera_flight.py`, `degrade.py`). Metrics: `baseline/src/evaluation/navigation_metrics.py`.
- Runners and checks: `baseline/scripts/run_sim_navigator.py`, `run_visloc_navigator.py`,
  `scripts/check_save_point.py` (the regression check), `scripts/sim_dev_check.py` (the improvement
  loop's criteria), `scripts/sim_progress.py`, `scripts/sim_figures.py`. Config:
  `baseline/configs/sim_navigator.yaml`.
- Simulator (Gazebo Harmonic, ROS 2 Jazzy, Docker): `sim/` (launch file, recorder, route flight, sensor
  noise). Recordings in `recordings/` (not in git), format described in `sim/nodes/recorder.py`.
- Not in the submission's story: `TRN/` (another teammate's terrain navigation). Alessandro's filter is
  in `vio/`; Dan's radio navigation in `sim/nodes/rf_*.py`.
- Uncommitted and under test right now: a fog option (`detail_scaling` in `camera_navigator.py`,
  `picture_detail` and `detail_for_flight` in `image_motion.py`, `detail_path` in
  `run_sim_navigator.py`). Review it too; it already showed a side effect in clear weather.

## Rules

1. **Read only.** Do not edit, commit, push, or delete anything. Write your report as your answer.
2. **Do not touch the sealed flights.** `recordings/wufeng_north_90m` and `recordings/wufeng_south_110m`
   must not be run, opened or plotted: they are the final test, run once after the freeze. Development
   flights (`wufeng_corridor_100m`, `wufeng_north_120m`, `wufeng_south_65m`) and the held-out flight
   (`wufeng_south_80m`) you may use.
3. You may run the tests (`cd baseline && ~/.venvs/defensehackathon/bin/python -m pytest -q`, about 2
   minutes) and small scripts on development flights. Do not start the simulator, and do not run jobs
   longer than about 10 minutes: the laptop is shared.
4. **Verify before you report.** For each finding show the code path, and where you can, a minimal
   reproduction (a few lines, a tiny input with a hand-checked answer). Mark each finding CONFIRMED (you
   reproduced it or traced it without doubt) or SUSPECTED (plausible, not shown). Do not pad the list.

## What to look for, most important first

1. **Truth leaking into the navigator after the jam.** The navigator may use GNSS truth only before the
   jam. Check every path from `position_gt`, `metadata["true_heading_deg"]`, `metadata["attitude_q"]` and
   `metadata["height_m"]` into `calibrate` and `navigate`. Sensor models may read the truth to simulate a
   sensor (the sun sensor, the realistic camera); the navigator itself may not.
2. **Frames and signs.** ENU (simulator) against north and east (navigator); heading = 90 degrees minus
   ENU yaw; `north_up` turning direction; body forward, left, up against the camera's image (top =
   forward); `Calibration.offset_at`; quaternion order (w, x, y, z) and `sun_sensor.rotations`,
   `yaw_and_tilt`; the sun sensor's levelling; the cloud-shadow sampling in `camera_model.py`. A sign
   error that a calibration absorbs on one flight can break another flight with other headings.
3. **Caches that can serve stale data.** `flow_path` and `detail_path` fingerprints (recording, heading
   settings, camera settings), the map cache `sim_map_<tif>_<mpp>.png`, the per-process `_FLIGHTS` cache,
   UAV-VisLoc and ALTO caches. Can a changed setting reuse an old result?
4. **The metrics and the checks.** `navigation_errors` (frame alignment after the jam), `integrity_summary`
   (nominal, misleading, hazardous, unavailable), the wrong-fix threshold (50 m), "within 3 sigma",
   medians over seeds against per-seed values. Does `scripts/check_save_point.py` catch a regression on
   ALTO (it runs UAV-VisLoc 03 seed 1 and simulated flight 1 seed 3)? Could the "works" criteria in
   `sim_dev_check.py` pass while something is wrong?
5. **Statistical honesty.** Three seeds, development flights all over the same corridor, settings tuned on
   them. Which claims in `docs/simulation-results.md` and `docs/status-saturday.md` section 2 does the
   evidence not support? Any number in the docs that no script produces, or that a script now produces
   differently?
6. **The simulator side.** The merged launch file (`sim/launch/sim.launch.py`: GNSS cut, stereo, wind,
   ships); the recorder (frames dropped under load: one sealed flight has 3,326 of 3,600 frames; do not
   open it, only reason about the effect); `sim_replay.load_sim_flight` (start and end of a flight,
   interpolation of the truth to image times).
7. **The sensor and camera models.** `sun_sensor.py` (field of view, photodiode solver, mounting error,
   tilt error, gyro hold, clouds) and `camera_model.py` (distortion direction, vignetting, exposure,
   determinism per frame). Physically wrong in a way that flatters the result?
8. **What the jury will miss.** Against the brief: can someone outside the team install and run it (setup,
   data downloads, the simulator build)? Is there a README that says what it does, the headline number, how
   to run it and the limits? Integration with an autopilot (none yet; a MAVLink `GPS_INPUT` adapter is
   planned), computing time on a small board (only laptop timings), night, fog, spoofing (a design point,
   not built).
9. **Tests.** Which important code has no test (`sim_replay.py`, `run_sim_navigator.py`, `camera_model.py`,
   the fog option)? Do any tests pass for the wrong reason?

## Your answer

1. **Findings,** most severe first. For each: CONFIRMED or SUSPECTED; severity (A: changes a reported number
   or a claim; B: could mislead the jury or a teammate; C: cosmetic or future); `file:line`; what is wrong in
   one sentence; a concrete scenario where it goes wrong; how you verified it; the smallest fix.
2. **Missing for the submission,** against the brief and the jury, each with an estimate of the work.
3. **Quick wins before 10:00,** at most five, each under an hour, ranked by value.
4. **What you checked and found sound,** briefly, so we know what not to recheck.

Write plainly, in short sentences. Explain any formula with a small numeric example. If you are unsure,
say so.

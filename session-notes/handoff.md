# Handoff

Last updated: Friday 2 October 2026, 21:50.

## Current objective

Confirm the plan as a team, assign roles, and produce the IMU-only baseline plot on Mid-Air.

## State

- Challenge 2, navigation without GNSS. Team of six, team name Taipei Drift, repository github.com/dwn97/TaipeiDrift (private).
- We build software, not a drone. Assumed platform: an existing drone with a downward camera, an IMU and a barometer. Flight height is not fixed.
- The single plan is `docs/PLAN.md`: a navigator (IMU baseline, camera speed, position fixes), an integrity check, and a drift budget. Not yet confirmed by the team.
- Data is on Dustin's laptop under `data/raw/` (not committed). `data/README.md` says how to get it.
  - Mid-Air: sensor records of every flight in every condition, and the downward camera for 21 flights. Each flight lasts about 88 seconds.
  - ALTO: the validation section of the competition sample. Real helicopter frames, reference images along the route, true positions. No raw IMU and no height above ground.
- Dataset decision: Mid-Air for the IMU baseline, camera speed and fog. ALTO for position fixes on real images.
- `docs/data.md` (Alessandro) proposes the pretrained DenseUAV network for position fixes. The plan lists it as an optional upgrade after classical matching. This is not yet agreed.
- Branch `simulations` (Dan): a Gazebo simulator in Docker with a drone that carries camera, IMU, barometer and GNSS. Not merged into main. Its role is an open question in the plan.
- `experiments/e_midair_imu_noise.py` measures the IMU errors in Mid-Air. The gyroscope white noise there is about 0.02 rad/s per sample, well above the 0.0005 to 0.005 the simulator assumes.
- Research: `docs/landscape.md`, `docs/challenge-2-research.md`, `docs/method.md`, `research/` (two papers as PDF).
- `experiments/a` to `d` and `docs/experiments.md` hold earlier work on Taiwan imagery and elevation. The particle filter and the camera-matching findings carry over; the rest is not part of the current plan.

## Open issues

- The team has not confirmed the plan or assigned roles.
- Position fixes: classical matching first, or DenseUAV from the start. To settle with Alessandro.
- The simulator's role: demo view, integration test, or both.
- No product code yet: no reader, no baseline, no camera speed, no filter.
- The four data checks in the plan have not been run.
- `docs/PLAN.md` contains a mentor table and `research/` contains other authors' papers. Remove both before the repository is made public. `docs/brief.md` paraphrases the members-only challenge page.
- Mid-Air is licensed for non-commercial use.
- The formulas in `docs/data.md` use `\[ ... \]`, which GitHub does not render.
- The native uv is at `~/.local/bin/uv` on Dustin's laptop; the `uv` on the shell path is still the Intel build.

## Next exact step

Dustin writes the IMU-only baseline for one Mid-Air flight in a notebook and plots it against the ground truth. In parallel: agree roles and the two open questions with Alessandro and Dan.

# Handoff

Last updated: Friday 2 October 2026, 22:45.

## Current objective

The team decides tonight whether to stay with Challenge 2, and if so confirms three results and their owners. The evidence is in `docs/findings.md`.

## State

- Challenge 2, navigation without GNSS. Team of six, team name Taipei Drift, repository github.com/dwn97/TaipeiDrift (private).
- We build software, not a drone. Assumed platform: an existing drone with a downward camera, an IMU and a barometer.
- `docs/findings.md` holds everything measured on Friday night. `docs/PLAN.md` predates those measurements and points to them; it is not yet rewritten.
- Data is on Dustin's laptop under `data/raw/` (not committed). `data/README.md` says how to get it.
  - Mid-Air: sensor records of every flight in every condition, and the downward camera for 21 flights (download was at 29 of 38 files at 22:32).
  - ALTO: the validation section of the competition sample.
- Measured so far, all as experiment scripts in `experiments/` (`e` to `k`):
  - Mid-Air IMU-only drift: 485 m after 78 s in the median, 50 m after 36 s.
  - Mid-Air gives the gyroscope around the map's axes. The attitude step goes on the left.
  - Camera speed with a barometer fails on Mid-Air (37 and 161 percent error).
  - ALTO end to end: camera only 472 m median error, about 30 m with a fix every 100 to 300 m.
  - Fixes fail when the gap exceeds 300 m with a fixed search. A score threshold and a search sized by the uncertainty repair that.
  - Keypoint matching fails on ALTO, brightness matching works.
- `docs/data.md` (Alessandro) proposes the pretrained DenseUAV network for position fixes. Findings propose it as an optional upgrade.
- Branch `simulations` (Dan): a Gazebo simulator in Docker, now with a world of two islands. Not merged into main.
- Branch `mid-air-baseline` (Alessandro, 22:06): product code for the IMU-only baseline on Mid-Air. It uses the textbook gyroscope rule and needs the eight-line change described in findings section 2.2. A tested patch is in the session scratchpad as `midair_gyro_axes.patch`; it has not been pushed. 10 of its 50 tests fail with the locked scipy version, all in the synthetic circle flight.

## Open issues

- The team has not decided: stay or switch, the three results, the owners.
- No product code: the experiments are single scripts, with no shared data format, no pipeline and no demo view.
- ALTO results are tuned and reported on the same 4.6 km section. The training section (9.93 GB) would give a held-out test.
- Turns are not handled on ALTO: the rotation is learned once and kept.
- Research on how Raptor and VNS01 work at night was started on Friday at 22:35; add the result to `docs/landscape.md`.
- `docs/PLAN.md` contains a mentor table and `research/` contains other authors' papers. Remove both before the repository is made public. `docs/brief.md` paraphrases the members-only challenge page.
- Mid-Air is licensed for non-commercial use.
- The formulas in `docs/data.md` use `\[ ... \]`, which GitHub does not render.
- The native uv is at `~/.local/bin/uv` on Dustin's laptop; the `uv` on the shell path is still the Intel build.

## Next exact step

After the team decision: rewrite `docs/PLAN.md` from section 6 of the findings, then turn `experiments/j_midair_imu_only.py` and `experiments/h_alto_end_to_end.py` into the shared pipeline.

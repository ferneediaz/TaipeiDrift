# Handoff

Last updated: Saturday 3 October 2026, 00:20.

## Current objective

The team stays with Challenge 2 (decided Friday night). Next: confirm the three results in `docs/PLAN.md` and their owners, then move the experiments into shared code.

## State

- Challenge 2, navigation without GNSS. Team of six, team name Taipei Drift, repository github.com/dwn97/TaipeiDrift (private).
- We build software, not a drone. Assumed platform: an existing drone with a downward camera, an IMU and a barometer.
- `docs/findings.md` holds everything measured on Friday night. `docs/PLAN.md` was rewritten at 22:55 to match it: three results, each with dataset, state and "done when". Owners are open.
- Data is on Dustin's laptop under `data/raw/` (not committed). `data/README.md` says how to get it.
  - Mid-Air: sensor records of every flight in every condition, and the downward camera for 21 flights. All 38 archives are complete and pass the integrity test (9.8 GB).
  - ALTO: the validation section of the competition sample.
- Measured so far, all as experiment scripts in `experiments/` (`e` to `k`):
  - Mid-Air IMU-only drift: 485 m after 78 s in the median, 50 m after 36 s.
  - Mid-Air gives the gyroscope around the map's axes. The attitude step goes on the left.
  - Camera speed with a barometer fails on Mid-Air (37 and 161 percent error).
  - ALTO end to end: camera only 472 m median error, about 30 m with a fix every 100 to 300 m.
  - Fixes fail when the gap exceeds 300 m with a fixed search. A score threshold and a search sized by the uncertainty repair that.
  - Keypoint matching fails on ALTO, brightness matching works.
- Branches on GitHub besides `main`:
  - `mid-air-baseline` (Alessandro, 22:06): product code for the IMU-only baseline. Uses the textbook gyroscope rule, which is wrong on Mid-Air.
  - `mid-air-baseline-fix` (pushed 23:00): the same plus one commit with the gyroscope rule, a scipy fix for the synthetic circle and three tests. 53 tests pass. Median error 491 m on the 30 sunny flights, in line with the experiment script. Not yet merged anywhere.
  - `simulations` (Dan): a Gazebo simulator in Docker, now with a world of two islands. Not merged into main.
  - `docs/denseuav-critical-review` (Ilhan, 22:46): a review of the DenseUAV idea with a proposal for verified fixes.
- `docs/data.md` (Alessandro) proposes the pretrained DenseUAV network for position fixes. The plan lists it as an optional upgrade.

## Open issues

- Owners of the three results are not agreed.
- Mentor feedback from Friday night is in `docs/PLAN.md`: the target is low-cost drones, with about 500 dollars as an orientation and not a hard cap, one thing done well, and a phone walk as a second demo. The phone walk has no owner and has not been tried.
- Results 2 and 3 exist only as experiment scripts, with no shared data format, no pipeline and no demo view.
- ALTO results are tuned and reported on the same 4.6 km section. The training section (9.93 GB) would give a held-out test. Not downloaded.
- Turns are not handled on ALTO: the rotation is learned once and kept.
- Research on how Raptor and VNS01 work, including at night, is done and written into `docs/landscape.md`.
- `docs/PLAN.md` contains a mentor table and `research/` contains other authors' papers. Remove both before the repository is made public. `docs/brief.md` paraphrases the members-only challenge page.
- Mid-Air is licensed for non-commercial use.
- The formulas in `docs/data.md` use `\[ ... \]`, which GitHub does not render.
- The native uv is at `~/.local/bin/uv` on Dustin's laptop; the `uv` on the shell path is still the Intel build.

## Next exact step

Alessandro checks out `mid-air-baseline-fix` and confirms it runs for him. Then the team fills the roles table in `docs/PLAN.md`, and `experiments/h_alto_end_to_end.py` is turned into shared code for result 2.

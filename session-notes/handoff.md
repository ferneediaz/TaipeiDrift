# Handoff

Last updated: Friday 2 October 2026, night.

## Current objective

Confirm the plan as a team, get the first Mid-Air data, and produce the IMU-only baseline plot.

## State

- Challenge 2, navigation without GNSS. Team of six, team name Taipei Drift, repository github.com/dwn97/TaipeiDrift (private).
- We build software, not a drone. Assumed platform: an existing drone with a downward camera, an IMU and a barometer. Flight height is not fixed.
- Dataset: Mid-Air (synthetic, low flight, IMU, downward camera, ground truth, several weather and season variants). No barometer and no aerial map in it.
- Second dataset under consideration: ALTO (real helicopter flights at over 300 m, downward camera, laser altimeter, aerial map). Only competition subsets on Dropbox are confirmed to exist; their content is unchecked. Blackbird was rejected.
- The single plan is `docs/PLAN.md`: a navigator (IMU baseline, camera speed, position fixes from ALTO map matching or Mid-Air route memory), an integrity check, and a drift budget. It also lists three checks to run on the data first, how results are measured, roles, timeline, gates, demo script and pitch content. Not yet confirmed by the team.
- Research: `docs/landscape.md`, `docs/challenge-2-research.md`, `research/` (two papers as PDF).
- `experiments/` and `docs/experiments.md` hold earlier work on Taiwan imagery and elevation with a laser and an airspeed sensor. The particle filter and the camera-matching findings carry over; the rest is not part of the current plan.

## Open issues

- The team has not confirmed the plan or assigned roles.
- Nobody has checked the ALTO Dropbox folders yet. Decide tonight, then stop looking at datasets.
- Nobody has requested the Mid-Air download links yet. The form needs a captcha, so a person has to do it. First selection: Down RGB, Kite training, sunny and foggy, trajectory 0003.
- No product code yet. No Mid-Air reader yet.
- The plan dropped satellite map matching and the navigability map of Taiwan when Mid-Air was chosen.
- `docs/PLAN.md` contains a mentor table and `research/` contains other authors' papers. Remove both before the repository is made public. `docs/brief.md` paraphrases the members-only challenge page.
- Mid-Air is licensed for non-commercial use.
- The native uv is at `~/.local/bin/uv` on Dustin's laptop; the `uv` on the shell path is still the Intel build.

## Next exact step

A team member requests the Mid-Air links and saves the text file into the repository folder. Then: fetch the sensor records, write the reader, and plot IMU-only dead reckoning against the ground truth for one flight.

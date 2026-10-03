# Tuniu Level 2: anti-cheat tests

Question: is the closed-loop result (median 3.3 m GNSS-free vs 54 m dead reckoning, [tuniu-level2-results.md](tuniu-level2-results.md))
real, or does the estimator secretly see the RTK truth? Three tests that a cheating system would fail, plus visuals.

Config for every test: real Tuniu photos 47–271 (225 per seed), heading `dji`, ground `dem_lifted`, closed loop,
seeds 0–4, estimator code unchanged (`experiments/x5_tuniu_closed_loop.py`, `run_sequence`).
Script: `experiments/x7_tuniu_anti_cheat.py`. Numbers: `data/processed/x_tuniu_l2/anti_cheat/summary.json`.

Reference, same 5 seeds: normal closed loop median error 3.0 m (per seed 2.7–4.1 m), 411 accepted fixes;
dead reckoning median 54.1 m, final 163.7 m.

## 1. Truth hidden

Setup. Everything the estimator may legitimately take from RTK (state at the cut, SIMULATED barometer series,
pre-cut calibration, stage-1 map offset) is precomputed into `anti_cheat/hidden_inputs.json`. The estimator then
runs in fresh processes where:

- `G.SEQ` points to `anti_cheat/replay_notruth/`: `images.csv`, `attitude.csv`, `meta.json` and the 226 photos
  used, with **no `truth.csv` and no `gnss.csv`**;
- the photos are copies with every EXIF/XMP/MPF segment removed (DJI writes GPS and RTK fields there); pixels
  unchanged (9/9 spot checks decode identically; 0 files still contain metadata);
- `x_tuniu_geo.truth_xy` raises, and a Python audit hook refuses any `open()` of `truth.csv`, `gnss.csv` or the
  original replay folder. A probe in a worker confirmed both paths are blocked (`hidden_raw/worker_probe.json`).

Scoring against RTK happens afterwards, in the parent.

| | A cheating system would… | Measured |
|---|---|---|
| Run completes | crash (PermissionError) | completes, 5/5 seeds |
| Post-cut estimates vs normal run | differ | max \|Δ position\| **4.7e-10 m** (CSV float round-off), 0/1125 fix-status and 0/1125 odometry-status differences |

Data files the estimator opened (audit log): the replay copy above, `stage1_calibration.json` (boresight + map
offset, pre-cut only) and the XFeat weights. Nothing else from `data/`.

## 2. Map shifted +30 m east

Setup. Copy of the main map with its georeference moved +30 m east, pixels unchanged
(`anti_cheat/maps/main_shift_e30_0.5m.tif`). The estimator still assumes the stage-1 map offset (+2.0 m E, −0.3 m N).
Runs: `runs/anticheat_shift30/`.

A system that uses the map must end up ~30 m east of the truth. A system that uses RTK would stay at ~3 m.

| | Normal | Shifted map |
|---|---|---|
| Median error (per-seed medians) | 3.0 m | **29.2 m** (28.6–29.5) |
| Accepted fixes − truth, median (E / N) | −0.2 / −0.7 m | **+29.8 / −0.6 m** |
| Estimate − truth, 2nd half of flight (E / N) | −0.5 / +0.5 m | **+28.6 / +0.8 m** |
| Accepted / gated fixes (5 seeds) | 411 / 3 | 394 / 17 |

What happens: the first 2–5 consensus fixes are rejected by the chi-square gate (30 m innovation vs a few metres of
uncertainty). The post-cut turn (photos 47–49, fallback motion) then inflates the predicted uncertainty to 13.5 m,
the gate opens at photo 52–57, and the filter locks onto the shifted map for the rest of the flight
(error 20–40 m on ~89 % of the remaining photos). The position follows the map, not the truth.

## 3. Wrong map

Two maps covering the flight area with the wrong content (`runs/anticheat_main_e250/`, `runs/anticheat_negs_here/`):

- **main +250 m east**: same scene, georeference moved 250 m east, so each window shows terrain from 250 m west.
  (+500 m, as first proposed, leaves the flight outside the map: no texture at all, too easy.)
- **negs moved onto the flight**: OAM scene `negs` (20190709 Miaoli Sanwan, ~0.9 km south, centre to centre)
  re-georeferenced by (−129 m E, +873 m N) so it sits on the flight area.

A cheating system would keep a few-metre error. A system that needs the map must lose every fix and fall back to
dead reckoning.

| | main +250 m | negs here |
|---|---|---|
| Accepted fixes / ZNCC–XFeat agreements within 4 m (5 seeds) | **0 / 0** | **0 / 0** |
| Single-method candidates (ZNCC / XFeat) | 855 / 42 | 1079 / 44 |
| Median error, final error | 54.1 m, 163.7 m | 54.1 m, 163.7 m |
| Max \|estimate − dead-reckoning estimate\| | **0 m** (bit-identical) | **0 m** (bit-identical) |

ZNCC always returns a best peak, and XFeat occasionally returns a homography on the wrong map, but they never agree,
so the pre-registered consensus rule rejects all of them. Without accepted fixes the estimator is exactly dead reckoning.
So every metre of improvement in the normal run comes through map fixes, and test 2 shows those fixes follow the
map's georeference.

## 4. Visuals (local only, derived from photos of unknown licence: not committed)

`data/processed/x_tuniu_l2/evidence/`:

- `overview_s0.png`: truth, closed loop, dead reckoning and accepted fixes over the map (seed 0).
- `checker_s0/fNNN.png`: every 10th test photo (23) + the 3 worst closed-loop photos (`worst_f259`, `worst_f258`,
  `worst_f226`, 39–43 m). Each image has three checkerboards: rectified photo squares alternating with map squares,
  placed at the **filter** estimate, at the **dead-reckoning** estimate, and at the RTK truth (reference).
  Roads, roofs and field edges continue across squares when the placement is right. `index.csv` lists the errors.
- `errors_anti_cheat.png`: error vs photo for normal / dead reckoning / shifted / wrong maps, 5 seeds each.

## Limits

- The audit hook only sees opens made from Python. The maps, the DEM (GDAL) and the photos (OpenCV) are read from C.
  This is covered by removing `truth.csv`/`gnss.csv` from the folder the code reads, stripping photo metadata and
  making `truth_xy` raise.
- Legitimate RTK-derived inputs remain: the state at the cut (1 m prior), the simulated barometer (truth altitude +
  simulated noise) and pre-cut calibration. Test 3 bounds what these can contribute: with no accepted fix, the
  output equals dead reckoning exactly.
- Test 2 also exposes a real limit. A map georeferencing error goes straight into the position, and the gate only
  delays it by a few photos once the uncertainty has grown. Absolute accuracy is bounded by the map's.
- 5 seeds, one flight, heading `dji` only.
- Harness guard (wrong-map runs only): on a fully empty reference window (outside the +250 m map), XFeat finds
  0 keypoints and `vismatch` raises `IndexError`. `x7` counts this as "no fix" (29 times in `main_e250`, 0 elsewhere;
  `anti_cheat/empty_ref_guard.json`). This latent crash in `x_tuniu_match.learned_fix` never occurs on the real map.

## Reproduce

```
.venv/bin/python experiments/x7_tuniu_anti_cheat.py precompute
.venv/bin/python experiments/x7_tuniu_anti_cheat.py hidden-run --workers 2
.venv/bin/python experiments/x7_tuniu_anti_cheat.py maps
.venv/bin/python experiments/x7_tuniu_anti_cheat.py run-maps --workers 2
.venv/bin/python experiments/x7_tuniu_anti_cheat.py summary
.venv/bin/python experiments/x7_tuniu_anti_cheat.py visual
```

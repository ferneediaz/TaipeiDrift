# Camera-to-map localization — final report

## Summary in 15 lines
1. **[MEASURED + SIMULATED]** The main benchmark covers 18 sites, 20 conditions, and 209,416 result rows (`r_map_benchmark_all/run.json`).
2. Across the six NLSC 2015→2023 sites, raw ZNCC is ≤10 m for 147/240 aligned positives; quad≥3 keeps 50 and UNION keeps 59 (`r_map_benchmark_all/matches.csv.gz`).
3. Across all sites/conditions, ZNCC quad≥3 gives 1,016 correct fixes/7,850 (1,017 accepted) and 1 false group/2,395 (95% bound: 0.198%; `integrity_summary.csv`).
4. ZNCC yaw/scale + quad≥3 gives 1,133 fixes ≤10 m/7,850 (1,135 accepted), 0 false groups/2,395 (bound 0.125%; same file).
5. UNION gives 1,878 fixes ≤10 m/7,850 (1,885 accepted), 0/2,395 false groups (bound 0.125%), but the confirmation run has one river false group (`r_map_benchmark_confirm/`).
6. **[MEASURED]** On real ALTO, ZNCC yaw/scale + quad≥3 accepts 44/300, with a 10.44 m median, 17.70 m maximum, and 0 negatives accepted (`r_alto_matchers/frames.csv`).
7. XFeat affine accepts 0/300 ALTO images; XFeat homography also accepts a negative (`r_alto_matchers/summary.csv`).
8. Synthetic haze retains 129/395 correct UNION fixes; 9 px blur retains 99/395, and 21 px blur retains 31/395 (`r_map_benchmark_all/integrity_summary.csv`).
9. UNION accepts 110/395 at 30° yaw, 126/395 at 90°; scale ×1.25 drops to 40/389 (`integrity_summary.csv`).
10. **[SIMULATED]** The final loop covers 19 sites, 4 seeds, and 7 variants; wall time was 46 min 25.6 s (`r_closed_loop_final/run.json`).
11. At a 10 s interval and 60 s blackout, union_driftgate reduces aggregated median error from 104.8 m (DR only) to 17.6 m; 868 fixes accepted, 0 false (`summary.csv`).
12. Without a gate, accept_all accepts 2,557 fixes, including 1,043 over 25 m (`summary.csv`).
13. With a 240 s blackout, union_adaptive ends with 18.9 m final median error, versus 220.7 m for DR only (`summary.csv`).
14. In prior-free search, the large aligned mosaic gets 40.6% top‑1 ≤10 m and 23.8% after quad≥3; the mosaic without the source site accepts 0/143 negatives (`r_lost_mode/summary.csv`).
15. **Recommendation:** ZNCC + quad≥3 + an innovation gate with drift covariance; use UNION only after validation on independent real river/water images.

## Scope and labels

- **[MEASURED + SIMULATED]** The map images are real orthophotos; the camera queries in the `r_map_benchmark*`, `r_closed_loop*`, and `r_lost_mode` benchmarks are rendered by the pinhole generator. Rates from these benchmarks are therefore not real-flight rates. Protocols and geometry: `data/processed/r_map_benchmark_all/run.json`, `r_closed_loop_final/run.json`, `r_lost_mode/run.json`.
- **[MEASURED]** ALTO contains real images and flight metadata; prior offsets and negatives are simulated. The matcher receives only the image and map window (`r_alto_matchers/run.json`).
- **[MEASURED]** The `r_matchers` latencies are CPU measurements on 10 pairs, not measurements of power or onboard compute (`r_matchers/timing.csv`).
- Output files provide detailed figures; the citations below identify the source file for each result.

## Methods compared

| Method | Rule | Reading the results |
|---|---|---|
| Local ZNCC | Normalized translation correlation over a local window, with a synthetic ±60 m prior. | Very cheap; sensitive to time offset, scale, and repetitive appearances (`r_map_benchmark_all/matches.csv.gz`). |
| ZNCC quad≥3 | Four disjoint 56 px subtemplates; at least three centers must be ≤4 px from the full ZNCC center. | This is a non-learned check with good precision but limited coverage (`experiments/r_map_benchmark.py`, `r_map_benchmark_all/integrity_summary.csv`). |
| ZNCC yaw/scale | Yaw hypotheses −20°…+20° in 10° steps and scales 0.9/1/1.1; same quad check. | Helps with small attitude/altitude errors, but not with a 20–25% scale change (`experiments/r_map_benchmark.py`, `r_map_benchmark_all/integrity_summary.csv`). |
| XFeat affine / homography | XFeat matches, RANSAC, geometric thresholds for scale, spread, and inliers. | Affine fails on ALTO; homography accepts a negative on ALTO (`r_alto_matchers/summary.csv`, `frames.csv`). |
| ZNCC × XFeat agreement | Two method families must localize within ≤10 m. | `zncc`/`xfeat_affine` agreement is conservative; agreement between two ZNCC variants is not method independence (`r_map_benchmark_all/integrity_summary.csv`, `false_accepts_by_land_cover.csv`). |
| UNION | Accepts quad≥3 from a ZNCC variant or Euclidean agreement ≤10 m across families; the envelope of all accepted candidates must remain ≤10 m per axis. | Best coverage in the main run, but a river false positive in the confirmation run (`experiments/r_integrity.py`, `r_map_benchmark_confirm/false_accepts_by_land_cover.csv`). |

## Integrity and false positives

Definition used by `experiments/r_integrity.py`: false accept = an accepted negative **or** a positive whose error exceeds 25 m. Repeated conditions from the same window/scene are grouped by `site|pair|centre|kind`; the one-sided 95% Clopper–Pearson upper bound assumes these groups are independent.

| Rule | Correct ≤10 m / positives | False groups / groups | 95% upper bound |
|---|---:|---:|---:|
| `zncc[quad>=3]` | 1,016/7,850 = 12.94% (1,017 accepted) | 1/2,395 | 0.198% |
| `zncc_yaw_scale[quad>=3]` | 1,133/7,850 = 14.43% (1,135 accepted) | 0/2,395 | 0.125% |
| `agree(zncc,xfeat_affine)` | 661/7,850 = 8.42% (665 accepted) | 0/2,395 | 0.125% |
| `UNION` | 1,878/7,850 = 23.92% (1,885 accepted) | 0/2,395 | 0.125% |

Source for all four rows: `data/processed/r_map_benchmark_all/integrity_summary.csv` (18 sites, 20 conditions). ZNCC quad has one positive over 25 m; UNION accepts no negatives in this run.

**Separate, smaller confirmation run:** `r_map_benchmark_confirm/integrity_summary.csv` aggregates 1,957 positives, 10,551 negatives, and 2,103 groups across six conditions. UNION gives 397 correct fixes/1,957 (20.29%), but has 3 false positives (accepted negatives) in **one** river group: bound 0.225%. `agree(zncc,xfeat_affine)` gives 165/1,957 (8.43%) and 0/2,103 false, bound 0.142%. The three river rows are shown in `r_map_benchmark_confirm/false_accepts_by_land_cover.csv`. Do not present the zero from the main run as a general guarantee.

### Six NLSC sites

2015 maps against queries rendered from 2023; aligned condition, 40 centers per site. Raw ZNCC is ≤10 m on 147/240 positives; quad≥3 retains only 50/240 (20.83%), UNION 59/240 (24.58%). Both gated rules have 0 false groups among 1,679 NLSC groups, bound 0.178% (`r_map_benchmark_all/matches.csv.gz`, integrity calculation on these rows).

| NLSC site | Raw ZNCC ≤10 m |
|---|---:|
| Changhua rice | 17/40 |
| Guanyin coast | 15/40 |
| Kaohsiung port | 23/40 |
| Nantou hills | 35/40 |
| Taipei urban | 39/40 |
| Zhuoshui river | 18/40 |

These differences show that a single threshold or rate is not representative of all six coverage areas (`r_map_benchmark_all/matches.csv.gz`).

## ALTO: real images

`r_alto_matchers/run.json` documents 300 sampled positive images from the ALTO Val flight, simulated negative windows at ≥600 m, a simulated prior of ±60 m per axis, and a map at 0.6 m/px. The matcher does not see the ground truth.

- `zncc_yaw_scale`, accepted only if quad≥3: **44/300** positive matches accepted; among them, 22/300 are ≤10 m, accepted median 10.44 m, maximum 17.70 m; none over 25 m and **0/300** negatives accepted (`r_alto_matchers/frames.csv`, `summary.csv`).
- `xfeat_affine`: **0/300** positive matches accepted. `xfeat_homography`: 2 positive matches accepted, but also **1** negative (`summary.csv`, `frames.csv`).
- Simple ZNCC accepts 25/300 positive matches after quad≥3; yaw/scale increases coverage to 44/300 (`summary.csv`).

The naive binomial 95% bound for 0/300 would be 0.994%; it is not an independent-flight bound because the images come from one correlated flight.

## Robustness to camera effects

`UNION`, 18 sites and generated conditions; fractions = accepted fixes ≤10 m / positives. Data by condition and false groups are in `r_map_benchmark_all/integrity_summary.csv`; all UNION rows in the main run have 0 false groups.

| Condition | UNION correct ≤10 m |
|---|---:|
| Aligned | 133/395 = 33.7% |
| Haze (0.55) | 129/395 = 32.4% |
| Motion blur 9 px / 21 px | 99/395 = 25.1% / 31/395 = 7.8% |
| Local shadow | 28/395 = 7.1% |
| Yaw 30° / 90° / 180° | 110/395 = 27.8% / 126/395 = 31.6% / 124/395 = 31.4% |
| Tilt 10° / rectified; tilt 20° / rectified | 95/391 = 24.3% / 107/391 = 27.1%; 59/375 = 15.5% / 110/381 = 28.6% |
| Scale ×1.25 | 40/389 = 10.3% |

`UNION` includes `zncc_heading` (heading search in 15° steps); yaw/scale search limited to ±20° no longer recovers a correct quad beyond 10°. Tilt rectification is **SIMULATED**, with 1° attitude noise; it is not a real IMU measurement (`r_map_benchmark_dates/run.json`, `r_map_benchmark_all/integrity_summary.csv`). The score collapses with shadows, strong blur, and scale errors outside the tested range.

## Matcher cost on CPU

Single-thread measurements, 10 pairs; latencies and localization errors (`r_matchers/timing.csv`, `r_matchers/loc_error.csv`).

| Matcher | Median / p95 latency | Median / p90 error |
|---|---:|---:|
| XFeat MNN | 44 / 58 ms | 4.46 / 127.42 m |
| XFeat + LighterGlue | 149 / 179 ms | 3.84 / 32.95 m |
| TinyRoMa | 66 / 75 ms | 3.28 / 4.94 m |
| ALIKED + LightGlue | 558 / 593 ms | 3.17 / 3.86 m |
| DISK + LightGlue | 696 / 943 ms | 3.49 / 3.89 m |
| SIFT + LightGlue | 2,152 / 2,432 ms | 3.83 / 10.43 m |
| RoMa outdoor | 12,416 / 13,350 ms | 3.52 / 3.90 m |

The sample is too small to choose a single matcher. RoMa outdoor is unsuitable for a correction every 10 s on this CPU; XFeat MNN has a high p90 error despite its low latency. The `r_matchers_smoke/timing.json` file has **no pairs**: `n_pairs=0`, `IndexError`; do not cite its latencies.

## Closed loop and truth leakage

**[SIMULATED]** The final test covers 19 sites, 4 seeds per site/configuration, 12 sweep conditions, and 7 variants; 912 tasks, 76 trajectories per summary row. Command: `time .venv/bin/python experiments/r_closed_loop.py --output data/processed/r_closed_loop_final`. Measured wall time: **46 min 25.6 s**; `run.json` reports 2,785.1 s (`r_closed_loop_final/run.json`).

Leakage check: `Estimator` receives odometry increments, initial/pre-cutoff GNSS state, images, map, and navigation offset derived from simulated attitude; its methods have no `truth` argument (`experiments/r_closed_loop.py`). In `run`, GNSS is injected only if `t < GNSS_CUT` (20 s); after the cutoff, truth is used by the trajectory/image generator and scorer, not by the estimator (`r_closed_loop_final/run.json`, `experiments/r_closed_loop.py`).

Measurements with 10 s between corrections and a 60 s blackout starting 60 s after GNSS cutoff. The `median_err`, `p95_err`, and `final_err` fields are medians of per-trajectory metrics, not pooled percentiles.

| Variant | Median error / median P95 / final | Fixes accepted / attempts | >25 m |
|---|---:|---:|---:|
| DR only | 104.8 / 208.4 / 220.7 m | 0/2,584 | 0 |
| Fixed UNION | 101.0 / 203.5 / 218.4 m | 442/2,584 | 0 |
| UNION + drift gate | 17.6 / 62.6 / 18.4 m | 868/2,584 | 0 |
| UNION adaptive | 23.4 / 80.4 / 17.8 m | 842/2,584 | 1 |
| Accept all, no gate | 10.2 / 171.5 / 6.1 m | 2,557/2,584 | **1,043** |

Source: `r_closed_loop_final/summary.csv`; false fix = accepted at >25 m (`WRONG=25` in the script). The very low median for accept_all therefore hides a high share of dangerous errors.

| Blackout, 10 s interval | UNION + drift gate: median / final error | Accepted / attempts |
|---|---:|---:|
| 0 s | 15.3 / 18.6 m | 1,035/3,040 |
| 60 s | 17.6 / 18.4 m | 868/2,584 |
| 120 s | 21.8 / 18.6 m | 715/2,128 |
| 240 s | 61.5 / 22.6 m | 355/1,216 |

At 240 s, `union_adaptive` gives 58.8 m median error and 18.9 m final error (393/1,216 fixes); the fixed window loses more corrections. Isolated false fixes occur elsewhere in the sweep: union_driftgate has one >25 m with no blackout and under `motion9`/`scale1.25` (`summary.csv`).

## Lost mode, without a prior

**[MEASURED + SIMULATED]** 18 sites, non-georeferenced mosaic of 23.31 km² (11,516×10,850 px), 288 tasks, aligned and 10° yaw conditions. Search: ×8 pyramid, 40 coarse peaks, full-resolution refinement ±64 px (`r_lost_mode/run.json`). Measured wall time **1 min 14.4 s**; `run.json` reports 71.5 s of search.

| Search region | Valid positives | Top‑1 ≤10 m, aligned | Quad≥3 correct, aligned | Verified XFeat correct, aligned |
|---|---:|---:|---:|---:|
| 384×384 px window | 143 | 59.4% | 26.6% | 17.5% |
| 768×768 px window | 90 | 55.6% | 22.2% | 12.2% |
| 1536×1536 px window | 84 | 48.8% | 23.8% | 13.1% |
| Full site | 143 | 49.7% | 26.6% | 18.2% |
| 23.31 km² mosaic | 143 | 40.6% | 23.8% | 17.5% |

In `mosaic_without_site`, source images are absent; **0/143** negatives are accepted by quad≥3 or verified XFeat, for each of the two conditions (`r_lost_mode/summary.csv`, `lost_mode.csv`). Median ZNCC time over the mosaic is 96.8 ms and verified XFeat time is 282.0 ms. The sample is modest; there is no reliable statistical bound. The 384/768/1536 windows are selected to contain the truth, so only the mosaic tests a true global search without a prior.

## Inventory of `r_*` outputs

Output counts and all cited files are under `data/processed/`. Older `run.json` files do not always record the full argv; the pinned reproduction commands below use the sites, seeds, methods, and conditions retained in the outputs.

| Directory | Reproduction command | Headline result and file |
|---|---|---|
| `r_map_benchmark/` | `C1` | 7 sites, 163,152 rows, 1,188.0 s; quad: 584/5,297 correct fixes, 585 accepted, 1/1,855 false groups. `integrity_summary.csv`, `run.json`. |
| `r_map_benchmark_all/` | `C2` + integrity | 18 sites, 209,416 rows, 1,727.2 s; main benchmark, 20 conditions. `run.json`, `integrity_summary.csv`, `matches.csv.gz`. |
| `r_map_benchmark_confirm/` | `C3` + integrity | 18 sites, seed 777, 75,048 rows, 1,779.2 s; UNION accepted 3 negatives as fixes, in 1 river group. `run.json`, `integrity_summary.csv`, `false_accepts_by_land_cover.csv`. |
| `r_map_benchmark_dates/` | `C4` | 6 dates, 5 pairs from 2018, 6,632 rows, 63.5 s; 2018→2019‑12: ZNCC ≤10 m on 12/28, median error 66.82 m. `matches.csv.gz`. |
| `r_map_benchmark_wufeng/` | `C5` + integrity | 1 site, 15,352 rows, 118.2 s; quad: 195/499 correct fixes, 0/175 false groups (bound 1.697%). `integrity_summary.csv`, `run.json`. |
| `r_map_benchmark_smoke/` | `C6` | 24 rows, 3.8 s; smoke test only, no statistical conclusion. `run.json`, `matches.csv.gz`. |
| `r_matchers/` | `C7` | 10 pairs; costs and errors detailed above. `timing.csv`, `loc_error.csv`, `benchmark_run.log`. |
| `r_matchers_smoke/` | old argv not preserved | 0 pairs; all calls fail with `IndexError`. `timing.json`. |
| `r_alto_matchers/` | `C8` | 300 real images; ALTO details above. `summary.csv`, `frames.csv`, `run.json`. |
| `r_closed_loop/` | `C9` | Old sweep: 7 sites × 4 seeds; 1,697.6 s. Superseded by the full run below. `summary.csv`, `run.json`. |
| `r_closed_loop_smoke/` | `C10` | 1 site, 3 seeds, UNION and UNION adaptive; at 240 s blackout, median errors 57.4 m and 21.2 m. `summary.csv`, `run.json`. |
| `r_closed_loop_final/` | `C11` | Full run and runtime measurement; final results above. `summary.csv`, `runs.csv`, `run.json`. |
| `r_lost_mode/` | `C12` | Global search without a prior; final results above. `summary.csv`, `lost_mode.csv`, `run.json`. |

## Reproducible commands

```bash
# C1 — seven NLSC sites + Wufeng
.venv/bin/python experiments/r_map_benchmark.py \
  --sites nlsc_changhua_rice,nlsc_guanyin_coast,nlsc_kaohsiung_port,nlsc_nantou_hills,nlsc_taipei_urban,nlsc_zhuoshui_river,oam_wufeng \
  --seed 20261003 --workers 8 --max-centres 40 --neg-same 6 --neg-other 2 \
  --output data/processed/r_map_benchmark
.venv/bin/python experiments/r_integrity.py data/processed/r_map_benchmark

# C2 — 18 sites, default methods/conditions
.venv/bin/python experiments/r_map_benchmark.py \
  --sites nlsc_changhua_rice,nlsc_guanyin_coast,nlsc_kaohsiung_port,nlsc_nantou_hills,nlsc_taipei_urban,nlsc_zhuoshui_river,oam_113498,oam_20a26d,oam_20a2ac,oam_221052,oam_4c92fa,oam_4c9306,oam_4c932a,oam_4c932e,oam_61702e,oam_c7cc77,oam_e6abbd,oam_wufeng \
  --seed 20261003 --workers 8 --max-centres 40 --neg-same 6 --neg-other 2 \
  --output data/processed/r_map_benchmark_all
.venv/bin/python experiments/r_integrity.py data/processed/r_map_benchmark_all

# C3 — repeat with seed 777, additional matchers, and six conditions
.venv/bin/python experiments/r_map_benchmark.py \
  --sites nlsc_changhua_rice,nlsc_guanyin_coast,nlsc_kaohsiung_port,nlsc_nantou_hills,nlsc_taipei_urban,nlsc_zhuoshui_river,oam_113498,oam_20a26d,oam_20a2ac,oam_221052,oam_4c92fa,oam_4c9306,oam_4c932a,oam_4c932e,oam_61702e,oam_c7cc77,oam_e6abbd,oam_wufeng \
  --seed 777 --workers 8 --max-centres 40 --neg-same 6 --neg-other 2 \
  --methods zncc,zncc_yaw_scale,xfeat_affine,lib:tiny_roma,lib:aliked_lightglue,lib:xfeat_lighterglue \
  --conditions aligned,legacy_degraded,motion9,scale1.25,tilt10_rect,yaw10 \
  --output data/processed/r_map_benchmark_confirm
.venv/bin/python experiments/r_integrity.py data/processed/r_map_benchmark_confirm

# C4 — OAM temporal pairs; C5 — Wufeng only; C6 — smoke test of two units
.venv/bin/python experiments/r_map_benchmark.py --sites oam_e9d0dc --pairs-mode oldest --seed 20261003 --workers 4 --max-centres 40 --neg-same 3 --methods zncc,zncc_yaw_scale,xfeat_affine,lib:tiny_roma --conditions aligned,legacy_degraded,motion9 --output data/processed/r_map_benchmark_dates
.venv/bin/python experiments/r_map_benchmark.py --sites oam_wufeng --seed 20261003 --workers 8 --max-centres 40 --neg-same 6 --output data/processed/r_map_benchmark_wufeng
.venv/bin/python experiments/r_integrity.py data/processed/r_map_benchmark_wufeng
.venv/bin/python experiments/r_map_benchmark.py --sites oam_wufeng --limit-units 2 --workers 2 --conditions aligned --methods zncc,lib:tiny_roma,lib:aliked_lightglue,lib:xfeat_lighterglue --output data/processed/r_map_benchmark_smoke

# C7–C8 — matcher cost and ALTO images
.venv/bin/python experiments/r_matchers.py --threads 1 10 --loc-error
.venv/bin/python experiments/r_alto_matchers.py

# C9–C12 — closed loops and lost mode
.venv/bin/python experiments/r_closed_loop.py --sweep default --seeds 4 --workers 5 --sites oam_wufeng,nlsc_changhua_rice,nlsc_guanyin_coast,nlsc_kaohsiung_port,nlsc_nantou_hills,nlsc_taipei_urban,nlsc_zhuoshui_river --output data/processed/r_closed_loop
.venv/bin/python experiments/r_closed_loop.py --sweep default --seeds 3 --workers 4 --sites oam_wufeng --variants union,union_adaptive --output data/processed/r_closed_loop_smoke
time .venv/bin/python experiments/r_closed_loop.py --output data/processed/r_closed_loop_final
time .venv/bin/python experiments/r_lost_mode.py --workers 8
```

## Failures, recommendation, and limitations

### What failed

- **UNION is not yet a safety threshold:** zero false in the main run, then one group of false positives on negative river images in the confirmation run (`r_map_benchmark_confirm/false_accepts_by_land_cover.csv`).
- Agreement between two ZNCC variants is correlated and produces false accepts; scores alone are not enough. UNION_v1, which adds homography geometry, also adds one urban false group (`r_map_benchmark_all/false_accepts_by_land_cover.csv`).
- XFeat affine alone accepts no ALTO images; homography accepts a negative. Accept_all gives 1,043 fixes over 25 m in the nominal closed-loop sweep.
- Large scale differences, blur, and shadows sharply reduce gated coverage; quad becomes nearly silent at 10° yaw if the heading hypothesis is not searched.
- The full-resolution lost-mode sweep exceeded 3,600 s without producing a file. The final result uses a coarse-to-fine search, documented in `r_lost_mode/run.json`; it finished in 74.4 s. The matcher smoke test had no pairs.

### Prioritized recommendation for the map module

1. **Main prototype:** local ZNCC with four subtemplates and quad≥3 acceptance; χ² innovation gate at 99%, covariance increased by drift since the last fix, and search window adapted to 3σ (384–1,024 px). Reject the fix rather than force a correction.
2. **Camera hypotheses:** test a bounded yaw/scale bank and tilt rectification only with available, calibrated attitude/altitude; the tested hypotheses do not cover ×0.8 or ×1.25.
3. **XFeat:** keep it as an optional verifier in domains where it has been validated; do not make it mandatory, since affine gets 0/300 on ALTO. Revalidate UNION on river imagery before using it as the primary gate.
4. **Do not deploy as primary:** RoMa outdoor (12.4 s/pair measured here), XFeat homography without additional validation, agreement between ZNCC variants alone, or top‑1 acceptance without a gate.

### Limitations and next steps

- OAM maps are under CC‑BY 4.0 with attribution; some are small-area UAV flights, not large orthophotos. Land-cover labels are heuristic (`data/raw/aerial_pairs/manifest_oam.json`). Verify rights for offline caching and redistribution of NLSC tiles before deployment.
- Queries in the map benchmarks and lost-mode test are rendered from maps; they do not reproduce a real camera, vibration, occlusion, or full terrain variation. ALTO is a single domain/flight, and successive images are correlated.
- Clopper–Pearson bounds assume independent groups; neighboring windows may remain correlated. The confirmation run's river false group proves that 0 observed false accepts is not an operational guarantee.
- Lost mode uses a mosaic of disjoint sites and positive windows that contain the truth by construction; it is not a continuous georeferenced map of Taiwan.
- Next steps: validate on real camera captures from different dates; build a set of river/water/repetitive-texture negatives; calibrate without reusing the test scene; test the full chain on the onboard computer and confirm offline tile licenses.

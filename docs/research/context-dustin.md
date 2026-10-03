# Context for Dustin: what this branch adds to the ALTO navigator

Branch `research/offline-nav-evidence`, written in the night of 2 to 3 October 2026 from `main` 1286d5c. This file reads alongside your branch `alto-navigator` (83adc59) and its findings 3.6–3.8. It says what this branch measured, where it complements your navigator, and what it could not do.

Labels used everywhere:
- **MEASURED**: run on real data;
- **SIMULATED**: declared generator;
- **PUBLISHED**: claim from a paper, with a link;
- **INFERENCE**: our reasoning, not verified.

## 0. In one minute

1. **Your blocked held-out test can start now.** The three position files of the ALTO training section (`query.csv`, `reference.csv`, `gt_matches.csv`, 2.9 MB together) are on Ilhan's laptop. Section 1 says where and how.
2. **We ran a held-out replication of the section 3.4 chain on Round 2 Train** (37.4 km, parameters frozen, protocol written before running). It reproduces Val in only 3 of 8 sections. **But it used the `offset_0_None` reference images: your leak.** It has to be rerun with your one-map search. Section 2.
3. **Integrity without a learned threshold**: four disjoint sub-templates of the same frame must agree ("quad ≥ 3"). It complements your finding that agreement of nearby frames fails. Section 3.
4. **Scale is the weak point**: on Round 2, the zoom calibrated from three fixes before the jam is often wrong. Section 4.
5. **Learned matchers do not help here**: XFeat 0/300 on real ALTO frames. Section 5.

## 1. Unblocking your held-out test

Your findings 3.7: the Round 1 Train download stopped at 10.25 of about 10.66 GB, and the three position files sit at the end of the archive.

| What | Where on Ilhan's laptop | Size |
|---|---|---|
| Round 1 Train positions: `query.csv`, `reference.csv`, `gt_matches.csv` | `data/raw/alto/round1_csv/Train/` | 2.9 MB |
| Round 2 Train, complete | `data/raw/alto/UAV_Round2_Train.zip` | 11.26 GB |
| Round 2 truth | `data/raw/alto/gt_matches_round2.csv` (13,783 rows) | 0.9 MB |
| Val (identical in both rounds: same names, same CRC) | `data/raw/alto/Val.zip` | 1.86 GB |

The fastest path is for Ilhan to send you the three Round 1 CSVs (AirDrop or chat). Your partial archive already holds the images. Datasets stay out of git, as `data/README.md` says.

What Round 2 Train is (MEASURED, from the archive inventory):
- it contains Round 1 Train (10,436 frames, 28.5 km, no timestamps) plus about 9 km further east;
- 75.8 % of its points lie within 20 m of Round 1;
- 13,782 frames at 20 Hz (step 0.050 s, no gap above 1 s), timestamped, median speed 54.6 m/s, altitude 437–548 m above the ellipsoid;
- reference images in **three** folders: `offset_0_None`, `offset_40_North`, `offset_40_South`, 3,744 each. Your one-map builder uses five folders on Val; Round 2 has no 20 m offsets;
- the Test sections of both rounds have no truth;
- nearest point of Round 2 Train to Val: 37.75 km. No geographic leak between them.

How to download per file if needed (the folder `dl=1` link returns HTML or stalls): list the shared folder with `POST https://www.dropbox.com/list_shared_link_folder_entries`. This needs the `__Host-js_csrf` cookie, passed both as `t` and as the `X-CSRF-Token` header, plus `link_key`, `secure_hash`, `sub_path`, `rlkey` and `link_type=c`. Each file then gets its own `…?rlkey=…&dl=1` link that accepts HTTP Range requests (response 206), so downloads can resume. Details: `docs/research/datasets-replay-sim.md`, section Q2.

## 2. Held-out replication on Round 2 Train (and what your leak changes)

Script: `experiments/t_alto_heldout.py` (`--phase val|train|all|finalize`).
- It refactors `h_alto_end_to_end.py`, and reproduces the 14 rows of findings 3.4 on Val within 1 m, with the same fix counts (`val_reproduction.csv`).
- The protocol was written before running: `data/processed/t_alto_heldout/preregistration.md`.

Results: 8 sections of about 4.6 km plus the full 37 km, every parameter frozen as tuned on Val. MEASURED.

| Configuration | Val (findings 3.4) | Median of the 8 sections | Min–max |
|---|---|---|---|
| Camera only | 472 m | 219 m | 68–420 m |
| Fix every 100 m, 7 nearest images | 26 m | 177 m | 17–725 m |
| Fix every 300 m, 7 nearest images | 31 m | 94 m | 20–339 m |
| Every 300 m, sized search + score 0.33 | 31 m | 128 m | 26–1,142 m |
| Every 1,000 m, sized search + score 0.33 | 56 m | 160 m | 50–980 m |
| Every 2,000 m, sized search + score 0.33 | 116 m | 191 m | 51–401 m |

- Val is reproduced only in sections 1, 4 and 8 (17–33 m).
- Fix every 300 m with 7 nearest images, summed over the 8 sections: 85 fixes used, 15 rejected, 35 of the used ones more than 50 m wrong (`data/processed/t_alto_heldout/summary.csv`).
- Full 37 km in one run, fix every 300 m: median 1,091 m, end 6,421 m; 43 fixes used, 56 rejected, 2 wrong (from `datasets-replay-sim.md`; config as reported there).

**Your leak applies here.** `t_alto_heldout.py` searches `offset_0_None`, so every fix was constrained to about 48 m of the true path. That made the test easier, so the conclusion "Val does not transfer" stands. The exact numbers do not:
- with your circle search around the estimate, the cliff disappears on Val;
- on Round 2 the result could go either way [INFERENCE]: more room to recover after a large drift, but also more look-alike places.

**Rerunning this with your one-map search, built from the three Round 2 folders, is the most useful single experiment for the team.**

Diagnosis, `experiments/t_alto_diag.py` (truth used only to score):
1. **The zoom calibrated from 3 fixes before the jam is the first cause.** With a better zoom, correct fixes rise from 21–43 % to 86–100 % in sections 2, 3 and 7.
   - Calibrated zooms per section: 1.00, 0.65, 0.60, 0.95, 0.80, 0.65, 0.65, 1.00. Three sit on an edge of the 0.60–1.00 grid.
   - The zoom is not monotonic with altitude: both 1.00 and 0.65 occur at 528 m.
   - Calibrated offsets reach 37 m, against 7.6 m on Val.
2. **The ZNCC score is not an oracle.** In sections 4–6 it peaks at the zoom grid edge (0.40), where the fixes are wrong.
3. **Matching fails whatever the zoom** in only one section (6).
4. **Without fixes, the camera's scale error ranges from −34 % to +19 %** by section, against −13 % on Val.

This matches your findings 3.7: exposure swings and low-contrast forest stretches, and heading changes in the town that the rotation learned before the jam does not follow.

## 3. Integrity: where our check complements yours

| Your navigator (findings 3.6) | This branch | Complement |
|---|---|---|
| Score check 0.33, tuned on Val | Score thresholds do not transfer: calibrated on one half of a map, they accept negatives on the other half (track A, MEASURED on real orthophotos with a simulated camera) | Do not trust a single tuned threshold on a new area |
| Distance check against the stated uncertainty. Its known limit: a wrong place inside the uncertainty passes (your 200 m offset test) | "Quad ≥ 3": four **disjoint** sub-templates of the same frame must land within 4 m of the full ZNCC match. No learned threshold | It does not depend on the distance, so it can catch exactly the case your distance check misses [INFERENCE: not tested on your offset map] |
| Agreement of 3 frames 14 m apart: dropped, because they share the same ground | Quad uses disjoint parts of one frame, not overlapping frames | A different failure mode. It can still be fooled by repetitive texture (rice paddies, rows) |

Quad ≥ 3 numbers:
- **Real ALTO frames** (`experiments/r_alto_matchers.py`, 300 frames): ZNCC with yaw/scale search + quad ≥ 3 accepts 44/300, median 10.4 m, max 17.7 m, 0 wrong, 0 of 300 negatives accepted. Raw ZNCC is within 25 m on 169/300: **the check throws away many good fixes.** This script also uses `offset_0_None` tiles: rerun it on your map.
- **Real orthophotos, simulated camera, 18 sites, 20 conditions** (`experiments/r_map_benchmark.py`, `r_integrity.py`): 1,133 correct out of 7,850 positives, 0 false on 2,395 negative groups (95 % upper bound 0.125 %).
- **Unseen real site, OrthoLoC** (`experiments/v3_ortholoc_heldout.py`, 60 real drone frames at about 100 m, 21–23° oblique): 3–5 accepted out of 60, 0 wrong, but 3 negatives out of 3,540 pairs accepted. **Not a universal zero.**
- **Round 2 sections** (`experiments/v1_round2_integrity.py`, exploratory, not held-out): with fixes every 300 m, quad ≥ 3 cuts wrong fixes > 50 m from 6 to 0, but fixes used from 35 to 13.
- **Closed loop with a drift-consistency gate** (`experiments/r_closed_loop.py`, SIMULATED camera from orthophotos, 19 sites): 868 of 2,584 fixes accepted, 0 false; median error 104.8 → 17.6 m. Without a gate, 1,043 of 2,557 accepted fixes are more than 25 m off.

Your stated uncertainty is overconfident exactly when it breaks (error above 3 sigma in 76 % of frames at the cliff). Alessandro has a consistency module (`vio/evaluation/consistency.py` on `mid-air-vio`). A shared NEES / within-3-sigma check for both navigators would make that visible in one plot.

## 4. Scale: the biggest lever and what failed

- **Diagnostic** (track B, truth used): with the flow-to-ground calibration known over the whole path, the end error on Val drops from 608 m to 74 m. With perfect height above ground: −30 %. With perfect heading: −6 %. Both: −39 %.
- **Online calibration of scale and heading from consecutive fixes: failed** its pre-registered criterion (`experiments/s_alto_online_calib.py`): 162 m at 1,000 m spacing, against your 56 m. Two fixes at ±15 m, 300 m apart, measure the scale only to about ±7 %. "Online + zoom" gets 63 m and 30 m at 300 m spacing.
  - Learned parameters went the right way: steps lengthened by 15–25 %, heading corrected by +3.2 to +4.2°.
  - The 300 m straight-line calibration of the flow-to-ground matrix is badly conditioned across the track.
- **The fix zoom is the best scale sensor we measured.** Suggestions [INFERENCE]: feed each accepted fix's zoom back as a scale measurement; widen the zoom grid (0.35–1.20); calibrate on more fixes before the jam.
- **Zoom predicted from height above ground** = (barometer − Copernicus DEM) [V1, exploratory, barometer SIMULATED since ALTO has none]:
  - alone, it doubles wrong fixes (12 against 6);
  - with quad ≥ 3, the median of section medians drops from 128 m to 70 m and the median end error from 833 m to 296 m, with 0 wrong fixes but only 10 of 118 attempts accepted.

## 5. Matchers, rotation and degraded frames

- **XFeat** (Apache-2.0, CPU): 0/300 on real ALTO frames, 1/60 on OrthoLoC. It works on orthophoto crops but collapses with motion blur (9 px: 12 %, 21 px: 0 %) and yaw ≥ 30°.
- **DenseUAV**: weights not downloadable by script; the Hugging Face repository holds only the dataset. Kinnari et al.: checkpoint withdrawn in 2025. **RoMa**: strongest in a published benchmark (AnyVisLoc, 70.1 % within 5 m with metadata priors, PUBLISHED), but heavy and not run here.
- **Rotation**: ZNCC with a yaw/scale search per fix handles your turn problem. In track A's tests, the UNION rule recovered yaw up to 180°. For oblique views, rectify with the drone's attitude: on OrthoLoC, without rectification nothing matches.
- **Degraded frames** (your next step), track A on real orthophotos with a simulated camera: haze is handled; motion blur of 21 px and a 25 % scale error make acceptance collapse.

## 6. Data you may want for the "full map" test

Your next step names UAV-VisLoc with full satellite maps. From our dataset audit (`docs/research/data-manifest.md`):

| Dataset | Real nadir frames | Truth | Map | Licence | Notes |
|---|---|---|---|---|---|
| UAV-VisLoc | Yes, 400–2,000 m | GPS per image, accuracy not published | 11 satellite maps from Google Earth | None stated; map rights belong to a third party | 16.4 GB, sample 2.04 GB |
| OrthoLoC (sample on Ilhan's laptop) | Yes, about 100 m, oblique | Pose per frame | Orthophoto + surface model per frame | CC BY-NC-SA 4.0 | Single frames, not a sequence |
| `tuniu_tw` (OpenDroneMap index), Miaoli, Taiwan | DJI Phantom 4 RTK photos, 100 m, camera at 60°, 80 % forward overlap | RTK | Build from the second flight, or NLSC/OpenAerialMap | Not stated; ask the author | 271 + 297 photos, 2.1 + 2.4 GB; the only real Taiwan drone imagery found |
| MARS-LVIG | Video at 10 Hz, 80–130 m | RTK | Not established | CC BY-NC-SA 4.0 | 8.5–30 GB per sequence |

## 7. Proposed work, in order

| # | Task | Done when | Estimate |
|---|---|---|---|
| 1 | Get the Round 1 Train CSVs from Ilhan, run your held-out test on Round 1 Train with the one-map search | Your findings table on Train, parameters frozen | 1–2 h |
| 2 | Same on Round 2 Train's 8 sections (three reference folders), to compare with section 2 above | Median, worst and end error per section; wrong fixes counted | 1–2 h |
| 3 | Try quad ≥ 3 as an extra acceptance check next to the score and distance checks (logic in `experiments/r_integrity.py`) | Wrong fixes and fixes used, with and without | 2 h |
| 4 | Feed the accepted fix zoom back as scale, and widen the zoom grid | End error vs the frozen chain on the same sections | 2–3 h |
| 5 | Shared consistency plot with Alessandro (NEES or within 3 sigma) | One function used by both navigators | 1 h |

Outside reviewers (Grok 4.7 and GPT-6 Astra, `docs/research/outside-reviews.md`) both advise to freeze features within about 12 hours, then spend the rest on figures, video and checking. They also say:
- never open the pitch with 26–31 m;
- show the acceptance rate next to every median;
- present the system as one that refuses to give a position rather than give a wrong one.

## 8. Files on this branch that matter to you

- `docs/research/overnight-synthesis.md`: overnight summary and ranked proposal.
- `docs/research/datasets-replay-sim.md`: held-out replication, diagnosis, Dropbox method, replay format, simulator patch.
- `docs/research/map-localization.md`: track A, matchers, integrity, robustness, closed loop, lost mode.
- `docs/research/sensor-fusion.md`: barometer model, drift budget, scale and heading analysis.
- `docs/research/context-alessandro-en.md`: what Alessandro gets for the VIO, in English.
- `experiments/t_alto_heldout.py`, `t_alto_diag.py`: held-out replication and diagnosis.
- `experiments/r_alto_matchers.py`, `r_integrity.py`, `r_map_benchmark.py`, `r_closed_loop.py`, `r_lost_mode.py`: matchers and integrity.
- `experiments/v1_round2_integrity.py`, `v3_ortholoc_heldout.py`: quad + zoom prior on Round 2, unseen OrthoLoC site.
- `experiments/s_alto_online_calib.py`, `s_alto_heading.py`, `s_alto_speed_residual.py`: scale and heading on ALTO.

Outputs are in `data/processed/` on Ilhan's laptop (not committed). Every script prints its command in its header.

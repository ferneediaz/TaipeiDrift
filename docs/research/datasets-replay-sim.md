# Track C: data, replay, and simulation (overnight report, 2026-10-03)

Labels: MEASURED (run on real data), SIMULATED (declared generator), PUBLISHED (cited source), INFERENCE. Detailed inventory: `docs/research/data-manifest.md`.

## Recommendation: the strongest honest demo

1. **Quantitative evidence from a real flight: ALTO Round 2 Train** (MEASURED). This real, timestamped 37.4 km flight was never used for tuning. Command: `AltoConfig(data_root="data/raw/alto/round2", section="Train")`.
   - **The frozen-parameter replication (next section) shows that Val's 26–31 m does not transfer**: median error of 94–177 m depending on the fix spacing, with large variation across sections.
   - For the demo: show the variance (3 good sections out of 8) and the mechanism, not Val's best figure.
2. **Visual story in Taiwan: `wufeng_sim_base`** (SIMULATED, real imagery). The camera is rendered from the 2020 orthophoto and the map is the 2018 orthophoto. The IMU, barometer, and GNSS are synthetic, with declared noise. This is the only nadir sequence in Taiwan with a map from a different year. Label it on screen “simulated sensors, real images.”
3. **Altitude on real data**: raw barometer with independent ground truth. Zurich provides photogrammetric ground truth, INSANE provides RTK ground truth, and PX4 provides two RTK logs. The measurements themselves belong to Track B.
4. **Gazebo**: only for a shot of “the system running in a loop with GNSS cut off,” with the patch below. It provides no quantitative evidence because the terrain is procedural.
5. **Avoid**:
   - Zurich for map localization: its camera faces forward at street height.
   - The 2nd PX4 video (`aa0ae4df`): its video/log offset is ambiguous.
   - YouTube videos in the jury video until their license is known.

## Held-out replication: ALTO Round 2 Train (negative result, MEASURED)

- **Protocol preregistered before any computation**: `data/processed/t_alto_heldout/preregistration.md`. All `h_alto_end_to_end.py` parameters are frozen at the values tuned on Val. Script: `experiments/t_alto_heldout.py`.
- **Fidelity**: the refactored code reproduces the 14 rows of findings table 3.4 on Val to within 1 m, with the same number of fixes (`val_reproduction.csv`).
- **Leakage**: none. The closest Round 2 Train point is 37.75 km from Val.
- **Data**: 8 sections of about 4.6 km and one complete 37.4 km flight. The numbers below were rechecked in `results.csv`.

  | Configuration | Val (findings) | Median across 8 sections | Min–max |
  |---|---|---|---|
  | camera only | 472 m | 219 m | 68–420 m |
  | fix every 100 m, 7 neighbors | 26 m | 177 m | 17–725 m |
  | 300 m, 7 neighbors | 31 m | 94 m | 20–339 m |
  | 300 m, sized search + threshold | 31 m | 128 m | 26–1142 m |
  | 1000 m, sized search + threshold | 56 m | 160 m | 50–980 m |
  | 2000 m, sized search + threshold | 116 m | 191 m | 51–401 m |

- **Complete 37 km flight** (median / final; fixes used / rejected / false by more than 50 m):
  - no fixes: 1120 / 2392 m;
  - fix every 300 m: 1091 / 6421 m (43 / 56 / 2);
  - every 1000 m: 1490 / 7240 m (14 / 15 / 2);
  - every 2000 m: 593 / 751 m (7 / 10 / 1).
- **Predictions**:
  - P1 false: median under 60 m in only 1, 3, and 3 of 8 sections for 100, 200, and 300 m.
  - P2 false: only 6 of 8 sections have no false fixes.
  - P3 false.
  - P4 true: threshold 0.33 rejects more correct fixes.
- **Implication (preregistered rule)**: Val's figures (26–31 m) must be presented as **tuned on the test set**. They reproduce in 3 of 8 sections (1, 4, and 8: 17–33 m) and fail elsewhere.
- **Probable mechanism (INFERENCE, to test)**: the calibration before the jam, done with 3 fixes and a zoom grid from 0.60 to 1.00, is unstable.
  - Calibrated zoom: 1.00; 0.65; 0.60; 0.95; 0.80; 0.65; 0.65; 1.00. Three sections hit a grid edge.
  - Zoom is not monotonic with altitude: both 1.00 and 0.65 at 528 m.
  - Calibrated offsets reach 37 m, versus 7.6 m on Val.
  - Scores for accepted fixes are lower (median by section 0.19–0.55).
  - Altitude is higher than on Val: 437–548 m above the ellipsoid versus 432 m.
- Files: `data/processed/t_alto_heldout/{results,summary,calibration,fix_scores,leak_check}.csv` and `sections.png`.
- Round 2 was not used to tune anything.

### Failure diagnosis (MEASURED; ground truth used only for evaluation)

Script: `experiments/t_alto_diag.py`. Outputs: `data/processed/t_alto_heldout/diag/`.

**Method.** Fixes are taken alone every 300 m, with a search centered on the true position. We compare two zooms: the zoom calibrated by the pipeline, and the zoom that maximizes the score over a broad 0.40–1.40 grid.

**Share of fixes within 30 m of ground truth:**

| Section | Calibrated zoom | Best-scoring zoom |
|---|---|---|
| Val | 71% | – |
| S1 | 86% | 86% |
| S2 | 43% | 100% |
| S3 | 21% | 93% |
| S4 | 57% | 7% (zoom 0.40) |
| S5 | 86% | 21% (zoom 0.40) |
| S6 | 21% | 14% (zoom 0.40) |
| S7 | 43% | 86% |
| S8 | 94% | 69% |

**My conclusions** (I do not use the sub-agent's “terrain” verdict):
1. **Zoom calibration is the first mechanism.** With a better zoom, S2, S3, and S7 improve from 21–43% to 86–100% correct fixes. Calibration with 3 fixes selects the wrong zoom.
2. **The score is not a reliable oracle.** In S4–S6, the best score falls at the lower edge of the grid (0.40), where the fixes are wrong (86 m). ZNCC score rises when zooming out strongly. Any automatic zoom calibration must be bounded or validated by something other than the score (INFERENCE).
3. **Matching really fails in only one section**: S6 stays at 14–21% correct fixes regardless of zoom.
4. **Camera-only dead reckoning is much more variable than on Val.** Without any fixes, implied scale error ranges from −34% to +19%, and heading error from −9.5° to +6.1%, versus −13% and −2.6% on Val. A 40 m search is therefore exceeded well before 300 m in several sections.

**Full replay with the best-scoring zoom** (labeled “diagnostic, uses ground truth”): mixed results. Examples: in S2, fixing every 300 m reduces error from 120 to 46 m; in S3, sized search every 1000 m reduces it from 704 to 23 m; S4 worsens from 33 to 301 m. The sections that worsen are those where zoom hits the 0.40 edge. This confirms points 1 and 2.

## Results

### Q2. ALTO
- **MEASURED**: `dl=1` on the Round 1 folder returns HTML. The full Round 2 zip archive (16.6 GB) stalled at 491 MB and cannot resume; abandoned.
- **Solution found**: list the shared folder using `POST https://www.dropbox.com/list_shared_link_folder_entries`. The `__Host-js_csrf` cookie must be passed in `t` and `X-CSRF-Token`, along with `link_key`, `secure_hash`, `sub_path`, `rlkey`, and `link_type=c`. Each file then has a link `…/<file>?rlkey=…&dl=1` that accepts Range requests (206 response).
- **Inventory**: folder `6gwa0swtzj7pg1itk89hn` contains both rounds.

  | Round | File | Size |
  |---|---|---|
  | Round 1 (`UAV/`) | readme | – |
  | Round 1 | Train.zip | 10.66 GB |
  | Round 1 | Val.zip | 1.86 GB |
  | Round 1 | Test.zip | 2.04 GB |
  | Round 2 (`UAV_Round2/`) | gt_matches.csv | – |
  | Round 2 | Train.zip | 11.26 GB |
  | Round 2 | Val.zip | 1.86 GB |
  | Round 2 | Test.zip | 3.51 GB |

  Zip inventories are in `data/processed/t_inventory/alto_*.csv`.
- **MEASURED, Val**: Round 2 Val is identical to Round 1 Val (same filenames, same CRCs). **`data/raw/alto/Val.zip` is in place**: `unzip -t` reports no errors, sha256 `e468050d…`, and the team's loader reads 1684 images.
- **MEASURED, Round 2 Train**: it reuses Round 1 Train, which has 10,436 images over 28.5 km without timestamps. It adds about 9 km to the east (E to 534,877 m versus 526,084 m); 75.8% of its points are within 20 m of Round 1. Time step 0.050 s, no gap over 1 s, median speed 54.6 m/s, altitude 437–548 m above the ellipsoid. The references are offset by 0, +40 m, and −40 m north, 3744 images each.
- `gt_matches.csv` at the Round 2 root has 13,783 rows: this is Train ground truth. **Neither Test has ground truth**, so they cannot be used for evaluation.
- **PUBLISHED** (https://arxiv.org/abs/2207.12317): the full ALTO dataset includes an LCI-1 IMU at 200 Hz, NovAtel SPAN solution (1.5 m RMS), and a laser altimeter at 20 Hz. It is not published: the README at https://github.com/MetaSLAM/ALTO says “Full Dataset: Coming soon!” ALTO has no barometer.

### Q3. Zurich Urban MAV
- **MEASURED, access**: the server accepts Range requests. `experiments/t_remote_zip.py` reads the central directory (ZIP64): 81,331 files, 29.8 GB. It then extracts selected files by grouping adjacent entries in a single request.
  - Throughput: 6.6 MB/s when measured alone, about 1–3 MB/s during Dropbox downloads.
  - Extracted window: PX4 clock 1795–2405 s, 18,221 images, 6.5 GB, in about 40 min.
- **Decision: do not download the full dataset** (28 GB). The camera is a GoPro 1920×1080 that looks **forward and to the side, at street height** (image verified). The drone is tethered and slow (median speed 0.7 m/s, 1869 m of ground-truth path in 45 min). This is useful neither for orthophoto localization nor for the scenario.
- **MEASURED, IMU**: `RawGyro` and `RawAccel` are only 10 Hz and alias vibrations (the correlation sign flips at ±50 ms). The 50 Hz gyro from `OnboardPose` is the rotated raw gyro, fitted by least squares: P ≈ A·raw, with A ≈ [[−0.64, −0.64, 0], [−0.66, 0.67, 0], [0, 0, −0.99]]. This corresponds to a 45° rotation with z pointing up, and about 0.65× amplitude on x and y, so it is a filtered signal.
- **MEASURED, image timestamps (new)**: `experiments/t_zurich_sync_check.py` correlates yaw rate from images (phase correlation) with gyro z.
  - Across the full window: best offset −1.10 s (r = 0.62), versus r = 0.16 at 0 s.
  - Across 20 s windows: about +0.2 s through 100 s, about +0.13 s through 160 s, then a jump to −1.24 s that drifts linearly to −0.90 s at 540 s (+0.95 ms/s, max residual 0.020 s across 18 windows), then another jump to −1.86 s around 563 s.
  - INFERENCE: the jumps come from dropped or duplicated images in the imgid → timestamp association.
  - **Implication**: any camera/IMU fusion on AGZ must correct this offset. The correction for the stable segment (183–563 s) is in `meta.json`.
- The sequence in the common format is `data/processed/t_replay/zurich_agz_1800_2400`. The validator reports OK:

  | File | Rows | Frequency |
  |---|---|---|
  | IMU | 29,840 | 50 Hz |
  | baro | 5975 | 10 Hz |
  | deduplicated GNSS | 2988 | 5 Hz |
  | images | 17,921 | 30 Hz |
  | ground truth | 597 | 1 Hz |

### Q4. Other real flights with raw barometer, camera, and ground truth
- **INSANE (AAU Klagenfurt)**: this is the only real dataset found with a downward-facing camera, raw barometer, and RTK ground truth.
  - License “Data: INSANE Dataset; License: BSD-2-Clause,” with no resale rights: https://cns-data.aau.at/insane-dataset/LICENSE.txt.
  - Sensors for outdoor_1, mars_1, and mars_2 were downloaded by Track B. Track C added the Mars1 images (2.4 GB).
  - **Sequence `data/processed/t_replay/insane_mars_1`**, validated (MEASURED): 100 s, 87 m, 0–5 m altitude. IMU 196 Hz, barometer 18 Hz, PX4 GPS 5 Hz, 1454 downward-facing images at 15 Hz, RTK ground truth at 8 Hz, intrinsics and extrinsics included.
  - **Sequence `data/processed/t_replay/insane_outdoor_1`**, validated (MEASURED, Klagenfurt airfield): sensors for 260 s, images for 199 s, 3983 downward-facing images at 20 Hz (textured ground and drone shadow), 187 m, 0–24 m above takeoff.
  - Limit for outdoor_1: **RTK fixed only 21.9% of the time**, with a fixed-ground-truth gap up to 98.6 s.
  - Global outdoor_1 image/gyro offset: −0.04 s (gy axis, r = −0.69). Per-window estimates are unstable because the downward-facing camera provides a weak yaw signal.
  - Inventory of 20 sequences (0.01–12.6 GB) in `data/processed/t_insane/listing.csv`.
- **PX4 Flight Review** (logs “CC-BY PX4,” https://review.px4.io/browse): 471,956 logs listed, 26 downloaded (2.9 GB), 25 with `sensor_baro`. One camera trigger and no captures; no images in the logs.
  - Exports in `data/processed/t_px4/<id>/` with a schema close to the common format.
  - RTK fixed logs: `53736001…` (94%, 2334 s, but IMU logged at about 4 Hz) and `036fb3a7…` (84%, 1393 s, IMU 200 Hz). Sent to Track B.
- **Videos linked to PX4 logs (new)**: the `video_url` field in `dbinfo.json` contains 114 distinct video URLs, sorted in `data/processed/t_px4_video/candidates.csv`.
  - Two EasyStar flights (fixed-wing, forward-facing FPV camera) became sequences: `px4video_d4cc6eb1` (820 s, 296 m barometric climb) and `px4video_aa0ae4df` (1262 s).
  - **MEASURED, independent cross-check on `d4cc6eb1`**: offset 0.02 s, r = −0.79. Across 100 s windows, from −0.10 to +0.10 s, with |r| between 0.75 and 0.89. Command: `.venv/bin/python experiments/t_zurich_sync_check.py data/processed/t_replay/px4video_d4cc6eb1 100`.
  - `aa0ae4df`: r = 0.43 with a second peak at 0.33, so ambiguous.
  - Limits: barometer logged at only 1 Hz; video license unknown.
- **Other datasets (PUBLISHED, sub-agent research, citations in manifest)**: MARS-LVIG, MUN-FRL, VPAIR, UAV-VisLoc, AerialVL, AnyVisLoc, UAVD4L, and AerialExtreMatch have nadir cameras but no listed barometer. NTU VIRAL, Blackbird, FusionPortable, and GND are unsuitable.
  - DJI photo sets `tuniu_tw_1/2` (Taiwan, RTK) are in the ODM index. Their `RelativeAltitude` is a **fused** altitude according to DJI documentation (https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-altitude.html). They would therefore serve Track A, not barometer evaluation.
  - Simplified and Traditional Chinese searches found no Taiwan dataset with a raw barometer within this search budget (8 queries). This is not proof that none exist.

### Q5. Simulation: proposed patch (not applied to the team branch)
The diff `data/processed/t_sim_rec/sim_patch.diff` (706 lines) is against a working copy, `data/raw/t_sim_work/sim`. The team's snapshot was not modified. Contents:
1. **Orientation leak fixed**: `sensor_noise.py` zeros the quaternion in addition to setting the covariance to −1. Checked with `ros2 topic echo`: orientation 0/0/0/0.
2. **GNSS cutoff**: new node `gnss_gate.py`. The bridge sends `/sim/gps_raw`, which is republished on `/gps/fix` while simulated time is less than `gnss_cut_s`.
3. **Optional stereo**: `stereo:=true` adds a 2nd camera offset downward by 0.30 m, using a temporary SDF variant.
   - At 60 m, expected disparity is 256 × 0.30 / 60 = **1.28 px**, below matching noise (INFERENCE). Stereo on the drone does not provide scale at that altitude; only a multi-view baseline can (consistent with Song et al. 2017).
4. **Recorder** `recorder.py`, which writes directly to the common format.
5. **Scenario** `t_scenario.py`: climb to 60 m, 120 s at 8 m/s, 90° turn, 60 s, with altitude and heading correction. The first open-loop attempt hit the ground at 156 s.

SIMULATED result: `data/processed/t_sim_rec/t_sim_terrain_cut60`, 255 s, 1.36 km, 467 MB, validated.
- Last GNSS measurement at t_s 53.27.
- Real-time factor 0.57–0.95 with 2 cameras, each at 512 px.
- Barometer–ground-truth difference: standard deviation 0.98 m, 1.45 m at end (10 Pa noise and the team's model drift).
- **Manually fixed defect**: `meta.gnss_cut_s` was 60 (simulated time), while the t_s origin is at 5.73 s. I changed it to 54.27. The patch must write the cutoff in the t_s reference frame.

**Comparison with Python replay** (`experiments/t_gen_wufeng_replay.py`): Python generates real Taiwan imagery deterministically in a few minutes, without Docker. Its limits: flat ground, no parallax, no flight dynamics. Gazebo adds flight dynamics, 3D terrain (trees), and a closed loop, but its terrain is procedural or draped and its real-time factor is below 1. No other simulator seems clearly useful tonight.

### Q6. Common format and loaders
- Format `taipeidrift-replay/1`, defined in `experiments/t_replay.py`:
  - Files: `meta.json`, `imu.csv`, `baro.csv`, `gnss.csv`, `images.csv`, `truth.csv` (evaluation only).
  - One clock; each sensor's provenance and the direction of barometric altitude are required.
  - `load(seq, cut_s)` removes GNSS after the cutoff; `load_truth` is separate.
  - Check: `validate`.
- Exporters and generators: `t_export_zurich.py`, `t_export_insane.py`, `t_px4_export.py`, `t_gen_wufeng_replay.py`, plus Gazebo's `recorder.py`.
- Nine sequences validate OK: Zurich, INSANE Mars1 and outdoor_1, 2 PX4 videos, 2 Wufeng, Gazebo.
- **SIMULATED, Wufeng checks**: without noise, the IMU integrated in inertial navigation has 0.05 m error at 60 s, confirming consistent coordinate frames. White barometer noise 0.67 m. Black pixels in images: 0.001%.

## Verification commands
```
.venv/bin/python experiments/t_replay.py validate data/processed/t_replay/* data/processed/t_sim_rec/t_sim_terrain_cut60
.venv/bin/python experiments/t_remote_zip.py list https://download.ifi.uzh.ch/rpg/AGZ_data/AGZ.zip
.venv/bin/python experiments/t_zurich_sync_check.py data/processed/t_replay/zurich_agz_1800_2400 20
.venv/bin/python experiments/t_export_zurich.py --t0 1800 --t1 2400 --images "data/raw/zurich_mav/AGZ_window/AGZ/MAV Images" --out data/processed/t_replay/zurich_agz_1800_2400
```

## Open questions (also in `questions.md`)
- License for YouTube videos linked to PX4 logs.
- Use of AGZ images in the jury video.
- Applying the simulator patch by its author.

## Next experiments
1. Track A: plug other matchers (XFeat + RANSAC) into `experiments/t_alto_heldout.py`, keeping the same frozen protocol and the same 8 Round 2 sections.
2. INSANE outdoor_1: raw barometer versus RTK, but only during fixed-RTK periods. Also test optical flow from the downward-facing camera over grass and the runway.
3. Fix the recorder (cutoff in t_s) and add Copernicus terrain to the Gazebo world.
4. Download an ODM `tuniu_tw` dataset to test Track A on real Taiwan drone photos against NLSC/OAM.
5. Make zoom calibration robust: more fixes during the GNSS phase, bounded zoom, and validation using consistency between successive fixes rather than the score.
   - Note: the diagnostic has already looked at all 8 sections. There is no longer any unseen ALTO data with ground truth; Round 1 Train is included in Round 2 and Test has no ground truth.
   - A clean test requires another nadir flight with ground truth: MUN-FRL (CC BY 4.0, RTK/PPK) or MARS-LVIG (CC BY-NC-SA, RTK).

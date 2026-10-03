# Overnight research, 2 to 3 October 2026

Branch `research/offline-nav-evidence`, started from `origin/main` 1286d5c and pushed on the morning of 3 October.

**Warning.** The ALTO scripts on this branch still search the reference images centred on the true path. Dustin found this leak (`alto-navigator`, findings 3.8). Their numbers have to be rerun with his one-map search before anyone quotes them.

Detailed reports in this folder:

- `map-localization.md`: track A, camera-to-map fixes;
- `sensor-fusion.md`: track B, height, speed and heading;
- `datasets-replay-sim.md` and `data-manifest.md`: track C, data, replay and simulator;
- `contesto-alessandro.md` (Italian) and `context-alessandro-en.md` (English): context for the VIO work;
- `context-dustin.md`: where this branch complements the ALTO navigator;
- `outside-reviews.md`: two outside reviews of the plan (Grok 4.7, GPT-6 Astra);
- `../../questions.md`: questions for the team.

Labels: **MEASURED** (run here on real data), **SIMULATED** (declared generator), **PUBLISHED** (cited source), **INFERENCE**.

## 1. Five points

1. **ALTO's 26–31 m do not hold on a flight we never tuned on.**
   - ALTO Round 2 Train: 37.4 km of real flight, protocol written before running, parameters frozen.
   - The team's chain gives a median of 94 m per section with a fix every 300 m (20–339 m depending on the section), against 31 m on Val. It reproduces Val in only 3 sections out of 8.
   - The Val numbers must be presented as **tuned on the test section**. [MEASURED, `datasets-replay-sim.md`, on the leaky reference images]
2. **The main cause is the image zoom (scale), calibrated from three fixes**, not the matching itself.
   - With a better zoom, the share of correct fixes rises from 21–43 % to 86–100 % in three sections.
   - When the match is wrong, the correlation score peaks at the edge of the zoom grid. [MEASURED, a diagnostic that uses truth for scoring only]
3. **An integrity check without a learned threshold works better than a score threshold.**
   - "Quad ≥ 3": four disjoint sub-templates must land in the same place.
   - On 300 real ALTO frames, ZNCC with yaw/scale search + quad ≥ 3 accepts 44 fixes: median 10.4 m, worst 17.7 m, 0 wrong, 0 of 300 negatives accepted.
   - Score thresholds do not transfer from one half of a map to the other. [MEASURED, track A]
4. **The learned matcher XFeat is no use on the real aerial images we tested**: 0/300 on ALTO, 1/60 on an unseen OrthoLoC site. It works on orthophoto crops (Wufeng), but collapses with motion blur and yaw of 30° or more. [MEASURED]
5. **Extra sensors.**
   - A real barometer holds relative height to 0.6 m (median) after 60 s and 1.8 m after 5 min.
   - The sun sensor reduces cross-track error, but final error on ALTO by only 6 %.
   - Off-the-shelf stereo is useful only below about 30–90 m above ground.
   - The biggest lever measured is the calibration of camera speed: 74 m at the end of the section if it were known over the whole path, against 608 m. [MEASURED and SIMULATED, track B]

## 2. Proposal for the team

From the safest to the most speculative.

| Rank | Element | Why | Evidence |
|---|---|---|---|
| 1 | **ZNCC fix with yaw and scale search, image rectified with the IMU attitude** | The only method that works on real images (ALTO). Without rectification, 0 successes on OrthoLoC at 21° oblique view | MEASURED (A, V3) |
| 2 | **Integrity from internal consistency (quad ≥ 3), no score threshold; agreement between dissimilar methods only as a complement** | ZNCC + yaw/scale + quad ≥ 3: 0 false on 2,395 negative groups at 18 sites (95 % upper bound 0.125 %). Score thresholds accept negatives outside the area they were tuned on. Not a universal zero: the UNION rule let one river negative through, and 3 negatives out of 3,540 pairs on the OrthoLoC site | MEASURED on real orthophotos with a simulated camera (A); real images (V3) |
| 3 | **Fix scale from height above ground (baro minus elevation model), with a wide zoom grid, always combined with quad ≥ 3** | Attacks cause no. 1 of the held-out failure. On Round 2 (exploratory, section 6): alone, it doubles wrong fixes; with quad ≥ 3, median of sections 70 m instead of 128 m and no wrong fix | MEASURED (C, V1); barometer SIMULATED |
| 4 | **Barometer for height, gyroscope for heading between fixes** | Real barometer: 0.30 m noise, 0.112 m/√s random walk; the IMU does not improve height drift | MEASURED (Zurich, INSANE, PX4 RTK logs) |
| 5 | **Declared drift budget when no fix is available** (sea, forest, night) | Without fixes: median 219 m per 4.6 km section on Round 2; no cue tested tonight replaces it | MEASURED |
| 6 | Slit sun sensor (option) | Cross-track error 196 → 31 m on ALTO, but −6 % on final error; needs sun below 70°, clear sky (40 % of the day in Taichung in October) and attitude within 0.5° | SIMULATED on a real path (B) |
| 7 | Stereo (option, low altitude only) | Baseline 0.30 m: 1.3 px of disparity at 60 m | SIMULATED (C, B) |

## 3. Tried and dropped

Every test had a kill criterion written before it ran.

| Idea | Result | File |
|---|---|---|
| Shadow compass from the image | Spread 23° (gradient) and 51° (silhouettes), criterion 10°: dropped | `experiments/u1_shadow_compass.py` |
| Baro minus elevation model as scale, without fixes | Final error 639 m against 608 m; median 372 against 472 m; even a perfect height leaves 426 m | `u2_baro_dem_scale.py` |
| Fixes on OpenStreetMap roads (chamfer) | After fixing a bug, the true position is not even a local minimum: dropped | `u5_osm_fix.py` |
| Fixes on the coastline (Sentinel-2) | No site passed; Taichung port promising (4 m) but 0/3 accepted; Changhua mudflats biased by the tide (5.4 m range) | `u3_coastline.py` |
| Speed from drag + wind on PX4 logs | On average worse than holding the last GNSS speed, on 3 flights; the identification is ill-posed | `u6_px4_drag.py` |
| Magnetic anomaly maps (EMAG2v3) | Median precision 9.6 km over the strait: dropped | `u7_magnav_kill.py` |
| Online calibration of scale and heading from fixes | Failed its pre-registered criterion: 162 m at 1,000 m spacing against 56 m for the team | `s_alto_online_calib.py` |
| Learned matcher (XFeat) on real images | 0/300 ALTO, 1/60 OrthoLoC | `r_alto_matchers.py`, `v3_ortholoc_heldout.py` |
| Altitude estimated from video instead of a barometer | Underestimates drift by 2.7–3 times: do not use it for evaluation | `sensor-fusion.md` |

## 4. Where each cue can work in Taiwan

Map: `data/processed/u8_taiwan_coverage/coverage_map.png`. The Copernicus GLO-30 elevation model and the ESA WorldCover land cover are real data. The rule "map fix likely" is an INFERENCE (textured ground, not forest or water); the 4 m height error is an assumption.

| Area | Map fix likely | Terrain informative (420 m footprint) | Neither |
|---|---|---|---|
| Main island | 21 % | 72 % | 9 % |
| 20 km coastal strip | 33 % | 55 % | 14 % |
| Taipei | 20 % | 81 % | 4 % |
| Taichung | 45 % | 50 % | 9 % |
| Kaohsiung | 47 % | 17 % | 37 % |

Forest covers 76 % of the island. Orthophoto fixes are unlikely there, and terrain matching becomes the main cue. Terrain matching was not tested on a real flight tonight.

## 5. Data on disk (ignored by git)

Details and licences: `data-manifest.md`.

- ALTO Val (1.86 GB) and Round 2 Train (11.3 GB), fetched with per-file Dropbox links.
- Zurich Urban MAV: full logs, the sample, and a 600 s window of images (forward camera, useless for the map).
- INSANE (downward camera, raw barometer, RTK), 26 PX4 logs including 2 with RTK, MUN-FRL (sample), OrthoLoC (unseen site, 60 frames).
- Wufeng orthophotos 2018/2020, NLSC and OpenAerialMap samples, Sentinel-2, OSM Taiwan, EMAG2v3, Copernicus, WorldCover.
- XFeat (code and weights, Apache-2.0). Docker image of the simulator (4.8 GB).

## 6. Last tests of the night

### Track A, final (`map-localization.md`)

- **Wider benchmark** [MEASURED on real orthophotos, SIMULATED camera]: 18 sites, 20 conditions.
  - Out of 7,850 positives, ZNCC + yaw/scale + quad ≥ 3 accepts 1,133 correct ones, with no false fix on 2,395 negative groups (95 % upper bound 0.125 %).
  - The UNION rule accepts 1,878. But a confirmation run found one false fix on a river negative: **UNION is not a guarantee**; recheck it over water.
- **Robustness**: haze, yaw up to 180° and a rectified 20° tilt remain usable. Motion blur of 21 px and a 25 % scale error make acceptance collapse.
- **Closed loop** [SIMULATED, 19 sites, 4 seeds]:
  - With 60 s without fixes, the drift-gated variant accepts 868 of 2,584 fixes, with no false one. Median error drops from 104.8 m (dead reckoning only) to 17.6 m.
  - Without the gate, 1,043 of the 2,557 accepted fixes are more than 25 m off.
  - With 240 s without fixes, the adaptive variant ends at 18.9 m against 220.7 m for dead reckoning.
- **Lost mode, no prior**: on a 23.3 km² mosaic, 40.6 % of the best candidates are within 10 m (aligned view), and 0/143 negatives accepted after the gate.
- These numbers come from cropped orthophotos. On real images, the only numbers are ALTO (44/300 accepted) and OrthoLoC (V3 below).

### Unseen site: OrthoLoC `test_outPlace/L08` (V3)

- 60 real drone frames at about 100 m above ground, 21–23° oblique, with orthophoto, surface model and pose per frame (licence CC BY-NC-SA 4.0).
- No tuning on this site. Rectification with the attitude perturbed by 1° per axis and the height by ±3 % (SIMULATED). Position prior within ±40 m.
- Script: `experiments/v3_ortholoc_heldout.py`.

| Method | Raw correct within 10 m | Accepted | Accepted but wrong | Negatives accepted / 3,540 |
|---|---|---|---|---|
| ZNCC | 26/60 | 3 | 0 | 1 |
| ZNCC + yaw/scale | 25/60 | 5 | 0 | 3 |
| XFeat + RANSAC | 1/60 | 1 | 0 | 0 |
| UNION rule | 5/60 | 5 | 0 | 3 |
| ZNCC without rectification | 3/60 | 0 | 0 | 0 |

Reading: without rectification by attitude nothing works. With it, the match is right 4 times in 10, but the integrity check keeps only one in ten.

### Integrity and zoom on Round 2 (V1, exploratory)

- **Not held-out**: the diagnostic had already looked at the 8 sections, and no ALTO flight with truth remains unseen.
- Barometer SIMULATED (ALTO altitude + real Zurich residuals), Copernicus elevation model. `experiments/v1_round2_integrity.py`, 84 min.

| Fix every | Variant | Median of section medians | Median of end errors | Fixes used / rejected / wrong > 50 m |
|---|---|---|---|---|
| 300 m | frozen chain (sized search + 0.33 threshold) | 128 m | 833 m | 35 / 89 / 6 |
| 300 m | + quad ≥ 3 | 126 m | 839 m | 13 / 111 / 0 |
| 300 m | + baro − DEM zoom | 406 m | 975 m | 30 / 56 / 12 |
| 300 m | + quad ≥ 3 + baro − DEM zoom | **70 m** | **296 m** | 10 / 108 / 0 |
| 1,000 m | frozen chain | 160 m | 919 m | 13 / 21 / 2 |
| 1,000 m | + quad ≥ 3 + baro − DEM zoom | 118 m | 479 m | 3 / 28 / 0 |

Reading:
- the quad ≥ 3 check removes the wrong fixes; the zoom from the barometer helps only together with it;
- the price: only one fix attempt in ten is accepted;
- the next gain will come from a matcher that accepts more without false fixes, validated on a truly new flight (MUN-FRL with an orthophoto, MARS-LVIG, or a team flight).

## 7. Limits

- Not one real flight with a downward camera, a raw barometer, an IMU and an orthophoto of the same place. Each piece of evidence covers part of the chain.
- No flight over Taiwan with a raw barometer found (bounded search, including YouTube and Chinese queries). DJI videos give an altitude fused by the manufacturer.
- The orthophoto tests crop images from above: no perspective and no real camera, except ALTO and OrthoLoC.
- Nothing has run on a small onboard board; times were measured on one Mac core.

## 8. To pick up the work

```bash
cd TaipeiDrift && uv sync --extra research
.venv/bin/python experiments/t_alto_heldout.py        # ALTO Round 2 held-out (see its --help)
.venv/bin/python experiments/r_alto_matchers.py       # matchers on 300 real ALTO frames
.venv/bin/python experiments/p_zurich_baro.py         # real barometer against photogrammetry
.venv/bin/python experiments/u8_taiwan_coverage.py    # coverage map of Taiwan
```

Simulator:
- `data/processed/sim_smoke/smoke_report.txt`: sensor check passed, real-time factor 0.99.
- Proposed patch `docs/research/sim_patch.diff`: removes the true attitude leaked on `/imu/data`, adds the GNSS cut, a recorder and a second camera. It is not applied to Dan's branch.

## 9. How the night was organised

- Opus orchestrated and reasoned. Three Opus track leads delegated search, downloads and code to GPT-6 Luna agents.
- A Haiku agent watched the Claude quota.
- Featherless DeepSeek was tried first: its plan's limit of 32,768 tokens per request made it unusable for agents with tools. It stays as a fallback.
- Configuration specific to this folder: `../../.omp/config.yml` and `../../.omp/agents/`, outside the git repository.

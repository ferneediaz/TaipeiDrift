# Context for Alessandro: what's in this branch and how to use it for VIO

English version of contesto-alessandro.md (Italian original).

Branch `research/offline-nav-evidence`, written overnight between 2 and 3 October 2026 from `main` 1286d5c. This file is in Italian; the rest of the overnight research in `docs/research/` is in English. The sections you need are summarised here, with the numbers.

Labels used throughout:
- **MEASURED**: run on real data;
- **SIMULATED**: generated with stated assumptions;
- **PUBLISHED**: a claim from a paper, with a link;
- **INFERENCE**: our unverified reasoning.

## 0. Where to start (10 minutes)

1. `git fetch && git switch research/offline-nav-evidence`
2. `uv sync --extra research`. The optional `research` group adds torch CPU, pyulog, rosbags, pyshp and tqdm; it is not needed for the barometer.
3. Read sections 3 and 4 below: the real barometer model and real data for VIO.
4. Work proposal: section 7.

The downloaded data and results (`data/raw/`, `data/processed/`) are **not in the repository**: they are on a separate laptop. Section 4 explains how to download or regenerate them. To get the files ready now, ask Ilhan for a copy of `data/processed/t_replay/insane_mars_1` (2 MB, but the images it points to take 4.5 GB).

## 1. The project in brief

- **Challenge 2**: navigate after GNSS loss. GNSS is available at takeoff, then disappears; position is estimated afterward. Software only, for a low-cost drone (about 500 USD) with a downward-facing camera, IMU and barometer. No LiDAR. Maps and models on board, offline.
- **The judges want**:
  - an estimated navigation baseline (dead reckoning);
  - at least one sensor correction or fusion;
  - plots of the estimated trajectory against the reference, and of error over time;
  - limits when sensors fail or noise grows.
- **Scoring criteria**: reduced position error, technical validity, noise tolerance, compute and integration requirements, and deployment feasibility. The user, deployment and scalability also count.
- **Final video is prerecorded.** Recorded or simulated flights can be replayed, as long as they are labelled.

## 2. Where everyone else is (fetch on 3 October, 10)

| Who | Branch | Status |
|---|---|---|
| You | `mid-air-vio` | ESKF VIO on Mid-Air: 13–33 m after 83 s without GNSS on 3 flights not used for calibration, versus 116–441 m with IMU + barometer and 338–912 m with IMU alone. Barometer simulated. |
| Dustin | `alto-navigator` | Camera navigator on ALTO as shared code (`baseline/`, 91 tests), with stated uncertainty and tracking / degraded / lost states. He found an error in the ALTO protocol (section 6). Agreement across 3 nearby frames (14 m) was tested and discarded: nearby frames make mistakes in the same place. His test on unseen data is blocked because he is missing the ALTO Train positions; this branch has them (section 5). |
| Felix | `main`, folder `TRN/` | Plan for a terrain-navigation simulator with a laser rangefinder, using Taiwan Ministry of the Interior 20 m elevation models. He is waiting for approval before writing code. |
| This branch | `research/offline-nav-evidence` | Overnight research: data, experiments, discarded ideas, and a proposal for the judges. |

Open team decision: which IMU baseline goes on `main`, `mid-air-baseline-fix` or your `mid-air-vio`. They use the same gyroscope rule.

## 3. The real barometer: the model to put in your ESKF

### 3.1 Where it comes from (MEASURED)

| Source | Sensor | Reference | Script |
|---|---|---|---|
| Zurich Urban MAV, 45 min, tethered flight (Fotokite) | Pixhawk barometer, 10 Hz | Camera positions from Pix4D photogrammetry, 1 Hz | `experiments/p_zurich_baro.py`, `experiments/s_zurich_vertical.py` |
| INSANE | MS5611 in PX4, about 18–20 Hz | Fixed RTK | `experiments/s_insane_baro.py` |
| 2 public PX4 logs with RTK | PX4 barometer | Fixed RTK | `experiments/s_px4_rtk_baro.py` |

The Zurich barometer's “altitude” column is exactly the standard-atmosphere formula applied to pressure, accurate to 3 mm. So it is a raw barometer reading, not fused altitude.

### 3.2 The numbers

Model fitted on Zurich (`data/processed/zurich_vertical/baro_error_model.csv`, `experiments/s_zurich_vertical.py`), confirmed by INSANE between 10 and 120 s:

- white noise **0.30 m**;
- random walk **0.112 m/√s**;
- linear drift per flight **0.0024 m/s**.

Relative-altitude error after a GNSS cut, Zurich, overlapping windows from a single flight:

| After cut | Median | p95 |
|---|---|---|
| 10 s | 0.41 m | 1.42 m |
| 60 s | 0.58 m | 2.12 m |
| 2 min | 0.82 m | 2.97 m |
| 5 min | 1.75 m | 4.04 m |
| 10 min | 2.49 m | 6.11 m |
| 20 min | 4.50 m | 8.83 m |

For comparison, GNSS altitude on the same flight has a median of 1.8–4.6 m and a p95 of 6.9–22 m.

Notes:
- the two PX4 logs with RTK drift about **2 times less** at 600 s (median about 1 m versus 2.45 m): the model is pessimistic, so it is conservative;
- measured barometer scale error is between 3 and 7%, in both directions;
- speed effect: about 0.7 m at 10 m/s, but on **one log only**. It is not validated beyond that.

### 3.3 How to use it

Example generator, simulation side only. The ESKF sees only `baro`, never `alt_true`:

```python
import numpy as np

def simulated_baro(t_s, alt_true_m, rng, white=0.30, rw=0.112, ramp_sigma=0.0024):
    """SIMULATED barometer calibrated on real logs (Zurich, INSANE, PX4 RTK)."""
    dt = np.diff(t_s, prepend=t_s[0])
    walk = np.cumsum(rng.normal(0.0, rw * np.sqrt(np.maximum(dt, 0.0))))
    ramp = rng.normal(0.0, ramp_sigma) * (t_s - t_s[0])
    return alt_true_m + rng.normal(0.0, white, size=t_s.shape) + walk + ramp
```

Protocol that will stand up to expert judges:
1. at least **20 seeds** per flight; report the median and p95, not a single run;
2. three noise levels, **×0.5, ×1 and ×2**, for noise tolerance, which is a scoring criterion;
3. label every plot “SIMULATED barometer, calibrated on real logs”.

### 3.4 Two cautions

- **The IMU does not reduce altitude drift.** On INSANE, with a real IMU at 196 Hz, IMU + barometer fusion smooths noise by about 1.6 times, but drift at 60 s stays the same (`experiments/s_insane_vertical.py`, MEASURED). On Zurich the IMU adds nothing: a 10 Hz accelerometer with no pre-integration.
- **Never replace the barometer with altitude derived from video.** In simulation it underestimates drift by **2.7–3 times** (`docs/research/sensor-fusion.md`).

## 4. Real data to test VIO beyond Mid-Air

Mid-Air is synthetic, and the judges will notice. No public dataset found has everything together (downward-facing camera, raw barometer, IMU, ground truth and orthophoto at 100 m or higher); these are the closest matches for your ESKF.

### 4.1 Common format `taipeidrift-replay/1`

Defined and validated by `experiments/t_replay.py`. One folder per sequence:

```
meta.json    provenance per sensor, axes, origin, license, label MEASURED/SIMULATED
imu.csv      t_s, gx, gy, gz [rad/s], ax, ay, az [m/s^2]   (axes listed in meta.sensors.imu.frame)
baro.csv     t_s, pressure_pa, temperature_c, alt_isa_m   (alt_isa_m = ISA with p0 = 101325 Pa)
gnss.csv     t_s, lat_deg, lon_deg, alt_m, fix_type, hacc_m, vacc_m, ve_mps, vn_mps, vu_mps, nsat
images.csv   t_s, cam, path   (path relative to the sequence folder)
truth.csv    t_s, e_m, n_m, u_m, lat_deg, lon_deg, alt_m, qw, qx, qy, qz   EVALUATION ONLY
```

- One clock, `t_s` in seconds from 0.
- `t_replay.load(seq, cut_s=...)` removes every GNSS row with `t_s >= cut_s` and **does not return ground truth**.
- Ground truth is read only with `load_truth(seq)`, for use exclusively in evaluation code.
- Validation: `.venv/bin/python experiments/t_replay.py validate data/processed/t_replay/<sequenza>`.

### 4.2 The sequences

| Sequence | Contents | Limits | For VIO |
|---|---|---|---|
| **INSANE `mars_1`** (Negev Desert) | Downward-facing 2056×1542 camera at 15 Hz with K calibration, PX4 IMU at 196 Hz, **raw barometer** at about 18 Hz, non-RTK GNSS at about 5 Hz, fixed RTK ground truth | Low, short flight: about 100 s, altitude around 5 m | **First candidate**: everything on the same clock, real barometer |
| **INSANE `outdoor_1`** (Klagenfurt airfield) | Same sensors | 260 s; fixed RTK only 22% of the time; images 12.6 GB | Second test |
| **MUN-FRL**, `lighthouse` sample | Nadir camera 1440×1080 at 20 Hz, IMU at 400 Hz, RTK at 5 Hz, separate PPK, ROS bag of 3.8 GB | No barometer; 181 s | VIO without a barometer |
| Zurich `agz_1800_2400` | Filtered 50 Hz IMU + raw 10 Hz, barometer, GNSS, photogrammetric ground truth | **Forward-facing** camera at road level; image timestamps are wrong by −1.9 to +0.3 s | Barometer only |
| `wufeng_sim_base` / `wufeng_sim_low` | Camera rendered from Taiwan's real 2020 orthophoto, 2018 map; IMU, barometer and GNSS SIMULATED | Flat ground, no parallax | Taiwan demonstration, label SIMULATED |

### 4.3 How to regenerate INSANE `mars_1`

License: BSD-2 with additional conditions, no selling, cite the authors; [dataset page](https://cns-data.aau.at/insane-dataset/).

```bash
mkdir -p data/raw/insane/mars_1_images && cd data/raw/insane
curl -LO https://cns-data.aau.at/insane-dataset/mars_1_sensors.zip
cd mars_1_images
curl -LO https://cns-data.aau.at/insane-dataset/mars_1_nav_cam.zip
curl -LO https://cns-data.aau.at/insane-dataset/insane_sensor_calib_preprocessed.zip
unzip -q mars_1_nav_cam.zip && unzip -q insane_sensor_calib_preprocessed.zip
cd ../../../.. && .venv/bin/python experiments/t_export_insane.py --sequence mars_1
```

Check the expected paths in `SEQUENCES` at the start of `experiments/t_export_insane.py`.

Cautions from INSANE's `meta.json`:
- the dataset README declares the IMU ENU and it is exported without rotation: **check the axes** before using it, as you did for Mid-Air, by comparing gyro rates with the ground-truth attitude;
- the camera timestamp file header says `t[ns]`, but the values are in seconds;
- RTK is shifted by 1.606 s (`t_mag_gps`) to align it with the common clock: the exporter already does this.

### 4.4 MUN-FRL

- Sample already downloaded: `data/raw/mun_frl/lighthouse_francis_sample.bag` and `flight_dataset5_ppk.pos`. License CC BY 4.0.
- Inventory: `.venv/bin/python experiments/v2_heldout_inventory.py`.
- The bag does not contain the PPK/INS pose topics: the separate PPK file gives position only. Its alignment with the bag's 181 s has not been established.

## 5. Other overnight results that affect VIO

1. **Downward optical flow does not observe direction.** You had already seen this; we confirm it on ALTO, a real helicopter flight.
   - With perfect direction, final error drops by only 6%; with perfect height above ground, by 30%; with both, by 39% (SIMULATED on the real route).
   - A solar sensor (a slit on a linear sensor, Tsinghua literature, PUBLISHED: [Wei et al. 2011](https://pmc.ncbi.nlm.nih.gov/articles/PMC3231287/)) would reduce cross-track error from 196 to 31 m on ALTO, but reduce final error by only 6%. It needs the sun below 70° and clear skies: about 40% of the day in Taichung in October.
2. **The biggest lever on ALTO is camera-speed calibration.** With the flow → ground conversion known across the whole route (a diagnostic that uses ground truth), final error drops from 608 to 74 m. Estimating it in flight from pairs of successive fixes failed: a fix at ±15 m gives the scale only to about ±7%.
3. **Commercial stereo**: useful only below 30–90 m above ground. With a 0.30 m baseline at 60 m, disparity is 1.3 px.
4. **The filter gets overconfident when it is wrong.** You see it with the downward-facing camera alone, and Dustin sees it in the ALTO navigator: when the fix is wrong, error exceeds 3 σ in 76% of frames. Your `vio/evaluation/consistency.py` could become a shared check for both of you.
5. **Ideas tested and discarded**, with stopping criteria written in advance: compass from shadows, matching on OpenStreetMap roads, magnetic field (EMAG2: about 10 km accuracy), speed from aerodynamic drag and wind on PX4 logs. Do not repeat them without a new idea.

## 6. Rules for not sounding like “bullshit” in front of the judges

1. **Ground truth never enters the estimator after the GNSS cut.** A simulated sensor is generated from ground truth only inside a labelled generator. Your `vio/estimation/oracle.py` is already labelled correctly: keep it that way.
2. **Every number carries a label**: MEASURED, SIMULATED or ORACLE.
3. **Use unseen data for final numbers.** Dustin and this overnight work found that ALTO's 26–31 m results were tuned on the same segment. On ALTO Train (37.4 km, preregistered protocol, frozen parameters), the median per segment is 94 m instead of 31, and the result reproduces on only 3 of 8 segments. You already held out the test flights: good.
4. **Error found by Dustin**: ALTO's reference images (`offset_0_None`) were centred on the true trajectory, so every fix fell within about 48 m of the true path. He fixed it by using one map and searching a circle around the estimate. This branch's ALTO scripts (`t_alto_heldout.py`, `r_alto_matchers.py`) still use those images: their numbers need to be rerun.
5. **Mid-Air**: the gyroscope is expressed in world axes, contrary to the documentation. You already know this; it applies to anyone who reuses your code.

## 7. Proposed work, in order

| # | Task | Success criterion | Estimated time |
|---|---|---|---|
| 1 | Replace your ESKF's simulated barometer with the section 3 model, 20 seeds, noise ×0.5 / ×1 / ×2 | Table for the 4 flights with median and p95 at each noise level, labelled | 1–2 h |
| 2 | Run the ESKF on INSANE `mars_1`: real barometer, real IMU, downward-facing camera, GNSS cut after 20 s | Plots of trajectory against RTK and error over time, IMU alone versus IMU + barometer + camera | 3–4 h (including axes and clocks) |
| 3 | Shared consistency check (NEES, fraction of frames within 3 σ) to apply to Dustin's ALTO navigator too | One function and one plot used by both of you | 1–2 h |
| 4 | Decide with Dustin which baseline goes on `main` | A merge and one sentence in the README | 30 min |
| 5 | Optional: MUN-FRL without a barometer, to see VIO on a 400 Hz IMU and real nadir camera | Same plots | 3 h |

External consultants we contacted, Grok 4.7 and GPT-6 Astra (`docs/research/outside-reviews.md`), recommend **freezing features within about 12 hours** and spending the rest on figures, video and verification. For you, that means prioritising tasks 1 and 2.

## 8. File map for this branch

- `docs/research/overnight-synthesis.md`: overnight synthesis and ranked proposal.
- `docs/research/sensor-fusion.md`: barometer, IMU, stereo, solar sensor, drift budget.
- `docs/research/datasets-replay-sim.md` and `docs/research/data-manifest.md`: all datasets assessed, with licenses, sizes and status; replay format; proposed patch for the Gazebo simulator.
- `docs/research/map-localization.md`: camera → map matching.
- `docs/research/outside-reviews.md`: views from Grok 4.7 and GPT-6 Astra.
- `questions.md`: open questions for the team.
- `experiments/`: scripts with prefixes `n_` through `v`. For you:
  - `n_sensor_fusion.py`: vertical simulation and direction;
  - `p_zurich_baro.py`, `s_zurich_vertical.py`, `s_insane_baro.py`, `s_insane_vertical.py`, `s_px4_rtk_baro.py`: real barometer;
  - `t_replay.py`, `t_export_insane.py`, `t_export_zurich.py`, `t_gen_wufeng_replay.py`: replay format;
  - `v2_heldout_inventory.py`: MUN-FRL and OrthoLoC.
- Gazebo simulator (Dan's `simulations` branch): sensors run in real time on a Mac, but the `/imu/data` topic published the **ground-truth attitude**. Proposed fix in `docs/research/sim_patch.diff` (GNSS cut, recorder, second camera, attitude leak), described in `datasets-replay-sim.md`. It has not been applied to Dan's branch.

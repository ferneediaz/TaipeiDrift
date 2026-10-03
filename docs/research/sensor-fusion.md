# Track B: altitude, speed, and heading (overnight report)

Labels: **MEASURED** (real data, run here), **SIMULATED** (declared generator), **PUBLISHED** (source with link), **INFERENCE**.
All commands run from the repository root with `.venv/bin/python`.

## Ranked recommendation (read first)

Finding that drives everything else (MEASURED on the ALTO camera, simulated oracles and sensors, section 7): of ALTO's 608 m final error, perfect ground height removes 30%, perfect heading removes 6%, and both remove 39%. **367 m remain along the route even with perfect height and heading**: camera steps are 10% too short because the flow → ground calibration is learned from only the first 300 m. Calibrated over the entire route (diagnostic, ground truth used), the error drops to 74 m. The largest source of error is therefore camera speed calibration, not a missing sensor.

1. **Barometer + elevation model (DEM), with map fixes**: $0, 0 W. On ALTO, with a fix every 300 m, it reduces failed fixes (error > 60 m at arrival) from 32% to 25% (oracle: 19%). Without fixes it does nothing (639 m vs. 608 m): the DEM is read at a position that is already wrong. It does nothing on flat terrain (Wufeng-like), either. MEASURED (camera) + SIMULATED (barometer calibrated to real barometers, fixes).
2. **Onboard gyroscope for heading**: $0, 0 W. In simulation, cross-track drift is 161 → 74 m over 4.6 km if its bias is estimated before the cutoff to within 0.005 °/s (assumption). Not testable on ALTO (no public IMU, unknown time base). SIMULATED.
3. **Solar sensor**: ≈ $150 in open hardware (Foresail), 0.01 W. On ALTO, a sensor accurate to ≈ 1° removes 84% of cross-track drift (196 → 31 m), but only 6% of final error while camera speed is uncorrected. It requires an attitude reference accurate to 0.5° (at 2°, it performs worse than the gyro in simulation), the sun below ≈ 70° elevation (unusable from May to August around noon), and a clear sky (≈ 40% of the day in Taichung in October, 12% in Taipei in winter). MEASURED (camera) + SIMULATED + PUBLISHED (CWA).
4. **Commercial stereo**: $230–780, 2.5–5.5 W. Useful only below 27–92 m above ground (5% height error). An OAK-D LR performs worse than barometer + DEM at 120 m. Only a wide baseline (0.3–1 m, custom-built) covers 100–300 m. SIMULATED, specs PUBLISHED.
5. **The IMU does not improve long-term height**: with a real IMU at 196 Hz, it smooths the barometer without delay (noise ÷ 1.6) but does not change drift at 60 s. MEASURED (INSANE).

Next action (≈ 2 min): run `python experiments/s_alto_heading.py` and read the `true_agl heading_oracle` row: this is the error floor that only fixes or better camera speed can beat.

---

## 1. Review of `experiments/n_sensor_fusion.py` (SIMULATED)

Command: `python experiments/n_sensor_fusion.py --quick` (3 s), then `python experiments/n_sensor_fusion.py` (17 s, 20 flights per condition, 600 s after the cutoff). Outputs: `data/processed/sensor_fusion/`. All 22 of 22 invariants pass.

Ground-truth leakage: none found. The estimators read only the barometer, accelerometer, disparity, GNSS before the cutoff, and noisy AHRS roll/pitch; the causality test (changing post-cutoff data does not change pre-cutoff results) passes. The DEM appears only in a labeled ablation, read at a simulated horizontal position with error.

Physics checked: disparity = f·B·cos(tilt)/height along the axis; filter Jacobian is correct; NED→body and body→horizontal rotations are correct; NOAA ephemerides; null space (z+c, b−c, h+c) of barometer+stereo+IMU verified (rank 4 of 5).

Fixed bugs:

| Bug | Effect before fix | Fix |
|---|---|---|
| Initialization: terrain uncorrelated with altitude | at the first GNSS update, estimated ground height became negative; stereo was never accepted again (0 frames accepted); errors of 50 to 240 m everywhere | initial covariance h = z − stereo height (cov(z,h) = var z) |
| Terrain random-walk noise too low (slope 0.05) | over hilly terrain, the validation gate rejected every frame: ground height was wrong by 26 m (RMSE), versus 4.4 m for stereo alone | slope 0.2 (≈ RMS slope 0.165 of declared terrain) |
| No recovery after gate lockout | after texture loss, a flight stayed 19 m wrong to the end | reset terrain after 10 consistently rejected frames (median difference ≤ 10%) |
| Solar heading: AHRS tilt errors treated as white noise at 10 Hz | filter too confident (6 to 33% of samples within 2σ), diverged with the sun at 150° elevation | Gauss-Markov solar error state (60 s): 87 to 99% within 2σ; reject measurements with σ > 10° |
| Nominal barometer model under-dispersed | p95 at 600 s: 2.7 m simulated versus 6.0 m measured | model calibrated to Zurich (section 2) |

Results (SIMULATED, after fixes, p95 ground-height error after cutoff, nominal barometer condition):

| Scenario | barometer only | IMU + barometer | stereo only | fusion (terrain state) | fusion + DEM (ablation) |
|---|---|---|---|---|---|
| flat, 40 m, good texture | 5.2 m | 5.2 m | 2.0 m | 2.2 m | 2.2 m |
| hilly, 40 m, good texture | 90.6 m | 90.5 m | 15.1 m | 3.4 m | 3.4 m |
| hilly, 40 m, texture loss | 90.6 m | 90.5 m | 743.6 m | 17.6 m | 10.3 m |
| flat, 300 m | 43.7 m | 43.6 m | 53.8 m | 49.7 m | 16.6 m |
| hilly, 300 m | 95.1 m | 95.2 m | 64.3 m | 58.1 m | 35.2 m |

What this means:
- Stereo (B = 0.20 m, f = 1,000 px) gives ground height at 40 m, not at 300 m (0.67 px disparity).
- Stereo never corrects barometer altitude drift: with strong drift, final altitude error is 12.6 m for a true drift of 12.6 m (H4). Only a DEM read at the correct position reduces it (1.7 m).
- Without GNSS, absolute altitude remains unknown (0.99999 correlation with barometer offset), but ground height is just as accurate (1.69 m RMSE in both cases).
- A disparity offset of 0.05 px (calibration) causes 22 m of height bias at 300 m; the filter does not model it. Declared limitation.
- Preregistered hypotheses: H1 to H6 confirmed, H7 rejected (99.5% coverage on flat terrain: filter is slightly too conservative). Y1 to Y4 confirmed; Y2 is trivial: at the zenith the filter rejects the sun and falls back to the gyro.

Heading (SIMULATED, Taipei, p95 over 600 s, gyro bias 0.001 rad/s): gyro only 42.1°; gyro + October noon sun, 0.1° sensor/AHRS 0.5°: 1.70°; 1° sensor/AHRS 2°, clouds: 6.04°; June 21 at noon: sun rejected; December 21 at 15:30 (elevation 16–18°): outside the field of view of a ±60° sensor.

## 2. Zurich barometer data and generator calibration (MEASURED)

Command: `python experiments/s_zurich_vertical.py` (4 s). Outputs: `data/processed/zurich_vertical/`.

Structure function of (barometer − photogrammetry), all pairs on the 1 Hz grid, 45 min, one flight:

| Horizon | 1 s | 10 s | 60 s | 300 s | 600 s | 1 200 s |
|---|---|---|---|---|---|---|
| median | 0.22 m | 0.38 m | 0.55 m | 1.53 m | 2.45 m | 4.45 m |
| p95 | 0.76 m | 1.46 m | 1.96 m | 3.98 m | 6.01 m | 8.83 m |

Fitted model (white noise + random walk + per-flight ramp): **0.30 m; 0.112 m/√s; 0.0024 m/s**. The calibrated generator reproduces the measured p95 at 60/300/600/1,200 s: 1.91/4.11/6.17/9.63 m versus 1.96/3.98/6.01/8.83 m. The old nominal generator gave 1.14/1.67/2.71/4.89 m (too optimistic beyond 60 s); the old “strong” one gave 2.2/7.9/15.2/29.6 m. Both are replaced in `n_sensor_fusion.py` (nominal = Zurich; strong = 0.15 m/√s random walk + 0.015–0.025 m/s ramp).

Other measurements:
- Barometer white noise at 10 Hz: 0.16 m. Photogrammetry reference noise (second-difference proxy): 0.03 m.
- **Scale error**: barometer altitude changes are 7.2% larger than photogrammetry; GNSS agrees with photogrammetry to within 0.3%. The standard atmosphere explains only ≈ 1.5% (INFERENCE). Removing this scale reduces drift at 600 s only from 3.15 to 2.92 m RMS: scale is not the main source of drift.
- No visible dependence on speed (correlation −0.01 with V²), but the flight was at 0.8 m/s: the effect of dynamic pressure (≈ 138 Pa, or ≈ 11 m at 15 m/s if fully seen by the static port) was **not tested**. INFERENCE.

Remaining gap (why this calibration is not enough): one flight, one day, a tethered drone carried on foot, altitude 460 to 489 m, 6 to 10 °C, reference with unknown low-frequency error (so the model is rather pessimistic).

General findings (PUBLISHED):
- Morales et al. 2022 (Matrice 600, 11 flights, GPS/pressure altitude compared with RTK): mean drift of 0.6 m per 10 min, up to 1.2 m ([AMT](https://amt.copernicus.org/articles/15/2177/2022/)). Zurich gives a 2.45 m median at 600 s: our model is 2 to 4 times more pessimistic, consistent with an imperfect reference.
- Wu et al. 2026 (thermal VTOL, 100/250/500 m, 0–4 m/s gusts): baseline barometer altitude compared with RTK, RMSE 4.05/1.82/4.76 m ([Sensors](https://pmc.ncbi.nlm.nih.gov/articles/PMC12987359/)).
- Datasets found with raw barometer and independent RTK: INSANE (MS5611 20 Hz, dual RTK, 25–40 m), CTU-MRS MAS, and Cooperative UAV (Pixhawk, Emlid RTK). Nothing usable in NTU VIRAL, MARS-LVIG, UrbanNav, GVINS, EuRoC, UZH-FPV, or Blackbird (no published barometer). INSANE is tested in section 8.

## 3. Real vertical replay in Zurich: what does the IMU add? (MEASURED)

Same command. GNSS until the cutoff, then barometer (± raw 10 Hz accelerometer rotated by the PX4 quaternion), scored against photogrammetry; 86 cutoffs every 30 s.

| Horizon | barometer only | 1 s filtered barometer | IMU + barometer (EKF) | IMU only after cutoff |
|---|---|---|---|---|
| 10 s (median) | 0.43 m | 0.33 m | 0.41 m | 2.8 m |
| 60 s | 0.58 m | 0.51 m | 0.51 m | 69 m |
| 300 s | 1.76 m | 1.76 m | 1.68 m | 1,517 m |
| 600 s (p95) | 6.18 m | 5.75 m | 6.33 m | 26,175 m |

High-frequency error (60 s after cutoff, mean removed): barometer 0.44 m, filtered barometer 0.38 m, IMU + barometer 0.44 m.

Zurich conclusion: **in these data, the IMU adds nothing to height estimation**; a simple low-pass filter does better. Measured reason: the raw accelerometer is sampled at 10 Hz without pre-integration, so vibrations alias (vertical standard deviation 0.62 m/s²). Declared possible leakage: the PX4 quaternion may use GNSS to compensate for accelerations (a second-order effect on the vertical).

Counter-test with a real IMU (MEASURED, INSANE, PX4 IMU at 196 Hz averaged to 10 Hz, tilt from a 2 s gyro + accelerometer complementary filter, no ground truth; fixed 1 Hz RTK before the cutoff, full-rate RTK for scoring): `python experiments/s_insane_vertical.py` (3 s; a child agent's script, rerun here). Median altitude-change error:

| Sequence (cutoffs) | horizon | raw barometer | 1 s filtered barometer | IMU + barometer | IMU only |
|---|---|---|---|---|---|
| mars_2 (25) | 10 s | 0.58 m | 0.34 m | 0.33 m | 0.77 m |
| mars_2 (21 / 15) | 30 s / 60 s | 0.82 / 0.58 m | 0.80 / 0.97 m | 0.62 / 0.55 m | 3.9 / 9.9 m |
| mars_1 (12 / 8) | 10 s / 30 s | 0.26 / 0.36 m | 0.22 / 0.40 m | 0.21 / 0.18 m | 0.95 / 4.2 m |

High-frequency noise (10 s after cutoff, mean removed): mars_2 0.44 (raw) / 0.31 (filtered) / 0.27 m (IMU + barometer).

Conclusion: **with a correctly sampled IMU, the IMU smooths the barometer without delay** (noise divided by ≈ 1.6, and better than the low-pass filter during climbs at 30–60 s), **but does not change drift** (at 60 s, 0.55 m versus 0.58 m for the raw barometer). Alone, it diverges: 4 m at 30 s, 10 m at 60 s. Small sample (2 short flights, 12 to 25 overlapping cutoffs).

## 4. Stereo height: covered altitude range (SIMULATED, specs PUBLISHED)

Command: `python experiments/s_drift_budget.py` (6 s), part S. Assumptions: disparity noise 0.1 px per frame, 0.05 px offset per flight, average over 10 frames (optimistic: noise assumed independent).

| Camera | baseline | f (px) | maximum height for 5% | for 13% | disparity at 100 m |
|---|---|---|---|---|---|
| RealSense D435 ($375, 75 g, 3.4 W) | 50 mm | 686 | 29 m | 75 m | 0.34 px |
| Orbbec Gemini 2 ($234, 98 g, 2.5 W) | 50 mm | 629 | 27 m | 68 m | 0.31 px |
| OAK-D Lite ($269, 61 g, 3 W) | 75 mm | 433 | 27 m | 71 m | 0.32 px |
| OAK-D Pro ($429, 91 g) | 75 mm | 763 | 48 m | 125 m | 0.57 px |
| RealSense D455 ($499, 116 g, 3.5 W) | 95 mm | 686 | 54 m | 142 m | 0.65 px |
| ZED 2i ($499, 229 g, 1.9 W) | 120 mm | 774 | 78 m | 204 m | 0.93 px |
| OAK-D LR ($779, 415 g, 5.5 W) | 150 mm | 736 | 92 m | 241 m | 1.10 px |
| custom 0.30 m (George 2023) | 300 mm | 1,000 | 253 m | > 600 m | 3.0 px |
| custom 0.41 m (Song 2017) | 410 mm | 1,000 | 346 m | > 600 m | 4.1 px |
| custom 1 m (wing) | 1,000 mm | 1,000 | > 600 m | > 600 m | 10 px |

Published specs (manufacturer pages, f calculated from the horizontal field of view): [D435](https://www.realsenseai.com/products/stereo-depth-camera-d435/), [D455](https://www.realsenseai.com/products/real-sense-depth-camera-d455f/), [OAK-D Lite](https://docs.luxonis.com/hardware/products/OAK-D%20Lite), [OAK-D Pro](https://docs.luxonis.com/hardware/products/OAK-D%20Pro), [OAK-D LR](https://docs.luxonis.com/hardware/products/OAK-D%20LR) (82° lens assumed), [ZED 2i](https://docs.stereolabs.com/docs/products/cameras/zed/specifications), [Gemini 2](https://www.orbbec.com/products/stereo-vision-camera/gemini-2/). Subpixel noise: < 0.1 px RMS on a textured flat target according to the [RealSense guide](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/); this is not a guarantee outdoors. Manufacturer accuracy claims (< 2% at 2–4 m) say nothing about height at 50–300 m.

Published flight results: George et al. 2023 (0.30 m baseline, 40 to 100 m): best trajectory error of 2.2 m at 60 m, but 102 to 123 m at 100 m for pure stereo VO and 9.5 to 12.9 m with IMU ([PDF](https://mdpi-res.com/d_attachment/drones/drones-07-00036/article_deploy/drones-07-00036-with-cover.pdf?version=1705552179)). This confirms the rapid degradation beyond a few dozen metres for every decimetre of baseline. PUBLISHED.

Comparison: barometer (Zurich) + DEM on flat terrain gives 4.2% at 100 m and 2.8% at 150 m after 600 s; on 15%-slope terrain with a 100 m position error, 15.6% at 100 m.

Temporal multi-view stereo (Song et al. 2017, PUBLISHED https://pmc.ncbi.nlm.nih.gov/articles/PMC5298584/): the baseline is V·Δt, so relative height error cannot be lower than relative speed error. But speed is what we are trying to estimate (optical flow = V/h). **Circular dependency**: temporal stereo provides no scale without an independent speed source (GNSS in Song 2017). INFERENCE (geometry).

What height provides:
- camera speed: scale error = relative height error;
- map fix: the zoom search covers ± 3σ of the relative error; at 5% one zoom is almost enough, while at 13% several are needed (ALTO: a 7% zoom change dropped raw ZNCC to 37–40%, MEASURED by track A).

## 5. Altitude from video: evaluation bias (SIMULATED)

Part V of the same script. Flat terrain, ground at 120 m, 15 m/s, along-track drift due to height alone:

| Height source | 60 s (900 m) p50/p95 | 300 s (4.5 km) | 600 s (9 km) |
|---|---|---|---|
| Zurich-calibrated barometer + 3 m error at cutoff | 16/44 m | 82/232 m | 183/520 m |
| Zurich-calibrated barometer, perfect height at cutoff | 3/9 m | 31/90 m | 87/248 m |
| “Video” altitude (SfM-like, 1% scale, no drift) | 6/18 m | 30/87 m | 61/174 m |

Interpretation: an altitude reconstructed from video **underestimates drift by a factor of 2.7 to 3** compared with a real barometer. It does not have the barometer's random walk and shares information with the camera (leakage).

Proposed fair protocol:
1. Never call an altitude derived from video a “barometer.”
2. Synthetic barometer = reference altitude (GNSS/RTK from the flight or simulation) + error drawn from a model calibrated on real logs (Zurich: 0.30 m; 0.112 m/√s; 0.0024 m/s), at least 20 draws, results as median/p95.
3. Also show the result with a real barometer (Zurich or PX4 logs) to check the calibration.
4. Declare the error at cutoff (vertical GNSS + DEM): this is the dominant term at 10 min.

## 6. Solar sensor (SIMULATED + PUBLISHED)

σ_cap = √((σ_capteur/cos é)² + (σ_inclinaison · tan é)²). Part H of the script. Examples (degrees):

| Sensor / AHRS tilt | é = 30° | 50° | 70° | 80° |
|---|---|---|---|---|
| 0.1° / 0.5° | 0.31 | 0.62 | 1.40 | 2.89 |
| 0.5° / 0.5° | 0.65 | 0.98 | 2.01 | 4.04 |
| 1.0° / 2.0° | 1.63 | 2.85 | 6.22 | 12.72 |

The attitude reference's tilt error, not the sensor, dominates above 40° elevation.

Taiwan geometry (NOAA ephemerides, 7–17 h, sensor at ±60°, σ ≤ 2°): 50 to 74% of the day with a good attitude reference; 11 to 21% with a “field-grade” reference (2°). Noon elevation: 43° in December (Taipei) to 88–89° in June–July.

CWA sunshine (PUBLISHED, 1991–2020 normals, https://www.cwa.gov.tw/V8/C/Statistics/MonthlyMean/MOD/Taiwan_sunshine.html; `python experiments/s_taiwan_sunshine.py`): annual sunshine fraction is 0.32 in Taipei, 0.46 in Taichung, and 0.52 in Kaohsiung; Taipei is 0.23 to 0.25 from January to April; Taichung is 0.58 in October. Product (INFERENCE, assuming independence): usable sun for 40% of the day in Taichung in October, 12% in Taipei in January.

Share of ALTO cross-track drift (198 m, 3°) removed, SIMULATED over 4.6 km: good attitude reference, 76% (161 → 38 m); with 40% availability and gyro between updates, 75% (40 m); 2° reference, 30% (113 m). On the actual ALTO camera motion (section 7): a sensor accurate to ≈ 1° reduces cross-track error from 196 m to 31 m (84%); at 2.85°, it is 32 m. Final error falls by only 6% because along-track error dominates. The solar sensor does not affect the 575 m along the route.

Hardware (PUBLISHED):

| Sensor | accuracy | field of view | mass / power | price |
|---|---|---|---|---|
| [Foresail-1 PSS](https://github.com/foresail/fs1_psd_sun_sensor), open source, 4-electrode photodiode + pinhole | < 5° (paper); ± 1° (README) | ± 50° | 4 g / 4 mW | ≈ €100 in parts |
| [Foresail-1 DSS](https://github.com/foresail/fs1_dss_sun_sensor), open source, CMOS + lithographed pinhole | < 0.5° | ± 18° | 4.4 g / 10 mW average | ≈ €200 |
| [Solar MEMS nanoSSOC-A60](https://solar-mems.com/wp-content/uploads/2024/01/nanoSSOC-A60.pdf) | < 0.5° (3σ) | ± 60° | 4 g / < 10 mW | €2,500 |
| [Bradford Mini-FSS](https://www.bradford-space.com/products/mfss), passive quadrant | ± 1.5° without a table, ± 0.2° with one | ± 64° | 50 g / 0 W | not published |
| Polarization compass (Sony IMX250MZR camera, [Pan et al.](https://doi.org/10.1364/OE.510283)) | 0.10° clear sky, 0.18° haze, 0.3–0.5° thick clouds (ground) | sky | camera | $1,500–3,000 |

Published flight tests: a photoresistor solar sensor on an AR.Drone, uncertainty “mostly under 10°” ([Liu et al. 2013](https://www.uaslaboratory.com/_files/ugd/49bf50_d749c969f91a4d4cabaf281f1edf2454.pdf)); a polarization compass on a quadrotor, < 2° ([Zhi et al. 2018](https://pmc.ncbi.nlm.nih.gov/articles/PMC5795797/)); ≈ 0.5° at 310 m ([Zhao et al. 2022](https://doi.org/10.1016/j.measurement.2022.110734)). JPL measured tilt-to-heading coupling on a rover: ≈ 0.5° heading per degree of roll/pitch ([Trebi-Ollennu et al.](https://robotics.jpl.nasa.gov/media/documents/IEEETRA_Sun.pdf)); our formula gives this ratio at 27° elevation (the JPL test elevation has not been verified).

Interpretation for Taiwan (INFERENCE): a polarization compass tolerates overcast better than a direct solar sensor (CWA sunshine counts only direct sunlight). It is the only “celestial heading” option that still works under Taipei skies with 25% sunshine, but it costs $1,500 or more and has the same tilt coupling.

Alternative tested by another agent (MEASURED, `experiments/u1_shadow_compass.py`, `data/processed/u1_shadow_compass/explicit_shadow_test.json`): read heading from shadows in the camera image. On ALTO, 6 of 100 images were valid, with a 51° circular standard deviation; on Wufeng, 1 patch of 30 was valid. Abandoned (stop threshold: 10°).

## 7. Combination: which sensor per dollar and per watt (SIMULATED, then checked on the real ALTO route)

Part C: 1,000 flights, 4.6 km at 15 m/s, cruising at a constant altitude of 120 m above rolling terrain (ground 60 to 181 m below the drone). The fixed scale reproduces ALTO's order of magnitude (17% relative error at the end, 484 m along the route; ALTO: 13%, 575 m).

| Scale | Heading | final error p50 / p95 | distance until p95 > 60 m | added cost | power |
|---|---|---|---|---|---|
| fixed (current) | camera 3° | 549 / 1,360 m | 315 m | $0 | 0 W |
| barometer only | camera 3° | 542 / 1,329 m | 315 m | $0 | 0 W |
| barometer + DEM | camera 3° | 234 / 490 m | 563 m | $0 | 0 W |
| barometer + DEM | gyro | 147 / 320 m | 900 m | $0 | 0 W |
| barometer + DEM | sun, 2° attitude reference | 187 / 382 m | 720 m | ≈ $150 | 0.01 W |
| barometer + DEM | sun, good attitude reference | 123 / 292 m | 945 m | ≈ $1,650 | 1 W |
| OAK-D LR | gyro | 205 / 506 m | 540 m | $779 | 5.5 W |
| 1 m stereo | sun, good attitude reference | 48 / 110 m | 2,520 m | ≈ $2,050 | 5 W |

The barometer alone adds nothing in level-altitude cruise: it tracks altitude, not the ground. A DEM is needed. Costs: an open Foresail-type solar sensor costs ≈ $150 in parts (section 6 table); a “good attitude reference” (0.5° tilt in flight without GNSS) costs ≈ $1,500, INFERENCE with no source; custom 1 m stereo costs ≈ $400 (two OV9282 modules at $90 each + bar + compute), INFERENCE.

Sensitivity (SIMULATED, same seed, median final along-track error):

| Variant | fixed scale | barometer + DEM | OAK-D LR | 1 m stereo |
|---|---|---|---|---|
| reference terrain (ground 60–181 m below drone) | 484 m | 103 m | 169 m | 19 m |
| flat (terrain × 0.1; Wufeng-like) | 87 m | 100 m | 173 m | 19 m |
| terrain × 2 (some flights skim the ground) | 984 m | 121 m | 162 m | 18 m |
| DEM error 8 m (DSM in a city or forest) | 484 m | 247 m | 169 m | 19 m |

Commands: `python experiments/s_drift_budget.py --terrain-scale 0.1 --out data/processed/drift_budget_flat`, `--terrain-scale 2 --out data/processed/drift_budget_hilly2x`, `--dem-sigma 8 --out data/processed/drift_budget_dem8m`.

Interpretation: **the DEM adds nothing on flat terrain** (cutoff height error dominates); the free gain then comes from the gyro. The DEM becomes decisive when ground elevation varies by more than a few percent of flight height. With an 8 m DEM error, the OAK-D LR moves ahead again.

Check on the actual ALTO route (MEASURED camera and terrain, SIMULATED barometer; another agent's unchanged script: `.venv/bin/python experiments/u2_baro_dem_scale.py`, results in `data/processed/u2_baro_dem_scale/summary.json`): camera fix over the first 300 m, then 4.3 km without a fix.

| Height used for scale | median scale error | median error | final error |
|---|---|---|---|
| fixed (team) | 9.1% | 472 m | 608 m |
| simulated barometer − DEM read at estimated position | 10.5% | 372 m | 639 m |
| true ground height (oracle) | 0% | 270 m | 426 m |

Interpretation: **my simulation overestimates the DEM's benefit on ALTO.** It attributes all along-track error to ground height; on the real flight, perfect height removes only 30% of final error. The rest comes from heading (198 m cross-track) and errors in camera speed itself (flow → ground calibration, tilt, terrain parallax). INFERENCE about the breakdown. Barometer + DEM fails without fixes because the estimated position drifts.

Simulated fixes on the actual ALTO route (camera MEASURED, barometer and fixes SIMULATED: fix = ground truth + 15 m, zoom refreshed to within 5.7%; 20 seeds): `python experiments/s_alto_fix_spacing.py` (1 s; another agent's script, reproduces u2 within 0.001 m). Share of fixes arriving with more than 60 m error (map-matching bench search radius):

| Spacing | fixed (team) | barometer − DEM | true height (oracle) |
|---|---|---|---|
| 300 m | 31.8% | 24.6% | 18.9% |
| 500 m | 71.9% | 58.8% | 39.4% |
| 1,000 m | 98.8% | 100% | 100% |

Interpretation: **with fixes, barometer + DEM helps**: it removes about a quarter of failed fixes at 300 m (31.8 → 24.6%) and a fifth at 500 m. It cannot increase fix spacing beyond ≈ 300 m on ALTO, even with perfect height: the remaining error comes from elsewhere (heading, camera speed). This matches the “cliff” the team measured between 300 and 400 m.

Heading on the actual ALTO route (camera MEASURED; oracle heading = ALTO yaw; sensors SIMULATED over 20 seeds): `python experiments/s_alto_heading.py` (3 s; another agent's script, reproduces u2 within 0.04 m). Final error and components (projected onto the flight direction over the last 200 m):

| Scale | Heading | final error | along-track | cross-track |
|---|---|---|---|---|
| fixed | fixed (team) | 608 m | −576 m | −196 m |
| fixed | oracle | 572 m | −571 m | +31 m |
| fixed | sun ≈ 1° | 573 m | −571 m | +31 m |
| fixed | sun 2.85° | 583 m | −573 m | +32 m |
| true height | fixed | 426 m | −372 m | −208 m |
| true height | oracle | 369 m | −367 m | +33 m |
| true height | sun ≈ 1° | 371 m | −367 m | +34 m |

Interpretation: heading error is orthogonal to along-track error, so it matters only after the latter is reduced. The gyro variant (637 m) is not interpretable: it assumes one ALTO row per second, an unverified time base.

Where do the remaining 367 m come from? (MEASURED, diagnostic with oracles; `python experiments/s_alto_speed_residual.py`, 1 s, another agent's script reproducing 368.5 m):

| Variant (true height + oracle heading) | final error | along-track | median estimated step / true step |
|---|---|---|---|
| flow → ground calibration over 300 m (team) | 369 m | −367 m | 0.90 |
| calibration over the entire route (diagnostic, ground truth used) | 74 m | −73 m | 0.97 |
| isotropic calibration (scale + rotation) over 300 m | 439 m | −418 m | 0.90 |

Across 22 segments of 200 m, the step ratio first tracks image-flow amplitude in pixels (r = 0.56), then ground height (0.35); tilt (−0.29) and terrain slope (0.12–0.16) have little effect. Diagnosis: calibration learned over 300 m is biased by ≈ 7–10%, and the bias depends on image-motion amplitude (nonlinear flow estimator behavior, motion blur; unproven assumption). Options: calibrate over a longer GNSS distance or at several speeds, correct flow nonlinearity, and let each map fix refresh the scale (which the team already does). This is the biggest opportunity, ahead of any sensor.

## 8. Other real barometers compared with RTK: INSANE and PX4 logs (MEASURED)

Command: `python experiments/s_insane_baro.py` (1 s; another agent's script, reviewed and rerun here). Data: [INSANE](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/datasets/insane-dataset/) (BSD-2 + no-sale restriction), 3 kg multirotor, MS5611 barometer at 20 Hz, u-blox RTK; only fixed RTK samples are retained. Three sequences of 18, 8, and 26 MB in `data/raw/insane/`.

| Sequence | fixed RTK duration | altitude | 10 s median / p95 | 60 s | 120 s | scale error |
|---|---|---|---|---|---|---|
| mars_1 | 100 s | 16–21 m | 0.38 / 1.40 m | 0.66 / 1.64 m | – | +6.7% |
| mars_2 | 164 s | 16–32 m | 0.48 / 2.06 m | 0.63 / 2.23 m | 0.96 / 2.48 m | +3.0% |
| outdoor_1 | 219 s (22% fixed) | 0–24 m | 0.09 / 0.29 m | 0.63 m (64 pairs) | – | −0.8% |
| Zurich (reference) | 2,713 s | 460–489 m | 0.38 / 1.46 m | 0.55 / 1.96 m | 0.80 / 2.55 m | +7.2% |

Interpretation:
- At short horizons (10 to 120 s), two vehicles, two barometers, and two different references (photogrammetry, RTK) give the **same order of magnitude**: ≈ 0.4 m at 10 s, ≈ 0.6 m at 60 s, ≈ 0.8–1 m at 120 s. The Zurich-calibrated model is plausible up to 2 min.
- **Nothing confirms it beyond 2 min**: the INSANE sequences are too short. Drift at 5–10 min has been measured on only one flight (Zurich); the literature (Morales 2022: 0.6 m/10 min) reports less.
- The positive **scale error** recurs (+3 to +7%) on two of the three vehicles: the barometer exaggerates altitude changes. Cause unknown (propeller wash, temperature). Practical implication: estimate a barometer scale factor before the cutoff if altitude changes during the GNSS phase. INFERENCE.

PX4 Flight Review logs with fixed RTK (MEASURED, CC BY 4.0, exported by track C): `python experiments/s_px4_rtk_baro.py` (2 s; another agent's script, rerun here). Reference = fixed RTK GNSS altitude from the same log (independent of the barometer, but not a geodetic survey).

| Log | in flight | altitude / speed | 60 s median | 300 s | 600 s | 1,200 s | p95 600 s |
|---|---|---|---|---|---|---|---|
| 036fb3a7, 10" octocopter | 1,167 s | 3–108 m; 7.9 m/s median, 10.3 max | 1.08 m | 1.34 m | 1.13 m | – | 4.48 m |
| a2a30b98, quadcopter | 1,951 s | 3–15 m; 0.7 m/s | 0.73 m | 0.81 m | 0.99 m | 1.45 m | 2.77 m |
| 53736001, quadcopter (non-standard firmware) | 328 s in 3 segments | −6–96 m; 0.3 m/s | 0.61 m | (95 pairs) | – | – | – |
| Zurich (reference) | 2,713 s | 460–489 m; 0.8 m/s | 0.55 m | 1.53 m | 2.45 m | 4.45 m | 6.01 m |

Interpretation:
- At 5–20 min, the two long PX4 logs drift **half as much as Zurich** (median ≈ 1 m at 600 s versus 2.45 m; p95 2.8–4.5 m versus 6.0 m). With Morales 2022 (0.6 m/10 min), the Zurich model is a **pessimistic bound**; we keep it in simulations for lack of better data and to be cautious.
- At short horizons, noise depends on the vehicle: median 0.5 to 0.8 m at 10 s on the PX4 logs (propeller wash, flight at 3–15 m), 0.4 m in Zurich and INSANE.
- **Effect of speed** (octocopter log, up to 10 m/s): coefficient 0.0066 m per (m/s)², or ≈ 0.7 m at 10 m/s and ≈ 1.5 m at 15 m/s by extrapolation. Small compared with drift, but confounded with altitude in the regression. The other two logs are too slow to draw conclusions (their coefficients have opposite signs and are unusable).
- Scale errors: −5.9% (octocopter), −17% (a2a30b98, over only 12 m of elevation change), +5.2%. With Zurich (+7.2%) and INSANE (+3.0% and +6.7%), **barometer scale is off by 3 to 7% in either direction**: estimate it before cutoff whenever altitude changes.

## 9. Limitations and open questions

- Sections 4 to 6 and the synthetic part of section 7 are SIMULATED. MEASURED: barometer noise (Zurich 45 min, INSANE 3 short flights, 3 PX4 RTK logs), IMU replays (Zurich, INSANE), and ALTO camera motion in the section 7 replays (barometer, fixes, and heading sensors are still simulated). No real measured flight here is 100–300 m above ground at 15 m/s for 10 min.
- The synthetic simulation in section 7 overestimates the DEM gain: it assigns all along-track error to ground height, while on ALTO height accounts for only about one third.
- GLO-30 DEM is a surface model (roofs, trees): this is what the camera sees, but the DEM and true ground can differ by 10 m in a city or forest. INFERENCE.
- The effect of speed on the barometer's static port has only been measured up to 10 m/s (≈ 0.7 m), on one log.
- Residual gyro bias of 0.005 °/s: assumption not measured on the team's hardware.
- Questions for the team (in `questions.md`): demo date/time, autopilot tilt error, terrain under the route.

## Files

- `experiments/n_sensor_fusion.py` (reviewed, fixed) → `data/processed/sensor_fusion/`
- `experiments/s_zurich_vertical.py` → `data/processed/zurich_vertical/`
- `experiments/s_drift_budget.py` → `data/processed/drift_budget/`
- `experiments/s_taiwan_sunshine.py` → `data/processed/taiwan_sunshine/` (raw: `data/raw/cwa_sunshine/`)
- `experiments/s_insane_baro.py` → `data/processed/insane_baro/` (raw: `data/raw/insane/`)
- `experiments/s_insane_vertical.py` → `data/processed/insane_vertical/`
- `experiments/s_px4_rtk_baro.py` → `data/processed/px4_rtk_baro/` (logs exported by track C)
- `experiments/s_alto_fix_spacing.py`, `experiments/s_alto_heading.py` → `data/processed/alto_fix_spacing/`, `data/processed/alto_heading/` (import `experiments/u2_baro_dem_scale.py` from another agent, without modifying it)
- `experiments/s_alto_speed_residual.py` → `data/processed/alto_speed_residual/`

## 10. Online calibration of camera odometry from map fixes (preregistered before execution)

Question: the fixed calibration over 300 m explains most of the ALTO error (369 → 74 m with perfect calibration, section 7). Can scale and heading bias for camera odometry be learned in flight, without ground truth, from the displacements between successive accepted map fixes?

Method (`experiments/s_alto_online_calib.py`): team fix logic reused from `h_alto_end_to_end.py` (brightness correlation, score threshold 0.33, search sized by uncertainty). State: position, log-scale, heading bias. Between accepted fixes, compare the fix-to-fix displacement with integrated camera displacement: the length ratio gives log-scale, the angle difference gives heading, with noise of √2 × 15 m / distance. Ground truth is used only for scoring.

Criteria set before the run:
- **P1**: fixes every 1,000 m, median error ≤ 40 m (team: 56 m) **and** no accepted fix more than 50 m wrong.
- **P2**: fixes every 300 m, median error no more than 3 m worse than the team's (31 m).
- **P3**: fixes every 300 m until 2 km after cutoff, then none: distance travelled before 100 m error is at least 1.5 times that of uncalibrated odometry.
- Safety failure: a false fix more than 50 m wrong is accepted in a configuration where the team accepted none.

Results (MEASURED: ALTO images and real correlation fixes; `python experiments/s_alto_online_calib.py`, 11 min; another agent's script, team variant reproduces `findings.md` within 0.5 m). Median / final position error; fixes accepted / rejected / accepted but more than 50 m wrong:

| Spacing | team (zoom refreshed) | online (fix to fix) | online + zoom |
|---|---|---|---|
| 300 m | 31 / 38 m; 12/1/0 | 36 / 360 m; 10/5/**1** | **30** / 116 m; 12/2/0 |
| 500 m | 36 / 36 m; 6/1/0 | 162 / 81 m; 3/4/0 | **30** / 87 m; 7/1/0 |
| 1,000 m | **56** / 10 m; 4/0/0 | 162 / 164 m; 2/1/0 | 63 / 138 m; 4/0/0 |
| 2,000 m | 115 / 197 m; 1/0/0 | 107 / 96 m; 1/0/0 | 108 / 174 m; 1/0/0 |

Cut off fixes at 2 km (after fixes every 300 m): distance before 50 m / 100 m error: team 200 / 952 m; online + zoom 631 / 982 m; online 476 / 902 m. Odometry without any fixes already exceeds 100 m error at cutoff.

Parameters learned in flight (online + zoom, 300 m): log-scale +0.15 (steps lengthened by 16%), heading bias +3.2°. These agree with defects measured elsewhere (steps 10% too short, heading off by 3°). The “calibration over the entire route” oracle cannot be reduced to scale + rotation (relative singular values 3.1 and 1.1): matrix A0 learned on a 300 m straight line is poorly conditioned along the cross-track axis.

Preregistered verdict:
- **P1 fails**: at 1,000 m, 63 m (online + zoom) and 162 m (online) versus the target of ≤ 40 m; the team gets 56 m.
- **P2**: passed for online + zoom (30 m versus 31 m); failed for online alone (36 m).
- **P3 fails**: my criterion said “uncalibrated odometry,” which was ambiguous. Without any fixes, error already exceeds 100 m at cutoff (ratio undefined). Against the team (fixes, zoom-based scale, no online calibration): 982 m versus 952 m before 100 m error (× 1.03). Only the 50 m threshold is reached later (631 m versus 200 m, × 3.2), a criterion that was not preregistered.
- **Safety failure** for online-only: one false fix was accepted at 300 m (the team accepted none).

Interpretation: the learned parameters point in the right direction, but a pair of fixes accurate to within 15 m over 300 m measures scale to within ≈ 7%, about the size of the defect to correct. The first noisy updates move the prediction; the search starts in the wrong place, and fixes are rejected or wrong. Fix zoom remains a better scale sensor than fix-to-fix displacement. Next idea (untested): learn only heading bias (stable at +3.2 to +4.2° across all spacings), keep the team's zoom for scale, and relearn A0 as a similarity transform on a route with turns.

## Next experiments

1. Camera speed calibration: learn flow → ground over a longer GNSS distance (500–1,000 m) or at several flow amplitudes, and measure flow estimator nonlinearity (r = 0.56 with amplitude); measured potential: 369 → 74 m with perfect calibration. Do this with the camera track.
2. Effect of speed on the barometer's static port during fast flights (> 15 m/s) with RTK: one log reaches only 10 m/s.
3. Measure the team's real autopilot tilt error without GNSS (filmed flight + log): this determines whether a solar sensor helps.
4. Add a barometer scale-factor state to the filter, estimated before cutoff (measured scale errors of 3 to 7%, in both directions).
5. Recover ALTO's time base (rows per second) to test the gyro variant on the actual route.

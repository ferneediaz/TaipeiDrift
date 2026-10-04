# REPORT — Obscuration-aware laser TRN for a GNSS-denied drone over Taiwan

Status: **final, computation stopped 2026-10-04** (M6 altitude/map sweeps and the M4 rerun were not completed).
How to run everything: [TUTORIAL.md](TUTORIAL.md). Design and data checks: [PLAN.md](PLAN.md).
All tables: [docs/results_tables.md](docs/results_tables.md).

## 1. What was built

A complete, reproducible simulator built from Taiwan's MOI 20 m elevation data:
- **Truth world:** the DSM (canopy and buildings), with ground under the canopy taken from the DEM.
- **Onboard map:** the DEM resampled to 30 m. Tests enforce that the filters never see the truth.
- **Flight and inertial navigation:** a fixed-wing trajectory, and a strapdown INS (navigation, tactical or MEMS grade) that matches textbook drift.
- **Sensors and weather:** a multi-echo 1550 nm laser altimeter, plus a cloud and valley-fog model.
- **Filters compared:**
  1. TERCOM;
  2. a literature-baseline marginalized particle filter (MPF) using a Gaussian likelihood on the last echo;
  3. the same MPF with a 4σ innovation gate;
  4. the **proposed obscuration-aware MPF**, whose likelihood is a mixture over ground, cloud/fog/canopy, clutter and no-return.

All filters run on identical sensor data (paired Monte Carlo). The suite has 38 unit tests, and a filter runs at ~2 ms per 10 Hz step, ≈ 50× faster than real time with 2000 particles.

## 2. Results

Each flight lasts ~40 min after GNSS loss. "CEP50" is the typical horizontal error; "div" is the share of flights ending more than 1 km off.

| condition | literature baseline | gated MPF | **proposed MPF** |
|---|---|---|---|
| Mountains, clear air (nav / tactical IMU) | 0.1–0.5 km, 60–80 % div | 8–13 m, 10–15 % | **7–12 m, 0–15 %** |
| Foothills, clear air | 0.1–0.4 km, 40–75 % | 11–18 m, 5–25 % | **9–14 m, 0 %** |
| 10 % cloud (mountains / foothills) | 3–28 km, 85–100 % | 8–19 m, 20–30 % | 8–18 m, 20–30 % |
| 30 % cloud | 19–61 km, 100 % | 1.4–6.7 km, 60–90 % | 0.02–7.9 km, 60–70 % |
| ≥ 50 % cloud | lost | lost | lost |
| Coastal plain, any condition | lost | lost | lost |
| 3 slanted beams instead of 1 (10 % cloud, foothills) | 18 km, 80 % | — | **12 m, 20 %** (vs 40 % with nadir) |
| MEMS IMU | lost | — | lost (85–100 %) |
| No barometer | — | — | same as with barometer |

TERCOM fails in almost every case under tactical-grade drift.

Key statistics:
- **Clear air:** in paired flights the proposed filter diverged in 1 of 80, the gated filter in 10 of 80 (McNemar p = 0.02).
- **10 % cloud:** no significant difference (p = 1.0).
- **30 % cloud:** the trend favours the proposed filter (19 vs 12 one-sided divergences), but it is not significant (p = 0.28).

## 3. Findings

1. **Forest canopy is itself an obscuration problem.** About 90 % of pulses over the mountains hit tree tops 10–30 m above the bare-earth map. The literature last-echo MPF therefore fails even in clear air. Explicit canopy/cloud modelling (the proposed likelihood) removes this failure and is the only variant with 0 % divergence in clear air.
2. **Light cloud (≤ 10 %) is handled** by both robust MPFs (≈ 10–20 m typical). The proposed model is not better than a simple gate here.
3. **Moderate cloud (30 %)** splits the flights. The proposed filter often keeps a much smaller typical error, but 60–70 % of flights still diverge. **≥ 50 % cloud is not survivable** with any filter: cloud patches hide the ground for minutes, and the INS drifts kilometres in that time.
4. **Flat terrain (the western coastal plain) is not navigable by laser TRN** with any IMU tested.
5. **Hardware matters as much as the likelihood:**
   - 3 slanted beams make even the baseline work in clear air and halve the proposed filter's divergence under cloud;
   - a MEMS IMU is not sufficient;
   - the barometer is not needed.
6. **Safety-relevant flaw:** with little information (heavy cloud, flat terrain), the particle filters drift *faster than the INS alone* (e.g. 3–12 km vs ~2 km with a navigation-grade IMU). A real system must detect "no usable terrain information" and fall back to the INS.

## 4. Can this guide a drone without GNSS?

**Partly — as an aid, not as a stand-alone navigation solution.**

- **Usable:** over mountains and foothills with up to ~10 % cloud, a tactical- or navigation-grade IMU and a laser altimeter (ideally 3 beams). The proposed filter keeps ~10–20 m typical accuracy with honest uncertainty (NEES ≈ 1–2). That is enough for mid-course navigation and for cueing a terminal sensor.
- **Not usable:**
  - over flat terrain;
  - in cloud cover above ~30 %;
  - with a MEMS IMU;
  - without an INS-fallback / integrity monitor. Today the filter can be worse than doing nothing when information is missing.
- **Before any real flight:**
  - fix the information-starvation behaviour (INS fallback, integrity check);
  - validate against real multi-echo laser data and a *foreign* map (e.g. Copernicus GLO-30). The truth and onboard map here share the MOI production lineage, so real map errors will be larger;
  - test real cloud climatology (our cloud model is synthetic).

## 5. Limitations (honest list)

- Simulation only. The cloud/fog model, canopy gap probability (0.3) and laser detection curve are assumptions, not measurements.
- The truth and onboard map come from the same agency (DSM 2024 / DEM 2025). The truth ground under canopy is the native DEM (a partial "inverse crime", reduced by 30 m resampling).
- Filter settings were tuned on separate seeds (results/tune…tune5), but the tuning was not exhaustive. Several bugs were found and fixed during the work:
  - overcounted correlated measurements;
  - a random walk in the linear-state regularisation;
  - a lose-lock adaptation loop.
- Sample sizes: 20 paired runs per M5/M6 point. Many differences between the gated and proposed filters are within the confidence intervals.
- Not completed: the M6 altitude and map-quality sweeps, and the M4 rerun with the final code. `docs/figures/m4_error_vs_time.png` shows the earlier code version; M5 cloud fraction 0 is the valid clear-air result.
- Flat-earth local frame; TM2 scale factor and meridian convergence are ignored; attitude errors are not modelled in the beam pointing of the filter.

# PLAN — Obscuration-Aware Laser TRN Simulator (Taiwan)

Status: **M0 data inspection done. Waiting for approval before any simulation code.**
Everything below the "Open questions" section is a proposal you can change.

---

## 1. M0 data inspection results

Scripts (read-only access to `Data/`): `scripts/m0_inspect.py`, `scripts/m0_compare.py`,
`scripts/m0_overview.py`, `scripts/m0_routes.py`. All of them read the rasters strip by strip
(1024 rows at a time) or decimated. No raster is ever loaded whole at full resolution.

### 1.1 Raster headers and statistics

| | DSM (truth) `DSMg_tawiwan_20m_20240627_g14.tif` | DEM (onboard source) `DEM_tawiwan_V2025.tif` |
|---|---|---|
| Size (cols × rows) | 10 173 × 19 081 | 10 035 × 18 852 |
| Pixel size | 20 m × 20 m | 20 m × 20 m |
| Data type | float32, 1 band, uncompressed, strip-tiled (1 row/strip) | same |
| Nodata | −32767 | −32767 |
| Bounds (TM2 m, pixel edges) | E 148 330 – 351 790, N 2 420 110 – 2 801 730 | E 150 970 – 351 670, N 2 422 130 – 2 799 170 |
| Embedded CRS | Transverse Mercator, CM 121°E, k 0.9999, FE 250 000, WGS84 ellipsoid (WKT named "GCS_WGS_1984", no EPSG code) | identical WKT |
| AREA_OR_POINT | Point | Point |
| Valid pixels | 94.67 M (48.8 %) | 90.57 M (47.9 %) |
| Elevation min / max / mean | −12.31 / 3947.52 / 734.1 m | −14.24 / 3947.46 / 757.6 m |
| Size in memory (float32) | 776 MB | 757 MB |
| Spikes (\|z − mean of 4 neighbours\| > 300 m) | 0 | 0 |

The maximum of 3947.5 m is Yushan (official height 3952 m), which is plausible.

### 1.2 CRS: evidence that this is TWD97 / TM2 zone 121 (EPSG:3826)

1. **Embedded GeoTIFF tags**: Transverse Mercator, central meridian 121°, scale 0.9999,
   false easting 250 000 m, false northing 0. These are exactly the TM2 zone-121 parameters.
   The datum is labelled WGS84 rather than TWD97/GRS80, but the two agree to well under
   1 m, so this does not matter at 20 m resolution.
2. **Coordinate sizes**: eastings run from 148 k to 352 k and northings from 2.42 M to 2.80 M.
   This fits the expected EPSG:3826 range (E 150–350 k, N 2.42–2.80 M).
3. **DEM `Metadata.xml`** states it directly: "EPSG:3826 (TWD97/121分帶)". The bounding box
   is 119.88–122.24°E, 21.86–25.33°N.
4. **Cross-check**: Yushan's main peak (23.4700°N, 120.9573°E) converts with pyproj to
   (245 638, 2 596 330). Within ±1 km of that point, the DSM maximum (3947.5 m) lies at
   (245 640, 2 596 320), i.e. **inside the same 20 m cell**. This confirms the horizontal georeferencing.

→ Processed tiles will be written with an explicit **EPSG:3826** tag.

### 1.3 Grid alignment and pixel-origin convention

- **.tfw files** give the *centre* of the upper-left pixel: DSM (148 340, 2 801 720), DEM (150 980, 2 799 160).
  Rasterio reports the *corner* 10 m further up and to the left (148 330 / 150 970). Both files
  are consistent with each other and with `AREA_OR_POINT=Point`, so values are posts at pixel centres.
- **Lattice**: the DEM origin falls at DSM pixel (col 132, row 128), and both offsets are exact
  integers. The two grids share **the same lattice**, so cropping needs no resampling.
- The DSM covers a larger area (an extra 4.07 M pixels) because it is cut in rectangular blocks
  that include sea. The DEM is clipped at the coastline. Only 37 k pixels are valid in the DEM
  and missing in the DSM.

### 1.4 Vertical datum consistency (DSM − DEM)

Bare = DSM 3×3 standard deviation < 0.3 m (no trees or buildings), slope < 1°, 0.5 m < DEM < 150 m.

| Area | n (bare px) | median | mean | robust σ (1.4826·MAD) |
|---|---|---|---|---|
| All flat lowland, bare (island-wide) | 2.40 M | **+0.04 m** | +0.10 m | 0.07 m |
| Changhua–Yunlin plain | 248 k | +0.12 m | +0.19 m | 0.12 m |
| Chiayi plain | 209 k | +0.15 m | +0.28 m | 0.19 m |
| Pingtung plain | 87 k | −0.01 m | +0.20 m | 0.33 m |
| Yilan plain | 58 k | +0.04 m | +0.06 m | 0.03 m |

→ **No vertical datum offset** (it is below 0.2 m everywhere). The metadata does not state
the vertical datum. Both rasters are almost certainly TWVD2001 orthometric heights, and I will
treat heights as "MSL" throughout. The remaining +0.1–0.3 m is consistent with crops and
stubble in 2024 and with the 2024 → 2025 difference.

Overall DSM − DEM over all land: median +7.8 m, mean +9.2 m, p95 +25.7 m, p99 +35.9 m.
In the high mountains (Yushan box) the mean is +12.6 m and p95 is +32 m. This is forest canopy,
and it is **the dominant "map error" between truth and onboard** (see open question Q1).
About 30 k pixels differ by more than 60 m. The sampled examples cluster in Taipei/Keelung
(buildings) and on some mountain slopes (forest, landslides, or 2024→2025 changes).

### 1.5 Nodata, sea, artifacts

- **Sea**: inside its rectangular blocks the DSM contains sea values ≈ 0.2 m and a constant
  fill of about −3.0 m (−3.00 … −3.03: ≈ 180 k pixels). The DEM marks sea as nodata.
  → Land mask = DEM valid. Sea is set to 0 m in both truth and onboard maps, with a
  `is_sea` flag (see Q2).
- **Large hole**: a 154 km² square of nodata in **both** rasters at TM2 E 250 770–263 370,
  N 2 704 770–2 718 170 (121.01–121.13°E, 24.45–24.57°N, in the Xueshan / Dabajian area).
  The routes avoid it. The code will treat any nodata cell as "unknown" (the filter gives
  particles on nodata a flat, low likelihood).
- **Small interior holes** (at 200 m decimation): 7 regions in the DSM, 31 in the DEM, all small.
  These will be masked.
- **Negative values**: 2.36 M DSM pixels and 0.89 M DEM pixels. They are mostly sea fill,
  fish ponds and coastal land below MSL (down to −14 m), and are kept as real values on land.
- Figures: `docs/figures/m0_overview.png` (DSM, DEM, DSM−DEM) and `docs/figures/m0_routes.png`.

### 1.6 Machine constraints (important)

- 8 GB RAM, 8 cores (Apple Silicon, arm64). Python 3.11.13 in `.venv` (created with `uv`).
  Versions are pinned in `requirements.txt`.
- **Only ~2.8 GB of free disk space.** The `.venv` already takes 374 MB. Each route tile
  (≥ 20 km margin) is about 70–90 MB per float32 layer. With 3 routes × (truth 20 m + truth-ground 20 m
  + onboard 30 m + mask) that comes to ≈ 0.7 GB. Monte Carlo results must be kept small: summary
  metrics plus decimated tracks, not full particle clouds. Please free up some disk space if you can.

---

## 2. Proposed routes (please approve or adjust)

See `configs/routes_candidate.yaml` and `docs/figures/m0_routes.png`. The check uses DSM
heights with a ±500 m corridor.

| Route | Waypoints (lon, lat) | Length | Alt mode (default) | Terrain under track min / max | Min clearance (corridor) | Max nadir / 25° slant range |
|---|---|---|---|---|---|---|
| **A** Central Mtn crossing (Zhushan → Yushan → Yuli, W→E) | (120.62,23.72) (120.80,23.60) (120.96,23.47) (121.13,23.40) (121.32,23.33) | 84 km | 4500 m MSL | 125 / 3819 m | 552 m | 4375 / 4828 m ✓ |
| **B** Coastal plain (Changhua → Yunlin → Chiayi → Xinying, N→S) | (120.50,24.05) (120.42,23.85) (120.33,23.65) (120.28,23.45) (120.30,23.25) | 93 km | 1500 m MSL | 1 / 48 m (σ 4.9 m) | 1415 m | 1499 / 1654 m ✓ |
| **C** Taichung foothills → Puli → Sun Moon Lake → Zhushan | (120.62,24.17) (120.78,24.13) (120.88,24.03) (120.98,23.96) (120.92,23.85) (120.83,23.78) (120.66,23.74) | 88 km | 2000 m MSL | 70 / 1138 m | 792 m | 1930 / 2129 m ✓ |

At 35 m/s each route takes ≈ 40–44 min, which at 10 Hz is ≈ 25 k pulses.
For route A I can extend the start westward, into the plain, to make it ~110 km if you want
a plain → mountain transition inside one run.
Constant-AGL mode will smooth the commanded altitude, rate-limit it to a fixed-wing climb
rate, and assert clearance ≥ 300 m.

---

## 3. Module design

```
TRN/
  configs/            data.yaml (only place with absolute paths), routes.yaml, imu.yaml, laser.yaml,
                      atmosphere.yaml, onboard_map.yaml, filters.yaml, experiments/*.yaml
  trn/
    common/           config loading (yaml → frozen dataclasses), RNG streams, frames (pyproj), logging
    terrain/          Grid (affine + memmap array, point-convention), numba bilinear interp,
                      tile cropper (windowed rasterio → GeoTIFF + .npy memmap), roughness/gradient metrics
                      onboard_map.py → OnboardMap (DEM resample 30/40 m, correlated error field,
                      horizontal shift, GLO-30 placeholder loader with EGM2008/WGS84 TODOs)
    truth/            TruthMap (DSM 20 m + truth-ground layer, see Q1)  ← ONLY imported by sim side
    trajectory/       fixed-wing kinematics (coordinated turns, bank/climb limits, 30–40 m/s),
                      waypoint lon/lat → TM2, constant MSL / constant AGL, clearance + range asserts
    ins/              IMU error models (bias GM, ARW/VRW, scale factor) + strapdown mechanization in a
                      local-level frame; 15-state error model (F, Q) shared with the filters;
                      presets: navigation, tactical, MEMS; barometer model (bias + GM drift + noise)
    atmosphere/       cloud layers (base/top, coverage via thresholded Gaussian random field,
                      advected by wind), valley fog from terrain (fill below fog-top), extinction
                      → optical depth per beam, cloud-echo range sampling
    laser/            numba ray caster (march + bisection) on any Grid, beam geometry (nadir / N slant),
                      divergence footprint averaging, multi-echo (≤3) generation, P_detect(range, τ),
                      noise + quantization, 10 Hz pulse scheduler
    filters/          base.py (Filter interface: init(prior), predict(ins_delta, dt), update(pulse) → estimate, cov)
                      tercom.py, mpf.py (baseline), mpf_obscuration.py (proposed), likelihoods.py,
                      resampling.py (systematic, N_eff, roughening), diagnostics.py (NEES, divergence)
    experiments/      runner.py (one run = seed + config), monte_carlo.py (multiprocessing.Pool),
                      milestone scripts m1_…m7_
    analysis/         metrics (RMSE, CEP50/95, convergence time, false fix, divergence, NEES, usable-echo %),
                      plots, tables (CSV/markdown)
  tests/              pytest
  data/processed/     tiles (gitignored)
  results/<exp>/      config copy + npz + figures (gitignored)
```

**Truth-map isolation (structural):**
- `trn.filters` gets only an `OnboardMap` instance. The constructors check
  `isinstance(map, OnboardMap)` and reject anything else.
- `OnboardMap` is built only from the DEM path, the onboard-map config and a seed. It has no
  constructor that accepts a `TruthMap`.
- `tests/test_isolation.py` parses every module under `trn/filters/` (and the modules it imports
  inside `trn/`) with the AST. It fails if any of them imports `trn.truth`, references the DSM path
  key from `data.yaml`, or calls `rasterio.open`. A second test checks at runtime that
  `trn.truth` is not in `sys.modules` after a filter-only import.

**Frames:** horizontal position is in TM2 metres (E, N), and heights are orthometric (≈ MSL).
The INS runs in a local-level ENU frame aligned with grid north. Meridian convergence is
< 0.3° on these routes and the scale factor is 0.9999. Both are ignored and the omission is
documented. Earth curvature over a 25° slant beam (≤ 2.3 km horizontal) is ≈ 0.4 m and is also ignored.

**Performance:** for a nadir beam the predicted range per particle is a single bilinear lookup.
For slant beams it is one ray cast on the onboard map per particle. For 2000 particles × 3 beams × 10 Hz
that is 60 k ray casts/s, with each ray needing ~40 interpolations. This runs in numba
(`parallel=True` over particles). In the MPF the linear-state covariance is shared by all
particles (the standard MPF structure), so each step costs one KF covariance update plus a
per-particle mean update. Target: a full 40 min route in < 5 min with 2000 particles.

---

## 4. Modelling choices (proposal; numbers are config defaults)

- **Trajectory**: kinematic fixed-wing at 35 m/s, bank limit 30°, climb ≤ 5 m/s, smooth turn arcs.
  Truth outputs position, velocity, attitude, specific force and angular rate at 100 Hz.
- **INS**: ideal IMU from the truth, plus errors (bias as first-order GM, white ARW/VRW,
  optional scale factor), fed into a strapdown mechanization (flat-earth local level with
  Earth rate and normal gravity).
  The navigation/tactical/MEMS presets use textbook values, e.g. gyro bias 0.003 / 1 / 10 °/h,
  accel bias 25 / 300 / 2000 µg. GNSS loss happens just before TRN: initial errors are
  pos 500 m (1σ, horizontal), vel 0.5 m/s, attitude per grade.
  M3 sanity check: Schuler-bounded drift ~1–2 km/h for nav grade, tens of km/h for MEMS.
- **Barometer** (optional): bias 0 ± 30 m, GM drift σ 10 m / τ 600 s, noise 1 m.
  Without the baro, the vertical channel must be estimated from the laser. The MPF linear states
  include altitude.
- **Laser**: σ_range 0.3 m, quantization 0.1 m, R_max 5000 m, 10 Hz, divergence 0.5 mrad.
  P_detect for a clear-air ground return is about 0.99 at 1 km and 0.9 at 5 km (config curve),
  multiplied by exp(−2τ). Footprint averaging uses 7 rays across the divergence cone
  (sub-cell, via interpolation). The range is measured along the beam, and the INS attitude
  error enters through beam pointing.
- **Atmosphere**: (i) stratiform/cumuliform layers with base/top and a coverage field given by a
  Gaussian random field (correlation length 2–10 km) thresholded to the target cloud fraction.
  Extinction coefficient σ_ext ~ 20–100 km⁻¹ inside cloud (so a 100 m thick cloud gives
  τ ≈ 2–10), advected with the wind. (ii) Valley fog: cells where terrain < fog_top and lying in
  concave valleys (topographic position index < 0) are filled up to fog_top. (iii) Optional
  "mountain cloud belt" at 1500–2500 m, typical of Taiwan.
  Echo: the cloud echo range is drawn inside the cloud near its base, using an exponential
  penetration depth 1/σ_ext. The ground echo survives with probability exp(−2τ).
- **Filters**:
  - *TERCOM*: profiles of length L (default 8 km) are matched by grid search (MAD and MSD) over
    a ±3σ window, using the INS-relative profile shape. The fix is fed in as a position reset.
    Profiles with < 70 % valid echoes are skipped.
  - *Baseline MPF*: nonlinear states are horizontal position (E, N). The remaining 13 error
    states are linear, handled by a KF. The likelihood is Gaussian on the **last** echo with
    σ² = σ_laser² + σ_map² (σ_map from the M1 map-difference report, default 10 m). Missing
    echoes are skipped.
  - *Proposed obscuration-aware MPF*: for the echo set Z = {r₁ < … < r_k}, k = 0..3:
    p(Z|x) = (1−P_D(x)) Πᵢ c(rᵢ|x) + P_D(x) Σⱼ N(rⱼ; ρ(x), σ²) Π_{i<j} c_short(rᵢ|x) Π_{i>j} c_clutter(rᵢ).
    Here ρ(x) is the predicted ground range from the onboard map. c_short is the density of
    cloud and canopy echoes, defined only on [r_min, ρ(x) − δ]. The hypotheses are therefore
    position dependent: an echo *longer* than the predicted ground range cannot be cloud, and
    this carries information. c_clutter is uniform on [0, R_max]. With k = 0 the likelihood is
    (1−P_D(x)), which depends on range. Mixture weights (P_D, cloud and clutter rates) can be
    `fixed` or `adaptive` (online EM / Beta-Bernoulli on recent pulses) in config. A robust
    floor prevents particle collapse when there is a single outlier.
  - Shared by all filters: systematic resampling when N_eff < 0.5 N, roughening (Gordon,
    K = 0.2), divergence detection (innovation χ² window and N_eff collapse with re-init over a
    widened prior), NEES on (E, N).
- **Monte Carlo**: `multiprocessing.Pool(7)`, one seed per run derived via `numpy.random.SeedSequence`.
  Quick mode is 10 runs and default is 100. Raw per-run outputs are stored decimated to 1 Hz.
  Time budget: M5 (10 cloud fractions × 2 filters × 3 routes × 100 runs) at ≈ 3 min/run on
  7 cores comes to ≈ 4 h. I will first measure the actual runtime in M4.

---

## 5. Milestones (each ends with tests, a plot, a summary, and a stop for your review)

| M | Deliverable |
|---|---|
| M0 | ✅ inspection (this doc). After approval: `configs/data.yaml`, cropped and aligned route tiles in `data/processed/` (GeoTIFF EPSG:3826 + .npy memmap), land/sea and nodata masks |
| M1 | OnboardMap (30 m default, error field, shift, GLO-30 stub), map-difference report (histogram, bias, std, by mountain/plain and vegetation proxy), route terrain profiles, isolation test |
| M2 | Ray caster + clear-air laser; unit tests on analytic flat and sloped planes; simulated vs true profile plots |
| M3 | Trajectory + IMU + strapdown; unaided drift per grade vs expected; baro on/off |
| M4 | TERCOM + baseline MPF in clear air on A/B/C; profiling; expect A good, B poor (investigate if not) |
| M5 | Atmosphere on; cloud fraction 0–90 %; baseline vs proposed with CIs |
| M6 | Sweeps: AGL 1000–5000 m, nadir vs 3 slant, IMU grade, map res/errors, baro on/off |
| M7 | REPORT.md with honest limitations (shared MOI lineage, synthetic clouds, no real sensor data) |

---

## 6. Open questions — please decide (my default in **bold**)

1. **Vegetation / truth ground (most important).** The DSM includes canopy. A real 1550 nm
   multi-echo laser often gets a canopy echo first and a ground echo through gaps, but the DSM
   alone does not contain the ground under the canopy. Options:
   (a) **Truth = DSM for the top surface, plus a truth-ground layer = native 20 m DEM.** Where
   DSM − DEM > 2 m, the pulse produces a canopy echo and, with probability p_gap (default 0.3),
   a ground echo. The onboard map stays a degraded 30 m DEM with an error field. The proposed
   filter then models canopy echoes as "short returns", just like clouds.
   (b) Truth = DSM only. The laser always hits the canopy, and the filter sees an
   effective ±10–30 m map bias in forests.
   Option (a) partly re-uses the DEM on the truth side, so it is a partial inverse crime.
   It is mitigated by the 20 → 30 m resampling and the error field, and I would state it in REPORT.md.
2. **Sea**: **0 m in truth and onboard, with water returns enabled** (they are weak on calm
   water; the sea P_D is configurable)? Or forbid the routes from going over sea (none do now)?
3. **Routes**: approve A/B/C as listed, or extend A to ~110 km starting in the plain?
4. **Cloud parameters**: are you OK with my default ranges (σ_ext 20–100 km⁻¹, layer bases
   600–2500 m, valley-fog top 300–1500 m, mountain cloud belt 1500–2500 m)? Do you have
   Taiwan-specific climatology you want to use?
5. **INS fidelity**: **full strapdown mechanization with a synthetic IMU** (more realistic,
   slower), or propagate the 15-state linear error model directly (faster, but the same model
   as in the filter, which is an INS-side inverse crime)?
6. **Output format**: npz + CSV (no new dependencies), or add pandas + pyarrow for parquet?
7. **Disk space**: can you free ≥ 5 GB? If not, I will use one shared tile per route and
   store results compressed and decimated.

Once you approve (with any changes), I will start M0 tiling and M1.

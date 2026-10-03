# Tuniu digital twin: pre-registration (written before any navigation run on the twin)

## What the twin is

A virtual camera flies inside OpenDroneMap's own textured 2.5D mesh of the Tuniu reach (survey flight
2019-09-16) and takes one picture at every pose of the real April 2019 flight: RTK position, DJI gimbal
yaw/pitch/roll, plus the pre-cut boresight from `stage1_calibration.json`. The picture is an ideal pinhole
image with the real intrinsics scaled to 1368 × 912 (a quarter of the real 5472 × 3648), principal point
honoured, no lens distortion. Pixels where the mesh is not seen are black and flagged invalid. The navigation
code reads the twin exactly as it reads the real photos (same CSV files, same camera fields).

Code: `experiments/x9_tuniu_twin.py` (renderer, fidelity check, realistic camera) and
`experiments/x9_tuniu_twin_nav.py` (runs the unchanged Level 1 and Level 2 scripts on a twin folder). Renders
and comparisons stay in `data/processed` (licence of the survey photos unknown).

## Already done before this document (disclosed)

- Texture size chosen from the mesh itself: at 4096 px, half of the surface has a texel of 7 cm or less and
  97 % has 25 cm or less (2048 px: 14 cm and 85 %). Matching runs at 0.5 m.
- Projection check: markers at known 3D points land within 0.07 px of the predicted pixel.
- Placement: the mesh is offset from RTK by about 1.9 m. A constant camera offset (east +1.17 m, north
  −1.48 m) was estimated on the two pre-cut legs only (photos 1–46, one leg flown east and one west).
- Fidelity of the placed twin, all 271 photos, real vs twin rectified north-up at 0.5 m/px: median ZNCC 0.60
  at zero shift; median remaining shift 1.8 m (90th percentile 3.5 m); median rotation 0.25°. Under the
  ~2 m limit, no rotation bias. The remaining shifts vary by flight leg.
- Diagnostic `twin_att`: gimbal errors fitted per leg to the real photos (all photos) bring the twin closer to
  the photos (median shift 1.0 m) but raise its pre-cut odometry drift from 0.89 m to 1.5 m, while the real
  photos give 0.87 m. So those leg-wise differences are mostly mesh geometry, not camera attitude. The main
  twin keeps the nominal attitude. `twin_att_rc` is run below as a diagnostic only.
- Realistic camera (TaipeiDrift-sim `camera_model.py`, `cameras.realistic` settings, adapted to an oblique
  rectangular frame): applied to `twin` → `twin_rc`. On the 17 sample photos (rectified, 0.5 m) it brings
  contrast and gradient energy below the real photos (contrast sd: real 51, twin 55, twin_rc 43).
- Smoke tests on pre-cut photos only: Level 1 on 4 pre-cut photos (seed 0), and the Level 2 pre-cut
  calibration (`x5_tuniu_closed_loop.py calibrate`) of `twin`, `twin_rc`, `twin_att`, `twin_att_rc`.
  No test photo (47–271) has been run through either pipeline on any twin variant.

## Runs

All runs use the real-run settings unchanged: test photos 47–271, main map OAM 2019-12-12 at 0.5 m/px,
stage-1 map offset, Copernicus terrain, simulated barometer, seeds as listed.

**A. Sim-to-real on the same route (seeds 0–4).**

| Variant | Level 1 (consensus, ZNCC + XFeat within 4 m) | Level 2 (closed loop + dead reckoning, `dji`, `dem_lifted`) |
|---|---|---|
| real photos | re-run seeds 0–4 (seed 0 was never run; seeds 1–4 must reproduce the stored rows) | existing `x_tuniu_l2/runs/main`, seeds 0–4 |
| `twin` | yes | yes |
| `twin_rc` | yes | yes |
| `twin_att_rc` (diagnostic) | yes | yes |

Each twin variant's Level 2 noise model comes from its own pre-cut photos (`calibrate`), as for the real run.

**B. "Twin agrees with reality"** for a variant iff all four hold:

1. Level 2 closed-loop median error (pooled, seeds 0–4) within a factor 1.5 of real (real: 3.15 m → 2.1–4.7 m).
2. Level 2 accepted fixes (share of test photos) within ±10 points of real (real: 36.5 % → 26.5–46.5 %).
3. Level 1 consensus acceptance (mean over seeds 0–4) within ±10 points of the real re-run.
4. Level 1 median error of accepted fixes within a factor 1.5 of the real re-run.

Each criterion is reported separately; no criterion is changed after the runs.

**C. Matched-reality baseline for the sweep.** Between `twin` and `twin_rc`, the one whose Level 2 median is
closer to real in ratio (|log(twin / real)| smaller); tie (within 5 %) → the one with acceptance closer to
real. The chosen camera option (ideal or realistic) is applied to every sweep variant.

**D. Factor sweep (one factor at a time, closed loop + dead reckoning, seeds 0–2).** Uncontrollable factors
stay as in the baseline: the same September mesh texture and lighting, the same December map, the same
barometer model, the DJI attitude history of the real flight.

| Factor | Levels (baseline in bold) | Variant |
|---|---|---|
| Camera tilt from nadir | 0°, 15°, **30°**, 45° | `tilt0`, `tilt15`, `tilt45` |
| Photo interval | **2.8 s**, 1 s, 0.5 s (poses interpolated along the RTK track and attitude) | `dt1`, `dt05` |
| Altitude above the real track | **+0 m**, +50 m | `alt50` |
| Image size | **quarter (1368 × 912)**, half (2736 × 1824) | `half` |

Each variant gets its own pre-cut calibration. Same photo times for the cut (127.4 s).

Metrics per variant: pooled median / p90 / max error, share of photos under 10 m, accepted fixes (% of photos
and per minute), wrong accepted fixes (> 10 m), seeds without loss of lock, longest gap without a fix,
dead-reckoning median.

Decision rule per level vs baseline (3 seeds only, so coarse): **helps** = median error at least 25 % lower
with no new loss of lock and no wrong fix; **hurts** = median at least 25 % higher, or any loss of lock, or
any wrong fix the baseline did not have; otherwise **no clear effect**.

Expectation written now (not a criterion): image size should show no effect, because the pipeline decodes
every image down to the same pixel count before matching at 0.5 m (the real photos are read at 1/8 size).

## Limits known in advance

The mesh is 2.5D (no facades, smeared tree crowns), the texture is a September mosaic with its lighting baked
in, the map/texture date gap is 3 months in the twin against 8 months for the real photos, the twin camera is
a pinhole (narrower field of view than the real barrel-distorted lens), and there is no motion blur unless
the realistic camera is applied.

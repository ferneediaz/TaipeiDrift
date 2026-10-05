# Tuniu digital twin: results

Pre-registration: `docs/research/tuniu-twin-prereg.md` (commit 38b4cd7, pushed before any twin navigation
run). Code: `experiments/x9_tuniu_twin.py` (renderer, fidelity, realistic camera) and
`experiments/x9_tuniu_twin_nav.py` (runs the unchanged Level 1 / Level 2 scripts on a twin folder). All
renders, side-by-side images and run files stay in `data/processed` (licence of the photos unknown).

## Summary

- **The twin agrees with reality on all four pre-registered criteria**, for all three variants (ideal camera,
  realistic camera, fitted attitude). Same route, same code, same map, 5 seeds: Level 2 closed-loop median
  **2.81 m** on the twin with the realistic camera (`twin_rc`) against **3.15 m** on the real photos; accepted
  fixes 36.7 % against 36.5 %; no wrong accepted fix (> 10 m) on either.
- **Slightly optimistic in the median, pessimistic in the tails.** The twin medians are 3–14 % lower than real,
  but its 90th percentile is 22–25 m against 18 m, and 3 of 5 twin seeds lose lock at least once (real: 0 of 5).
- **Baseline for the sweep: `twin_rc`** (rule C below).
- **Sweep (3 seeds, one factor at a time, rule D):** a photo every 0.5 s **helps** (median 2.05 m against
  2.87 m, p90 7.9 m against 18.9 m, no loss of lock). A photo every 1 s and +50 m altitude: **no clear effect**
  by the rule (−19 % and −20 %), with no loss of lock in 3/3 seeds and a lower p90. Image size: no effect, as
  expected. Camera tilt 0°, 15° or 45° instead of 30°: **hurts** (median 15–102 m), on a twin only checked at 30°.
- The real Level 1 re-run reproduces the stored rows of seeds 1–4 exactly (7,200 rows, 0 difference in error,
  score or decoy position), as the pre-registration required.

## What the twin is

- **World**: OpenDroneMap's own textured 2.5D mesh of the 2019-09-16 survey (`odm_texturing_25d`), never
  rebuilt or edited. Textures are read at 4096 px instead of 8192 px to fit in memory. At 4096 px, half of
  the surface has a texel of 7 cm or less, 90 % has 15 cm or less, 97 % has 25 cm or less (2048 px: 14 cm,
  29 cm, 85 %). That is finer than the 0.5 m/px matching grid and finer than the pixel the pipeline actually
  uses (real photos decoded at 1/8 size: about 0.25–0.32 m on the ground). Source: `x9_tuniu_twin/texel.json`.
- **Camera**: one picture per real photo pose: RTK position, DJI gimbal yaw / pitch / roll, plus the pre-cut
  boresight (pitch −1°, yaw −0.5°, roll −1°). Ideal pinhole, real intrinsics scaled to 1368 × 912, fx and fy
  kept separate, principal point honoured through an explicit projection matrix. Check: markers placed at
  known 3D points land within 0.07 px of the predicted pixel. No lens distortion, no lighting (texture
  colours as they are), trilinear mipmapped textures, rendered at 2736 × 1824 and averaged down 2 × 2.
  Pixels with no mesh are black and flagged invalid; the pipelines ignore them.
- **Placement**: the mesh sits about 1.9 m off the RTK frame. A constant camera offset (east +1.17 m,
  north −1.48 m) was estimated on the two pre-cut legs only (photos 1–46).
- **How the pipelines read it**: the twin folder has the same CSV files as the real replay. The pipelines run
  unchanged; the wrapper only points them at the twin folder and decodes the renders to the same pixel count
  the real photos are decoded to (1368 px wide → 684 px, like 5472 px → 684 px).
- **Realistic camera** (`_rc`): TaipeiDrift-sim `camera_model.py` with the `cameras.realistic` settings of
  `sim_navigator.yaml` (cloud shadows 30 % cover / 35 % darker, haze 0.9, vignetting 0.4, residual
  distortion 0.004, blur 0.6 px, auto-exposure wobble, shot noise, JPEG 80), adapted to an oblique rectangular
  frame (cloud shadows placed where each pixel's ray meets the ground).

## Fidelity, checked before any navigation run

Real photo vs twin render of the same pose, both flattened north-up at 0.5 m/px with the same geometry
(RTK pose, DJI attitude + boresight, Copernicus terrain). "Shift" is the move that best lines the twin up
with the photo (ZNCC search ±12 m, rotation about the nadir ±3°). "Reliable" = photos where that search has
a clear peak (ZNCC ≥ 0.6, not at the search edge).

| | Photos | ZNCC at zero shift (median) | Shift median / p90 | Rotation median / abs p90 | Photos within 2 m |
|---|---|---|---|---|---|
| Twin before placement, all photos | 271 | 0.57 | 2.2 / 4.7 m | 0.25° / – | – |
| Twin before placement, reliable photos | 172 | – | 2.7 / 4.3 m | 0.5° / 1.75° | 39 % |
| **Twin (placed), all photos** | 271 | **0.60** | **1.8 / 3.5 m** | **0.25°** / – | – |
| **Twin (placed), reliable photos** | 167 | – | **1.65 / 2.5 m** | **0.25° / 1.5°** | **71 %** |
| Twin, rule-fixed sample (every 20th + 47, 100, 185) | 17 | 0.62 | 1.9 / 4.9 m | 0.25° | – |
| Twin + realistic camera, same sample | 17 | 0.60 | 2.6 / 4.2 m | 0.25° | – |

For scale: the real photos against the December map score a median ZNCC of 0.55 on the pre-cut photos.

What the pairs show (images in `data/processed/x9_tuniu_twin/fidelity/twin/frame_*.jpg`: top real | twin,
bottom real patch | twin patch | 20 m checkerboard):

- Roads, roofs, the school pools and the river banks line up; the checkerboard edges are continuous on
  photos 1, 47, 141, 161, 261.
- Forest is where the twin is weakest. The 2.5D mesh smears tree crowns into streaks (photos 181, 185:
  ZNCC 0.06–0.20). The real pipeline never fixes on these forest photos either.
- The remaining shift changes from one flight leg to the next (±1–2 m), with no overall rotation. Fitting
  per-leg gimbal errors to the photos (`twin_att`) lowers the median shift to 1.0 m, but it also raises the
  twin's pre-cut odometry drift from 0.89 m to 1.5 m, while the real photos give 0.87 m. So most of the leg-wise
  difference is in the mesh geometry, not in the camera attitude. The main twin keeps the nominal attitude.

Image quality, same 17 photos (rectified 0.5 m patches, medians): contrast sd real 51, twin 55, twin + realistic
camera 43; gradient energy 114 / 116 / 81; fine-detail noise 5.9 / 7.4 / 4.4. The ideal twin is already as
sharp and contrasted as the real photos. The realistic camera makes it duller than reality.

## Sim-to-real: same route, same code (seeds 0–4)

**Level 1** (each test photo matched on its own, consensus ZNCC + XFeat within 4 m, 1,125 attempts and 3,375
decoys per variant). Source: `x9_tuniu_twin/summary_l1.csv`.

| Variant | Accepted (mean, min–max per seed) | Wrong accepted (> 10 m) | Decoys accepted | Median / p90 error of accepted fixes |
|---|---|---|---|---|
| Real photos (re-run) | 31.5 % (28.4–34.7) | 0 | 0 | 2.47 / 4.02 m |
| `twin` | 37.9 % (35.1–41.8) | 0 | 1 | 1.94 / 3.99 m |
| `twin_rc` | 33.4 % (30.7–36.9) | 1 | 2 | 2.05 / 4.14 m |
| `twin_att_rc` (diagnostic) | 33.0 % (31.6–34.7) | 2 | 1 | 2.28 / 4.51 m |

**Level 2** (closed loop, GNSS-free after photo 46, `dji` heading, `dem_lifted` ground, 225 test photos per
seed). "Lock lost" = the truth leaves the filter's search window at least once. Source: `x9_tuniu_twin/summary_l2.csv`.

| Variant | Median / p90 / max error | Photos < 10 m | Accepted fixes | Wrong fixes | Seeds never losing lock | Longest gap without a fix | Without map fixes, median |
|---|---|---|---|---|---|---|---|
| Real photos | 3.15 / 17.9 / 42.5 m | 74.1 % | 36.5 % | 0 | 5/5 | 62 s | 52.5 m |
| `twin` | 2.72 / 22.5 / 53.4 m | 77.8 % | 42.0 % | 0 | 2/5 | 64 s | 52.0 m |
| `twin_rc` | 2.81 / 24.0 / 59.4 m | 73.5 % | 36.7 % | 0 | 2/5 | 67 s | 50.1 m |
| `twin_att_rc` (diagnostic) | 3.07 / 25.0 / 53.4 m | 73.4 % | 34.3 % | 0 | 2/5 | 67 s | 42.8 m |

The real-photo row is seeds 0–4 of `x_tuniu_l2/runs/main` (the 20-seed figure quoted elsewhere is 3.25 m).

**Criteria B, as written before the runs:**

| Criterion | Allowed range | `twin` | `twin_rc` | `twin_att_rc` |
|---|---|---|---|---|
| 1. Level 2 median within a factor 1.5 of real (3.15 m) | 2.10–4.73 m | 2.72 ✓ | 2.81 ✓ | 3.07 ✓ |
| 2. Level 2 accepted fixes within ±10 points of real (36.5 %) | 26.5–46.5 % | 42.0 ✓ | 36.7 ✓ | 34.3 ✓ |
| 3. Level 1 acceptance within ±10 points of real (31.5 %) | 21.5–41.5 % | 37.9 ✓ | 33.4 ✓ | 33.0 ✓ |
| 4. Level 1 median error within a factor 1.5 of real (2.47 m) | 1.65–3.71 m | 1.94 ✓ | 2.05 ✓ | 2.28 ✓ |

What the criteria do not cover: the twin loses lock in 3 of 5 seeds where the real photos never do, and its
worst errors are larger (p90 22–25 m against 18 m). The realistic camera brings acceptance back to the real
level (37.9 % → 33.4 % in Level 1, 42.0 % → 36.7 % in Level 2) but adds one wrong Level 1 fix.

**Rule C, baseline for the sweep:** |log(median / real)| is 0.147 for `twin` and 0.114 for `twin_rc`; the two
medians are also within 5 % of each other, and the acceptance tie-break gives the same answer (36.7 % is closer to
36.5 % than 42.0 %). Baseline: `twin_rc`, so every sweep variant uses the realistic camera.

## Factor sweep (closed loop, seeds 0–2, realistic camera)

One factor changed at a time from the baseline (30° tilt, a photo every 2.8 s, real altitude, quarter-size
renders). Each variant has its own pre-cut calibration. No variant accepted a wrong fix (> 10 m).

| Variant | Change | Median / p90 / max error | Accepted fixes | Fixes per minute | Seeds never losing lock | Longest gap without a fix | Without map fixes, median | Rule D |
|---|---|---|---|---|---|---|---|---|
| `twin_rc` (baseline) | – | 2.87 / 18.9 / 46.4 m | 38.5 % | 8.3 | 2/3 | 64 s | 49.6 m | – |
| `dt05_rc` | a photo every 0.5 s | 2.05 / 7.9 / 21.9 m | 37.0 % | 44.4 | 3/3 | 54 s | 21.0 m | **helps** (−29 %) |
| `dt1_rc` | a photo every 1 s | 2.32 / 10.5 / 26.3 m | 36.2 % | 21.7 | 3/3 | 62 s | 18.9 m | no clear effect (−19 %) |
| `alt50_rc` | 50 m higher | 2.30 / 14.5 / 40.9 m | 46.4 % | 9.9 | 3/3 | 62 s | 62.2 m | no clear effect (−20 %) |
| `half_rc` | half-size images | 2.71 / 21.3 / 55.5 m | 38.8 % | 8.3 | 2/3 | 64 s | 48.9 m | no clear effect (−6 %) |
| `tilt0_rc` | camera straight down | 19.0 / 50.6 / 95.0 m | 18.5 % | 4.0 | 0/3 | 101 s | 80.5 m | **hurts** |
| `tilt15_rc` | 15° from vertical | 102.4 / 162.9 / 194.5 m | 11.6 % | 2.5 | 0/3 | 484 s | 89.4 m | **hurts** |
| `tilt45_rc` | 45° from vertical | 14.9 / 75.2 / 177.1 m | 7.4 % | 1.6 | 2/3 | 358 s | 54.7 m | **hurts** |

Source: `x9_tuniu_twin/summary_l2.csv` (rows with seed set 0-1-2).

- **Photo rate.** The share of photos giving a fix stays near 37 %, so more photos mean 2.6× (1 s) to 5.4×
  (0.5 s) more fixes per minute. The forest gaps barely shorten (54–62 s against 64 s), but the drift inside them
  is smaller: the worst error halves (22–26 m against 46 m). The poses between real photos are interpolated, so
  turns are smoother than in reality (see Limits).
- **Altitude +50 m.** More ground per photo: acceptance rises to 46 %, the median falls by 20 %.
- **Image size.** No effect, as written in the pre-registration: the pipeline decodes every image to the same
  pixel count before matching.
- **Tilt.** At 15°, in all three seeds the estimate leaves the search window at photo 104–105 and the gate then
  rejects every later fix (seed 0: on photos 97–103 the odometry measures steps of 9–17 m instead of about 20 m).
  At 0°, the pre-cut odometry is already noisier (median absolute deviation 0.86 m against 0.14 m) and no
  20-photo drift window could be fitted. INFERENCE: the twin was only checked against real photos at 30°; at 45° it
  shows more of the smeared vertical surfaces of the 2.5D mesh, at 0° a smaller footprint. These rows say that the
  pipeline as tuned at 30° does not transfer to other tilts on this twin, not how a real camera would do.

## Limits

- **2.5D mesh**: no facades, smeared tree crowns, no overhangs. Forest photos render as streaks. Tilted
  cameras (45°) look at more of the smeared vertical surfaces than the real camera would.
- **Baked September lighting**: one texture mosaic with the September sun and shadows. No time of day, no
  season. The twin's map/texture gap is 3 months (Sept → Dec map); the real photos' gap is 8 months
  (April → Dec).
- **Pinhole camera**: the twin has no lens distortion, so it sees a narrower field than the real
  barrel-distorted lens (on the rectified patches the twin covers about 10–15 % less ground).
- **No motion blur, no rolling shutter**. The realistic camera adds blur, noise, haze, vignetting and cloud
  shadows. On this flight it overshoots: it makes the images duller than the real ones.
- **Attitude**: the twin camera follows the DJI angles exactly, so it has no attitude error beyond what the
  boresight absorbs. The diagnostic `twin_att` shows that the real attitude errors cannot be separated from
  mesh distortions with these data.
- **Interpolated poses** (1 s, 0.5 s): straight-line positions and linearly interpolated attitude between
  the real 2.8 s photos; turns are smoother than in reality.
- **One flight, one site, few seeds** (5 for sim-to-real, 3 for the sweep). The barometer is simulated as in
  the real runs.

## Reproduce

```bash
# rendering (uv adds pyvista 0.49 / VTK 9.7; offscreen)
uv run --with pyvista python experiments/x9_tuniu_twin.py prepare            # 4096 px textures + MTL
uv run --with pyvista python experiments/x9_tuniu_twin.py texel
uv run --with pyvista python experiments/x9_tuniu_twin.py projtest
uv run --with pyvista python experiments/x9_tuniu_twin.py render twin --no-placement
mv data/processed/x9_tuniu_twin/fidelity/twin data/processed/x9_tuniu_twin/fidelity/twin_unplaced  # after:
.venv/bin/python experiments/x9_tuniu_twin.py fidelity --all --tag twin_unplaced   # (run before the mv instead)
.venv/bin/python experiments/x9_tuniu_twin.py placement                      # pre-cut legs only
uv run --with pyvista python experiments/x9_tuniu_twin.py render twin --force
.venv/bin/python experiments/x9_tuniu_twin.py fidelity --all && .venv/bin/python experiments/x9_tuniu_twin.py fidelity
.venv/bin/python experiments/x9_tuniu_twin.py attitude && uv run --with pyvista python experiments/x9_tuniu_twin.py render twin_att
for v in tilt0 tilt15 tilt45 alt50 dt1 dt05 half; do uv run --with pyvista python experiments/x9_tuniu_twin.py render $v; done
for v in twin twin_att tilt0 tilt15 tilt45 alt50 dt1 dt05 half; do .venv/bin/python experiments/x9_tuniu_twin.py degrade $v; done
.venv/bin/python experiments/x9_tuniu_twin.py quality --variants twin twin_rc
# navigation (2 worker processes each)
for v in real twin twin_rc twin_att_rc; do .venv/bin/python experiments/x9_tuniu_twin_nav.py l1 $v --seeds 0 1 2 3 4; done
for v in twin twin_rc twin_att_rc; do .venv/bin/python experiments/x9_tuniu_twin_nav.py l2 $v --seeds 0 1 2 3 4; done
for v in tilt0_rc tilt15_rc tilt45_rc alt50_rc half_rc dt1_rc dt05_rc; do .venv/bin/python experiments/x9_tuniu_twin_nav.py l2 $v --seeds 0 1 2; done
.venv/bin/python experiments/x9_tuniu_twin_nav.py summary                     # -> summary_l1.csv, summary_l2.csv
```

# Visual-inertial relative navigation on Mid-Air

After GNSS is lost, this folder estimates the drone's motion from the IMU, two cameras and a barometer, and measures the drift against ground truth. It is **continuous relative navigation**. Its error still grows, only more slowly than the IMU's. Absolute localization (map matching, sun heading, signals of opportunity) is a later stage and is not part of this folder.

There are two estimators:

1. **ESKF** (current): an error-state Kalman filter that estimates position, velocity, attitude and both IMU biases. It takes visual and barometer measurements as filter updates. Script: `scripts/run_midair_eskf.py`.
2. **Complementary attitude correction** (superseded, kept as a comparator): pulls the attitude towards the camera's relative rotation and estimates no biases. Script: `scripts/run_midair_vio.py`.

The IMU-only baseline in [../baseline](../baseline) is unchanged.

## Run

From the repository root, with the data in `data/MidAir`:

```bash
python -m pytest                                                      # all tests (baseline and vio)
python vio/scripts/tune_eskf.py --data-root data/MidAir               # tuning on sunny/trajectory_0001 only
python vio/scripts/run_midair_eskf.py --data-root data/MidAir         # 5 filters x 4 flights
python vio/scripts/run_midair_eskf.py --data-root data/MidAir --inject-bias   # same, with known IMU biases added
```

The default flights are sunny 0000 and 0001 and cloudy 3000 and 3001; choose others with `--flights sunny:0 cloudy:3000`. The first run tracks features in about 40 s per camera and flight, then caches the tracks in `outputs/midair_eskf/cache/`.

Results (not committed) go to `outputs/midair_eskf/`:

- `summary.md` and `summary.json`, plus the same with `_injected`.
- `tuning.json`.
- Per flight: `metrics.json`, `trajectory.png`, `position_error.png`, `attitude_error.png`, `bias.png`, `nis.png` and `nees.png`.

## What was found on `main` and reused

`origin/main` (10 commits ahead of the local `main`) holds the team's Friday-night experiments in `docs/findings.md` and `experiments/`:

- **The Mid-Air gyroscope reports rates around the world axes.** Confirmed independently on 84 flights. The filter supports this directly: see below.
- **IMU error statistics** (findings section 2.3): gyro noise 0.021 rad/s per sample, offset about 0.001 rad/s that changes by about 0.003 rad/s within a flight; accelerometer 0.04 m/s², offset 0.025 m/s², change 0.04 m/s². These set the filter's process noise and bias priors.
- **Camera speed with a barometer on Mid-Air** (`experiments/g_midair_scale_check.py`) gave 37 % and 161 % speed error. The drone flies about 15 m above hilly ground, so the barometer misses the terrain. The same effect appears here. The flow's direction is accurate to 0.5 deg, but its magnitude is off by a factor of 1.1 to 4 wherever the terrain changes. The flow measurement was built around that (see below).
- **`mid-air-baseline-fix`**: only the scipy 1.18 compatibility line for the synthetic circle was new. It is ported into `baseline/src/data/synthetic.py`, and results are unchanged.
- **The ALTO work** (position fixes on a real helicopter flight) belongs to the later absolute-localization stage. No branch contained a Kalman filter.

## The Mid-Air cameras

Checked on the downloaded Kite_training files and calibrated once, offline, on sunny/trajectory_0000. No estimator run uses ground truth after the cutoff.

| | |
|---|---|
| Streams downloaded | `color_down` and `color_left` for sunny 0000, 0001 and cloudy 3000, 3001 |
| Files | `<condition>/<stream>/<trajectory>/frames.zip`, read directly from the zip, 1024 x 1024 RGB, processed at 512 x 512 |
| Rate and sync | 25 Hz. Frame `i` is taken at IMU sample `4 i` (offset 0, measured) |
| Intrinsics | 90 deg field of view, so f = 512 px at full size (fitted; not in the files) |
| Mounting `R_bc` | `color_left` faces forward: `[[0,0,1],[1,0,0],[0,1,0]]`. `color_down`, image top = forward: `[[0,-1,0],[1,0,0],[0,0,1]]` |
| Lever arm | `local_position = (0, 0, 0)` for both cameras, so no lever-arm term |
| GNSS | `gps/velocity`, NED, 1 Hz, about 1 mm/s from the truth. Used only before the cutoff |

## ESKF

**Nominal state:** `x = [p, v, q, b_a, b_g]`, with position and velocity in NED, `q` rotating body to world, `b_a` in body axes and `b_g` in the gyroscope's axes. **Error state** (15): `[δp, δv, δθ, δb_a, δb_g]`. The attitude error sits on the world side, `R = Exp(δθ) R̂`, which suits Mid-Air's world-frame gyro. A 3-state clone `δθ_c` of the keyframe attitude is added for the forward camera ([estimation/eskf.py](estimation/eskf.py)).

**Prediction** at 100 Hz. This is the same integration as the baseline, with the biases removed:

```
ω = ½(ω_m,k + ω_m,k+1) − b_g        R_k+1 = Exp(ω dt) R_k           (world gyro; body gyro: R_k Exp(ω dt))
a_k = R_k (a_m,k − b_a) + g          v_k+1 = v_k + ½(a_k + a_k+1) dt   p_k+1 = p_k + ½(v_k + v_k+1) dt

δṗ = δv     δv̇ = −[R f]× δθ − R δb_a − R n_a     δθ̇ = −G δb_g − G n_g  (G = I world gyro, R body gyro)
δḃ_a = n_ba  δḃ_g = n_bg              Φ = I + A dt + (A dt)²/2,   Q_d = diag(σ_a², σ_g², σ_ba², σ_bg²) dt
```

With no updates the ESKF reproduces the IMU-only baseline to 5e-12 m on Mid-Air. Quaternions are renormalised by scipy at every step. Updates use the Joseph form.

**Forward camera: relative rotation with stochastic cloning.** At each keyframe the filter clones the attitude, and the clone's covariance is kept correlated with the state. At the end of a keyframe span (25 frames, or fewer when tracks are lost) the essential matrix gives the relative rotation `C`, mapped to the body frame with `R_bc`. The update is:

```
r = Log(Ĉᵀ C),  Ĉ = R̂_cloneᵀ R̂_now,  r ≈ R̂_nowᵀ (δθ_now − δθ_clone) + n,   σ = 0.2 deg per axis
```

The camera never claims an absolute attitude. The information comes from the difference between camera and gyro over the span, so it reaches `b_g` through the `δθ`–`δb_g` cross-covariance. Absolute yaw stays unobservable. The bias of the yaw rate does become observable.

**Down camera: flow velocity** ([vision/optical_flow.py](vision/optical_flow.py)):

1. **Tracking and quality checks.** Consecutive frames are tracked, and a homography is fitted with RANSAC, keeping its inliers (the points on the dominant ground plane). A pair is skipped if it has fewer than 40 tracks, fewer than 50 % plane inliers, or a velocity fit with more than 1 px rms residual.
2. **Derotation** uses the filter's own attitudes, which already integrate `ω_m − b̂_g`, never the raw gyro. Rays, the ground normal and the translation are expressed in camera b axes, the frame of the measurement model.
3. **Velocity.** With a ground plane at height `h`, each feature's depth is `h / (n·x)`, which accounts for tilt. The camera translation follows by least squares, giving the lateral camera velocity `z = S R_bcᵀ R̂ᵀ v`.
4. **Height.** `h = h₀ + (baro(t) − baro_ref)`. `h₀` is learned in the 2 s before the cutoff, from flow and the GNSS velocity. The barometer is **simulated**: ground-truth altitude plus 0.3 m white noise and a 0.05 m/√s drifting offset ([sensors/simulated.py](sensors/simulated.py)). The filter sees only that stream.
5. **Noise model.** A wrong height scales the velocity along itself, so the noise is anisotropic: `R = R_fit + σ_min² I + (|v| σ_rel)² uuᵀ + (|v| σ_dir)² (I − uuᵀ)`, with `u` the measured direction, `σ_rel = 1.0` and `σ_dir = 2 deg`. The direction is trusted; the magnitude barely is.
6. **Rate and gating.** Updates run at 5 Hz, every 5th pair, because consecutive pairs have correlated errors and 25 Hz made the filter overconfident. Every update passes a 99 % Mahalanobis gate.

**Barometer: altitude update** at 5 Hz, `z = up·p`, with `σ = √(0.3² + 0.05² t)`. Without it, the down-camera updates corrupt the vertical channel. The lateral camera axes see vertical velocity only through a tilt of a few milliradians, but the vertical velocity is so uncertain that this tiny coupling earned a large gain. The accelerometer z-bias then absorbed the horizontal residuals; it reached −0.22 m/s² on a synthetic flight whose true value was +0.03. The platform in `docs/PLAN.md` carries a barometer, so every visual configuration is compared against IMU + barometer.

**Tuning** ([scripts/tune_eskf.py](scripts/tune_eskf.py)): only on sunny/trajectory_0001, with the rules fixed in the script. The rotation σ is the smallest value whose NIS stays at most 1.2 × 3. The flow settings are those with the lowest ATE among grid points with NEES below 50.

## Results: four-way ablation

GNSS is lost at 5 s, followed by about 83 s without GNSS. "Final" is the position error at the end of the flight. ATE is the position RMSE with no alignment (the start is shared at the cutoff). RPE is the translation error over 10 s windows. **0001 is the tuning flight**; 0000, 3000 and 3001 are held out.

| Flight | IMU only | IMU + baro | + forward rotation | + down flow | **+ both** |
|---|---|---|---|---|---|
| **Final position error (m)** | | | | | |
| sunny 0000 | 684 | 114 | 108 | 98 | **12.7** |
| sunny 0001 (tuning) | 381 | 145 | 85 | 8.1 | **12.6** |
| cloudy 3000 | 912 | 441 | 278 | 156 | **33.1** |
| cloudy 3001 | 338 | 116 | 108 | 22.4 | **17.3** |
| **ATE (m)** | | | | | |
| sunny 0000 | 223 | 38.3 | 31.1 | 34.2 | **15.1** |
| sunny 0001 (tuning) | 210 | 68.8 | 44.6 | 14.5 | **4.4** |
| cloudy 3000 | 371 | 208 | 148 | 120 | **40.6** |
| cloudy 3001 | 137 | 55.9 | 45.7 | 25.0 | **15.1** |
| **RPE over 10 s (m)** | | | | | |
| sunny 0000 | 115 | 40.3 | 33.2 | 26.6 | **12.1** |
| cloudy 3000 | 182 | 67.8 | 44.6 | 43.6 | **21.4** |
| cloudy 3001 | 48.4 | 33.0 | 23.5 | 21.3 | **10.9** |
| **Final attitude error (deg)** | | | | | |
| sunny 0000 | 6.42 | 3.26 | 3.34 | 22.4 | **1.92** |
| sunny 0001 (tuning) | 0.58 | 1.48 | 1.79 | 1.54 | 0.91 |
| cloudy 3000 | 20.3 | 15.3 | 6.80 | 17.4 | **5.25** |
| cloudy 3001 | 0.92 | 1.51 | 3.86 | 9.67 | 4.20 |

Reading:

- **Both cameras together are clearly best.** On the three held-out flights the final error is 13–33 m, against 338–912 m for IMU only and 114–441 m for IMU + barometer. That is 4–9× lower than IMU + barometer and 20–50× lower than IMU only.
- **The forward camera alone brings little in position (5–37 % over IMU + barometer), but it controls attitude.** On 3000 it takes the attitude error from 15.3 to 6.8 deg.
- **The down camera alone helps position a lot but loses attitude:** 9.7–22 deg, mostly yaw, which flow cannot observe. Its covariance is then badly overconfident (NEES up to 226). Flow alone should not be deployed without the forward camera.
- **On flights where the gyro barely drifts (0001, 3001), any visual update adds about 1–3 deg of attitude noise.** Position still improves.
- **Position error still grows.** On 3000 it is 33 m after 83 s with both cameras. Absolute fixes remain necessary.

## Results: bias recovery with injected biases

Known biases are added to the IMU: `b_g = (3.0, −2.0, 2.5) mrad/s` in the gyro's (world) axes and `b_a = (0.06, −0.05, 0.04) m/s²` in the body frame. Mid-Air already contains its own biases, so the truth is "injected + native". The native part is measured from ground truth (10 s moving average of IMU minus true motion; evaluation only). The table gives the error of the estimate averaged over the last 20 s, with the error of the zero estimate in brackets.

| Flight | Gyro bias error (mrad/s): IMU + baro | + forward | + down | + both | Accel bias error (m/s²): + forward | + down | + both |
|---|---|---|---|---|---|---|---|
| sunny 0000 | 4.02 [3.53] | 1.78 | 1.82 | **1.33** | 0.115 [0.141] | 0.054 | 0.093 |
| sunny 0001 (tuning) | 4.26 [4.08] | 2.33 | 4.88 | 2.66 | 0.050 [0.060] | 0.084 | 0.048 |
| cloudy 3000 | 5.17 [7.33] | 3.87 | 6.80 | **3.91** | 0.132 [0.080] | 0.055 | 0.054 |
| cloudy 3001 | 3.25 [4.06] | 1.65 | 2.74 | **1.98** | 0.197 [0.237] | 0.194 | 0.179 |

With injected biases the final position error with both cameras is 15.7, 30.7 and 19.2 m on 0000, 3000 and 3001. IMU only reaches 1,340–2,807 m, and IMU + barometer 193–250 m.

- **Gyro bias: the forward camera gives real observability.** The error falls to 40–60 % of the zero estimate with the forward camera or both cameras. Only the forward camera estimates the yaw-axis bias: with the barometer alone the z component stays at 0, because gravity and altitude cannot see it.
- **Accelerometer bias: partial and inconsistent, as expected.** It improves modestly on 0000, 3000 and 3001 (by 25–35 % with both cameras), not at all on 3001 with the forward camera alone, and worse on 3000 with the forward camera alone. Accelerometer bias is entangled with tilt (`δa ≈ g δθ`), so this is reported as an experimental result, not as convergence.

## Filter consistency (NIS, NEES)

Native runs, accepted updates. A consistent filter has mean NIS equal to the measurement dimension and mean NEES equal to 9 (position, velocity, attitude).

| | Rotation NIS (dof 3) | Flow NIS (dof 2) | Baro NIS (dof 1) | NEES9, + both | NEES9, + down only |
|---|---|---|---|---|---|
| sunny 0000 | 2.45 | 0.45 | 0.53 | 16.3 | 37.9 |
| sunny 0001 (tuning) | 3.86 | 0.78 | 0.53 | 14.9 | 28.9 |
| cloudy 3000 | 3.52 | 0.76 | 0.56 | 41.9 | 226 |
| cloudy 3001 | 3.86 | 0.94 | 0.53 | 33.0 | 57.5 |

- **Rotation NIS is close to 3:** the 0.2 deg noise is about right.
- **Flow and barometer NIS are about 0.5–0.9:** both noise models are conservative, on purpose for flow, whose magnitude errors are not white.
- **NEES with both cameras is 15–42, against an expected 9:** the full filter is 2–5× overconfident. The likely cause is correlated (non-white) measurement errors: terrain-induced flow scale, and keyframe-shared rotation noise. Flow alone is much worse.
- **The trajectory errors are good, but the covariance should not yet be used as an integrity bound.** Inflating `Q` or applying the flow update less often would be the next consistency step.

Update acceptance with both cameras: forward rotation 76–93 % (the rest fail the gate or cheirality), and down flow 71–76 % (rejected for fit residual, plane inliers or tracks). After any rejection the filter continues on the IMU.

## Previous result: complementary attitude correction

The earlier fusion (`run_midair_vio.py`) moved the attitude 5 % towards the camera per frame and estimated no biases. It cut the attitude error by 25–29 % on 0000 and 3000 and made it worse on 0001 and 3001. With perfect attitude (oracle) the position error fell by 80–90 %, which is why the ESKF targets the gyro bias. The camera calibration tables and the per-span accuracy analysis that chose the forward camera still apply: over 40 ms the gyro is about 10× more precise than vision, and vision only pays off over spans of seconds.

## Limits

- **This is not absolute localization.** Position error grows without bound; 13–33 m after 83 s here.
- **The barometer is simulated from ground truth.** The height above ground is the barometer altitude plus a height learned under GNSS. It drifts with the terrain, which is why the flow magnitude is barely trusted.
- **Tuning used one flight** (0001). Three held-out flights, two conditions, one environment (Kite) is little evidence. The fog and season flights have no downloaded images.
- **The filter is overconfident with visual updates** (NEES 2–5× too high).
- **Each forward-camera keyframe span gives only one relative-rotation update.** Measurements inside the span are discarded to avoid double-counting.
- **The flow model assumes locally flat ground below the camera,** and the plane-inlier check skips frames where the ground is not flat.
- **The prior gyro-bias σ of 0.003 rad/s gives `IMU + baro` visible horizontal jumps** when barometer updates move the tilt through the cross-covariance.
- **The raw angular-flow measurement** (height as a filter state, flow residual in rad/s) is the next step for the down camera. It would move the height uncertainty from `R` into the state.

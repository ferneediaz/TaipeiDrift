"""Simulation: what stereo, barometer and IMU can and cannot say about height, plus a sun-compass ablation.

Everything here is SYNTHETIC. A generator draws a flight, a terrain and sensor readings from declared
noise assumptions. Estimators read only the sensor readings (and, in one labelled ablation, a DEM sampled
at a simulated horizontal position). Truth is used only to score. No number written by this script is a
measurement of real hardware.

Part 1, height (vertical channel):
    state of the full filter: altitude z (above the datum), vertical speed, barometer offset b,
    accelerometer offset, terrain elevation h under the aircraft. Height above ground = z - h.
    baro reads z + b; stereo disparity reads f * B * cos(tilt) / (z - h); IMU drives z'' ; GNSS reads z
    until the cut. Without GNSS or a DEM, (z, b, h) -> (z + c, b - c, h + c) leaves every reading unchanged.

Part 2, heading (yaw channel): gyro yaw rate vs. a sun-vector sensor turned into a heading with an
    ephemeris and AHRS roll/pitch. Theoretical compass aid, not a reconstruction of any line sensor.

Run from the repository root:
    python experiments/n_sensor_fusion.py --help
    python experiments/n_sensor_fusion.py                 # 20 seeds, 600 s after the cut
    python experiments/n_sensor_fusion.py --quick         # 3 seeds, 240 s, smoke run
Outputs (default data/processed/sensor_fusion/): results.json, *.csv, *.png.
Exit code 1 if an invariant fails (listed in results.json).
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
import time
import warnings
from dataclasses import asdict, dataclass, fields
from datetime import date
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import lfilter

G = 9.80665
EARTH_RADIUS = 6_371_000.0


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------------------------------------------------------------------------------------------
# Declared assumptions. Generator-side and estimator-side settings are separate on purpose.
# ---------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class StereoRig:
    """Calibrated rig as the estimator knows it. The generator perturbs baseline and disparity offset."""

    focal_px: float = 1000.0  # about 1280 px wide at about 65 deg horizontal field of view
    baseline_m: float = 0.20
    rate_hz: float = 5.0
    d_min_px: float = 0.2  # reject d <= d_min: non-positive geometry or beyond usable range (fB/d_min = 1000 m)
    d_max_px: float = 64.0  # reject d > d_max: outside the disparity search range


@dataclass(frozen=True)
class VerticalSimModel:
    """GENERATOR ONLY. Declared assumptions, not calibrations of a real unit."""

    dt: float = 0.1  # IMU pre-integrated to 10 Hz; baro at 10 Hz
    speed_range: tuple = (12.0, 18.0)  # m/s ground speed, drawn per seed (estimator does not get it)
    agl_low: float = 40.0
    agl_high: float = 300.0
    # Nominal barometer = fit to the REAL Zurich log (experiments/s_zurich_vertical.py, structure function of
    # baro minus photogrammetry, 1-1200 s): white 0.30 m, random walk 0.112 m/sqrt(s), per-flight ramp 0.0024 m/s.
    # One tethered flight, one day, slow; the reference error is included, so this is an upper-side estimate.
    # The first draft used 0.4 m / 0.01 m/sqrt(s) / 0.002 m/s: p95 at 600 s 2.7 m vs 6.0 m measured.
    baro_noise: float = 0.30
    baro_offset_sigma: float = 20.0  # m, absolute offset (unknown sea-level pressure)
    baro_drift_nominal_sigma: float = 0.0024  # m/s
    baro_rw_nominal: float = 0.112  # m/sqrt(s)
    baro_drift_strong: tuple = (0.015, 0.025)  # m/s, random sign: stress case (front + thermal transient)
    baro_rw_strong: float = 0.15  # must stay above the nominal (Zurich) random walk
    accel_noise: float = 0.05  # m/s^2 per 10 Hz sample after gravity removal, incl. vibration
    accel_bias_sigma: float = 0.05  # m/s^2 (Mid-Air median constant offset is 0.025, findings 2.3)
    accel_bias_rw: float = 0.001  # m/s^2/sqrt(s)
    gnss_white: float = 1.5  # m
    gnss_gm_sigma: float = 2.0  # m, time-correlated vertical GNSS error
    gnss_gm_tau: float = 60.0  # s
    gnss_rate_hz: float = 1.0
    tilt_noise_deg: float = 0.5  # AHRS roll/pitch white error
    tilt_bias_deg: float = 0.3  # AHRS roll/pitch constant error per flight
    baseline_rel_err: float = 0.005  # true baseline differs from the calibrated one by this (1 sigma)
    disp_offset_px: float = 0.05  # residual rectification offset per flight
    disp_noise_px: float = 0.20  # per-frame matching noise on textured ground
    outlier_rate: float = 0.01
    dropout_rate: float = 0.02
    loss_window: tuple = (0.35, 0.55)  # texture-loss window as fractions of the post-cut duration
    loss_dropout: float = 0.85
    loss_outlier_share: float = 0.7  # share of returned matches that are garbage inside the window
    matcher_range_px: tuple = (-1.0, 96.0)  # garbage disparities, may be <= 0 or beyond the search range
    dem_bias_sigma: float = 1.5  # Copernicus GLO-30 spec: absolute < 4 m LE90, relative < 2 m LE90 (slope <= 20 %)
    dem_corr_sigma: float = 1.5
    nav_scale_err_sigma: float = 0.03  # ablation only: horizontal error = 3 % of distance since the cut
    nav_rw: float = 1.0  # ablation only: m/sqrt(s)
    nav_precut_sigma: float = 3.0  # ablation only: GNSS horizontal error before the cut
    dem_rate_hz: float = 1.0


@dataclass(frozen=True)
class VerticalEstimatorConfig:
    """ESTIMATOR ONLY. One tuning for every scenario: the filter is never told the regime."""

    baro_sigma: float = 0.5
    baro_bias_rw: float = 0.12  # m/sqrt(s), from the real Zurich log (0.112), not from this generator
    accel_sigma: float = 0.07  # m/s^2 per 10 Hz sample
    accel_bias_rw: float = 0.002
    accel_bias_sigma0: float = 0.1
    manoeuvre_sigma: float = 0.5  # m/s^2, constant-velocity model when the IMU is not used
    vz_sigma0: float = 5.0
    gnss_sigma: float = 2.5
    disp_sigma_px: float = 0.25
    gate_sigma: float = 3.5
    nominal_speed: float = 15.0  # commanded cruise speed, a setting, not a truth time series
    # Terrain random walk: q_h = (V_nom * slope_sigma)^2. 0.2 ~ the RMS slope (0.165) of the declared hilly
    # terrain with margin. The first draft used 0.05: on hilly terrain the gate then locked stereo out for good
    # (the terrain state cannot follow, every disparity is gated). Swept by experiments/s_fusion_sweeps.py.
    terrain_slope_sigma: float = 0.2
    reacquire_frames: int = 10  # consecutive gated stereo frames (2 s at 5 Hz) before a terrain re-initialisation
    reacquire_mad_frac: float = 0.1
    uninformative_sigma: float = 1000.0
    agl_prior_mean: float = 100.0  # used only if no valid disparity in the first 2 s
    no_datum_baro_offset_sigma: float = 30.0  # prior on the absolute baro offset when no GNSS ever
    dem_sigma: float = 2.5
    dem_nav_sigma0: float = 3.0
    dem_nav_growth: float = 0.03  # assumed horizontal sigma = sigma0 + growth * V_nom * (t - t_cut)
    agl_min: float = 1.0


@dataclass(frozen=True)
class Scenario:
    terrain: str  # flat | changing
    altitude: str  # low | high
    texture: str  # good | loss
    baro: str  # nominal | strong

    @property
    def name(self) -> str:
        return f"{self.terrain}-{self.altitude}-{self.texture}-{self.baro}"


SCENARIOS = tuple(
    Scenario(*c)
    for c in itertools.product(("flat", "changing"), ("low", "high"), ("good", "loss"), ("nominal", "strong"))
)


@dataclass(frozen=True)
class Variant:
    name: str
    role: str
    use_imu: bool
    use_baro: bool
    stereo_before_cut: bool
    stereo_after_cut: bool
    use_gnss: bool
    terrain_after_cut: str  # random_walk | frozen (frozen = terrain assumed constant from the cut on)
    use_dem: bool = False


VARIANTS = (
    Variant("baro_only", "baseline", False, True, True, False, True, "frozen"),
    Variant("stereo_only", "baseline", False, False, True, True, True, "frozen"),
    Variant("imu_baro", "baseline", True, True, True, False, True, "frozen"),
    Variant("fusion_const_terrain", "control: deliberately wrong constant terrain", True, True, True, True, True, "frozen"),
    Variant("fusion_terrain_state", "proposed", True, True, True, True, True, "random_walk"),
    Variant("fusion_no_datum", "unobservability demo: no GNSS at any time", True, True, True, True, False, "random_walk"),
    Variant("fusion_known_dem", "ABLATION: DEM at simulated horizontal estimate", True, True, True, True, True, "random_walk", True),
)

TERRAIN_LAM = np.array([2500.0, 900.0, 300.0, 100.0, 25.0])  # m
TERRAIN_AMP = np.array([30.0, 15.0, 6.0, 2.0, 0.3])  # m
FOLLOW = TERRAIN_LAM >= 900.0  # low flight follows only the long wavelengths
MANOEUVRE_PERIOD = np.array([90.0, 37.0])  # s
MANOEUVRE_AMP = np.array([8.0, 3.0])  # m
DEM_ERR_LAM = np.array([150.0, 350.0, 700.0])


# ---------------------------------------------------------------------------------------------------
# Containers. Truth never reaches an estimator.
# ---------------------------------------------------------------------------------------------------


@dataclass
class VerticalTruth:
    """Generator output for scoring only."""

    z: np.ndarray  # (B, N) altitude, m, up
    h: np.ndarray  # terrain elevation at nadir
    agl: np.ndarray  # z - h
    baro_bias: np.ndarray
    texture_loss: np.ndarray  # bool


@dataclass
class VerticalMeasurements:
    """Everything an estimator may read."""

    t: np.ndarray  # (N,)
    dt: float
    k_cut: int  # explicit GNSS cut: first sample without GNSS
    accel_up: np.ndarray  # (B, N) vertical acceleration from the IMU after gravity removal, m/s^2
    baro_alt: np.ndarray  # (B, N) pressure altitude, m
    gnss_alt: np.ndarray  # (B, N) NaN without a fix; all NaN from k_cut on
    disparity_px: np.ndarray  # (B, N) NaN without a frame or a match
    cos_tilt_ahrs: np.ndarray  # (B, N) cos of camera off-nadir angle from AHRS roll and pitch


@dataclass
class KnownDemAblation:
    """LABELLED ABLATION INPUT: DEM (with map error) read at a SIMULATED horizontal estimate."""

    dem_h: np.ndarray  # (B, N) NaN except at DEM sample times
    dem_slope: np.ndarray  # (B, N) map slope at the same place


ARRAY_FIELDS = ("accel_up", "baro_alt", "gnss_alt", "disparity_px", "cos_tilt_ahrs")


def subset_measurements(meas: VerticalMeasurements, idx) -> VerticalMeasurements:
    return VerticalMeasurements(
        t=meas.t, dt=meas.dt, k_cut=meas.k_cut, **{f: getattr(meas, f)[idx].copy() for f in ARRAY_FIELDS}
    )


# ---------------------------------------------------------------------------------------------------
# Vertical generator
# ---------------------------------------------------------------------------------------------------


def _sines(x, amp, lam, phase, order=0):
    """sum_k amp_k sin(2 pi x / lam_k + phase_k), or its first/second derivative in x."""
    w = 2 * np.pi / lam
    arg = np.outer(x, w) + phase
    if order == 0:
        return np.sin(arg) @ amp
    if order == 1:
        return np.cos(arg) @ (amp * w)
    return -(np.sin(arg) @ (amp * w * w))


def _gen_vertical_one(sc: Scenario, rng, t, k_cut, duration, sim: VerticalSimModel, rig: StereoRig):
    n, dt = t.size, sim.dt
    k = np.arange(n)
    # All draws happen in the same order for every scenario: seed j gives common random numbers.
    speed = rng.uniform(*sim.speed_range)
    h0 = rng.uniform(20.0, 200.0)
    amp_scale = rng.uniform(0.7, 1.3, TERRAIN_LAM.size)
    t_phase = rng.uniform(0, 2 * np.pi, TERRAIN_LAM.size)
    slope = rng.uniform(-0.01, 0.01)
    m_phase = rng.uniform(0, 2 * np.pi, 2)
    att_phase = rng.uniform(0, 2 * np.pi, 2)
    tilt_bias = np.radians(sim.tilt_bias_deg) * rng.standard_normal(2)
    tilt_noise = np.radians(sim.tilt_noise_deg) * rng.standard_normal((2, n))
    accel_bias0 = sim.accel_bias_sigma * rng.standard_normal()
    accel_bias_steps = sim.accel_bias_rw * np.sqrt(dt) * rng.standard_normal(n)
    accel_noise = sim.accel_noise * rng.standard_normal(n)
    baro_offset0 = sim.baro_offset_sigma * rng.standard_normal()
    drift_nominal = sim.baro_drift_nominal_sigma * rng.standard_normal()
    drift_strong = (1.0 if rng.random() < 0.5 else -1.0) * rng.uniform(*sim.baro_drift_strong)
    baro_rw_unit = np.sqrt(dt) * rng.standard_normal(n)
    baro_noise = sim.baro_noise * rng.standard_normal(n)
    gnss_white = sim.gnss_white * rng.standard_normal(n)
    gnss_drive = rng.standard_normal(n)
    baseline_true = rig.baseline_m * (1.0 + sim.baseline_rel_err * rng.standard_normal())
    disp_offset = sim.disp_offset_px * rng.standard_normal()
    disp_noise = sim.disp_noise_px * rng.standard_normal(n)
    u_drop, u_out = rng.random(n), rng.random(n)
    outlier_val = rng.uniform(*sim.matcher_range_px, n)
    dem_bias = sim.dem_bias_sigma * rng.standard_normal()
    dem_err_amp = np.full(DEM_ERR_LAM.size, sim.dem_corr_sigma * np.sqrt(2.0 / DEM_ERR_LAM.size))
    dem_phase = rng.uniform(0, 2 * np.pi, DEM_ERR_LAM.size)
    nav_scale = sim.nav_scale_err_sigma * rng.standard_normal()
    nav_steps = sim.nav_rw * np.sqrt(dt) * rng.standard_normal(n)
    nav_pre = sim.nav_precut_sigma * rng.standard_normal(n)

    changing = sc.terrain == "changing"
    amp = TERRAIN_AMP * amp_scale * (1.0 if changing else 0.0)
    amp[-1] = TERRAIN_AMP[-1] * amp_scale[-1]  # 0.3 m roughness everywhere
    slope = slope if changing else 0.0

    def terrain(x, a, order=0):
        if order == 0:
            base = h0 + slope * x
        elif order == 1:
            base = np.full_like(x, slope)
        else:
            base = np.zeros_like(x)
        return base + _sines(x, a, TERRAIN_LAM, t_phase, order)

    s = speed * t
    h = terrain(s, amp)
    man = [_sines(t, MANOEUVRE_AMP, MANOEUVRE_PERIOD, m_phase, o) for o in (0, 1, 2)]
    if sc.altitude == "low":
        amp_f = np.where(FOLLOW, amp, 0.0)
        z = terrain(s, amp_f) + sim.agl_low + man[0]
        az = speed**2 * terrain(s, amp_f, 2) + man[2]
    else:
        z = h0 + sim.agl_high + man[0]
        az = man[2]
    agl = z - h

    # Attitude: forward-flight nose-down pitch with oscillation, small roll. Terrain is 1-D along track,
    # so the cross-track offset of the optical axis does not change the ground height (stated limit).
    pitch = -np.radians(5.0 + 2.0 * np.sin(2 * np.pi * t / 11.0 + att_phase[0]))
    roll = np.radians(3.0 * np.sin(2 * np.pi * t / 17.0 + att_phase[1]))
    cos_tilt = np.cos(pitch) * np.cos(roll)
    s_hit = s + agl * np.tan(pitch)
    s_hit = s + (z - terrain(s_hit, amp)) * np.tan(pitch)  # one fixed-point step of the ray-terrain hit
    depth = (z - terrain(s_hit, amp)) / cos_tilt  # range along the optical axis

    frame = (k % max(1, int(round(1.0 / (rig.rate_hz * dt))))) == 0
    t_rel = t - t[k_cut]
    loss = (sc.texture == "loss") & (t_rel >= sim.loss_window[0] * duration) & (t_rel < sim.loss_window[1] * duration)
    drop_p = np.where(loss, sim.loss_dropout, sim.dropout_rate)
    out_p = np.where(loss, sim.loss_outlier_share, sim.outlier_rate)
    disp = rig.focal_px * baseline_true / depth + disp_offset + disp_noise
    disp = np.where(u_out < out_p, outlier_val, disp)
    disp = np.where((u_drop < drop_p) | ~frame, np.nan, disp)

    pitch_m = pitch + tilt_bias[0] + tilt_noise[0]
    roll_m = roll + tilt_bias[1] + tilt_noise[1]
    accel = az + accel_bias0 + np.cumsum(accel_bias_steps) + accel_noise

    drift, rw = (drift_nominal, sim.baro_rw_nominal) if sc.baro == "nominal" else (drift_strong, sim.baro_rw_strong)
    baro_bias = baro_offset0 + drift * t + rw * np.cumsum(baro_rw_unit)
    baro = z + baro_bias + baro_noise

    phi = np.exp(-dt / sim.gnss_gm_tau)
    drive = sim.gnss_gm_sigma * np.sqrt(1 - phi**2) * gnss_drive
    drive[0] = sim.gnss_gm_sigma * gnss_drive[0]
    gm = lfilter([1.0], [1.0, -phi], drive)
    gnss_ok = (k % max(1, int(round(1.0 / (sim.gnss_rate_hz * dt)))) == 0) & (k < k_cut)
    gnss = np.where(gnss_ok, z + gm + gnss_white, np.nan)

    # Known-DEM ablation: the map smooths away the 25 m roughness, damps 100 m relief, adds DEM error.
    amp_dem = amp.copy()
    amp_dem[-1] = 0.0
    amp_dem[-2] *= 0.7
    since_cut = np.clip(s - s[k_cut], 0.0, None)
    nav_err = np.where(k < k_cut, nav_pre, nav_scale * since_cut + np.cumsum(np.where(k >= k_cut, nav_steps, 0.0)))
    s_hat = s + nav_err
    dem_ok = (k % max(1, int(round(1.0 / (sim.dem_rate_hz * dt))))) == 0
    dem_h = terrain(s_hat, amp_dem) + dem_bias + _sines(s_hat, dem_err_amp, DEM_ERR_LAM, dem_phase)
    dem_slope = terrain(s_hat, amp_dem, 1) + _sines(s_hat, dem_err_amp, DEM_ERR_LAM, dem_phase, 1)

    return dict(
        truth=dict(z=z, h=h, agl=agl, baro_bias=baro_bias, texture_loss=loss),
        meas=dict(
            accel_up=accel,
            baro_alt=baro,
            gnss_alt=gnss,
            disparity_px=disp,
            cos_tilt_ahrs=np.cos(pitch_m) * np.cos(roll_m),
        ),
        dem=dict(dem_h=np.where(dem_ok, dem_h, np.nan), dem_slope=np.where(dem_ok, dem_slope, np.nan)),
    )


def generate_vertical(seeds, seed0, t, k_cut, duration, sim, rig):
    parts, elem_scn, elem_seed = [], [], []
    for si, sc in enumerate(SCENARIOS):
        for j in range(seeds):
            rng = np.random.default_rng([seed0, 1, j])
            parts.append(_gen_vertical_one(sc, rng, t, k_cut, duration, sim, rig))
            elem_scn.append(si)
            elem_seed.append(j)

    def stack(group, key):
        return np.stack([p[group][key] for p in parts])

    truth = VerticalTruth(**{f.name: stack("truth", f.name) for f in fields(VerticalTruth)})
    meas = VerticalMeasurements(t=t, dt=sim.dt, k_cut=k_cut, **{f: stack("meas", f) for f in ARRAY_FIELDS})
    dem = KnownDemAblation(dem_h=stack("dem", "dem_h"), dem_slope=stack("dem", "dem_slope"))
    return truth, meas, dem, np.array(elem_scn), np.array(elem_seed)


# ---------------------------------------------------------------------------------------------------
# Vertical estimators: one batched EKF, variants differ by the sensors they read and the terrain model.
# State x = [z, vz, b_baro, b_accel, h_terrain].
# ---------------------------------------------------------------------------------------------------


def _update(x, P, y, hx, H, R, gate=None):
    """Batched scalar EKF update in place. Returns (accepted, gated) masks."""
    valid = np.isfinite(y) & np.isfinite(hx)
    nu = np.where(valid, y - np.where(np.isfinite(hx), hx, 0.0), 0.0)
    PHt = np.einsum("bij,bj->bi", P, H)
    S = np.einsum("bi,bi->b", H, PHt) + R
    ok = valid & (nu * nu <= gate * gate * S) if gate is not None else valid
    K = PHt / S[:, None]
    w = ok.astype(float)
    x += K * (nu * w)[:, None]
    P -= w[:, None, None] * (K[:, :, None] * PHt[:, None, :])
    return ok, valid & ~ok


def _stereo_valid(d, rig: StereoRig):
    return np.isfinite(d) & (d > rig.d_min_px) & (d <= rig.d_max_px)


def _initial_state(meas: VerticalMeasurements, rig, cfg, v: Variant):
    """Built from the first samples only. No truth, no re-initialisation later."""
    B = meas.baro_alt.shape[0]
    n0 = max(1, int(round(2.0 / meas.dt)))
    d = meas.disparity_px[:, :n0]
    ok = _stereo_valid(d, rig)
    agl = np.where(ok, rig.focal_px * rig.baseline_m * meas.cos_tilt_ahrs[:, :n0] / np.where(ok, d, 1.0), np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        agl0 = np.nanmedian(agl, axis=1)
    has_st = np.isfinite(agl0)
    agl0 = np.where(has_st, agl0, cfg.agl_prior_mean)
    # Height above ground known to ~the single-frame stereo sigma (or a vague prior), altitude unknown.
    # h = z - agl0 with agl0 independent of z, so cov(z, h) = var(z): without this correlation a later
    # GNSS or baro update moves z alone and the AGL estimate can start negative, which disables stereo.
    sig_agl0 = np.where(has_st, np.maximum(agl0 * agl0 * cfg.disp_sigma_px / (rig.focal_px * rig.baseline_m), 1.0),
                        cfg.agl_prior_mean)
    x = np.zeros((B, 5))
    x[:, 0] = meas.baro_alt[:, 0]
    x[:, 4] = x[:, 0] - agl0
    P = np.zeros((B, 5, 5))
    big = cfg.uninformative_sigma**2
    P[:, 0, 0] = big
    P[:, 1, 1] = cfg.vz_sigma0**2
    P[:, 2, 2] = big if v.use_gnss else cfg.no_datum_baro_offset_sigma**2
    P[:, 3, 3] = cfg.accel_bias_sigma0**2
    P[:, 4, 4] = big + sig_agl0**2
    P[:, 0, 4] = P[:, 4, 0] = big
    return x, P


def run_vertical_variant(meas: VerticalMeasurements, rig: StereoRig, cfg: VerticalEstimatorConfig, v: Variant,
                         dem: KnownDemAblation | None = None):
    if v.use_dem and dem is None:
        raise ValueError(f"{v.name} needs the labelled DEM ablation input")
    B, N = meas.baro_alt.shape
    dt, k_cut = meas.dt, meas.k_cut
    F = np.eye(5)
    F[0, 1] = dt
    if v.use_imu:
        F[0, 3] = -0.5 * dt * dt
        F[1, 3] = -dt
    g = np.array([0.5 * dt * dt, dt])
    sig_a = cfg.accel_sigma if v.use_imu else cfg.manoeuvre_sigma
    Q = np.zeros((5, 5))
    Q[:2, :2] = sig_a**2 * np.outer(g, g)
    Q[2, 2] = cfg.baro_bias_rw**2 * dt
    Q[3, 3] = cfg.accel_bias_rw**2 * dt if v.use_imu else 0.0
    q_h = (cfg.nominal_speed * cfg.terrain_slope_sigma) ** 2 * dt
    Q_pre, Q_post = Q.copy(), Q.copy()
    Q_pre[4, 4] = q_h
    Q_post[4, 4] = q_h if v.terrain_after_cut == "random_walk" else 0.0

    x, P = _initial_state(meas, rig, cfg, v)
    H_gnss = np.tile([1.0, 0, 0, 0, 0], (B, 1))
    H_baro = np.tile([1.0, 0, 1.0, 0, 0], (B, 1))
    H_dem = np.tile([0.0, 0, 0, 0, 1.0], (B, 1))
    fb = rig.focal_px * rig.baseline_m
    every_st = max(1, int(round(1.0 / (rig.rate_hz * dt))))

    out = {key: np.empty((B, N), np.float32) for key in ("z", "h", "b", "var_z", "var_agl")}
    counts = {key: np.zeros(B, int) for key in ("frames", "geometry_rejected", "gated", "accepted", "reacquired")}
    # Gate lock-out guard: after cfg.reacquire_frames consecutive gated frames whose implied heights agree
    # (MAD <= cfg.reacquire_mad_frac x median), re-initialise the terrain as h = z - median height. Uses only
    # disparities already received; disabled for the frozen-terrain control.
    nre = cfg.reacquire_frames
    ring = np.full((B, nre), np.nan)
    streak = np.zeros(B, int)
    for k in range(N):
        if k > 0:
            Qk = Q_pre if k < k_cut else Q_post
            u = np.zeros((B, 5))
            if v.use_imu:
                a = meas.accel_up[:, k - 1]
                u[:, 0] = g[0] * a
                u[:, 1] = g[1] * a
            x = x @ F.T + u
            P = F @ P @ F.T + Qk
            P = 0.5 * (P + P.transpose(0, 2, 1))
        if k == k_cut and v.terrain_after_cut == "frozen":
            P[:, 4, :] = 0.0  # terrain becomes an exactly known constant: the wrong assumption under test
            P[:, :, 4] = 0.0
        if v.use_gnss and k < k_cut:
            y = meas.gnss_alt[:, k]
            if np.isfinite(y).any():
                _update(x, P, y, x[:, 0].copy(), H_gnss, cfg.gnss_sigma**2)
        if v.use_baro:
            _update(x, P, meas.baro_alt[:, k], x[:, 0] + x[:, 2], H_baro, cfg.baro_sigma**2)
        use_st = v.stereo_before_cut if k < k_cut else v.stereo_after_cut
        if use_st and k % every_st == 0:
            d = meas.disparity_px[:, k]
            geom = _stereo_valid(d, rig)
            agl_hat = x[:, 0] - x[:, 4]
            pos = agl_hat > cfg.agl_min
            agl_s = np.maximum(agl_hat, cfg.agl_min)
            kk = fb * meas.cos_tilt_ahrs[:, k]
            H = np.zeros((B, 5))
            H[:, 0] = -kk / agl_s**2
            H[:, 4] = kk / agl_s**2
            ok, gated = _update(x, P, np.where(geom & pos, d, np.nan), kk / agl_s, H, cfg.disp_sigma_px**2,
                                cfg.gate_sigma)
            reacq = np.zeros(B, bool)
            if v.terrain_after_cut == "random_walk" or k < k_cut:
                implied = np.where(geom, kk / np.where(geom, d, 1.0), np.nan)
                rej = geom & ~ok  # gated, or refused because the predicted height above ground is <= agl_min
                ring[rej, streak[rej] % nre] = implied[rej]
                streak = np.where(rej, streak + 1, np.where(ok, 0, streak))
                full = streak >= nre
                if full.any():
                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore", RuntimeWarning)
                        med = np.nanmedian(ring, axis=1)
                        mad = np.nanmedian(np.abs(ring - med[:, None]), axis=1)
                    reacq = full & (mad <= cfg.reacquire_mad_frac * med)
                    for i in np.flatnonzero(reacq):
                        sig = max(med[i] * med[i] * cfg.disp_sigma_px / fb, 1.0)
                        x[i, 4] = x[i, 0] - med[i]
                        P[i, 4, :] = P[i, 0, :]
                        P[i, :, 4] = P[i, :, 0]
                        P[i, 4, 4] = P[i, 0, 0] + sig * sig
                    streak = np.where(reacq, 0, streak)
            if k >= k_cut:
                counts["frames"] += np.isfinite(d)
                counts["geometry_rejected"] += np.isfinite(d) & ~geom
                counts["gated"] += gated
                counts["accepted"] += ok
                counts["reacquired"] += reacq
        if v.use_dem:
            y = dem.dem_h[:, k]
            if np.isfinite(y).any():
                sig_nav = cfg.dem_nav_sigma0 + cfg.dem_nav_growth * cfg.nominal_speed * max(0.0, meas.t[k] - meas.t[k_cut])
                R = cfg.dem_sigma**2 + (np.nan_to_num(dem.dem_slope[:, k]) * sig_nav) ** 2
                _update(x, P, y, x[:, 4].copy(), H_dem, R, cfg.gate_sigma)
        out["z"][:, k] = x[:, 0]
        out["h"][:, k] = x[:, 4]
        out["b"][:, k] = x[:, 2]
        out["var_z"][:, k] = P[:, 0, 0]
        out["var_agl"][:, k] = P[:, 0, 0] + P[:, 4, 4] - 2 * P[:, 0, 4]
    out["counts"] = counts
    return out


# ---------------------------------------------------------------------------------------------------
# Observability (linear, constant offsets)
# ---------------------------------------------------------------------------------------------------


def observability_analysis(dt: float):
    F = np.eye(5)
    F[0, 1] = dt
    F[0, 3] = -0.5 * dt * dt
    F[1, 3] = -dt
    rows = {
        "baro": [1.0, 0, 1.0, 0, 0],
        "stereo": [1.0, 0, 0, 0, -1.0],  # linearised disparity, any nonzero scale
        "gnss": [1.0, 0, 0, 0, 0],
        "dem": [0.0, 0, 0, 0, 1.0],
    }
    cases = {
        "baro+imu": ["baro"],
        "stereo+imu": ["stereo"],
        "baro+stereo+imu": ["baro", "stereo"],
        "baro+stereo+imu+gnss": ["baro", "stereo", "gnss"],
        "baro+stereo+imu+dem": ["baro", "stereo", "dem"],
    }
    res = {}
    for name, sensors in cases.items():
        O = np.vstack([np.array(rows[s]) @ np.linalg.matrix_power(F, i) for i in range(5) for s in sensors])
        _, sv, vt = np.linalg.svd(O)
        rank = int(np.sum(sv > 1e-9 * sv[0]))
        null = vt[rank:].tolist()
        res[name] = {"rank": rank, "of": 5, "null_space": null}
    shift = np.array([1.0, 0, -1.0, 0, 1.0]) / np.sqrt(3.0)
    null3 = np.array(res["baro+stereo+imu"]["null_space"])
    res["expected_null_direction"] = "(z, vz, b, b_acc, h) -> (z + c, vz, b - c, b_acc, h + c)"
    res["null_alignment"] = float(abs(null3 @ shift)[0]) if null3.shape[0] == 1 else float("nan")
    # Nonlinear check: the disparity and baro readings are unchanged along the shift.
    xs = np.array([140.0, 0.5, 7.0, 0.02, 95.0])
    c = 37.0
    x2 = xs + c * np.array([1.0, 0, -1.0, 0, 1.0])
    fb = 200.0
    res["nonlinear_invariance_max_abs"] = float(max(abs((xs[0] + xs[2]) - (x2[0] + x2[2])),
                                                    abs(fb / (xs[0] - xs[4]) - fb / (x2[0] - x2[4]))))
    return res


# ---------------------------------------------------------------------------------------------------
# Scoring helpers
# ---------------------------------------------------------------------------------------------------


def _stats(err, sd=None):
    a = np.abs(err)
    r = {
        "p50_abs": float(np.percentile(a, 50)),
        "p95_abs": float(np.percentile(a, 95)),
        "rmse": float(np.sqrt(np.mean(err**2))),
        "max_abs": float(a.max()),
        "final_p50_abs": float(np.percentile(a[:, -1], 50)),
        "final_p95_abs": float(np.percentile(a[:, -1], 95)),
        "coverage_2sigma": float(np.mean(a <= 2 * sd)) if sd is not None else float("nan"),
    }
    return r


def _check(checks, name, ok, detail=""):
    checks.append({"name": name, "passed": bool(ok), "detail": detail})


def _hyp(hid, statement, observed, supported):
    return {"id": hid, "statement": statement, "observed": observed,
            "verdict": "supported" if supported else "refuted"}


def vertical_hypotheses(summary: pd.DataFrame, per_seed: pd.DataFrame):
    def sv(scn, var, q, stat):
        r = summary[(summary.scenario == scn) & (summary.variant == var) & (summary.quantity == q)]
        return float(r[stat].iloc[0])

    H = []
    a, b = sv("flat-low-good-nominal", "fusion_const_terrain", "agl", "rmse"), sv("flat-low-good-nominal", "fusion_terrain_state", "agl", "rmse")
    H.append(_hyp("H1", "Flat terrain, low, good texture: constant-terrain control AGL RMSE <= 1.5 x terrain-state fusion (the control is not a straw man)",
                  {"const_terrain_agl_rmse": a, "terrain_state_agl_rmse": b}, a <= 1.5 * b))
    scn = "changing-low-good-nominal"
    a, c, d = (sv(scn, "fusion_terrain_state", "agl", "rmse"), sv(scn, "fusion_const_terrain", "agl", "rmse"),
               sv(scn, "imu_baro", "agl", "rmse"))
    H.append(_hyp("H2", "Changing terrain, low: terrain-state fusion AGL RMSE <= 0.5 x constant-terrain control and <= 0.5 x IMU+baro",
                  {"terrain_state": a, "const_terrain": c, "imu_baro": d}, a <= 0.5 * c and a <= 0.5 * d))
    nd = per_seed[(per_seed.variant == "fusion_no_datum") & per_seed.scenario.isin(["flat-low-good-nominal", scn])]
    corr = float(np.corrcoef(nd.final_err_z, nd.truth_baro_bias_final)[0, 1]) if len(nd) >= 3 else float("nan")
    a, b = sv(scn, "fusion_no_datum", "agl", "rmse"), sv(scn, "fusion_terrain_state", "agl", "rmse")
    H.append(_hyp("H3", "No GNSS datum: final altitude error tracks the unknown baro offset (corr >= 0.9) while AGL RMSE stays <= 1.5 x the datum case",
                  {"corr_final_z_err_vs_true_baro_offset": corr, "no_datum_agl_rmse": a, "datum_agl_rmse": b,
                   "median_final_sd_z_no_datum": float(nd.final_sd_z.median())},
                  bool(corr >= 0.9) and a <= 1.5 * b))
    st = per_seed[per_seed.scenario == "changing-low-good-strong"]
    drift = float(st[st.variant == "fusion_terrain_state"].truth_baro_drift_since_cut.abs().median())
    ts = float(st[st.variant == "fusion_terrain_state"].final_err_z.abs().median())
    kd = float(st[st.variant == "fusion_known_dem"].final_err_z.abs().median())
    H.append(_hyp("H4", "Strong baro drift, no DEM: terrain-state fusion final |z error| >= 0.5 x true drift since the cut; known-DEM ablation <= 0.5 x that",
                  {"median_abs_drift_since_cut": drift, "terrain_state_final_abs_z": ts, "known_dem_final_abs_z": kd},
                  ts >= 0.5 * drift and kd <= 0.5 * ts))
    a, b = sv("flat-high-good-nominal", "stereo_only", "agl", "p95_abs"), sv("flat-low-good-nominal", "stereo_only", "agl", "p95_abs")
    H.append(_hyp("H5", "Instantaneous stereo with the declared rig: AGL p95 at 300 m >= 10 x at 40 m (sigma_h = h^2 sigma_d / (f B))",
                  {"high_p95": a, "low_p95": b}, a >= 10 * b))
    scn = "changing-low-loss-nominal"
    a, b = sv(scn, "fusion_terrain_state", "agl", "p95_abs"), sv(scn, "stereo_only", "agl", "p95_abs")
    H.append(_hyp("H6", "Texture loss, changing terrain: gated fusion AGL p95 < stereo-only AGL p95",
                  {"fusion_p95": a, "stereo_only_p95": b}, a < b))
    cov = {s: sv(s, "fusion_terrain_state", "agl", "coverage_2sigma") for s in ("flat-low-good-nominal", "changing-low-good-nominal")}
    H.append(_hyp("H7", "Consistency: terrain-state fusion AGL within 2 sigma for 85-99 % of samples (low, good, nominal)",
                  cov, all(0.85 <= c <= 0.99 for c in cov.values())))
    return H


def run_vertical(args, out: Path, checks, make_figures):
    sim = VerticalSimModel(disp_noise_px=args.disp_sigma_px)
    rig = StereoRig(focal_px=args.focal_px, baseline_m=args.baseline_m)
    cfg = VerticalEstimatorConfig(terrain_slope_sigma=args.terrain_slope_sigma)
    n = int(round((args.t_cut + args.duration) / sim.dt))
    t = np.arange(n) * sim.dt
    k_cut = int(round(args.t_cut / sim.dt))
    truth, meas, dem, elem_scn, elem_seed = generate_vertical(args.seeds, args.seed0, t, k_cut, args.duration, sim, rig)

    _check(checks, "vertical: no GNSS from the cut on", np.isnan(meas.gnss_alt[:, k_cut:]).all())
    _check(checks, "vertical: GNSS present before the cut in every flight", np.isfinite(meas.gnss_alt[:, :k_cut]).any(axis=1).all())
    _check(checks, "vertical: AGL truth positive", (truth.agl > 0).all(), f"min {truth.agl.min():.1f} m")
    obs = observability_analysis(sim.dt)
    _check(checks, "observability: baro+stereo+IMU rank 4 of 5", obs["baro+stereo+imu"]["rank"] == 4)
    _check(checks, "observability: null space is (z+c, b-c, h+c)", obs["null_alignment"] > 1 - 1e-9, f"{obs['null_alignment']:.12f}")
    _check(checks, "observability: GNSS datum restores rank 5", obs["baro+stereo+imu+gnss"]["rank"] == 5)
    _check(checks, "observability: DEM restores rank 5", obs["baro+stereo+imu+dem"]["rank"] == 5)
    _check(checks, "observability: readings invariant along the null direction", obs["nonlinear_invariance_max_abs"] < 1e-9)

    # Causality: perturbing every post-cut reading must not change any pre-cut estimate.
    idx = np.arange(min(4, meas.baro_alt.shape[0]))
    ref_m = subset_measurements(meas, idx)
    pert = subset_measurements(meas, idx)
    pert.baro_alt[:, k_cut:] += 50.0
    pert.disparity_px[:, k_cut:] *= 1.3
    pert.accel_up[:, k_cut:] += 0.5
    v_ts = next(v for v in VARIANTS if v.name == "fusion_terrain_state")
    e0, e1 = run_vertical_variant(ref_m, rig, cfg, v_ts), run_vertical_variant(pert, rig, cfg, v_ts)
    _check(checks, "vertical: estimator is causal (post-cut readings do not change pre-cut estimates)",
           np.array_equal(e0["z"][:, :k_cut], e1["z"][:, :k_cut]) and not np.array_equal(e0["z"][:, k_cut:], e1["z"][:, k_cut:]))

    post = slice(k_cut, None)
    drift_since_cut = truth.baro_bias[:, -1] - truth.baro_bias[:, k_cut]
    ex_scn = SCENARIOS.index(Scenario("changing", "low", "loss", "strong"))
    ex = int(np.flatnonzero(elem_scn == ex_scn)[0])
    examples = {}
    summary_rows, seed_rows, count_rows = [], [], []
    for v in VARIANTS:
        t0 = time.perf_counter()
        est = run_vertical_variant(meas, rig, cfg, v, dem if v.use_dem else None)
        elapsed = time.perf_counter() - t0
        z, h, b = (est[key].astype(float) for key in ("z", "h", "b"))
        sd_z = np.sqrt(np.maximum(est["var_z"].astype(float), 0))
        sd_agl = np.sqrt(np.maximum(est["var_agl"].astype(float), 0))
        _check(checks, f"vertical: {v.name} estimates finite", np.isfinite(z).all() and np.isfinite(h).all())
        if v.terrain_after_cut == "frozen":
            _check(checks, f"vertical: {v.name} terrain constant after the cut", (np.ptp(h[:, post], axis=1) == 0).all())
        errs = {"z": (z - truth.z, sd_z), "agl": ((z - h) - truth.agl, sd_agl), "terrain": (h - truth.h, None)}
        if v.use_baro:
            errs["baro_bias"] = (b - truth.baro_bias, None)
        for si, sc in enumerate(SCENARIOS):
            sel = elem_scn == si
            for q, (e, sd) in errs.items():
                row = {"scenario": sc.name, **asdict(sc), "variant": v.name, "role": v.role, "quantity": q,
                       "n_seeds": int(sel.sum())}
                row.update(_stats(e[sel][:, post], sd[sel][:, post] if sd is not None else None))
                summary_rows.append(row)
            c = {key: int(val[sel].sum()) for key, val in est["counts"].items()}
            count_rows.append({"scenario": sc.name, "variant": v.name, **c})
        for i in range(z.shape[0]):
            ez, ea = errs["z"][0][i, post], errs["agl"][0][i, post]
            seed_rows.append({
                "scenario": SCENARIOS[elem_scn[i]].name, "seed": int(elem_seed[i]), "variant": v.name,
                "rmse_z": float(np.sqrt(np.mean(ez**2))), "rmse_agl": float(np.sqrt(np.mean(ea**2))),
                "p95_abs_agl": float(np.percentile(np.abs(ea), 95)),
                "final_err_z": float(ez[-1]), "final_err_agl": float(ea[-1]),
                "final_sd_z": float(sd_z[i, -1]), "final_sd_agl": float(sd_agl[i, -1]),
                "truth_baro_bias_final": float(truth.baro_bias[i, -1]),
                "truth_baro_drift_since_cut": float(drift_since_cut[i]),
            })
        examples[v.name] = {"z": z[ex], "h": h[ex], "b": b[ex], "sd_agl": sd_agl[ex]}
        print(f"   {v.name:22s} {elapsed:5.1f} s")

    summary = pd.DataFrame(summary_rows)
    per_seed = pd.DataFrame(seed_rows)
    counts = pd.DataFrame(count_rows)
    summary.to_csv(out / "vertical_summary.csv", index=False)
    per_seed.to_csv(out / "vertical_per_seed.csv", index=False)
    counts.to_csv(out / "vertical_stereo_counts.csv", index=False)
    hyps = vertical_hypotheses(summary, per_seed)

    fb = rig.focal_px * rig.baseline_m
    physics = {
        f"agl_{a:.0f}m": {"disparity_px": fb / a, "single_frame_sigma_agl_m": a * a * sim.disp_noise_px / fb,
                          "offset_bias_agl_m": a * a * sim.disp_offset_px / fb}
        for a in (sim.agl_low, sim.agl_high)
    }
    if make_figures:
        ex_truth = {key: getattr(truth, key)[ex] for key in ("z", "h", "agl", "baro_bias", "texture_loss")}
        _fig_vertical_example(out / "vertical_example.png", t, k_cut, ex_truth, examples, SCENARIOS[ex_scn].name)
        _fig_vertical_bars(out / "vertical_rmse.png", summary)
    return {
        "generator_model": asdict(sim), "stereo_rig": asdict(rig), "estimator_config": asdict(cfg),
        "variants": [asdict(v) for v in VARIANTS], "scenarios": [s.name for s in SCENARIOS],
        "observability": obs, "stereo_physics_declared_rig": physics, "hypotheses": hyps,
        "example_flight": {"scenario": SCENARIOS[ex_scn].name, "seed": int(elem_seed[ex])},
    }


# ---------------------------------------------------------------------------------------------------
# Sun ephemeris (NOAA GML "General Solar Position Calculations")
# ---------------------------------------------------------------------------------------------------


def solar_az_el(lat_deg, lon_deg, year, doy0, utc_hours):
    """Azimuth (rad, clockwise from north) and elevation (rad). utc_hours may leave [0, 24)."""
    doy = doy0 + np.floor(utc_hours / 24.0)
    hour = np.mod(utc_hours, 24.0)
    ndays = 366 if (year % 4 == 0 and (year % 100 != 0 or year % 400 == 0)) else 365
    gam = 2 * np.pi / ndays * (doy - 1 + (hour - 12) / 24)
    eqtime = 229.18 * (0.000075 + 0.001868 * np.cos(gam) - 0.032077 * np.sin(gam)
                       - 0.014615 * np.cos(2 * gam) - 0.040849 * np.sin(2 * gam))
    decl = (0.006918 - 0.399912 * np.cos(gam) + 0.070257 * np.sin(gam) - 0.006758 * np.cos(2 * gam)
            + 0.000907 * np.sin(2 * gam) - 0.002697 * np.cos(3 * gam) + 0.00148 * np.sin(3 * gam))
    tst = hour * 60 + eqtime + 4 * lon_deg
    ha = np.radians(tst / 4 - 180)
    lat = np.radians(lat_deg)
    el = np.arcsin(np.clip(np.sin(lat) * np.sin(decl) + np.cos(lat) * np.cos(decl) * np.cos(ha), -1, 1))
    az = np.arctan2(-np.cos(decl) * np.sin(ha), np.sin(decl) * np.cos(lat) - np.cos(decl) * np.sin(lat) * np.cos(ha))
    return np.mod(az, 2 * np.pi), el


TAIPEI = (25.033, 121.565)
TAIWAN_SITES = {"Taipei": TAIPEI, "Wufeng_Taichung": (24.06, 120.70), "Kaohsiung": (22.627, 120.301)}
UTC_OFFSET_TAIWAN = 8.0


def _doy(iso: str) -> tuple[int, int]:
    d = date.fromisoformat(iso)
    return d.year, d.timetuple().tm_yday


def solar_noon_utc_hours(lat, lon, iso):
    year, doy = _doy(iso)
    hours = np.arange(0, 24, 1 / 60) - UTC_OFFSET_TAIWAN + 12  # local 00:00-24:00 as UTC hours
    _, el = solar_az_el(lat, lon, year, doy, hours)
    return float(hours[np.argmax(el)])


def taiwan_sun_table(year: int = 2026):
    rows = []
    days = np.arange(1, 366)
    hours = np.arange(0, 24, 1 / 60)
    for site, (lat, lon) in TAIWAN_SITES.items():
        _, el = solar_az_el(lat, lon, year, days[:, None], hours[None, :])
        max_el = np.degrees(el.max(axis=1))
        rows.append({"site": site, "lat": lat, "lon": lon, "min_noon_elev_deg": float(max_el.min()),
                     "max_noon_elev_deg": float(max_el.max()),
                     "days_noon_elev_ge_80": int((max_el >= 80).sum()), "days_noon_elev_ge_85": int((max_el >= 85).sum())})
    return pd.DataFrame(rows), days


# ---------------------------------------------------------------------------------------------------
# Yaw Monte Carlo
# ---------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class YawSimModel:
    """GENERATOR ONLY."""

    dt: float = 0.1
    speed: float = 15.0
    gyro_noise_100hz: float = 0.02  # rad/s per 100 Hz sample: Mid-Air median, findings 2.3 (synthetic IMU)
    gyro_bias_sigma: float = 0.001  # rad/s: Mid-Air median constant offset, findings 2.3
    gyro_bias_rw: float = 2e-5  # rad/s/sqrt(s)
    heading_at_cut_sigma_deg: float = 2.0
    turn_rate_deg: float = 6.0
    straight_s: tuple = (30.0, 60.0)
    turn_factor: float = 0.1  # AHRS roll error = -turn_factor * bank angle in coordinated turns
    sun_fov_half_deg: float = 60.0  # assumed sensor half field of view, boresight up
    sun_min_elev_deg: float = 5.0
    sun_rate_hz: float = 10.0
    cloud_mean_clear_s: float = 40.0
    cloud_mean_blocked_s: float = 40.0


@dataclass(frozen=True)
class YawEstimatorConfig:
    gyro_noise: float = 0.02 / np.sqrt(10)  # rad/s per 10 Hz sample
    gyro_bias_sigma0: float = 0.003
    gyro_bias_rw: float = 5e-5
    heading_at_cut_sigma_deg: float = 2.0
    gate_sigma: float = 4.0
    sun_err_tau: float = 60.0  # s, correlation time of the AHRS-tilt-induced sun-heading error
    max_sun_sigma_deg: float = 10.0  # refuse a sun heading whose declared sigma is larger (sun near zenith)


SENSOR_SPECS = {"lab": (0.1, 0.5), "field": (1.0, 2.0)}  # (sun-vector noise deg, AHRS tilt error deg)
GEOMETRIES = {
    "oct_morning": ("2026-10-03", 9.0),
    "oct_noon": ("2026-10-03", "solar_noon"),
    "jun_noon": ("2026-06-21", "solar_noon"),
    "dec_afternoon": ("2026-12-21", 15.5),
}


@dataclass(frozen=True)
class YawRegime:
    geometry: str
    sensor: str
    sky: str  # clear | broken

    @property
    def name(self) -> str:
        return f"{self.geometry}-{self.sensor}-{self.sky}"


YAW_REGIMES = tuple(YawRegime(*c) for c in itertools.product(GEOMETRIES, SENSOR_SPECS, ("clear", "broken")))
YAW_METHODS = ("gyro_only", "sun_only_hold", "gyro_sun_kf")


@dataclass
class YawMeasurements:
    """Everything a heading estimator may read. Time 0 is the GNSS cut."""

    t: np.ndarray
    dt: float
    gyro_yaw_rate: np.ndarray  # (B, N) rad/s, yaw-rate channel incl. bias
    sun_body: np.ndarray  # (B, N, 3) unit vector in body axes, NaN without sun
    roll_ahrs: np.ndarray
    pitch_ahrs: np.ndarray
    heading_at_cut: np.ndarray  # (B,) last heading reference before the cut (with error)
    lat_at_cut: np.ndarray  # (B,) position at the cut, used for the ephemeris throughout
    lon_at_cut: np.ndarray
    year: int
    doy0: np.ndarray  # (B,)
    utc0_hours: np.ndarray  # (B,)
    sun_sigma_spec: np.ndarray  # (B,) rad, sensor datasheet value as declared by the regime
    tilt_sigma_spec: np.ndarray  # (B,) rad


def _ned_to_body(v, psi, theta, phi):
    x, y, z = v[:, 0], v[:, 1], v[:, 2]
    c, s = np.cos(psi), np.sin(psi)
    x, y = c * x + s * y, -s * x + c * y
    c, s = np.cos(theta), np.sin(theta)
    x, z = c * x - s * z, s * x + c * z
    c, s = np.cos(phi), np.sin(phi)
    y, z = c * y + s * z, -s * y + c * z
    return np.stack([x, y, z], axis=-1)


def _body_to_level(v, theta, phi):
    x, y, z = v[..., 0], v[..., 1], v[..., 2]
    c, s = np.cos(phi), np.sin(phi)
    y, z = c * y - s * z, s * y + c * z
    c, s = np.cos(theta), np.sin(theta)
    x, z = c * x + s * z, -s * x + c * z
    return np.stack([x, y, z], axis=-1)


def _yaw_draws(rng, n, sim: YawSimModel):
    dt = sim.dt
    psi0 = rng.uniform(0, 2 * np.pi)
    rate = np.zeros(n)
    turn_len = int(round(90.0 / sim.turn_rate_deg / dt))
    k = 0
    while k < n:
        k += int(round(rng.uniform(*sim.straight_s) / dt))
        rate[k:k + turn_len] = (1.0 if rng.random() < 0.5 else -1.0) * np.radians(sim.turn_rate_deg)
        k += turn_len
    d = dict(psi0=psi0, rate=rate)
    d["att_phase"] = rng.uniform(0, 2 * np.pi, 2)
    d["bg0"] = sim.gyro_bias_sigma * rng.standard_normal()
    d["bg_steps"] = sim.gyro_bias_rw * np.sqrt(dt) * rng.standard_normal(n)
    d["gyro_noise"] = sim.gyro_noise_100hz / np.sqrt(10) * rng.standard_normal(n)
    d["tilt_bias"] = rng.standard_normal(2) / np.sqrt(2)
    d["tilt_white"] = rng.standard_normal((2, n)) / np.sqrt(2)
    d["sun_noise"] = rng.standard_normal((n, 3))
    u = rng.random(n)
    blocked = np.empty(n, bool)
    state = rng.random() < sim.cloud_mean_blocked_s / (sim.cloud_mean_clear_s + sim.cloud_mean_blocked_s)
    p_end_blocked, p_end_clear = dt / sim.cloud_mean_blocked_s, dt / sim.cloud_mean_clear_s
    for i in range(n):
        if u[i] < (p_end_blocked if state else p_end_clear):
            state = not state
        blocked[i] = state
    d["blocked"] = blocked
    d["heading_ref"] = rng.standard_normal()
    return d


def generate_yaw(seeds, seed0, duration, sim: YawSimModel):
    n = int(round(duration / sim.dt))
    t = np.arange(n) * sim.dt
    draws = [_yaw_draws(np.random.default_rng([seed0, 2, j]), n, sim) for j in range(seeds)]
    lat0, lon0 = TAIPEI
    every_sun = max(1, int(round(1.0 / (sim.sun_rate_hz * sim.dt))))
    sample = (np.arange(n) % every_sun) == 0
    T = {key: [] for key in ("psi", "el")}
    M = {key: [] for key in ("gyro", "sun", "roll", "pitch", "href", "doy0", "utc0", "sun_spec", "tilt_spec")}
    elem_reg, elem_seed = [], []
    for ri, reg in enumerate(YAW_REGIMES):
        iso, start = GEOMETRIES[reg.geometry]
        year, doy0 = _doy(iso)
        if start == "solar_noon":
            utc0 = solar_noon_utc_hours(lat0, lon0, iso) - duration / 2 / 3600
        else:
            utc0 = start - UTC_OFFSET_TAIWAN
        sun_sig, tilt_sig = (np.radians(v) for v in SENSOR_SPECS[reg.sensor])
        for j, d in enumerate(draws):
            psi = d["psi0"] + sim.dt * np.concatenate([[0.0], np.cumsum(d["rate"][:-1])])
            bank = np.arctan(sim.speed * d["rate"] / G)
            roll = bank + np.radians(2.0) * np.sin(2 * np.pi * t / 17.0 + d["att_phase"][0])
            pitch = -np.radians(5.0 + 1.5 * np.sin(2 * np.pi * t / 11.0 + d["att_phase"][1]))
            gyro = d["rate"] + d["bg0"] + np.cumsum(d["bg_steps"]) + d["gyro_noise"]
            roll_m = roll + tilt_sig * (d["tilt_bias"][0] + d["tilt_white"][0]) - sim.turn_factor * bank
            pitch_m = pitch + tilt_sig * (d["tilt_bias"][1] + d["tilt_white"][1])
            north = sim.dt * np.cumsum(sim.speed * np.cos(psi))
            east = sim.dt * np.cumsum(sim.speed * np.sin(psi))
            lat = lat0 + np.degrees(north / EARTH_RADIUS)
            lon = lon0 + np.degrees(east / (EARTH_RADIUS * np.cos(np.radians(lat0))))
            az, el = solar_az_el(lat, lon, year, doy0, utc0 + t / 3600)
            s_ned = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), -np.sin(el)], axis=1)
            s_body = _ned_to_body(s_ned, psi, pitch, roll)
            noisy = s_body + sun_sig * d["sun_noise"]
            noisy /= np.linalg.norm(noisy, axis=1, keepdims=True)
            visible = (sample & (el > np.radians(sim.sun_min_elev_deg))
                       & (-s_body[:, 2] >= np.cos(np.radians(sim.sun_fov_half_deg))))
            if reg.sky == "broken":
                visible &= ~d["blocked"]
            T["psi"].append(psi)
            T["el"].append(el)
            M["gyro"].append(gyro)
            M["sun"].append(np.where(visible[:, None], noisy, np.nan))
            M["roll"].append(roll_m)
            M["pitch"].append(pitch_m)
            M["href"].append(psi[0] + np.radians(sim.heading_at_cut_sigma_deg) * d["heading_ref"])
            M["doy0"].append(doy0)
            M["utc0"].append(utc0)
            M["sun_spec"].append(sun_sig)
            M["tilt_spec"].append(tilt_sig)
            elem_reg.append(ri)
            elem_seed.append(j)
    B = len(elem_reg)
    meas = YawMeasurements(
        t=t, dt=sim.dt, gyro_yaw_rate=np.stack(M["gyro"]), sun_body=np.stack(M["sun"]),
        roll_ahrs=np.stack(M["roll"]), pitch_ahrs=np.stack(M["pitch"]), heading_at_cut=np.array(M["href"]),
        lat_at_cut=np.full(B, lat0), lon_at_cut=np.full(B, lon0), year=year, doy0=np.array(M["doy0"]),
        utc0_hours=np.array(M["utc0"]), sun_sigma_spec=np.array(M["sun_spec"]), tilt_sigma_spec=np.array(M["tilt_spec"]),
    )
    sample_count = int(sample.sum())
    return {"psi": np.stack(T["psi"]), "el": np.stack(T["el"])}, meas, np.array(elem_reg), np.array(elem_seed), sample_count


def sun_heading_measurements(meas: YawMeasurements):
    """Heading from the sun vector: ephemeris at the cut position and clock, de-tilt with AHRS roll/pitch.

    Returns the heading, the white variance and the variance of the slowly varying part. AHRS roll/pitch
    errors are mostly slow (bias, turn-induced), so their heading effect tan(el) * tilt is not averaged
    away by a high sun-sensor rate: it is carried as a Gauss-Markov state in the filter, not as white noise.
    """
    az, el = solar_az_el(meas.lat_at_cut[:, None], meas.lon_at_cut[:, None], meas.year, meas.doy0[:, None],
                         meas.utc0_hours[:, None] + meas.t[None, :] / 3600)
    lev = _body_to_level(meas.sun_body, meas.pitch_ahrs, meas.roll_ahrs)
    psi = wrap(az - np.arctan2(lev[..., 1], lev[..., 0]))
    cos_el = np.maximum(np.cos(el), 1e-3)
    tilt_part = (meas.tilt_sigma_spec[:, None] * np.sin(el) / cos_el) ** 2
    var_white = (meas.sun_sigma_spec[:, None] / cos_el) ** 2 + tilt_part
    return psi, var_white, tilt_part


def run_yaw_methods(meas: YawMeasurements, cfg: YawEstimatorConfig):
    """State [heading, gyro bias, sun-heading error]; the last is first-order Gauss-Markov (cfg.sun_err_tau)."""
    B, N = meas.gyro_yaw_rate.shape
    dt = meas.dt
    psi_sun, var_sun, var_corr = sun_heading_measurements(meas)
    usable = var_sun + var_corr <= np.radians(cfg.max_sun_sigma_deg) ** 2
    psi_sun = np.where(usable, psi_sun, np.nan)
    gyro_only = meas.heading_at_cut[:, None] + dt * np.concatenate(
        [np.zeros((B, 1)), np.cumsum(meas.gyro_yaw_rate[:, :-1], axis=1)], axis=1)
    hold = np.empty((B, N))
    last = meas.heading_at_cut.copy()
    x = np.stack([meas.heading_at_cut, np.zeros(B), np.zeros(B)], axis=1)
    P = np.zeros((B, 3, 3))
    P[:, 0, 0] = np.radians(cfg.heading_at_cut_sigma_deg) ** 2
    P[:, 1, 1] = cfg.gyro_bias_sigma0**2
    P[:, 2, 2] = var_corr[:, 0]
    phi = np.exp(-dt / cfg.sun_err_tau)
    F = np.array([[1.0, -dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, phi]])
    kf = np.empty((B, N))
    kf_sd = np.empty((B, N))
    accepted = np.zeros(B, int)
    H = np.tile([1.0, 0.0, 1.0], (B, 1))
    for k in range(N):
        y = psi_sun[:, k]
        last = np.where(np.isfinite(y), y, last)
        hold[:, k] = last
        if k > 0:
            x[:, 0] += dt * meas.gyro_yaw_rate[:, k - 1]
            x = x @ F.T
            Q = np.zeros((B, 3, 3))
            Q[:, 0, 0] = (cfg.gyro_noise * dt) ** 2
            Q[:, 1, 1] = cfg.gyro_bias_rw**2 * dt
            Q[:, 2, 2] = var_corr[:, k] * (1 - phi * phi)
            P = F @ P @ F.T + Q
        # wrapped innovation: present the measurement unwrapped around the current prediction
        pred = x[:, 0] + x[:, 2]
        y_unwrapped = pred + wrap(y - pred)
        ok, _ = _update(x, P, y_unwrapped, pred.copy(), H, var_sun[:, k], cfg.gate_sigma)
        accepted += ok
        x[:, 0] = wrap(x[:, 0])
        kf[:, k] = x[:, 0]
        kf_sd[:, k] = np.sqrt(P[:, 0, 0])
    return {"gyro_only": gyro_only, "sun_only_hold": hold, "gyro_sun_kf": kf}, kf_sd, accepted, np.isfinite(psi_sun)


def yaw_hypotheses(summary: pd.DataFrame):
    def sv(reg, m, stat="p95_abs_deg"):
        r = summary[(summary.regime == reg) & (summary.method == m)]
        return float(r[stat].iloc[0])

    H = []
    r = "oct_noon-lab-clear"
    a, b = sv(r, "gyro_sun_kf"), sv(r, "gyro_only")
    H.append(_hyp("Y1", "Oct noon Taipei, lab-grade sensor, clear sky: gyro+sun p95 <= 2 deg and below gyro-only",
                  {"gyro_sun_p95": a, "gyro_only_p95": b}, a <= 2.0 and a < b))
    a, b = sv("jun_noon-lab-clear", "gyro_sun_kf"), sv(r, "gyro_sun_kf")
    H.append(_hyp("Y2", "Sun near zenith (Taipei, 21 June, noon): gyro+sun p95 >= 3 x the October-noon value",
                  {"jun_noon_p95": a, "oct_noon_p95": b}, a >= 3 * b))
    r = "oct_noon-field-broken"
    a, b, c = sv(r, "gyro_sun_kf"), sv(r, "gyro_only"), sv(r, "sun_only_hold")
    H.append(_hyp("Y3", "Field-grade sensor, broken cloud: gyro+sun p95 <= 0.5 x gyro-only", {"gyro_sun_p95": a, "gyro_only_p95": b},
                  a <= 0.5 * b))
    H.append(_hyp("Y4", "Field-grade sensor, broken cloud: gyro+sun p95 <= sun-only hold-last p95", {"gyro_sun_p95": a, "sun_only_p95": c},
                  a <= c))
    return H


def run_yaw(args, out: Path, checks, make_figures):
    sim = YawSimModel(gyro_bias_sigma=args.gyro_bias_sigma)
    cfg = YawEstimatorConfig()
    truth, meas, elem_reg, elem_seed, n_samples = generate_yaw(args.seeds, args.seed0, args.duration, sim)
    est, kf_sd, accepted, has_sun = run_yaw_methods(meas, cfg)
    _check(checks, "yaw: heading reference at the cut has error (not truth)",
           np.abs(wrap(meas.heading_at_cut - truth["psi"][:, 0])).max() > 0)
    _check(checks, "yaw: estimates finite", all(np.isfinite(e).all() for e in est.values()))
    rows, seed_rows = [], []
    for ri, reg in enumerate(YAW_REGIMES):
        sel = elem_reg == ri
        for m, e in est.items():
            err = np.degrees(wrap(e[sel] - truth["psi"][sel]))
            sd = np.degrees(kf_sd[sel]) if m == "gyro_sun_kf" else None
            s = _stats(err, sd)
            rows.append({"regime": reg.name, **asdict(reg), "method": m, "n_seeds": int(sel.sum()),
                         "sun_availability": float(has_sun[sel].sum() / (n_samples * sel.sum())),
                         "kf_accept_share": float(accepted[sel].sum() / max(1, has_sun[sel].sum())),
                         "elev_min_deg": float(np.degrees(truth["el"][sel].min())),
                         "elev_max_deg": float(np.degrees(truth["el"][sel].max())),
                         **{f"{k}_deg" if not k.startswith("coverage") else k: v for k, v in s.items()}})
            for i in np.flatnonzero(sel):
                ei = np.degrees(wrap(e[i] - truth["psi"][i]))
                seed_rows.append({"regime": reg.name, "seed": int(elem_seed[i]), "method": m,
                                  "rmse_deg": float(np.sqrt(np.mean(ei**2))), "final_err_deg": float(ei[-1]),
                                  "p95_abs_deg": float(np.percentile(np.abs(ei), 95))})
    summary = pd.DataFrame(rows)
    summary.to_csv(out / "yaw_summary.csv", index=False)
    pd.DataFrame(seed_rows).to_csv(out / "yaw_per_seed.csv", index=False)
    sun_table, _ = taiwan_sun_table(2026)
    sun_table.to_csv(out / "sun_elevation_taiwan.csv", index=False)
    if make_figures:
        _fig_yaw(out, meas.t, truth, est, elem_reg, summary)
        _fig_sun_year(out / "sun_noon_elevation_taiwan.png")
    return {"generator_model": asdict(sim), "estimator_config": asdict(cfg), "sensor_specs_deg": SENSOR_SPECS,
            "geometries": {k: list(v) for k, v in GEOMETRIES.items()}, "regimes": [r.name for r in YAW_REGIMES],
            "hypotheses": yaw_hypotheses(summary), "taiwan_sun_table": sun_table.to_dict(orient="records")}


# ---------------------------------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------------------------------


def _plt():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _shade(ax, t, k_cut, loss):
    ax.axvline(t[k_cut], color="k", ls="--", lw=0.8)
    if loss.any():
        ax.axvspan(t[loss][0], t[loss][-1], color="0.85", zorder=0)


def _fig_vertical_example(path, t, k_cut, tr, ex, title):
    plt = _plt()
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    ax = axes[0]
    ax.plot(t, tr["z"], "k", lw=1.5, label="truth")
    for name in ("fusion_terrain_state", "fusion_const_terrain", "imu_baro", "fusion_known_dem", "fusion_no_datum"):
        ax.plot(t, ex[name]["z"], lw=0.9, label=name)
    ax.set_ylabel("altitude z [m]")
    ax = axes[1]
    ax.plot(t, tr["agl"], "k", lw=1.5, label="truth")
    for name in ("fusion_terrain_state", "fusion_const_terrain", "stereo_only", "imu_baro"):
        ax.plot(t, ex[name]["z"] - ex[name]["h"], lw=0.9, label=name)
    ax.set_ylabel("height above ground [m]")
    ax = axes[2]
    ax.plot(t, tr["h"], "k", lw=1.5, label="truth")
    for name in ("fusion_terrain_state", "fusion_const_terrain", "fusion_known_dem"):
        ax.plot(t, ex[name]["h"], lw=0.9, label=name)
    ax.set_ylabel("terrain elevation [m]")
    ax = axes[3]
    ax.plot(t, tr["baro_bias"], "k", lw=1.5, label="truth")
    for name in ("fusion_terrain_state", "fusion_known_dem", "fusion_no_datum"):
        ax.plot(t, ex[name]["b"], lw=0.9, label=name)
    ax.set_ylabel("baro offset [m]")
    ax.set_xlabel("time [s] (dashed: GNSS cut, grey: texture loss)")
    for ax in axes:
        _shade(ax, t, k_cut, tr["texture_loss"])
        ax.legend(fontsize=7, loc="best")
        ax.grid(alpha=0.3)
    fig.suptitle(f"SIMULATION, one flight: {title}")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def _fig_vertical_bars(path, summary):
    plt = _plt()
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    scns = [s.name for s in SCENARIOS]
    x = np.arange(len(scns))
    w = 0.8 / len(VARIANTS)
    for ax, q in zip(axes, ("agl", "z")):
        for i, v in enumerate(VARIANTS):
            r = summary[(summary.variant == v.name) & (summary.quantity == q)].set_index("scenario").loc[scns]
            ax.bar(x + (i - len(VARIANTS) / 2 + 0.5) * w, r["rmse"], w, label=v.name)
        ax.set_yscale("log")
        ax.set_ylabel(f"{q} RMSE after cut [m]")
        ax.grid(alpha=0.3, axis="y")
    axes[0].legend(fontsize=7, ncol=4)
    axes[1].set_xticks(x, scns, rotation=60, ha="right", fontsize=8)
    fig.suptitle("SIMULATION: height errors by regime (pooled over seeds)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def _fig_yaw(out, t, truth, est, elem_reg, summary):
    plt = _plt()
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for ax, rname in zip(axes, ("oct_noon-field-broken", "jun_noon-lab-clear")):
        ri = [r.name for r in YAW_REGIMES].index(rname)
        i = int(np.flatnonzero(elem_reg == ri)[0])
        for m, e in est.items():
            ax.plot(t, np.degrees(wrap(e[i] - truth["psi"][i])), lw=0.9, label=m)
        ax.set_title(f"SIMULATION, one flight: {rname}", fontsize=10)
        ax.set_ylabel("heading error [deg]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[1].set_xlabel("time since GNSS cut [s]")
    fig.tight_layout()
    fig.savefig(out / "yaw_example.png", dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 5))
    names = [r.name for r in YAW_REGIMES]
    x = np.arange(len(names))
    w = 0.8 / len(YAW_METHODS)
    for i, m in enumerate(YAW_METHODS):
        r = summary[summary.method == m].set_index("regime").loc[names]
        ax.bar(x + (i - 1) * w, r["p95_abs_deg"], w, label=m)
    ax.set_yscale("log")
    ax.set_ylabel("heading error p95 [deg]")
    ax.set_xticks(x, names, rotation=60, ha="right", fontsize=8)
    ax.grid(alpha=0.3, axis="y")
    ax.legend()
    ax.set_title("SIMULATION: sun-compass ablation by regime")
    fig.tight_layout()
    fig.savefig(out / "yaw_p95.png", dpi=120)
    plt.close(fig)


def _fig_sun_year(path):
    plt = _plt()
    days = np.arange(1, 366)
    hours = np.arange(0, 24, 1 / 60)
    fig, ax = plt.subplots(figsize=(9, 4))
    for site, (lat, lon) in TAIWAN_SITES.items():
        _, el = solar_az_el(lat, lon, 2026, days[:, None], hours[None, :])
        ax.plot(days, np.degrees(el.max(axis=1)), label=site)
    ax.axhline(85, color="k", ls="--", lw=0.8)
    ax.set_xlabel("day of 2026")
    ax.set_ylabel("sun elevation at solar noon [deg]")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------------------------------


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seeds", type=int, default=20, help="Monte Carlo flights per regime (default 20)")
    p.add_argument("--seed0", type=int, default=0, help="base seed; flights use SeedSequence([seed0, part, j])")
    p.add_argument("--duration", type=float, default=600.0, help="seconds flown after the GNSS cut (default 600)")
    p.add_argument("--t-cut", type=float, default=60.0, help="seconds of GNSS before the cut, vertical part (default 60)")
    p.add_argument("--part", choices=("all", "vertical", "yaw"), default="all")
    p.add_argument("--out", default="data/processed/sensor_fusion")
    p.add_argument("--baseline-m", type=float, default=0.20, help="calibrated stereo baseline (default 0.20 m)")
    p.add_argument("--focal-px", type=float, default=1000.0, help="stereo focal length in pixels (default 1000)")
    p.add_argument("--disp-sigma-px", type=float, default=0.20, help="generator disparity noise (default 0.20 px)")
    p.add_argument("--gyro-bias-sigma", type=float, default=0.001, help="generator yaw gyro bias sigma, rad/s")
    p.add_argument("--terrain-slope-sigma", type=float, default=0.2, help="estimator terrain random walk (default 0.2)")
    p.add_argument("--no-figures", action="store_true")
    p.add_argument("--quick", action="store_true", help="smoke run: 3 seeds, 240 s")
    args = p.parse_args(argv)
    if args.quick:
        args.seeds, args.duration = 3, 240.0
    if args.seeds < 1 or args.duration <= 0 or args.t_cut < 3.0:
        p.error("need --seeds >= 1, --duration > 0, --t-cut >= 3")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()
    checks: list[dict] = []
    results = {
        "kind": "SIMULATION: synthetic sensors generated from synthetic truth with declared noise; not a measurement",
        "script": "experiments/n_sensor_fusion.py",
        "args": vars(args),
    }
    if args.part in ("all", "vertical"):
        print("Part 1, height: running variants")
        results["vertical"] = run_vertical(args, out, checks, not args.no_figures)
    if args.part in ("all", "yaw"):
        print("Part 2, heading: sun-compass ablation")
        results["yaw"] = run_yaw(args, out, checks, not args.no_figures)
    results["invariants"] = checks
    results["runtime_s"] = round(time.perf_counter() - t0, 1)
    with open(out / "results.json", "w") as f:
        json.dump(results, f, indent=2, default=lambda o: o.tolist() if isinstance(o, np.ndarray) else str(o))

    failed = [c for c in checks if not c["passed"]]
    print(f"\nInvariants: {len(checks) - len(failed)} of {len(checks)} pass")
    for c in failed:
        print(f"   FAILED: {c['name']} {c['detail']}")
    for part in ("vertical", "yaw"):
        for h in results.get(part, {}).get("hypotheses", []):
            print(f"   {h['id']:3s} {h['verdict']:9s} {h['statement']}")
    print(f"\nWritten to {out}/ in {results['runtime_s']} s")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

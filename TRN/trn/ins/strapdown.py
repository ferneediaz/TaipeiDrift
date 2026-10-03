"""Strapdown INS: ideal-IMU inversion of the truth, IMU error generation, and mechanization.

Local-level ENU mechanization with Earth rate, transport rate (Schuler loop) and normal gravity
(unstable vertical channel). Discrete equations (k -> k+1):
    C_{k+1} = Exp(-w_in dt) C_k Exp(w_ib dt)
    v_{k+1} = v_k + dt (C_k f_b - (2 w_ie + w_en) x v_k + g)
    p_{k+1} = p_k + dt v_k
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numba import njit

from trn.ins.nav_math import exp_so3, latitude, log_so3, nav_rates

DEG = math.pi / 180.0
UG = 9.80665e-6


@njit(cache=True)
def ideal_imu(pos, vel, C, dt, lat_ref, n_ref, R, omega):
    """Specific force and angular rate (body) that reproduce the truth exactly through :func:`mechanize`."""
    n = pos.shape[0] - 1
    f = np.empty((n, 3)); w = np.empty((n, 3))
    for k in range(n):
        lat = latitude(pos[k, 1], lat_ref, n_ref, R)
        wie, wen, g = nav_rates(lat, pos[k, 2], vel[k], R, omega)
        win = wie + wen
        an = (vel[k + 1] - vel[k]) / dt + np.cross(2.0 * wie + wen, vel[k]) - g
        f[k] = C[k].T @ an
        M = C[k].T @ exp_so3(win * dt) @ C[k + 1]
        w[k] = log_so3(M) / dt
    return f, w


@njit(cache=True)
def mechanize(p0, v0, C0, f, w, dt, lat_ref, n_ref, R, omega, dec):
    """Integrate the IMU; return states every ``dec`` steps and the mean nav-frame specific force per interval."""
    n = f.shape[0]
    m = n // dec + 1
    P = np.empty((m, 3)); Vv = np.empty((m, 3)); CC = np.empty((m, 3, 3)); FN = np.zeros((m, 3))
    p = p0.copy(); v = v0.copy(); Cb = C0.copy()
    P[0] = p; Vv[0] = v; CC[0] = Cb
    acc = np.zeros(3)
    j = 1
    for k in range(n):
        lat = latitude(p[1], lat_ref, n_ref, R)
        wie, wen, g = nav_rates(lat, p[2], v, R, omega)
        fn = Cb @ f[k]
        acc += fn
        vn = v + dt * (fn - np.cross(2.0 * wie + wen, v) + g)
        p = p + dt * v
        Cb = exp_so3(-(wie + wen) * dt) @ Cb @ exp_so3(w[k] * dt)
        v = vn
        if (k + 1) % dec == 0:
            P[j] = p; Vv[j] = v; CC[j] = Cb; FN[j - 1] = acc / dec
            acc[:] = 0.0
            j += 1
    FN[j - 1] = FN[j - 2] if j >= 2 else FN[j - 1]
    return P[:j], Vv[:j], CC[:j], FN[:j]


@dataclass
class ImuErrorParams:
    """IMU error parameters in SI units (rad/s, m/s^2)."""

    gyro_bias: float
    gyro_tau: float
    arw: float          # rad/sqrt(s)
    accel_bias: float
    accel_tau: float
    vrw: float          # m/s/sqrt(s)
    init_att: np.ndarray  # rad (roll, pitch, yaw) 1-sigma

    @classmethod
    def from_config(cls, imu_cfg: dict, grade: str | None = None) -> "ImuErrorParams":
        p = imu_cfg["presets"][grade or imu_cfg["grade"]]
        return cls(gyro_bias=p["gyro_bias_dph"] * DEG / 3600.0, gyro_tau=float(p["gyro_tau_s"]),
                   arw=p["arw_dpsh"] * DEG / 60.0, accel_bias=p["accel_bias_ug"] * UG,
                   accel_tau=float(p["accel_tau_s"]), vrw=p["vrw_ug_rthz"] * UG,
                   init_att=np.asarray(p["init_att_deg"], float) * DEG)


def gauss_markov(n: int, sigma: float, tau: float, dt: float, rng: np.random.Generator) -> np.ndarray:
    """(n,3) stationary first-order Gauss-Markov sequence."""
    a = math.exp(-dt / tau)
    q = sigma * math.sqrt(1.0 - a * a)
    x = np.empty((n, 3))
    x[0] = sigma * rng.standard_normal(3)
    e = q * rng.standard_normal((n, 3))
    _gm_loop(x, e, a)
    return x


@njit(cache=True)
def _gm_loop(x, e, a):
    for k in range(1, x.shape[0]):
        x[k] = a * x[k - 1] + e[k]


def corrupt_imu(f: np.ndarray, w: np.ndarray, p: ImuErrorParams, dt: float, rng: np.random.Generator
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Add GM biases and white noise. Returns (f_meas, w_meas, accel_bias, gyro_bias)."""
    n = f.shape[0]
    ba = gauss_markov(n, p.accel_bias, p.accel_tau, dt, rng)
    bg = gauss_markov(n, p.gyro_bias, p.gyro_tau, dt, rng)
    fm = f + ba + (p.vrw / math.sqrt(dt)) * rng.standard_normal((n, 3))
    wm = w + bg + (p.arw / math.sqrt(dt)) * rng.standard_normal((n, 3))
    return fm, wm, ba, bg


@dataclass
class InsOutput:
    """INS solution at the output rate (aligned with truth samples ``idx``)."""

    idx: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    C: np.ndarray
    fn: np.ndarray
    init_err: dict


def run_ins(traj, imu_cfg: dict, out_rate_hz: float, rng: np.random.Generator, grade: str | None = None,
            perfect: bool = False) -> InsOutput:
    """Simulate an INS that loses GNSS at t=0 (initial errors per config) and free-runs along the truth."""
    R, omega = float(imu_cfg["earth_radius_m"]), imu_cfg["earth_rate_rps"]
    f, w = ideal_imu(traj.pos, traj.vel, traj.C, traj.dt, traj.lat_ref, traj.n_ref, R, omega)
    p = ImuErrorParams.from_config(imu_cfg, grade)
    ie = imu_cfg["init_errors"]
    if perfect:
        fm, wm = f, w
        dp = np.zeros(3); dv = np.zeros(3); phi = np.zeros(3)
    else:
        fm, wm, _, _ = corrupt_imu(f, w, p, traj.dt, rng)
        dp = np.array([ie["horiz_pos_m"] * rng.standard_normal(), ie["horiz_pos_m"] * rng.standard_normal(),
                       ie["vert_pos_m"] * rng.standard_normal()])
        dv = ie["vel_mps"] * rng.standard_normal(3)
        # attitude error: roll/pitch/yaw sigmas mapped to nav-frame misalignment (E,N,U ~ level, level, heading)
        phi = p.init_att[[0, 1, 2]] * rng.standard_normal(3)
    dec = int(round(1.0 / (traj.dt * out_rate_hz)))
    C0 = exp_so3(-phi) @ traj.C[0]          # C_ins = (I - [phi x]) C_true
    P, V, CC, FN = mechanize(traj.pos[0] + dp, traj.vel[0] + dv, C0, fm, wm, traj.dt,
                             traj.lat_ref, traj.n_ref, R, omega, dec)
    idx = np.arange(P.shape[0]) * dec
    return InsOutput(idx, P, V, CC, FN, dict(dp=dp, dv=dv, phi=phi))


def baro_measurements(true_alt: np.ndarray, dt: float, baro_cfg: dict, rng: np.random.Generator) -> np.ndarray:
    """Barometric altitude = truth + GM bias + white noise (NaN array if disabled)."""
    if not baro_cfg["enabled"]:
        return np.full(true_alt.shape, np.nan)
    b = gauss_markov(true_alt.shape[0], baro_cfg["bias_sigma_m"], baro_cfg["bias_tau_s"], dt, rng)[:, 0]
    return true_alt + b + baro_cfg["noise_m"] * rng.standard_normal(true_alt.shape[0])

"""Gyro-bias Kalman filter: relative visual rotations -> gyro bias; optional gravity -> roll/pitch.

The camera measures a RELATIVE body rotation C = R_i^T R_j over an interval i -> j.
Vision is used only to correct the gyroscope bias estimate b_hat. It never pulls the
attitude towards the camera: a better bias slows FUTURE attitude drift; drift already
accumulated stays. The optional gravity update is the only absolute attitude cue, and it
corrects roll/pitch only.

Conventions
-----------
Attitude error:   R_true = Exp(dtheta) R_hat           (world side, both gyro types)
Bias error:       delta_b = b_true - b_hat             (in the gyroscope's own axes)

WORLD-frame gyro (Mid-Air, validated earlier):
    R_hat_k+1 = Exp((w_m,k - b_hat) dt) R_hat_k
    dtheta' = w x dtheta - delta_b   (the rotation term is verified by a finite-difference test)
    over an interval: dtheta_j = M dtheta_i - A delta_b,  M = prod Exp(psi_k),  A <- Exp(psi_k) A + dt I
    residual  r = Log(W_vis W_imu^T),  W_vis = R_hat_i C R_hat_i^T,  W_imu = M (pure propagation)
              = (M - I) dtheta_i - A delta_b + n
    state at j:  H = [I - M^T,  -M^T A]      (H_bg ~ -Delta_t I for small interval rotations)

BODY-frame gyro (a standard strapdown IMU, e.g. NTU VIRAL):
    R_hat_k+1 = R_hat_k Exp((w_m,k - b_hat) dt)
    dtheta' = -R_hat delta_b
    over an interval the propagation product P_b = prod Exp(psi_k) = C_true Exp(A_b delta_b),
    A_b <- Exp(psi_k)^T A_b + dt I.  Residual r = Log(P_b^T C) = -A_b delta_b + n,
    independent of the absolute attitude:  H = [0,  -A_b]

Why dtheta is carried (world gyro). (M - I) dtheta_i is the interval rotation times the
attitude error. On Mid-Air it is as large as the bias signal or larger. A pure 3-state
filter (``consider_attitude=False``) mistakes it for bias and runs away (synthetic tests).
With ``consider_attitude=True`` dtheta is a Schmidt "consider" state for vision: its
uncertainty and correlation with the bias enter the gain, but vision never corrects it.

Applied vs estimated bias. The IMU is propagated with b_applied (= b_hat unless a shrinkage
floor is set; gravity and vision updates also change b_hat between interval ends). The known
part c = A b_hat - D, with D <- Exp(psi) D + dt b_applied (world; transposed for body), is added
to the residual so it matches the model r = -A (b - b_hat).

GRAVITY (optional absolute roll/pitch reference)
------------------------------------------------
The accelerometer measures specific force f = R^T (a_w - g_w). When the linear acceleration
is small, f ~ -R^T g_w, so u_meas = -mean(a)/||mean(a)|| (window ``gravity_window_s``) is the
gravity direction in body axes. Predicted u_pred = R_hat^T g_unit. With R = Exp(dtheta) R_hat:
    u_meas - u_pred = R_hat^T [g_unit]x dtheta + n,     H = [R_hat^T [g_unit]x,  0]
[g_unit]x has no response to rotation about gravity: heading is unobservable. In addition,
the attitude part of every gravity gain is projected onto the plane orthogonal to gravity,
so correlations cannot leak a heading correction either.
Trust rule (no ground truth): accept only if | ||mean a|| - g | < gravity_norm_tol and the
std of ||a|| over the window < gravity_std_tol. A norm test cannot see sustained
HORIZONTAL acceleration (1 m/s^2 changes ||a|| by only 0.05 m/s^2 but tilts the apparent
gravity by ~6 deg); gravity_sigma_deg has to absorb that.

Noise: bias random walk; gyro white noise density in the attitude propagation; visual noise
per interval (per-interval sigma if given, else sigma_vis_deg) plus gyro noise inside the
interval. Chi-square(3) gating for both update types.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import chi2

from src.data.trajectory import quat_wxyz_to_rotation, rotation_to_quat_wxyz
from src.estimation.inertial_dead_reckoning import DeadReckoningResult, NavState
from vio.estimation.visual_attitude_fusion import integrate_translation


@dataclass
class BiasKFConfig:
    sigma_vis_deg: float = 0.3  # visual relative-rotation noise per interval, per axis (if the interval gives none)
    gyro_noise_density: float = 0.021 * 0.1  # rad/s/sqrt(Hz); Mid-Air per-sample 0.021 rad/s
    bias_walk: float = 1e-5  # rad/s/sqrt(s)
    init_bias_std: float = 0.003  # rad/s (Mid-Air offsets ~0.001, up to 0.006)
    init_att_std_deg: float = 0.2  # attitude uncertainty at the cutoff (GNSS-aided)
    consider_attitude: bool = True  # Schmidt consider state for dtheta (see module docstring)
    gate_prob: float | None = 0.99
    apply_floor_deg_s: float = 0.0  # soft threshold on the bias APPLIED to the gyro; 0 = apply b_hat fully
    gravity_update: bool = False
    gravity_sigma_deg: float = 2.0  # direction noise per update (absorbs unmodelled linear acceleration)
    gravity_norm_tol: float = 0.3  # m/s^2
    gravity_std_tol: float = 0.3  # m/s^2
    gravity_window_s: float = 0.2
    gravity_rate_hz: float = 5.0
    sun_update: bool = False  # absolute sun-direction update (needs sun_world and observations)
    sun_sigma_deg: float = 5.0
    sun_min_confidence: float = 0.5


def soft_threshold_bias(b_hat: np.ndarray, floor_rad_s: float) -> np.ndarray:
    """Per-axis shrinkage: sign(b) * max(|b| - floor, 0). Zero below the floor, shifted by the floor above it."""
    b_hat = np.asarray(b_hat, dtype=float)
    return np.sign(b_hat) * np.maximum(np.abs(b_hat) - floor_rad_s, 0.0)


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


@dataclass
class BiasInterval:
    """One visual relative rotation between IMU samples i and j (absolute flight indices)."""

    imu_i: int
    imu_j: int
    C_body: np.ndarray | None  # R_i^T R_j from the camera; None when the camera failed
    reason: str = ""
    sigma_deg: float | None = None  # per-interval noise (quality-conditioned); None = cfg.sigma_vis_deg


@dataclass
class BiasUpdateLog:
    imu_j: int
    interval_s: float
    accepted: bool
    nis: float
    residual_deg: float
    visual_rotation_deg: float
    reason: str = ""


@dataclass
class GravityLog:
    imu_index: int
    accepted: bool
    norm_deviation: float  # | ||mean a|| - g |, m/s^2
    norm_std: float  # std of ||a|| in the window, m/s^2
    residual_deg: float  # angle between measured and predicted gravity direction
    reason: str = ""


@dataclass
class SunObservation:
    """Detected sun direction in body axes at an IMU sample (absolute flight index)."""

    imu_index: int
    s_body: np.ndarray  # unit vector towards the sun, body axes
    confidence: float


@dataclass
class SunLog:
    imu_index: int
    accepted: bool
    residual_deg: float  # angle between observed and predicted sun direction
    nis: float
    reason: str = ""


@dataclass
class BiasKFOutput:
    result: DeadReckoningResult
    bias: np.ndarray  # (n, 3) rad/s, gyro axes
    bias_std: np.ndarray  # (n, 3)
    updates: list[BiasUpdateLog] = field(default_factory=list)
    bias_applied: np.ndarray | None = None  # (n, 3) bias actually subtracted from the gyro (after shrinkage)
    gravity: list[GravityLog] = field(default_factory=list)
    sun: list[SunLog] = field(default_factory=list)


def run_bias_kf(timestamp: np.ndarray, accelerometer: np.ndarray, gyroscope: np.ndarray, gyroscope_frame: str,
                gravity_world: np.ndarray, initial: NavState, start_index: int, intervals: list[BiasInterval],
                cfg: BiasKFConfig | None = None, sun_observations: list[SunObservation] | None = None,
                sun_world: np.ndarray | None = None) -> BiasKFOutput:
    """Dead reckoning with the gyro bias estimated from relative visual rotations and, optionally,
    roll/pitch corrected by gravity.

    Receives IMU samples from the cutoff, the initial state and camera intervals only: no ground
    truth after the cutoff. Intervals must not overlap.
    """
    if gyroscope_frame not in ("world", "body"):
        raise ValueError(f"gyroscope_frame must be 'world' or 'body', got {gyroscope_frame!r}")
    world = gyroscope_frame == "world"
    cfg = cfg or BiasKFConfig()
    n, k0 = len(timestamp), start_index
    starts, ends = {}, {}
    for iv in intervals:
        i, j = iv.imu_i - k0, iv.imu_j - k0
        if 0 <= i < j < n:
            starts.setdefault(i, []).append(iv)
            ends.setdefault(j, []).append(iv)

    b = np.zeros(3)
    P = np.zeros((6, 6))  # [dtheta, delta_b]
    P[:3, :3] = np.eye(3) * np.deg2rad(cfg.init_att_std_deg) ** 2
    P[3:, 3:] = np.eye(3) * cfg.init_bias_std**2
    qb, qg = cfg.bias_walk**2, cfg.gyro_noise_density**2
    floor = np.deg2rad(cfg.apply_floor_deg_s)
    b_app = soft_threshold_bias(b, floor)
    gate = chi2.ppf(cfg.gate_prob, 3) if cfg.gate_prob is not None else np.inf
    I3 = np.eye(3)
    track_attitude = cfg.consider_attitude or cfg.gravity_update or not world
    g_norm = float(np.linalg.norm(gravity_world))
    g_unit = np.asarray(gravity_world, dtype=float) / g_norm
    proj_tilt = I3 - np.outer(g_unit, g_unit)
    dt_med = float(np.median(np.diff(timestamp)))
    g_every = max(1, int(round(1.0 / (cfg.gravity_rate_hz * dt_med))))
    g_win = max(1, int(round(cfg.gravity_window_s / dt_med)))
    sig_g = np.deg2rad(cfg.gravity_sigma_deg)

    state = {"R": quat_wxyz_to_rotation(initial.attitude), "b": b, "b_app": b_app, "P": P}
    quats = np.empty((n, 4))
    bias_hist, std_hist, app_hist = np.empty((n, 3)), np.empty((n, 3)), np.empty((n, 3))
    open_iv: dict[int, dict] = {}
    logs: list[BiasUpdateLog] = []
    glogs: list[GravityLog] = []
    slogs: list[SunLog] = []
    use_sun = cfg.sun_update and sun_observations and sun_world is not None
    sun_w = np.asarray(sun_world, float) / np.linalg.norm(sun_world) if use_sun else None
    sun_at: dict[int, list[SunObservation]] = {}
    for o in (sun_observations or []) if use_sun else []:
        if 0 < o.imu_index - k0 < n:
            sun_at.setdefault(o.imu_index - k0, []).append(o)
    sig_sun = np.deg2rad(cfg.sun_sigma_deg)

    def kalman(H, r, Rm, attitude_gain: bool, full_attitude: bool = False):
        P = state["P"]
        S = H @ P @ H.T + Rm
        nis = float(r @ np.linalg.solve(S, r))
        if nis > gate:
            return False, nis
        K = P @ H.T @ np.linalg.inv(S)
        if full_attitude:
            pass  # sun: an absolute direction; rotation about the sun ray is unobservable through H itself
        elif attitude_gain:
            K[:3] = proj_tilt @ K[:3]  # gravity never corrects heading, not even through correlations
        else:
            K[:3] = 0.0  # vision is bias-only
        dx = K @ r
        state["R"] = Rotation.from_rotvec(dx[:3]) * state["R"]
        state["b"] = state["b"] + dx[3:]
        state["b_app"] = soft_threshold_bias(state["b"], floor)
        I_KH = np.eye(6) - K @ H
        state["P"] = I_KH @ P @ I_KH.T + K @ Rm @ K.T
        return True, nis

    def gravity(k: int) -> None:
        a = accelerometer[max(0, k - g_win + 1): k + 1]
        am = a.mean(0)
        nm = float(np.linalg.norm(am))
        dev = abs(nm - g_norm)
        sd = float(np.std(np.linalg.norm(a, axis=1)))
        u_meas = -am / nm
        Rinv = state["R"].inv()
        u_pred = Rinv.apply(g_unit)
        ang = float(np.degrees(np.arccos(np.clip(u_meas @ u_pred, -1.0, 1.0))))
        if dev > cfg.gravity_norm_tol or sd > cfg.gravity_std_tol:
            glogs.append(GravityLog(k0 + k, False, dev, sd, ang, "norm" if dev > cfg.gravity_norm_tol else "variance"))
            return
        H = np.hstack([Rinv.as_matrix() @ skew(g_unit), np.zeros((3, 3))])
        ok, _ = kalman(H, u_meas - u_pred, I3 * sig_g**2, attitude_gain=True)
        glogs.append(GravityLog(k0 + k, ok, dev, sd, ang, "" if ok else "chi-square gate"))

    def sun(o: SunObservation, k: int) -> None:
        s_meas = np.asarray(o.s_body, float) / np.linalg.norm(o.s_body)
        Rinv = state["R"].inv()
        s_pred = Rinv.apply(sun_w)
        ang = float(np.degrees(np.arccos(np.clip(s_meas @ s_pred, -1.0, 1.0))))
        if o.confidence < cfg.sun_min_confidence:
            slogs.append(SunLog(k0 + k, False, ang, np.nan, "low confidence"))
            return
        # R_true = Exp(dtheta) R_hat -> R_true^T s_w = s_pred + R_hat^T [s_w]x dtheta  (no position/velocity term)
        H = np.hstack([Rinv.as_matrix() @ skew(sun_w), np.zeros((3, 3))])
        ok, nis = kalman(H, s_meas - s_pred, np.eye(3) * sig_sun**2, attitude_gain=True, full_attitude=True)
        slogs.append(SunLog(k0 + k, ok, ang, nis, "" if ok else "chi-square gate"))

    def process(k: int) -> None:
        if cfg.gravity_update and k > 0 and k % g_every == 0:
            gravity(k)
        for o in sun_at.get(k, []):
            sun(o, k)
        for iv in ends.get(k, []):
            i = iv.imu_i - k0
            acc = open_iv.pop(i, None)
            dt_iv = float(timestamp[k] - timestamp[i])
            if iv.C_body is None or acc is None:
                logs.append(BiasUpdateLog(iv.imu_j, dt_iv, False, np.nan, np.nan, np.nan, iv.reason or "camera failed"))
                continue
            C = Rotation.from_matrix(iv.C_body)
            b_now = state["b"]
            if world:
                M, A, D, Ri = acc["M"], acc["A"], acc["D"], acc["R_i"]
                r = (Ri * C * Ri.inv() * Rotation.from_matrix(M).inv()).as_rotvec() + (A @ b_now - D)
                H = np.hstack([I3 - M.T, -M.T @ A]) if cfg.consider_attitude else np.hstack([np.zeros((3, 3)), -A])
            else:
                Pb, A, D = acc["Pb"], acc["A"], acc["D"]
                r = (Rotation.from_matrix(Pb).inv() * C).as_rotvec() + (A @ b_now - D)
                H = np.hstack([np.zeros((3, 3)), -A])
            sig = np.deg2rad(iv.sigma_deg if iv.sigma_deg is not None else cfg.sigma_vis_deg)
            Rm = I3 * (sig**2 + qg * dt_iv)
            ok, nis = kalman(H, r, Rm, attitude_gain=False)
            logs.append(BiasUpdateLog(iv.imu_j, dt_iv, ok, nis, float(np.degrees(np.linalg.norm(r))),
                                      float(np.degrees(C.magnitude())), "" if ok else "chi-square gate"))
        if k in starts:
            open_iv[k] = ({"M": I3.copy(), "A": np.zeros((3, 3)), "D": np.zeros(3), "R_i": state["R"]} if world
                          else {"Pb": I3.copy(), "A": np.zeros((3, 3)), "D": np.zeros(3)})
        quats[k] = state["R"].as_quat()
        bias_hist[k], std_hist[k], app_hist[k] = state["b"], np.sqrt(np.diag(state["P"])[3:]), state["b_app"]

    process(0)
    Phi = np.eye(6)
    for k in range(n - 1):
        dt = float(timestamp[k + 1] - timestamp[k])
        b_app = state["b_app"]
        inc = Rotation.from_rotvec((0.5 * (gyroscope[k] + gyroscope[k + 1]) - b_app) * dt)
        Minc = inc.as_matrix()
        if world:
            state["R"] = inc * state["R"]
            for acc in open_iv.values():
                acc["M"] = Minc @ acc["M"]
                acc["A"] = Minc @ acc["A"] + dt * I3
                acc["D"] = Minc @ acc["D"] + dt * b_app
            Phi[:3, :3], Phi[:3, 3:] = Minc, -dt * I3
        else:
            Rprev = state["R"].as_matrix()
            state["R"] = state["R"] * inc
            for acc in open_iv.values():
                acc["Pb"] = acc["Pb"] @ Minc
                acc["A"] = Minc.T @ acc["A"] + dt * I3
                acc["D"] = Minc.T @ acc["D"] + dt * b_app
            Phi[:3, :3], Phi[:3, 3:] = I3, -dt * Rprev
        P = state["P"]
        if track_attitude:
            P = Phi @ P @ Phi.T
            P[:3, :3] += I3 * qg * dt
        P[3:, 3:] += I3 * qb * dt
        state["P"] = P
        process(k + 1)

    att = Rotation.from_quat(quats)
    position, velocity = integrate_translation(timestamp, accelerometer, att, initial, gravity_world)
    res = DeadReckoningResult(timestamp, position, velocity, rotation_to_quat_wxyz(att), start_index=k0, t0=float(timestamp[0]))
    return BiasKFOutput(res, bias_hist, std_hist, logs, app_hist, glogs, slogs)

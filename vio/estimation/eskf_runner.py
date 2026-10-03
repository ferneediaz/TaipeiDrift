"""Run the ESKF over a flight after the GNSS cutoff: IMU prediction, visual updates.

The runner receives only what a deployed system would have after the cutoff:
IMU samples, the initial state at the cutoff, camera measurements, and the
simulated barometer's height-above-ground estimate. It never receives a
Trajectory, so ground truth after the cutoff cannot reach it.

Event order at an IMU sample k:
    predict to k  ->  relative-rotation update (forward camera, end of a keyframe span)
                  ->  flow-velocity update (down camera)
                  ->  clone the attitude if k starts a new forward-camera keyframe span
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from src.data.trajectory import rotation_to_quat_wxyz
from src.estimation.inertial_dead_reckoning import DeadReckoningResult, NavState
from vio.estimation.eskf import ESKF, N_ERR, ImuNoiseModel
from vio.vision.measurements import VisualMeasurement
from vio.vision.optical_flow import FlowConfig, FlowPair, camera_velocity_from_flow


@dataclass
class RotationUpdateConfig:
    enabled: bool = True
    sigma_deg: float = 0.2  # per axis, for one keyframe span
    gate_prob: float = 0.99


@dataclass
class FlowUpdateConfig:
    enabled: bool = True
    rel_height_std: float = 0.3  # relative uncertainty of height above ground (terrain unknown)
    dir_sigma_deg: float = 1.5  # direction noise of the measured velocity (slopes, derotation)
    min_sigma_mps: float = 0.05  # floor on the velocity noise
    every_n_frames: int = 1  # use one pair in n (consecutive pairs have correlated errors)
    gate_prob: float = 0.99
    flow: FlowConfig = field(default_factory=FlowConfig)


@dataclass
class BaroUpdateConfig:
    """Relative-altitude update (vertical position only). Noise follows the measured barometer:
    sigma^2 = white^2 + rw^2 t + (drift t)^2 + (scale |dz|)^2, with t since the cut and dz the
    barometric altitude change since the cut. No bias state: drift and scale are folded into the noise."""

    enabled: bool = True
    white_std_m: float = 0.30
    bias_walk_m_per_sqrt_s: float = 0.112
    drift_sigma_m_per_s: float = 0.0024
    scale_error_rms: float = 0.052  # rms of +/- U(0.03, 0.07)
    every_n_samples: int = 20  # 5 Hz
    gate_prob: float = 0.99


@dataclass
class DirectionUpdateConfig:
    """Visual translation DIRECTION as a velocity-direction measurement (no speed information).

    sigma_deg: per-axis direction noise; the Mid-Air diagnostic gives a median angular error of
    1.0 deg (p90 2.7 deg) over 0.5 s spans, i.e. ~0.85 deg per axis; 1.5 deg is used to cover the
    heavier tail and the mean-velocity approximation. Updates need ||v_hat|| >= min_speed_mps.
    """

    enabled: bool = True
    sigma_deg: float = 1.5
    min_speed_mps: float = 0.5
    gate_prob: float = 0.99


@dataclass
class UpdateLog:
    kind: str  # "rotation", "flow", "baro" or "direction"
    imu_index: int  # absolute flight index
    accepted: bool
    nis: float
    dof: int
    reason: str


@dataclass
class EskfInputs:
    """Everything the estimator may use after the cutoff."""

    timestamp: np.ndarray  # (n,) from the cutoff
    accelerometer: np.ndarray
    gyroscope: np.ndarray
    gyroscope_frame: str
    gravity_world: np.ndarray
    initial: NavState  # the only ground truth: the state at the cutoff
    start_index: int  # absolute flight index of timestamp[0]
    rotation_measurements: list[VisualMeasurement] | None = None
    R_bc_forward: np.ndarray | None = None
    flow_pairs: list[FlowPair] | None = None
    R_bc_down: np.ndarray | None = None
    focal_px_down: float = 1.0  # focal length of the processed down image, for residuals in pixels
    height_above_ground: np.ndarray | None = None  # (n,) metres, from the simulated barometer
    baro_altitude_change: np.ndarray | None = None  # (n,) metres up since the cutoff, simulated barometer


@dataclass
class EskfOutput:
    result: DeadReckoningResult
    gyro_bias: np.ndarray  # (n, 3)
    accel_bias: np.ndarray  # (n, 3)
    std: np.ndarray  # (n, 15) square roots of the covariance diagonal
    nav_cov_index: np.ndarray  # local indices where the 15x15 covariance was stored
    nav_cov: np.ndarray  # (m, 15, 15)
    updates: list[UpdateLog]


def run_eskf(inp: EskfInputs, noise: ImuNoiseModel | None = None, rot_cfg: RotationUpdateConfig | None = None,
             flow_cfg: FlowUpdateConfig | None = None, cov_every: int = 25,
             baro_cfg: BaroUpdateConfig | None = None, dir_cfg: DirectionUpdateConfig | None = None) -> EskfOutput:
    """Filter from the cutoff to the end of the flight."""
    rot_cfg = rot_cfg or RotationUpdateConfig(enabled=False)
    flow_cfg = flow_cfg or FlowUpdateConfig(enabled=False)
    baro_cfg = baro_cfg or BaroUpdateConfig(enabled=False)
    dir_cfg = dir_cfg or DirectionUpdateConfig(enabled=False)
    use_baro = baro_cfg.enabled and inp.baro_altitude_change is not None
    use_dir = dir_cfg.enabled and inp.rotation_measurements is not None and inp.R_bc_forward is not None
    up = -inp.gravity_world / np.linalg.norm(inp.gravity_world)  # world 'up'
    alt0 = float(up @ np.asarray(inp.initial.position))
    n, k0 = len(inp.timestamp), inp.start_index
    f = ESKF(inp.initial.position, inp.initial.velocity, inp.initial.attitude, inp.gravity_world,
             inp.gyroscope_frame, noise)

    use_rot = rot_cfg.enabled and inp.rotation_measurements is not None and inp.R_bc_forward is not None
    use_flow = (flow_cfg.enabled and inp.flow_pairs is not None and inp.R_bc_down is not None
                and inp.height_above_ground is not None)
    rot_at: dict[int, list[VisualMeasurement]] = {}
    keyframes: set[int] = set()
    if use_rot:
        for m in inp.rotation_measurements:
            keyframes.add(m.keyframe_imu_index - k0)
            if m.end_of_span:
                rot_at.setdefault(m.imu_index - k0, []).append(m)
    dir_at: dict[int, list[VisualMeasurement]] = {}
    if use_dir:
        for m in inp.rotation_measurements:
            if m.end_of_span:
                dir_at.setdefault(m.imu_index - k0, []).append(m)
    flow_at = {p.imu_index - k0: p for i, p in enumerate(inp.flow_pairs or []) if i % max(1, flow_cfg.every_n_frames) == 0}         if use_flow else {}
    clone_index = None
    sigma_rot = np.deg2rad(rot_cfg.sigma_deg)

    quats = np.empty((n, 4))
    pos, vel = np.empty((n, 3)), np.empty((n, 3))
    bg, ba, std = np.empty((n, 3)), np.empty((n, 3)), np.empty((n, N_ERR))
    cov_idx, covs, logs = [], [], []

    def record(k: int) -> None:
        quats[k], pos[k], vel[k], bg[k], ba[k] = f.R.as_quat(), f.p, f.v, f.bg, f.ba
        std[k] = np.sqrt(np.clip(np.diag(f.P)[:N_ERR], 0, None))
        if k % cov_every == 0 or k == n - 1:
            cov_idx.append(k)
            covs.append(f.P[:N_ERR, :N_ERR].copy())

    def at_sample(k: int) -> None:
        nonlocal clone_index
        for m in rot_at.get(k, []):
            kf = m.keyframe_imu_index - k0
            if not m.valid or not m.body_rotation_candidates:
                logs.append(UpdateLog("rotation", m.imu_index, False, np.nan, 3, m.reason or "invalid"))
            elif clone_index != kf:
                logs.append(UpdateLog("rotation", m.imu_index, False, np.nan, 3, "no clone for this keyframe"))
            else:
                # with several candidates take the one closest to the filter's prediction
                C_hat = f.R_clone.inv() * f.R
                C = min(m.body_rotation_candidates,
                        key=lambda c: (C_hat.inv() * Rotation.from_matrix(c)).magnitude())
                u = f.update_relative_rotation(C, sigma_rot, rot_cfg.gate_prob)
                logs.append(UpdateLog("rotation", m.imu_index, u.accepted, u.nis, u.dof, u.reason))
        for m in dir_at.get(k, []):
            logs.append(_direction_update(f, m, k, k0, quats, vel, inp, dir_cfg))
        pair = flow_at.get(k)
        if pair is not None:
            logs.append(_flow_update(f, pair, k, k0, quats, inp, flow_cfg))
        if use_baro and k > 0 and k % max(1, baro_cfg.every_n_samples) == 0:
            t = float(inp.timestamp[k] - inp.timestamp[0])
            dz = float(inp.baro_altitude_change[k])
            sigma = np.sqrt(baro_cfg.white_std_m**2 + baro_cfg.bias_walk_m_per_sqrt_s**2 * t
                            + (baro_cfg.drift_sigma_m_per_s * t) ** 2 + (baro_cfg.scale_error_rms * dz) ** 2)
            u = f.update_altitude(alt0 + dz, up, sigma, baro_cfg.gate_prob)
            logs.append(UpdateLog("baro", k0 + k, u.accepted, u.nis, u.dof, u.reason))
        if k in keyframes:
            f.clone_attitude()
            clone_index = k

    record(0)
    at_sample(0)
    for k in range(n - 1):
        f.predict(inp.accelerometer[k], inp.accelerometer[k + 1], inp.gyroscope[k], inp.gyroscope[k + 1],
                  float(inp.timestamp[k + 1] - inp.timestamp[k]))
        at_sample(k + 1)
        record(k + 1)

    att = rotation_to_quat_wxyz(Rotation.from_quat(quats))
    result = DeadReckoningResult(inp.timestamp, pos, vel, att, start_index=k0, t0=float(inp.timestamp[0]))
    return EskfOutput(result, bg, ba, std, np.array(cov_idx), np.array(covs), logs)


def direction_jacobian(v_hat: np.ndarray, d_vis_world: np.ndarray, n: int):
    """Residual and Jacobian of the velocity-direction measurement.

    d_pred = v_hat / ||v_hat||; B = [e1 e2] spans the plane orthogonal to d_pred.
    Residual r = B^T d_vis (predicted value B^T d_pred = 0).  With R_true = Exp(dtheta) R_hat and
    d_vis computed with R_hat:  d_vis ~ d_true - dtheta x d_true, d_true ~ d_pred + (I - d d^T) dv / ||v||, so
        r = B^T dv / ||v||  +  B^T [d_pred]x dtheta  + noise.
    B^T v_hat = 0: no sensitivity along the velocity, the speed magnitude stays unobserved.
    """
    from vio.estimation.eskf import BA, BG, P_, TH, V_, skew  # noqa: F401
    speed = float(np.linalg.norm(v_hat))
    d = v_hat / speed
    a = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(d, a)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(d, e1)
    B = np.column_stack([e1, e2])
    r = B.T @ (d_vis_world / np.linalg.norm(d_vis_world))
    H = np.zeros((2, n))
    H[:, V_] = B.T / speed
    H[:, TH] = B.T @ skew(d)
    return r, H, B


def _direction_update(f: ESKF, m: VisualMeasurement, k: int, k0: int, quats: np.ndarray, vel: np.ndarray,
                      inp: EskfInputs, cfg: DirectionUpdateConfig) -> UpdateLog:
    if not m.valid or m.translation_dir_cam is None:
        return UpdateLog("direction", m.imu_index, False, np.nan, 2, m.reason or "no translation direction")
    kf = m.keyframe_imu_index - k0
    if kf < 0 or kf >= k:
        return UpdateLog("direction", m.imu_index, False, np.nan, 2, "keyframe before cutoff")
    # mean predicted velocity over the span (the camera measures the displacement direction)
    v_mean = np.vstack([vel[kf:k], f.v[None]]).mean(0)
    if np.linalg.norm(v_mean) < cfg.min_speed_mps:
        return UpdateLog("direction", m.imu_index, False, np.nan, 2, "speed below threshold")
    R_kf = Rotation.from_quat(quats[kf])  # estimator attitude at the keyframe (camera i axes)
    d_vis = R_kf.apply(inp.R_bc_forward @ m.translation_dir_cam)
    r, H, _ = direction_jacobian(v_mean, d_vis, f.n)
    u = f.update(r, H, np.eye(2) * np.deg2rad(cfg.sigma_deg) ** 2, cfg.gate_prob)
    return UpdateLog("direction", m.imu_index, u.accepted, u.nis, u.dof, u.reason)


def _flow_update(f: ESKF, pair: FlowPair, k: int, k0: int, quats: np.ndarray, inp: EskfInputs,
                 cfg: FlowUpdateConfig) -> UpdateLog:
    if not pair.valid:
        return UpdateLog("flow", pair.imu_index, False, np.nan, 2, pair.reason)
    ka = pair.prev_imu_index - k0
    if ka < 0:
        return UpdateLog("flow", pair.imu_index, False, np.nan, 2, "previous frame before cutoff")
    R_bc = inp.R_bc_down
    Ra = Rotation.from_quat(quats[ka]).as_matrix()
    Rb = f.R.as_matrix()
    R_ab_cam = R_bc.T @ (Ra.T @ Rb) @ R_bc  # from the filter: bias-corrected gyro, never raw
    n_cam = (Ra @ R_bc).T @ np.array([0.0, 0.0, 1.0])  # world down in camera a (NED)
    h = float(inp.height_above_ground[k])
    dt = float(inp.timestamp[k] - inp.timestamp[ka])
    fv = camera_velocity_from_flow(pair.xa, pair.xb, R_ab_cam, n_cam, h, dt, inp.focal_px_down, cfg.flow)
    if fv is None:
        return UpdateLog("flow", pair.imu_index, False, np.nan, 2, "flow fit rejected")
    # A wrong height scales the measured velocity along itself: the height uncertainty enters
    # only along the measured direction u, the direction itself stays well determined.
    z = fv.v_cam[:2]
    speed = max(float(np.linalg.norm(z)), float(np.linalg.norm((R_bc.T @ Rb.T @ f.v)[:2])))
    u = z / max(np.linalg.norm(z), 1e-9)
    uu = np.outer(u, u)
    Rm = (fv.cov_xy + np.eye(2) * cfg.min_sigma_mps**2 + (speed * cfg.rel_height_std) ** 2 * uu
          + (speed * np.deg2rad(cfg.dir_sigma_deg)) ** 2 * (np.eye(2) - uu))
    res = f.update_camera_velocity_xy(z, R_bc, Rm, cfg.gate_prob)
    return UpdateLog("flow", pair.imu_index, res.accepted, res.nis, res.dof, res.reason)

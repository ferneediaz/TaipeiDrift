"""Gyro-bias experiment: synthetic cases, real-data visual intervals, drift-slope metrics.

Synthetic: known motion, ideal attitude, IMU with a known constant WORLD-frame gyro bias,
and an independent simulated camera whose relative rotations come from the TRUE attitude
plus noise (never from the biased gyro).

Real data: visual intervals come from the unchanged front end (Shi-Tomasi, Lucas-Kanade,
essential matrix + RANSAC, color_left). The keyframe tracker gives keyframe-relative
rotations; a 1.0 s interval is a full 25-frame keyframe span (end-of-span measurement), a
0.5 s interval is the first 12 frames of each span. Both are non-overlapping.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from src.data.synthetic import ImuNoise, make_synthetic_trajectory
from src.data.trajectory import Trajectory, quat_wxyz_to_rotation
from src.estimation.inertial_dead_reckoning import NavState
from vio.estimation.gyro_bias_kf import BiasInterval, BiasKFConfig, BiasKFOutput, run_bias_kf
from vio.vision.measurements import VisualMeasurement

DEG = np.pi / 180.0


# ------------------------------------------------------------------ synthetic
def synthetic_case(bias_deg_s, duration: float = 120.0, interval_s: float = 1.0, vis_sigma_deg: float = 0.005,
                   gyro_white_deg_s: float = 0.0, fail_every: int = 0, seed: int = 0, scenario: str = "circle"):
    """Trajectory with a world-frame gyro bias, plus camera intervals from the true attitude."""
    bias = tuple(np.asarray(bias_deg_s, dtype=float) * DEG)
    traj = make_synthetic_trajectory(scenario, duration=duration, gyroscope_frame="world",
                                     noise=ImuNoise(gyro_bias=bias, gyro_white_std=gyro_white_deg_s * DEG, seed=seed))
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    rng = np.random.default_rng(seed + 1)
    k0 = traj.index_at(5.0)
    step = int(round(interval_s * 100))
    intervals = []
    for n, i in enumerate(range(k0, len(traj) - step, step)):
        j = i + step
        if fail_every and n % fail_every == 0:
            intervals.append(BiasInterval(i, j, None, "simulated camera failure"))
            continue
        C = (R[i].inv() * R[j]) * Rotation.from_rotvec(rng.normal(0, vis_sigma_deg * DEG, 3))
        intervals.append(BiasInterval(i, j, C.as_matrix()))
    return traj, intervals, np.asarray(bias)


def run_on_trajectory(traj: Trajectory, cutoff: float, intervals: list[BiasInterval], cfg: BiasKFConfig) -> BiasKFOutput:
    """The only ground truth handed over is the state at the cutoff."""
    k0 = traj.index_at(cutoff)
    initial = NavState(traj.position_gt[k0].copy(), traj.velocity_gt[k0].copy(), traj.attitude_gt[k0].copy())
    return run_bias_kf(traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], traj.gyroscope_frame,
                       traj.gravity_world, initial, k0, intervals, cfg)


# ------------------------------------------------------------------ real data
def intervals_from_measurements(measurements: list[VisualMeasurement], interval_frames: int, step: int = 4) -> list[BiasInterval]:
    """Non-overlapping intervals of ``interval_frames`` frames from keyframe-relative measurements.

    For each keyframe, take the measurement exactly ``interval_frames`` after it. If the
    span ends earlier (tracks lost), use its end-of-span measurement when it covers at
    least half the interval. Invalid measurements become failed intervals (IMU fallback).
    """
    by_kf: dict[int, list[VisualMeasurement]] = {}
    for m in measurements:
        if not m.new_keyframe:
            by_kf.setdefault(m.keyframe_index, []).append(m)
    out = []
    for kf, ms in sorted(by_kf.items()):
        exact = [m for m in ms if m.frame_index - kf == interval_frames]
        if exact:
            m = exact[0]
        else:
            ends = [m for m in ms if m.end_of_span and m.frame_index - kf >= interval_frames // 2]
            if not ends:
                continue
            m = ends[0]
        if m.valid and m.body_rotation_candidates:
            out.append(BiasInterval(m.keyframe_imu_index, m.imu_index, m.body_rotation_candidates[0]))
        else:
            out.append(BiasInterval(m.keyframe_imu_index, m.imu_index, None, m.reason or "invalid"))
    return out


# ------------------------------------------------------------------ metrics
def slope_deg_per_s(t: np.ndarray, err_deg: np.ndarray, t0: float, t1: float) -> float:
    """Least-squares slope of attitude error over [t0, t1) seconds since GNSS loss."""
    m = (t >= t0) & (t < t1)
    if m.sum() < 10:
        return float("nan")
    return float(np.polyfit(t[m], err_deg[m], 1)[0])


GROWTH_WINDOWS = [(5.0, 30.0), (30.0, 60.0), (60.0, float("inf"))]


def growth_rates(t: np.ndarray, err_deg: np.ndarray) -> dict:
    return {f"{a:g}-{'end' if b == float('inf') else f'{b:g}'}s": slope_deg_per_s(t, err_deg, a, b) for a, b in GROWTH_WINDOWS}

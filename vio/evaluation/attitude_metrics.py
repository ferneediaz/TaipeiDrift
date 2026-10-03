"""Attitude error: geodesic angle between estimated and true orientation."""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from src.data.trajectory import Trajectory
from src.estimation.inertial_dead_reckoning import DeadReckoningResult


def geodesic_angle_deg(q_est: np.ndarray, q_gt: np.ndarray) -> np.ndarray:
    """Rotation angle of R_est^T R_gt, in degrees, for (N, 4) or (4,) quaternions (w, x, y, z).

    Uses 2 atan2(|v|, |w|) on the relative quaternion: insensitive to the sign of
    either quaternion (q and -q are the same rotation) and accurate near zero.
    """
    a = np.atleast_2d(np.asarray(q_est, dtype=float))
    b = np.atleast_2d(np.asarray(q_gt, dtype=float))
    a = a / np.linalg.norm(a, axis=1, keepdims=True)
    b = b / np.linalg.norm(b, axis=1, keepdims=True)
    # relative quaternion conj(a) * b
    w1, v1 = a[:, 0], -a[:, 1:]
    w2, v2 = b[:, 0], b[:, 1:]
    w = w1 * w2 - np.sum(v1 * v2, axis=1)
    v = w1[:, None] * v2 + w2[:, None] * v1 + np.cross(v1, v2)
    angle = np.degrees(2.0 * np.arctan2(np.linalg.norm(v, axis=1), np.abs(w)))
    return angle if np.ndim(q_est) > 1 else angle[0]


@dataclass
class AttitudeErrorSeries:
    time_since_loss: np.ndarray  # (M,) s
    error_deg: np.ndarray  # (M,)


@dataclass
class AttitudeSummary:
    final_deg: float
    max_deg: float
    mean_deg: float
    rmse_deg: float

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


def attitude_error_series(result: DeadReckoningResult, traj: Trajectory) -> AttitudeErrorSeries:
    """Attitude error at every estimated sample."""
    q_gt = traj.attitude_gt[result.start_index : result.start_index + len(result.timestamp)]
    return AttitudeErrorSeries(result.timestamp - result.t0, geodesic_angle_deg(result.attitude, q_gt))


def summarize_attitude(series: AttitudeErrorSeries) -> AttitudeSummary:
    e = series.error_deg
    return AttitudeSummary(float(e[-1]), float(e.max()), float(e.mean()), float(np.sqrt(np.mean(e**2))))


def attitude_at_horizons(series: AttitudeErrorSeries, horizons: list[float]) -> dict[float, float | None]:
    """Attitude error at fixed times after GNSS loss; None beyond the end of the flight."""
    return {h: (None if h > series.time_since_loss[-1] else float(np.interp(h, series.time_since_loss, series.error_deg)))
            for h in horizons}


def tilt_heading_error_deg(q_est: np.ndarray, q_gt: np.ndarray, gravity_unit_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split the attitude error into tilt and heading without Euler angles (swing-twist about gravity).

    Error E = R_gt R_est^T (world side). Twist = the part of E about the gravity axis (heading,
    signed, deg); swing = E twist^-1 (tilt: the angle by which the estimated gravity direction is off).
    """
    from scipy.spatial.transform import Rotation as _R
    a = np.asarray(gravity_unit_world, float) / np.linalg.norm(gravity_unit_world)
    E = _R.from_quat(np.atleast_2d(q_gt)[:, [1, 2, 3, 0]]) * _R.from_quat(np.atleast_2d(q_est)[:, [1, 2, 3, 0]]).inv()
    q = E.as_quat()  # x, y, z, w
    w, v = q[:, 3], q[:, :3]
    p = v @ a
    heading = np.degrees(2.0 * np.arctan2(p, w))
    heading = (heading + 180.0) % 360.0 - 180.0
    twist = _R.from_rotvec(np.radians(heading)[:, None] * a)
    tilt = np.degrees((E * twist.inv()).magnitude())
    return tilt, heading

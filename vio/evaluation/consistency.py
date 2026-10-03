"""Filter consistency (NIS, NEES), trajectory metrics (ATE, RPE) and IMU bias truth.

Everything here is evaluation: it may read ground truth, and nothing here is
called by the estimator.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import chi2

from src.data.trajectory import Trajectory, quat_wxyz_to_rotation
from src.estimation.inertial_dead_reckoning import DeadReckoningResult


# ----------------------------------------------------------------- NIS / NEES
def nis(r: np.ndarray, S: np.ndarray) -> float:
    """Normalised innovation squared r^T S^-1 r."""
    r = np.atleast_1d(r)
    return float(r @ np.linalg.solve(S, r))


def nees(e: np.ndarray, P: np.ndarray) -> float:
    """Normalised estimation error squared e^T P^-1 e."""
    e = np.atleast_1d(e)
    return float(e @ np.linalg.solve(P, e))


def chi2_interval(dof: int, n_samples: int = 1, prob: float = 0.95) -> tuple[float, float]:
    """Two-sided interval for the MEAN of n chi-square(dof) samples (a consistent filter falls inside)."""
    lo = chi2.ppf((1 - prob) / 2, dof * n_samples) / n_samples
    hi = chi2.ppf(1 - (1 - prob) / 2, dof * n_samples) / n_samples
    return float(lo), float(hi)


@dataclass
class ConsistencySummary:
    dof: int
    count: int
    mean: float
    median: float
    expected: float  # = dof
    fraction_inside_95: float  # share of single values inside the two-sided 95 % chi-square bounds

    def as_dict(self) -> dict:
        return self.__dict__.copy()


def summarize_chi2(values: np.ndarray, dof: int) -> ConsistencySummary | None:
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return None
    lo, hi = chi2_interval(dof)
    return ConsistencySummary(dof, int(v.size), float(v.mean()), float(np.median(v)), float(dof),
                              float(np.mean((v >= lo) & (v <= hi))))


def state_error(est_p, est_v, est_q_wxyz, gt_p, gt_v, gt_q_wxyz) -> np.ndarray:
    """(m, 9) error in the filter's convention: true minus estimate, attitude via R_true = Exp(e) R_est."""
    est_q, gt_q = np.atleast_2d(est_q_wxyz), np.atleast_2d(gt_q_wxyz)
    dth = (quat_wxyz_to_rotation(gt_q) * quat_wxyz_to_rotation(est_q).inv()).as_rotvec()
    return np.column_stack([np.atleast_2d(gt_p) - np.atleast_2d(est_p), np.atleast_2d(gt_v) - np.atleast_2d(est_v), dth])


# ---------------------------------------------------------------- ATE / RPE
def ate_rmse(result: DeadReckoningResult, traj: Trajectory) -> float:
    """Absolute trajectory error: RMSE of position, no alignment (the start is shared at the cutoff)."""
    gt = traj.position_gt[result.start_index: result.start_index + len(result.timestamp)]
    return float(np.sqrt(np.mean(np.sum((result.position - gt) ** 2, axis=1))))


def rpe(result: DeadReckoningResult, traj: Trajectory, delta_s: float = 10.0) -> dict[str, float]:
    """Relative pose error over windows of ``delta_s`` seconds (translation in m, rotation in deg)."""
    k0, n = result.start_index, len(result.timestamp)
    dt = float(np.median(np.diff(result.timestamp)))
    d = int(round(delta_s / dt))
    if d >= n:
        return {"delta_s": delta_s, "trans_rmse_m": float("nan"), "trans_median_m": float("nan"), "rot_rmse_deg": float("nan")}
    gt_p = traj.position_gt[k0:k0 + n]
    trans = np.linalg.norm((result.position[d:] - result.position[:-d]) - (gt_p[d:] - gt_p[:-d]), axis=1)
    Re, Rg = quat_wxyz_to_rotation(result.attitude), quat_wxyz_to_rotation(traj.attitude_gt[k0:k0 + n])
    rel_e = Re[:-d].inv() * Re[d:]
    rel_g = Rg[:-d].inv() * Rg[d:]
    rot = np.degrees((rel_e.inv() * rel_g).magnitude())
    return {"delta_s": delta_s, "trans_rmse_m": float(np.sqrt(np.mean(trans**2))),
            "trans_median_m": float(np.median(trans)), "rot_rmse_deg": float(np.sqrt(np.mean(rot**2)))}


# ------------------------------------------------------------- bias truth
def native_imu_bias(traj: Trajectory, window_s: float = 10.0) -> tuple[np.ndarray, np.ndarray]:
    """Slowly varying IMU bias already present in the data, from ground truth (evaluation only).

    Gyro: measured rate minus the true rate, in the gyroscope's axes. The true
    rate is the change of the true attitude (world or body side to match
    ``gyroscope_frame``). Accelerometer: measured minus true specific force,
    R^T (dv/dt - g), body frame. Both are smoothed by a centred moving average
    of ``window_s`` to remove white noise.
    """
    t = traj.timestamp
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    dt = np.diff(t)[:, None]
    if traj.gyroscope_frame == "world":
        rate = (R[1:] * R[:-1].inv()).as_rotvec() / dt
    else:
        rate = (R[:-1].inv() * R[1:]).as_rotvec() / dt
    gyro_mid = 0.5 * (traj.gyroscope[:-1] + traj.gyroscope[1:])
    g_err = np.vstack([gyro_mid - rate, (gyro_mid - rate)[-1:]])
    acc_w = np.gradient(traj.velocity_gt, t, axis=0)
    f_true = R.inv().apply(acc_w - traj.gravity_world)
    a_err = traj.accelerometer - f_true
    w = max(1, int(round(window_s / float(np.median(dt)))))
    return _centred_mean(g_err, w), _centred_mean(a_err, w)


def _centred_mean(x: np.ndarray, w: int) -> np.ndarray:
    """Centred moving average that shrinks the window at the ends instead of padding."""
    n = len(x)
    c = np.vstack([np.zeros((1, x.shape[1])), np.cumsum(x, axis=0)])
    lo = np.clip(np.arange(n) - w // 2, 0, n)
    hi = np.clip(np.arange(n) + (w - w // 2), 0, n)
    return (c[hi] - c[lo]) / (hi - lo)[:, None]

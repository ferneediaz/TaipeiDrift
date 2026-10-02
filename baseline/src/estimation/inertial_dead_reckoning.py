"""Strapdown inertial dead reckoning: the GNSS-denied baseline.

At the GNSS cutoff t0 the estimator is initialised once from the true
position, velocity and attitude. After t0 it uses only the IMU:

    R_{k+1} = R_k * Exp(0.5 (w_k + w_{k+1}) dt)        attitude, midpoint gyro
    a_k     = R_k f_k + g_world                         specific force -> world acceleration
    v_{k+1} = v_k + 0.5 (a_k + a_{k+1}) dt              trapezoid
    p_{k+1} = p_k + 0.5 (v_k + v_{k+1}) dt              trapezoid

Conventions are those of ``src.data.trajectory``. No GNSS and no ground truth
after t0 are read.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from src.data.trajectory import Trajectory, quat_wxyz_to_rotation, rotation_to_quat_wxyz


@dataclass
class NavState:
    """Navigation state in the world frame."""

    position: np.ndarray  # (3,) m
    velocity: np.ndarray  # (3,) m/s
    attitude: np.ndarray  # (4,) quaternion (w, x, y, z), body -> world


@dataclass
class DeadReckoningResult:
    """Estimated trajectory from the GNSS cutoff to the end of the flight."""

    timestamp: np.ndarray  # (M,) s
    position: np.ndarray  # (M, 3) m
    velocity: np.ndarray  # (M, 3) m/s
    attitude: np.ndarray  # (M, 4) quaternion (w, x, y, z)
    start_index: int  # index into the source trajectory of the first sample
    t0: float  # time of the first sample, i.e. when GNSS was lost


def propagate(
    timestamp: np.ndarray,
    accelerometer: np.ndarray,
    gyroscope: np.ndarray,
    initial: NavState,
    gravity_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate IMU samples from an initial state.

    Args:
        timestamp: (M,) seconds, strictly increasing; ``initial`` holds at timestamp[0].
        accelerometer: (M, 3) specific force in body frame, m/s^2.
        gyroscope: (M, 3) body angular velocity, rad/s.
        initial: state at timestamp[0].
        gravity_world: (3,) gravity vector in the world frame, m/s^2.

    Returns:
        position (M, 3), velocity (M, 3), attitude (M, 4) as (w, x, y, z).
    """
    dt = np.diff(timestamp)
    if np.any(dt <= 0):
        raise ValueError("timestamps must be strictly increasing")

    increments = Rotation.from_rotvec(0.5 * (gyroscope[:-1] + gyroscope[1:]) * dt[:, None])
    quats = np.empty((len(timestamp), 4))  # scipy order (x, y, z, w) while integrating
    r = quat_wxyz_to_rotation(initial.attitude)
    quats[0] = r.as_quat()
    for k in range(len(dt)):
        r = r * increments[k]  # right-multiply: the increment is in the body frame
        quats[k + 1] = r.as_quat()
    attitude = Rotation.from_quat(quats)

    accel_world = attitude.apply(accelerometer) + gravity_world
    velocity = np.vstack(
        [initial.velocity, initial.velocity + np.cumsum(0.5 * (accel_world[:-1] + accel_world[1:]) * dt[:, None], axis=0)]
    )
    position = np.vstack(
        [initial.position, initial.position + np.cumsum(0.5 * (velocity[:-1] + velocity[1:]) * dt[:, None], axis=0)]
    )
    return position, velocity, rotation_to_quat_wxyz(attitude)


def run_dead_reckoning(traj: Trajectory, gnss_cutoff: float) -> DeadReckoningResult:
    """Lose GNSS at ``gnss_cutoff`` seconds and dead-reckon to the end of the flight.

    The true state at the first sample at or after the cutoff initialises the
    estimator. That is the only ground truth used.
    """
    k0 = traj.index_at(gnss_cutoff)
    initial = NavState(
        position=traj.position_gt[k0].copy(),
        velocity=traj.velocity_gt[k0].copy(),
        attitude=traj.attitude_gt[k0].copy(),
    )
    t = traj.timestamp[k0:]
    position, velocity, attitude = propagate(
        t, traj.accelerometer[k0:], traj.gyroscope[k0:], initial, traj.gravity_world
    )
    return DeadReckoningResult(t, position, velocity, attitude, start_index=k0, t0=float(t[0]))

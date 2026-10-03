"""Common trajectory representation shared by every data source.

Every loader (synthetic, Mid-Air, later others) converts its data into these
canonical conventions, so the estimator and the evaluation never need to know
where the data came from.

Conventions
-----------
- time: seconds, float64, strictly increasing.
- world frame: a local tangent frame named by ``Trajectory.world_frame``
  ("NED" or "ENU"), position in metres, velocity in m/s.
- gravity: ``Trajectory.gravity_world`` is the gravitational acceleration
  vector in the world frame, in m/s^2. NED: (0, 0, +g). ENU: (0, 0, -g).
- attitude: unit quaternion, scalar first (w, x, y, z). It rotates vectors
  from the body frame into the world frame: ``v_world = R(q) @ v_body``.
- accelerometer: specific force in the body frame, in m/s^2:
  ``f_body = R(q)^T (a_world - gravity_world)``. A level drone at rest in NED
  reads (0, 0, -g).
- gyroscope: angular velocity of the body relative to the world frame, in
  rad/s, expressed in the frame named by ``Trajectory.gyroscope_frame``:
  "body" (what a real strapdown gyro measures, the default) or "world"
  (what Mid-Air records, see src/data/midair.py). Earth rotation is ignored.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

STANDARD_GRAVITY = 9.81  # m/s^2, the value used in experiments/e_midair_imu_noise.py
GYRO_FRAMES = ("body", "world")


def gravity_vector(world_frame: str, g: float = STANDARD_GRAVITY) -> np.ndarray:
    """Gravity acceleration vector in the given world frame ("NED" or "ENU")."""
    frame = world_frame.upper()
    if frame == "NED":
        return np.array([0.0, 0.0, g])
    if frame == "ENU":
        return np.array([0.0, 0.0, -g])
    raise ValueError(f"unsupported world frame {world_frame!r}, expected 'NED' or 'ENU'")


def quat_wxyz_to_rotation(q: np.ndarray) -> Rotation:
    """Scalar-first quaternion(s) (w, x, y, z) to a scipy Rotation (body -> world)."""
    q = np.asarray(q, dtype=float)
    return Rotation.from_quat(q[..., [1, 2, 3, 0]])


def rotation_to_quat_wxyz(r: Rotation) -> np.ndarray:
    """scipy Rotation to scalar-first quaternion(s) (w, x, y, z)."""
    return r.as_quat()[..., [3, 0, 1, 2]]


@dataclass
class TimedObservations:
    """Measurements on their own clock, e.g. GNSS fixes at 1 Hz.

    Kept apart from the IMU stream so that new sensors (GNSS, camera frames,
    barometer) can be attached to a trajectory without touching the estimator.
    """

    timestamp: np.ndarray  # (M,) seconds, same clock as Trajectory.timestamp
    values: np.ndarray  # (M, ...) sensor-specific


@dataclass
class Trajectory:
    """One flight: IMU stream plus ground truth, all at the same timestamps.

    Ground truth is for initialisation at the GNSS cutoff and for evaluation
    only. See the module docstring for units and frames.
    """

    timestamp: np.ndarray  # (N,) s
    position_gt: np.ndarray  # (N, 3) m, world frame
    velocity_gt: np.ndarray  # (N, 3) m/s, world frame
    attitude_gt: np.ndarray  # (N, 4) quaternion (w, x, y, z), body -> world
    accelerometer: np.ndarray  # (N, 3) m/s^2, specific force, body frame
    gyroscope: np.ndarray  # (N, 3) rad/s, in gyroscope_frame
    world_frame: str = "NED"
    gyroscope_frame: str = "body"
    gravity_world: np.ndarray = field(default_factory=lambda: gravity_vector("NED"))
    name: str = ""
    gps: TimedObservations | None = None  # never read by the dead-reckoning baseline
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.timestamp = np.asarray(self.timestamp, dtype=float)
        n = self.timestamp.shape[0]
        if self.timestamp.ndim != 1 or n < 2:
            raise ValueError("timestamp must be a 1-D array with at least two samples")
        if np.any(np.diff(self.timestamp) <= 0):
            raise ValueError("timestamps must be strictly increasing")
        for name, width in [
            ("position_gt", 3),
            ("velocity_gt", 3),
            ("attitude_gt", 4),
            ("accelerometer", 3),
            ("gyroscope", 3),
        ]:
            arr = np.asarray(getattr(self, name), dtype=float)
            if arr.shape != (n, width):
                raise ValueError(f"{name} has shape {arr.shape}, expected ({n}, {width})")
            if not np.all(np.isfinite(arr)):
                raise ValueError(f"{name} contains non-finite values")
            setattr(self, name, arr)
        norms = np.linalg.norm(self.attitude_gt, axis=1)
        if np.max(np.abs(norms - 1.0)) > 1e-3:
            raise ValueError("attitude_gt quaternions are not unit length")
        self.gravity_world = np.asarray(self.gravity_world, dtype=float)
        if self.gyroscope_frame not in GYRO_FRAMES:
            raise ValueError(f"gyroscope_frame must be one of {GYRO_FRAMES}, got {self.gyroscope_frame!r}")

    def __len__(self) -> int:
        return self.timestamp.shape[0]

    @property
    def duration(self) -> float:
        """Length of the flight in seconds."""
        return float(self.timestamp[-1] - self.timestamp[0])

    def index_at(self, t: float) -> int:
        """Index of the first sample at or after time ``t`` (seconds)."""
        k = int(np.searchsorted(self.timestamp, t, side="left"))
        if k >= len(self) - 1:
            raise ValueError(
                f"time {t} s leaves no samples to propagate; the flight spans "
                f"{self.timestamp[0]:.2f} to {self.timestamp[-1]:.2f} s"
            )
        return k


def imu_consistency(traj: Trajectory) -> dict[str, float]:
    """Check the IMU against what the ground truth implies.

    A diagnostic for frame and quaternion conventions, used before trusting a
    new data source. Expected accelerometer: ``R^T (dv/dt - g)`` with ``dv/dt``
    from central differences of ``velocity_gt``. Expected gyroscope: the
    rotation between consecutive ground-truth attitudes divided by dt.

    With correct conventions the residuals are at the sensor noise level. A
    wrong gravity sign or quaternion order gives accelerometer residuals of
    order g (about 10 to 20 m/s^2).

    The gyroscope is also compared in the other frame (body vs world); if that
    fits better, ``gyroscope_frame`` is probably wrong.

    Returns the median norm of each residual.
    """
    t = traj.timestamp
    r = quat_wxyz_to_rotation(traj.attitude_gt)
    accel_world = (traj.velocity_gt[2:] - traj.velocity_gt[:-2]) / (t[2:] - t[:-2])[:, None]
    expected_f = r[1:-1].inv().apply(accel_world - traj.gravity_world)
    accel_res = np.linalg.norm(traj.accelerometer[1:-1] - expected_f, axis=1)

    dt = np.diff(t)[:, None]
    rate = {
        "body": (r[:-1].inv() * r[1:]).as_rotvec() / dt,
        "world": (r[1:] * r[:-1].inv()).as_rotvec() / dt,
    }
    other = "world" if traj.gyroscope_frame == "body" else "body"
    gyro_mid = 0.5 * (traj.gyroscope[:-1] + traj.gyroscope[1:])
    return {
        "accel_residual_median": float(np.median(accel_res)),
        "gyro_residual_median": float(np.median(np.linalg.norm(gyro_mid - rate[traj.gyroscope_frame], axis=1))),
        "gyro_residual_median_other_frame": float(np.median(np.linalg.norm(gyro_mid - rate[other], axis=1))),
    }

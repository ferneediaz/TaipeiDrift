"""Deterministic synthetic flights with IMU readings consistent with the motion.

Each scenario defines position, velocity, acceleration, attitude and body
angular velocity analytically, in NED. The IMU is then computed exactly from
that motion:

    accelerometer = R^T (a_world - g_world)      (specific force, body frame)
    gyroscope     = R^T omega_world              (body frame)

With noise disabled, a correct dead-reckoning estimator started from the true
state reproduces the true trajectory up to integration error. This is what the
tests rely on.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from src.data.trajectory import (
    STANDARD_GRAVITY,
    Trajectory,
    gravity_vector,
    rotation_to_quat_wxyz,
)


@dataclass(frozen=True)
class ImuNoise:
    """Additive IMU errors. All zero means ideal measurements."""

    accel_white_std: float = 0.0  # m/s^2 per sample
    gyro_white_std: float = 0.0  # rad/s per sample
    accel_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)  # m/s^2, body frame, constant
    gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)  # rad/s, body frame, constant
    seed: int = 0


@dataclass
class Motion:
    """Analytic motion sampled at given times, NED world frame."""

    position: np.ndarray  # (N, 3)
    velocity: np.ndarray  # (N, 3)
    acceleration: np.ndarray  # (N, 3)
    attitude: Rotation  # N rotations, body -> world
    angular_velocity_body: np.ndarray  # (N, 3)


def _stationary(t: np.ndarray) -> Motion:
    """Hovering at 30 m with a fixed tilted attitude: tests the gravity handling."""
    n = len(t)
    attitude = Rotation.from_euler("ZYX", [30.0, 5.0, 10.0], degrees=True)  # yaw, pitch, roll
    return Motion(
        position=np.tile([10.0, -5.0, -30.0], (n, 1)),
        velocity=np.zeros((n, 3)),
        acceleration=np.zeros((n, 3)),
        attitude=Rotation.concatenate([attitude] * n),
        angular_velocity_body=np.zeros((n, 3)),
    )


def _constant_velocity(t: np.ndarray) -> Motion:
    """Straight climbing line at constant velocity, level, nose along the track."""
    v = np.array([3.0, 4.0, -0.5])  # north, east, down (climbing at 0.5 m/s)
    p0 = np.array([0.0, 0.0, -30.0])
    yaw = np.arctan2(v[1], v[0])
    n = len(t)
    return Motion(
        position=p0 + t[:, None] * v,
        velocity=np.tile(v, (n, 1)),
        acceleration=np.zeros((n, 3)),
        attitude=Rotation.concatenate([Rotation.from_euler("Z", yaw)] * n),
        angular_velocity_body=np.zeros((n, 3)),
    )


def _circle(t: np.ndarray) -> Motion:
    """Banked turn on a 25 m circle while accelerating upwards.

    Exercises centripetal acceleration, a time-varying attitude and body
    rates on two axes (the bank angle splits the yaw rate between body y and z).
    """
    radius, rate = 25.0, 0.25  # m, rad/s
    climb_accel = 0.02  # m/s^2 upwards, so -0.02 along Down
    bank = np.deg2rad(15.0)
    z0 = -30.0
    c, s = np.cos(rate * t), np.sin(rate * t)
    position = np.column_stack([radius * c, radius * s, z0 - 0.5 * climb_accel * t**2])
    velocity = np.column_stack([-radius * rate * s, radius * rate * c, -climb_accel * t])
    acceleration = np.column_stack(
        [-radius * rate**2 * c, -radius * rate**2 * s, np.full_like(t, -climb_accel)]
    )
    yaw = rate * t + np.pi / 2  # nose along the direction of travel
    attitude = Rotation.from_euler("Z", yaw) * Rotation.from_euler("X", bank)
    omega_world = np.tile([0.0, 0.0, rate], (len(t), 1))  # R = Rz(yaw(t)) * const
    return Motion(position, velocity, acceleration, attitude, attitude.inv().apply(omega_world))


def _spinning_hover(t: np.ndarray) -> Motion:
    """Fixed position while rotating at a constant body rate about a tilted axis.

    Gravity sweeps through every body axis, so any error in attitude
    propagation leaks gravity into the position estimate.
    """
    omega_body = np.array([0.3, -0.2, 0.5])  # rad/s
    r0 = Rotation.from_euler("ZYX", [10.0, -5.0, 20.0], degrees=True)
    n = len(t)
    return Motion(
        position=np.tile([0.0, 0.0, -30.0], (n, 1)),
        velocity=np.zeros((n, 3)),
        acceleration=np.zeros((n, 3)),
        attitude=r0 * Rotation.from_rotvec(t[:, None] * omega_body),
        angular_velocity_body=np.tile(omega_body, (n, 1)),
    )


SCENARIOS: dict[str, Callable[[np.ndarray], Motion]] = {
    "stationary": _stationary,
    "constant_velocity": _constant_velocity,
    "circle": _circle,
    "spinning_hover": _spinning_hover,
}


def make_synthetic_trajectory(
    scenario: str = "circle",
    duration: float = 60.0,
    rate_hz: float = 100.0,
    noise: ImuNoise | None = None,
    gravity: float = STANDARD_GRAVITY,
) -> Trajectory:
    """Build a synthetic flight in NED with IMU readings derived from the motion.

    Args:
        scenario: one of ``SCENARIOS``.
        duration: flight length in seconds.
        rate_hz: sample rate of IMU and ground truth.
        noise: IMU errors to add; ``None`` gives ideal measurements.
        gravity: magnitude of gravity in m/s^2.
    """
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario {scenario!r}, choose from {sorted(SCENARIOS)}")
    noise = noise or ImuNoise()
    n = int(round(duration * rate_hz)) + 1
    t = np.arange(n) / rate_hz
    m = SCENARIOS[scenario](t)
    g_world = gravity_vector("NED", gravity)

    accel = m.attitude.inv().apply(m.acceleration - g_world)
    gyro = m.angular_velocity_body.copy()
    rng = np.random.default_rng(noise.seed)
    accel += np.asarray(noise.accel_bias) + noise.accel_white_std * rng.standard_normal((n, 3))
    gyro += np.asarray(noise.gyro_bias) + noise.gyro_white_std * rng.standard_normal((n, 3))

    return Trajectory(
        timestamp=t,
        position_gt=m.position,
        velocity_gt=m.velocity,
        attitude_gt=rotation_to_quat_wxyz(m.attitude),
        accelerometer=accel,
        gyroscope=gyro,
        world_frame="NED",
        gravity_world=g_world,
        name=f"synthetic_{scenario}",
        metadata={"source": "synthetic", "scenario": scenario, "noise": noise.__dict__},
    )

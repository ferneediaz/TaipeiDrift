"""Sensors that Mid-Air lacks, SIMULATED FROM GROUND TRUTH, and controlled IMU bias injection.

Ground truth enters only here, to generate a noisy sensor stream. The estimator
receives the stream, never the ground truth it was generated from.

- ``SimulatedBarometer``: altitude above the take-off point with white noise and
  a slowly drifting offset (random walk). It measures ALTITUDE, not height
  above the ground below: Mid-Air flies about 15 m above hilly terrain
  (docs/findings.md on main, section 2.5), so the two differ by the terrain.
- ``inject_imu_bias``: adds known constant biases to the IMU so bias recovery
  can be scored against them.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np

from src.data.trajectory import Trajectory


@dataclass(frozen=True)
class BarometerConfig:
    white_std_m: float = 0.3  # per sample
    bias_walk_m_per_sqrt_s: float = 0.05  # random-walk drift of the offset
    initial_bias_m: float = 0.0
    seed: int = 0


@dataclass
class BarometerStream:
    """What the estimator sees: noisy altitude (up positive) at the IMU timestamps."""

    timestamp: np.ndarray
    altitude_m: np.ndarray


def simulate_barometer(traj: Trajectory, cfg: BarometerConfig | None = None) -> BarometerStream:
    """Generate a barometer stream from the true altitude. Ground truth -> noisy sensor."""
    cfg = cfg or BarometerConfig()
    rng = np.random.default_rng(cfg.seed)
    up = -traj.position_gt[:, 2] if traj.world_frame == "NED" else traj.position_gt[:, 2]
    dt = np.diff(traj.timestamp, prepend=traj.timestamp[0])
    walk = np.cumsum(cfg.bias_walk_m_per_sqrt_s * np.sqrt(dt) * rng.standard_normal(len(traj)))
    noisy = up + cfg.initial_bias_m + walk + cfg.white_std_m * rng.standard_normal(len(traj))
    return BarometerStream(traj.timestamp.copy(), noisy)


@dataclass(frozen=True)
class InjectedBias:
    """Known constant biases added to the IMU. Gyro bias is in the gyroscope's own axes."""

    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)  # rad/s
    accel: tuple[float, float, float] = (0.0, 0.0, 0.0)  # m/s^2, body frame


def inject_imu_bias(traj: Trajectory, bias: InjectedBias) -> Trajectory:
    """Copy of ``traj`` with constant biases added to the IMU. Ground truth is unchanged."""
    out = copy.deepcopy(traj)
    out.gyroscope = out.gyroscope + np.asarray(bias.gyro)
    out.accelerometer = out.accelerometer + np.asarray(bias.accel)
    out.metadata = {**out.metadata, "injected_bias": {"gyro": list(bias.gyro), "accel": list(bias.accel)}}
    return out

"""Sensors that Mid-Air lacks, SIMULATED FROM GROUND TRUTH, and controlled IMU bias injection.

Ground truth enters only here, to generate a noisy sensor stream. The estimator
receives the stream, never the ground truth it was generated from.

- ``simulate_barometer``: barometric altitude with the MEASURED real-sensor error model
  (origin/research/offline-nav-evidence, docs/research/context-alessandro-en.md section 3;
  fitted on the Zurich Urban MAV Pixhawk barometer, experiments/s_zurich_vertical.py,
  confirmed by INSANE between 10 and 120 s, experiments/s_insane_baro.py):

      z_baro(t) = z(t0) + (1 + s) (z_true(t) - z_true(t0)) + b_rw(t) + d (t - t0) + n_white(t)

      n_white ~ N(0, 0.30 m)                       white noise
      b_rw(k+1) = b_rw(k) + 0.112 m/sqrt(s) sqrt(dt) eps   random walk
      d ~ N(0, 0.0024 m/s), one draw per flight    linear drift (as in the branch's generator)
      s = +/- U(0.03, 0.07), one draw per flight   scale error, "3-7 %, both directions"

  It measures ALTITUDE, not height above the ground below. The two agree only over flat
  terrain: Mid-Air flies about 15 m above hilly ground (docs/findings.md, section 2.5), so
  barometric altitude must not be used as camera-to-ground range for optical-flow scale.
  Use it after GNSS loss as a RELATIVE altitude: z_baro(t) - z_baro(t0).
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
    """Measured real-sensor barometer model (see module docstring). Simulation only."""

    white_std_m: float = 0.30
    bias_walk_m_per_sqrt_s: float = 0.112
    drift_sigma_m_per_s: float = 0.0024  # per-flight linear drift slope ~ N(0, sigma)
    scale_error_min: float = 0.03  # per-flight scale error magnitude ~ U(min, max), random sign
    scale_error_max: float = 0.07
    initial_bias_m: float = 0.0
    noise_factor: float = 1.0  # x0.5 / x1 / x2 noise-tolerance runs (scales white, walk and drift)
    seed: int = 0


@dataclass
class BarometerStream:
    """What the estimator sees: noisy altitude (up positive) at the given timestamps.

    ``scale_error`` and ``drift_m_per_s`` are the per-flight draws, kept for evaluation only.
    """

    timestamp: np.ndarray
    altitude_m: np.ndarray
    scale_error: float = 0.0
    drift_m_per_s: float = 0.0


def generate_barometer(t: np.ndarray, alt_true_up: np.ndarray, cfg: BarometerConfig | None = None,
                       ref_index: int = 0) -> BarometerStream:
    """Measured error model applied to a true altitude series (up positive). Deterministic per seed."""
    cfg = cfg or BarometerConfig()
    rng = np.random.default_rng(cfg.seed)
    t = np.asarray(t, float)
    alt = np.asarray(alt_true_up, float)
    s = (1.0 if rng.random() < 0.5 else -1.0) * rng.uniform(cfg.scale_error_min, cfg.scale_error_max)
    d = cfg.noise_factor * cfg.drift_sigma_m_per_s * rng.standard_normal()
    dt = np.diff(t, prepend=t[0])
    walk = np.cumsum(cfg.noise_factor * cfg.bias_walk_m_per_sqrt_s * np.sqrt(np.maximum(dt, 0.0)) * rng.standard_normal(len(t)))
    white = cfg.noise_factor * cfg.white_std_m * rng.standard_normal(len(t))
    z0 = alt[ref_index]
    z = z0 + (1.0 + s) * (alt - z0) + cfg.initial_bias_m + walk + d * (t - t[0]) + white
    return BarometerStream(t.copy(), z, float(s), float(d))


def relative_altitude(stream: BarometerStream, k0: int) -> np.ndarray:
    """Barometric altitude change since the GNSS cut, z_baro(t) - z_baro(t0): what the estimator uses."""
    return stream.altitude_m[k0:] - stream.altitude_m[k0]


def simulate_barometer(traj: Trajectory, cfg: BarometerConfig | None = None) -> BarometerStream:
    """Generate a barometer stream from the true altitude. Ground truth -> noisy sensor."""
    up = -traj.position_gt[:, 2] if traj.world_frame == "NED" else traj.position_gt[:, 2]
    return generate_barometer(traj.timestamp, up, cfg)


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

"""Linearised INS error model used by the filters (16 states = 15-state INS + baro bias).

State order: xi = [dE, dN, dU] (position error, INS - truth)  |  x_l = [dv(3), phi(3), b_a(3), b_g(3), b_baro]
Continuous model (psi-angle form, local-level ENU):
    d(xi)/dt  = dv
    d(dv)/dt  = [f^n x] phi + C b_a - [(2 w_ie + w_en) x] dv + (0, 0, 2g/R dU)
    d(phi)/dt = -[w_in x] phi + M dv - C b_g          (M: transport-rate sensitivity -> Schuler loop)
    d(b)/dt   = -b / tau + w                           (first-order Gauss-Markov biases)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from trn.ins.strapdown import ImuErrorParams

N_XI, N_L = 3, 13


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


@dataclass
class ErrorModel:
    """Discrete-time error model builder for one IMU grade (+ optional baro bias)."""

    imu: ImuErrorParams
    baro_sigma: float
    baro_tau: float
    baro_noise: float
    R: float
    omega: float
    lat_ref: float
    n_ref: float

    def qc_diag(self) -> np.ndarray:
        p = self.imu
        return np.concatenate([np.zeros(3), np.full(3, p.vrw ** 2), np.full(3, p.arw ** 2),
                               np.full(3, 2 * p.accel_bias ** 2 / p.accel_tau),
                               np.full(3, 2 * p.gyro_bias ** 2 / p.gyro_tau),
                               [2 * self.baro_sigma ** 2 / self.baro_tau]])

    def P0_linear(self, vel_sigma: float) -> np.ndarray:
        p = self.imu
        return np.diag(np.concatenate([np.full(3, vel_sigma ** 2), p.init_att ** 2, np.full(3, p.accel_bias ** 2),
                                       np.full(3, p.gyro_bias ** 2), [self.baro_sigma ** 2]]))

    def discrete(self, fn: np.ndarray, v: np.ndarray, C: np.ndarray, pos: np.ndarray, dt: float) -> tuple:
        """Return (A_xixi, A_n, A_lxi, A_l, Q_n, Q_l) for one step of length dt."""
        lat = self.lat_ref + (pos[1] - self.n_ref) / self.R
        Rh = self.R + pos[2]
        wie = np.array([0.0, self.omega * math.cos(lat), self.omega * math.sin(lat)])
        wen = np.array([-v[1] / Rh, v[0] / Rh, v[0] * math.tan(lat) / Rh])
        g = 9.780327 * (1 + 0.0053024 * math.sin(lat) ** 2)
        F = np.zeros((16, 16))
        F[0:3, 3:6] = np.eye(3)
        F[3:6, 3:6] = -_skew(2 * wie + wen)
        F[3:6, 6:9] = _skew(fn)
        F[3:6, 9:12] = C
        F[5, 2] = 2 * g / Rh
        F[6:9, 6:9] = -_skew(wie + wen)
        F[6:9, 3:6] = np.array([[0, -1 / Rh, 0], [1 / Rh, 0, 0], [math.tan(lat) / Rh, 0, 0]])
        F[6:9, 12:15] = -C
        F[9:12, 9:12] = -np.eye(3) / self.imu.accel_tau
        F[12:15, 12:15] = -np.eye(3) / self.imu.gyro_tau
        F[15, 15] = -1.0 / self.baro_tau
        Fd = F * dt
        Phi = np.eye(16) + Fd + 0.5 * Fd @ Fd
        Qc = np.diag(self.qc_diag())
        Qd = 0.5 * (Phi @ Qc @ Phi.T + Qc) * dt
        return Phi[0:3, 0:3], Phi[0:3, 3:], Phi[3:, 0:3], Phi[3:, 3:], Qd[0:3, 0:3], Qd[3:, 3:]


def build_error_model(cfg: dict, lat_ref: float, n_ref: float) -> ErrorModel:
    """Error model for the configured IMU grade and barometer."""
    ic = cfg["imu"]
    b = ic["baro"]
    return ErrorModel(imu=ImuErrorParams.from_config(ic), baro_sigma=float(b["bias_sigma_m"]),
                      baro_tau=float(b["bias_tau_s"]), baro_noise=float(b["noise_m"]),
                      R=float(ic["earth_radius_m"]), omega=float(ic["earth_rate_rps"]), lat_ref=lat_ref, n_ref=n_ref)

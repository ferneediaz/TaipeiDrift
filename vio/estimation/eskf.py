"""Error-state Kalman filter (ESKF) for inertial navigation with visual updates.

Nominal state:  p, v (world, m and m/s), R (body -> world), b_a (body, m/s^2), b_g (gyro axes, rad/s).
Error state:    dx = [dp, dv, dtheta, db_a, db_g]  (15), optionally followed by a clone
                dtheta_c (3) of the attitude error at the current camera keyframe.

Attitude error is defined on the WORLD side (left):  R_true = Exp(dtheta) R_hat.

Prediction (same integration as the IMU-only baseline, with biases removed):
    w      = 0.5 (w_m,k + w_m,k+1) - b_g
    R_k+1  = Exp(w dt) R_k            gyro in world axes (Mid-Air)
    R_k+1  = R_k Exp(w dt)            gyro in body axes (a real strapdown IMU)
    a_k    = R_k (a_m,k - b_a) + g
    v_k+1  = v_k + 0.5 (a_k + a_k+1) dt
    p_k+1  = p_k + 0.5 (v_k + v_k+1) dt

Error dynamics (continuous):
    dp'     = dv
    dv'     = -[R f]x dtheta - R db_a - R n_a
    dtheta' = -G db_g - G n_g          G = I (world gyro) or R (body gyro)
    db_a'   = n_ba,   db_g' = n_bg
discretised as Phi = I + A dt + (A dt)^2 / 2,  Q_d = G_n Qc G_n^T dt.

Updates use the Joseph form and inject dx into the nominal state:
p += dp, v += dv, R <- Exp(dtheta) R, b += db, R_clone <- Exp(dtheta_c) R_clone.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import chi2

from src.data.trajectory import quat_wxyz_to_rotation

P_, V_, TH, BA, BG = slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15)
N_ERR = 15


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


@dataclass
class ImuNoiseModel:
    """Continuous-time noise densities and initial uncertainties.

    Defaults follow the IMU errors measured on the 30 sunny Mid-Air flights
    (docs/findings.md on main, section 2.3): gyro white noise 0.021 rad/s and
    accelerometer 0.04 m/s^2 per 100 Hz sample, gyro offsets of about 0.001
    (up to 0.006) rad/s changing by about 0.003 rad/s within a flight,
    accelerometer offsets of about 0.025 (up to 0.1) m/s^2 changing by about 0.04.
    """

    gyro_noise: float = 0.021 * 0.1  # rad/s/sqrt(Hz)  = per-sample std * sqrt(dt)
    accel_noise: float = 0.04 * 0.1  # m/s^2/sqrt(Hz)
    gyro_bias_walk: float = 0.003 / np.sqrt(80.0)  # rad/s/sqrt(s)
    accel_bias_walk: float = 0.04 / np.sqrt(80.0)  # m/s^2/sqrt(s)
    init_pos_std: float = 0.5  # m, GNSS-aided state at the cutoff
    init_vel_std: float = 0.05  # m/s
    init_att_std_deg: float = 0.2
    init_gyro_bias_std: float = 0.003  # rad/s
    init_accel_bias_std: float = 0.06  # m/s^2


@dataclass
class UpdateResult:
    accepted: bool
    nis: float
    dof: int
    reason: str = ""


class ESKF:
    """15-state ESKF with an optional 3-state attitude clone for relative-rotation updates."""

    def __init__(self, p0, v0, q0_wxyz, gravity_world, gyroscope_frame: str = "world",
                 noise: ImuNoiseModel | None = None, ba0=None, bg0=None) -> None:
        if gyroscope_frame not in ("body", "world"):
            raise ValueError(f"gyroscope_frame must be 'body' or 'world', got {gyroscope_frame!r}")
        self.noise = noise or ImuNoiseModel()
        self.g = np.asarray(gravity_world, dtype=float)
        self.world_gyro = gyroscope_frame == "world"
        self.p = np.array(p0, dtype=float)
        self.v = np.array(v0, dtype=float)
        self.R = quat_wxyz_to_rotation(np.asarray(q0_wxyz, dtype=float))
        self.ba = np.zeros(3) if ba0 is None else np.array(ba0, dtype=float)
        self.bg = np.zeros(3) if bg0 is None else np.array(bg0, dtype=float)
        n = self.noise
        self.P = np.diag(np.r_[np.full(3, n.init_pos_std**2), np.full(3, n.init_vel_std**2),
                               np.full(3, np.deg2rad(n.init_att_std_deg) ** 2),
                               np.full(3, n.init_accel_bias_std**2), np.full(3, n.init_gyro_bias_std**2)])
        self.R_clone: Rotation | None = None

    # ------------------------------------------------------------------ state
    @property
    def n(self) -> int:
        return self.P.shape[0]

    @property
    def has_clone(self) -> bool:
        return self.R_clone is not None

    def clone_attitude(self) -> None:
        """Store the current attitude (and its error correlations) as the keyframe clone."""
        P15 = self.P[:N_ERR, :N_ERR]
        J = np.vstack([np.eye(N_ERR), np.hstack([np.zeros((3, 6)), np.eye(3), np.zeros((3, 6))])])
        self.P = J @ P15 @ J.T
        self.R_clone = self.R

    def drop_clone(self) -> None:
        self.P = self.P[:N_ERR, :N_ERR].copy()
        self.R_clone = None

    # ------------------------------------------------------------- prediction
    def predict(self, acc_k, acc_k1, gyro_k, gyro_k1, dt: float) -> None:
        """Propagate nominal state and covariance over one IMU interval."""
        if dt <= 0:
            raise ValueError("dt must be positive")
        f_k = np.asarray(acc_k) - self.ba
        f_k1 = np.asarray(acc_k1) - self.ba
        w = 0.5 * (np.asarray(gyro_k) + np.asarray(gyro_k1)) - self.bg
        R_k = self.R
        inc = Rotation.from_rotvec(w * dt)
        R_k1 = inc * R_k if self.world_gyro else R_k * inc
        a_k = R_k.apply(f_k) + self.g
        a_k1 = R_k1.apply(f_k1) + self.g
        v_k1 = self.v + 0.5 * (a_k + a_k1) * dt
        self.p = self.p + 0.5 * (self.v + v_k1) * dt
        self.v = v_k1
        self.R = R_k1

        Rm = R_k.as_matrix()
        G = np.eye(3) if self.world_gyro else Rm
        A = np.zeros((N_ERR, N_ERR))
        A[P_, V_] = np.eye(3)
        A[V_, TH] = -skew(Rm @ f_k)
        A[V_, BA] = -Rm
        A[TH, BG] = -G
        Ad = A * dt
        Phi = np.eye(N_ERR) + Ad + 0.5 * Ad @ Ad
        nz = self.noise
        Qd = np.zeros((N_ERR, N_ERR))
        Qd[V_, V_] = np.eye(3) * nz.accel_noise**2 * dt
        Qd[TH, TH] = np.eye(3) * nz.gyro_noise**2 * dt
        Qd[BA, BA] = np.eye(3) * nz.accel_bias_walk**2 * dt
        Qd[BG, BG] = np.eye(3) * nz.gyro_bias_walk**2 * dt

        P = self.P
        if self.n == N_ERR:
            self.P = Phi @ P @ Phi.T + Qd
        else:  # the clone does not move: identity block, cross terms carried by Phi
            Pxx, Pxc, Pcc = P[:N_ERR, :N_ERR], P[:N_ERR, N_ERR:], P[N_ERR:, N_ERR:]
            newxx = Phi @ Pxx @ Phi.T + Qd
            newxc = Phi @ Pxc
            self.P = np.block([[newxx, newxc], [newxc.T, Pcc]])
        self.P = 0.5 * (self.P + self.P.T)

    # ---------------------------------------------------------------- updates
    def update(self, r: np.ndarray, H: np.ndarray, Rm: np.ndarray, gate_prob: float | None = 0.99) -> UpdateResult:
        """Generic EKF update with Mahalanobis gating. Rejected updates leave the state untouched."""
        r = np.atleast_1d(np.asarray(r, dtype=float))
        S = H @ self.P @ H.T + Rm
        try:
            Sinv_r = np.linalg.solve(S, r)
        except np.linalg.LinAlgError:
            return UpdateResult(False, np.nan, len(r), "singular innovation covariance")
        nis = float(r @ Sinv_r)
        if not np.isfinite(nis):
            return UpdateResult(False, nis, len(r), "non-finite innovation")
        if gate_prob is not None and nis > chi2.ppf(gate_prob, len(r)):
            return UpdateResult(False, nis, len(r), "Mahalanobis gate")
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ r
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self._inject(dx)
        return UpdateResult(True, nis, len(r))

    def _inject(self, dx: np.ndarray) -> None:
        self.p = self.p + dx[P_]
        self.v = self.v + dx[V_]
        self.R = Rotation.from_rotvec(dx[TH]) * self.R
        self.ba = self.ba + dx[BA]
        self.bg = self.bg + dx[BG]
        if self.has_clone:
            self.R_clone = Rotation.from_rotvec(dx[N_ERR:N_ERR + 3]) * self.R_clone

    def update_relative_rotation(self, C_body: np.ndarray, sigma_rad: float, gate_prob: float | None = 0.99) -> UpdateResult:
        """Relative rotation C = R_clone^T R_now measured by the camera (body frame).

        Residual r = Log(C_hat^T C) = R_now^T (dtheta_now - dtheta_clone) + noise.
        """
        if not self.has_clone:
            return UpdateResult(False, np.nan, 3, "no keyframe clone")
        C_hat = self.R_clone.inv() * self.R
        r = (C_hat.inv() * Rotation.from_matrix(C_body)).as_rotvec()
        RnT = self.R.as_matrix().T
        H = np.zeros((3, self.n))
        H[:, TH] = RnT
        H[:, N_ERR:N_ERR + 3] = -RnT
        return self.update(r, H, np.eye(3) * sigma_rad**2, gate_prob)

    def update_altitude(self, z_up: float, up_axis: np.ndarray, sigma_m: float, gate_prob: float | None = 0.99) -> UpdateResult:
        """Altitude (barometer): z = up . p + noise, with ``up_axis`` the world 'up' unit vector."""
        up = np.asarray(up_axis, dtype=float)
        H = np.zeros((1, self.n))
        H[0, P_] = up
        return self.update(np.array([z_up - up @ self.p]), H, np.array([[sigma_m**2]]), gate_prob)

    def update_camera_velocity_xy(self, z: np.ndarray, R_bc: np.ndarray, Rm: np.ndarray,
                                  gate_prob: float | None = 0.99) -> UpdateResult:
        """Lateral velocity of the camera in its own x, y axes: z = S R_bc^T R^T v + noise."""
        Rw = self.R.as_matrix()
        M = (R_bc.T @ Rw.T)[:2]  # S R_bc^T R^T
        r = np.asarray(z) - M @ self.v
        H = np.zeros((2, self.n))
        H[:, V_] = M
        H[:, TH] = M @ skew(self.v)  # d(R^T v)/d(dtheta) = R^T [v]x for R = Exp(dtheta) R_hat
        return self.update(r, H, Rm, gate_prob)

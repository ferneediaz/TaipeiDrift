"""INS mechanization and error model."""
import numpy as np
import pytest

from trn.common.config import load_base_config
from trn.ins.error_model import build_error_model
from trn.ins.nav_math import exp_so3, log_so3
from trn.ins.strapdown import ideal_imu, mechanize


def _synthetic_traj(n=6000, dt=0.01):
    """Level coordinated circle-ish trajectory with climb, built directly (no map needed)."""
    from trn.ins.nav_math import euler_to_C
    t = np.arange(n) * dt
    yaw = 0.02 * t
    V = 35.0
    vel = np.column_stack([V * np.cos(yaw), V * np.sin(yaw), 2.0 * np.sin(0.05 * t)])
    pos = np.zeros((n, 3)); pos[0] = [250000.0, 2600000.0, 2000.0]
    pos[1:] = pos[0] + np.cumsum(vel[:-1] * dt, axis=0)
    pitch = np.arctan2(vel[:, 2], V)
    roll = np.arctan(V * 0.02 / 9.8) * np.ones(n)
    C = euler_to_C(-roll, pitch, yaw)
    return pos, vel, C, dt


def test_so3_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(20):
        w = rng.normal(size=3) * 0.5
        np.testing.assert_allclose(log_so3(exp_so3(w)), w, atol=1e-10)


def test_perfect_imu_reproduces_truth():
    pos, vel, C, dt = _synthetic_traj()
    lat, nref, R, om = np.radians(23.5), 2600000.0, 6371000.0, 7.292115e-5
    f, w = ideal_imu(pos, vel, C, dt, lat, nref, R, om)
    P, V, CC, FN = mechanize(pos[0], vel[0], C[0], f, w, dt, lat, nref, R, om, 10)
    np.testing.assert_allclose(P, pos[::10][:P.shape[0]], atol=1e-6)
    np.testing.assert_allclose(V, vel[::10][:V.shape[0]], atol=1e-8)


def test_accel_bias_matches_error_model():
    """A constant accel bias: nonlinear INS error vs. linear 16-state propagation over 60 s."""
    pos, vel, C, dt = _synthetic_traj()
    lat, nref, R, om = np.radians(23.5), 2600000.0, 6371000.0, 7.292115e-5
    f, w = ideal_imu(pos, vel, C, dt, lat, nref, R, om)
    ba = np.array([3e-3, -2e-3, 1e-3])
    P, V, CC, FN = mechanize(pos[0], vel[0], C[0], f + ba, w, dt, lat, nref, R, om, 10)
    cfg = load_base_config()
    em = build_error_model(cfg, lat, nref)
    em.imu.accel_tau = em.imu.gyro_tau = 1e12
    x = np.zeros(16); x[9:12] = ba
    for k in range(P.shape[0] - 1):
        Axx, An, Alx, Al, _, _ = em.discrete(FN[k], V[k], CC[k], P[k], 0.1)
        x = np.concatenate([Axx @ x[:3] + An @ x[3:], Alx @ x[:3] + Al @ x[3:]])
    actual = P[-1] - pos[::10][P.shape[0] - 1]
    np.testing.assert_allclose(x[:3], actual, rtol=0.05, atol=0.5)

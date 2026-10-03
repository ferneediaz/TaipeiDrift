"""Filters on a toy 1-D terrain (sinusoid along x, flat across) with a known answer."""
import copy

import numpy as np
import pytest

from trn.common.config import load_base_config
from trn.filters.base import Measurement
from trn.filters.mpf import MarginalizedPF
from trn.filters.tercom import Tercom
from trn.ins.error_model import build_error_model
from trn.terrain.grid import Grid
from trn.terrain.onboard_map import OnboardMap
from trn.terrain.metrics import slope_grid


def toy_world():
    dx = 30.0
    n = 800
    x = np.arange(n) * dx
    y = (n - 1 - np.arange(n)) * dx
    X, Y = np.meshgrid(x, y)
    # non-separable: a north offset changes the profile SHAPE (a separable f(x)+g(y) would make it a pure
    # height offset, indistinguishable from an altitude error without a barometer)
    Z = 300 + 120 * np.sin(2 * np.pi * X / 2300.0) + 80 * np.sin(2 * np.pi * (X + 0.8 * Y) / 900.0 + 1.0) \
        + 60 * np.sin(2 * np.pi * Y / 1700.0) * np.cos(2 * np.pi * X / 3100.0)
    g = Grid(Z, 0.0, (n - 1) * dx, dx)
    return OnboardMap(g, Grid(slope_grid(Z, dx), 0.0, (n - 1) * dx, dx), "toy")


def run(filter_name, offset=np.array([350.0, -250.0])):
    cfg = load_base_config()
    cfg["laser"]["beam_set"] = "nadir"
    cfg["imu"]["baro"]["enabled"] = False
    ob = toy_world()
    em = build_error_model(cfg, np.radians(23.5), 0.0)
    em.imu.accel_bias = em.imu.gyro_bias = 1e-9
    em.imu.vrw = em.imu.arw = 1e-9
    em.imu.init_att = np.full(3, 1e-9)
    cfg["imu"]["init_errors"]["vel_mps"] = 1e-3
    rng = np.random.default_rng(0)
    dt, V = 0.1, 35.0
    f = (Tercom(ob, cfg) if filter_name == "tercom" else
         MarginalizedPF(ob, cfg, filter_name, em, np.random.default_rng(1), baro_enabled=False))
    errs = []
    for k in range(3000):
        tpos = np.array([2000.0 + V * k * dt, 12000.0, 1500.0])
        ins = tpos + np.array([offset[0], offset[1], 0.0])
        rho = tpos[2] - ob.grid.interp(tpos[:1], tpos[1:2])[0]
        r = np.array([[rho + 0.3 * rng.standard_normal(), np.nan, np.nan]])
        m = Measurement(k * dt, ins, np.array([V, 0, 0.0]), np.eye(3), np.array([0, 0, 9.79]), r, np.nan)
        e = f.initialize(m) if k == 0 else f.step(m)
        errs.append(e.pos[:2] - tpos[:2])
    return np.array(errs)


@pytest.mark.parametrize("name", ["baseline", "proposed"])
def test_mpf_finds_known_offset(name):
    e = run(name)
    assert np.hypot(*e[-1]) < 30.0


def test_tercom_finds_known_offset():
    e = run("tercom")
    assert np.hypot(*e[-1]) < 60.0

"""Interpolation and ray-casting against analytic surfaces."""
import math

import numpy as np
import pytest

from trn.terrain.grid import Grid


def plane_grid(a=0.0, b=0.0, c=100.0, n=401, dx=20.0) -> Grid:
    """z = a*x + b*y + c on a grid with origin (0, (n-1)*dx) (row 0 north)."""
    x = np.arange(n) * dx
    y = (n - 1 - np.arange(n)) * dx
    X, Y = np.meshgrid(x, y)
    return Grid((a * X + b * Y + c).astype(np.float64), 0.0, (n - 1) * dx, dx)


def test_bilinear_exact_on_plane():
    g = plane_grid(0.3, -0.2, 50.0)
    rng = np.random.default_rng(0)
    x, y = rng.uniform(0, 7900, 500), rng.uniform(0, 7900, 500)
    np.testing.assert_allclose(g.interp(x, y), 0.3 * x - 0.2 * y + 50.0, atol=1e-6)


def test_bilinear_nodes_and_outside():
    z = np.arange(12, dtype=float).reshape(3, 4)
    g = Grid(z, 100.0, 200.0, 10.0)
    assert g.interp(np.array([100.0]), np.array([200.0]))[0] == 0.0       # upper-left post
    assert g.interp(np.array([130.0]), np.array([180.0]))[0] == 11.0      # lower-right post
    assert g.interp(np.array([115.0]), np.array([195.0]))[0] == pytest.approx(3.5)
    assert np.isnan(g.interp(np.array([99.0]), np.array([200.0]))[0])


def test_nan_propagates():
    z = np.ones((3, 3)); z[1, 1] = np.nan
    g = Grid(z, 0.0, 20.0, 10.0)
    assert np.isnan(g.interp(np.array([5.0]), np.array([15.0]))[0])


def test_raycast_flat_nadir_and_slant():
    g = plane_grid(c=100.0)
    o = np.array([[4000.0, 4000.0, 3100.0]])
    assert g.raycast(o, np.array([[0, 0, -1.0]]), 5000)[0] == pytest.approx(3000.0, abs=0.05)
    for ang in (10, 25, 40):
        d = np.array([[math.sin(math.radians(ang)), 0.0, -math.cos(math.radians(ang))]])
        assert g.raycast(o, d, 5000)[0] == pytest.approx(3000.0 / math.cos(math.radians(ang)), abs=0.05)


@pytest.mark.parametrize("a,b", [(0.2, 0.0), (0.0, -0.5), (0.4, 0.3), (-1.0, 0.7)])
def test_raycast_sloped_plane_analytic(a, b):
    g = plane_grid(a, b, 500.0)
    rng = np.random.default_rng(1)
    for _ in range(20):
        p = np.array([rng.uniform(2500, 5500), rng.uniform(2500, 5500), 0.0])
        p[2] = a * p[0] + b * p[1] + 500.0 + rng.uniform(300, 2000)
        th, az = math.radians(rng.uniform(0, 30)), rng.uniform(0, 2 * math.pi)
        u = np.array([math.sin(th) * math.cos(az), math.sin(th) * math.sin(az), -math.cos(th)])
        # analytic: p_z + t u_z = a (p_x + t u_x) + b (p_y + t u_y) + c
        t = (a * p[0] + b * p[1] + 500.0 - p[2]) / (u[2] - a * u[0] - b * u[1])
        r = g.raycast(p[None], u[None], 6000)[0]
        assert r == pytest.approx(t, abs=0.05)


def test_raycast_no_hit_beyond_max_range():
    g = plane_grid(c=0.0)
    assert np.isnan(g.raycast(np.array([[4000.0, 4000.0, 6000.0]]), np.array([[0, 0, -1.0]]), 5000)[0])


def test_raycast_does_not_tunnel_through_ridge():
    g = plane_grid(c=0.0)
    z = np.array(g.z); z[:, 210:212] = 900.0          # thin wall 900 m high at x = 4200..4220
    g = Grid(z, g.x0, g.y0, g.dx)
    d = np.array([[math.sin(math.radians(30)), 0.0, -math.cos(math.radians(30))]])
    r = g.raycast(np.array([[3600.0, 4000.0, 1600.0]]), d, 5000)[0]   # flat ground would be hit at x=4524
    hit_x = 3600 + r * d[0, 0]
    assert 4180 < hit_x < 4201                                          # on the wall's bilinear ramp

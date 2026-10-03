"""Terrain information metrics along a track: roughness and gradient."""
from __future__ import annotations

import numpy as np
from scipy.ndimage import uniform_filter1d

from trn.terrain.grid import Grid


def track_profile(grid: Grid, e: np.ndarray, n: np.ndarray) -> np.ndarray:
    """Terrain elevation under a track."""
    return grid.interp(e, n)


def roughness_along_track(h: np.ndarray, ds: float, window_m: float = 1000.0) -> np.ndarray:
    """Local terrain standard deviation [m] in a moving window along the track (ds = sample spacing)."""
    w = max(3, int(round(window_m / ds)))
    h = np.nan_to_num(h, nan=np.nanmean(h))
    m = uniform_filter1d(h, w, mode="nearest")
    m2 = uniform_filter1d(h * h, w, mode="nearest")
    return np.sqrt(np.maximum(m2 - m * m, 0.0))


def gradient_along_track(grid: Grid, e: np.ndarray, n: np.ndarray, step: float | None = None) -> np.ndarray:
    """Terrain gradient magnitude |grad h| (dimensionless) at track points, central differences."""
    s = step or grid.dx
    gx = (grid.interp(e + s, n) - grid.interp(e - s, n)) / (2 * s)
    gy = (grid.interp(e, n + s) - grid.interp(e, n - s)) / (2 * s)
    return np.hypot(gx, gy)


def slope_grid(z: np.ndarray, dx: float) -> np.ndarray:
    """|grad z| on the grid (NaN preserved)."""
    gy, gx = np.gradient(np.asarray(z, dtype=np.float64), dx)
    return np.hypot(gx, gy).astype(np.float32)

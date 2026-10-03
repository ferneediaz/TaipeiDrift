"""Regular elevation grid (point convention) with numba bilinear interpolation and ray casting.

Convention: ``z[i, j]`` is the elevation at the post (x0 + j*dx, y0 - i*dx), i.e. row 0 is the
northernmost row and (x0, y0) is the CENTRE of the upper-left pixel (GeoTIFF AREA_OR_POINT=Point,
the same convention as the .tfw world files). NaN marks unknown terrain.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numba import njit, prange


@dataclass
class Grid:
    """Elevation grid. ``z`` may be a read-only numpy memmap."""

    z: np.ndarray
    x0: float
    y0: float
    dx: float
    max_slope: float = float("nan")

    @property
    def shape(self) -> tuple[int, int]:
        return self.z.shape

    @property
    def extent(self) -> tuple[float, float, float, float]:
        """(xmin, xmax, ymin, ymax) of pixel edges, for matplotlib imshow."""
        h, w = self.z.shape
        return (self.x0 - self.dx / 2, self.x0 + (w - 0.5) * self.dx,
                self.y0 - (h - 0.5) * self.dx, self.y0 + self.dx / 2)

    def interp(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Vectorised bilinear interpolation (NaN outside or on NaN cells)."""
        x = np.ascontiguousarray(np.atleast_1d(x), dtype=np.float64)
        y = np.ascontiguousarray(np.atleast_1d(y), dtype=np.float64)
        out = np.empty(x.shape, np.float64)
        interp_many(self.z, self.x0, self.y0, self.dx, x.ravel(), y.ravel(), out.ravel())
        return out

    def raycast(self, origin: np.ndarray, direction: np.ndarray, max_range: float,
                min_step: float = 0.5, tol: float = 0.02) -> np.ndarray:
        """Ray-terrain intersection ranges for (n,3) origins and (n,3) unit directions."""
        o = np.ascontiguousarray(np.atleast_2d(origin), dtype=np.float64)
        d = np.ascontiguousarray(np.atleast_2d(direction), dtype=np.float64)
        if d.shape[0] == 1 and o.shape[0] > 1:
            d = np.repeat(d, o.shape[0], axis=0)
        out = np.empty(o.shape[0])
        raycast_many(self.z, self.x0, self.y0, self.dx, self.slope_bound(), o, d, max_range, min_step, tol, out)
        return out

    def slope_bound(self) -> float:
        """Upper bound of |gradient| of the bilinear surface (cached); used for safe ray marching."""
        if not np.isfinite(self.max_slope):
            self.max_slope = compute_max_slope(self.z, self.dx)
        return self.max_slope

    def xy_to_rc(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Fractional (row, col) of map coordinates."""
        return (self.y0 - np.asarray(y)) / self.dx, (np.asarray(x) - self.x0) / self.dx

    def save(self, folder: Path, name: str) -> None:
        """Save as ``name.npy`` + ``name.json`` (georeferencing)."""
        folder.mkdir(parents=True, exist_ok=True)
        np.save(folder / f"{name}.npy", np.ascontiguousarray(self.z, dtype=np.float32))
        (folder / f"{name}.json").write_text(json.dumps(
            dict(x0=self.x0, y0=self.y0, dx=self.dx, shape=list(self.z.shape),
                 max_slope=float(self.slope_bound()), convention="point, (x0,y0)=centre of upper-left pixel")))

    @classmethod
    def load(cls, folder: Path, name: str, mmap: bool = True) -> "Grid":
        """Load a grid saved with :meth:`save`, memory-mapped read-only by default."""
        meta = json.loads((folder / f"{name}.json").read_text())
        z = np.load(folder / f"{name}.npy", mmap_mode="r" if mmap else None)
        return cls(z=z, x0=meta["x0"], y0=meta["y0"], dx=meta["dx"], max_slope=meta.get("max_slope", float("nan")))


def compute_max_slope(z: np.ndarray, dx: float, rows_per_block: int = 2048) -> float:
    """Max of sqrt(max|dz/dx|^2 + max|dz/dy|^2) over cells (NaN-safe, blockwise)."""
    sx = sy = 0.0
    for r0 in range(0, z.shape[0], rows_per_block):
        a = np.asarray(z[r0:r0 + rows_per_block + 1], dtype=np.float64)
        gx = np.abs(np.diff(a, axis=1)); gy = np.abs(np.diff(a, axis=0))
        if np.isfinite(gx).any():
            sx = max(sx, float(np.nanmax(gx)))
        if gy.size and np.isfinite(gy).any():
            sy = max(sy, float(np.nanmax(gy)))
    return math.hypot(sx, sy) / dx + 1e-6


@njit(cache=True, inline="always")
def interp1(z, x0, y0, dx, x, y):
    """Bilinear interpolation at one point; NaN outside the grid or next to NaN posts."""
    c = (x - x0) / dx
    r = (y0 - y) / dx
    nr, nc = z.shape
    if not (r >= 0.0 and c >= 0.0 and r <= nr - 1 and c <= nc - 1):
        return np.nan
    i = int(r)
    j = int(c)
    if i > nr - 2:
        i = nr - 2
    if j > nc - 2:
        j = nc - 2
    fr = r - i
    fc = c - j
    return ((1.0 - fr) * ((1.0 - fc) * z[i, j] + fc * z[i, j + 1])
            + fr * ((1.0 - fc) * z[i + 1, j] + fc * z[i + 1, j + 1]))


@njit(cache=True, parallel=True)
def interp_many(z, x0, y0, dx, xs, ys, out):
    for k in prange(xs.shape[0]):
        out[k] = interp1(z, x0, y0, dx, xs[k], ys[k])


@njit(cache=True)
def raycast1(z, x0, y0, dx, smax, px, py, pz, ux, uy, uz, rmax, min_step, tol):
    """Range along unit direction u from p to the first terrain intersection (NaN if none / unknown).

    Safe ray marching: the height gap above terrain can shrink at most at rate |u_z| + smax*|u_h| per
    metre, so stepping gap/rate never jumps over terrain; the final bracket is refined by bisection.
    """
    t = interp1(z, x0, y0, dx, px, py)
    if np.isnan(t):
        return np.nan
    gap = pz - t
    if gap <= 0.0:
        return 0.0
    uh = math.sqrt(ux * ux + uy * uy)
    rate = -uz + smax * uh
    if rate <= 1e-9:
        return np.nan  # ray never descends faster than terrain could rise
    r = 0.0
    for _ in range(100000):
        step = gap / rate
        if step < min_step:
            step = min_step
        rn = r + step
        if rn > rmax:
            rn = rmax
        t = interp1(z, x0, y0, dx, px + ux * rn, py + uy * rn)
        if np.isnan(t):
            return np.nan
        g = pz + uz * rn - t
        if g < 0.0:
            lo = r
            hi = rn
            for _k in range(60):
                mid = 0.5 * (lo + hi)
                tm = interp1(z, x0, y0, dx, px + ux * mid, py + uy * mid)
                if np.isnan(tm):
                    return np.nan
                if pz + uz * mid - tm > 0.0:
                    lo = mid
                else:
                    hi = mid
                if hi - lo < 1e-3:
                    break
            return 0.5 * (lo + hi)
        if g <= tol:
            return rn
        if rn >= rmax:
            return np.nan
        r = rn
        gap = g
    return np.nan


@njit(cache=True, parallel=True)
def raycast_many(z, x0, y0, dx, smax, o, d, rmax, min_step, tol, out):
    for k in prange(o.shape[0]):
        out[k] = raycast1(z, x0, y0, dx, smax, o[k, 0], o[k, 1], o[k, 2], d[k, 0], d[k, 1], d[k, 2],
                          rmax, min_step, tol)

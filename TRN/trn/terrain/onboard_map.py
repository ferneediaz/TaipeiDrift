"""ONBOARD map container: the only terrain the navigation filters may use.

The container holds an elevation grid and its slope grid. It is built by
:mod:`trn.terrain.onboard_builder` from the MOI DEM tile (never the DSM). This module deliberately imports
no raster I/O so that filters importing it cannot reach any file-based data.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from trn.terrain.grid import Grid


def gaussian_random_field(shape: tuple[int, int], corr_cells: float, rng: np.random.Generator) -> np.ndarray:
    """Zero-mean, unit-variance periodic Gaussian random field with Gaussian correlation (FFT method)."""
    ky = np.fft.fftfreq(shape[0])[:, None]
    kx = np.fft.rfftfreq(shape[1])[None, :]
    spec = np.exp(-0.5 * (2 * np.pi * corr_cells) ** 2 * (kx ** 2 + ky ** 2) / 2.0)
    w = np.fft.rfft2(rng.standard_normal(shape))
    f = np.fft.irfft2(w * spec, s=shape)
    f -= f.mean()
    return (f / (f.std() + 1e-12)).astype(np.float32)


@dataclass
class OnboardMap:
    """Onboard elevation map with precomputed slope grid (for the map-error model)."""

    grid: Grid
    slope: Grid
    description: str

    def height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return self.grid.interp(x, y)

"""Resampling and roughening."""
from __future__ import annotations

import numpy as np


def systematic_resample(w: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Systematic resampling indices for normalised weights w."""
    n = w.size
    u = (rng.random() + np.arange(n)) / n
    c = np.cumsum(w)
    c[-1] = 1.0
    return np.searchsorted(c, u)


def roughen(x: np.ndarray, K: float, rng: np.random.Generator) -> None:
    """Gordon roughening in place: jitter sigma_d = K * E_d * N^(-1/d), E_d = particle range in dim d."""
    n, d = x.shape
    E = x.max(axis=0) - x.min(axis=0)
    x += rng.standard_normal(x.shape) * (K * E * n ** (-1.0 / d))

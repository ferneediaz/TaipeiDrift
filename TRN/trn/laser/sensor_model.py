"""Sensor knowledge shared by simulator and filters (beam geometry, detection curve). No truth data."""
from __future__ import annotations

import math

import numpy as np
from numba import njit


def beam_body_vectors(beams_deg: list) -> np.ndarray:
    """Unit beam vectors in body FLU from [off-nadir deg, azimuth deg (0 fwd, + right)] pairs."""
    out = []
    for a, az in beams_deg:
        a, az = math.radians(a), math.radians(az)
        out.append([math.sin(a) * math.cos(az), -math.sin(a) * math.sin(az), -math.cos(a)])
    return np.array(out)


@njit(cache=True)
def p_detect(r, r50, width, pmax):
    """Clear-air detection probability of a hard-target return at range r (logistic)."""
    return pmax / (1.0 + math.exp((r - r50) / width))

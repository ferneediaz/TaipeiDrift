"""Coordinate frames: WGS84 lon/lat <-> TWD97 TM2 (EPSG:3826) and the local-level ENU frame.

The navigation frame is East-North-Up aligned with TM2 grid axes. Meridian convergence
(< 0.3 deg on the routes) and the TM2 scale factor (0.9999) are neglected (documented in REPORT.md).
"""
from __future__ import annotations

import numpy as np
from pyproj import Transformer

_TO_TM2 = Transformer.from_crs("EPSG:4326", "EPSG:3826", always_xy=True)
_TO_LL = Transformer.from_crs("EPSG:3826", "EPSG:4326", always_xy=True)


def lonlat_to_tm2(lonlat: np.ndarray) -> np.ndarray:
    """Convert an (n, 2) array of [lon, lat] degrees to (n, 2) TM2 [E, N] metres."""
    ll = np.asarray(lonlat, float)
    e, n = _TO_TM2.transform(ll[:, 0], ll[:, 1])
    return np.column_stack([e, n])


def tm2_to_lonlat(en: np.ndarray) -> np.ndarray:
    """Convert (n, 2) TM2 [E, N] to (n, 2) [lon, lat] degrees."""
    en = np.asarray(en, float)
    lo, la = _TO_LL.transform(en[:, 0], en[:, 1])
    return np.column_stack([lo, la])


def latitude_model(e_ref: float, n_ref: float) -> tuple[float, float]:
    """Return (lat_ref_rad, n_ref) for the linear latitude model lat(N) = lat_ref + (N - n_ref)/R."""
    lat = tm2_to_lonlat(np.array([[e_ref, n_ref]]))[0, 1]
    return float(np.radians(lat)), float(n_ref)

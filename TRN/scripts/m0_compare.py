"""M0: grid alignment and vertical consistency check DSM vs DEM (read-only, strip-wise)."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rasterio
from rasterio.windows import Window, from_bounds
from scipy.ndimage import uniform_filter

DSM = Path("Data/DSM/DSMg_tawiwan_20m_20240627_g14.tif")
DEM = Path("Data/DEM/DEM_tawiwan_V2025.tif")
STRIP = 1024
FLAT_SLOPE_DEG = 1.0      # "flat" threshold
SMOOTH_STD_M = 0.3        # 3x3 DSM std below this -> no trees/buildings (bare)
LOW_ELEV_M = 150.0        # plains


def stats(x: np.ndarray) -> dict:
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {"n": 0}
    q = np.percentile(x, [1, 5, 25, 50, 75, 95, 99])
    return dict(n=int(x.size), mean=float(x.mean()), std=float(x.std()), median=float(q[3]),
                p01=float(q[0]), p05=float(q[1]), p25=float(q[2]), p75=float(q[4]), p95=float(q[5]), p99=float(q[6]),
                mad_robust_std=float(1.4826 * np.median(np.abs(x - q[3]))))


with rasterio.open(DSM) as s, rasterio.open(DEM) as d:
    col_off = (d.transform.c - s.transform.c) / s.transform.a
    row_off = (d.transform.f - s.transform.f) / s.transform.e
    print("DEM origin in DSM pixel coords: col", col_off, "row", row_off)
    assert col_off.is_integer() and row_off.is_integer()
    co, ro = int(col_off), int(row_off)
    # DSM valid but DEM nodata (outside DEM footprint or masked) and vice-versa
    acc = {k: [] for k in ["all", "flat_low", "flat_low_bare", "dem_only_valid_dsm_vals"]}
    n_dsm_only = n_dem_only = n_both = 0
    rng = np.random.default_rng(0)
    for r0 in range(0, d.height, STRIP):
        h = min(STRIP + 2, d.height - r0)
        b = d.read(1, window=Window(0, r0, d.width, h))
        a = s.read(1, window=Window(co, ro + r0, d.width, h))
        va, vb = a != s.nodata, b != d.nodata
        n_both += int((va & vb).sum()); n_dsm_only += int((va & ~vb).sum()); n_dem_only += int((~va & vb).sum())
        A = np.where(va, a, np.nan).astype(np.float64); B = np.where(vb, b, np.nan).astype(np.float64)
        diff = A - B
        gy, gx = np.gradient(B, 20.0)
        slope = np.degrees(np.arctan(np.hypot(gx, gy)))
        m = uniform_filter(np.nan_to_num(A), 3); m2 = uniform_filter(np.nan_to_num(A) ** 2, 3)
        sd = np.sqrt(np.maximum(m2 - m * m, 0))
        both = va & vb
        flat_low = both & (slope < FLAT_SLOPE_DEG) & (B < LOW_ELEV_M) & (B > 0.5)
        bare = flat_low & (sd < SMOOTH_STD_M)
        sub = lambda x, k=200_000: x if x.size <= k else rng.choice(x, k, replace=False)
        acc["all"].append(sub(diff[both])); acc["flat_low"].append(sub(diff[flat_low]))
        acc["flat_low_bare"].append(sub(diff[bare]))
        acc["dem_only_valid_dsm_vals"].append(sub(A[va & ~vb]))
    res = {k: stats(np.concatenate(v)) for k, v in acc.items()}
    res["pixels"] = dict(both=n_both, dsm_only=n_dsm_only, dem_only=n_dem_only)

    # named sample areas (TWD97 TM2 bounds: xmin, ymin, xmax, ymax)
    areas = {
        "Changhua_Yunlin_plain": (185000, 2615000, 200000, 2635000),
        "Chiayi_plain": (175000, 2585000, 190000, 2600000),
        "Pingtung_plain": (190000, 2500000, 205000, 2515000),
        "Yilan_plain": (325000, 2735000, 335000, 2745000),
        "Yushan_high_mtn": (240000, 2590000, 250000, 2600000),
    }
    for name, bb in areas.items():
        w = from_bounds(*bb, transform=d.transform).round_offsets().round_lengths()
        b = d.read(1, window=w).astype(np.float64)
        a = s.read(1, window=Window(w.col_off + co, w.row_off + ro, w.width, w.height)).astype(np.float64)
        ok = (a != s.nodata) & (b != d.nodata)
        m = uniform_filter(a, 3); m2 = uniform_filter(a ** 2, 3); sd = np.sqrt(np.maximum(m2 - m * m, 0))
        res[name] = dict(all=stats((a - b)[ok]), bare=stats((a - b)[ok & (sd < SMOOTH_STD_M)]),
                         dem_mean=float(b[ok].mean()) if ok.any() else None)

print(json.dumps(res, indent=1))

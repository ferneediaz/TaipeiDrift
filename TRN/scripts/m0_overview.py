"""M0: decimated overview maps, nodata holes, negative-value areas and large DSM-DEM differences."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rasterio
from rasterio.enums import Resampling
from rasterio.windows import Window
from scipy.ndimage import binary_fill_holes, label

DSM = Path("Data/DSM/DSMg_tawiwan_20m_20240627_g14.tif")
DEM = Path("Data/DEM/DEM_tawiwan_V2025.tif")
OUT = Path("docs/figures"); OUT.mkdir(parents=True, exist_ok=True)
DEC = 10  # 200 m overview
STRIP = 1024
res = {}

with rasterio.open(DSM) as s, rasterio.open(DEM) as d:
    co = int((d.transform.c - s.transform.c) / 20); ro = int((s.transform.f - d.transform.f) / 20)
    # exact full-res scan for holes / negative / large diffs, strip-wise
    big_diff = []; neg_vals = []; dsm_only_vals = []
    hole_cnt = 0
    for r0 in range(0, d.height, STRIP):
        h = min(STRIP, d.height - r0)
        b = d.read(1, window=Window(0, r0, d.width, h)); a = s.read(1, window=Window(co, ro + r0, d.width, h))
        va, vb = a != s.nodata, b != d.nodata
        diff = np.where(va & vb, a - b, np.nan)
        rr, cc = np.nonzero(np.abs(diff) > 60)
        for r, c in zip(rr[::50], cc[::50]):
            x, y = d.xy(r0 + r, c)
            big_diff.append((x, y, float(diff[r, c]), float(b[r, c])))
        neg_vals.append(a[va & (a < 0)]); dsm_only_vals.append(a[va & ~vb])
    res["n_absdiff_gt60m_sampled"] = len(big_diff)
    res["big_diff_examples"] = big_diff[:15]
    nv = np.concatenate(neg_vals); dv = np.concatenate(dsm_only_vals)
    u, c = np.unique(np.round(dv, 2), return_counts=True)
    res["dsm_only_top_values"] = sorted(zip(u.tolist(), c.tolist()), key=lambda t: -t[1])[:8]
    u, c = np.unique(np.round(nv, 2), return_counts=True)
    res["dsm_negative_top_values"] = sorted(zip(u.tolist(), c.tolist()), key=lambda t: -t[1])[:8]

    # decimated overviews
    shp = (d.height // DEC, d.width // DEC)
    B = d.read(1, out_shape=shp, resampling=Resampling.nearest).astype(float)
    A = s.read(1, window=Window(co, ro, d.width, d.height), out_shape=shp, resampling=Resampling.nearest,
               boundless=True, fill_value=s.nodata).astype(float)
    A[A == s.nodata] = np.nan; B[B == d.nodata] = np.nan
    # holes: nodata enclosed by valid land
    for name, Z in [("DSM", A), ("DEM", B)]:
        valid = np.isfinite(Z); filled = binary_fill_holes(valid); holes = filled & ~valid
        lab, n = label(holes)
        res[f"{name}_interior_holes_200m"] = dict(n_regions=int(n), n_px=int(holes.sum()))
    ext = (d.bounds.left, d.bounds.right, d.bounds.bottom, d.bounds.top)
    fig, ax = plt.subplots(1, 3, figsize=(15, 9), constrained_layout=True)
    for k, (Z, t, cm, vr) in enumerate([(A, "DSM 2024 (truth)", "terrain", (0, 3950)),
                                       (B, "DEM 2025 (onboard source)", "terrain", (0, 3950)),
                                       (A - B, "DSM − DEM [m]", "RdBu_r", (-30, 30))]):
        im = ax[k].imshow(Z, extent=ext, cmap=cm, vmin=vr[0], vmax=vr[1], interpolation="nearest")
        ax[k].set_title(t); ax[k].set_xlabel("E TWD97 TM2 [m]"); plt.colorbar(im, ax=ax[k], shrink=0.6)
    ax[0].set_ylabel("N [m]")
    fig.savefig(OUT / "m0_overview.png", dpi=110)
print(json.dumps(res, indent=1))

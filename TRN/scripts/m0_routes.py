"""M0: candidate route check (length, terrain clearance, max nadir range) and route overview plot."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rasterio
import yaml
from pyproj import Transformer
from rasterio.enums import Resampling
from scipy.ndimage import binary_fill_holes, label, maximum_filter

DSM = Path("Data/DSM/DSMg_tawiwan_20m_20240627_g14.tif")
DEM = Path("Data/DEM/DEM_tawiwan_V2025.tif")
ROUTES = yaml.safe_load(Path("configs/routes_candidate.yaml").read_text())
to_tm2 = Transformer.from_crs("EPSG:4326", "EPSG:3826", always_xy=True)
to_ll = Transformer.from_crs("EPSG:3826", "EPSG:4326", always_xy=True)
CORRIDOR_M = 500.0
MAX_RANGE = 5000.0
MIN_CLEAR = 300.0
res = {}

with rasterio.open(DSM) as s, rasterio.open(DEM) as d:
    # locate interior nodata holes (200 m decimation)
    shp = (d.height // 10, d.width // 10)
    B = d.read(1, out_shape=shp, resampling=Resampling.nearest)
    valid = B != d.nodata
    holes = binary_fill_holes(valid) & ~valid
    lab, n = label(holes)
    big = []
    for i in range(1, n + 1):
        rr, cc = np.nonzero(lab == i)
        if rr.size > 50:
            x0, y0 = d.transform * (cc.min() * 10, rr.max() * 10 + 10)
            x1, y1 = d.transform * (cc.max() * 10 + 10, rr.min() * 10)
            lo0, la0 = to_ll.transform(x0, y0); lo1, la1 = to_ll.transform(x1, y1)
            big.append(dict(tm2=[x0, y0, x1, y1], lonlat=[round(lo0, 4), round(la0, 4), round(lo1, 4), round(la1, 4)],
                            km2=rr.size * 0.04))
    res["large_nodata_holes"] = big

    fig, ax = plt.subplots(figsize=(8, 11), constrained_layout=True)
    Z = np.where(valid, B, np.nan)
    ext = (d.bounds.left, d.bounds.right, d.bounds.bottom, d.bounds.top)
    im = ax.imshow(Z, extent=ext, cmap="terrain", vmin=0, vmax=3950)
    plt.colorbar(im, ax=ax, shrink=0.5, label="DEM elevation [m]")
    for name, r in ROUTES.items():
        ll = np.array(r["waypoints_lonlat"], float)
        x, y = to_tm2.transform(ll[:, 0], ll[:, 1])
        seg = np.hypot(np.diff(x), np.diff(y)); L = seg.sum()
        # densify every 20 m, sample DSM max in a +-CORRIDOR_M box
        t = np.concatenate([[0], np.cumsum(seg)]); ss = np.arange(0, L, 20.0)
        px, py = np.interp(ss, t, x), np.interp(ss, t, y)
        xmin, xmax = px.min() - CORRIDOR_M, px.max() + CORRIDOR_M
        ymin, ymax = py.min() - CORRIDOR_M, py.max() + CORRIDOR_M
        win = rasterio.windows.from_bounds(xmin, ymin, xmax, ymax, s.transform).round_offsets().round_lengths()
        A = s.read(1, window=win).astype(float); A[A == s.nodata] = np.nan
        wt = s.window_transform(win)
        cc, rr = ~wt * (px, py); cc = cc.astype(int); rr = rr.astype(int)
        under = A[rr, cc]
        k = int(CORRIDOR_M / 20) * 2 + 1
        cmax = maximum_filter(np.nan_to_num(A, nan=-1e3), size=k)[rr, cc]
        alt = float(r["altitude_msl_m"])
        res[name] = dict(length_km=round(L / 1000, 1), alt_msl=alt,
                         terrain_min=float(np.nanmin(under)), terrain_max=float(np.nanmax(under)),
                         terrain_mean=float(np.nanmean(under)), terrain_std=float(np.nanstd(under)),
                         corridor_max=float(cmax.max()), min_clearance_corridor=float(alt - cmax.max()),
                         max_nadir_range=float(alt - np.nanmin(under)),
                         max_slant25_range=float((alt - np.nanmin(under)) / np.cos(np.radians(25))),
                         frac_nodata_under=float(np.isnan(under).mean()),
                         flight_time_min_at_35mps=round(L / 35 / 60, 1))
        res[name]["ok"] = bool(res[name]["min_clearance_corridor"] >= MIN_CLEAR and res[name]["max_nadir_range"] <= MAX_RANGE)
        ax.plot(x, y, "-o", ms=3, lw=2, label=f"{name} ({L/1000:.0f} km, {alt:.0f} m MSL)")
        ax.annotate(name.split("_")[0], (x[0], y[0]), fontsize=9, color="k", weight="bold")
    for h in big:
        x0, y0, x1, y1 = h["tm2"]
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="red", lw=1.5))
    ax.set_title("Candidate TRN routes on MOI DEM (TWD97 TM2); red = nodata hole")
    ax.legend(loc="lower right", fontsize=8); ax.set_xlabel("E [m]"); ax.set_ylabel("N [m]")
    fig.savefig("docs/figures/m0_routes.png", dpi=110)
print(json.dumps(res, indent=1))

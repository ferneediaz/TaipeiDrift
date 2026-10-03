#!/usr/bin/env python3
"""Check the OpenDroneMap products of the 2019-09-16 Tuniu survey flight and put them on the step-1 grid.

Inputs (all produced by OpenDroneMap 3.6.2, nothing re-computed here):
  data/processed/x_tuniu_survey_odm/odm_orthophoto/odm_orthophoto.tif  (5 cm, UTM 51N, RGBA)
  data/processed/x_tuniu_survey_odm/odm_dem/{dsm,dtm}.tif             (10 cm, ellipsoidal heights)

1. Format conversion only: the ODM orthophoto is resampled onto the step-1 map grid (EPSG:3826,
   0.25 / 0.5 / 1 m) with the SAME code as the OpenAerialMap maps (x2_tuniu_map.read_cog_to_grid /
   write_rgba) -> data/raw/x_tuniu/maps/odm_{res}m.tif, readable by x_tuniu_geo.MapRaster.
2. Horizontal check: ZNCC shift of 80 m tiles of the ODM map against the OAM 2019-12-12 map (0.5 m) and
   against the OAM 2019-09-16 scene (1 m, built by the photo author from the same photos). Combined with
   the step-1 pre-cut calibration (OAM 2019-12 minus April RTK), this gives ODM minus April RTK.
3. Vertical check: ODM DTM against Copernicus GLO-30 (+ EGM2008 undulation = ellipsoidal), and DSM - DTM.

Run: .venv/bin/python experiments/x6_tuniu_odm_check.py
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import rasterio
from rasterio.warp import Resampling, reproject

import x_tuniu_geo as G
from x2_tuniu_map import read_cog_to_grid, write_rgba

ROOT = Path(__file__).resolve().parents[1]
ODM = ROOT / "data/processed/x_tuniu_survey_odm"
ORTHO = ODM / "odm_orthophoto/odm_orthophoto.tif"
SEPT_SCENE = G.MAPDIR / "scene_5df8a789a8b3c40005179431_1m.tif"
TILE_M, SEARCH_M, MIN_PEAK = 80.0, 10.0, 0.5


def tile_shifts(odm: G.MapRaster, ref: G.MapRaster) -> np.ndarray:
    """(dE, dN) = ODM position of a feature minus its position in `ref`, per tile with a clear peak."""
    res = odm.res
    if abs(ref.res - res) > 1e-9:
        raise ValueError("maps must share the resolution")
    t, s = int(TILE_M / res), int(SEARCH_M / res)
    H, W = odm.gray.shape
    out = []
    for r0 in range(s, H - t - s, t):
        for c0 in range(s, W - t - s, t):
            if odm.valid[r0:r0 + t, c0:c0 + t].mean() < 0.98:
                continue
            x, y = odm.to_xy(c0 - s, r0 - s)
            g, v, _ = ref.window(float(x), float(y), t + 2 * s, t + 2 * s)
            if v.mean() < 0.98:
                continue
            sc = cv2.matchTemplate(g, odm.gray[r0:r0 + t, c0:c0 + t], cv2.TM_CCOEFF_NORMED)
            _, peak, _, (px, py) = cv2.minMaxLoc(sc)
            if peak >= MIN_PEAK:
                # template found at (px, py) in the ref window; zero shift is (s, s)
                out.append(((s - px) * res, -(s - py) * res, peak))
    return np.array(out)


def stats(d: np.ndarray) -> dict:
    if len(d) == 0:
        return {"tiles": 0}
    med = np.median(d[:, :2], axis=0)
    mad = np.median(np.abs(d[:, :2] - med), axis=0)
    return {"tiles": int(len(d)), "dE_m": float(med[0]), "dN_m": float(med[1]),
            "mad_E_m": float(mad[0]), "mad_N_m": float(mad[1]), "median_peak": float(np.median(d[:, 2]))}


def main() -> None:
    extent = tuple(json.loads((G.XOUT / "stage1_oam_scenes.json").read_text())["grid_3826"])
    for res in (0.25, 0.5, 1.0):
        p = G.map_path("odm", res)
        if not p.exists():
            write_rgba(p, read_cog_to_grid(str(ORTHO), res, extent), res, extent)
        print("map", p.relative_to(ROOT))

    cal = json.loads((G.XOUT / "stage1_calibration.json").read_text())["map_offsets"]["main"]
    odm05, odm1 = G.MapRaster(G.map_path("odm", 0.5)), G.MapRaster(G.map_path("odm", 1.0))
    vs_dec = stats(tile_shifts(odm05, G.MapRaster(G.map_path("main", 0.5))))
    vs_sep = stats(tile_shifts(odm1, G.MapRaster(SEPT_SCENE))) if SEPT_SCENE.exists() else {"tiles": 0}
    rtk = ({"dE_m": vs_dec["dE_m"] + cal["de_m"], "dN_m": vs_dec["dN_m"] + cal["dn_m"]}
           if vs_dec["tiles"] else None)

    with rasterio.open(ODM / "odm_dem/dtm.tif") as dtm, rasterio.open(ODM / "odm_dem/dsm.tif") as dsm, \
            rasterio.open(G.DEM_TIF) as cop:
        step = 30.0 / dtm.res[0]
        shape = (int(dtm.height / step), int(dtm.width / step))
        tf = dtm.transform * dtm.transform.scale(dtm.width / shape[1], dtm.height / shape[0])
        def avg(src):
            a = np.full(shape, np.nan, np.float32)
            reproject(rasterio.band(src, 1), a, dst_transform=tf, dst_crs=dtm.crs,
                      src_nodata=src.nodata, dst_nodata=np.nan, resampling=Resampling.average)
            return a
        g, s = avg(dtm), avg(dsm)
        c = np.full(shape, np.nan, np.float32)
        reproject(rasterio.band(cop, 1), c, dst_transform=tf, dst_crs=dtm.crs, dst_nodata=np.nan,
                  resampling=Resampling.bilinear)
    c = c + G.geoid_undulation_m()
    ok = np.isfinite(g) & np.isfinite(c)
    dz = (g - c)[ok]
    veg = (s - g)[np.isfinite(s) & np.isfinite(g)]
    report = {
        "products": {"orthophoto": str(ORTHO.relative_to(ROOT)), "res_m": 0.05,
                     "dem": "odm_dem/dsm.tif, dtm.tif, 0.10 m, ellipsoidal"},
        "horizontal_vs_oam_2019_12_at_0.5m": vs_dec,
        "horizontal_vs_oam_2019_09_same_photos_at_1m": vs_sep,
        "odm_minus_april_rtk_m (via step-1 OAM 2019-12 calibration)": rtk,
        "vertical_dtm_minus_copernicus_ellipsoidal_m": {
            "cells_30m": int(ok.sum()), "median": float(np.median(dz)),
            "mad": float(np.median(np.abs(dz - np.median(dz)))),
            "p5": float(np.percentile(dz, 5)), "p95": float(np.percentile(dz, 95))},
        "dsm_minus_dtm_m_30m_cells": {"p50": float(np.percentile(veg, 50)), "p90": float(np.percentile(veg, 90))},
    }
    out = G.XOUT / "odm_check.json"
    out.write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()

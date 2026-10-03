"""M0: crop aligned per-route tiles from the island-wide rasters (windowed reads, raw data read-only).

Output per route in ``data/processed/<route>/``:
  dsm20.npy/.json/.tif  truth top surface (sea set to 0 m, unknown = NaN)
  dem20.npy/.json/.tif  bare-earth DEM, same lattice (sea 0 m, holes NaN)
  sea.npy               uint8 sea mask derived from the DEM only
The DSM and DEM share one 20 m lattice (PLAN.md §1.3), so no resampling is needed.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rasterio
from rasterio.crs import CRS
from rasterio.transform import from_origin
from rasterio.windows import Window
from scipy.ndimage import binary_fill_holes

from trn.common.config import load_base_config, project_path
from trn.common.frames import lonlat_to_tm2
from trn.terrain.grid import Grid


def route_bbox(route_cfg: dict, margin: float) -> tuple[float, float, float, float]:
    """(xmin, ymin, xmax, ymax) of the route waypoints plus margin, in TM2 metres."""
    en = lonlat_to_tm2(np.array(route_cfg["waypoints_lonlat"]))
    return (en[:, 0].min() - margin, en[:, 1].min() - margin, en[:, 0].max() + margin, en[:, 1].max() + margin)


def _read_window(path: str, bbox: tuple, strip: int) -> tuple[np.ndarray, float, float, float]:
    """Read the bbox from a raster strip-wise, boundless (outside -> NaN). Returns z, x0, y0 (centres), dx."""
    with rasterio.open(path) as ds:
        dx = ds.transform.a
        c0 = int(np.floor((bbox[0] - ds.transform.c) / dx))
        c1 = int(np.ceil((bbox[2] - ds.transform.c) / dx))
        r0 = int(np.floor((ds.transform.f - bbox[3]) / dx))
        r1 = int(np.ceil((ds.transform.f - bbox[1]) / dx))
        out = np.empty((r1 - r0, c1 - c0), np.float32)
        for rr in range(r0, r1, strip):
            h = min(strip, r1 - rr)
            a = ds.read(1, window=Window(c0, rr, c1 - c0, h), boundless=True, fill_value=ds.nodata)
            out[rr - r0:rr - r0 + h] = np.where(a == ds.nodata, np.nan, a)
        x0 = ds.transform.c + (c0 + 0.5) * dx
        y0 = ds.transform.f - (r0 + 0.5) * dx
    return out, x0, y0, dx


def _write_tif(path: Path, z: np.ndarray, x0: float, y0: float, dx: float, crs: str) -> None:
    prof = dict(driver="GTiff", height=z.shape[0], width=z.shape[1], count=1, dtype=z.dtype.name,
                crs=CRS.from_string(crs), transform=from_origin(x0 - dx / 2, y0 + dx / 2, dx, dx),
                compress="deflate", predictor=3 if z.dtype.kind == "f" else 2, tiled=True,
                nodata=np.nan if z.dtype.kind == "f" else None)
    with rasterio.open(path, "w", **prof) as dst:
        dst.write(z, 1)
        dst.update_tags(AREA_OR_POINT="Point")


def build_route_tiles(route: str, cfg: dict | None = None, force: bool = False) -> Path:
    """Create the aligned tiles for one route (idempotent)."""
    cfg = cfg or load_base_config()
    d = cfg["data"]
    out = project_path(d["processed_dir"]) / route
    if (out / "dem20.json").exists() and (out / "dsm20.json").exists() and not force:
        return out
    out.mkdir(parents=True, exist_ok=True)
    bbox = route_bbox(cfg["routes"][route], d["tile_margin_m"])
    dem, x0, y0, dx = _read_window(d["dem_path"], bbox, d["read_strip_rows"])
    dsm, x0s, y0s, dxs = _read_window(d["dsm_path"], bbox, d["read_strip_rows"])
    assert (x0, y0, dx) == (x0s, y0s, dxs) and dem.shape == dsm.shape, "DSM/DEM lattices differ"
    valid = np.isfinite(dem)
    land = binary_fill_holes(valid)          # sea = DEM nodata connected to the outside
    sea = ~land
    holes = land & ~valid
    dem[sea] = 0.0
    dsm[sea] = 0.0
    dsm[holes] = np.nan
    Grid(dsm, x0, y0, dx).save(out, "dsm20")
    Grid(dem, x0, y0, dx).save(out, "dem20")
    np.save(out / "sea.npy", sea.astype(np.uint8))
    _write_tif(out / "dsm20.tif", dsm, x0, y0, dx, d["crs"])
    _write_tif(out / "dem20.tif", dem, x0, y0, dx, d["crs"])
    (out / "tile.json").write_text(json.dumps(dict(
        route=route, bbox_tm2=bbox, shape=list(dem.shape), dx=dx, crs=d["crs"],
        frac_sea=float(sea.mean()), frac_holes=float(holes.mean()),
        frac_dsm_nan_land=float((np.isnan(dsm) & land).mean())), indent=1))
    return out


if __name__ == "__main__":
    c = load_base_config()
    for r in c["routes"]:
        p = build_route_tiles(r, c)
        print(r, (p / "tile.json").read_text())

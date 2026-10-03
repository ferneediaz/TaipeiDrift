"""Build the ONBOARD map from the MOI DEM 2025 tile (never the DSM).

Resampled to 30/40 m (area average), optionally corrupted with a smooth correlated error field and a
horizontal shift. Placeholder for Copernicus GLO-30 (EGM2008 heights on WGS84: reproject to EPSG:3826
and convert to TWVD2001 heights).
"""
from __future__ import annotations

import numpy as np
from rasterio.enums import Resampling
from rasterio.transform import from_origin
from rasterio.warp import reproject

from trn.common.config import project_path
from trn.terrain.grid import Grid
from trn.terrain.metrics import slope_grid
from trn.terrain.onboard_map import OnboardMap, gaussian_random_field


def resample_average(src: Grid, res: float) -> Grid:
    """Area-average resampling onto a coarser grid covering the same extent."""
    h, w = src.z.shape
    xmin, ymax = src.x0 - src.dx / 2, src.y0 + src.dx / 2
    nw, nh = int((w * src.dx) // res), int((h * src.dx) // res)
    dst = np.full((nh, nw), np.nan, np.float32)
    reproject(np.asarray(src.z, np.float32), dst,
              src_transform=from_origin(xmin, ymax, src.dx, src.dx), src_crs="EPSG:3826",
              dst_transform=from_origin(xmin, ymax, res, res), dst_crs="EPSG:3826",
              src_nodata=np.nan, dst_nodata=np.nan, resampling=Resampling.average)
    return Grid(dst, xmin + res / 2, ymax - res / 2, res)


def load_glo30(oc: dict) -> OnboardMap:
    raise NotImplementedError(
        "Copernicus GLO-30 loader placeholder: reproject from EPSG:4326/EGM2008 to EPSG:3826, convert EGM2008 "
        "heights to TWVD2001 (check the offset on flat bare areas as in PLAN.md section 1.4), then resample to the "
        "configured resolution. Set onboard_map.glo30.path.")


def build_onboard_map(cfg: dict, route: str, rng: np.random.Generator | None = None) -> OnboardMap:
    """Build from config section ``onboard_map``. The deterministic resampled DEM is cached on disk."""
    oc = cfg["onboard_map"]
    if oc["source"] == "glo30":
        return load_glo30(oc)
    folder = project_path(cfg["data"]["processed_dir"]) / route
    res = float(oc["resolution_m"])
    name = f"onboard_dem_{int(res)}m"
    if not (folder / f"{name}.json").exists():
        resample_average(Grid.load(folder, "dem20"), res).save(folder, name)
    g = Grid.load(folder, name)
    es = float(oc["error_field"]["sigma_m"])
    shift = np.asarray(oc["shift_m"], float)
    desc = f"MOI DEM 2025 -> {res:.0f} m"
    if es > 0:
        if rng is None:
            raise ValueError("error field needs an rng")
        corr = float(oc["error_field"]["corr_length_m"]) / res
        z = np.asarray(g.z, np.float32) + es * gaussian_random_field(g.z.shape, corr, rng)
        g = Grid(z, g.x0, g.y0, g.dx)
        desc += f", error field sigma {es} m / L {oc['error_field']['corr_length_m']} m"
    if np.any(shift != 0):
        g = Grid(g.z, g.x0 + shift[0], g.y0 + shift[1], g.dx, g.max_slope)
        desc += f", shift {shift.tolist()} m"
    return OnboardMap(grid=g, slope=Grid(slope_grid(g.z, g.dx), g.x0, g.y0, g.dx), description=desc)

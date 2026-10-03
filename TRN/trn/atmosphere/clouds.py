"""Cloud and valley-fog model (simulator side).

* Cloud layer: base altitude (MSL, per route); thickness from a thresholded Gaussian random field so
  that the covered fraction equals ``cloud_fraction``; thickness tapers to zero at cloud edges (thin,
  semi-transparent edges). Advected with the wind (the periodic field is shifted by wind * t).
* Valley fog: in valleys (terrain below its local mean) and below a fog-top altitude, patchy coverage
  from a second random field. Generated from the (truth) terrain itself.
* Clear-air extinction with exponential height profile.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import uniform_filter

from trn.terrain.onboard_map import gaussian_random_field


@dataclass
class AtmosphereFields:
    """Arrays consumed by the numba laser simulator (all extinctions in 1/m)."""

    enabled: bool
    x0: float                 # centre of upper-left field cell
    y0: float
    res: float
    thickness: np.ndarray     # (h,w) cloud thickness [m], 0 = clear
    base: float               # cloud base MSL [m]
    ext_cloud: float
    fog: np.ndarray           # (h,w) uint8 fog present
    fog_top: float
    ext_fog: float
    ext0: float               # clear-air extinction at sea level
    scale_h: float
    wind: np.ndarray          # (2,) m/s
    cloud_fraction: float

    def as_tuple(self) -> tuple:
        return (self.x0, self.y0, self.res, self.thickness, self.base, self.ext_cloud, self.fog, self.fog_top,
                self.ext_fog, self.ext0, self.scale_h, float(self.wind[0]), float(self.wind[1]),
                1 if self.enabled else 0)


def build_atmosphere(cfg: dict, route: str, ground_grid, rng: np.random.Generator) -> AtmosphereFields:
    """Create the cloud/fog fields for one run on a coarse grid covering the route tile."""
    ac = cfg["atmosphere"]
    res = float(ac["field_res_m"])
    f = int(round(res / ground_grid.dx))
    zg = np.asarray(ground_grid.z, np.float64)
    h, w = (zg.shape[0] // f) * f, (zg.shape[1] // f) * f
    G = np.nan_to_num(zg[:h, :w], nan=0.0).reshape(h // f, f, w // f, f).mean(axis=(1, 3))
    x0 = ground_grid.x0 + (f - 1) * ground_grid.dx / 2
    y0 = ground_grid.y0 - (f - 1) * ground_grid.dx / 2
    cf = float(ac["cloud_fraction"]) if ac["enabled"] else 0.0
    lay = ac["layer"]
    thick = np.zeros(G.shape, np.float32)
    if cf > 0:
        fld = gaussian_random_field(G.shape, lay["corr_length_m"] / res, rng)
        thr = np.quantile(fld, 1.0 - cf)
        thick = (lay["max_thickness_m"] * np.clip((fld - thr) / lay["edge_width"], 0.0, 1.0)).astype(np.float32)
        thick[fld <= thr] = 0.0
    fog = np.zeros(G.shape, np.uint8)
    vf = ac["valley_fog"]
    fog_top = float(vf["top_msl_m"][route])
    if ac["enabled"] and vf["enabled"]:
        frac = float(vf["fraction"]) * (cf if vf["scale_with_cloud_fraction"] else 1.0)
        if frac > 0:
            tpi = G - uniform_filter(G, size=max(3, int(2 * vf["tpi_radius_m"] / res) + 1), mode="nearest")
            valley = (G < fog_top) & (tpi < 0) & (G > 0.5)
            if valley.any():
                f2 = gaussian_random_field(G.shape, vf["corr_length_m"] / res, rng)
                thr2 = np.quantile(f2[valley], 1.0 - min(frac, 1.0))
                fog = (valley & (f2 >= thr2)).astype(np.uint8)
    ca = ac["clear_air"]
    return AtmosphereFields(
        enabled=bool(ac["enabled"]), x0=x0, y0=y0, res=res, thickness=thick,
        base=float(lay["base_msl_m"][route]), ext_cloud=lay["ext_per_km"] / 1000.0, fog=fog, fog_top=fog_top,
        ext_fog=vf["ext_per_km"] / 1000.0, ext0=ca["ext_surface_per_km"] / 1000.0,
        scale_h=float(ca["scale_height_m"]), wind=np.asarray(ac["wind_mps"], float), cloud_fraction=cf)

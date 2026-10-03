"""Shared geometry for the Tuniu real-photo experiments (x1..x4).

Frames
  map    EPSG:3826 (TWD97 / TM2), x = east, y = north, metres. Grid convergence here is -0.02 deg and
         the scale factor 0.9999, so map axes are used as local east/north.
  camera OpenCV: x right, y down, z forward.
  DJI gimbal angles (XMP) are absolute: yaw from north clockwise, pitch negative = down, roll.

Ground patch convention (rectify): a north-up raster at `res` m/px whose pixel (col, row) centre is at
ground offset (x0 + col*res, y0 - row*res) metres east/north of the camera NADIR. So the nadir sits at
patch pixel (-x0/res, y0/res). The photo centre is ~58 m ahead of the nadir at 30 deg off-nadir; the
estimators locate the nadir, never the image centre.
"""
from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from pyproj import Transformer

ROOT = Path(__file__).resolve().parents[1]
SEQ = ROOT / "data/processed/t_replay/tuniu_tw_1"
XOUT = ROOT / "data/processed/x_tuniu"
RAW = ROOT / "data/raw/x_tuniu"
MAPDIR = RAW / "maps"
DEM_TIF = RAW / "copernicus_glo30_tuniu.tif"
RESOLUTIONS = (0.1, 0.25, 0.5, 1.0)

TO_3826 = Transformer.from_crs("EPSG:4326", "EPSG:3826", always_xy=True)
TO_LONLAT = Transformer.from_crs("EPSG:3826", "EPSG:4326", always_xy=True)

# Simulated barometer, error model fitted on real logs (docs/research/overnight-synthesis.md, row 4)
BARO_WHITE_M = 0.30
BARO_RW_M_SQRT_S = 0.112
BARO_RAMP_SD_M_S = 0.0024

R_BODY_CAM = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])   # columns: cam x,y,z in body (fwd,right,down)
R_ENU_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])


# ----------------------------------------------------------------------------- data

def meta() -> dict:
    return json.loads((SEQ / "meta.json").read_text())


def protocol() -> dict:
    return json.loads((XOUT / "stage0_protocol.json").read_text())


def photos() -> pd.DataFrame:
    """Estimator-side table: frame, t_s, path, DJI fused attitude. No position."""
    img = pd.read_csv(SEQ / "images.csv")
    att = pd.read_csv(SEQ / "attitude.csv")
    df = att.copy()
    df["path"] = [str(SEQ / p) for p in img.path]
    return df


def truth_xy() -> pd.DataFrame:
    """EVALUATOR / pre-cut calibration only: RTK in EPSG:3826 + ellipsoidal altitude."""
    tr = pd.read_csv(SEQ / "truth.csv")
    x, y = TO_3826.transform(tr.lon_deg.to_numpy(), tr.lat_deg.to_numpy())
    return pd.DataFrame({"frame": np.arange(1, len(tr) + 1), "t_s": tr.t_s, "x": x, "y": y, "alt_ell": tr.alt_m})


@lru_cache(maxsize=1)
def geoid_undulation_m() -> float:
    """EGM2008 undulation at the origin (PROJ network grid). Copernicus heights are EGM2008."""
    from pyproj import network
    network.set_network_enabled(True)
    m = meta()["origin"]
    t = Transformer.from_crs("EPSG:4979", "EPSG:4326+3855", always_xy=True)
    _, _, h = t.transform(m["lon_deg"], m["lat_deg"], 0.0)
    return float(-h)


# ----------------------------------------------------------------------------- DEM

@lru_cache(maxsize=1)
def _dem():
    import rasterio
    with rasterio.open(DEM_TIF) as ds:
        return ds.read(1).astype(np.float32), ds.transform


def dem_height(x, y) -> np.ndarray:
    """Copernicus GLO-30 height (EGM2008, metres) at EPSG:3826 points, bilinear."""
    data, t = _dem()
    lon, lat = TO_LONLAT.transform(np.asarray(x, float).ravel(), np.asarray(y, float).ravel())
    col = (lon - t.c) / t.a - 0.5
    row = (lat - t.f) / t.e - 0.5
    from scipy.ndimage import map_coordinates
    out = map_coordinates(data, [row, col], order=1, mode="nearest")
    return out.reshape(np.shape(x))


def dem_ellipsoidal(x, y) -> np.ndarray:
    return dem_height(x, y) + geoid_undulation_m()


# ----------------------------------------------------------------------------- barometer

def simulated_baro(t_s: np.ndarray, alt_true: np.ndarray, seed: int, cut_s: float) -> np.ndarray:
    """SIMULATED barometer: truth + white + random walk + per-flight ramp. The constant offset is
    calibrated on pre-cut samples (allowed: RTK available before the cut)."""
    rng = np.random.default_rng(seed)
    t = np.asarray(t_s, float)
    dt = np.diff(t, prepend=t[0])
    rw = np.cumsum(rng.normal(0, 1, len(t)) * BARO_RW_M_SQRT_S * np.sqrt(np.maximum(dt, 0)))
    ramp = rng.normal(0, BARO_RAMP_SD_M_S) * t
    raw = alt_true + rng.normal(0, BARO_WHITE_M, len(t)) + rw + ramp + rng.normal(0, 50)  # unknown offset
    pre = t < cut_s
    return raw - np.mean(raw[pre] - alt_true[pre])


# ----------------------------------------------------------------------------- camera

class Camera:
    def __init__(self, cam: dict | None = None):
        cam = cam or meta()["camera"]
        self.w, self.h = cam["image_width_px"], cam["image_height_px"]
        self.K = np.array([[cam["fx_px"], 0, cam["cx_px"]], [0, cam["fy_px"], cam["cy_px"]], [0, 0, 1.]])
        self.dist = np.array(cam["distortion_k1_k2_p1_p2_k3"], float)
        # largest undistorted normalised radius inside the image (distortion is monotonic up to it)
        corners = np.array([[0, 0], [self.w - 1, 0], [0, self.h - 1], [self.w - 1, self.h - 1]], np.float32)
        und = cv2.undistortPoints(corners[:, None], self.K, self.dist)[:, 0]
        self.r_max = float(np.hypot(und[:, 0], und[:, 1]).max()) * 1.02

    def scaled_K(self, factor: float) -> np.ndarray:
        """K for an image downscaled by `factor` (pixel centres convention of cv2.resize)."""
        K = self.K.copy()
        K[0, 0] /= factor
        K[1, 1] /= factor
        K[0, 2] = (K[0, 2] + 0.5) / factor - 0.5
        K[1, 2] = (K[1, 2] + 0.5) / factor - 0.5
        return K


def rot_enu_cam(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """Camera(OpenCV) -> ENU rotation from DJI absolute gimbal angles."""
    y, p, r = np.radians([yaw_deg, pitch_deg, roll_deg])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return R_ENU_NED @ (Rz @ Ry @ Rx) @ R_BODY_CAM


REDUCE_FLAGS = {1: cv2.IMREAD_GRAYSCALE, 2: cv2.IMREAD_REDUCED_GRAYSCALE_2,
                4: cv2.IMREAD_REDUCED_GRAYSCALE_4, 8: cv2.IMREAD_REDUCED_GRAYSCALE_8}


def reduce_factor(res: float) -> int:
    """JPEG decode reduction so that the source GSD (~2.7-4 cm) stays below res / 1.5."""
    f = 1
    while f < 8 and 0.04 * f * 2 <= res / 1.5:
        f *= 2
    return f


def load_gray(path: str, factor: int) -> np.ndarray:
    img = cv2.imread(path, REDUCE_FLAGS[factor])
    if img is None:
        raise FileNotFoundError(path)
    return img


def footprint_offsets(cam: Camera, R: np.ndarray, height: float, far_m: float, n: int = 64) -> np.ndarray:
    """Ground offsets (E, N) from the nadir of the image border, flat ground `height` m below,
    clipped to `far_m` ahead of the nadir along the camera heading."""
    w, h = cam.w, cam.h
    s = np.linspace(0, 1, n)
    border = np.concatenate([np.c_[s * (w - 1), np.zeros(n)], np.c_[np.full(n, w - 1), s * (h - 1)],
                             np.c_[(1 - s) * (w - 1), np.full(n, h - 1)], np.c_[np.zeros(n), (1 - s) * (h - 1)]])
    und = cv2.undistortPoints(border.astype(np.float32)[:, None], cam.K, cam.dist)[:, 0]
    rays = (R @ np.c_[und, np.ones(len(und))].T).T
    fwd = R[:, 2].copy()
    fwd[2] = 0
    fwd /= np.linalg.norm(fwd)
    pts = []
    for d in rays:
        if d[2] >= -1e-6:
            d = d.copy()
            d[2] = -1e-6                       # above the horizon: push far, clipping takes over
        t = height / -d[2]
        p = d[:2] * t
        f = p @ fwd[:2]
        if f > far_m:
            p = p * far_m / f
        pts.append(p)
    return np.array(pts)


def rectify(img: np.ndarray, factor: int, cam: Camera, R: np.ndarray, height: float, res: float,
            far_m: float = 100.0, ground=None) -> dict:
    """North-up ground patch around the camera nadir.

    height: camera height above the ground plane (m). ground: optional callable (dx, dy) -> z offset of
    the ground relative to the flat plane (m, up positive), for DEM-lifted geometry.
    """
    poly = footprint_offsets(cam, R, height, far_m)
    x0 = math.floor(poly[:, 0].min() / res) * res
    x1 = math.ceil(poly[:, 0].max() / res) * res
    y0 = math.floor(poly[:, 1].min() / res) * res
    y1 = math.ceil(poly[:, 1].max() / res) * res
    cols, rows = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    gx = x0 + (np.arange(cols) + 0.5) * res
    gy = y1 - (np.arange(rows) + 0.5) * res
    X, Y = np.meshgrid(gx, gy)
    Z = np.full_like(X, -height)
    if ground is not None:
        # the DEM is 30 m: evaluate the lift on a ~2 m sub-grid and interpolate
        k = max(1, int(round(2.0 / res)))
        gz = ground(X[::k, ::k], Y[::k, ::k]).astype(np.float32)
        Z = Z + cv2.resize(gz, (cols, rows), interpolation=cv2.INTER_LINEAR)
    P = np.stack([X, Y, Z], -1).reshape(-1, 3)
    C = P @ R                                   # = (R^T P^T)^T, camera coordinates
    z = C[:, 2]
    ok = z > 1e-3
    xn = np.where(ok, C[:, 0] / np.where(ok, z, 1), 0)
    yn = np.where(ok, C[:, 1] / np.where(ok, z, 1), 0)
    ok &= np.hypot(xn, yn) < cam.r_max
    k1, k2, p1, p2, k3 = cam.dist
    r2 = xn * xn + yn * yn
    rad = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    xd = xn * rad + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
    yd = yn * rad + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
    K = cam.scaled_K(factor)
    u = K[0, 0] * xd + K[0, 2]
    v = K[1, 1] * yd + K[1, 2]
    fwd = R[:, 2].copy()
    fwd[2] = 0
    fwd /= np.linalg.norm(fwd)
    ok &= (P[:, 0] * fwd[0] + P[:, 1] * fwd[1]) <= far_m
    hh, ww = img.shape[:2]
    ok &= (u >= 0) & (u <= ww - 1) & (v >= 0) & (v <= hh - 1)
    mapx = np.where(ok, u, -10).astype(np.float32).reshape(rows, cols)
    mapy = np.where(ok, v, -10).astype(np.float32).reshape(rows, cols)
    patch = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    valid = ok.reshape(rows, cols)
    return dict(patch=patch, valid=valid, x0=x0 + res / 2, y0=y1 - res / 2, res=res,
                nadir_px=np.array([-(x0 + res / 2) / res, (y1 - res / 2) / res]))


# ----------------------------------------------------------------------------- maps

class MapRaster:
    """A north-up EPSG:3826 map loaded in memory (gray + valid)."""

    def __init__(self, path: Path):
        import rasterio
        with rasterio.open(path) as ds:
            rgb = ds.read([1, 2, 3])
            self.res = float(ds.transform.a)
            self.left, self.top = float(ds.transform.c), float(ds.transform.f)
            alpha = ds.read(4) if ds.count >= 4 else (rgb.max(0) > 0).astype(np.uint8) * 255
        self.gray = cv2.cvtColor(np.ascontiguousarray(rgb.transpose(1, 2, 0)), cv2.COLOR_RGB2GRAY)
        del rgb
        self.valid = alpha > 0
        self.path = path

    def to_px(self, x, y):
        return (np.asarray(x) - self.left) / self.res - 0.5, (self.top - np.asarray(y)) / self.res - 0.5

    def to_xy(self, col, row):
        return self.left + (np.asarray(col) + 0.5) * self.res, self.top - (np.asarray(row) + 0.5) * self.res

    def window(self, x_min, y_max, cols, rows):
        """Integer-pixel window whose top-left pixel centre is nearest (x_min, y_max). Returns
        gray, valid, (x of col 0 centre, y of row 0 centre). Outside the raster = invalid."""
        c0 = int(round((x_min - self.left) / self.res - 0.5))
        r0 = int(round((self.top - y_max) / self.res - 0.5))
        H, W = self.gray.shape
        g = np.zeros((rows, cols), np.uint8)
        v = np.zeros((rows, cols), bool)
        a0, b0 = max(0, r0), max(0, c0)
        a1, b1 = min(H, r0 + rows), min(W, c0 + cols)
        if a1 > a0 and b1 > b0:
            g[a0 - r0:a1 - r0, b0 - c0:b1 - c0] = self.gray[a0:a1, b0:b1]
            v[a0 - r0:a1 - r0, b0 - c0:b1 - c0] = self.valid[a0:a1, b0:b1]
        xc, yc = self.to_xy(c0, r0)
        return g, v, (float(xc), float(yc))


def map_path(key: str, res: float) -> Path:
    return MAPDIR / f"{key}_{res:g}m.tif"

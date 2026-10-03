#!/usr/bin/env python3
"""S1, team demo v1: GNSS-free closed loop with camera-to-map fixes on a simulated Wufeng flight.

Recording: taipeidrift-replay/1 (sim/nodes/recorder.py), default recordings/ilhan_wufeng_south_80m.
Ground in the simulator = the 2020 Wufeng orthophoto on flat ground (+ synthetic trees); map for fixes = the
2018 orthophoto (data/raw/aerial/wufeng_2018-05-03_x4.tif) resampled to 0.5 m/px north-up EPSG:3826.

Protocol (Dustin's convention, baseline/src/estimation/camera_navigator.py):
  flight = first image at >= 95 % of the route height .. last image before the drone slows below 2 m/s;
  GNSS until the true distance flown reaches 450 m (the cut), none afterwards. Scoring after the cut.

Estimator inputs after the cut:
  images (5 Hz; processed every --odo-step images), image times,
  roll/pitch from the truth quaternion (stand-in for an AHRS, which gets them from gravity),
  heading = truth + SIMULATED drift (per-seed constant N(0, 2 deg) + random walk 0.1 deg/sqrt(s), x5 `drift`),
  height above the take-off ground = SIMULATED barometer (x_tuniu_geo.simulated_baro; the world is flat),
  the state at the cut from the last pre-cut GNSS fix, and the pre-cut calibration. Never truth position.

Filter (x5_tuniu_closed_loop.py, + a scale state): EKF on (east, north, heading error b, scale error k).
  predict  p += (1 + k) Rot(-b) D, D = nadir displacement between consecutive processed images (every 5th
           recorded image, chosen on the pre-cut segment; rectified north-up patches + ZNCC, x4's displacement
           with the template taken inside the ground both patches see); on failure (no match, > 16 m/s,
           peak < 0.5) the last measured speed along the current heading. Noise calibrated pre-cut only (vs GNSS).
           k: prior and random walk from the SIMULATED barometer's specification. --filter x5 drops k.
  update   consensus fix at 1 Hz (every 5th recorded image): ZNCC (yaw +-4 deg, scale +-6 %) and XFeat
           (vismatch, MAGSAC, plausibility) on the same patch, window centred on the FILTER estimate, half-size
           max(45 m, 3 sigma) capped at 120 m; ACCEPT iff both exist and agree within 4 m, position = ZNCC fix;
           chi-square 99 % innovation gate.
  Dead reckoning (DR) = the same odometry stream, predict only, run in the same pass.

Commands:
  prepare-map   2018 orthophoto -> outputs/s1_sim/map_2018_0.5m.tif (EPSG:3826, 0.5 m/px, RGBA)
  calibrate     pre-cut noise model, map offset, odometry step -> outputs/s1_sim/calibration[_<camera>].json
  run           seeds x camera -> outputs/s1_sim/runs[_x5filter]/<camera>/loop_s<seed>.csv (loop + DR, scored)
  dustin        Dustin's navigator per frame (camera_alone, map_2018) -> outputs/s1_sim/dustin_<camera>/frames_*.csv
                (run baseline/scripts/run_sim_navigator.py first: it caches the camera motion)
Importable for the ESKF fusion: fix_at(image_index, prior_en, sigma_m, heading_deg, height_m).

  PY=/Users/ilhan.neuville/dev/hackathon/TaipeiDrift/.venv/bin/python
  $PY experiments/s1_sim_map_fix.py prepare-map
  $PY experiments/s1_sim_map_fix.py calibrate [--camera realistic --odo-steps 5]
  $PY experiments/s1_sim_map_fix.py run --seeds 0 1 2 3 4 --camera ideal --workers 3 [--filter x5]
  $PY experiments/s1_sim_map_fix.py dustin --seeds 0 1 2 3 4 --camera ideal
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from functools import lru_cache
from multiprocessing import get_context
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(__file__))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import x_tuniu_geo as G  # noqa: E402
import x_tuniu_match as M  # noqa: E402
from x3_tuniu_fix import seed_of  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs/s1_sim"
REC_DEFAULT = ROOT / "recordings/ilhan_wufeng_south_80m"
ROUTE_DEFAULT = ROOT / "sim/scenarios/wufeng_south_80m.json"
MAP_SRC = ROOT / "data/raw/aerial/wufeng_2018-05-03_x4.tif"
MAP = OUT / "map_2018_0.5m.tif"
NAV_CFG = ROOT / "baseline/configs/sim_navigator.yaml"
# learned-matcher weights live in the main checkout (this worktree has no data/raw/models)
for _k, _sub in (("TORCH_HOME", "torch_hub"), ("HF_HOME", "hf")):
    for _base in (ROOT / "data/raw/models", ROOT.parent / "TaipeiDrift/data/raw/models"):
        if (_base / _sub).is_dir():
            os.environ.setdefault(_k, str(_base / _sub))
            break

RES = 0.5                       # m/px, fixes and odometry
AGREE_M = 4.0                   # ZNCC / XFeat agreement (pre-registered consensus rule)
WIN_MIN_M, WIN_MAX_M, WIN_SIGMA = 45.0, 120.0, 3.0
GATE_FIX = 9.2103               # chi-square, 2 dof, 99 %
PEAK_MIN = 0.5                  # odometry ZNCC peak below which the pair is not trusted
V_MAX = 16.0                    # m/s, 2x the route speed: faster odometry is implausible
JAM_AFTER_M = 450.0             # GNSS until this true distance flown (Dustin's convention)
START_FRAC, STOP_SPEED = 0.95, 2.0   # flight start / end, as baseline/src/data/sim_replay.py
FIX_EVERY = 5                   # recorded images between fix attempts (5 Hz -> 1 Hz)
ODO_WINDOW_S = 20.0             # odometry noise matched to the drift over this many seconds pre-cut
SIGMA_FLOOR_ODO, SIGMA_FLOOR_FIX = 0.02, 1.0
P0_POS_SD = 2.0                 # m, state at the cut (last GNSS fix)
GNSS_SD_M = 1.5                 # m per axis: sim/models/midair_quad/model.sdf NavSat 1.4e-5 deg
B0_SD_DEG, B_RW_DEG_SQRT_S = 2.0, 0.1   # heading-error prior of the filter (= simulated drift model)
K0_HEIGHT_SD_M = 1.5           # scale state prior: baro height error at the cut (offset zeroed pre-cut), m
RAMP_HORIZON_S = 600.0         # baro ramp folded into the scale random walk over this flight time
WRONG_M = 10.0
METHODS = ("zncc", "xfeat")
R_BODY_CV = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]]).T   # rows: cv x,y,z in body FLU -> columns
# OpenCV camera axes in body FLU (meta.json T_body_cam; camera_model.py: image top = forward, right = body right):
# x_cv = body right = (0,-1,0), y_cv = body back = (-1,0,0), z_cv = down = (0,0,-1)


def wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


# ----------------------------------------------------------------------------- recording

@dataclass
class Recording:
    path: Path
    paths: list            # image paths (cam0), index = image_index
    t: np.ndarray          # image times (s)
    q: np.ndarray          # (N, 4) truth quaternion at image times: roll/pitch for the AHRS stand-in, sim heading
    e: np.ndarray          # truth ENU at image times: EVALUATOR + simulated sensors only
    n: np.ndarray
    u: np.ndarray
    first: int             # flight start / end image index (inclusive)
    last: int
    cut: int               # first image with true distance flown >= JAM_AFTER_M
    travelled: np.ndarray  # true distance flown since `first` (N,), nan before first
    origin: tuple          # route origin EPSG:3826 (east, north)
    gnss_t: np.ndarray     # pre-cut GNSS fixes (ENU m), recorded or SIMULATED
    gnss_e: np.ndarray
    gnss_n: np.ndarray
    gnss_source: str
    cam: dict


def r_enu_body(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def heading_of(R_eb: np.ndarray) -> float:
    """Bearing (deg, clockwise from north) of the body forward axis."""
    return float(np.degrees(math.atan2(R_eb[0, 0], R_eb[1, 0])) % 360.0)


def rz_up(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    from pyproj import Transformer
    ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
    x, y, z = ecef.transform(np.asarray(lon), np.asarray(lat), np.asarray(alt))
    x0, y0, z0 = ecef.transform(lon0, lat0, alt0)
    la, lo = math.radians(lat0), math.radians(lon0)
    d = np.stack([x - x0, y - y0, z - z0])
    E = -math.sin(lo) * d[0] + math.cos(lo) * d[1]
    N = -math.sin(la) * math.cos(lo) * d[0] - math.sin(la) * math.sin(lo) * d[1] + math.cos(la) * d[2]
    return E, N


@lru_cache(maxsize=4)
def load_recording(rec: str = str(REC_DEFAULT), route: str = str(ROUTE_DEFAULT)) -> Recording:
    rec_p = Path(rec)
    meta = json.loads((rec_p / "meta.json").read_text())
    rt = json.loads(Path(route).read_text())
    truth = pd.read_csv(rec_p / "truth.csv")
    img = pd.read_csv(rec_p / "images.csv")
    img = img[img.cam == "cam0"].reset_index(drop=True)
    t = img.t_s.to_numpy(float)
    tt = truth.t_s.to_numpy(float)
    e, n, u = (np.interp(t, tt, truth[c].to_numpy()) for c in ("e_m", "n_m", "u_m"))
    near = np.clip(np.searchsorted(tt, t), 1, len(tt) - 1)
    near -= (t - tt[near - 1]) < (tt[near] - t)
    q = truth[["qw", "qx", "qy", "qz"]].to_numpy()[near]
    speed = np.r_[0.0, np.hypot(np.diff(e), np.diff(n)) / np.maximum(np.diff(t), 1e-6)]
    first = int(np.nonzero(u >= START_FRAC * rt["altitude_m"])[0][0])
    moving = np.nonzero(speed[first:] >= STOP_SPEED)[0]
    last = first + int(moving[-1])
    trav = np.full(len(t), np.nan)
    trav[first:] = np.r_[0.0, np.cumsum(np.hypot(np.diff(e[first:]), np.diff(n[first:])))]
    cut = int(first + np.searchsorted(trav[first:last + 1], JAM_AFTER_M))
    # pre-cut GNSS: recorded gnss.csv if it covers the pre-cut flight, else SIMULATED (truth + NavSat SDF noise)
    o = meta["origin"]
    g = pd.read_csv(rec_p / "gnss.csv")
    if len(g) and g.t_s.max() >= t[cut]:
        ge, gn = geodetic_to_enu(g.lat_deg.to_numpy(), g.lon_deg.to_numpy(), g.alt_m.to_numpy(),
                                 o["lat_deg"], o["lon_deg"], o["alt_m"])
        gt, src = g.t_s.to_numpy(float), "recorded gnss.csv"
    else:
        gt = np.arange(math.ceil(tt[0]), t[cut] + 1e-9, 1.0)
        rng = np.random.default_rng(seed_of("s1-gnss"))
        ge = np.interp(gt, tt, truth.e_m) + rng.normal(0, GNSS_SD_M, len(gt))
        gn = np.interp(gt, tt, truth.n_m) + rng.normal(0, GNSS_SD_M, len(gt))
        src = (f"SIMULATED 1 Hz, truth + N(0, {GNSS_SD_M} m) per axis (gnss.csv stops at t = "
               f"{g.t_s.max() if len(g) else float('nan'):.1f} s)")
    keep = gt <= t[cut]
    K = meta["sensors"]["camera"]["cams"]["cam0"]["K"]
    cam = dict(image_width_px=meta["sensors"]["camera"]["cams"]["cam0"]["width"],
               image_height_px=meta["sensors"]["camera"]["cams"]["cam0"]["height"],
               fx_px=K[0], fy_px=K[4], cx_px=K[2], cy_px=K[5], distortion_k1_k2_p1_p2_k3=[0, 0, 0, 0, 0])
    return Recording(rec_p, [str(rec_p / p) for p in img.path], t, q, e, n, u, first, last, cut, trav,
                     (rt["origin_easting_m"], rt["origin_northing_m"]), gt[keep], np.asarray(ge)[keep],
                     np.asarray(gn)[keep], src, cam)


def gnss_at(rec: Recording, t) -> np.ndarray:
    return np.stack([np.interp(t, rec.gnss_t, rec.gnss_e), np.interp(t, rec.gnss_t, rec.gnss_n)], -1)


# ----------------------------------------------------------------------------- query (rectified patch)

_W: dict = {}


def camera(rec: Recording) -> G.Camera:
    key = ("cam", str(rec.path))
    if key not in _W:
        _W[key] = G.Camera(rec.cam)
    return _W[key]


def realistic_camera(rec: Recording):
    """Dustin's realistic camera (baseline/src/data/camera_model.py) on the same flight slice as his loader."""
    key = ("real", str(rec.path))
    if key not in _W:
        import yaml
        sys.path.insert(0, str(ROOT / "baseline"))
        from src.data.camera_model import CameraModel, RealisticCamera
        from src.data.sim_replay import heading_from_quaternion
        cfg = yaml.safe_load(NAV_CFG.read_text())
        k = slice(rec.first, rec.last + 1)
        ne = np.column_stack([rec.n[k], rec.e[k]]) - np.array([rec.n[rec.first], rec.e[rec.first]])
        _W[key] = RealisticCamera(CameraModel(**cfg["cameras"]["realistic"]), rec.t[k] - rec.t[rec.first], ne,
                                  heading_from_quaternion(*rec.q[k].T), rec.u[k])
    return _W[key]


def load_image(rec: Recording, i: int, cam_kind: str) -> np.ndarray:
    img = cv2.imread(rec.paths[i], cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(rec.paths[i])
    if cam_kind == "realistic":
        img = realistic_camera(rec).frame(i - rec.first, img)
    return img


def attitude(rec: Recording, i: int, heading_deg: float, R_body_to_enu=None) -> np.ndarray:
    """Camera(OpenCV) -> ENU: truth roll/pitch (AHRS stand-in) with the estimator's heading.
    R_body_to_enu (optional, 3x3 body FLU -> ENU): an estimator's full attitude, used as is (no truth)."""
    if R_body_to_enu is not None:
        return np.asarray(R_body_to_enu, float) @ R_BODY_CV
    R_true = r_enu_body(rec.q[i])
    R_used = rz_up(-float(wrap180(heading_deg - heading_of(R_true)))) @ R_true
    return R_used @ R_BODY_CV


def make_query(rec: Recording, i: int, heading_deg: float, height_m: float, cam_kind: str,
               R_body_to_enu=None) -> dict:
    R = attitude(rec, i, heading_deg, R_body_to_enu)
    q = G.rectify(load_image(rec, i, cam_kind), 1, camera(rec), R, height_m, RES, far_m=1e3)
    q["height_used"] = height_m
    q["fwd"] = np.array([math.sin(math.radians(heading_deg)), math.cos(math.radians(heading_deg))])
    return q


# ----------------------------------------------------------------------------- odometry (copied from x4)

def _subpix(s, loc):
    x, y = int(loc[0]), int(loc[1])
    dx = dy = 0.0
    if 0 < x < s.shape[1] - 1:
        a, b, c = s[y, x - 1], s[y, x], s[y, x + 1]
        den = a - 2 * b + c
        dx = 0.5 * (a - c) / den if abs(den) > 1e-9 else 0.0
    if 0 < y < s.shape[0] - 1:
        a, b, c = s[y - 1, x], s[y, x], s[y + 1, x]
        den = a - 2 * b + c
        dy = 0.5 * (a - c) / den if abs(den) > 1e-9 else 0.0
    return np.array([x + np.clip(dx, -1, 1), y + np.clip(dy, -1, 1)])


ODO_MARGIN_M = 10.0             # template kept this far inside the ground both patches see (prediction error)


def displacement(qa: dict, qb: dict, D_pred=None):
    """Nadir displacement (E, N) metres from patch a (earlier) to patch b (later); x4_tuniu_speed.displacement,
    except the template: with a predicted displacement D_pred, it is the largest rectangle of b inside the
    ground both patches see (a shifted by D_pred), ODO_MARGIN_M inside its edge. x4's rule (b minus its far 30 %)
    fails in the simulator's turns: 20 deg of heading change and 15 deg of tilt per second move the footprints
    apart, so the true match position no longer holds the template inside a's valid ground."""
    res = qa["res"]
    vb = qb["valid"].copy()
    rows, cols = vb.shape
    nb = qb["nadir_px"]
    if D_pred is None:
        yy, xx = np.mgrid[0:rows, 0:cols]
        fwd = qb["fwd"]
        along = (xx - nb[0]) * fwd[0] - (yy - nb[1]) * fwd[1]
        vb &= along * res <= 0.7 * along[qb["valid"]].max() * res
    else:
        tx = (qb["x0"] + D_pred[0] - qa["x0"]) / res
        ty = (qa["y0"] - qb["y0"] - D_pred[1]) / res
        A = np.array([[1.0, 0.0, tx], [0.0, 1.0, ty]])
        va = cv2.warpAffine(qa["valid"].astype(np.uint8), A, (cols, rows), flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP)
        k = 2 * int(round(ODO_MARGIN_M / res)) + 1
        vb &= cv2.erode((va > 0).astype(np.uint8) & vb.astype(np.uint8), np.ones((k, k), np.uint8)) > 0
    r0, c0, r1, c1 = M.template_rect(vb, nb, (0.0,), (1.0,))
    if r1 - r0 < 16 or c1 - c0 < 16:
        return None, np.nan
    t = qb["patch"][r0:r1, c0:c1]
    ok = M._valid_scores(qa["valid"], r1 - r0, c1 - c0)
    if not ok.any() or t.std() < 2:
        return None, np.nan
    s, peak, loc = M._match(qa["patch"], t, ok)
    loc = _subpix(s, loc)
    o_a = np.array([qa["x0"] + loc[0] * res, qa["y0"] - loc[1] * res])
    o_b = np.array([qb["x0"] + c0 * res, qb["y0"] - r0 * res])
    return o_a - o_b, peak


def odo_status(D_m, peak, dt) -> str:
    if D_m is None:
        return "nomatch"
    if np.hypot(*D_m) / dt > V_MAX:
        return "implausible"
    if not peak >= PEAK_MIN:
        return "lowpeak"
    return "ok"


def fallback(D_last, dt_last: float, dt: float, heading_deg: float) -> np.ndarray:
    """Last measured speed along the current (estimator) heading, in the odometry frame."""
    yaw = math.radians(heading_deg)
    return float(np.hypot(*D_last)) * dt / dt_last * np.array([math.sin(yaw), math.cos(yaw)])


# ----------------------------------------------------------------------------- filter (copied from x5)

def rot(b):
    """Rotation that undoes a heading error b (rad): measured vectors are turned clockwise by b."""
    c, s = math.cos(b), math.sin(b)
    return np.array([[c, -s], [s, c]])


class Filter:
    """EKF on (east, north, heading error b[, scale error k]). Odometry D_m is measured in a frame turned by b
    and scaled by 1/(1 + k). `scale=False` is the x5 filter. The scale state (added after the x5 filter lost lock
    on seed 2, see RESULTS.md) carries the barometer's height error: the rectified patches are scaled by
    true height / baro height, so odometry is too short or too long by the same ratio until fixes estimate it.
    Its prior and random walk come from the SIMULATED barometer's specification (x_tuniu_geo), not from truth."""

    def __init__(self, xy, pos_sd=P0_POS_SD, b_sd_deg=B0_SD_DEG, b_rw=B_RW_DEG_SQRT_S, scale=False,
                 height_m=80.0):
        self.n = 4 if scale else 3
        self.x = np.zeros(self.n)
        self.x[:2] = xy
        p0 = [pos_sd ** 2, pos_sd ** 2, math.radians(b_sd_deg) ** 2]
        q = [math.radians(b_rw) ** 2]
        if scale:
            p0.append((K0_HEIGHT_SD_M / height_m) ** 2)
            q.append((G.BARO_RW_M_SQRT_S ** 2 + G.BARO_RAMP_SD_M_S ** 2 * RAMP_HORIZON_S) / height_m ** 2)
        self.P = np.diag(p0)
        self.q = np.array(q)

    def predict(self, Dm, sigma, dt):
        b = self.x[2]
        k = 1.0 + self.x[3] if self.n == 4 else 1.0
        c, s = math.cos(b), math.sin(b)
        RD = rot(b) @ Dm
        self.x[:2] += k * RD
        F = np.eye(self.n)
        F[:2, 2] = k * (np.array([[-s, -c], [c, -s]]) @ Dm)
        if self.n == 4:
            F[:2, 3] = RD
        self.P = F @ self.P @ F.T + np.diag(np.r_[sigma ** 2, sigma ** 2, self.q * dt])

    def half_window(self) -> float:
        lam = float(np.linalg.eigvalsh(self.P[:2, :2]).max())
        return min(WIN_MAX_M, max(WIN_MIN_M, WIN_SIGMA * math.sqrt(lam)))

    def update(self, z, sigma) -> tuple[float, bool]:
        Hm = np.zeros((2, self.n))
        Hm[0, 0] = Hm[1, 1] = 1.0
        R = np.eye(2) * sigma ** 2
        y = np.asarray(z, float) - self.x[:2]
        S = self.P[:2, :2] + R
        nis = float(y @ np.linalg.solve(S, y))
        if nis > GATE_FIX:
            return nis, False
        K = self.P @ Hm.T @ np.linalg.inv(S)
        self.x += K @ y
        IKH = np.eye(self.n) - K @ Hm
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        return nis, True


# ----------------------------------------------------------------------------- fixes

def load_map(path: Path = MAP) -> G.MapRaster:
    key = ("map", str(path))
    if key not in _W:
        if not Path(path).exists():
            raise SystemExit(f"{path} missing: run `prepare-map` first")
        _W[key] = G.MapRaster(Path(path))
    return _W[key]


def consensus_fix(m: G.MapRaster, q: dict, centre_map_xy, half: float) -> dict:
    """Pre-registered agreement rule (x5): both fixes exist and agree within 4 m -> ZNCC fix (MAP coordinates)."""
    ref = M.reference_for(m, q, centre_map_xy, half)
    out = dict(ref_valid=float(ref["valid"].mean()))
    fx = {}
    for meth in METHODS:
        r = M.run_method(meth, q, ref)
        fx[meth] = r["fix"]
        out[f"{meth}_s"] = r["latency_s"]
        out[f"{meth}_mx"] = np.nan if r["fix"] is None else r["fix"][0]
        out[f"{meth}_my"] = np.nan if r["fix"] is None else r["fix"][1]
        if meth == "zncc":
            out["zncc_score"] = r.get("score", np.nan)
        else:
            out["xfeat_inliers"] = r.get("inliers", 0)
    if fx["zncc"] is None or fx["xfeat"] is None:
        out["fix_status"] = "nofix"
    elif float(np.hypot(*(np.asarray(fx["zncc"]) - np.asarray(fx["xfeat"])))) > AGREE_M:
        out["fix_status"] = "disagree"
    else:
        out["fix_status"] = "agree"
    return out


def calib_path(cam_kind: str = "ideal") -> Path:
    return OUT / ("calibration.json" if cam_kind == "ideal" else f"calibration_{cam_kind}.json")


def calibration(cam_kind: str = "ideal") -> dict:
    p = calib_path(cam_kind)
    return json.loads(p.read_text()) if p.exists() else {}


def fix_at(image_index: int, prior_en, sigma_m: float, heading_deg: float, height_m: float,
           recording: str = str(REC_DEFAULT), route: str = str(ROUTE_DEFAULT), cam_kind: str = "ideal",
           R_body_to_enu=None):
    """Camera-to-map fix for one recorded image (cam0 row of images.csv), for an external filter.

    prior_en: filter estimate, ENU metres from the route origin (= the recording's truth frame).
    sigma_m: 1-sigma horizontal uncertainty of the prior; search half-size = clip(3 sigma, 45 m, 120 m).
    heading_deg: the filter's heading (bearing, clockwise from north); roll/pitch come from the truth
    quaternion (AHRS stand-in). height_m: height above the take-off ground (flat world).
    Returns (e, n, cov_2x2, diagnostics) if ZNCC and XFeat agree within 4 m (position = ZNCC fix, map offset
    calibrated pre-cut removed, cov = fix_sd^2 I from outputs/s1_sim/calibration[_<camera>].json), else None.
    No innovation gate here: the caller gates.
    """
    rec = load_recording(str(recording), str(route))
    cal = calibration(cam_kind)
    off = np.array(cal.get("map_offset_m", (0.0, 0.0)))
    sd = float(cal.get("fix_sd_m", 3.0))
    half = min(WIN_MAX_M, max(WIN_MIN_M, WIN_SIGMA * float(sigma_m)))
    q = make_query(rec, int(image_index), float(heading_deg), float(height_m), cam_kind, R_body_to_enu)
    centre = np.asarray(prior_en, float) + np.array(rec.origin) + off
    t0 = time.perf_counter()
    fx = consensus_fix(load_map(), q, centre, half)
    fx.update(half_m=half, latency_s=time.perf_counter() - t0)
    if fx["fix_status"] != "agree":
        return None
    e = fx["zncc_mx"] - rec.origin[0] - off[0]
    n = fx["zncc_my"] - rec.origin[1] - off[1]
    return float(e), float(n), np.eye(2) * sd ** 2, fx


# ----------------------------------------------------------------------------- simulated sensors (parent)

def simulated_heading_error(t_s: np.ndarray, seed: int) -> np.ndarray:
    """SIMULATED heading error (deg), x5 `drift`: per-seed constant N(0, 2 deg) + random walk 0.1 deg/sqrt(s)."""
    rng = np.random.default_rng(seed_of("headdrift", seed))
    dt = np.diff(np.asarray(t_s, float) - t_s[0], prepend=0.0)
    return rng.normal(0, B0_SD_DEG) + np.cumsum(rng.normal(0, 1, len(t_s)) * B_RW_DEG_SQRT_S * np.sqrt(dt))


def simulated_baro(rec: Recording, seed: int) -> np.ndarray:
    """SIMULATED barometer at every image time: height above the take-off ground (x_tuniu_geo model)."""
    return G.simulated_baro(rec.t, rec.u, seed_of("baro", seed), rec.t[rec.cut])


def true_headings(rec: Recording, idx) -> np.ndarray:
    return np.array([heading_of(r_enu_body(rec.q[i])) for i in idx])


# ----------------------------------------------------------------------------- closed loop

def start_row(u: dict) -> dict:
    """The filter state at its first image (last pre-cut GNSS fix), as a run row."""
    e, n = u["xy0"]
    return dict(image=u["idx"][0], t_s=float(u["t"][0]), dt=float(u["dt0"]), odo_status="start", pred_e=e, pred_n=n,
                pred_sd_e=P0_POS_SD, pred_sd_n=P0_POS_SD, half_m=WIN_MIN_M, height_used=u["baro"][0],
                heading_used=u["heading"][0], dr_e=e, dr_n=n, dr_sd=P0_POS_SD, fix_try=False, post_e=e, post_n=n,
                post_sd_e=P0_POS_SD, post_sd_n=P0_POS_SD, odo_e=float(u["D0"][0]), odo_n=float(u["D0"][1]))


def run_sequence(u: dict) -> list[dict]:
    """Estimator side. Inputs: images, times, heading (truth + SIMULATED drift), SIMULATED baro, the state at
    the last pre-cut GNSS fix and the pre-cut calibration. No truth position."""
    rec = load_recording(u["rec"], u["route"])
    c = u["calib"]
    m = load_map()
    off = np.array(c["map_offset_m"])
    org = np.array(rec.origin)
    idx, t, hd, baro = u["idx"], np.asarray(u["t"]), np.asarray(u["heading"]), np.asarray(u["baro"])
    loop = Filter(u["xy0"], scale=u["filter"] == "scale", height_m=baro[0])
    dr = Filter(u["xy0"])
    q_prev = make_query(rec, idx[0], hd[0], baro[0], u["camera"])
    D_last, dt_last = np.asarray(u["D0"], float), float(u["dt0"])
    rows = [start_row(u)]
    fix_m, since_try = u.get("fix_every_m"), 0.0
    for j in range(1, len(idx)):
        t0 = time.perf_counter()
        i = idx[j]
        dt = float(t[j] - t[j - 1])
        q = make_query(rec, i, hd[j], baro[j], u["camera"])
        D_fb = fallback(D_last, dt_last, dt, hd[j])
        D_m, peak = displacement(q_prev, q, D_fb)
        status = odo_status(D_m, peak, dt)
        if status == "ok":
            D_use, s_use = D_m, c["odo_sd_m"]
            D_last, dt_last = D_m, dt
        else:
            D_use, s_use = D_fb, c["fb_sd_m"]
        before = loop.x[:2].copy()
        loop.predict(D_use, s_use, dt)
        since_try += float(np.hypot(*(loop.x[:2] - before)))      # ESTIMATED distance flown since last attempt
        dr.predict(D_use, s_use, dt)
        pred, Ppred, half = loop.x[:2].copy(), loop.P.copy(), loop.half_window()
        row = dict(image=i, t_s=float(t[j]), dt=dt, odo_status=status, odo_peak=peak,
                   odo_e=np.nan if D_m is None else D_m[0], odo_n=np.nan if D_m is None else D_m[1],
                   pred_e=pred[0], pred_n=pred[1], pred_sd_e=math.sqrt(Ppred[0, 0]),
                   pred_sd_n=math.sqrt(Ppred[1, 1]), half_m=half, height_used=baro[j], heading_used=hd[j],
                   dr_e=dr.x[0], dr_n=dr.x[1], dr_sd=math.sqrt(max(dr.P[0, 0], dr.P[1, 1])), fix_try=False)
        if u["mode"] == "loop" and (i - idx[0]) % FIX_EVERY == 0 and (fix_m is None or since_try >= fix_m):
            since_try = 0.0
            fx = consensus_fix(m, q, pred + org + off, half)
            row.update(fx, fix_try=True)
            if fx["fix_status"] == "agree":
                z = np.array([fx["zncc_mx"], fx["zncc_my"]]) - org - off
                nis, ok = loop.update(z, c["fix_sd_m"])
                row.update(z_e=z[0], z_n=z[1], nis=nis, fix_status="accepted" if ok else "gated")
        row.update(post_e=loop.x[0], post_n=loop.x[1], post_sd_e=math.sqrt(loop.P[0, 0]),
                   post_sd_n=math.sqrt(loop.P[1, 1]), b_hat_deg=math.degrees(loop.x[2]),
                   b_sd_deg=math.degrees(math.sqrt(loop.P[2, 2])),
                   k_hat=loop.x[3] if loop.n == 4 else np.nan, step_s=time.perf_counter() - t0)
        rows.append(row)
        q_prev = q
    return rows


def unit_for(rec: Recording, seed: int, cam_kind: str, mode: str, calib: dict, rec_s: str, route_s: str,
             filt: str = "scale") -> dict:
    """Parent: estimator inputs. Truth is used only for the SIMULATED heading/baro and for the flight slice."""
    step = int(calib["odo_step"])
    # start at the image nearest the last pre-cut GNSS fix, on the processing grid that ends on the fix images
    k0 = int(np.argmin(np.abs(rec.t[:rec.cut + 1] - rec.gnss_t[-1])))
    idx = list(range(k0, rec.last + 1, step))
    if mode == "loop":
        assert step <= FIX_EVERY and FIX_EVERY % step == 0
    t = rec.t[idx]
    err = simulated_heading_error(t, seed)
    heading = (true_headings(rec, idx) + err) % 360.0
    baro = simulated_baro(rec, seed)[idx]
    xy0 = (float(rec.gnss_e[-1]), float(rec.gnss_n[-1]))
    g = gnss_at(rec, [rec.t[k0 - step], rec.t[k0]])
    return dict(seed=seed, camera=cam_kind, mode=mode, filter=filt, rec=rec_s, route=route_s, calib=calib, idx=idx,
                t=t.tolist(), heading=heading.tolist(), heading_err=err.tolist(), baro=baro.tolist(), xy0=xy0,
                D0=tuple((g[1] - g[0]).tolist()), dt0=float(rec.t[k0] - rec.t[k0 - step]))


def score(rows: list[dict], u: dict, rec: Recording) -> pd.DataFrame:
    """Evaluator: errors against truth (parent only)."""
    df = pd.DataFrame(rows)
    i = df.image.to_numpy()
    df["true_e"], df["true_n"] = rec.e[i], rec.n[i]
    df["dist_m"] = rec.travelled[i]
    df["dist_since_cut_m"] = rec.travelled[i] - rec.travelled[rec.cut]
    df["after_cut"] = i >= rec.cut
    df["err_m"] = np.hypot(df.post_e - df.true_e, df.post_n - df.true_n)
    df["err_pred_m"] = np.hypot(df.pred_e - df.true_e, df.pred_n - df.true_n)
    df["err_dr_m"] = np.hypot(df.dr_e - df.true_e, df.dr_n - df.true_n)
    df["lol"] = df.err_pred_m > df.half_m
    if "z_e" in df:
        df["fix_err_m"] = np.hypot(df.z_e - df.true_e, df.z_n - df.true_n)
    j = {k: n for n, k in enumerate(u["idx"])}
    df["heading_err_sim_deg"] = [u["heading_err"][j[k]] for k in i]
    df["seed"], df["camera"], df["mode"], df["filter"] = u["seed"], u["camera"], u["mode"], u["filter"]
    return df


def run_path(cam_kind: str, mode: str, seed: int, filt: str = "scale", fix_m: float | None = None) -> Path:
    """Final runs (EKF with scale state) in runs/; the x5 3-state EKF in runs_x5filter/; fix-spacing stress
    runs (--fix-every-m) in stress/<camera>/<mode>_s<seed>_fix<m>m.csv."""
    if fix_m is not None:
        return OUT / "stress" / cam_kind / f"{mode}_s{seed}_fix{fix_m:g}m.csv"
    return OUT / ("runs" if filt == "scale" else f"runs_{filt}filter") / cam_kind / f"{mode}_s{seed}.csv"


def _worker_init():
    cv2.setNumThreads(1)
    try:
        import torch
        torch.set_num_threads(1)
    except ImportError:
        pass


def cmd_run(args):
    if not MAP.exists():
        cmd_prepare_map(argparse.Namespace(src=str(MAP_SRC), out=str(MAP)))
    cal_p = calib_path(args.camera)
    if not cal_p.exists() or json.loads(cal_p.read_text()).get("recording") != str(Path(args.recording)):
        print(f"no pre-cut calibration for this recording in {cal_p}: calibrating first", flush=True)
        cmd_calibrate(argparse.Namespace(recording=args.recording, route=args.route, camera=args.camera,
                                         odo_steps=[1, 5]))
    calib = json.loads(cal_p.read_text())
    rec = load_recording(args.recording, args.route)
    fm = args.fix_every_m
    units = [dict(unit_for(rec, s, args.camera, args.mode, calib, args.recording, args.route, args.filter),
                  fix_every_m=fm)
             for s in args.seeds if not run_path(args.camera, args.mode, s, args.filter, fm).exists()]
    run_path(args.camera, args.mode, 0, args.filter, fm).parent.mkdir(parents=True, exist_ok=True)
    print(f"{len(units)} runs ({args.mode}, camera {args.camera}), {len(units[0]['idx']) if units else 0} images "
          f"each, workers {args.workers}; pre-cut GNSS: {rec.gnss_source}", flush=True)
    t0 = time.time()
    if units:
        with get_context("spawn").Pool(min(args.workers, len(units)), initializer=_worker_init) as pool:
            for u, rows in zip(units, pool.imap(run_sequence, units, chunksize=1)):
                df = score(rows, u, rec)
                df.to_csv(run_path(u["camera"], u["mode"], u["seed"], u["filter"], fm), index=False)
                a = df[df.after_cut]
                acc = int((a.get("fix_status") == "accepted").sum()) if "fix_status" in a else 0
                print(f"  seed {u['seed']}: loop median {a.err_m.median():.1f} m (max {a.err_m.max():.1f}), DR "
                      f"median {a.err_dr_m.median():.1f} m (final {a.err_dr_m.iloc[-1]:.1f}), accepted {acc}, "
                      f"{time.time() - t0:.0f} s", flush=True)
    if args.mode == "loop" and fm is None:
        score_with_dustin(args, rec)


# ----------------------------------------------------------------------------- Dustin's format and scorer

def _baseline_imports():
    for p in (ROOT / "baseline", ROOT / "baseline/scripts"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))


def dustin_flight(rec_s: str, route_s: str):
    """Dustin's CameraFlight of the recording (baseline/src/data/sim_replay.py), for his scorer only."""
    _baseline_imports()
    from src.data.sim_replay import SimReplayConfig, load_sim_flight
    return load_sim_flight(SimReplayConfig(recording=rec_s, map_tif=str(MAP_SRC), route=route_s,
                                           cache_dir=str(ROOT / "data/processed")))


def as_navigator_result(df: pd.DataFrame, rec: Recording, flight, start: dict | None = None):
    """Our run as Dustin's NavigatorResult: one row per flight frame from his jam frame to the end.
    We estimate every 5th image; frames in between = the last estimate + its last motion step, scaled by the
    time elapsed (causal; from the start row, the last pre-cut GNSS motion). sigma = sqrt(sd_e^2 + sd_n^2).
    Fixes = the agreeing ones (accepted = used). `start` (start_row) is prepended if the run lacks it."""
    _baseline_imports()
    from src.estimation.camera_navigator import FixRecord, NavigatorConfig, NavigatorResult, jam_index
    if start is not None and df.image.iloc[0] != start["image"]:
        df = pd.concat([pd.DataFrame([start]), df], ignore_index=True)
    jam = jam_index(flight, JAM_AFTER_M)
    if len(flight) != rec.last - rec.first + 1 or rec.first + jam != rec.cut:
        raise SystemExit("flight slice differs from Dustin's loader: cannot score with his tools")
    img = rec.first + np.arange(jam, len(flight))
    proc = df.image.to_numpy()
    j = np.searchsorted(proc, img, side="right") - 1
    if (j < 0).any():
        raise SystemExit("first estimate is after Dustin's jam frame")
    post = df[["post_e", "post_n"]].to_numpy()
    step = df[["pred_e", "pred_n"]].to_numpy() - np.vstack([post[:1], post[:-1]])
    if df.odo_status.iloc[0] != "start":
        raise SystemExit("run has no start row: pass start=start_row(...)")
    step[0] = df[["odo_e", "odo_n"]].to_numpy()[0]       # start row: last pre-cut GNSS motion over dt0
    frac = (rec.t[img] - df.t_s.to_numpy()[j]) / df.dt.to_numpy()[j]
    est = post[j] + frac[:, None] * step[j]
    o = np.array([rec.e[rec.first], rec.n[rec.first]])
    sigma = np.hypot(df.post_sd_e.to_numpy(), df.post_sd_n.to_numpy())[j]
    fixes = []
    if "fix_status" in df:
        for r in df[df.fix_status.isin(["accepted", "gated"]) & (df.image >= rec.cut)].itertuples():
            fixes.append(FixRecord(frame=int(r.image - rec.first), score=float(r.zncc_score),
                                   position=np.array([r.z_n - o[1], r.z_e - o[0]]), zoom=1.0, angle=0.0,
                                   reference_index=-1, candidates=1,
                                   distance=float(np.hypot(r.z_e - r.pred_e, r.z_n - r.pred_n)), allowed=float("nan"),
                                   used=r.fix_status == "accepted", reason="OK" if r.fix_status == "accepted" else "GATED",
                                   frames_agreeing=2, search_radius_m=float(r.half_m)))
    return NavigatorResult(start_index=int(jam), position=np.column_stack([est[:, 1] - o[1], est[:, 0] - o[0]]),
                           sigma=sigma, status=["TRACKING"] * len(img), fixes=fixes, calibration=None,
                           config=NavigatorConfig(jam_after_m=JAM_AFTER_M))


def frames_table(res, flight, rec: Recording) -> pd.DataFrame:
    """Per-frame table, same columns for our runs and Dustin's (`dustin` command)."""
    from src.evaluation.navigation_metrics import navigation_errors
    er = navigation_errors(res, flight)
    k0 = res.start_index
    used = {int(f.frame) for f in res.fixes if f.used}
    return pd.DataFrame(dict(image=rec.first + k0 + np.arange(len(res.position)), dist_since_cut_m=er.distance_since_jam,
                             err_m=er.error, sigma_m=er.sigma, est_n=res.position[:, 0] + rec.n[rec.first],
                             est_e=res.position[:, 1] + rec.e[rec.first],
                             fix_used=[int(k0 + j) in used for j in range(len(res.position))]))


def score_with_dustin(args, rec: Recording):
    """Writes frames_ours_<camera>_s<seed>.csv (Dustin's per-frame columns) and scores_<camera>.csv: our scorer
    (1 Hz estimates after the cut) next to Dustin's (summarize_navigation, integrity_summary; every frame)."""
    import yaml
    _baseline_imports()
    from src.evaluation.navigation_metrics import integrity_summary, navigation_errors, summarize_navigation
    alert = float(yaml.safe_load(NAV_CFG.read_text())["alert_limit_m"])
    flight = dustin_flight(args.recording, args.route)
    tag = "" if args.filter == "scale" else f"_{args.filter}"
    calib = json.loads(calib_path(args.camera).read_text())
    rows = []
    for seed in args.seeds:
        p = run_path(args.camera, "loop", seed, args.filter)
        if not p.exists():
            continue
        df = pd.read_csv(p)
        res = as_navigator_result(df, rec, flight, start_row(unit_for(rec, seed, args.camera, "loop", calib,
                                                                      args.recording, args.route, args.filter)))
        frames_table(res, flight, rec).to_csv(OUT / f"frames_ours{tag}_{args.camera}_s{seed}.csv", index=False)
        a = df[df.after_cut]
        acc = a.fix_status == "accepted"
        ours = dict(median_m=a.err_m.median(), p90_m=a.err_m.quantile(0.9), max_m=a.err_m.max(),
                    final_m=a.err_m.iloc[-1], accepted=int(acc.sum()),
                    wrong_accepted_10m=int((acc & (a.fix_err_m > WRONG_M)).sum()), lol_frames=int(a.lol.sum()))
        his = summarize_navigation(res, flight).as_dict()
        integ = integrity_summary(navigation_errors(res, flight), alert).as_dict()
        rows.append(dict(recording=rec.path.name, camera=args.camera, filter=args.filter, seed=seed,
                         **{f"ours_scorer_{k}": v for k, v in ours.items()},
                         **{f"dustin_scorer_{k}": v for k, v in his.items()},
                         **{f"dustin_integrity_{k}": v for k, v in integ.items() if k != "alert_limit_m"}))
    if rows:
        out = pd.DataFrame(rows)
        out.to_csv(OUT / f"scores{tag}_{args.camera}.csv", index=False)
        with pd.option_context("display.width", 250, "display.max_columns", 12, "display.precision", 1):
            print(out[["seed", "ours_scorer_median_m", "dustin_scorer_median", "ours_scorer_max_m", "dustin_scorer_worst",
                       "dustin_scorer_end", "dustin_scorer_fixes_used", "dustin_scorer_used_but_wrong",
                       "dustin_scorer_within_3_sigma", "dustin_integrity_hazardous"]].to_string(index=False))
        print(f"wrote {OUT / f'scores{tag}_{args.camera}.csv'} and frames_ours{tag}_{args.camera}_s*.csv")


# ----------------------------------------------------------------------------- calibration (pre-cut only)

def odo_stream(rec: Recording, idx, heading, baro, cam_kind):
    """Odometry over consecutive processed images, predicted displacement = fallback (as in the loop)."""
    D, peaks, st = [], [], []
    D_last, dt_last = np.zeros(2), 1.0
    qp = make_query(rec, idx[0], heading[0], baro[0], cam_kind)
    for j in range(1, len(idx)):
        dt = rec.t[idx[j]] - rec.t[idx[j - 1]]
        q = make_query(rec, idx[j], heading[j], baro[j], cam_kind)
        d, p = displacement(qp, q, fallback(D_last, dt_last, dt, heading[j]))
        D.append(d)
        peaks.append(p)
        st.append(odo_status(d, p, dt))
        if st[-1] == "ok":
            D_last, dt_last = d, dt
        qp = q
    return D, np.array(peaks), st


def cmd_calibrate(args):
    """Pre-cut images (GNSS available): reference = pre-cut GNSS interpolated to image times, truth heading
    (GNSS-aided before the cut), seed-0 SIMULATED baro. Truth is printed only as an evaluator diagnostic."""
    rec = load_recording(args.recording, args.route)
    baro = simulated_baro(rec, 0)
    t_cut = rec.t[rec.cut]
    print(f"flight images {rec.first}..{rec.last} ({rec.last - rec.first + 1}), cut at image {rec.cut} "
          f"(t = {t_cut:.1f} s), {rec.travelled[rec.last] / 1000:.2f} km flown, "
          f"{(rec.travelled[rec.last] - JAM_AFTER_M) / 1000:.2f} km after the cut; pre-cut GNSS: {rec.gnss_source}")
    out = dict(recording=str(rec.path), first=rec.first, last=rec.last, cut=rec.cut, t_cut_s=float(t_cut),
               gnss_source=rec.gnss_source, steps={})
    for step in args.odo_steps:
        idx = list(range(rec.first, rec.cut + 1, step))
        hd = true_headings(rec, idx)
        D, peaks, st = odo_stream(rec, idx, hd, baro[idx], args.camera)
        ok = np.array([s == "ok" for s in st])
        g = gnss_at(rec, rec.t[idx])
        dt = np.diff(rec.t[idx])
        # the displacement the filter would use: odometry when ok, else the fallback (last ok speed, heading)
        Dm = np.array([d if s == "ok" else (np.nan, np.nan) for d, s in zip(D, st)])
        Du, D_last, dt_last = np.zeros_like(Dm), g[1] - g[0], dt[0]
        for j in range(len(Dm)):
            if ok[j]:
                Du[j], D_last, dt_last = Dm[j], Dm[j], dt[j]
            else:
                Du[j] = fallback(D_last, dt_last, dt[j], hd[j + 1])
        W = max(1, int(round(ODO_WINDOW_S / np.median(dt))))
        res = np.array([Du[a:a + W].sum(0) - (g[a + W] - g[a]) for a in range(0, len(Du) - W + 1)])
        sd = max(SIGMA_FLOOR_ODO, math.sqrt(float(np.mean(res ** 2)) / W)) if len(res) else 1.0
        # fallback (constant velocity along the heading) vs the measured odometry on consecutive ok pairs
        fb = [fallback(Dm[j - 1], dt[j - 1], dt[j], hd[j + 1]) - Dm[j] for j in range(1, len(Dm)) if ok[j] and ok[j - 1]]
        fb_sd = max(sd, math.sqrt(float(np.mean(np.square(fb)))) if fb else 3 * sd)
        tr = np.column_stack([rec.e[idx], rec.n[idx]])
        diag_res = [Du[a:a + W].sum(0) - (tr[a + W] - tr[a]) for a in range(0, len(Du) - W + 1)]
        Dm = Du
        out["steps"][str(step)] = dict(
            pairs=len(D), ok=int(ok.sum()), window_steps=W, odo_sd_m=sd, fb_sd_m=fb_sd,
            window_rms_vs_gnss_m=float(np.sqrt(np.mean(np.sum(res ** 2, 1)))) if len(res) else None,
            peak_p1=float(np.nanpercentile(peaks, 1)), peak_min=float(np.nanmin(peaks)),
            diag_window_rms_vs_truth_m=float(np.sqrt(np.mean(np.sum(np.square(diag_res), 1)))) if diag_res else None,
            diag_total_drift_vs_truth_m=float(np.linalg.norm(np.nansum(Dm, 0) - (tr[-1] - tr[0]))))
        print(f"  odometry step {step}: {out['steps'][str(step)]}", flush=True)
    # chosen step: lowest pre-cut drift (vs GNSS) over the same 20 s windows
    best = min(out["steps"], key=lambda s: out["steps"][s]["window_rms_vs_gnss_m"] or 1e9)
    out.update(odo_step=int(best), odo_sd_m=out["steps"][best]["odo_sd_m"], fb_sd_m=out["steps"][best]["fb_sd_m"])
    # fixes at 1 Hz, window centred on the GNSS position, offset 0
    m = load_map()
    org = np.array(rec.origin)
    rows = []
    for i in range(rec.first, rec.cut + 1, FIX_EVERY):
        hd = heading_of(r_enu_body(rec.q[i]))
        q = make_query(rec, i, hd, baro[i], args.camera)
        g = gnss_at(rec, rec.t[i])
        fx = consensus_fix(m, q, g + org, WIN_MIN_M)
        rows.append(dict(image=i, g_e=g[0], g_n=g[1], true_e=rec.e[i], true_n=rec.n[i], **fx))
    df = pd.DataFrame(rows)
    agree = df[df.fix_status == "agree"]
    d = np.column_stack([agree.zncc_mx - org[0] - agree.g_e, agree.zncc_my - org[1] - agree.g_n])
    off = np.median(d, 0) if len(d) >= 5 else np.zeros(2)
    fix_sd = max(SIGMA_FLOOR_FIX, math.sqrt(float(np.mean((d - off) ** 2)))) if len(d) >= 5 else 5.0
    dt_ = np.column_stack([agree.zncc_mx - org[0] - agree.true_e, agree.zncc_my - org[1] - agree.true_n])
    out.update(map_offset_m=off.tolist(), fix_sd_m=fix_sd, precut_fix_tries=len(df), precut_fix_agree=len(agree),
               precut_status=df.fix_status.value_counts().to_dict(),
               diag_fix_offset_vs_truth_m=np.median(dt_, 0).tolist() if len(dt_) else None,
               diag_fix_rms_vs_truth_after_offset_m=float(np.sqrt(np.mean(np.sum((dt_ - off) ** 2, 1)))) if len(dt_) else None)
    df.to_csv(OUT / f"calibration_fixes_{args.camera}.csv", index=False)
    calib_path(args.camera).write_text(json.dumps(out, indent=2))
    print(json.dumps({k: v for k, v in out.items() if k != "steps"}, indent=2))
    print(f"wrote {calib_path(args.camera)}")


# ----------------------------------------------------------------------------- map

def cmd_prepare_map(args):
    """2018 orthophoto -> EPSG:3826, north-up, exactly RES m/px, RGBA; no-data = Dustin's rule (grey < 8,
    opened 7x7, baseline/src/data/sim_replay.py)."""
    import rasterio
    from rasterio.transform import from_origin
    from rasterio.warp import Resampling, reproject
    with rasterio.open(args.src) as src:
        b = src.bounds
        w, h = int(math.floor((b.right - b.left) / RES)), int(math.floor((b.top - b.bottom) / RES))
        tf = from_origin(b.left, b.top, RES, RES)
        rgb = np.zeros((3, h, w), np.uint8)
        for k in range(3):
            reproject(rasterio.band(src, k + 1), rgb[k], src_transform=src.transform, src_crs=src.crs,
                      dst_transform=tf, dst_crs="EPSG:3826", resampling=Resampling.average)
    gray = cv2.cvtColor(np.ascontiguousarray(rgb.transpose(1, 2, 0)), cv2.COLOR_RGB2GRAY)
    empty = cv2.morphologyEx((gray < 8).astype(np.uint8), cv2.MORPH_OPEN, np.ones((7, 7), np.uint8)).astype(bool)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    prof = dict(driver="GTiff", crs="EPSG:3826", transform=tf, width=w, height=h, count=4, dtype="uint8",
                compress="deflate", tiled=True)
    with rasterio.open(args.out, "w", **prof) as dst:
        dst.write(rgb, [1, 2, 3])
        dst.write(np.where(empty, 0, 255).astype(np.uint8), 4)
    print(f"wrote {args.out} ({w} x {h} px at {RES} m/px, EPSG:3826, {empty.mean():.1%} no-data)")


# ----------------------------------------------------------------------------- Dustin's navigator, per frame

def cmd_dustin(args):
    """Dustin's navigator (frozen config) on this recording, per-frame errors. Same config mutations as
    baseline/scripts/run_sim_navigator.py main(); camera motion cached in data/processed by his script."""
    import yaml
    _baseline_imports()
    import run_sim_navigator as R
    cfg = yaml.safe_load(NAV_CFG.read_text())
    cfg["recording"], cfg["route"] = args.recording, args.route
    cfg["camera"] = cfg.get("cameras", {}).get(args.camera)
    out = OUT / f"dustin_{args.camera}"
    out.mkdir(parents=True, exist_ok=True)
    rec = load_recording(args.recording, args.route)
    for name in ("camera_alone", "map_2018"):
        for seed in args.seeds:
            p = out / f"frames_{name}_s{seed}.csv"
            if p.exists():
                continue
            r = R.one_run((cfg, name, cfg["runs"][name], seed, True))
            res = r.pop("_result")
            ft = frames_table(res, R.flight_for(cfg, "2018"), rec)
            ft.to_csv(p, index=False)
            print(f"  {name} seed {seed}: median {ft.err_m.median():.1f} m, run summary {r}", flush=True)


# ----------------------------------------------------------------------------- main

def _resolve(p: str) -> Path:
    """Absolute path; relative paths are taken from the current folder, else from the repository root."""
    q = Path(p).expanduser()
    return q.resolve() if q.is_absolute() or q.exists() else ROOT / q


def main():
    global OUT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    pm = sub.add_parser("prepare-map")
    pm.add_argument("src", nargs="?", default=str(MAP_SRC))
    pm.add_argument("out", nargs="?", default=str(MAP))
    for name in ("calibrate", "run", "dustin"):
        p = sub.add_parser(name)
        p.add_argument("--recording", default=str(REC_DEFAULT), help="recordings/<name> (taipeidrift-replay/1)")
        p.add_argument("--route", default=str(ROUTE_DEFAULT), help="sim/scenarios/<route>.json of that recording")
        p.add_argument("--out", default=str(OUT), help="output folder (calibration, runs, scores, frames)")
        p.add_argument("--camera", default="ideal", choices=("ideal", "realistic"))
        if name == "calibrate":
            p.add_argument("--odo-steps", type=int, nargs="+", default=[1, 5])
        else:
            p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
        if name == "run":
            p.add_argument("--mode", default="loop", choices=("loop", "dr"))
            p.add_argument("--filter", default="scale", choices=("scale", "x5"),
                           help="scale: EKF (e, n, heading error, scale error); x5: the 3-state x5 EKF")
            p.add_argument("--fix-every-m", type=float, default=None,
                           help="attempt a map fix only after this ESTIMATED distance since the last attempt "
                                "(default: every 5th image, i.e. once per second); odometry unchanged")
            p.add_argument("--workers", type=int, default=3)
    args = ap.parse_args()
    if args.cmd != "prepare-map":
        OUT = _resolve(args.out)
        args.recording, args.route = str(_resolve(args.recording)), str(_resolve(args.route))
    OUT.mkdir(parents=True, exist_ok=True)
    {"prepare-map": cmd_prepare_map, "calibrate": cmd_calibrate, "run": cmd_run, "dustin": cmd_dustin}[args.cmd](args)


if __name__ == "__main__":
    main()

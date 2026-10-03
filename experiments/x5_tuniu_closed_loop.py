#!/usr/bin/env python3
"""Level 2: closed-loop, GNSS-free replay of the Tuniu flight after the cut (photos 47-271).

Pre-registration: docs/research/tuniu-level2-prereg.md. Results: docs/research/tuniu-level2-results.md.

The starting estimate of every photo comes from the navigation filter itself, never from RTK.
Filter (EKF, state = east, north, heading error b):
  predict  p += Rot(-b) D, D = nadir displacement measured between consecutive photos (rectified patches +
           ZNCC, x4_tuniu_speed.displacement); if odometry fails (no match, > 16 m/s, ZNCC peak < 0.5),
           D = last measured speed along the camera heading. Noise model fitted on PRE-CUT photos only.
  update   absolute fix = pre-registered agreement rule (ZNCC and XFeat nadir fixes within 4 m, position =
           ZNCC fix), 0.5 m/px map, search window centred on the FILTER estimate, half-size max(45 m, 3 sigma)
           capped at 120 m; chi-square 99 % innovation gate.
Inputs after the cut: photos, MRK times, DJI gimbal attitude + pre-cut boresight (optionally + a SIMULATED
heading drift), SIMULATED barometer, terrain model, map. RTK is used only for the state at the cut,
pre-cut calibration, the simulated barometer, and scoring, all in the parent process.

  calibrate   pre-cut noise model (odometry, fallback, fix) per ground config -> calibration_<tag>.json
  run         seeds x heading x ground x mode (loop | dr) -> runs/<tag>/<mode>_<heading>_<ground>_s<seed>.csv
  summarize   per-run and pooled metrics + pass criteria -> summary_<tag>_runs.csv, summary_<tag>.json
  figure      error vs time for a few seeds -> fig_error_vs_time_<tag>.png (no photo pixels)
  prepare-map reproject any GeoTIFF (e.g. the OpenDroneMap orthophoto) to EPSG:3826 north-up at --res

Swappable map and terrain (e.g. OpenDroneMap products):
  --map main | <EPSG:3826 GeoTIFF>      --map-offset stage1 | calib | <de,dn>
  --terrain copernicus | <GeoTIFF, any CRS>   --terrain-dz <metres added to the raster heights>

Examples:
  .venv/bin/python experiments/x5_tuniu_closed_loop.py calibrate
  .venv/bin/python experiments/x5_tuniu_closed_loop.py run --mode loop --heading dji --ground dem_lifted --seeds 0 1
  .venv/bin/python experiments/x5_tuniu_closed_loop.py summarize
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from multiprocessing import get_context
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import x_tuniu_geo as G  # noqa: E402
from x3_tuniu_fix import seed_of  # noqa: E402

OUT = G.ROOT / "data/processed/x_tuniu_l2"
RES = 0.5                       # m/px, fixes and odometry
FAR_M = 100.0                   # rectified ground kept <= 100 m ahead of the nadir (step 1)
AGREE_M = 4.0                   # ZNCC / XFeat agreement (pre-registered consensus rule)
WIN_MIN_M, WIN_MAX_M, WIN_SIGMA = 45.0, 120.0, 3.0
GATE_FIX = 9.2103               # chi-square, 2 dof, 99 %
PEAK_MIN = 0.5                  # odometry ZNCC peak; pre-cut: every bad pair <= 0.28, every good pair >= 0.72
ODO_HORIZON = 20                # photos (~56 s): odometry noise is set to match the drift measured over this
                                # many consecutive pre-cut photos (errors are correlated, not white)
V_MAX = 16.0                    # m/s, Phantom 4 RTK maximum speed: faster odometry is implausible
TURN_DEG = 10.0                 # |gimbal yaw change| between consecutive photos above which a pair is a turn
SIGMA_FLOOR_ODO, SIGMA_FLOOR_FIX = 0.5, 1.0
P0_POS_SD = 1.0                 # m, state at the cut (last pre-cut RTK fix)
B0_SD_DEG, B_RW_DEG_SQRT_S = 2.0, 0.1   # heading-error prior of the filter (= simulated drift model)
WRONG_M = 10.0
METHODS = ("zncc", "xfeat")


def wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


# ----------------------------------------------------------------------------- terrain and map

class Terrain:
    """Ellipsoidal ground height at EPSG:3826 points. 'copernicus' = x_tuniu_geo.dem_ellipsoidal; otherwise
    any single-band GeoTIFF (e.g. an OpenDroneMap DSM) in any CRS, bilinear, + dz; no-data -> Copernicus."""

    def __init__(self, spec: str = "copernicus", dz: float = 0.0):
        self.spec, self.dz = spec, dz
        if spec == "copernicus":
            return
        import rasterio
        from pyproj import Transformer
        with rasterio.open(spec) as ds:
            self.data = ds.read(1, masked=True).astype(np.float64).filled(np.nan)
            self.t = ds.transform
            self.tf = Transformer.from_crs("EPSG:3826", ds.crs, always_xy=True)

    def __call__(self, x, y):
        if self.spec == "copernicus":
            return G.dem_ellipsoidal(x, y)
        from scipy.ndimage import map_coordinates
        x, y = np.asarray(x, float), np.asarray(y, float)
        u, v = self.tf.transform(x.ravel(), y.ravel())
        inv = ~self.t
        col, row = inv * (u, v)
        z = map_coordinates(self.data, [np.asarray(row) - 0.5, np.asarray(col) - 0.5], order=1, mode="nearest",
                            cval=np.nan)
        bad = ~np.isfinite(z)
        if bad.any():
            z[bad] = G.dem_ellipsoidal(x.ravel()[bad], y.ravel()[bad]) - self.dz
        return (z + self.dz).reshape(np.shape(x))


def load_map(spec: str) -> G.MapRaster:
    if spec in ("main", "y2021"):
        return G.MapRaster(G.map_path(spec, RES))
    import rasterio
    with rasterio.open(spec) as ds:
        if ds.crs is None or ds.crs.to_epsg() != 3826 or abs(ds.transform.a - RES) > 1e-6:
            raise SystemExit(f"{spec}: needs EPSG:3826 at {RES} m/px; run `prepare-map` first")
    return G.MapRaster(Path(spec))


def map_offset(spec_map: str, spec_off: str, calib: dict | None) -> tuple[float, float]:
    """Map georeferencing offset (map = true + offset), always estimated on pre-cut photos."""
    if spec_off == "stage1":
        if spec_map != "main":
            raise SystemExit("--map-offset stage1 is only defined for --map main")
        o = json.loads((G.XOUT / "stage1_calibration.json").read_text())["map_offsets"]["main"]
        return float(o["de_m"]), float(o["dn_m"])
    if spec_off == "calib":
        return tuple(calib["map_offset_precut"])
    de, dn = (float(v) for v in spec_off.split(","))
    return de, dn


# ----------------------------------------------------------------------------- query (rectified patch)

def make_query(img, factor, att, cam, ground: str, baro_alt: float, est_xy, yaw_err: float, bs: dict,
               terrain: Terrain) -> dict:
    """Same as x_tuniu_match.build_query(height=baro_dem, attitude=dji) with a swappable terrain model.
    Height above ground = SIMULATED baro - terrain under the FILTER estimate."""
    yaw = att.gimbal_yaw_deg + bs["yaw"] + yaw_err
    R = G.rot_enu_cam(yaw, att.gimbal_pitch_deg + bs["pitch"], att.gimbal_roll_deg + bs["roll"])
    z0 = float(terrain(est_xy[0], est_xy[1]))
    gfun = None
    if ground == "dem_lifted":
        def gfun(X, Y):
            return terrain(est_xy[0] + X, est_xy[1] + Y) - z0
    elif ground != "dem_prior":
        raise ValueError(ground)
    q = G.rectify(img, factor, cam, R, baro_alt - z0, RES, far_m=FAR_M, ground=gfun)
    q["height_used"] = baro_alt - z0
    q["fwd"] = np.array([math.sin(math.radians(yaw)), math.cos(math.radians(yaw))])
    return q


# ----------------------------------------------------------------------------- filter

def rot(b):
    """Rotation that undoes a heading error b (rad): measured vectors are turned clockwise by b."""
    c, s = math.cos(b), math.sin(b)
    return np.array([[c, -s], [s, c]])


class Filter:
    """EKF on (east, north, heading error b). Odometry D_m is measured in a frame turned by b."""

    def __init__(self, xy, pos_sd=P0_POS_SD, b_sd_deg=B0_SD_DEG, b_rw=B_RW_DEG_SQRT_S):
        self.x = np.array([xy[0], xy[1], 0.0])
        self.P = np.diag([pos_sd ** 2, pos_sd ** 2, math.radians(b_sd_deg) ** 2])
        self.qb = math.radians(b_rw) ** 2

    def predict(self, Dm, sigma, dt):
        b = self.x[2]
        c, s = math.cos(b), math.sin(b)
        self.x[:2] += rot(b) @ Dm
        F = np.eye(3)
        F[:2, 2] = np.array([[-s, -c], [c, -s]]) @ Dm
        self.P = F @ self.P @ F.T + np.diag([sigma ** 2, sigma ** 2, self.qb * dt])

    def half_window(self) -> float:
        lam = float(np.linalg.eigvalsh(self.P[:2, :2]).max())
        return min(WIN_MAX_M, max(WIN_MIN_M, WIN_SIGMA * math.sqrt(lam)))

    def update(self, z, sigma) -> tuple[float, bool]:
        Hm = np.zeros((2, 3))
        Hm[0, 0] = Hm[1, 1] = 1.0
        R = np.eye(2) * sigma ** 2
        y = np.asarray(z, float) - self.x[:2]
        S = self.P[:2, :2] + R
        nis = float(y @ np.linalg.solve(S, y))
        if nis > GATE_FIX:
            return nis, False
        K = self.P @ Hm.T @ np.linalg.inv(S)
        self.x += K @ y
        IKH = np.eye(3) - K @ Hm
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        return nis, True


# ----------------------------------------------------------------------------- matching helpers

_W: dict = {}


def _worker_init():
    import cv2
    cv2.setNumThreads(1)
    try:
        import torch
        torch.set_num_threads(1)
    except ImportError:
        pass


def _static():
    if "cam" not in _W:
        cal = json.loads((G.XOUT / "stage1_calibration.json").read_text())
        _W.update(cam=G.Camera(), ph=G.photos(), bs=cal["boresight_deg"], factor=G.reduce_factor(RES))
    return _W


def odometry(qa, qb):
    from x4_tuniu_speed import displacement
    d, peak = displacement(qa, qb)
    return d, peak


def fallback(D_last, dt_last: float, dt: float, yaw_a_deg: float, yaw_b_deg: float) -> np.ndarray:
    """Displacement when odometry fails: last measured speed, along the mean camera heading of the pair
    (DJI gimbal yaw + boresight, + the simulated drift when active), in the same frame as odometry."""
    yaw = math.radians(yaw_a_deg + float(wrap180(yaw_b_deg - yaw_a_deg)) / 2)
    return float(np.hypot(*D_last)) * dt / dt_last * np.array([math.sin(yaw), math.cos(yaw)])


def consensus_fix(m: G.MapRaster, q: dict, centre_map_xy, half: float) -> dict:
    """Pre-registered agreement rule on one query: both fixes exist and agree within 4 m -> ZNCC fix.
    Positions returned in MAP coordinates (offset not removed)."""
    import x_tuniu_match as M
    ref = M.reference_for(m, q, centre_map_xy, half)
    out = dict(ref_valid=float(ref["valid"].mean()))
    fx = {}
    for meth in METHODS:
        r = M.run_method(meth, q, ref)
        fx[meth] = r["fix"]
        out[f"{meth}_s"] = r["latency_s"]
        out[f"{meth}_mx"] = np.nan if r["fix"] is None else r["fix"][0]
        out[f"{meth}_my"] = np.nan if r["fix"] is None else r["fix"][1]
    if fx["zncc"] is None or fx["xfeat"] is None:
        out["fix_status"] = "nofix"
    elif float(np.hypot(*(np.asarray(fx["zncc"]) - np.asarray(fx["xfeat"])))) > AGREE_M:
        out["fix_status"] = "disagree"
    else:
        out["fix_status"] = "agree"
    return out


# ----------------------------------------------------------------------------- one closed-loop sequence

def run_sequence(u: dict) -> list[dict]:
    """Estimator side. `u` holds only estimator inputs: frames, times, SIMULATED baro and heading error,
    the state at the cut, the calibration and the configuration. No RTK after the cut."""
    W = _static()
    cam, ph, bs, fac = W["cam"], W["ph"], W["bs"], W["factor"]
    key = ("map", u["map"])
    if key not in _W:
        _W[key] = load_map(u["map"]) if u["mode"] == "loop" else None
    tkey = ("terrain", u["terrain"], u["terrain_dz"])
    if tkey not in _W:
        _W[tkey] = Terrain(u["terrain"], u["terrain_dz"])
    m, terrain = _W[key], _W[tkey]
    c = u["calib"]
    off = np.array(u["offset"])
    frames, t = u["frames"], np.asarray(u["t_s"])
    flt = Filter(u["xy0"])
    att0 = ph.iloc[frames[0] - 1]
    q_prev = make_query(G.load_gray(att0.path, fac), fac, att0, cam, u["ground"], u["baro"][0], flt.x[:2],
                        u["yaw_err"][0], bs, terrain)
    D_last, dt_last = np.asarray(u["D0"], float), float(u["dt0"])
    rows = []
    for i in range(1, len(frames)):
        t0 = time.perf_counter()
        f = frames[i]
        att, att_prev = ph.iloc[f - 1], ph.iloc[frames[i - 1] - 1]
        dt = float(t[i] - t[i - 1])
        dyaw = float(wrap180(att.gimbal_yaw_deg - att_prev.gimbal_yaw_deg))
        turn = abs(dyaw) > TURN_DEG
        D_fb = fallback(D_last, dt_last, dt, att_prev.gimbal_yaw_deg + bs["yaw"] + u["yaw_err"][i - 1],
                        att.gimbal_yaw_deg + bs["yaw"] + u["yaw_err"][i])
        prov = flt.x[:2] + rot(flt.x[2]) @ D_fb          # provisional estimate, only for the terrain height
        q = make_query(G.load_gray(att.path, fac), fac, att, cam, u["ground"], u["baro"][i], prov,
                       u["yaw_err"][i], bs, terrain)
        D_m, peak = odometry(q_prev, q)
        s_odo = c["odo_sd_turn"] if turn else c["odo_sd_straight"]
        s_fb = c["fb_sd_turn"] if turn else c["fb_sd_straight"]
        if D_m is None:
            status = "nomatch"
        elif np.hypot(*D_m) / dt > V_MAX:
            status = "implausible"
        elif not peak >= PEAK_MIN:
            status = "lowpeak"
        else:
            status = "ok"
        if status == "ok":
            D_use, s_use = D_m, s_odo
            D_last, dt_last = D_m, dt
        else:
            D_use, s_use = D_fb, s_fb
        flt.predict(D_use, s_use, dt)
        pred = flt.x[:2].copy()
        Ppred = flt.P.copy()
        half = flt.half_window()
        row = dict(frame=f, t_s=float(t[i]), dt=dt, dyaw=dyaw, turn=turn, odo_status=status, odo_peak=peak,
                   odo_e=np.nan if D_m is None else D_m[0], odo_n=np.nan if D_m is None else D_m[1],
                   used_e=D_use[0], used_n=D_use[1], q_sd=s_use, pred_x=pred[0], pred_y=pred[1],
                   pred_sd_e=math.sqrt(Ppred[0, 0]), pred_sd_n=math.sqrt(Ppred[1, 1]), pred_cov_en=Ppred[0, 1],
                   half_m=half, height_used=q["height_used"], yaw_err_sim_deg=u["yaw_err"][i])
        if u["mode"] == "loop":
            fx = consensus_fix(m, q, pred + off, half)
            row.update(fx)
            if fx["fix_status"] == "agree":
                z = np.array([fx["zncc_mx"], fx["zncc_my"]]) - off
                nis, ok = flt.update(z, c["fix_sd"])
                row.update(z_x=z[0], z_y=z[1], nis=nis, fix_status="accepted" if ok else "gated")
        row.update(post_x=flt.x[0], post_y=flt.x[1], post_sd_e=math.sqrt(flt.P[0, 0]),
                   post_sd_n=math.sqrt(flt.P[1, 1]), b_hat_deg=math.degrees(flt.x[2]),
                   b_sd_deg=math.degrees(math.sqrt(flt.P[2, 2])), step_s=time.perf_counter() - t0)
        rows.append(row)
        q_prev = q
    return rows


# ----------------------------------------------------------------------------- parent: inputs and scoring

def simulated_heading_error(heading: str, t_s: np.ndarray, seed: int, t_start: float) -> np.ndarray:
    """SIMULATED heading error (deg) added to the DJI gimbal yaw: 0 for `dji`; for `drift` a per-seed
    constant N(0, 2 deg) + random walk 0.1 deg/sqrt(s) starting at the first sequence photo."""
    if heading == "dji":
        return np.zeros(len(t_s))
    rng = np.random.default_rng(seed_of("headdrift", seed))
    dt = np.diff(np.asarray(t_s, float) - t_start, prepend=0.0)
    return rng.normal(0, B0_SD_DEG) + np.cumsum(rng.normal(0, 1, len(t_s)) * B_RW_DEG_SQRT_S * np.sqrt(dt))


def unit_for(seed, heading, ground, mode, args, calib, offset) -> dict:
    """Parent: builds the estimator inputs (uses RTK only for the cut state and the SIMULATED baro)."""
    prot = G.protocol()
    tr = G.truth_xy()
    last_pre = prot["precut_frames"][-1]
    frames = [last_pre] + list(prot["test_frames"])
    baro = G.simulated_baro(tr.t_s.to_numpy(), tr.alt_ell.to_numpy(), seed_of("baro", seed), prot["cut_s"])
    idx = np.array(frames) - 1
    t_s = tr.t_s.to_numpy()[idx]
    p_last, p_prev = tr.iloc[last_pre - 1], tr.iloc[last_pre - 2]
    return dict(seed=seed, heading=heading, ground=ground, mode=mode, map=args.map, terrain=args.terrain,
                terrain_dz=args.terrain_dz, offset=offset, calib=calib[ground], frames=frames, t_s=t_s.tolist(),
                baro=baro[idx].tolist(), yaw_err=simulated_heading_error(heading, t_s, seed, t_s[0]).tolist(),
                xy0=(float(p_last.x), float(p_last.y)),
                D0=(float(p_last.x - p_prev.x), float(p_last.y - p_prev.y)), dt0=float(p_last.t_s - p_prev.t_s))


def score(rows: list[dict], u: dict) -> pd.DataFrame:
    """Evaluator: errors against RTK (parent process only)."""
    df = pd.DataFrame(rows)
    tr = G.truth_xy().set_index("frame")
    tx, ty = tr.loc[df.frame, "x"].to_numpy(), tr.loc[df.frame, "y"].to_numpy()
    df["true_x"], df["true_y"] = tx, ty
    df["err_pred_m"] = np.hypot(df.pred_x - tx, df.pred_y - ty)
    df["err_m"] = np.hypot(df.post_x - tx, df.post_y - ty)
    df["lol"] = df.err_pred_m > df.half_m
    df["lol_cheb"] = np.maximum(abs(df.pred_x - tx), abs(df.pred_y - ty)) > df.half_m
    if "z_x" in df:
        df["fix_err_m"] = np.hypot(df.z_x - tx, df.z_y - ty)
    for k in ("seed", "heading", "ground", "mode", "map", "terrain"):
        df[k] = u[k]
    return df


def run_path(tag, mode, heading, ground, seed) -> Path:
    return OUT / "runs" / tag / f"{mode}_{heading}_{ground}_s{seed}.csv"


def cmd_run(args):
    calib = json.loads((OUT / f"calibration_{args.calib_tag}.json").read_text())
    offset = map_offset(args.map, args.map_offset, calib.get(args.map, {}) if args.map_offset == "calib" else None)
    units = []
    for mode in args.mode:
        for heading in args.heading:
            for ground in args.ground:
                for seed in args.seeds:
                    if run_path(args.tag, mode, heading, ground, seed).exists():
                        continue          # checkpoint: finished runs are not repeated
                    units.append(unit_for(seed, heading, ground, mode, args, calib, offset))
    (OUT / "runs" / args.tag).mkdir(parents=True, exist_ok=True)
    print(f"{len(units)} runs to do (tag {args.tag}, offset {offset}), workers {args.workers}", flush=True)
    t0 = time.time()
    with get_context("spawn").Pool(args.workers, initializer=_worker_init, maxtasksperchild=8) as pool:
        for u, rows in zip(units, pool.imap(run_sequence, units, chunksize=1)):
            df = score(rows, u)
            df.to_csv(run_path(args.tag, u["mode"], u["heading"], u["ground"], u["seed"]), index=False)
            print(f"  {u['mode']} {u['heading']} {u['ground']} seed {u['seed']}: median {df.err_m.median():.1f} m, "
                  f"max {df.err_m.max():.1f} m, LoL photos {int(df.lol.sum())}, "
                  f"accepted {int((df.get('fix_status') == 'accepted').sum())}, {time.time() - t0:.0f} s", flush=True)


# ----------------------------------------------------------------------------- calibration (pre-cut only)

def cmd_calibrate(args):
    """Pre-cut photos 1-46, GNSS still available: est = RTK, seed-0 SIMULATED baro, DJI heading.
    Odometry and fallback residuals vs RTK displacement; consensus-fix residuals vs RTK."""
    import cv2
    cv2.setNumThreads(1)
    W = _static()
    cam, ph, bs, fac = W["cam"], W["ph"], W["bs"], W["factor"]
    prot = G.protocol()
    tr = G.truth_xy().set_index("frame")
    pre = prot["precut_frames"]
    baro = G.simulated_baro(tr.t_s.to_numpy(), tr.alt_ell.to_numpy(), seed_of("baro", 0), prot["cut_s"])
    m = load_map(args.map)
    terrain = Terrain(args.terrain, args.terrain_dz)
    off_stage1 = map_offset("main", "stage1", None) if args.map == "main" else (0.0, 0.0)
    out, rows = {}, []
    for ground in ("dem_prior", "dem_lifted"):
        qs = {}
        for f in pre:
            att = ph.iloc[f - 1]
            xy = (tr.loc[f, "x"], tr.loc[f, "y"])
            qs[f] = q = make_query(G.load_gray(att.path, fac), fac, att, cam, ground, baro[f - 1], xy, 0.0, bs,
                                   terrain)
            fx = consensus_fix(m, q, np.array(xy) + np.array(off_stage1), WIN_MIN_M)
            r = dict(ground=ground, frame=f, kind="fix", **fx,
                     raw_e=fx["zncc_mx"] - xy[0], raw_n=fx["zncc_my"] - xy[1])
            rows.append(r)
            if f > pre[0]:
                a, b = ph.iloc[f - 2], att
                dyaw = float(wrap180(b.gimbal_yaw_deg - a.gimbal_yaw_deg))
                d, peak = odometry(qs[f - 1], q)
                true_d = np.array([tr.loc[f, "x"] - tr.loc[f - 1, "x"], tr.loc[f, "y"] - tr.loc[f - 1, "y"]])
                dt = tr.loc[f, "t_s"] - tr.loc[f - 1, "t_s"]
                rr = dict(ground=ground, frame=f, kind="odo", dyaw=dyaw, turn=abs(dyaw) > TURN_DEG, peak=peak,
                          odo_e=np.nan if d is None else d[0] - true_d[0],
                          odo_n=np.nan if d is None else d[1] - true_d[1],
                          speed=np.nan if d is None else np.hypot(*d) / dt)
                if f - 1 > pre[0]:
                    prev = np.array([tr.loc[f - 1, "x"] - tr.loc[f - 2, "x"], tr.loc[f - 1, "y"] - tr.loc[f - 2, "y"]])
                    dtp = tr.loc[f - 1, "t_s"] - tr.loc[f - 2, "t_s"]
                    fb = fallback(prev, dtp, dt, a.gimbal_yaw_deg + bs["yaw"], b.gimbal_yaw_deg + bs["yaw"])
                    rr.update(fb_e=fb[0] - true_d[0], fb_n=fb[1] - true_d[1])
                rows.append(rr)
            print(f"  {ground} frame {f} {rows[-1]}", flush=True) if args.verbose else None
        d = pd.DataFrame([r for r in rows if r["ground"] == ground])
        fix = d[(d.kind == "fix") & (d.fix_status == "agree")]
        odo = d[d.kind == "odo"].copy()
        odo["turn"] = odo.turn.astype(bool)
        ok = odo.odo_e.notna() & (odo.speed <= V_MAX) & (odo.peak >= PEAK_MIN)

        def mad_sd(v):
            v = np.asarray(v, float)
            v = v[np.isfinite(v)]
            return float(1.4826 * np.median(np.abs(v - np.median(v)))) if len(v) else np.nan

        def rms(v):
            v = np.asarray(v, float)
            v = v[np.isfinite(v)]
            return float(np.sqrt(np.mean(v ** 2))) if len(v) else np.nan

        def drift_sd(n):
            """Per-photo sd that reproduces the RMS error accumulated over n consecutive straight pairs."""
            good = (ok & ~odo.turn).to_numpy()
            e, nn, fr = odo.odo_e.to_numpy(), odo.odo_n.to_numpy(), odo.frame.to_numpy()
            acc = []
            for i in range(len(odo) - n + 1):
                sl = slice(i, i + n)
                if good[sl].all() and fr[i + n - 1] - fr[i] == n - 1:
                    acc.append(e[sl].sum() ** 2 + nn[sl].sum() ** 2)
            return float(np.sqrt(np.mean(acc) / (2 * n))) if acc else np.nan, len(acc)

        st, tu = odo[ok & ~odo.turn], odo[ok & odo.turn]
        fix_res = np.r_[fix.raw_e - off_stage1[0], fix.raw_n - off_stage1[1]]
        sd_h, n_win = drift_sd(ODO_HORIZON)
        odo_sd = max(SIGMA_FLOOR_ODO, sd_h)
        out[ground] = dict(
            odo_sd_straight=odo_sd,
            odo_sd_turn=max(odo_sd, rms(np.r_[tu.odo_e, tu.odo_n])),
            fb_sd_straight=max(SIGMA_FLOOR_ODO, rms(np.r_[odo.fb_e[~odo.turn], odo.fb_n[~odo.turn]])),
            fb_sd_turn=max(SIGMA_FLOOR_ODO, rms(np.r_[odo.fb_e[odo.turn], odo.fb_n[odo.turn]])),
            fix_sd=max(SIGMA_FLOOR_FIX, mad_sd(fix_res)),
            raw=dict(odo_drift_sd_by_horizon={n: drift_sd(n)[0] for n in (1, 5, 10, 20)}, odo_drift_windows=n_win,
                     odo_pairs=len(odo), odo_ok=int(ok.sum()), odo_turn_pairs=int(odo.turn.sum()),
                     odo_mad_sd_straight=mad_sd(np.r_[st.odo_e, st.odo_n]), odo_rms_turn=rms(np.r_[tu.odo_e, tu.odo_n]),
                     odo_abs_max_straight=float(np.nanmax(np.abs(np.r_[st.odo_e, st.odo_n]))) if len(st) else np.nan,
                     fb_rms_straight=rms(np.r_[odo.fb_e[~odo.turn], odo.fb_n[~odo.turn]]),
                     fb_rms_turn=rms(np.r_[odo.fb_e[odo.turn], odo.fb_n[odo.turn]]),
                     fix_agree=len(fix), fix_photos=int((d.kind == "fix").sum()), fix_mad_sd=mad_sd(fix_res),
                     fix_res_abs_max=float(np.nanmax(np.abs(fix_res))) if len(fix) else np.nan))
        print(ground, json.dumps(out[ground], indent=1), flush=True)
    allfix = pd.DataFrame([r for r in rows if r["kind"] == "fix" and r.get("fix_status") == "agree"])
    out[args.map] = dict(map_offset_precut=[float(allfix.raw_e.median()), float(allfix.raw_n.median())])
    out["meta"] = dict(map=args.map, terrain=args.terrain, terrain_dz=args.terrain_dz, res=RES, baro_seed=0,
                       frames=f"{pre[0]}-{pre[-1]} (pre-cut)", note="pre-cut photos only; RTK pose as estimate")
    OUT.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(OUT / f"calibration_{args.tag}_rows.csv", index=False)
    (OUT / f"calibration_{args.tag}.json").write_text(json.dumps(out, indent=1))
    print(f"wrote {OUT / f'calibration_{args.tag}.json'}")


# ----------------------------------------------------------------------------- main

def add_inputs(p):
    p.add_argument("--map", default="main", help="main | y2021 | path to an EPSG:3826 GeoTIFF at 0.5 m/px")
    p.add_argument("--terrain", default="copernicus", help="copernicus | path to a DSM/DTM GeoTIFF")
    p.add_argument("--terrain-dz", type=float, default=0.0, help="metres added to the terrain raster heights")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("calibrate")
    add_inputs(c)
    c.add_argument("--tag", default="main")
    c.add_argument("--verbose", action="store_true")
    r = sub.add_parser("run")
    add_inputs(r)
    r.add_argument("--map-offset", default="stage1", help="stage1 | calib | de,dn")
    r.add_argument("--calib-tag", default="main")
    r.add_argument("--tag", default="main")
    r.add_argument("--mode", nargs="+", choices=["loop", "dr"], default=["loop"])
    r.add_argument("--heading", nargs="+", choices=["dji", "drift"], default=["dji"])
    r.add_argument("--ground", nargs="+", choices=["dem_prior", "dem_lifted"], default=["dem_lifted"])
    r.add_argument("--seeds", nargs="+", type=int, default=list(range(20)))
    r.add_argument("--workers", type=int, default=2)
    args = ap.parse_args()
    {"calibrate": cmd_calibrate, "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    main()

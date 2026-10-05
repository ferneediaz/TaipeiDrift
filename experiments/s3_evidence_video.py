#!/usr/bin/env python3
"""S3: human-verifiable visual evidence for the SIMULATED Wufeng flight (Gazebo, Dan's simulator).

For every second of flight (one image out of five: the images OUR EKF processes, each one a map-fix attempt):
  LEFT    the raw down-camera image (what the drone sees);
  MIDDLE  a checkerboard (40 px squares) alternating the camera image, rectified north-up at map scale, and the
          2018 map, placed at OUR estimate (S1 EKF + consensus map fixes, outputs/s1_sim/runs/<camera>/loop_s0.csv;
          before the GNSS cut: the GNSS-aided ESKF). Roads and field edges continue across squares iff the
          estimate is right. Below: the same checkerboard at Dustin's estimate and at dead reckoning.
          Rectification = the matcher's (s1_sim_map_fix.make_query): roll/pitch from the truth quaternion (AHRS
          stand-in), heading and height exactly as our EKF passed them to the matcher (heading_used = truth +
          SIMULATED drift, height_used = SIMULATED barometer). Dustin / dead-reckoning panels use the same heading
          and height, so that only the position differs. Before the cut: the ESKF's own heading and altitude.
          The map is placed with the pre-cut calibrated map offset (calibration.json), as the matcher does.
  RIGHT   overview of the 2018 map with trajectories so far, accepted fixes, rejected / no-consensus attempts and
          the 450 m GNSS segment.
  HEADER  time, distance since the GNSS cut, horizontal error of each estimator, fix status at this frame.

Inputs (read only): recordings/ilhan_wufeng_south_80m, outputs/s1_sim (map, calibration, runs, dustin_<camera>),
outputs/s2_sim/runs/<camera> (A/B tracks and fix logs).

Outputs: outputs/evidence/evidence_s0.mp4 (+ evidence_realistic_s0.mp4 with --camera realistic), keyframes/,
why_sim_is_easier.png, frame_table_<camera>_s0.csv.

  PY=/Users/ilhan.neuville/dev/hackathon/TaipeiDrift/.venv/bin/python
  $PY experiments/s3_evidence_video.py video --camera ideal          # ~10 min, 2 workers
  $PY experiments/s3_evidence_video.py why
  $PY experiments/s3_evidence_video.py video --camera realistic --clip-m 1200
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from functools import lru_cache
from multiprocessing import get_context
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(__file__))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import s1_sim_map_fix as S1  # noqa: E402

ROOT = S1.ROOT
OUT = ROOT / "outputs/evidence"
S2 = ROOT / "outputs/s2_sim/runs"
FFMPEG = "/opt/homebrew/bin/ffmpeg"
TUNIU_CHECK = Path("/Users/ilhan.neuville/dev/hackathon/TaipeiDrift/data/processed/x_tuniu/gallery/"
                   "main_xfeat_0.25m/accepted_f200_s0.png")   # local only, licence unknown: never publish
GROUND_2020 = ROOT / "data/raw/aerial/wufeng_2020-03-23_x4.tif"
SEED = 0
FPS = 6
SQ = 40                      # checkerboard square, display px
CROP_M = 120.0               # checkerboard extent (m), centred on the nadir
W, H = 1920, 1080
KEY_EVERY_M = 400.0          # keyframe rule fixed before looking: every 400 m after the cut
WRONG_M = 10.0

# BGR colours
C_TRUTH, C_OURS, C_DUSTIN, C_A, C_DR, C_B = ((255, 255, 255), (255, 140, 30), (60, 200, 60), (0, 150, 255),
                                              (40, 40, 230), (230, 220, 120))
C_GREY = (150, 150, 150)
FONT = cv2.FONT_HERSHEY_SIMPLEX


# ----------------------------------------------------------------------------- data

def s1_run(camera: str) -> tuple[pd.DataFrame, str]:
    """Our EKF (S1) + dead reckoning: final runs/ if present, else the x5-filter runs."""
    for p in (S1.run_path(camera, "loop", SEED), S1.run_path(camera, "loop", SEED, "x5")):
        if p.exists() and p.stat().st_size > 0:
            return pd.read_csv(p), str(p.relative_to(ROOT))
    raise SystemExit(f"no S1 loop run for {camera}")


@lru_cache(maxsize=2)
def load_all(camera: str) -> dict:
    rec = S1.load_recording()
    B = pd.read_csv(S2 / camera / f"B_s{SEED}_track.csv").set_index("image")
    A = pd.read_csv(S2 / camera / f"A_s{SEED}_track.csv").set_index("image")
    fx = pd.read_csv(S2 / camera / f"B_s{SEED}_fixes.csv")
    s1, s1_src = s1_run(camera)
    dus = pd.read_csv(ROOT / f"outputs/s1_sim/dustin_{camera}/frames_map_2018_s{SEED}.csv").set_index("image")
    cal = S1.calibration(camera)
    off = np.array(cal.get("map_offset_m", (0.0, 0.0)))
    # frame grid: OUR EKF's processed images (one per second, every one a fix attempt after the cut), extended
    # back to the flight start on the same 5-image grid. Before the cut the panel shows the GNSS-aided ESKF.
    s1 = s1.set_index("image")
    i0 = int(s1.index.min())
    first = rec.first + (i0 - rec.first) % S1.FIX_EVERY
    grid = np.arange(first, min(rec.last, B.index.max(), s1.index.max()) + 1, S1.FIX_EVERY)

    def col(df, c):
        return df[c].reindex(grid).to_numpy(float)

    tq = np.array([S1.heading_of(S1.r_enu_body(rec.q[i])) for i in grid])
    T = pd.DataFrame(dict(image=grid, t_s=rec.t[grid], true_e=rec.e[grid], true_n=rec.n[grid], true_u=rec.u[grid],
                          true_heading=tq, travelled=rec.travelled[grid],
                          dist_since_cut=rec.travelled[grid] - rec.travelled[rec.cut], after_cut=grid >= rec.cut))
    T["b_e"], T["b_n"], T["b_u"] = col(B, "est_e"), col(B, "est_n"), col(B, "est_u")
    T["b_heading"] = (tq + col(B, "heading_err_deg")) % 360
    T["a_e"], T["a_n"] = col(A, "est_e"), col(A, "est_n")
    T["s1_e"], T["s1_n"] = col(s1, "post_e"), col(s1, "post_n")
    T["dr_e"], T["dr_n"] = col(s1, "dr_e"), col(s1, "dr_n")
    # what our EKF passed to the matcher: heading = truth + SIMULATED drift, height = SIMULATED barometer
    T["s1_heading"], T["s1_height"] = col(s1, "heading_used"), col(s1, "height_used")
    T["d_e"], T["d_n"] = col(dus, "est_e"), col(dus, "est_n")
    for k in ("b", "a", "s1", "dr", "d"):
        T[f"err_{k}"] = np.hypot(T[f"{k}_e"] - T.true_e, T[f"{k}_n"] - T.true_n)
    # main panel: our EKF after the cut, the GNSS-aided ESKF before it
    pre = ~T.after_cut.to_numpy() | ~np.isfinite(T.s1_e.to_numpy())
    for a, b in (("e", "e"), ("n", "n"), ("heading", "heading"), ("height", "u")):
        T[f"m_{a}"] = np.where(pre, T[f"b_{b}"], T[f"s1_{a}"])
    T["fix_status"] = s1.fix_status.reindex(grid).fillna("").astype(str).to_numpy()
    T["fix_err"] = col(s1, "fix_err_m")
    T["z_e"], T["z_n"] = col(s1, "z_e"), col(s1, "z_n")
    T["b_fix_status"] = fx.set_index("image").status.reindex(grid).fillna("").astype(str).to_numpy()
    return dict(rec=rec, T=T, fixes=fx, off=off, s1_src=s1_src, camera=camera,
                b_src=f"outputs/s2_sim/runs/{camera}/B_s{SEED}_track.csv")


# ----------------------------------------------------------------------------- map

class ColourMap:
    """2018 map (RGB) in memory; window at EPSG:3826 coordinates."""

    def __init__(self, path=S1.MAP):
        import rasterio
        with rasterio.open(path) as ds:
            rgb = ds.read([1, 2, 3]).transpose(1, 2, 0)
            self.res = float(ds.transform.a)
            self.left, self.top = float(ds.transform.c), float(ds.transform.f)
        self.bgr = np.ascontiguousarray(rgb[..., ::-1])

    def crop(self, xc, yc, half_m, out_px, interp=cv2.INTER_LINEAR):
        """North-up crop centred on (xc, yc), half-size half_m, resampled to out_px x out_px."""
        s = out_px / (2 * half_m)                       # display px per metre
        # display pixel centre (u, v) -> map x = xc - half + (u + .5)/s ; map col = (x - left)/res - .5
        a = 1 / (s * self.res)
        M = np.array([[a, 0, (xc - half_m - self.left) / self.res - 0.5 + 0.5 * a],
                      [0, a, (self.top - (yc + half_m)) / self.res - 0.5 + 0.5 * a]], np.float32)
        return cv2.warpAffine(self.bgr, M, (out_px, out_px), flags=interp | cv2.WARP_INVERSE_MAP,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))


class Ground2020:
    """The orthophoto Gazebo renders (2020, 0.14 m/px), read by window."""

    def __init__(self, path=GROUND_2020):
        import rasterio
        self.ds = rasterio.open(path)

    def crop(self, xc, yc, half_m, out_px):
        from rasterio.enums import Resampling
        from rasterio.windows import from_bounds
        w = from_bounds(xc - half_m, yc - half_m, xc + half_m, yc + half_m, self.ds.transform)
        a = self.ds.read([1, 2, 3], window=w, out_shape=(3, out_px, out_px), boundless=True,
                         resampling=Resampling.bilinear)
        return np.ascontiguousarray(a.transpose(1, 2, 0)[..., ::-1])


_MAP: dict = {}


def cmap() -> ColourMap:
    if "m" not in _MAP:
        _MAP["m"] = ColourMap()
    return _MAP["m"]


# ----------------------------------------------------------------------------- checkerboard

def rectified_crop(rec, i, heading, height, camera, out_px, half_m=CROP_M / 2):
    """Camera image rectified north-up exactly as the matcher does, cropped +-half_m around the nadir and
    resampled to out_px. Returns (gray uint8, valid bool)."""
    q = S1.make_query(rec, int(i), float(heading), float(height), camera)
    s = out_px / (2 * half_m)
    a = 1 / (s * q["res"])
    # display pixel u -> ground x = -half + (u + .5)/s ; patch col = (x - x0)/res  (x0 = first column centre)
    M = np.array([[a, 0, (-half_m - q["x0"]) / q["res"] + 0.5 * a],
                  [0, a, (q["y0"] - half_m) / q["res"] + 0.5 * a]], np.float32)
    g = cv2.warpAffine(q["patch"], M, (out_px, out_px), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    v = cv2.warpAffine(q["valid"].astype(np.uint8) * 255, M, (out_px, out_px),
                       flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP) > 0
    return g, v


def checker(cam_g, cam_v, map_bgr, sq=SQ):
    n = cam_g.shape[0]
    yy, xx = np.mgrid[0:n, 0:n]
    use_cam = (((yy // sq) + (xx // sq)) % 2 == 0) & cam_v
    out = map_bgr.copy()
    out[use_cam] = np.repeat(cam_g[use_cam][:, None], 3, 1)
    # thin outline of the camera footprint
    cnt, _ = cv2.findContours(cam_v.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, cnt, -1, (0, 255, 255), 1)
    c = n // 2
    cv2.drawMarker(out, (c, c), (0, 255, 255), cv2.MARKER_CROSS, 14, 1)
    return out


def checker_at(D, i, e, n, heading, height, out_px):
    """Checkerboard of image i placed at ENU (e, n) with the given heading/height inputs."""
    if not np.isfinite([e, n, heading, height]).all():
        img = np.full((out_px, out_px, 3), 30, np.uint8)
        put(img, "no estimate", (10, out_px // 2), 0.7, C_GREY)
        return img
    g, v = rectified_crop(D["rec"], i, heading, height, D["camera"], out_px)
    x, y = e + D["rec"].origin[0] + D["off"][0], n + D["rec"].origin[1] + D["off"][1]
    return checker(g, v, cmap().crop(x, y, CROP_M / 2, out_px))


def put(img, text, org, scale=0.6, colour=(255, 255, 255), thick=1):
    cv2.putText(img, text, (org[0] + 1, org[1] + 1), FONT, scale, (0, 0, 0), thick, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, colour, thick, cv2.LINE_AA)


def put_multi(img, parts, org, scale=0.6, thick=1, gap=14):
    """Draw [(text, colour), ...] left to right."""
    x, y = org
    for text, colour in parts:
        put(img, text, (x, y), scale, colour, thick)
        x += cv2.getTextSize(text, FONT, scale, thick)[0][0] + gap


# ----------------------------------------------------------------------------- overview (right panel)

class Overview:
    def __init__(self, D, w=700, h=850):
        T, rec = D["T"], D["rec"]
        self.org = np.array(rec.origin) + D["off"]
        e = np.r_[T.true_e, T.dr_e.dropna(), T.d_e.dropna()]
        n = np.r_[T.true_n, T.dr_n.dropna(), T.d_n.dropna()]
        e0, e1, n0, n1 = e.min() - 60, e.max() + 60, n.min() - 60, n.max() + 60
        self.s = min(w / (e1 - e0), h / (n1 - n0))           # px per metre
        self.w, self.h = int((e1 - e0) * self.s), int((n1 - n0) * self.s)
        self.e0, self.n1 = e0, n1
        xc, yc = (e0 + e1) / 2 + self.org[0], (n0 + n1) / 2 + self.org[1]
        half = max(e1 - e0, n1 - n0) / 2
        big = cmap().crop(xc, yc, half, int(round(2 * half * self.s)), cv2.INTER_AREA)
        oy, ox = (big.shape[0] - self.h) // 2, (big.shape[1] - self.w) // 2
        base = (big[oy:oy + self.h, ox:ox + self.w] * 0.7).astype(np.uint8)
        # 450 m GNSS segment, shaded
        pre = T[~T.after_cut]
        over = base.copy()
        cv2.polylines(over, [self.px(pre.true_e, pre.true_n)], False, (0, 220, 255), 22, cv2.LINE_AA)
        self.base = cv2.addWeighted(over, 0.45, base, 0.55, 0)
        p = self.px([pre.true_e.iloc[0]], [pre.true_n.iloc[0]])[0]
        put(self.base, "GNSS ON (first 450 m)", (int(p[0]) + 14, int(p[1]) + 5), 0.5, (0, 220, 255))
        # 200 m scale bar
        L = int(200 * self.s)
        cv2.line(self.base, (15, self.h - 18), (15 + L, self.h - 18), (255, 255, 255), 3)
        put(self.base, "200 m", (15, self.h - 26), 0.5)
        put(self.base, "N ^", (self.w - 50, 25), 0.6)

    def px(self, e, n):
        e, n = np.asarray(e, float), np.asarray(n, float)
        return np.stack([(e - self.e0) * self.s, (self.n1 - n) * self.s], -1).round().astype(np.int32)

    def draw(self, D, k):
        T = D["T"].iloc[:k + 1]
        img = self.base.copy()
        r = T.iloc[-1]
        fx = T[T.after_cut & (T.fix_status != "")]
        bad = fx[fx.fix_status != "accepted"]
        for p in self.px(bad.s1_e, bad.s1_n):
            cv2.drawMarker(img, tuple(int(v) for v in p), C_GREY, cv2.MARKER_TILTED_CROSS, 7, 1)
        acc = fx[fx.fix_status == "accepted"]
        for p in self.px(acc.z_e, acc.z_n):
            cv2.circle(img, tuple(int(v) for v in p), 2, C_OURS, -1)
        for (ce, cn), c, th in ((("dr_e", "dr_n"), C_DR, 2), (("a_e", "a_n"), C_A, 2), (("d_e", "d_n"), C_DUSTIN, 2),
                                (("b_e", "b_n"), C_B, 1), (("true_e", "true_n"), C_TRUTH, 2),
                                (("s1_e", "s1_n"), C_OURS, 1)):
            ok = T[ce].notna()
            if ok.sum() > 1:
                cv2.polylines(img, [self.px(T[ce][ok], T[cn][ok])], False, c, th, cv2.LINE_AA)
        for ce, cn, c in (("dr_e", "dr_n", C_DR), ("a_e", "a_n", C_A), ("d_e", "d_n", C_DUSTIN),
                          ("true_e", "true_n", C_TRUTH), ("m_e", "m_n", C_OURS)):
            if np.isfinite([r[ce], r[cn]]).all():
                p = tuple(int(v) for v in self.px([r[ce]], [r[cn]])[0])
                cv2.circle(img, p, 6, c, 2, cv2.LINE_AA)
        return img


# ----------------------------------------------------------------------------- frame

def fmt_err(v):
    return "  -  " if not np.isfinite(v) else (f"{v:.1f} m" if v < 100 else f"{v:.0f} m")


def render(D, k, ov: Overview) -> np.ndarray:
    T, rec = D["T"], D["rec"]
    r = T.iloc[k]
    i = int(r.image)
    F = np.full((H, W, 3), 22, np.uint8)
    # header
    put(F, f"SIMULATED flight (Gazebo, Dan's simulator) - map 2018 vs rendered ground 2020 - seed {SEED}, "
           f"{D['camera']} camera", (12, 28), 0.72, (0, 230, 255), 2)
    if r.after_cut:
        st = f"GNSS OFF - {r.dist_since_cut:6.0f} m flown since the GNSS cut"
    else:
        st = f"GNSS ON - GNSS cut in {-r.dist_since_cut:4.0f} m"
    put(F, f"t = {r.t_s:6.1f} s   image {i}   {st}", (12, 60), 0.68, (255, 255, 255), 1)
    if r.after_cut:
        s = r.fix_status
        if s == "accepted":
            ft, fc = f"map fix ACCEPTED at this frame (fix error vs truth {r.fix_err:.1f} m)", C_OURS
        elif s == "gated":
            ft, fc = "map fix REJECTED by the 99 % gate", C_GREY
        elif s in ("nofix", "disagree"):
            ft, fc = f"no map fix: ZNCC and XFeat {'disagree' if s == 'disagree' else 'did not both match'}", C_GREY
        else:
            ft, fc = "", C_GREY
        put(F, ft, (1010, 60), 0.62, fc, 1)
    put(F, "horizontal error now:", (12, 92), 0.6, (200, 200, 200))
    put_multi(F, [(f"OURS (our EKF + consensus fixes) {fmt_err(r.err_s1 if r.after_cut else np.nan)}", C_OURS),
                  (f"ESKF + our fixes {fmt_err(r.err_b)}", C_B),
                  (f"Dustin {fmt_err(r.err_d)}", C_DUSTIN),
                  (f"ESKF alone {fmt_err(r.err_a)}", C_A),
                  (f"dead reckoning {fmt_err(r.err_dr)}", C_DR)], (215, 92), 0.6, 1, 22)
    cv2.line(F, (0, 104), (W, 104), (90, 90, 90), 1)
    # left: raw camera
    raw = cv2.imread(rec.paths[i], cv2.IMREAD_COLOR)
    if D["camera"] == "realistic":
        raw = cv2.cvtColor(S1.load_image(rec, i, "realistic"), cv2.COLOR_GRAY2BGR)
    raw = cv2.resize(raw, (560, 560), interpolation=cv2.INTER_AREA)
    F[140:700, 12:572] = raw
    put(F, "1. What the drone sees (raw down camera, top = drone forward)", (12, 124), 0.55)
    lines = ["2. How to read the middle panel:",
             " grey squares = camera photo, rectified north-up",
             " colour squares = 2018 map, cut out at the estimate",
             " roads, field edges continue across squares",
             "   -> the estimate is right",
             " lines jump at square borders",
             "   -> the estimate is wrong (jump = error)",
             f" squares {SQ} px = {SQ * CROP_M / 580:.1f} m (big), {SQ * CROP_M / 290:.1f} m (small)",
             " yellow + = estimated position; yellow line =",
             "   camera footprint"]
    for j, t in enumerate(lines):
        put(F, t, (12, 770 + 26 * j), 0.56, (230, 230, 230) if j else (255, 255, 255))
    # middle: checkerboards
    who = "OUR EKF estimate (GNSS OFF)" if r.after_cut else "GNSS-aided ESKF (GNSS ON)"
    put(F, f"Photo placed on the 2018 map at {who}", (590, 124), 0.58, C_OURS)
    F[132:712, 590:1170] = checker_at(D, i, r.m_e, r.m_n, r.m_heading, r.m_height, 580)
    if r.after_cut:
        put(F, f"at Dustin's estimate ({fmt_err(r.err_d)} off)", (590, 734), 0.5, C_DUSTIN)
        F[742:1032, 590:880] = checker_at(D, i, r.d_e, r.d_n, r.s1_heading, r.s1_height, 290)
        put(F, f"at dead reckoning ({fmt_err(r.err_dr)} off)", (890, 734), 0.5, C_DR)
        F[742:1032, 880:1170] = checker_at(D, i, r.dr_e, r.dr_n, r.s1_heading, r.s1_height, 290)
    else:
        put(F, "Dustin / dead reckoning: start at the GNSS cut", (590, 760), 0.55, C_GREY)
    # right: overview
    o = ov.draw(D, k)
    y0, x0 = 132, 1200 + (710 - o.shape[1]) // 2
    F[y0:y0 + o.shape[0], x0:x0 + o.shape[1]] = o
    put(F, "3. Trajectories so far on the 2018 map", (1200, 124), 0.58)
    put_multi(F, [("truth", C_TRUTH), ("ours", C_OURS), ("ESKF+fixes", C_B), ("Dustin", C_DUSTIN),
                  ("ESKF alone", C_A), ("dead reck.", C_DR)], (1200, y0 + o.shape[0] + 22), 0.5, 1, 12)
    put_multi(F, [("o accepted fix (ours)", C_OURS), ("x rejected / no consensus", C_GREY)],
              (1200, y0 + o.shape[0] + 44), 0.5, 1, 16)
    # footer
    put(F, "Rectification = the matcher's: roll/pitch from the simulator truth (AHRS stand-in); heading and height as "
           "the estimator passed them to the matcher (ours: truth + SIMULATED drift, SIMULATED baro). Map placed with "
           "the pre-cut calibrated offset.", (12, 1056), 0.4, (200, 200, 200))
    put(F, f"Sources: {D['s1_src']} (ours, dead reckoning), {D['b_src']} (ESKF + fixes), A_s{SEED}_track.csv, "
           f"outputs/s1_sim/dustin_{D['camera']}/frames_map_2018_s{SEED}.csv. Truth is used here only to print errors.",
           (12, 1074), 0.4, (200, 200, 200))
    return F


# ----------------------------------------------------------------------------- frames, keyframes, video

def _render_range(job):
    camera, ks, tmp = job
    S1._worker_init()
    D = load_all(camera)
    ov = Overview(D)
    for k in ks:
        cv2.imwrite(str(Path(tmp) / f"{k:05d}.png"), render(D, k, ov))
    return len(ks)


def keyframe_list(D) -> list[tuple[int, str]]:
    T = D["T"]
    a = T[T.after_cut & T.err_s1.notna()]
    out = []
    for d in np.arange(0, a.dist_since_cut.max() + 1e-9, KEY_EVERY_M):      # rule fixed before looking
        k = int((a.dist_since_cut - d).abs().idxmin())
        out.append((k, f"kf_{int(d):04d}m"))
    for j, k in enumerate(a.err_s1.nlargest(3).index):
        out.append((int(k), f"worst{j + 1}_ours_{a.err_s1[k]:.1f}m"))
    wrong = a[(a.fix_status == "accepted") & (a.fix_err > WRONG_M)]
    for k in wrong.index:
        out.append((int(k), f"wrongfix_{a.fix_err[k]:.0f}m"))
    return out


def cmd_video(args):
    import shutil
    import tempfile
    D = load_all(args.camera)
    T = D["T"]
    ks = list(range(len(T)))
    if args.clip_m:
        ks = [k for k in ks if T.after_cut.iloc[k] and T.dist_since_cut.iloc[k] <= args.clip_m]
    tmp = Path(tempfile.mkdtemp(prefix=f"s3_{args.camera}_"))
    chunks = [(args.camera, ks[j::args.workers], str(tmp)) for j in range(args.workers)]
    with get_context("spawn").Pool(args.workers) as pool:
        print(sum(pool.map(_render_range, chunks)), "frames rendered in", tmp, flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    name = "evidence_s0.mp4" if args.camera == "ideal" else f"evidence_{args.camera}_s0.mp4"
    seq = tmp / "seq"
    seq.mkdir()
    for n, k in enumerate(ks):
        os.link(tmp / f"{k:05d}.png", seq / f"{n:05d}.png")
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(seq / "%05d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-preset", "medium",
                    "-movflags", "+faststart", str(OUT / name)], check=True)
    kd = OUT / ("keyframes" if args.camera == "ideal" else f"keyframes_{args.camera}")
    kd.mkdir(exist_ok=True)
    for k, tag in keyframe_list(D):
        if k in ks:
            shutil.copy(tmp / f"{k:05d}.png", kd / f"{tag}_img{int(T.image.iloc[k])}.png")
    T.to_csv(OUT / f"frame_table_{args.camera}_s{SEED}.csv", index=False)
    shutil.rmtree(tmp)
    print(OUT / name, flush=True)


# ----------------------------------------------------------------------------- why the simulation is easier

def cmd_why(args):
    D = load_all("ideal")
    T, rec = D["T"], D["rec"]
    a = T[T.after_cut]
    pick = np.sort(np.random.default_rng(2026).choice(a.index.to_numpy(), 4, replace=False))
    g20 = Ground2020()
    P = 360
    rows = []
    for k in pick:
        r = T.loc[k]
        g, v = rectified_crop(rec, r.image, r.true_heading, r.true_u, "ideal", P)
        x, y = r.true_e + rec.origin[0] + D["off"][0], r.true_n + rec.origin[1] + D["off"][1]
        m18 = cmap().crop(x, y, CROP_M / 2, P)
        # the simulator renders the 2020 orthophoto at the route's own georeference (no calibrated offset)
        m20 = g20.crop(r.true_e + rec.origin[0], r.true_n + rec.origin[1], CROP_M / 2, P)
        cam = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        cam[~v] = 0
        tiles = [cam, m20, m18, checker(g, v, m18)]
        caps = [f"sim camera, rectified at TRUE pose (img {int(r.image)})", "2020 orthophoto = what Gazebo renders",
                "2018 map (what we match against)", "checkerboard camera / 2018 map"]
        for t, c in zip(tiles, caps):
            put(t, c, (6, 18), 0.42)
        rows.append(np.hstack([np.pad(t, ((4, 4), (4, 4), (0, 0))) for t in tiles]))
    sim = np.vstack(rows)
    real = cv2.imread(str(TUNIU_CHECK))
    w3 = real.shape[1] // 3
    real = real[22:, w3:2 * w3]                              # "at TRUE position" panel, its title row removed
    RW = 900
    real = cv2.resize(real, (RW, RW * real.shape[0] // real.shape[1]), interpolation=cv2.INTER_AREA)
    panel = np.full((sim.shape[0], RW, 3), 22, np.uint8)
    panel[40:40 + real.shape[0]] = real
    put(panel, "REAL photo (Tuniu, DJI), rectified at the RTK truth, checkerboard with its map", (8, 26), 0.6)
    y = 40 + real.shape[0] + 40
    for t, c in (("Even at the TRUE position, buildings lean and their roofs and shadows", (230, 230, 230)),
                 ("do not continue across squares: 3-D relief, sun, a real lens.", (230, 230, 230)),
                 ("In the simulation the ground is a flat photo: everything continues.", (230, 230, 230)),
                 ("", (0, 0, 0)),
                 ("source: TaipeiDrift/data/processed/x_tuniu/gallery/main_xfeat_0.25m/", (170, 170, 170)),
                 ("        accepted_f200_s0.png (middle panel)", (170, 170, 170)),
                 ("LOCAL ONLY - licence unknown - do not publish", (0, 0, 255))):
        put(panel, t, (8, y), 0.6, c, 2 if c == (0, 0, 255) else 1)
        y += 30
    real = panel
    body = np.hstack([sim, np.full((sim.shape[0], 16, 3), 22, np.uint8), real])
    head = np.full((110, body.shape[1], 3), 22, np.uint8)
    put(head, "Why the simulation is easier than reality: 4 random simulated frames (seed 2026) vs 1 real photo",
        (12, 34), 0.9, (255, 255, 255), 2)
    put(head, "Sim: the camera sees a flat orthophoto (2020) + synthetic trees, ideal pinhole camera, 80 m. "
              "Real: 3-D buildings, shadows, perspective, a real lens.", (12, 66), 0.62, (230, 230, 230))
    put(head, "Fix acceptance: 83 % in simulation vs 30 % on real photos.", (12, 96), 0.75, (0, 230, 255), 2)
    cv2.imwrite(str(OUT / "why_sim_is_easier.png"), np.vstack([head, body]))
    print(OUT / "why_sim_is_easier.png", [int(T.image[k]) for k in pick])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("video")
    v.add_argument("--camera", default="ideal", choices=("ideal", "realistic"))
    v.add_argument("--clip-m", type=float, default=0.0, help="only the first CLIP_M metres after the cut")
    v.add_argument("--workers", type=int, default=2)
    sub.add_parser("why")
    args = ap.parse_args()
    {"video": cmd_video, "why": cmd_why}[args.cmd](args)


if __name__ == "__main__":
    main()

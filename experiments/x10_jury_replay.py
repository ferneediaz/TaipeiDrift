#!/usr/bin/env python3
"""Jury video of the REAL Tuniu flight (DJI Phantom 4 RTK, 2019-04-11, 271 photos, one every 2.8 s), in the layout of
experiments/s3_evidence_video.py on branch ilhan/sim-demo (helpers copied from it). One video frame per photo. 100 %
real: real photos, our system, RTK truth, dead reckoning (labelled "without map fixes"); no rendered image in the
main video.

  HEADER  time, photo index, GNSS ON (calibration, first 2:08) / OFF, metres flown since the GNSS cut, horizontal
          error now (ours | dead reckoning), fix status at this photo and its error vs truth, fixes so far.
  LEFT    top: the real photo (what the drone sees); bottom: error vs time, ours vs dead reckoning, filling up,
          GNSS-cut line, grey spans = more than 20 s without an accepted fix.
  MIDDLE  checkerboard alternating the real photo, rectified north-up at 0.5 m/px exactly as the navigation does
          it (x5_tuniu_closed_loop.make_query: DJI gimbal attitude + pre-cut boresight, seed-0 SIMULATED barometer,
          Copernicus terrain lifted), and the Dec 2019 map (OAM 2019-12-12, 'main') cut out at OUR estimate with the
          pre-cut stage-1 map offset, as the matcher does. Below: the same photo at dead reckoning. Before the cut:
          placed at the RTK position.
  RIGHT   overview of the Dec 2019 map with the tracks so far, accepted fixes o, rejected attempts x, the GNSS-on
          segment, 200 m scale.
  START   3 s title card: where the data come from (docs/research/tuniu-team-explainer.md, section 2).
  END     4 s card: 20-seed results of this flight (x_tuniu_l2 summary_main_pooled.csv, dji + dem_lifted) and the
          pre-registered digital-twin comparison (x9_tuniu_twin summary_l2.csv).
  FOOTER  data provenance and what is real / simulated, always visible.

Numbers come from the seed-0 run files (RUN_* below) and the summary CSVs, never recomputed; truth (RTK) is only
used to print errors and the distance flown.

    .venv/bin/python experiments/x10_jury_replay.py export                 # data.json + 1280 px photos (model frame)
    .venv/bin/python experiments/x10_jury_replay.py video --stills 70 190 250
    .venv/bin/python experiments/x10_jury_replay.py check                  # stills' numbers vs the CSV rows
    .venv/bin/python experiments/x10_jury_replay.py video --start 60 --end 72 --out preview.mp4 --no-card
    .venv/bin/python experiments/x10_jury_replay.py video                  # jury_replay.mp4
    .venv/bin/python experiments/x10_jury_replay.py clip                   # jury_forest_gap.mp4, cut from it

Optional 3D clip (pyvista, offscreen): the OpenDroneMap model + a surroundings layer (Copernicus terrain draped
with the Dec 2019 map, grey where the map has no data) + the tracks, camera following the drone:
    uv run --with pyvista python experiments/x10_jury_replay.py flythrough --stills 190
    uv run --with pyvista python experiments/x10_jury_replay.py flythrough               # jury_3d_flythrough.mp4
Other model (same OpenDroneMap layout, e.g. the full 3D mesh): `flythrough --src <.../odm_texturing>`; if its
odm_georeferencing/coords.txt differs, run `export --src <same folder>` first (tracks in that model's frame:
UTM 51N minus coords.txt as x8_odm_viewer.offset(), plus the x9 twin placement).
The photos' licence is unknown and every frame derives from them: everything stays under data/processed/x10_jury_replay.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(__file__))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import x5_tuniu_closed_loop as X5  # noqa: E402
import x8_odm_viewer as X8  # noqa: E402
import x_tuniu_geo as G  # noqa: E402
from x3_tuniu_fix import seed_of  # noqa: E402

ROOT = G.ROOT
REAL = ROOT / "data/processed/t_replay/tuniu_tw_1"
TWIN = ROOT / "data/processed/t_replay/tuniu_tw_1_twin_rc"
RUN_LOOP = ROOT / "data/processed/x_tuniu_l2/runs/main/loop_dji_dem_lifted_s0.csv"
RUN_DR = ROOT / "data/processed/x_tuniu_l2/runs/main/dr_dji_dem_lifted_s0.csv"
RUN_SIM = ROOT / "data/processed/x9_tuniu_twin/nav/twin_rc/l2/runs/main/loop_dji_dem_lifted_s0.csv"  # export only
SUMMARY_L2_POOLED = ROOT / "data/processed/x_tuniu_l2/summary_main_pooled.csv"
SUMMARY_L2 = ROOT / "data/processed/x9_tuniu_twin/summary_l2.csv"
PREREG = "docs/research/tuniu-twin-prereg.md, commit 38b4cd7"
TWIN_CAL = ROOT / "data/processed/x9_tuniu_twin/twin_calibration.json"
OUT = ROOT / "data/processed/x10_jury_replay"
DEFAULT_SRC = ROOT / "data/processed/x_tuniu_survey_odm_full3d/odm_texturing_25d"
FFMPEG = shutil.which("ffmpeg") or "/opt/homebrew/bin/ffmpeg"

SEED = 0
FPS = 4
CARD_S = 4.0                 # end card
TITLE_S = 3.0                # title card (data provenance)
SQ, SQ_SMALL = 24, 16       # checkerboard square, display px (big 580 px panel, small 290 px panels)
CROP_M = 240.0               # checkerboard extent (m), centred on the photo footprint
W, H = 1920, 1080
WRONG_M = X5.WRONG_M         # accepted fix farther than this from RTK = wrong fix
GAP_S = 20.0                 # chart: shade spans longer than this without an accepted fix
EXPORT_W, EXPORT_Q = 1280, 85
RAMP_M = 60.0                # 3D surroundings: ramp from the model edge back to the terrain

# BGR colours (s3 palette)
C_TRUTH, C_OURS, C_DR = (255, 255, 255), (255, 140, 30), (40, 40, 230)
C_GREY, C_GNSS, C_HEAD = (150, 150, 150), (0, 220, 255), (0, 230, 255)
HONEST = ("Real: photos, DJI attitude (GNSS-aided), RTK truth (scoring only). Simulated: barometer.")
# provenance: docs/research/tuniu-team-explainer.md section 2
PROVENANCE = ("Real flight: DJI P4 RTK, Tuniu River, 2019-04-11 (ODM community dataset) | map OAM 2019-12-12 "
              "CC BY 4.0 | terrain Copernicus GLO-30")
FONT = cv2.FONT_HERSHEY_SIMPLEX


def mmss(t: float) -> str:
    t = int(math.floor(t))
    return f"{t // 60}:{t % 60:02d}"


def rel(p: Path) -> str:
    return str(Path(p).resolve().relative_to(ROOT))


# ----------------------------------------------------------------------------- data

def load_table() -> pd.DataFrame:
    """One row per photo (frame 1-271): RTK, the three seed-0 runs, fix status, seed-0 SIMULATED baro."""
    L, D, S = (pd.read_csv(p).set_index("frame") for p in (RUN_LOOP, RUN_DR, RUN_SIM))
    tr = G.truth_xy()
    prot = G.protocol()
    T = pd.DataFrame(dict(frame=tr.frame, t_s=tr.t_s, true_x=tr.x, true_y=tr.y)).set_index("frame", drop=False)
    for k, d in (("o", L), ("d", D), ("s", S)):
        T[f"{k}_x"], T[f"{k}_y"], T[f"err_{k}"] = d.post_x, d.post_y, d.err_m
        assert np.allclose(d.true_x, T.true_x[d.index]) and np.allclose(d.t_s, T.t_s[d.index])
    T["fix_status"] = L.fix_status.reindex(T.index).fillna("").astype(str)
    T["fix_err"], T["z_x"], T["z_y"] = L.fix_err_m, L.z_x, L.z_y
    T["corr_m"] = np.hypot(L.post_x - L.pred_x, L.post_y - L.pred_y)      # size of the filter update
    T["s_fix_status"] = S.fix_status.reindex(T.index).fillna("").astype(str)
    T["after_cut"] = T.frame >= int(L.index.min())
    step = np.r_[0.0, np.hypot(np.diff(T.true_x), np.diff(T.true_y))]
    last_pre = prot["precut_frames"][-1]
    T["dist_since_cut"] = np.cumsum(step) - np.cumsum(step)[last_pre - 1]
    T["baro"] = G.simulated_baro(tr.t_s.to_numpy(), tr.alt_ell.to_numpy(), seed_of("baro", SEED), prot["cut_s"])
    # main panel: our estimate after the cut, the RTK position (GNSS ON) before it
    T["m_x"] = np.where(T.after_cut, T.o_x, T.true_x)
    T["m_y"] = np.where(T.after_cut, T.o_y, T.true_y)
    return T


def no_fix_spans(t: np.ndarray, accepted: np.ndarray, t_cut: float) -> list[tuple[float, float]]:
    """(start, end) flight times with no accepted fix for more than GAP_S (the cut counts as the last fix)."""
    marks = [t_cut] + list(t[accepted])
    spans = [(a, b) for a, b in zip(marks[:-1], marks[1:]) if b - a > GAP_S]
    if t[-1] - marks[-1] > GAP_S:
        spans.append((marks[-1], float(t[-1])))
    return spans


# ----------------------------------------------------------------------------- map (copied from s3)

class ColourMap:
    """Dec 2019 map (RGB) in memory; window at EPSG:3826 coordinates."""

    def __init__(self, path=None):
        import rasterio
        with rasterio.open(path or G.map_path("main", X5.RES)) as ds:
            rgb = ds.read([1, 2, 3]).transpose(1, 2, 0)
            self.res = float(ds.transform.a)
            self.left, self.top = float(ds.transform.c), float(ds.transform.f)
        self.bgr = np.ascontiguousarray(rgb[..., ::-1])

    def crop(self, xc, yc, half_m, out_px, interp=cv2.INTER_LINEAR):
        """North-up crop centred on (xc, yc), half-size half_m, resampled to out_px x out_px."""
        s = out_px / (2 * half_m)
        a = 1 / (s * self.res)
        M = np.array([[a, 0, (xc - half_m - self.left) / self.res - 0.5 + 0.5 * a],
                      [0, a, (self.top - (yc + half_m)) / self.res - 0.5 + 0.5 * a]], np.float32)
        return cv2.warpAffine(self.bgr, M, (out_px, out_px), flags=interp | cv2.WARP_INVERSE_MAP,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))


# ----------------------------------------------------------------------------- checkerboard (s3, adapted)

class Queries:
    """The navigation's rectified query of each photo (x5 make_query), built at the main-panel estimate."""

    def __init__(self, T: pd.DataFrame):
        self.T = T
        self.W = X5._static()
        self.terrain = X5.Terrain("copernicus")
        self.off = np.array(X5.map_offset("main", "stage1", None))
        self.cache: dict[int, dict] = {}

    def get(self, f: int) -> dict:
        if f not in self.cache:
            W, r = self.W, self.T.loc[f]
            att = W["ph"].iloc[f - 1]
            q = X5.make_query(G.load_gray(att.path, W["factor"]), W["factor"], att, W["cam"], "dem_lifted",
                              float(r.baro), (float(r.m_x), float(r.m_y)), 0.0, W["bs"], self.terrain)
            ys, xs = np.nonzero(q["valid"])
            q["centre"] = (q["x0"] + (xs.min() + xs.max()) / 2 * q["res"], q["y0"] - (ys.min() + ys.max()) / 2 * q["res"])
            self.cache = {f: q}
        return self.cache[f]


def rectified_crop(q, out_px, half_m=CROP_M / 2):
    """Rectified photo cropped +-half_m around the footprint centre (offsets from the nadir), resampled to out_px.
    Returns (gray uint8, valid bool, nadir display pixel)."""
    cx, cy = q["centre"]
    s = out_px / (2 * half_m)
    a = 1 / (s * q["res"])
    M = np.array([[a, 0, (cx - half_m - q["x0"]) / q["res"] + 0.5 * a],
                  [0, a, (q["y0"] - cy - half_m) / q["res"] + 0.5 * a]], np.float32)
    g = cv2.warpAffine(q["patch"], M, (out_px, out_px), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    v = cv2.warpAffine(q["valid"].astype(np.uint8) * 255, M, (out_px, out_px),
                       flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP) > 0
    return g, v, (int(round((half_m - cx) * s)), int(round((half_m + cy) * s)))


def checker(cam_g, cam_v, map_bgr, nadir, sq=SQ):
    n = cam_g.shape[0]
    yy, xx = np.mgrid[0:n, 0:n]
    use_cam = (((yy // sq) + (xx // sq)) % 2 == 0) & cam_v
    out = map_bgr.copy()
    out[use_cam] = np.repeat(cam_g[use_cam][:, None], 3, 1)
    cnt, _ = cv2.findContours(cam_v.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, cnt, -1, (0, 255, 255), 1)
    cv2.drawMarker(out, nadir, (0, 255, 255), cv2.MARKER_CROSS, 14, 1)
    return out


def checker_at(Q: Queries, cm: ColourMap, f, x, y, out_px):
    """Checkerboard of photo f placed at EPSG:3826 (x, y), map shifted by the pre-cut map offset."""
    if not np.isfinite([x, y]).all():
        img = np.full((out_px, out_px, 3), 30, np.uint8)
        put(img, "no estimate", (10, out_px // 2), 0.7, C_GREY)
        return img
    q = Q.get(f)
    g, v, nadir = rectified_crop(q, out_px)
    cx, cy = q["centre"]
    return checker(g, v, cm.crop(x + Q.off[0] + cx, y + Q.off[1] + cy, CROP_M / 2, out_px), nadir,
                   SQ if out_px > 400 else SQ_SMALL)


def put(img, text, org, scale=0.6, colour=(255, 255, 255), thick=1):
    cv2.putText(img, text, (org[0] + 1, org[1] + 1), FONT, scale, (0, 0, 0), thick, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, colour, thick, cv2.LINE_AA)


def put_multi(img, parts, org, scale=0.6, thick=1, gap=14):
    """Draw [(text, colour), ...] left to right."""
    x, y = org
    for text, colour in parts:
        put(img, text, (x, y), scale, colour, thick)
        x += cv2.getTextSize(text, FONT, scale, thick)[0][0] + gap


# ----------------------------------------------------------------------------- overview (s3, adapted)

class Overview:
    def __init__(self, T, cm: ColourMap, off, w=700, h=800):
        self.off = off
        e = np.r_[T.true_x, T.d_x.dropna(), T.o_x.dropna()]
        n = np.r_[T.true_y, T.d_y.dropna(), T.o_y.dropna()]
        e0, e1, n0, n1 = e.min() - 60, e.max() + 60, n.min() - 60, n.max() + 60
        self.s = min(w / (e1 - e0), h / (n1 - n0))           # px per metre
        self.w, self.h = int((e1 - e0) * self.s), int((n1 - n0) * self.s)
        self.e0, self.n1 = e0, n1
        xc, yc = (e0 + e1) / 2 + off[0], (n0 + n1) / 2 + off[1]
        half = max(e1 - e0, n1 - n0) / 2
        big = cm.crop(xc, yc, half, int(round(2 * half * self.s)), cv2.INTER_AREA)
        oy, ox = (big.shape[0] - self.h) // 2, (big.shape[1] - self.w) // 2
        base = (big[oy:oy + self.h, ox:ox + self.w] * 0.7).astype(np.uint8)
        pre = T[~T.after_cut]
        over = base.copy()
        cv2.polylines(over, [self.px(pre.true_x, pre.true_y)], False, C_GNSS, 22, cv2.LINE_AA)
        self.base = cv2.addWeighted(over, 0.45, base, 0.55, 0)
        p = self.px([pre.true_x.iloc[0]], [pre.true_y.iloc[0]])[0]
        put(self.base, f"GNSS ON (first {mmss(T.t_s[T.after_cut].iloc[0])})", (int(p[0]) + 14, int(p[1]) + 24),
            0.5, C_GNSS)
        L = int(200 * self.s)
        cv2.line(self.base, (15, self.h - 18), (15 + L, self.h - 18), (255, 255, 255), 3)
        put(self.base, "200 m", (15, self.h - 26), 0.5)
        put(self.base, "N ^", (self.w - 50, 25), 0.6)

    def px(self, e, n):
        e, n = np.asarray(e, float), np.asarray(n, float)
        return np.stack([(e - self.e0) * self.s, (self.n1 - n) * self.s], -1).round().astype(np.int32)

    def draw(self, T, f):
        T = T.loc[:f]
        img = self.base.copy()
        r = T.iloc[-1]
        fx = T[T.after_cut & (T.fix_status != "")]
        bad = fx[fx.fix_status != "accepted"]
        for p in self.px(bad.o_x, bad.o_y):
            cv2.drawMarker(img, tuple(int(v) for v in p), C_GREY, cv2.MARKER_TILTED_CROSS, 7, 1)
        acc = fx[fx.fix_status == "accepted"]
        for p in self.px(acc.z_x, acc.z_y):
            cv2.circle(img, tuple(int(v) for v in p), 3, C_OURS, -1)
        for (ce, cn), c, th in ((("d_x", "d_y"), C_DR, 2), (("true_x", "true_y"), C_TRUTH, 2),
                                (("o_x", "o_y"), C_OURS, 2)):
            ok = T[ce].notna()
            if ok.sum() > 1:
                cv2.polylines(img, [self.px(T[ce][ok], T[cn][ok])], False, c, th, cv2.LINE_AA)
        for ce, cn, c in (("d_x", "d_y", C_DR), ("true_x", "true_y", C_TRUTH), ("m_x", "m_y", C_OURS)):
            if np.isfinite([r[ce], r[cn]]).all():
                p = tuple(int(v) for v in self.px([r[ce]], [r[cn]])[0])
                cv2.circle(img, p, 6, c, 2, cv2.LINE_AA)
        return img


# ----------------------------------------------------------------------------- chart: ours vs dead reckoning

class Chart:
    """Drawn once with matplotlib; per frame the plot area right of the current time comes from an empty copy."""

    def __init__(self, T: pd.DataFrame, w: int, h: int):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = T.t_s.to_numpy()
        t_cut = float(T.t_s[T.after_cut].iloc[0])
        last_pre = float(T.t_s[~T.after_cut].iloc[-1])
        tmax = float(t[-1]) + 5
        ymax = 60.0
        bgc = np.array([22, 22, 22]) / 255
        rgb = {k: np.array(c[::-1]) / 255 for k, c in (("o", C_OURS), ("d", C_DR))}
        imgs = []
        for full in (False, True):
            fig = plt.figure(figsize=(w / 100, h / 100), dpi=100, facecolor=bgc)
            ax = fig.add_axes([0.11, 0.09, 0.86, 0.72], facecolor=bgc)
            ax.set_xlim(0, tmax)
            ax.set_ylim(0, ymax)
            ax.tick_params(colors="0.8", labelsize=9)
            for s in ax.spines.values():
                s.set_color("0.4")
            ticks = np.arange(0, tmax, 120)
            ax.set_xticks(ticks, [mmss(v) for v in ticks])
            ax.set_ylabel("horizontal error (m)", color="0.85", fontsize=9)
            ax.grid(color="0.25", lw=0.6)
            ax.axvline(t_cut, color=np.array(C_GNSS[::-1]) / 255, lw=1.2)
            ax.text(t_cut + 4, ymax * 0.93, "GNSS cut", color=np.array(C_GNSS[::-1]) / 255, fontsize=8, va="top")
            handles = [ax.plot([], [], color=rgb["o"], lw=2, label="with map fixes (ours)")[0],
                       ax.plot([], [], color=rgb["d"], lw=1.4,
                               label=f"without map fixes (camera motion only; cut at {ymax:.0f} m, "
                                     f"max {T.err_d.max():.0f} m)")[0],
                       ax.fill_between([], [], color="0.5", alpha=0.35, label=f"no accepted fix > {GAP_S:.0f} s")]
            fig.text(0.02, 0.965, "Position error vs time: with vs without map fixes", color="white", fontsize=11,
                     weight="bold", va="center")
            fig.legend(handles=handles, loc="center", ncol=1, frameon=False, fontsize=8.5, labelcolor="0.9",
                       bbox_to_anchor=(0.5, 0.88), handlelength=1.6, labelspacing=0.3)
            if full:
                for a, b in no_fix_spans(t[T.after_cut], (T.fix_status[T.after_cut] == "accepted").to_numpy(),
                                         last_pre):
                    ax.axvspan(a, b, color="0.5", alpha=0.35, lw=0)
                ax.plot(t, np.clip(T.err_d, 0, ymax - 0.5), color=rgb["d"], lw=1.4)
                ax.plot(t, T.err_o, color=rgb["o"], lw=2)
            fig.canvas.draw()
            imgs.append(np.ascontiguousarray(np.asarray(fig.canvas.buffer_rgba())[..., 2::-1]))
            if full:
                x0, x1 = ax.transData.transform([(0, 0), (tmax, 0)])[:, 0]
                bb = ax.get_window_extent()
                self.ax_x0 = int(bb.x0)
                self.ax_y0, self.ax_y1 = h - int(math.ceil(bb.y1)), h - int(bb.y0)
                self.px = lambda tt, x0=x0, x1=x1: x0 + (x1 - x0) * tt / tmax
            plt.close(fig)
        self.empty, self.full = imgs

    def at(self, t: float) -> np.ndarray:
        out = self.empty.copy()
        xc = int(round(self.px(t)))
        out[:, : max(xc, self.ax_x0)] = self.full[:, : max(xc, self.ax_x0)]
        out[: self.ax_y0] = self.full[: self.ax_y0]
        cv2.line(out, (xc, self.ax_y0), (xc, self.ax_y1), (255, 255, 255), 1)
        return out


# ----------------------------------------------------------------------------- frame

def fmt_err(v):
    return "  -  " if not np.isfinite(v) else (f"{v:.1f} m" if v < 100 else f"{v:.0f} m")


def status_text(s: str, fix_err: float, corr: float = float("nan")) -> tuple[str, tuple]:
    """corr = distance between the filter's predicted and updated position at this photo (pred -> post)."""
    if s == "accepted":
        return f"map fix ACCEPTED (error vs truth {fix_err:.1f} m), correction {corr:.1f} m", C_OURS
    if s == "gated":
        return "map fix REJECTED by the 99 % gate", C_GREY
    if s == "disagree":
        return "no map fix: ZNCC and XFeat disagree (> 4 m)", C_GREY
    if s == "nofix":
        return "no map fix: ZNCC and XFeat did not both match", C_GREY
    return "", C_GREY


def readout(T: pd.DataFrame, f: int) -> dict:
    """Every number drawn for photo f (also written next to the stills for `check`)."""
    r = T.loc[f]
    upto = T.loc[:f]
    acc = upto.after_cut & (upto.fix_status == "accepted")
    cut = mmss(T.t_s[T.after_cut].iloc[0])
    if r.after_cut:
        st = f"GNSS OFF - {r.dist_since_cut:6.0f} m flown since the GNSS cut"
    else:
        st = f"GNSS ON (calibration, first {cut}) - GNSS cut in {-r.dist_since_cut:4.0f} m"
    return dict(frame=int(f), t=f"t = {r.t_s:6.1f} s", photo=f"photo {f} / {len(T)}", gnss=st,
                err_ours=fmt_err(r.err_o if r.after_cut else np.nan), err_dr=fmt_err(r.err_d),
                status=status_text(r.fix_status, r.fix_err, r.corr_m)[0] if r.after_cut else "",
                accepted=int(acc.sum()), attempts=int(upto.after_cut.sum()),
                wrong=int((acc & (upto.fix_err > WRONG_M)).sum()))


class Renderer:
    def __init__(self, T: pd.DataFrame):
        self.T = T
        self.cm = ColourMap()
        self.Q = Queries(T)
        self.ov = Overview(T, self.cm, self.Q.off)
        self.chart = Chart(T, 560, 490)
        self.paths = pd.read_csv(REAL / "images.csv").path.tolist()

    def photo(self, f: int, w: int, h: int) -> np.ndarray:
        exp = OUT / "images" / f"real_{f:04d}.jpg"          # 1280 px copy from `export`, else the original
        img = cv2.imread(str(exp)) if exp.exists() else cv2.imread(str(REAL / self.paths[f - 1]),
                                                                   cv2.IMREAD_REDUCED_COLOR_4)
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)

    def render(self, f: int) -> tuple[np.ndarray, dict]:
        T, r = self.T, self.T.loc[f]
        ro = readout(T, f)
        F = np.full((H, W, 3), 22, np.uint8)
        # header
        put(F, f"REAL flight (Tuniu, Taiwan, DJI P4 RTK, April 2019) - map Dec 2019 - seed {SEED}", (12, 28), 0.72,
            C_HEAD, 2)
        put(F, f"{ro['t']}   {ro['photo']}   {ro['gnss']}", (12, 60), 0.68)
        if r.after_cut:
            put(F, ro["status"], (1010, 60), 0.62, status_text(r.fix_status, r.fix_err, r.corr_m)[1])
        put(F, "horizontal error now:", (12, 92), 0.6, (200, 200, 200))
        put_multi(F, [(f"with map fixes (ours) {ro['err_ours']}", C_OURS),
                      (f"without map fixes (camera motion only) {ro['err_dr']}", C_DR)], (215, 92), 0.6, 1, 22)
        if r.after_cut:
            put(F, f"fixes accepted so far {ro['accepted']} / {ro['attempts']}   wrong fixes (> {WRONG_M:.0f} m) "
                   f"{ro['wrong']}", (1290, 92), 0.6)
        cv2.line(F, (0, 104), (W, 104), (90, 90, 90), 1)
        # left: real photo, error chart
        put(F, "1. What the drone sees (real photo)", (12, 124), 0.55)
        F[132:505, 12:572] = self.photo(f, 560, 373)
        c = self.chart.at(float(r.t_s))
        F[525:525 + c.shape[0], 12:12 + c.shape[1]] = c
        # middle: checkerboards
        who = "OUR estimate (GNSS OFF)" if r.after_cut else "the RTK position (GNSS ON)"
        put(F, f"2. Real photo placed on the Dec 2019 map at {who}", (590, 124), 0.55, C_OURS)
        F[132:712, 590:1170] = checker_at(self.Q, self.cm, f, r.m_x, r.m_y, 580)
        if r.after_cut:
            put(F, f"without map fixes ({fmt_err(r.err_d)} off)", (590, 728), 0.5, C_DR)
            F[734:1024, 590:880] = checker_at(self.Q, self.cm, f, r.d_x, r.d_y, 290)
        else:
            put(F, "without map fixes: starts at the GNSS cut", (590, 760), 0.5, C_GREY)
        lines = ["How to read the middle panel:",
                 " grey squares = real photo,",
                 "   rectified north-up",
                 " colour squares = Dec 2019 map,",
                 "   cut out at the estimate",
                 " roads, field edges continue",
                 "   across squares -> estimate right",
                 " lines jump at square borders",
                 "   -> wrong (jump = error)",
                 f" squares {SQ * CROP_M / 580:.0f} m (big), {SQ_SMALL * CROP_M / 290:.0f} m (small)",
                 " yellow + = estimated position,",
                 "   yellow line = photo footprint"]
        for j, t in enumerate(lines):
            put(F, t, (895, 748 + 23 * j), 0.47, (255, 255, 255) if not j else (230, 230, 230))
        # right: overview
        o = self.ov.draw(T, f)
        y0, x0 = 132, 1200 + (710 - o.shape[1]) // 2
        F[y0:y0 + o.shape[0], x0:x0 + o.shape[1]] = o
        put(F, "3. Tracks so far on the Dec 2019 map", (1200, 124), 0.55)
        yl = y0 + o.shape[0] + 22
        for j, (t, c) in enumerate([
                ("white = true path (drone's own RTK log),", C_TRUTH),
                ("   never given to the system after the cut", C_TRUTH),
                ("blue = with map fixes (ours): o accepted map fix, x rejected / no fix", C_OURS),
                ("red = same drone, same photos, map fixes switched off:", C_DR),
                ("   it only adds up the motion between photos", C_DR),
                ("a jump of the blue line = an accepted map fix correcting the drift", C_OURS),
                ("   built up since the previous fix (the drone itself flies smoothly: white)", C_OURS),
                ("yellow band = GNSS ON (calibration); circles = positions at this photo", (200, 200, 200))]):
            put(F, t, (1200, yl + 24 * j), 0.5, c)
        # footer
        put(F, PROVENANCE, (12, 1036), 0.4, (235, 235, 235))
        put(F, HONEST, (12, 1051), 0.4, C_HEAD)
        put(F, "Rectification = the navigation's (x5 make_query): DJI gimbal attitude + pre-cut boresight, SIMULATED "
               "barometer (seed 0), Copernicus terrain. Map placed with the pre-cut stage-1 map offset.",
            (12, 1065), 0.4, (200, 200, 200))
        put(F, f"Sources (seed {SEED}): {rel(RUN_LOOP)} (with map fixes), {rel(RUN_DR)} (without map fixes). "
               "Truth is used here only to print errors and the distance flown.", (12, 1078), 0.4, (200, 200, 200))
        return F, ro


def title_card(T: pd.DataFrame) -> np.ndarray:
    """First 3 s: where the data come from (docs/research/tuniu-team-explainer.md, section 2)."""
    cut = float(T.t_s[T.after_cut].iloc[0])
    km = float(T.dist_since_cut.iloc[-1]) / 1000
    F = np.full((H, W, 3), 22, np.uint8)
    put(F, "No drone of our own, so we used a real, published flight:", (120, 250), 1.2, C_HEAD, 2)
    lines = [("DJI Phantom 4 RTK, Tuniu River, Miaoli, Taiwan, 11 April 2019", (255, 255, 255)),
             (f"{len(T)} photos + centimetre RTK log, shared by Yu-Huang Wang on the OpenDroneMap community forum",
              (255, 255, 255)),
             ("Navigation map: a different flight, OpenAerialMap 12 Dec 2019 (CC BY 4.0)", (255, 255, 255)),
             (f"GNSS cut after {int(cut // 60)} min {int(cut % 60):02d} s; {km:.0f} km on photos only; "
              "the RTK log is used only to score", C_OURS)]
    for j, (t, c) in enumerate(lines):
        put(F, t, (160, 350 + 70 * j), 0.95, c, 2)
    put(F, PROVENANCE, (120, 900), 0.6, (200, 200, 200))
    return F


def summary_card(n_test: int) -> np.ndarray:
    """End card, every number read from the summary CSVs."""
    p = pd.read_csv(SUMMARY_L2_POOLED)
    sel = (p.heading == "dji") & (p.ground == "dem_lifted")
    a, d = p[sel & (p["mode"] == "loop")].iloc[0], p[sel & (p["mode"] == "dr")].iloc[0]
    tw = pd.read_csv(SUMMARY_L2)
    tw = tw[(tw["mode"] == "loop") & (tw.seeds == 5)].set_index("variant")
    real, sim = tw.loc["real"], tw.loc["twin_rc"]
    F = np.full((H, W, 3), 22, np.uint8)
    put(F, f"This flight, {int(a.seeds)} seeds (seed = simulated barometer noise), {n_test} photos after the GNSS cut",
        (120, 150), 1.0, C_HEAD, 2)
    put(F, "with map fixes (ours)", (820, 225), 0.9, C_OURS, 2)
    put(F, "without map fixes", (1330, 225), 0.9, C_DR, 2)
    rows = [("median horizontal error", f"{a.err_median_m:.2f} m", f"{d.err_median_m:.1f} m"),
            ("90 % of photos below", f"{a.err_p90_m:.1f} m", f"{d.err_p90_m:.1f} m"),
            (f"wrong fixes accepted (> {WRONG_M:.0f} m)", f"{a.accepted_wrong_gt10_total:.0f} of "
             f"{a.accepted_total:.0f} accepted", "-"),
            ("runs that never lost the lock", f"{a.seeds_without_lol:.0f} / {a.seeds:.0f}",
             f"{d.seeds_without_lol:.0f} / {d.seeds:.0f}")]
    y = 290
    for name, va, vb in rows:
        put(F, name, (160, y), 0.85, (235, 235, 235))
        put(F, va, (820, y), 0.9, C_OURS, 2)
        put(F, vb, (1330, y), 0.9, C_DR, 2)
        y += 58
    put(F, "Same flight replayed in a digital twin, same code:", (120, 600), 1.0, C_HEAD, 2)
    put(F, f"median {sim.err_median_m:.2f} m in the twin vs {real.err_median_m:.2f} m with the real photos "
           f"({int(real.seeds)} seeds)", (160, 660), 0.9, (235, 235, 235), 2)
    put(F, f"criteria fixed before the runs: {PREREG}", (160, 715), 0.8, (220, 220, 220))
    put(F, HONEST, (120, 850), 0.7, C_HEAD)
    put(F, PROVENANCE, (120, 880), 0.6, (220, 220, 220))
    put(F, f"Sources: {rel(SUMMARY_L2_POOLED)} (loop and dr, dji, dem_lifted); {rel(SUMMARY_L2)} (real, twin_rc, "
           "loop, 5 seeds)", (120, 910), 0.6, (180, 180, 180))
    return F


# ----------------------------------------------------------------------------- commands

def cmd_video(args) -> None:
    T = load_table()
    t0 = time.perf_counter()
    R = Renderer(T)
    print(f"setup {time.perf_counter() - t0:.1f} s", flush=True)
    if args.stills:
        shots = OUT / "shots"
        shots.mkdir(parents=True, exist_ok=True)
        for f in args.stills:
            img, ro = R.render(f)
            cv2.imwrite(str(shots / f"still_{f:04d}.png"), img)
            (shots / f"still_{f:04d}.json").write_text(json.dumps(ro, indent=1))
            print("still", shots / f"still_{f:04d}.png", ro, flush=True)
        cv2.imwrite(str(shots / "end_card.png"), summary_card(int(T.after_cut.sum())))
        cv2.imwrite(str(shots / "title_card.png"), title_card(T))
        return
    start, end = args.start or 1, args.end or len(T)
    out = OUT / (args.out or "jury_replay.mp4")
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [FFMPEG, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}",
           "-r", str(args.fps), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "20",
           "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)]
    ff = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    t1 = time.perf_counter()
    n = 0
    if not args.no_card:
        card = title_card(T)
        for _ in range(int(round(TITLE_S * args.fps))):
            ff.stdin.write(card.tobytes())
        n += int(round(TITLE_S * args.fps))
    for f in range(start, end + 1):
        img, _ = R.render(f)
        ff.stdin.write(img.tobytes())
        if f % 25 == 0:
            print(f"  photo {f}  {time.perf_counter() - t1:.0f} s", flush=True)
    n += end - start + 1
    if not args.no_card:
        card = summary_card(int(T.after_cut.sum()))
        for _ in range(int(round(CARD_S * args.fps))):
            ff.stdin.write(card.tobytes())
        n += int(round(CARD_S * args.fps))
    ff.stdin.close()
    if ff.wait() != 0:
        raise SystemExit("ffmpeg failed")
    print(f"wrote {out}: {n} frames, {n / args.fps:.1f} s at {args.fps} fps, {out.stat().st_size / 1e6:.1f} MB, "
          f"render {time.perf_counter() - t1:.0f} s", flush=True)


def longest_gap(T: pd.DataFrame) -> tuple[int, int]:
    """First and last photo of the longest run of test photos without an accepted fix."""
    a = T[T.after_cut]
    acc = (a.fix_status == "accepted").to_numpy()
    best, i = (0, -1), 0
    while i < len(a):
        if acc[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(a) and not acc[j + 1]:
            j += 1
        if j - i > best[1] - best[0]:
            best = (i, j)
        i = j + 1
    return int(a.frame.iloc[best[0]]), int(a.frame.iloc[best[1]])


def cmd_clip(args) -> None:
    """Forest-gap highlight cut from the main video: the longest span without an accepted fix and its recovery."""
    T = load_table()
    a, b = longest_gap(T)
    s, e = max(1, a - args.before), min(len(T), b + args.after)
    print(f"longest span without an accepted fix: photos {a}-{b} "
          f"({T.t_s[b + 1] - T.t_s[a - 1]:.1f} s between accepted fixes); clip photos {s}-{e}")
    src, out = OUT / "jury_replay.mp4", OUT / "jury_forest_gap.mp4"
    t_start = round(TITLE_S * args.fps) / args.fps + (s - 1) / args.fps        # main video opens with the title card
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-ss", f"{t_start:.4f}", "-i", str(src),
                    "-t", f"{(e - s + 1) / args.fps:.4f}", "-c:v", "libx264", "-preset", "medium", "-crf", "20",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)], check=True)
    print(f"wrote {out} ({(e - s + 1) / args.fps:.1f} s, {out.stat().st_size / 1e6:.1f} MB)")


def cmd_check(args) -> None:
    """Numbers drawn on the stills vs the run CSV rows, read again here from the CSV files."""
    L = pd.read_csv(RUN_LOOP).set_index("frame")
    D = pd.read_csv(RUN_DR).set_index("frame")
    bad = 0
    for p in sorted((OUT / "shots").glob("still_*.json")):
        ro = json.loads(p.read_text())
        f = ro["frame"]
        want = {}
        if f in L.index:
            upto = L.loc[:f]
            acc = upto.fix_status == "accepted"
            want = dict(t=f"t = {L.t_s[f]:6.1f} s", err_ours=fmt_err(L.err_m[f]), err_dr=fmt_err(D.err_m[f]),
                        status=status_text(L.fix_status[f], L.fix_err_m[f],
                                           math.hypot(L.post_x[f] - L.pred_x[f], L.post_y[f] - L.pred_y[f]))[0],
                        accepted=int(acc.sum()),
                        attempts=len(upto), wrong=int((acc & (upto.fix_err_m > WRONG_M)).sum()))
        diff = {k: (ro[k], v) for k, v in want.items() if ro[k] != v}
        bad += bool(diff)
        print(f"photo {f}: {'OK' if not diff else 'MISMATCH ' + str(diff)} | " + " | ".join(f"{k}: {ro[k]}" for k in want))
        if f in L.index:
            print(f"   CSV row: t_s={L.t_s[f]:.3f} err_m={L.err_m[f]:.3f} (dr {D.err_m[f]:.3f}) "
                  f"fix_status={L.fix_status[f]} fix_err_m={L.fix_err_m[f]}")
    if bad:
        raise SystemExit(f"{bad} still(s) disagree with the CSV")


# ----------------------------------------------------------------------------- export (model frame, for 3D views)

def model_frame(src: Path):
    """EPSG:3826 / lon-lat -> 3D model frame: UTM 51N minus coords.txt offset (x8_odm_viewer.offset()) plus the
    x9 twin placement offset (constant, so errors are unchanged)."""
    from pyproj import Transformer
    X8.SRC, X8.ODM = src, src.parent
    e0, n0 = X8.offset()
    pe, pn = json.loads(TWIN_CAL.read_text())["camera_offset_en_m"]
    t3826 = Transformer.from_crs("EPSG:3826", "EPSG:32651", always_xy=True)
    t4326 = Transformer.from_crs("EPSG:4326", "EPSG:32651", always_xy=True)

    def from_3826(x, y):
        e, n = t3826.transform(np.asarray(x, float), np.asarray(y, float))
        return np.asarray(e) - e0 + pe, np.asarray(n) - n0 + pn

    def from_lonlat(lon, lat):
        e, n = t4326.transform(np.asarray(lon, float), np.asarray(lat, float))
        return np.asarray(e) - e0 + pe, np.asarray(n) - n0 + pn
    return from_3826, from_lonlat, dict(utm51n_offset_en_m=[e0, n0], placement_en_m=[pe, pn])


def cmd_export(args) -> None:
    from PIL import Image
    src = Path(args.src)
    from_3826, from_lonlat, frame_info = model_frame(src)
    T = load_table()
    tr = pd.read_csv(REAL / "truth.csv")
    im, tw = pd.read_csv(REAL / "images.csv"), pd.read_csv(TWIN / "images.csv")
    L = pd.read_csv(RUN_LOOP).set_index("frame")
    assert len(im) == len(tw) == len(tr) == len(T) and np.allclose(im.t_s, tr.t_s) and np.allclose(im.t_s, tw.t_s)
    te, tn = from_lonlat(tr.lon_deg, tr.lat_deg)
    conv = {k: from_3826(T[f"{k}_x"], T[f"{k}_y"]) for k in ("o", "d", "s", "z")}
    sd = L[["post_sd_e", "post_sd_n"]].reindex(T.index)

    def num(v):
        return None if v is None or not math.isfinite(float(v)) else round(float(v), 3)

    (OUT / "images").mkdir(parents=True, exist_ok=True)
    frames = []
    for i, f in enumerate(T.frame):
        r = T.loc[f]
        rec = dict(frame=int(f), t_s=round(float(r.t_s), 3), phase="test" if r.after_cut else "gps_on_calibration",
                   truth_enu=[num(te[i]), num(tn[i]), num(tr.alt_m[i])],
                   real_jpg=f"images/real_{f:04d}.jpg", twin_jpg=f"images/twin_{f:04d}.jpg")
        if r.after_cut:
            rec.update(ours_en=[num(conv["o"][0][i]), num(conv["o"][1][i])],
                       ours_sd_en=[num(sd.post_sd_e[f]), num(sd.post_sd_n[f])],
                       dr_en=[num(conv["d"][0][i]), num(conv["d"][1][i])],
                       twin_en=[num(conv["s"][0][i]), num(conv["s"][1][i])],
                       err_ours_m=num(r.err_o), err_dr_m=num(r.err_d), err_twin_m=num(r.err_s),
                       fix_status=r.fix_status, fix_en=[num(conv["z"][0][i]), num(conv["z"][1][i])],
                       fix_err_m=num(r.fix_err), lol=bool(L.lol[f]), twin_fix_status=r.s_fix_status)
        frames.append(rec)
        for s, dst in ((REAL / im.path[i], OUT / rec["real_jpg"]), (TWIN / tw.path[i], OUT / rec["twin_jpg"])):
            if dst.exists():
                continue
            with Image.open(s) as img:
                img.draft("RGB", (EXPORT_W, EXPORT_W))           # JPEG decode at 1/2^k size: fast
                img = img.convert("RGB")
                img.resize((EXPORT_W, round(img.height * EXPORT_W / img.width)), Image.Resampling.LANCZOS).save(
                    dst, quality=EXPORT_Q)
    t = T.t_s.to_numpy()
    meta = dict(sequence="tuniu_tw_1", frames=len(T), first_test_frame=int(T.frame[T.after_cut].iloc[0]),
                model_src=rel(src), frame="UTM 51N minus coords.txt offset plus twin placement; z = ellipsoidal "
                "height (m); estimates are 2D", **frame_info,
                runs=dict(ours=rel(RUN_LOOP), dead_reckoning=rel(RUN_DR), twin=rel(RUN_SIM)), wrong_fix_m=WRONG_M,
                no_fix_spans_s=no_fix_spans(t[T.after_cut], (T.fix_status[T.after_cut] == "accepted").to_numpy(),
                                            float(t[~T.after_cut][-1])),
                images=f"{EXPORT_W} px wide JPEG q{EXPORT_Q}")
    (OUT / "data.json").write_text(json.dumps(dict(meta=meta, frames=frames), indent=1))
    print("wrote", OUT / "data.json", "and", OUT / "images")


# ----------------------------------------------------------------------------- 3D fly-through (optional clip)

def prepare_mesh(src: Path, tex_px: int) -> Path:
    """x8_odm_viewer.cmd_prepare into our own folder: textures downscaled to tex_px JPEG, geometry unchanged."""
    X8.SRC, X8.ODM = src, src.parent
    X8.OUT = OUT / f"mesh_{src.parent.name}_{src.name}_{tex_px}"
    X8.cmd_prepare(argparse.Namespace(tex_px=tex_px))
    return X8.OUT


def surroundings(src: Path, model_pts: np.ndarray, spacing: float, margin: float, lower: float):
    """Ground around and under the OpenDroneMap model: Copernicus terrain (x_tuniu_geo.dem_ellipsoidal, EPSG:3826
    grid every `spacing` m) over the April flight + margin, draped with the Dec 2019 map 'main' (map = true + pre-cut
    stage-1 offset) where the map has data, and with a plain grey hillshade of the same terrain where it has none.
    Under the model footprint (dilated 2 cells, so holes are covered too) it is kept `lower` m below the lowest
    model vertex nearby, so the model always wins, then ramps back to the terrain over RAMP_M m. No geometry is
    invented: holes in the model show this layer."""
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.windows import from_bounds
    from scipy.ndimage import distance_transform_edt, minimum_filter
    from_3826, _, _ = model_frame(src)
    tr = G.truth_xy()
    off = np.array(X5.map_offset("main", "stage1", None))
    x0, x1, y0, y1 = tr.x.min() - margin, tr.x.max() + margin, tr.y.min() - margin, tr.y.max() + margin
    X, Y = np.meshgrid(np.arange(x0, x1 + 1e-6, spacing), np.arange(y0, y1 + 1e-6, spacing))
    x1, y1 = X[0, -1], Y[-1, 0]
    Z = G.dem_ellipsoidal(X, Y).astype(float)
    tres = max(X5.RES, (x1 - x0) / 4096, (y1 - y0) / 4096)       # texture m/px, at most 4096 px
    tw, th = int(round((x1 - x0) / tres)), int(round((y1 - y0) / tres))
    with rasterio.open(G.map_path("main", X5.RES)) as ds:
        win = from_bounds(x0 + off[0], y0 + off[1], x1 + off[0], y1 + off[1], ds.transform)
        a = ds.read([1, 2, 3, 4], window=win, out_shape=(4, th, tw), boundless=True, fill_value=0,
                    resampling=Resampling.bilinear)
    rgb = a[:3].transpose(1, 2, 0)[..., ::-1]
    valid = (a[3] > 200) & ~(a[:3] > 245).all(0) & ~(a[:3] < 8).all(0)      # white / black = no data
    gy, gx = np.gradient(Z, spacing)                               # rows = south -> north
    shade = np.clip(0.55 + 0.45 * (-gx * 0.6 - gy * 0.6 + 1) / np.sqrt(1 + gx ** 2 + gy ** 2), 0.3, 1.0)
    shade = cv2.resize(np.flipud(shade).astype(np.float32), (tw, th), interpolation=cv2.INTER_LINEAR)
    hill = (shade[..., None] * np.array([80, 92, 88], np.float32)).astype(np.uint8)   # BGR grey (rendered lighter)
    tex = np.where(valid[..., None], rgb, hill)
    tex_path = OUT / "surroundings_main_map.jpg"
    cv2.imwrite(str(tex_path), np.ascontiguousarray(tex), [cv2.IMWRITE_JPEG_QUALITY, 90])
    E, N = from_3826(X, Y)
    # lowest model vertex per spacing cell (model frame), dilated by 2 cells
    ex0, ny0 = E.min(), N.min()
    nx, ny = int((E.max() - ex0) / spacing) + 2, int((N.max() - ny0) / spacing) + 2
    lo = np.full((ny, nx), np.inf)
    ix = np.clip(((model_pts[:, 0] - ex0) / spacing).astype(int), 0, nx - 1)
    iy = np.clip(((model_pts[:, 1] - ny0) / spacing).astype(int), 0, ny - 1)
    np.minimum.at(lo, (iy, ix), model_pts[:, 2])
    lo = minimum_filter(lo, size=5, mode="constant", cval=np.inf)
    under = lo[((N - ny0) / spacing).astype(int), ((E - ex0) / spacing).astype(int)]
    foot = np.isfinite(under)
    Zc = Z
    Z = np.where(foot, np.minimum(Zc, under - lower), Zc)
    if foot.any():       # ramp from the model edge to the terrain over RAMP_M, so the two never meet in a wall
        d, (ri, ci) = distance_transform_edt(~foot, return_indices=True)
        edge = Z[ri, ci]
        Z = np.where(foot, Z, edge + (Zc - edge) * np.clip(d * spacing / RAMP_M, 0, 1))
    u, v = (X - x0) / (x1 - x0), (Y - y0) / (y1 - y0)
    # written as OBJ + MTL and loaded with the same importer as the model (pyvista add_mesh(texture=...) renders
    # it untextured once the OBJ importer is in the scene)
    r, c = X.shape
    idx = np.arange(r * c).reshape(r, c) + 1
    quads = np.stack([idx[:-1, :-1], idx[:-1, 1:], idx[1:, 1:], idx[1:, :-1]], -1).reshape(-1, 4)
    obj = OUT / "surroundings.obj"
    with open(obj, "w") as fh:
        fh.write("mtllib surroundings.mtl\nusemtl ground\n")
        np.savetxt(fh, np.c_[E.ravel(), N.ravel(), Z.ravel()], fmt="v %.3f %.3f %.3f")
        np.savetxt(fh, np.c_[u.ravel(), v.ravel()], fmt="vt %.6f %.6f")
        np.savetxt(fh, np.repeat(quads, 2, axis=1), fmt="f %d/%d %d/%d %d/%d %d/%d")
    (OUT / "surroundings.mtl").write_text(f"newmtl ground\nKa 1 1 1\nKd 1 1 1\nmap_Kd {tex_path.name}\n")
    print(f"surroundings: {X.shape[1]} x {X.shape[0]} vertices every {spacing:g} m, texture {tw} x {th} px "
          f"({valid.mean() * 100:.0f} % with map), {foot.mean() * 100:.0f} % of vertices under the model footprint",
          flush=True)
    return obj


class Fly3D:
    """Offscreen pyvista scene: OpenDroneMap model (x8_odm_viewer display copy) + surroundings + tracks."""

    def __init__(self, src: Path, tex_px: int, w: int, h: int, spacing: float, margin: float):
        import pyvista as pv
        from vtkmodules.util.numpy_support import vtk_to_numpy
        self.pv = pv
        mesh = prepare_mesh(src, tex_px)
        pl = pv.Plotter(off_screen=True, window_size=(w, h))
        pl.import_obj(str(mesh / X8.OBJ), str(mesh / X8.MTL))
        actors = pl.renderer.GetActors()
        actors.InitTraversal()
        pts = np.concatenate([vtk_to_numpy(actors.GetNextActor().GetMapper().GetInput().GetPoints().GetData())
                              for _ in range(actors.GetNumberOfItems())])
        obj = surroundings(src, pts, spacing, margin, 1.0)
        pl.import_obj(str(obj), str(obj.with_suffix(".mtl")))
        actors.InitTraversal()
        for _ in range(actors.GetNumberOfItems()):   # photo colours as they are, as x8_odm_viewer.cmd_view
            prop = actors.GetNextActor().GetProperty()
            prop.LightingOff()
            prop.SetAmbient(0.0)
            prop.SetDiffuse(1.0)
            prop.SetSpecular(0.0)
        pl.set_background("#c9dcef", top="#4f7fb5")          # sky gradient, as x8_odm_viewer
        pl.enable_anti_aliasing("msaa")
        pl.camera.view_angle = 35.0
        self.pl = pl

    def _line(self, name, p, colour, width):
        p = p[np.isfinite(p).all(1)]
        if len(p) < 2:
            self.pl.remove_actor(name)
            return
        self.pl.add_mesh(self.pv.lines_from_points(p), color=colour, line_width=width, name=name, lighting=False,
                         render_lines_as_tubes=True, reset_camera=False)

    def draw(self, tk: dict, u: float, target, dist: float, elev: float) -> np.ndarray:
        pv, pl = self.pv, self.pl
        i = min(int(u), len(tk["t"]) - 1)
        rgb = lambda c: tuple(v / 255 for v in c[::-1])  # noqa: E731
        head = lambda a: lerp(a, u)  # noqa: E731
        truth = np.vstack([tk["truth"][: i + 1], head(tk["truth"])[None]])
        self._line("truth", truth, rgb(C_TRUTH), 4)
        pl.add_mesh(pv.Sphere(radius=3.0, center=truth[-1]), color=rgb(C_TRUTH), name="drone", lighting=False,
                    reset_camera=False)
        for name, key, c, dz in (("ours", "ours", C_OURS, 0.6), ("dr", "dr", C_DR, 0.3)):
            p = np.column_stack([tk[key][: i + 1], tk["truth"][: i + 1, 2] + dz])
            h = np.r_[head(tk[key]), head(tk["truth"])[2] + dz]
            p = np.vstack([p, h[None]]) if np.isfinite(h).all() else p
            self._line(name, p, rgb(c), 4)
            if np.isfinite(p[-1]).all() and i >= tk["test0"]:
                pl.add_mesh(pv.Sphere(radius=2.5, center=p[-1]), color=rgb(c), name=name + "_head", lighting=False,
                            reset_camera=False)
        acc = tk["acc"][: i + 1]
        fx = np.column_stack([tk["fix"][: i + 1][acc], tk["truth"][: i + 1, 2][acc] + 0.9])
        if len(fx):
            pl.add_mesh(pv.PolyData(fx), color=rgb(C_OURS), point_size=10, render_points_as_spheres=True,
                        name="fixes", lighting=False, reset_camera=False)
        e = math.radians(elev)
        pl.camera_position = [(target[0], target[1] - dist * math.cos(e), target[2] + dist * math.sin(e)),
                              tuple(target), (0, 0, 1)]
        pl.render()
        return np.ascontiguousarray(np.asarray(pl.screenshot(return_img=True))[..., 2::-1])


def lerp(a: np.ndarray, u: float) -> np.ndarray:
    i = min(int(math.floor(u)), len(a) - 1)
    if i + 1 >= len(a) or u == i:
        return a[i]
    return a[i] * (1 - (u - i)) + a[i + 1] * (u - i)


def cmd_flythrough(args) -> None:
    """Optional 3D clip (default: the longest no-fix span and its recovery, as `clip`), tracks from data.json."""
    src = Path(args.src)
    p = OUT / "data.json"
    if not p.exists():
        raise SystemExit("run `export` first")
    d = json.loads(p.read_text())
    meta, fr = d["meta"], d["frames"]
    X8.SRC, X8.ODM = src, src.parent
    if not np.allclose(X8.offset(), meta["utm51n_offset_en_m"]):
        raise SystemExit(f"data.json was exported for another model frame: run `export --src {args.src}` first")

    def col(key, dim=2):
        return np.array([r[key] if r.get(key) is not None else [np.nan] * dim for r in fr], float)
    tk = dict(t=np.array([r["t_s"] for r in fr]), truth=col("truth_enu", 3), ours=col("ours_en"), dr=col("dr_en"),
              fix=col("fix_en"), acc=np.array([r.get("fix_status") == "accepted" for r in fr]),
              test0=meta["first_test_frame"] - 1)
    T = load_table()
    if args.start or args.end:
        s, e = args.start or 1, args.end or len(T)
    else:
        a, b = longest_gap(T)
        s, e = max(1, a - 16), min(len(T), b + 24)
    fpp = args.fps * args.photo_seconds
    n = int(round((e - s) * fpp)) + 1
    us = np.minimum(s - 1 + np.arange(n) / fpp, len(fr) - 1)
    # camera target: RTK track smoothed along the photo index (Gaussian, sigma 2.5 photos)
    ug = np.arange(0, len(fr) - 1 + 1e-9, 0.05)
    raw = np.stack([np.interp(ug, np.arange(len(fr)), tk["truth"][:, j]) for j in range(3)], 1)
    k = np.exp(-0.5 * (np.arange(-200, 201) / 50.0) ** 2)
    k /= k.sum()
    sm = np.stack([np.convolve(np.pad(raw[:, j], 200, mode="edge"), k, "valid") for j in range(3)], 1)
    t0 = time.perf_counter()
    vw, vh = W, H - 200
    fly = Fly3D(src, args.tex_px, vw, vh, args.spacing, args.margin)
    print(f"scene loaded in {time.perf_counter() - t0:.1f} s", flush=True)

    def frame(u):
        f = int(u) + 1
        r = T.loc[f]
        tgt = np.array([np.interp(u, ug, sm[:, j]) for j in range(3)])
        F = np.full((H, W, 3), 22, np.uint8)
        F[110:110 + vh] = fly.draw(tk, u, tgt, args.dist, args.elev)
        ro = readout(T, f)
        put(F, f"3D view of the REAL flight, seed {SEED} - model: OpenDroneMap, Sept 2019 survey ({rel(src)})",
            (12, 28), 0.72, C_HEAD, 2)
        put(F, f"{ro['t']}   {ro['photo']}   {ro['gnss']}", (12, 60), 0.68)
        put_multi(F, [(f"with map fixes (ours) {ro['err_ours']}", C_OURS),
                      (f"without map fixes (camera motion only) {ro['err_dr']}", C_DR),
                      (ro["status"], C_GREY)], (12, 92), 0.6, 1, 22)
        put_multi(F, [("white = true path (drone's own RTK log, never given to the system after the cut)", C_TRUTH),
                      ("blue = with map fixes (ours), dots = accepted fixes", C_OURS),
                      ("red = without map fixes (camera motion only)", C_DR)],
                  (12, 110 + vh + 16), 0.55, 1, 24)
        put(F, "a jump of the blue line = an accepted map fix correcting the drift built up since the previous fix "
               "(the drone itself flies smoothly: white)", (12, 110 + vh + 36), 0.5, C_OURS)
        put(F, "Outside the drone survey: aerial map Dec 2019 on 30 m terrain (plain grey = terrain only, no map "
               "there)", (12, H - 36), 0.55, C_HEAD)
        put(F, PROVENANCE, (12, H - 20), 0.4, (235, 235, 235))
        put(F, HONEST + f"  Tracks: {rel(RUN_LOOP)}, {rel(RUN_DR)}", (12, H - 6), 0.4, (200, 200, 200))
        return F

    if args.stills:
        (OUT / "shots").mkdir(parents=True, exist_ok=True)
        for f in args.stills:
            cv2.imwrite(str(OUT / "shots" / f"flythrough_{f:04d}.png"), frame(float(f - 1)))
        print("stills written", flush=True)
        return
    out = OUT / (args.out or "jury_3d_flythrough.mp4")
    ff = subprocess.Popen([FFMPEG, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}",
                           "-r", str(args.fps), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "26",
                           "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)], stdin=subprocess.PIPE)
    t1 = time.perf_counter()
    for j, u in enumerate(us):
        ff.stdin.write(frame(float(u)).tobytes())
        if j % 100 == 0:
            print(f"  frame {j}/{n}  {time.perf_counter() - t1:.0f} s", flush=True)
    ff.stdin.close()
    if ff.wait() != 0:
        raise SystemExit("ffmpeg failed")
    print(f"wrote {out}: photos {s}-{e}, {n} frames, {n / args.fps:.1f} s, {out.stat().st_size / 1e6:.1f} MB, "
          f"render {time.perf_counter() - t1:.0f} s", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export")
    e.add_argument("--src", default=str(DEFAULT_SRC), help="OpenDroneMap texturing folder (defines the model frame)")
    v = sub.add_parser("video")
    v.add_argument("--start", type=int, help="first photo (default 1)")
    v.add_argument("--end", type=int, help="last photo (default 271)")
    v.add_argument("--fps", type=float, default=FPS)
    v.add_argument("--out", help="file name in data/processed/x10_jury_replay (default jury_replay.mp4)")
    v.add_argument("--no-card", action="store_true", help="skip the title card and the end card")
    v.add_argument("--stills", type=int, nargs="+", help="only write PNG stills of these photos into shots/")
    c = sub.add_parser("clip")
    c.add_argument("--fps", type=float, default=FPS, help="fps of jury_replay.mp4")
    c.add_argument("--before", type=int, default=16, help="photos before the gap")
    c.add_argument("--after", type=int, default=24, help="photos after the gap (recovery)")
    sub.add_parser("check")
    y = sub.add_parser("flythrough", help="optional 3D clip (needs: uv run --with pyvista)")
    y.add_argument("--src", default=str(DEFAULT_SRC), help="OpenDroneMap texturing folder (OBJ + MTL + textures)")
    y.add_argument("--tex-px", type=int, default=2048, help="texture size of the display copy (2048-4096)")
    y.add_argument("--start", type=int, help="first photo (default: 16 before the longest no-fix span)")
    y.add_argument("--end", type=int, help="last photo (default: 24 after it)")
    y.add_argument("--fps", type=int, default=24)
    y.add_argument("--photo-seconds", type=float, default=0.4, help="video seconds per photo")
    y.add_argument("--dist", type=float, default=300.0, help="camera distance to the drone (m)")
    y.add_argument("--elev", type=float, default=58.0, help="camera elevation angle (deg)")
    y.add_argument("--spacing", type=float, default=5.0, help="surroundings grid spacing (m)")
    y.add_argument("--margin", type=float, default=300.0, help="surroundings margin around the flight (m)")
    y.add_argument("--stills", type=int, nargs="+", help="only write PNG stills of these photos into shots/")
    y.add_argument("--out", help="file name (default jury_3d_flythrough.mp4)")
    a = ap.parse_args()
    {"export": cmd_export, "video": cmd_video, "clip": cmd_clip, "check": cmd_check,
     "flythrough": cmd_flythrough}[a.cmd](a)


if __name__ == "__main__":
    main()

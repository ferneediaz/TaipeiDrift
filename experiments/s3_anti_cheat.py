#!/usr/bin/env python3
"""S3 anti-cheat runs: do OUR estimates (S1 EKF + consensus map fixes) come from the camera and the map, or from
the truth? SIMULATED flight recordings/ilhan_wufeng_south_80m, ideal camera, seed 0, s1_sim_map_fix.run_sequence
unchanged. Four runs, the same estimator inputs (s1_sim_map_fix.unit_for, built BEFORE the run by the parent:
pre-cut GNSS state, SIMULATED heading = truth + drift, SIMULATED barometer, pre-cut calibration):

  normal        as outputs/s1_sim/runs/ideal/loop_s0.csv
  truth_hidden  the recording copied to a temporary folder WITHOUT truth.csv (images, images.csv, meta.json, and
                ahrs.csv = roll/pitch only, the AHRS stand-in, heading removed). In the worker the recording loader
                is replaced by one that sets every truth position to NaN, and any attempt to read a file named
                truth.csv raises. Expected: post-cut estimates identical to `normal`.
  map_shift30E  the 2018 map georeference moved +30 m east AFTER the pre-cut calibration (same pixels). If the
                estimates come from the map, the fixes move east by ~30 m and the estimate follows once the
                99 % gate lets them in.
  wrong_map     the 2018 map pixels rolled by (+1000 m east, +1500 m north), same georeference: at every place
                the map shows ground from ~1.8 km away. Expected: no consensus, estimate drifts like dead reckoning.

Scoring against truth happens in the parent only (s1_sim_map_fix.score).
Outputs: outputs/evidence/anti_cheat/{<run>.csv, table.csv}, outputs/evidence/anti_cheat.png

  PY=/Users/ilhan.neuville/dev/hackathon/TaipeiDrift/.venv/bin/python
  $PY experiments/s3_anti_cheat.py run      # 4 runs, 2 workers, ~6 min
  $PY experiments/s3_anti_cheat.py figure
"""
from __future__ import annotations

import argparse
import builtins
import json
import os
import shutil
import sys
import tempfile
import time
from multiprocessing import get_context
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import s1_sim_map_fix as S1  # noqa: E402
import x_tuniu_geo as G  # noqa: E402

ROOT = S1.ROOT
OUT = ROOT / "outputs/evidence/anti_cheat"
SEED, CAMERA = 0, "ideal"
SHIFT_E_M = 30.0
ROLL_M = (1000.0, 1500.0)          # wrong map: content moved by (east, north)
RUNS = ("normal", "truth_hidden", "map_shift30E", "wrong_map")
HIDDEN = Path(tempfile.gettempdir()) / "s3_truth_hidden" / "ilhan_wufeng_south_80m"


# ----------------------------------------------------------------------------- inputs (parent)

def write_maps():
    import rasterio
    from rasterio.transform import Affine
    OUT.mkdir(parents=True, exist_ok=True)
    with rasterio.open(S1.MAP) as ds:
        prof, data, tr = ds.profile.copy(), ds.read(), ds.transform
    prof.update(compress="deflate")
    p = OUT / "map_2018_shift30E.tif"
    if not p.exists():
        with rasterio.open(p, "w", **dict(prof, transform=Affine(tr.a, tr.b, tr.c + SHIFT_E_M, tr.d, tr.e, tr.f))) as o:
            o.write(data)
    p = OUT / "map_2018_wrong_rolled.tif"
    if not p.exists():
        # content at (x, y) = original content at (x + 1000, y + 1500): columns roll left, rows roll down
        dc, dr = int(round(ROLL_M[0] / tr.a)), int(round(ROLL_M[1] / -tr.e))
        with rasterio.open(p, "w", **prof) as o:
            o.write(np.roll(np.roll(data, -dc, axis=2), dr, axis=1))


def write_hidden_copy(rec: S1.Recording):
    """Recording copy without truth: images (symlink), images.csv, meta.json, ahrs.csv (roll/pitch only),
    slice.json (flight start / cut / end image indices of the replay)."""
    if HIDDEN.exists():
        shutil.rmtree(HIDDEN)
    HIDDEN.mkdir(parents=True)
    src = rec.path
    os.symlink(src / "images", HIDDEN / "images")
    for f in ("images.csv", "meta.json"):
        shutil.copy(src / f, HIDDEN / f)
    rows = []
    for q in rec.q:
        R = S1.r_enu_body(q)
        R0 = S1.rz_up(S1.heading_of(R)) @ R          # heading removed (bearing 0): roll/pitch only
        rows.append(rot_to_quat(R0))
    pd.DataFrame(rows, columns=["qw", "qx", "qy", "qz"]).to_csv(HIDDEN / "ahrs.csv", index_label="image")
    (HIDDEN / "slice.json").write_text(json.dumps(dict(first=rec.first, last=rec.last, cut=rec.cut)))
    assert not (HIDDEN / "truth.csv").exists()


def rot_to_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return w, x, y, z


# ----------------------------------------------------------------------------- worker

def truth_free_loader(rec_dir: str, route: str = "") -> S1.Recording:
    p = Path(rec_dir)
    meta = json.loads((p / "meta.json").read_text())
    rt = json.loads(Path(route).read_text())
    img = pd.read_csv(p / "images.csv")
    img = img[img.cam == "cam0"].reset_index(drop=True)
    q = pd.read_csv(p / "ahrs.csv")[["qw", "qx", "qy", "qz"]].to_numpy()
    sl = json.loads((p / "slice.json").read_text())
    nan = np.full(len(img), np.nan)
    K = meta["sensors"]["camera"]["cams"]["cam0"]["K"]
    cam = dict(image_width_px=meta["sensors"]["camera"]["cams"]["cam0"]["width"],
               image_height_px=meta["sensors"]["camera"]["cams"]["cam0"]["height"],
               fx_px=K[0], fy_px=K[4], cx_px=K[2], cy_px=K[5], distortion_k1_k2_p1_p2_k3=[0, 0, 0, 0, 0])
    e = np.array([])
    return S1.Recording(p, [str(p / x) for x in img.path], img.t_s.to_numpy(float), q, nan, nan, nan,
                        sl["first"], sl["last"], sl["cut"], nan, (rt["origin_easting_m"], rt["origin_northing_m"]),
                        e, e, e, "none (truth hidden)", cam)


def _forbid_truth():
    real_open = builtins.open

    def guarded(file, *a, **k):
        if "truth" in os.path.basename(str(file)):
            raise PermissionError(f"truth access blocked: {file}")
        return real_open(file, *a, **k)
    builtins.open = guarded
    real_csv = pd.read_csv

    def guarded_csv(f, *a, **k):
        if "truth" in os.path.basename(str(f)):
            raise PermissionError(f"truth access blocked: {f}")
        return real_csv(f, *a, **k)
    pd.read_csv = guarded_csv


def run_one(job):
    name, u = job
    if name == "truth_hidden":
        _forbid_truth()
        S1.load_recording = truth_free_loader
    elif name in ("map_shift30E", "wrong_map"):
        path = OUT / ("map_2018_shift30E.tif" if name == "map_shift30E" else "map_2018_wrong_rolled.tif")
        m = G.MapRaster(path)
        S1.load_map = lambda *a, **k: m
    t0 = time.time()
    return name, S1.run_sequence(u), time.time() - t0


# ----------------------------------------------------------------------------- commands

def cmd_run(args):
    rec = S1.load_recording()
    calib = json.loads(S1.calib_path(CAMERA).read_text())
    write_maps()
    write_hidden_copy(rec)
    u = S1.unit_for(rec, SEED, CAMERA, "loop", calib, str(S1.REC_DEFAULT), str(S1.ROUTE_DEFAULT), "scale")
    jobs = []
    for name in args.runs:
        uu = dict(u)
        if name == "truth_hidden":
            uu["rec"] = str(HIDDEN)
        jobs.append((name, uu))
    print(f"{len(jobs)} runs, workers {args.workers}", flush=True)
    # one job per worker process: the monkeypatches of one run never leak into another
    with get_context("spawn").Pool(args.workers, initializer=S1._worker_init, maxtasksperchild=1) as pool:
        for name, rows, dt in pool.imap_unordered(run_one, jobs):
            df = S1.score(rows, u, rec)
            df["run"] = name
            df.to_csv(OUT / f"{name}.csv", index=False)
            a = df[df.after_cut]
            print(f"  {name}: median {a.err_m.median():.1f} m, max {a.err_m.max():.1f} m, final "
                  f"{a.err_m.iloc[-1]:.1f} m, accepted {(a.fix_status == 'accepted').sum()}, {dt:.0f} s", flush=True)


def load_runs():
    return {n: pd.read_csv(OUT / f"{n}.csv") for n in RUNS if (OUT / f"{n}.csv").exists()}


def table(R) -> pd.DataFrame:
    base = R["normal"].set_index("image")
    out = []
    for n, df in R.items():
        a = df[df.after_cut]
        tries = a[a.fix_try]
        acc = tries[tries.fix_status == "accepted"]
        d = a.set_index("image")
        diff = np.hypot(d.post_e - base.post_e.reindex(d.index), d.post_n - base.post_n.reindex(d.index))
        fe = (acc.z_e - acc.true_e) if len(acc) else pd.Series(dtype=float)
        out.append(dict(run=n, median_err_m=a.err_m.median(), max_err_m=a.err_m.max(), final_err_m=a.err_m.iloc[-1],
                        median_east_err_m=(a.post_e - a.true_e).median(),
                        final_east_err_m=(a.post_e - a.true_e).iloc[-1],
                        dr_final_err_m=a.err_dr_m.iloc[-1], fix_tries=len(tries), accepted=len(acc),
                        gated=int((tries.fix_status == "gated").sum()),
                        no_consensus=int(tries.fix_status.isin(["nofix", "disagree"]).sum()),
                        accepted_fix_east_err_median_m=fe.median() if len(fe) else np.nan,
                        max_diff_vs_normal_m=float(diff.max()),
                        bit_identical_to_normal=bool((d.post_e.values == base.post_e.reindex(d.index).values).all()
                                                     and (d.post_n.values == base.post_n.reindex(d.index).values).all())))
    t = pd.DataFrame(out)
    t.to_csv(OUT / "table.csv", index=False)
    return t


def cmd_figure(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    R = load_runs()
    t = table(R)
    print(t.to_string(index=False))
    N = R["normal"][R["normal"].after_cut]
    fig, ax = plt.subplots(3, 1, figsize=(12, 12), sharex=True)

    def acc_marks(a, df, y, c):
        k = df[df.after_cut & (df.fix_status == "accepted")]
        a.plot(k.dist_since_cut_m, np.full(len(k), y), "|", color=c, ms=8, alpha=.6, label="| = fix accepted")

    # 1: truth hidden
    H = R.get("truth_hidden")
    a = ax[0]
    a.plot(N.dist_since_cut_m, N.err_m, color="tab:blue", lw=3, label="normal run (truth.csv present)")
    a.plot(N.dist_since_cut_m, N.err_dr_m, color="tab:red", lw=1, label="dead reckoning (same odometry, no fixes)")
    if H is not None:
        h = H[H.after_cut]
        r = t.set_index("run").loc["truth_hidden"]
        a.plot(h.dist_since_cut_m, h.err_m, color="yellow", lw=1.2, ls="--",
               label=f"truth hidden (no truth.csv, truth reads blocked): max diff vs normal "
                     f"{r.max_diff_vs_normal_m:.2g} m, bit-identical={r.bit_identical_to_normal}")
    a.set_title("(a) TRUTH HIDDEN: the estimator runs without truth.csv. Same estimates as the normal run.")
    # 2: map shift
    S = R.get("map_shift30E")
    a = ax[1]
    a.plot(N.dist_since_cut_m, N.post_e - N.true_e, color="tab:blue", lw=1.5, label="normal: east error")
    if S is not None:
        s = S[S.after_cut]
        a.plot(s.dist_since_cut_m, s.post_e - s.true_e, color="tab:purple", lw=1.8,
               label="map moved +30 m east: east error of the estimate")
        k = s[s.fix_try & s.zncc_mx.notna()]
        a.plot(k.dist_since_cut_m, k.zncc_mx - S1.load_recording().origin[0] - np.array(
            json.loads(S1.calib_path(CAMERA).read_text())["map_offset_m"])[0] - k.true_e, ".", color="violet", ms=3,
               label="raw ZNCC map fix, east error (before the gate)")
        acc_marks(a, S, -12, "tab:purple")
    a.axhline(SHIFT_E_M, color="grey", ls=":")
    a.set_ylim(-15, 45)
    a.set_ylabel("east error (m)")
    first = S[S.after_cut & (S.fix_status == "accepted")].dist_since_cut_m.min() if S is not None else np.nan
    a.set_title(f"(b) MAP SHIFTED +30 m east after calibration: the fixes move 30 m east; the 99 % gate refuses the "
                f"jump for {first:.0f} m, then the estimate follows the map", fontsize=10.5)
    # 3: wrong map
    Wm = R.get("wrong_map")
    a = ax[2]
    a.plot(N.dist_since_cut_m, N.err_m, color="tab:blue", lw=1.5, label="normal")
    a.plot(N.dist_since_cut_m, N.err_dr_m, color="tab:red", lw=1, label="dead reckoning")
    if Wm is not None:
        w = Wm[Wm.after_cut]
        r = t.set_index("run").loc["wrong_map"]
        a.plot(w.dist_since_cut_m, w.err_m, color="black", lw=1.8, ls="--",
               label=f"wrong map (ground from ~1.8 km away): {int(r.accepted)} fixes accepted of {int(r.fix_tries)}")
        acc_marks(a, Wm, -3, "black")
    a.set_title("(c) WRONG MAP: no consensus, no accepted fix, the estimate drifts like dead reckoning")
    for a in (ax[0], ax[2]):
        a.set_ylabel("horizontal error (m)")
    for a in ax:
        a.grid(alpha=.3)
        a.legend(fontsize=9, loc="upper left")
    ax[2].set_xlabel("distance flown since the GNSS cut (m)")
    fig.suptitle("Anti-cheat tests, SIMULATED Wufeng flight (Gazebo), seed 0, ideal camera, our EKF + consensus fixes",
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(OUT.parent / "anti_cheat.png", dpi=110)
    print(OUT.parent / "anti_cheat.png")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--runs", nargs="+", default=list(RUNS), choices=RUNS)
    r.add_argument("--workers", type=int, default=2)
    sub.add_parser("figure")
    args = ap.parse_args()
    {"run": cmd_run, "figure": cmd_figure}[args.cmd](args)


if __name__ == "__main__":
    main()

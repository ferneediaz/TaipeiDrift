#!/usr/bin/env python3
"""S1 report: metrics after the GNSS cut and the two pitch figures, from outputs/s1_sim/.

Inputs (written by experiments/s1_sim_map_fix.py):
  runs/<camera>/loop_s<seed>.csv          ours (EKF + consensus fixes) and dead reckoning (same pass)
  dustin_<camera>/frames_<run>_s<seed>.csv Dustin's navigator per frame (camera_alone, map_2018), if present
Outputs:
  summary_per_run.csv   one row per estimator x camera x seed (MEASURED on the simulated recording)
  summary.csv           median over seeds per estimator x camera
  fig_error_vs_distance_<camera>.png, fig_trajectory_<camera>_s<seed>.png  (1600 px wide)

  $PY experiments/s1_sim_report.py [--camera ideal] [--traj-seed 0]
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs/s1_sim"
WRONG_M = 10.0
COLORS = {"ours": "#1f77b4", "ours_x5_ekf": "#9467bd", "dead_reckoning": "#d62728", "dustin_map": "#2ca02c",
          "dustin_camera_alone": "#ff7f0e"}
LABELS = {"ours": "Ours: odometry + consensus map fixes (EKF with scale state)",
          "ours_x5_ekf": "Ours with the 3-state x5 EKF (no scale state)",
          "dead_reckoning": "Dead reckoning (same odometry, no fixes)",
          "dustin_map": "Dustin's navigator, 2018 map", "dustin_camera_alone": "Dustin's navigator, camera alone"}


def longest_gap(dist: np.ndarray, accepted: np.ndarray) -> float:
    """Longest distance flown after the cut without an accepted fix (cut->first and last->end included)."""
    marks = np.r_[dist[0], dist[accepted], dist[-1]]
    return float(np.diff(marks).max()) if len(marks) > 1 else float(dist[-1] - dist[0])


def metrics(err: np.ndarray) -> dict:
    return dict(median_m=float(np.median(err)), p90_m=float(np.percentile(err, 90)), max_m=float(err.max()),
                final_m=float(err[-1]))


def series(camera: str) -> list[dict]:
    """One dict per estimator x seed: dist (m since cut), err, and fix columns where defined.
    Estimators: ours (runs/), ours_x5_ekf (runs_x5filter/), dead_reckoning (from runs/), dustin_map,
    dustin_camera_alone (dustin_<camera>/)."""
    out = []
    for sub, name in (("runs", "ours"), ("runs_x5filter", "ours_x5_ekf")):
        for p in sorted((OUT / sub / camera).glob("loop_s*.csv")):
            seed = int(re.search(r"_s(\d+)", p.name).group(1))
            df = pd.read_csv(p)
            a = df[df.after_cut].reset_index(drop=True)
            d = a.dist_since_cut_m.to_numpy()
            st = a.fix_status.fillna("") if "fix_status" in a else pd.Series([""] * len(a))
            acc = (st == "accepted").to_numpy()
            fe = a.fix_err_m.to_numpy() if "fix_err_m" in a else np.full(len(a), np.nan)
            out.append(dict(estimator=name, seed=seed, dist=d, err=a.err_m.to_numpy(), df=a, extra=dict(
                fix_tries=int(a.fix_try.sum()), accepted=int(acc.sum()), gated=int((st == "gated").sum()),
                agree=int(st.isin(["accepted", "gated"]).sum()), wrong_accepted=int((acc & (fe > WRONG_M)).sum()),
                accepted_fix_err_median_m=float(np.nanmedian(fe[acc])) if acc.any() else np.nan,
                longest_gap_m=longest_gap(d, acc), lol_frames=int(a.lol.sum()), lol_share=float(a.lol.mean()),
                lol_at_end=bool(a.lol.iloc[-1]))))
            if name == "ours":
                out.append(dict(estimator="dead_reckoning", seed=seed, dist=d, err=a.err_dr_m.to_numpy(), df=a,
                                extra={}))
    for name, key in (("map_2018", "dustin_map"), ("camera_alone", "dustin_camera_alone")):
        for p in sorted((OUT / f"dustin_{camera}").glob(f"frames_{name}_s*.csv")):
            seed = int(re.search(r"_s(\d+)", p.name).group(1))
            df = pd.read_csv(p)
            ex = {}
            if name == "map_2018":
                used = df.fix_used.to_numpy(bool)
                ex = dict(accepted=int(used.sum()), longest_gap_m=longest_gap(df.dist_since_cut_m.to_numpy(), used))
            out.append(dict(estimator=key, seed=seed, dist=df.dist_since_cut_m.to_numpy(), err=df.err_m.to_numpy(),
                            df=df, extra=ex))
    return out


def summarize(camera: str, ss: list[dict]) -> pd.DataFrame:
    rows = [dict(estimator=s["estimator"], camera=camera, seed=s["seed"], km_after_cut=float(s["dist"][-1] / 1000),
                 **metrics(s["err"]), **s["extra"]) for s in ss]
    per = pd.DataFrame(rows)
    allp = OUT / "summary_per_run.csv"
    if allp.exists():
        old = pd.read_csv(allp)
        per = pd.concat([old[old.camera != camera], per], ignore_index=True)
    per.to_csv(allp, index=False)
    num = [c for c in per.columns if c not in ("estimator", "camera", "seed") and per[c].dtype != object]
    med = per.groupby(["estimator", "camera"])[num].median().reset_index()
    med.insert(2, "seeds", per.groupby(["estimator", "camera"]).seed.apply(lambda s: " ".join(map(str, sorted(s)))).values)
    med.to_csv(OUT / "summary.csv", index=False)
    return per


def fig_error(camera: str, ss: list[dict]):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 15})
    fig, ax = plt.subplots(figsize=(16, 8), dpi=100)
    for est in ("dead_reckoning", "dustin_camera_alone", "dustin_map", "ours"):
        mine = [s for s in ss if s["estimator"] == est]
        for k, s in enumerate(sorted(mine, key=lambda s: s["seed"])):
            ax.plot(s["dist"] / 1000, s["err"], color=COLORS[est], lw=2.4 if k == 0 else 0.9,
                    alpha=1.0 if k == 0 else 0.45,
                    label=f"{LABELS[est]} ({len(mine)} seed{'s' if len(mine) > 1 else ''})" if k == 0 else None)
    ax.set_xlabel("distance flown since GNSS was cut (km)      [thick line: seed 0; thin lines: the other seeds]")
    ax.set_ylabel("horizontal position error (m)")
    ax.set_yscale("symlog", linthresh=10)
    ax.set_ylim(0, None)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="upper left", fontsize=13)
    ax.set_title(f"Simulated flight over Wufeng, 80 m, GNSS cut after 450 m ({camera} camera) - MEASURED in simulation")
    fig.tight_layout()
    p = OUT / f"fig_error_vs_distance_{camera}.png"
    fig.savefig(p)
    plt.close(fig)
    print(f"wrote {p}")


def fig_traj(camera: str, ss: list[dict], seed: int):
    import json

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import rasterio
    from rasterio.windows import from_bounds
    cal = json.loads((OUT / ("calibration.json" if camera == "ideal" else f"calibration_{camera}.json")).read_text())
    route = json.loads((ROOT / "sim/scenarios/wufeng_south_80m.json").read_text())
    ox, oy = route["origin_easting_m"], route["origin_northing_m"]
    s = next(x for x in ss if x["estimator"] == "ours" and x["seed"] == seed)
    full = pd.read_csv(OUT / "runs" / camera / f"loop_s{seed}.csv")
    a = s["df"]
    pad = 80
    e0, e1 = min(full.true_e.min(), full.dr_e.min()) - pad, max(full.true_e.max(), full.dr_e.max()) + pad
    n0, n1 = min(full.true_n.min(), full.dr_n.min()) - pad, max(full.true_n.max(), full.dr_n.max()) + pad
    with rasterio.open(OUT / "map_2018_0.5m.tif") as ds:
        win = from_bounds(ox + e0, oy + n0, ox + e1, oy + n1, ds.transform)
        rgb = ds.read([1, 2, 3], window=win, boundless=True).transpose(1, 2, 0)
    off = np.array(cal["map_offset_m"])
    plt.rcParams.update({"font.size": 15})
    h = (n1 - n0) / (e1 - e0)
    fig, ax = plt.subplots(figsize=(16, max(8, min(24, 16 * h))), dpi=100)
    # map pixels are true + offset: shift the image by -offset so it sits in the truth frame
    ax.imshow(rgb, extent=(e0 - off[0], e1 - off[0], n0 - off[1], n1 - off[1]), alpha=0.8)
    ax.plot(full.true_e, full.true_n, color="white", lw=4.5, label="truth")
    ax.plot(full.true_e, full.true_n, color="black", lw=1.5)
    ax.plot(full.dr_e, full.dr_n, color=COLORS["dead_reckoning"], lw=2.2, label="dead reckoning")
    ax.plot(full.post_e, full.post_n, color=COLORS["ours"], lw=2.2, label="ours (EKF + map fixes)")
    acc = a[a.fix_status == "accepted"] if "fix_status" in a else a.iloc[:0]
    ax.scatter(acc.z_e, acc.z_n, s=14, color="yellow", edgecolor="black", lw=0.4, zorder=5,
               label=f"accepted map fixes ({len(acc)})")
    c = a.iloc[0]
    ax.scatter([c.true_e], [c.true_n], marker="X", s=220, color="magenta", edgecolor="black", zorder=6,
               label="GNSS cut (450 m)")
    ax.set_xlim(e0, e1)
    ax.set_ylim(n0, n1)
    ax.set_aspect("equal")
    ax.set_xlabel("east of the route origin (m)")
    ax.set_ylabel("north of the route origin (m)")
    ax.legend(loc="best", fontsize=13, framealpha=0.9)
    ax.set_title(f"Seed {seed}, {camera} camera, over the 2018 map (2 years older than the simulated ground)")
    fig.tight_layout()
    p = OUT / f"fig_trajectory_{camera}_s{seed}.png"
    fig.savefig(p)
    plt.close(fig)
    print(f"wrote {p}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default="ideal")
    ap.add_argument("--traj-seed", type=int, default=0)
    args = ap.parse_args()
    ss = series(args.camera)
    per = summarize(args.camera, ss)
    with pd.option_context("display.width", 250, "display.max_columns", 30, "display.precision", 1):
        print(per[per.camera == args.camera].sort_values(["estimator", "seed"]).to_string(index=False))
        print(pd.read_csv(OUT / "summary.csv").to_string(index=False))
    fig_error(args.camera, ss)
    fig_traj(args.camera, ss, args.traj_seed)


if __name__ == "__main__":
    main()

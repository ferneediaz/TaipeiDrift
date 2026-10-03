#!/usr/bin/env python3
"""S2 report: Alessandro's ESKF alone / with our map fixes, joined with S1's estimators on the same recording.

Inputs:  outputs/s2_sim/runs/<camera>/<config>_s<seed>_{summary.json,track.csv,fixes.csv} (s2_sim_eskf_fusion.py)
         outputs/s1_sim/runs/<camera>/loop_s<seed>.csv and Dustin's frames, via experiments/s1_sim_report.series
Outputs (outputs/s2_sim/, suffixed with the camera):
  summary_eskf_per_run_<camera>.csv, summary_eskf_median_over_seeds_<camera>.csv   ESKF configurations
  joint_table_<camera>_s<seed>.csv, joint_table_<camera>_median_over_seeds.csv     every estimator, same recording
  fig_error_vs_distance_<camera>_s<seed>.png
Tables are scored on S1's grid (every 5th image from the cut, 1 Hz, state after the fix update).

  $PY experiments/s2_sim_report.py [--camera ideal] [--seed 0]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "experiments"))
OUT = ROOT / "outputs/s2_sim"
COLORS = {"A": "#111111", "B": "#17becf"}


def metrics(err: np.ndarray) -> dict:
    return dict(median_m=float(np.median(err)), p90_m=float(np.percentile(err, 90)), max_m=float(err.max()),
                final_m=float(err[-1]))


def longest_gap(dist: np.ndarray, accepted_dist: np.ndarray) -> float:
    marks = np.r_[dist[0], np.sort(accepted_dist), dist[-1]]
    return float(np.diff(marks).max())


def eskf_rows(camera: str):
    rows, curves = [], {}
    runs = OUT / "runs" / camera
    for p in sorted(runs.glob("*_s*_summary.json")):
        s = json.loads(p.read_text())
        c, seed = s["config"], s["seed"]
        name = p.name.removesuffix("_summary.json")
        tr = pd.read_csv(runs / f"{name}_track.csv")
        a_all = tr[tr.after_cut]
        # score on S1's grid (every 5th image from the cut, 1 Hz, state after the fix update) for a like-for-like table
        a = a_all[(a_all.image - s["cut_image"]) % 5 == 0]
        d, e = a.dist_since_cut_m.to_numpy(), a.err_h_m.to_numpy()
        r = dict(config=c, seed=seed, estimator=s["label"], **metrics(e), scored_rows=len(a),
                 max_all_5hz_images_m=float(a_all.err_h_m.max()), dist_flown_after_cut_m=float(d[-1]),
                 at_cut_m=s["at_cut_err_m"], nees_h_mean=s["after_cut"]["nees_h_mean"],
                 nees_h_frac_above_99=s["after_cut"]["nees_h_frac_above_99"],
                 heading_err_abs_median_deg=s["after_cut"]["heading_err_abs_median_deg"],
                 gnss_precut_nis_mean=s["gnss_precut"]["nis_mean"],
                 flow_accepted=s["flow"]["accepted"], flow_attempted=s["flow"]["attempted"],
                 height_learned_m=s["height"]["height_learned_m"], true_height_at_cut_m=s["height"]["true_height_at_cut_m"])
        if "fixes" in s:
            fx = pd.read_csv(runs / f"{name}_fixes.csv")
            r.update({f"fix_{k}": v for k, v in s["fixes"].items() if not isinstance(v, list)})
            r["fix_nis_rejected"] = " ".join(f"{x:g}" for x in s["fixes"]["nis_rejected"])
            r["longest_gap_m"] = longest_gap(d, fx[fx.status == "accepted"].dist_since_cut_m.to_numpy())
        rows.append(r)
        curves[(c, seed)] = (s["label"], a_all.dist_since_cut_m.to_numpy(), a_all.err_h_m.to_numpy())
    return rows, curves


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="ideal")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rows, curves = eskf_rows(a.camera)
    per_run = pd.DataFrame(rows)
    per_run.to_csv(OUT / f"summary_eskf_per_run_{a.camera}.csv", index=False)
    num = per_run.select_dtypes("number").drop(columns="seed")
    agg = num.groupby(per_run.config).median().assign(seeds=per_run.groupby("config").seed.count())
    agg.to_csv(OUT / f"summary_eskf_median_over_seeds_{a.camera}.csv")
    print(agg[["median_m", "p90_m", "max_m", "final_m", "seeds"]].to_string(float_format=lambda x: f"{x:.1f}"))

    def jrow(est, source, seed, m, ex):
        return dict(estimator=est, source=source, seed=seed, camera=a.camera,
                    **{k: m[k] for k in ("median_m", "p90_m", "max_m", "final_m")},
                    accepted_fixes=ex.get("accepted"), wrong_accepted=ex.get("wrong_accepted"),
                    longest_gap_m=ex.get("longest_gap_m"))

    every = [jrow(r["estimator"], "s2 (ESKF)", r["seed"], r,
                  dict(accepted=r.get("fix_accepted"), wrong_accepted=r.get("fix_wrong_accepted"),
                       longest_gap_m=r.get("longest_gap_m"))) for r in rows]
    curves = [(c, *v) for (c, sd), v in sorted(curves.items()) if sd == a.seed]
    s1_curves = []
    try:
        import s1_sim_report as R1
        for sr in R1.series(a.camera):
            lab = R1.LABELS.get(sr["estimator"], sr["estimator"])
            every.append(jrow(lab, "s1", sr["seed"], metrics(sr["err"]), sr.get("extra", {})))
            if sr["seed"] == a.seed:
                s1_curves.append((sr["estimator"], lab, sr["dist"], sr["err"], R1.COLORS.get(sr["estimator"])))
    except Exception as exc:  # S1 outputs not there yet: report the ESKF rows only
        print("S1 series unavailable:", exc)
    ev = pd.DataFrame(every)
    jt = ev[ev.seed == a.seed]
    jt.to_csv(OUT / f"joint_table_{a.camera}_s{a.seed}.csv", index=False)
    print(jt.to_string(index=False, float_format=lambda x: f"{x:.1f}"))
    cols = ["median_m", "p90_m", "max_m", "final_m", "accepted_fixes", "wrong_accepted", "longest_gap_m"]
    jm = ev.groupby(["source", "estimator"], sort=False)[cols].median().assign(
        seeds=ev.groupby(["source", "estimator"], sort=False).seed.apply(lambda s: " ".join(map(str, sorted(s)))))
    jm.to_csv(OUT / f"joint_table_{a.camera}_median_over_seeds.csv")
    print(jm.to_string(float_format=lambda x: f"{x:.1f}"))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(16, 7), dpi=100)
    for c, lab, d, e in curves:
        ax.plot(d, e, color=COLORS[c], lw=1.6, label=lab)
    for _, lab, d, e, col in s1_curves:
        ax.plot(d, e, color=col, lw=1.2, alpha=0.85, label=f"S1: {lab}")
    ax.set_xlabel("distance flown since the GNSS cut (m)")
    ax.set_ylabel("horizontal position error (m)")
    ax.set_yscale("log")
    ax.grid(True, which="both", alpha=0.3)
    ax.set_title(f"SIMULATED Wufeng south 80 m, GNSS cut after 450 m; camera {a.camera}, seed {a.seed}")
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT / f"fig_error_vs_distance_{a.camera}_s{a.seed}.png")
    print("wrote", OUT / f"fig_error_vs_distance_{a.camera}_s{a.seed}.png")


if __name__ == "__main__":
    main()

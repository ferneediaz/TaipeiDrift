#!/usr/bin/env python3
"""S1 stress test: how our error grows when map fixes are attempted less often (seed 0, both cameras).

Inputs (experiments/s1_sim_map_fix.py run --seeds 0 --camera <c> --fix-every-m <m>):
  outputs/s1_sim/stress/<camera>/loop_s0_fix<m>m.csv     fix attempted after <m> m of ESTIMATED flight
  outputs/s1_sim/runs/<camera>/loop_s0.csv               default: every 5th image (once per second, ~7.6 m)
  outputs/s1_sim/dustin_<camera>/frames_map_2018_s0.csv  Dustin's navigator (fix about every 300 m), reference
Outputs: outputs/s1_sim/stress/summary.csv, outputs/s1_sim/stress/fig_stress_fix_spacing.png (1600 px wide).

  $PY experiments/s1_sim_stress.py
"""
from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs/s1_sim"
ST = OUT / "stress"
WRONG_M = 10.0
CAMS = ("ideal", "realistic")


def longest_gap(d, acc):
    marks = np.r_[d[0], d[acc], d[-1]]
    return float(np.diff(marks).max())


def load() -> tuple[pd.DataFrame, dict]:
    rows, curves = [], {}
    for cam in CAMS:
        files = [(None, OUT / "runs" / cam / "loop_s0.csv")]
        files += [(float(re.search(r"fix([\d.]+)m", p.name).group(1)), p)
                  for p in (ST / cam).glob("loop_s0_fix*m.csv")]
        for m, p in files:
            if not p.exists():
                continue
            a = pd.read_csv(p)
            a = a[a.after_cut].reset_index(drop=True)
            d = a.dist_since_cut_m.to_numpy()
            st = a.fix_status.fillna("")
            acc = (st == "accepted").to_numpy()
            tries = int(a.fix_try.sum())
            label = "1 s (default, ~7.5 m)" if m is None else f"{m:g} m"
            rows.append(dict(camera=cam, spacing=label, spacing_m=d[-1] / max(tries, 1) if m is None else m,
                             attempts=tries, accepted=int(acc.sum()), gated=int((st == "gated").sum()),
                             wrong_accepted=int((acc & (a.fix_err_m > WRONG_M)).sum()),
                             median_m=a.err_m.median(), p90_m=a.err_m.quantile(0.9), max_m=a.err_m.max(),
                             final_m=a.err_m.iloc[-1], longest_gap_m=longest_gap(d, acc),
                             lost_lock_frames=int(a.lol.sum()), source=str(p.relative_to(OUT))))
            curves[(cam, label)] = (d, a.err_m.to_numpy())
        p = OUT / f"dustin_{cam}" / "frames_map_2018_s0.csv"
        if p.exists():
            f = pd.read_csv(p)
            used = f.fix_used.to_numpy(bool)
            rows.append(dict(camera=cam, spacing="Dustin's navigator (~300 m)", spacing_m=300.0, attempts=np.nan,
                             accepted=int(used.sum()), median_m=f.err_m.median(), p90_m=f.err_m.quantile(0.9),
                             max_m=f.err_m.max(), final_m=f.err_m.iloc[-1],
                             longest_gap_m=longest_gap(f.dist_since_cut_m.to_numpy(), used), source=str(p.relative_to(OUT))))
            curves[(cam, "dustin")] = (f.dist_since_cut_m.to_numpy(), f.err_m.to_numpy())
    return pd.DataFrame(rows), curves


def figure(df: pd.DataFrame, curves: dict):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 14})
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(16, 7.5), dpi=100, gridspec_kw=dict(width_ratios=[1.5, 1]))
    ours = df[~df.spacing.str.startswith("Dustin")].sort_values("spacing_m")
    labels = list(dict.fromkeys(ours.spacing))
    cmap = plt.get_cmap("viridis")
    for k, lab in enumerate(labels):
        col = cmap(k / max(1, len(labels) - 1) * 0.9)
        for cam, ls in (("ideal", "-"), ("realistic", "--")):
            if (cam, lab) in curves:
                d, e = curves[(cam, lab)]
                ax.plot(d / 1000, e, ls, color=col, lw=2.0 if cam == "ideal" else 1.4,
                        label=f"fix every {lab}" if cam == "ideal" else None)
    for cam, ls in (("ideal", "-"), ("realistic", "--")):
        if (cam, "dustin") in curves:
            d, e = curves[(cam, "dustin")]
            ax.plot(d / 1000, e, ls, color="#d62728", lw=1.4, alpha=0.8,
                    label="Dustin's navigator, 2018 map" if cam == "ideal" else None)
    ax.set_yscale("symlog", linthresh=10)
    ax.set_ylim(0, None)
    ax.set_xlabel("distance flown since GNSS was cut (km)")
    ax.set_ylabel("horizontal position error (m)")
    ax.set_title("Error vs distance, seed 0 (solid: ideal camera, dashed: realistic)", fontsize=14)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="upper left", fontsize=12)
    for cam, mk in (("ideal", "o"), ("realistic", "s")):
        s = ours[ours.camera == cam]
        bx.plot(s.spacing_m, s.median_m, "-" + mk, color="#1f77b4", label=f"ours, median ({cam})",
                mfc="white" if cam == "realistic" else None)
        bx.plot(s.spacing_m, s.max_m, "--" + mk, color="#ff7f0e", label=f"ours, worst ({cam})",
                mfc="white" if cam == "realistic" else None)
        dz = df[(df.camera == cam) & df.spacing.str.startswith("Dustin")]
        if len(dz):
            bx.plot(dz.spacing_m, dz.median_m, mk, color="#d62728", ms=11, mfc="white" if cam == "realistic" else None,
                    label=f"Dustin, median ({cam})")
            bx.plot(dz.spacing_m, dz.max_m, mk, color="#7f0000", ms=11, mfc="white" if cam == "realistic" else None,
                    label=f"Dustin, worst ({cam})")
    bx.set_xscale("log")
    bx.set_yscale("log")
    bx.set_xlabel("distance between fix attempts (m, log)")
    bx.set_ylabel("error after the cut (m, log)")
    bx.set_title("Median and worst error vs fix spacing", fontsize=14)
    bx.grid(True, which="both", alpha=0.3)
    bx.legend(fontsize=10, loc="upper left")
    fig.suptitle("Stress test: fewer map fixes, simulated Wufeng flight, 4.5 km without GNSS - MEASURED in simulation")
    fig.tight_layout()
    p = ST / "fig_stress_fix_spacing.png"
    fig.savefig(p)
    plt.close(fig)
    print(f"wrote {p}")


def main():
    df, curves = load()
    df.to_csv(ST / "summary.csv", index=False)
    with pd.option_context("display.width", 250, "display.max_columns", 20, "display.precision", 1):
        print(df.drop(columns="source").to_string(index=False))
    figure(df, curves)


if __name__ == "__main__":
    main()

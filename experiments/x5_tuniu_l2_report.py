#!/usr/bin/env python3
"""Level 2 evaluator: metrics, pass criteria and figure from the per-run CSVs of x5_tuniu_closed_loop.py.

Metric definitions and pass criteria: docs/research/tuniu-level2-prereg.md.

  .venv/bin/python experiments/x5_tuniu_l2_report.py summarize [--tag main]
  .venv/bin/python experiments/x5_tuniu_l2_report.py figure [--tag main] [--seeds 0 1 2 3]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "data/processed/x_tuniu_l2"
WRONG_M = 10.0
PRIMARY = ("loop", "dji", "dem_lifted")
CONS_ROWS = ROOT / "data/processed/x_tuniu/stage3_rows_cons_test.csv"


def load_runs(tag: str) -> pd.DataFrame:
    files = sorted((OUT / "runs" / tag).glob("*.csv"))
    if not files:
        sys.exit(f"no runs in {OUT / 'runs' / tag}")
    return pd.concat([pd.read_csv(f) for f in files], ignore_index=True)


def episodes(flags: np.ndarray) -> int:
    f = np.asarray(flags, bool)
    return int(np.sum(f & ~np.r_[False, f[:-1]]))


def run_metrics(g: pd.DataFrame) -> dict:
    g = g.sort_values("frame")
    e = g.err_m.to_numpy()
    has_fix = "fix_status" in g and g.fix_status.notna().any()
    st = g.fix_status if has_fix else pd.Series("", index=g.index)
    acc = (st == "accepted").to_numpy()
    t0 = float(g.t_s.iloc[0] - g.dt.iloc[0])                       # photo 46 = state at the cut
    t_acc = g.t_s.to_numpy()[acc]
    gaps = np.diff(np.r_[t0, t_acc, g.t_s.iloc[-1]])
    wrong = int((g.fix_err_m[acc] > WRONG_M).sum()) if has_fix else 0
    return dict(
        photos=len(g), err_median_m=float(np.median(e)), err_p90_m=float(np.quantile(e, .9)),
        err_max_m=float(e.max()), err_final_m=float(e[-1]), pct_lt10=100 * float(np.mean(e < 10)),
        pct_lt25=100 * float(np.mean(e < 25)), agreed=int(st.isin(["accepted", "gated"]).sum()),
        accepted=int(acc.sum()), gated=int((st == "gated").sum()), accepted_wrong_gt10=wrong,
        accepted_fix_err_median_m=float(g.fix_err_m[acc].median()) if acc.any() else np.nan,
        accepted_fix_err_max_m=float(g.fix_err_m[acc].max()) if acc.any() else np.nan,
        gated_fix_err_median_m=float(g.fix_err_m[st == "gated"].median()) if (st == "gated").any() else np.nan,
        lol_photos=int(g.lol.sum()), lol_events=episodes(g.lol), lol_cheb_photos=int(g.lol_cheb.sum()),
        longest_no_fix_s=float(gaps.max()), half_max_m=float(g.half_m.max()), half_median_m=float(g.half_m.median()),
        odo_ok_pct=100 * float((g.odo_status == "ok").mean()),
        b_final_err_deg=float(g.b_hat_deg.iloc[-1] - g.yaw_err_sim_deg.iloc[-1]),
        lock_margin_min_m=float((g.half_m - g.err_pred_m).min()), lock_ratio_max=float((g.err_pred_m / g.half_m).max()),
        gated_wrong_gt10=int((g.fix_err_m[st == "gated"] > WRONG_M).sum()) if has_fix else 0,
    )


def summarize(tag: str) -> dict:
    df = load_runs(tag)
    keys = ["mode", "heading", "ground", "seed"]
    per = pd.DataFrame([dict(zip(keys, k), **run_metrics(g)) for k, g in df.groupby(keys)])
    dr = per[per["mode"] == "dr"].set_index(["heading", "ground", "seed"])
    for col in ("err_median_m", "err_max_m", "err_final_m"):
        per[f"dr_{col}"] = [dr[col].get((h, gr, s), np.nan) for h, gr, s in zip(per.heading, per.ground, per.seed)]
    per.to_csv(OUT / f"summary_{tag}_runs.csv", index=False)
    pooled = []
    for (mode, heading, ground), g in per.groupby(["mode", "heading", "ground"]):
        e = df[(df["mode"] == mode) & (df.heading == heading) & (df.ground == ground)]
        ed = df[(df["mode"] == "dr") & (df.heading == heading) & (df.ground == ground)]
        pooled.append(dict(
            mode=mode, heading=heading, ground=ground, seeds=len(g),
            err_median_m=float(e.err_m.median()), err_p90_m=float(e.err_m.quantile(.9)), err_max_m=float(e.err_m.max()),
            err_final_median_m=float(g.err_final_m.median()), pct_lt10=100 * float((e.err_m < 10).mean()),
            pct_lt25=100 * float((e.err_m < 25).mean()),
            accepted_total=int(g.accepted.sum()), accepted_per_run_mean=float(g.accepted.mean()),
            gated_total=int(g.gated.sum()), gated_wrong_gt10_total=int(g.gated_wrong_gt10.sum()),
            accepted_wrong_gt10_total=int(g.accepted_wrong_gt10.sum()),
            accepted_fix_err_median_m=float(e.fix_err_m[e.fix_status == "accepted"].median()) if "fix_status" in e else np.nan,
            accepted_fix_err_max_m=float(e.fix_err_m[e.fix_status == "accepted"].max()) if "fix_status" in e else np.nan,
            lock_margin_min_m=float(g.lock_margin_min_m.min()), lock_ratio_max=float(g.lock_ratio_max.max()),
            seeds_lock_ratio_gt_0_9=int((g.lock_ratio_max > 0.9).sum()),
            b_final_abs_err_deg_median=float(g.b_final_err_deg.abs().median()),
            seeds_without_lol=int((g.lol_photos == 0).sum()), lol_photos_total=int(g.lol_photos.sum()),
            lol_events_total=int(g.lol_events.sum()), seeds_without_lol_cheb=int((g.lol_cheb_photos == 0).sum()),
            longest_no_fix_s_median=float(g.longest_no_fix_s.median()), longest_no_fix_s_max=float(g.longest_no_fix_s.max()),
            dr_err_median_m=float(ed.err_m.median()) if len(ed) else np.nan,
            dr_err_max_m=float(ed.err_m.max()) if len(ed) else np.nan,
            dr_err_final_median_m=float(per[(per["mode"] == "dr") & (per.heading == heading)
                                            & (per.ground == ground)].err_final_m.median()) if len(ed) else np.nan,
        ))
    pooled = pd.DataFrame(pooled)
    pooled.to_csv(OUT / f"summary_{tag}_pooled.csv", index=False)
    verdict = {}
    p = pooled[(pooled["mode"] == PRIMARY[0]) & (pooled.heading == PRIMARY[1]) & (pooled.ground == PRIMARY[2])]
    if len(p):
        p = p.iloc[0]
        verdict = dict(config="/".join(PRIMARY), seeds=int(p.seeds),
                       primary_seeds_without_lol=int(p.seeds_without_lol),
                       primary_pass=bool(p.seeds == 20 and p.seeds_without_lol >= 18),
                       secondary_median_err_m=float(p.err_median_m),
                       secondary_wrong_fixes=int(p.accepted_wrong_gt10_total),
                       secondary_pass=bool(p.seeds == 20 and p.err_median_m <= 10 and p.accepted_wrong_gt10_total == 0))
    out = dict(tag=tag, verdict=verdict, pooled=pooled.to_dict(orient="records"))
    (OUT / f"summary_{tag}.json").write_text(json.dumps(out, indent=1))
    pd.set_option("display.width", 250)
    print(pooled.round(2).T.to_string())
    print(json.dumps(verdict, indent=1))
    print(markdown(pooled))
    return out


def markdown(pooled: pd.DataFrame) -> str:
    """Results table for docs/research/tuniu-level2-results.md (closed loop vs dead reckoning)."""
    lines = ["| Heading | Ground | Mode | Seeds | Median / p90 / max error (m) | Final (median) | < 10 m | < 25 m "
             "| Fixes accepted (gated) | Wrong > 10 m | Seeds without loss of lock | Longest no-fix gap (median / max) |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    order = {"dji": 0, "drift": 1, "dem_lifted": 0, "dem_prior": 1, "loop": 0, "dr": 1}
    rows = pooled.sort_values(["heading", "ground", "mode"], key=lambda s: s.map(order))
    for r in rows.itertuples():
        loop = r.mode == "loop"
        lines.append(
            f"| {r.heading} | {r.ground} | {'closed loop' if loop else 'dead reckoning'} | {r.seeds} "
            f"| {r.err_median_m:.1f} / {r.err_p90_m:.1f} / {r.err_max_m:.1f} | {r.err_final_median_m:.1f} "
            f"| {r.pct_lt10:.0f} % | {r.pct_lt25:.0f} % "
            f"| {f'{r.accepted_total} ({r.gated_total})' if loop else '–'} "
            f"| {r.accepted_wrong_gt10_total if loop else '–'} "
            f"| {f'{r.seeds_without_lol} / {r.seeds}' if loop else '– (no window used)'} "
            f"| {f'{r.longest_no_fix_s_median:.0f} / {r.longest_no_fix_s_max:.0f} s' if loop else '630 s'} |")
    return "\n".join(lines)


def step1_never_fixed() -> set:
    """Photos never accepted by the step-1 agreement rule (0.5 m, seeds 1-19): the forest gaps."""
    d = pd.read_csv(CONS_ROWS, usecols=["frame", "seed", "method", "res", "kind", "map", "cfg", "fix_x", "fix_y"])
    d = d[(d.kind == "pos") & (d.res == 0.5) & (d["map"] == "main") & (d.cfg == "h=baro_dem,att=dji,gnd=dem_prior")]
    z = d[d.method == "zncc"].set_index(["frame", "seed"])
    x = d[d.method == "xfeat"].set_index(["frame", "seed"])
    j = z.join(x, lsuffix="_z", rsuffix="_x", how="inner")
    agree = np.hypot(j.fix_x_z - j.fix_x_x, j.fix_y_z - j.fix_y_x) <= 4.0
    any_acc = agree.groupby(level="frame").any()
    return set(any_acc.index[~any_acc])


def figure(tag: str, seeds, heading="dji", ground="dem_lifted"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    df = load_runs(tag)
    gaps = step1_never_fixed()
    fig, axes = plt.subplots(len(seeds), 1, figsize=(11, 2.3 * len(seeds)), sharex=True)
    axes = np.atleast_1d(axes)
    for ax, s in zip(axes, seeds):
        lp = df[(df["mode"] == "loop") & (df.heading == heading) & (df.ground == ground) & (df.seed == s)].sort_values("frame")
        dr = df[(df["mode"] == "dr") & (df.heading == heading) & (df.ground == ground) & (df.seed == s)].sort_values("frame")
        if lp.empty:
            continue
        t0 = lp.t_s.iloc[0] - lp.dt.iloc[0]
        t = lp.t_s - t0
        for f, tt, dt in zip(lp.frame, t, lp.dt):
            if f in gaps:
                ax.axvspan(tt - dt / 2, tt + dt / 2, color="#cfe8cf", lw=0)
            if lp.lol[lp.frame == f].any():
                ax.axvspan(tt - dt / 2, tt + dt / 2, color="#f4b6b6", lw=0)
        ax.plot(t, lp.half_m, color="0.5", ls="--", lw=1, label="search half-window")
        if len(dr):
            ax.plot(dr.t_s - t0, dr.err_m, color="tab:orange", lw=1.2, label="dead reckoning")
        ax.plot(t, lp.err_m, color="tab:blue", lw=1.5, label="closed loop")
        acc = lp.fix_status == "accepted"
        ax.plot(t[acc], np.zeros(acc.sum()) + 1, "|", color="k", ms=8, label="accepted fix")
        gat = lp.fix_status == "gated"
        if gat.any():
            ax.plot(t[gat], np.zeros(gat.sum()) + 1, "x", color="r", ms=5, label="gated-out fix")
        ax.set_yscale("log")
        ax.set_ylim(0.7, 300)
        ax.set_ylabel(f"seed {s}\nerror (m)")
        ax.grid(alpha=.3, which="both")
    axes[0].legend(loc="upper left", ncol=5, fontsize=8)
    axes[-1].set_xlabel("time since GNSS cut (s); green = photo never fixed in step 1 (forest), red = loss of lock")
    fig.suptitle(f"Tuniu Level 2, {heading} heading, {ground}: position error vs RTK (MEASURED photos, "
                 "SIMULATED baro)", fontsize=10)
    fig.tight_layout()
    p = OUT / f"fig_error_vs_time_{tag}.png"
    fig.savefig(p, dpi=130)
    print(f"wrote {p}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["summarize", "figure"])
    ap.add_argument("--tag", default="main")
    ap.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2, 3])
    a = ap.parse_args()
    if a.cmd == "summarize":
        summarize(a.tag)
    else:
        figure(a.tag, a.seeds)


if __name__ == "__main__":
    main()

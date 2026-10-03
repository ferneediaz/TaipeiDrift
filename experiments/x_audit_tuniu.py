#!/usr/bin/env python3
"""Independent audit of the Tuniu step-1 results (written separately from the x1–x4 pipeline).

Checks, using only raw inputs and the pipeline's per-photo output rows:
1. Truth: re-parse the DJI MRK log of the April 2019 flight directly (not truth.csv) and convert the
   RTK antenna positions to EPSG:3826 with pyproj.
2. Error: recompute |fix - truth| for every positive row from fix_x/fix_y minus the PRE-CUT map offset
   (stage1_calibration.json) and compare with the pipeline's err_m.
3. Headline metrics: recompute accepted / wrong > 10 m / negatives accepted from the rows and the
   pipeline's thresholds, and compare with stage3_summary_*.csv.
4. Chance baseline: what the same metrics would be if a "fix" were just the simulated prior
   (truth + uniform +/-40 m per axis), i.e. if the matcher contributed nothing.
5. Leak check: the estimator-side replay files must not contain RTK rows after the cut.

Prints PASS/FAIL per check; exits non-zero on any FAIL.

Run: .venv/bin/python experiments/x_audit_tuniu.py
"""
from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import Transformer

ROOT = Path(__file__).resolve().parents[1]
X = ROOT / "data/processed/x_tuniu"
REPLAY = ROOT / "data/processed/t_replay/tuniu_tw_1"
PHOTOS = Path(os.environ.get(   # same folder as experiments/x1_tuniu_export.py SOURCE
    "TUNIU_TW1", ROOT / "data/raw/tuniu_tw_1/20190411_Miaoli_Toufeng_Tuniu-River_5.75K/100_0005"))
PRIOR_HALF_M = 40.0
WRONG_M = 10.0

failures: list[str] = []


def check(name: str, ok: bool, detail: str) -> None:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    if not ok:
        failures.append(name)


def mrk_truth_3826() -> pd.DataFrame:
    mrk = next(PHOTOS.glob("*.MRK"))
    rows = []
    for line in mrk.read_text().splitlines():
        lat = re.search(r"([-\d.]+),Lat", line)
        lon = re.search(r"([-\d.]+),Lon", line)
        if lat and lon:
            rows.append((int(line.split("\t")[0]), float(lat.group(1)), float(lon.group(1))))
    df = pd.DataFrame(rows, columns=["frame", "lat", "lon"])
    x, y = Transformer.from_crs("EPSG:4326", "EPSG:3826", always_xy=True).transform(df.lon.values, df.lat.values)
    return df.assign(tx=x, ty=y).set_index("frame")


def accepted_mask(d: pd.DataFrame, method: str, threshold: float | None) -> pd.Series:
    if method == "zncc+quad>=3":
        return d.quad_n >= 3
    if method == "zncc":
        return d.score >= threshold
    return (d.gate == "ok") & (d.inliers >= threshold)


def recompute(rows: pd.DataFrame, summary: pd.DataFrame, truth: pd.DataFrame, offsets: dict) -> None:
    worst = 0.0
    n_rows = 0
    for _, s in summary.iterrows():
        base = "zncc" if s.method.startswith("zncc") else s.method
        d = rows[(rows["method"] == base) & (rows["res"] == s["res"]) & (rows["map"] == s["map"])
                 & (rows["cfg"] == s["cfg"]) & (rows["seed"] == s["seed"])]
        pos, neg = d[d["kind"] == "pos"], d[d["kind"] != "pos"]
        if len(pos) == 0:
            continue
        off = offsets[s["map"]]
        t = truth.loc[pos.frame]
        err = np.hypot(pos.fix_x.values - off["de_m"] - t.tx.values, pos.fix_y.values - off["dn_m"] - t.ty.values)
        ok_err = np.isfinite(pos.err_m.values)
        worst = max(worst, float(np.nanmax(np.abs(err[ok_err] - pos.err_m.values[ok_err]))) if ok_err.any() else 0.0)
        thr = None if pd.isna(s.threshold) else float(s.threshold)
        a = accepted_mask(pos, s.method, thr).values & np.isfinite(err)
        an = accepted_mask(neg, s.method, thr).values
        got = (int(a.sum()), int((err[a] > WRONG_M).sum()), int(an.sum()))
        want = (int(s.accepted), int(s.accepted_wrong_gt10), int(s.neg_accepted))
        n_rows += 1
        if got != want:
            check(f"metrics {s['method']} {s['res']} {s['map']} {s['cfg']} seed {s['seed']}", False,
                  f"recomputed {got} vs summary {want}")
    check("error recomputed from raw MRK", worst < 0.05, f"max |independent - pipeline| = {worst:.3f} m")
    check("summary rows recomputed", n_rows > 0, f"{n_rows} summary rows reproduced exactly (mismatches listed above)")


def chance_baseline(rng: np.random.Generator) -> None:
    d = rng.uniform(-PRIOR_HALF_M, PRIOR_HALF_M, size=(1_000_000, 2))
    r = np.hypot(d[:, 0], d[:, 1])
    print(f"[INFO] prior alone (no matching): median error {np.median(r):.1f} m, within {WRONG_M:.0f} m: "
          f"{100 * (r <= WRONG_M).mean():.1f}% of photos")


def leak_check() -> None:
    meta = json.loads((REPLAY / "meta.json").read_text())
    cut = float(meta["gnss_cut_s"])
    gnss = pd.read_csv(REPLAY / "gnss.csv")
    check("no RTK after the cut in estimator files", float(gnss.t_s.max()) <= cut,
          f"gnss.csv last t = {gnss.t_s.max():.2f} s, cut = {cut:.2f} s")
    att = pd.read_csv(REPLAY / "attitude.csv")
    banned = [c for c in att.columns if re.search(r"speed|lat|lon|alt", c, re.I)]
    check("attitude.csv has no speed/position/altitude columns", not banned, f"columns: {list(att.columns)}")


def main() -> None:
    truth = mrk_truth_3826()
    offsets = json.loads((X / "stage1_calibration.json").read_text())["map_offsets"]
    leak_check()
    chance_baseline(np.random.default_rng(0))
    for tag in ("main", "abl", "abl025", "seeds", "seedsx"):
        rows_f, sum_f = X / f"stage3_rows_{tag}_test.csv", X / f"stage3_summary_{tag}.csv"
        if rows_f.exists() and sum_f.exists():
            print(f"--- {tag}")
            recompute(pd.read_csv(rows_f), pd.read_csv(sum_f), truth, offsets)
    print("RESULT:", "ALL PASS" if not failures else f"{len(failures)} FAIL")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()

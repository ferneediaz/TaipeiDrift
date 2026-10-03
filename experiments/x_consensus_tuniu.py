#!/usr/bin/env python3
"""Evaluate the pre-registered XFeat + ZNCC consensus gate (docs/research/tuniu-consensus-prereg.md).

Rule: accept a photo iff XFeat and ZNCC both return a camera-nadir fix on the same query and reference
window and the two fixes are within 4.0 m; the reported position is the ZNCC fix. No fitted threshold.
Seeds 1–19 (seed 0 informed the rule), test photos, main map 2019-12, default query config.

Rows: ZNCC 0.25/0.5 m and XFeat 0.5 m from stage3_rows_cons_test.csv; XFeat 0.25 m from
stage3_rows_seedsx_test.csv (same seeds, same deterministic priors and negatives).

Run: .venv/bin/python experiments/x_consensus_tuniu.py
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import beta

ROOT = Path(__file__).resolve().parents[1]
X = ROOT / "data/processed/x_tuniu"
CFG = "h=baro_dem,att=dji,gnd=dem_prior"
SEEDS = range(1, 20)
AGREE_M = 4.0
WRONG_M = 10.0
KEY = ["frame", "seed", "kind", "ref_map", "neg_prior_x", "neg_prior_y"]
REFERENCE = {"rule": "ZNCC+quad>=3, 1 m, gnd=dem_lifted (20 seeds)", "accepted": 1461, "wrong": 3}


def upper95(k: int, n: int) -> float:
    return float(beta.ppf(0.95, k + 1, n - k)) if n else float("nan")


def rows(path: Path, method: str, res: float) -> pd.DataFrame:
    d = pd.read_csv(path)
    d = d[(d["method"] == method) & (d["res"] == res) & (d["map"] == "main") & (d["cfg"] == CFG)
          & (d["seed"].isin(SEEDS))]
    return d[KEY + ["fix_x", "fix_y", "err_m"]]


def evaluate(res: float, z: pd.DataFrame, x: pd.DataFrame) -> dict:
    m = z.merge(x, on=KEY, suffixes=("_z", "_x"), how="inner", validate="one_to_one")
    if len(m) != len(z) or len(m) != len(x):
        raise ValueError(f"{res} m: ZNCC {len(z)} rows, XFeat {len(x)} rows, joined {len(m)}")
    both = m.fix_x_z.notna() & m.fix_x_x.notna()
    acc = both & (np.hypot(m.fix_x_z - m.fix_x_x, m.fix_y_z - m.fix_y_x) <= AGREE_M)
    pos = m["kind"] == "pos"
    a = m[acc & pos]
    n_pos, n_neg = int(pos.sum()), int((~pos).sum())
    wrong = int((a.err_m_z > WRONG_M).sum())
    neg_acc = int((acc & ~pos).sum())
    per = []
    for s, g in m.groupby("seed"):
        gp = g["kind"] == "pos"
        ga = g[acc.loc[g.index] & gp]
        gneg = acc.loc[g.index] & ~gp
        pct = 100 * len(ga) / gp.sum()
        w = int((ga.err_m_z > WRONG_M).sum())
        per.append(dict(seed=int(s), accepted_pct=pct, wrong=w, neg=int(gneg.sum()),
                        passed=bool(pct >= 30 and w == 0 and gneg.sum() <= 0.01 * (~gp).sum())))
    per = pd.DataFrame(per)
    return {
        "res_m": res, "seeds": len(per), "test_photos": n_pos, "negatives": n_neg,
        "accepted": len(a), "accepted_pct_mean": per.accepted_pct.mean(),
        "accepted_pct_min": per.accepted_pct.min(), "accepted_pct_max": per.accepted_pct.max(),
        "wrong_gt10": wrong, "wrong_rate_pct": 100 * wrong / max(len(a), 1),
        "wrong_upper95_pct": 100 * upper95(wrong, len(a)),
        "neg_accepted": neg_acc, "neg_upper95_pct": 100 * upper95(neg_acc, n_neg),
        "median_err_m": float(a.err_m_z.median()), "p90_err_m": float(a.err_m_z.quantile(0.9)),
        "max_err_m": float(a.err_m_z.max()), "seeds_passing": int(per.passed.sum()),
        "max_wrong_in_one_seed": int(per.wrong.max()),
    }


def main() -> None:
    cons, seedsx = X / "stage3_rows_cons_test.csv", X / "stage3_rows_seedsx_test.csv"
    out = [evaluate(0.5, rows(cons, "zncc", 0.5), rows(cons, "xfeat", 0.5)),
           evaluate(0.25, rows(cons, "zncc", 0.25), rows(seedsx, "xfeat", 0.25))]
    ref_rate = 100 * REFERENCE["wrong"] / REFERENCE["accepted"]
    for r in out:
        r["rule_success"] = bool(r["wrong_rate_pct"] <= ref_rate and r["accepted_pct_mean"] >= 30)
    (X / "consensus_summary.json").write_text(json.dumps({"reference": REFERENCE, "results": out}, indent=1))
    print(pd.DataFrame(out).round(3).T.to_string())
    print(f"reference wrong rate: {ref_rate:.3f}% ({REFERENCE['rule']})")


if __name__ == "__main__":
    main()

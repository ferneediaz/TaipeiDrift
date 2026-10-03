"""Aggregate per-run metrics into summary tables with confidence intervals."""
from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

FILTER_LABELS = {"tercom": "TERCOM", "mpf_baseline": "MPF baseline", "mpf_gated": "MPF gated",
                 "mpf_proposed": "MPF proposed"}


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval for a proportion."""
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (c - h, c + h)


def bootstrap_median_ci(x: np.ndarray, n_boot: int = 2000, seed: int = 0) -> tuple[float, float]:
    x = x[np.isfinite(x)]
    if x.size < 2:
        return (math.nan, math.nan)
    rng = np.random.default_rng(seed)
    b = np.median(rng.choice(x, (n_boot, x.size)), axis=1)
    return (float(np.percentile(b, 2.5)), float(np.percentile(b, 97.5)))


def load_rows(folder: Path) -> list[dict]:
    with open(folder / "metrics.csv") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            if k in ("route", "point", "filter"):
                continue
            try:
                r[k] = float(v) if v not in ("True", "False") else (v == "True")
            except (TypeError, ValueError):
                pass
    return rows


def summarize(folder: Path) -> list[dict]:
    """Write summary.csv: one row per (route, point, filter)."""
    rows = load_rows(folder)
    groups = defaultdict(list)
    for r in rows:
        groups[(r["route"], r["point"], r["filter"])].append(r)
    out = []
    for (route, point, filt), g in sorted(groups.items()):
        n = len(g)
        get = lambda k: np.array([float(x[k]) for x in g])
        div = int(sum(bool(x["diverged"]) for x in g))
        ff = int(sum(bool(x["false_fix"]) for x in g))
        lo, hi = bootstrap_median_ci(get("rmse_post"))
        dl, dh = wilson(div, n)
        conv = get("conv_time")
        out.append(dict(route=route, point=point, filter=filt, n_runs=n,
                        rmse_post_median=float(np.median(get("rmse_post"))), rmse_post_ci_lo=lo, rmse_post_ci_hi=hi,
                        cep50_median=float(np.median(get("cep50"))), cep95_median=float(np.median(get("cep95"))),
                        final_err_median=float(np.median(get("final_err"))),
                        conv_time_median=float(np.nanmedian(conv)) if np.isfinite(conv).any() else math.nan,
                        converged_frac=float(np.isfinite(conv).mean()),
                        divergence_rate=div / n, divergence_ci_lo=dl, divergence_ci_hi=dh, false_fix_rate=ff / n,
                        nees_median=float(np.median(get("nees_median"))),
                        usable_ground_frac=float(np.mean(get("usable_ground_frac"))),
                        runtime_ms=float(np.mean(get("runtime_ms"))), realtime_factor=float(np.mean(get("realtime_factor"))),
                        n_reinit_mean=float(np.mean(get("n_reinit")))))
    with open(folder / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    return out

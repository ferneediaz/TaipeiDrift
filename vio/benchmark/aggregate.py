"""Aggregate per-trajectory results: distributions, paired comparisons, drift strata, oracle gaps.

All statistics are NaN-aware: a trajectory that lacks a value (method unavailable,
horizon beyond the flight) is left out of that statistic, and ``n`` says how many
were used.
"""
from __future__ import annotations

import numpy as np

METHODS = ("imu_only", "visual", "ground_truth_attitude_oracle")


def describe(values) -> dict:
    """n, mean, median, std, p25, p75, p90, p95, min, max over the finite values."""
    v = np.asarray([x for x in values if x is not None], dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {"n": 0, **{k: None for k in ("mean", "median", "std", "p25", "p75", "p90", "p95", "min", "max")}}
    return {"n": int(v.size), "mean": float(v.mean()), "median": float(np.median(v)), "std": float(v.std(ddof=1)) if v.size > 1 else 0.0,
            "p25": float(np.percentile(v, 25)), "p75": float(np.percentile(v, 75)), "p90": float(np.percentile(v, 90)),
            "p95": float(np.percentile(v, 95)), "min": float(v.min()), "max": float(v.max())}


def value(r: dict, method: str, quantity: str, field: str) -> float:
    m = r["methods"].get(method)
    if not m:
        return float("nan")
    x = m[quantity].get(field)
    return float("nan") if x is None else float(x)


def distributions(results: list[dict], horizons: list[str]) -> dict:
    out = {}
    for q in ("position_m", "attitude_deg"):
        out[q] = {}
        for method in METHODS:
            out[q][method] = {h: describe([value(r, method, q, h) for r in results]) for h in horizons}
    return out


def classify(delta: float, reference: float, abs_tol: float, rel_tol: float) -> str:
    """'improved' (delta < 0), 'worsened' (delta > 0), or 'unchanged' within max(abs_tol, rel_tol * reference)."""
    if not np.isfinite(delta):
        return "unavailable"
    tol = max(abs_tol, rel_tol * abs(reference)) if np.isfinite(reference) else abs_tol
    if abs(delta) <= tol:
        return "unchanged"
    return "improved" if delta < 0 else "worsened"


def paired(results: list[dict], horizons: list[str], tolerances: dict, min_denominator: dict) -> dict:
    """visual - imu_only per trajectory, where both exist."""
    out = {}
    for q in ("position_m", "attitude_deg"):
        tol, den = tolerances[q], min_denominator[q]
        out[q] = {}
        for h in horizons:
            rows = []
            for r in results:
                a, b = value(r, "imu_only", q, h), value(r, "visual", q, h)
                if np.isfinite(a) and np.isfinite(b):
                    d = b - a
                    pct = 100.0 * d / a if a >= den else float("nan")
                    rows.append((d, pct, classify(d, a, tol["abs"], tol["rel"])))
            counts = {c: sum(1 for x in rows if x[2] == c) for c in ("improved", "worsened", "unchanged")}
            n = len(rows)
            d = np.array([x[0] for x in rows]) if rows else np.array([])
            p = np.array([x[1] for x in rows]) if rows else np.array([])
            p = p[np.isfinite(p)]
            out[q][h] = {
                "n_pairs": n, **counts,
                **{f"{c}_percent": (100.0 * counts[c] / n if n else None) for c in counts},
                "median_change": float(np.median(d)) if n else None,
                "median_abs_change": float(np.median(np.abs(d))) if n else None,
                "mean_change": float(np.mean(d)) if n else None,
                "median_percent_change": float(np.median(p)) if p.size else None,
                "n_percent_defined": int(p.size), "percent_min_denominator": den,
                "tolerance": tol,
            }
    return out


def drift_strata(results: list[dict], cfg: dict, tolerances: dict) -> dict:
    """Group paired trajectories by IMU-only final attitude error and compare visual vs IMU per group.

    ``cfg['thresholds_deg']`` = [low_max, high_min] gives fixed groups; otherwise tertiles of the
    paired trajectories are used. Descriptive only.
    """
    pairs = [r for r in results if r["methods"].get("visual")]
    drift = np.array([value(r, "imu_only", "attitude_deg", "final") for r in pairs])
    if not pairs:
        return {"method": None, "groups": {}}
    if cfg.get("thresholds_deg"):
        lo, hi = cfg["thresholds_deg"]
        method = f"fixed thresholds: low < {lo} deg <= medium < {hi} deg <= high"
    else:
        lo, hi = np.percentile(drift, [100 / 3, 200 / 3])
        method = f"tertiles of IMU-only final attitude error: {lo:.2f} and {hi:.2f} deg"
    groups = {"low": drift < lo, "medium": (drift >= lo) & (drift < hi), "high": drift >= hi}
    out = {"method": method, "boundaries_deg": [float(lo), float(hi)], "groups": {}}
    for g, mask in groups.items():
        sel = [p for p, m in zip(pairs, mask) if m]
        entry = {"n": len(sel), "imu_attitude_final_deg": describe([value(r, "imu_only", "attitude_deg", "final") for r in sel])}
        for q in ("attitude_deg", "position_m"):
            t = tolerances[q]
            deltas = [value(r, "visual", q, "final") - value(r, "imu_only", q, "final") for r in sel]
            cls = [classify(d, value(r, "imu_only", q, "final"), t["abs"], t["rel"]) for d, r in zip(deltas, sel)]
            entry[q] = {"improved": cls.count("improved"), "worsened": cls.count("worsened"), "unchanged": cls.count("unchanged"),
                        "median_change": float(np.median(deltas)) if deltas else None,
                        "median_ratio_visual_over_imu": float(np.median([value(r, "visual", q, "final") / max(value(r, "imu_only", q, "final"), 1e-9) for r in sel])) if sel else None}
        out["groups"][g] = entry
    return out


def oracle_gap(results: list[dict], min_denominator_m: float) -> dict:
    """Final position error above the perfect-attitude oracle, per trajectory and in aggregate."""
    rows = []
    for r in results:
        imu = value(r, "imu_only", "position_m", "final")
        vis = value(r, "visual", "position_m", "final")
        orc = value(r, "ground_truth_attitude_oracle", "position_m", "final")
        rows.append({"key": r["key"], "imu_minus_oracle_m": imu - orc, "visual_minus_oracle_m": vis - orc,
                     "oracle_reduction_vs_imu_percent": 100.0 * (imu - orc) / imu if imu >= min_denominator_m else float("nan"),
                     "visual_share_of_gap_closed_percent": (100.0 * (imu - vis) / (imu - orc)
                                                            if np.isfinite(vis) and (imu - orc) >= min_denominator_m else float("nan"))})
    return {"per_trajectory": rows,
            "imu_minus_oracle_m": describe([x["imu_minus_oracle_m"] for x in rows]),
            "visual_minus_oracle_m": describe([x["visual_minus_oracle_m"] for x in rows]),
            "oracle_reduction_vs_imu_percent": describe([x["oracle_reduction_vs_imu_percent"] for x in rows]),
            "visual_share_of_gap_closed_percent": describe([x["visual_share_of_gap_closed_percent"] for x in rows]),
            "n_oracle_worse_than_imu": sum(1 for x in rows if x["imu_minus_oracle_m"] < 0),
            "n_visual_better_than_oracle": sum(1 for x in rows if np.isfinite(x["visual_minus_oracle_m"]) and x["visual_minus_oracle_m"] < 0),
            "min_denominator_m": min_denominator_m}


def visual_validity_correlation(results: list[dict]) -> dict:
    """Spearman correlation between visual valid fraction and attitude / position change (visual - imu)."""
    from scipy.stats import spearmanr
    rows = [(r["visual_statistics"]["valid_fraction"],
             value(r, "visual", "attitude_deg", "final") - value(r, "imu_only", "attitude_deg", "final"),
             value(r, "visual", "position_m", "final") - value(r, "imu_only", "position_m", "final"),
             value(r, "imu_only", "attitude_deg", "final"))
            for r in results if r.get("visual_statistics")]
    if len(rows) < 3:
        return {"n": len(rows)}
    a = np.array(rows)
    sp = lambda x, y: float(spearmanr(x, y).statistic)  # noqa: E731
    return {"n": len(rows),
            "spearman_valid_fraction_vs_attitude_change": sp(a[:, 0], a[:, 1]),
            "spearman_valid_fraction_vs_position_change": sp(a[:, 0], a[:, 2]),
            "spearman_imu_attitude_drift_vs_attitude_change": sp(a[:, 3], a[:, 1]),
            "spearman_imu_attitude_drift_vs_relative_attitude_change": sp(a[:, 3], a[:, 1] / np.maximum(a[:, 3], 1e-9))}

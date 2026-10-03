"""Out-of-sample acceptance rules and false-accept bounds for r_map_benchmark outputs.

Every threshold is fitted on calibration folds that exclude the evaluated fold
(leave-one-site-out when >= 3 sites, else north/south halves of each site).
A "false accept" is an accepted fix on a negative (camera scene outside the map
window) or on a positive whose error exceeds 25 m. Bounds are one-sided 95 %
Clopper-Pearson on independent clusters (a cluster = one map window x one camera
source scene, pooled over all degradation conditions, so conditions do not count
as independent trials).

  .venv/bin/python experiments/r_integrity.py data/processed/r_map_benchmark
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import beta

KEY = ["site", "pair", "centre_id", "condition", "kind", "query_site", "truth_x", "truth_y"]
GOOD_M, BAD_M = 10., 25.


def upper95(k, n):
    return float(beta.ppf(.95, k + 1, n - k)) if n else np.nan


def folds(df):
    sites = sorted(df.site.unique())
    if len(sites) >= 3:
        return df.site
    # Fold by the map window (positive centre of the unit), never by the negative's source.
    pos = df[df.present & (df.condition == "aligned")]
    centre_y = pos.groupby(["site", "centre_id"]).truth_y.first()
    y = pd.Series(list(zip(df.site, df.centre_id)), index=df.index).map(centre_y)
    split = centre_y.groupby(level=0).median()
    return df.site + np.where(y.values > df.site.map(split).values, ":south", ":north")


def false_row(d):
    return d.fixed & (~d.present | (d.error_m > BAD_M))


def fit_threshold(d, feature):
    cal_false = d[false_row(d)][feature].dropna()
    return float(np.nextafter(cal_false.max(), np.inf)) if len(cal_false) else -np.inf


def single_rule(df, method, feature):
    d = df[df.method == method].copy()
    d["accept"] = False
    thresholds = {}
    for fold in d.fold.unique():
        cal = d[d.fold != fold]
        t = fit_threshold(cal, feature)
        thresholds[fold] = t
        sel = d.fold == fold
        d.loc[sel, "accept"] = d.loc[sel, "fixed"] & (d.loc[sel, feature] >= t)
    d["rule"] = f"{method}[{feature}]"
    return d, thresholds


def agreement_rule(df, a, b, tol=GOOD_M, use="b"):
    """Accept when two methods both produce a fix within tol metres of each other. No fitted threshold."""
    da = df[df.method == a].set_index(KEY)
    db = df[df.method == b].set_index(KEY)
    j = da.join(db, lsuffix="_a", rsuffix="_b", how="inner")
    gap = np.hypot(j.est_x_a - j.est_x_b, j.est_y_a - j.est_y_b)
    out = j[[c for c in j.columns if c.endswith("_" + use)]].copy()
    out.columns = [c[:-2] for c in out.columns]
    out = out.reset_index()
    out["accept"] = (j.fixed_a & j.fixed_b & (gap <= tol)).values
    out["latency_ms"] = (j.latency_ms_a + j.latency_ms_b).values
    out["rule"] = f"agree({a},{b})"
    return out


def any_agreement(df, family_a, family_b, tol=GOOD_M):
    """Accept if any method of family A agrees with any method of family B (estimate from B)."""
    parts = [agreement_rule(df, a, b, tol) for a in family_a for b in family_b
             if a in df.method.values and b in df.method.values]
    if not parts:
        return None
    cat = pd.concat(parts)
    # one row per scene: accepted if any combination agrees; keep the first accepting estimate
    cat = cat.sort_values("accept", ascending=False)
    first = cat.groupby(KEY, as_index=False).first()
    first["latency_ms"] = cat.groupby(KEY).latency_ms.max().values  # lower bound: parallel cost not summed
    first["rule"] = f"any_agree({'|'.join(family_a)};{'|'.join(family_b)})"
    return first


def summarise(d):
    rows = []
    for (rule, condition), g in d.groupby(["rule", "condition"]):
        rows.append(row(rule, condition, g))
    for rule, g in d.groupby("rule"):
        rows.append(row(rule, "ALL", g))
    return pd.DataFrame(rows)


def row(rule, condition, g):
    pos, neg = g[g.present], g[~g.present]
    acc_pos = pos[pos.accept]
    false = g[g.accept & (~g.present | (g.error_m > BAD_M))]
    clusters = g.assign(cluster=g.site + "|" + g.pair + "|" + g.centre_id.astype(str) + "|" + g.kind)
    clusters["false"] = g.accept & (~g.present | (g.error_m > BAD_M))
    per = clusters.groupby("cluster")["false"].any()
    k, n = int(per.sum()), int(len(per))
    return dict(rule=rule, condition=condition, positives=len(pos), negatives=len(neg),
                fix_rate_10m=float((acc_pos.error_m <= GOOD_M).sum() / max(1, len(pos))),
                accepted_pos=len(acc_pos), accepted_wrong_25m=int((acc_pos.error_m > BAD_M).sum()),
                negative_accepts=int(neg.accept.sum()), false_rows=len(false),
                false_clusters=k, clusters=n, false_cluster_rate_up95=upper95(k, n),
                median_err_accepted=float(acc_pos.error_m.median()) if len(acc_pos) else np.nan,
                p95_err_accepted=float(acc_pos.error_m.quantile(.95)) if len(acc_pos) else np.nan,
                latency_p50_ms=float(g.latency_ms.median()))


UNION_FEATURES = ("xfeat_affine", "xfeat_rot4")


def union_rule(df, tol=GOOD_M, features=UNION_FEATURES, name="UNION"):
    """Recommended module: accept if any uncalibrated check passes and all passing estimates agree.
    Checks: ZNCC-family quad>=3 (each hypothesis method) or ZNCC-family x feature-method agreement.
    v2 (after the first all-site run): the agreement path uses only `features`; xfeat_homography was
    dropped because a 6-inlier homography agreed with a weak ZNCC peak on a featureless negative."""
    methods = sorted(set(df.method))
    zf = [m for m in methods if m.startswith("zncc")]
    ff = [m for m in methods if m in features]
    df = df[df.method.isin(zf + ff)]
    wide = df.pivot_table(index=KEY, columns="method", values=["est_x", "est_y", "fixed", "quad_n", "latency_ms"],
                          aggfunc="first")
    base = df[df.method == zf[0]].set_index(KEY)
    cands = []
    for z in zf:
        ok = wide["fixed", z].fillna(False).astype(bool) & (wide["quad_n", z] >= 3)
        cands.append((ok, wide["est_x", z], wide["est_y", z]))
        for f in ff:
            if ("fixed", f) not in wide.columns:
                continue
            gap = np.hypot(wide["est_x", z] - wide["est_x", f], wide["est_y", z] - wide["est_y", f])
            ok = wide["fixed", z].fillna(False).astype(bool) & wide["fixed", f].fillna(False).astype(bool) & (gap <= tol)
            cands.append((ok, wide["est_x", f], wide["est_y", f]))
    any_ok = pd.concat([c[0] for c in cands], axis=1).fillna(False).any(axis=1)
    xs = pd.concat([c[1].where(c[0]) for c in cands], axis=1)
    ys = pd.concat([c[2].where(c[0]) for c in cands], axis=1)
    consistent = ((xs.max(axis=1) - xs.min(axis=1)).fillna(0) <= tol) & ((ys.max(axis=1) - ys.min(axis=1)).fillna(0) <= tol)
    est_x = xs.bfill(axis=1).iloc[:, 0]
    est_y = ys.bfill(axis=1).iloc[:, 0]
    out = base.loc[wide.index].reset_index()
    out["accept"] = (any_ok & consistent).values
    out["fixed"] = out["accept"]
    out["est_x"], out["est_y"] = est_x.values, est_y.values
    out["error_m"] = np.hypot(out.est_x - out.truth_x, out.est_y - out.truth_y)
    out["latency_ms"] = wide["latency_ms"].sum(axis=1).values
    out["rule"] = name
    return out


def auroc(df, method, feature):
    """Over all attempts of the method; attempts without a fix rank lowest."""
    d = df[df.method == method].copy()
    d[feature] = d[feature].where(d.fixed, -np.inf).fillna(-np.inf)
    good = d.present & (d.error_m <= GOOD_M)
    bad = ~d.present | (d.error_m > BAD_M) | (d.present & ~d.fixed)
    d, good, bad = d[good | bad], good[good | bad], bad[good | bad]
    if good.sum() == 0 or bad.sum() == 0:
        return np.nan
    r = d[feature].rank()
    return float((r[good].sum() - good.sum() * (good.sum() + 1) / 2) / (good.sum() * bad.sum()))


def quad_sweep(df):
    """Uncalibrated quad rule at other tolerances: >= k of 4 sub-templates within tol m of the full fix."""
    if "quad_d3" not in df.columns:
        return None
    out = []
    for method in [m for m in sorted(set(df.method)) if m.startswith("zncc")]:
        base = df[df.method == method]
        dists = base[["quad_d1", "quad_d2", "quad_d3", "quad_d4"]].to_numpy()
        for tol in (3, 4, 6, 8, 12):
            for k in (2, 3, 4):
                d = base.copy()
                d["accept"] = d.fixed & ((dists <= tol).sum(axis=1) >= k)
                r = row(f"{method}[quad{k}@{tol}m]", "ALL", d)
                out.append(dict(method=method, tol_m=tol, k=k, fix_rate_10m=r["fix_rate_10m"],
                                false_clusters=r["false_clusters"], clusters=r["clusters"],
                                up95=r["false_cluster_rate_up95"]))
    return pd.DataFrame(out)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run", type=Path)
    a = p.parse_args()
    df = pd.read_csv(a.run / "matches.csv.gz")
    df["fold"] = folds(df)
    methods = set(df.method)
    parts, thresholds = [], {}
    for method in sorted(methods):
        features = ["score"] + (["peak_ratio", "z", "margin", "quad_n", "quad_peak"] if method.startswith("zncc")
                                else ["inlier_ratio"])
        for feature in features:
            if feature in df.columns and df.loc[df.method == method, feature].notna().any():
                d, t = single_rule(df, method, feature)
                parts.append(d)
                thresholds[f"{method}[{feature}]"] = t
    if "quad_n" in df.columns:
        # Fixed, uncalibrated rules: >= 3 of 4 disjoint sub-templates agree with the full ZNCC fix.
        for method in [m for m in sorted(methods) if m.startswith("zncc")]:
            for need in (3, 4):
                d = df[df.method == method].copy()
                d["accept"] = d.fixed & (d.quad_n >= need)
                d["rule"] = f"{method}[quad>={need}]"
                parts.append(d)
    zf = [m for m in ("zncc", "zncc_yaw_scale", "zncc_heading") if m in methods]
    ff = [m for m in sorted(methods) if not m.startswith("zncc")]
    for x, y in [("zncc", "xfeat_affine"), ("zncc_yaw_scale", "xfeat_affine"), ("zncc", "zncc_yaw_scale"),
                 ("xfeat_affine", "xfeat_homography"), ("zncc_heading", "xfeat_rot4")]:
        if x in methods and y in methods:
            parts.append(agreement_rule(df, x, y))
    combo = any_agreement(df, zf, ff)
    if combo is not None:
        combo["rule"] = "any_agree(zncc-family; feature-family)"
        parts.append(combo)
    if zf and "quad_n" in df.columns:
        parts.append(union_rule(df))
        parts.append(union_rule(df, features=("xfeat_affine", "xfeat_rot4", "xfeat_homography"), name="UNION_v1"))
        if "lib:tiny_roma" in methods:
            parts.append(union_rule(df, features=UNION_FEATURES + ("lib:tiny_roma",), name="UNION+tiny_roma"))
        if "lib:aliked_lightglue" in methods:
            parts.append(union_rule(df, features=UNION_FEATURES + ("lib:aliked_lightglue",), name="UNION+aliked_lg"))
    allrules = pd.concat(parts, ignore_index=True)
    summary = summarise(allrules)
    summary.to_csv(a.run / "integrity_summary.csv", index=False)
    aucs = {f"{m}[{f}]": auroc(df, m, f) for m in sorted(methods)
            for f in ("score", "peak_ratio", "z", "margin", "quad_n", "quad_peak", "inlier_ratio")
            if f in df.columns and df.loc[df.method == m, f].notna().any()}
    land = allrules[allrules.accept & (~allrules.present | (allrules.error_m > BAD_M))] \
        .groupby(["rule", "land_cover"]).size().rename("false_rows").reset_index()
    land.to_csv(a.run / "false_accepts_by_land_cover.csv", index=False)
    sweep = quad_sweep(df)
    if sweep is not None:
        sweep.to_csv(a.run / "quad_tolerance_sweep.csv", index=False)
        print("\nquad tolerance sweep (all conditions):\n" + sweep.round(4).to_string(index=False))
    (a.run / "integrity.json").write_text(json.dumps(dict(
        folds=sorted(df.fold.unique()), thresholds=thresholds, auroc=aucs), indent=2, default=float) + "\n")
    pd.set_option("display.width", 250)
    cols = ["rule", "positives", "fix_rate_10m", "accepted_wrong_25m", "negative_accepts",
            "false_clusters", "clusters", "false_cluster_rate_up95", "latency_p50_ms"]
    print(summary[summary.condition == "ALL"][cols].round(3).to_string(index=False))
    print("\nAUROC correct(<=10 m) vs false:", json.dumps({k: round(v, 3) for k, v in aucs.items()}))
    key = [r for r in ("zncc[quad>=3]", "zncc_yaw_scale[quad>=3]", "zncc_heading[quad>=3]", "xfeat_affine[score]",
                       "xfeat_homography[score]", "lib:tiny_roma[score]", "lib:aliked_lightglue[score]",
                       "lib:xfeat_lighterglue[score]", "UNION", "UNION_v1", "UNION+tiny_roma",
                       "UNION+aliked_lg") if r in set(summary.rule)]
    piv = summary[summary.rule.isin(key)].pivot(index="condition", columns="rule", values="fix_rate_10m")
    piv.columns = [c[:24] for c in piv.columns]
    print("\nfix rate (accepted and <=10 m) per condition:\n" + piv[[c[:24] for c in key]].round(2).to_string())
    if len(land):
        print("\nfalse accepts by land cover:\n" + land.to_string(index=False))


if __name__ == "__main__":
    main()

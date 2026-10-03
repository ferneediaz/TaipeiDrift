#!/usr/bin/env python3
"""Stage 3: single-photo position fixes (Level 1) on the real Tuniu photos, scored against RTK.

Estimator inputs per photo: the photo, its time, DJI fused attitude (gimbal angles + pre-cut boresight),
a SIMULATED prior (RTK + uniform +-40 m per axis), a SIMULATED barometer (x_tuniu_geo.simulated_baro),
the Copernicus DEM and the map. RTK is used only for: the simulated prior/baro generators, pre-cut
calibration, and scoring (in this parent process, after the workers return).

  run        rows for one split (precut | test), maps x resolutions x methods x query config x seed
  summarize  gates fitted on pre-cut rows only, metrics on test rows -> stage3_summary_<tag>.csv
  latency    sequential single-thread timing on 20 test photos (Mac, not Jetson)

Examples:
  .venv/bin/python experiments/x3_tuniu_fix.py run --split precut --tag main
  .venv/bin/python experiments/x3_tuniu_fix.py run --split test --tag main
  .venv/bin/python experiments/x3_tuniu_fix.py summarize --tag main
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from multiprocessing import get_context
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import x_tuniu_geo as G  # noqa: E402

PRIOR_ERR_M = 40.0
NEG_MIN_CHEB_M = 300.0
NEG_MIN_VALID = 0.5
GOOD_M = 10.0
METHODS_DEFAULT = ["zncc", "aliked-lightglue", "disk-lightglue", "xfeat"]
FIELDS_KEEP = ["score", "second", "z", "hyp_angle", "hyp_scale", "quad_n", "quad_d", "matches", "inliers",
               "inlier_ratio", "h_scale", "h_rot", "h_aniso", "gate", "latency_s"]


def seed_of(*parts) -> int:
    return int.from_bytes(hashlib.sha256("|".join(map(str, parts)).encode()).digest()[:8], "little")


# ----------------------------------------------------------------------------- simulated inputs

def simulated_inputs(seed: int) -> pd.DataFrame:
    """Per frame: SIMULATED prior (x, y) and SIMULATED baro altitude. Built from RTK in the parent."""
    tr = G.truth_xy()
    cut = G.protocol()["cut_s"]
    baro = G.simulated_baro(tr.t_s.to_numpy(), tr.alt_ell.to_numpy(), seed_of("baro", seed), cut)
    rng = np.random.default_rng(seed_of("prior", seed))
    off = rng.uniform(-PRIOR_ERR_M, PRIOR_ERR_M, (len(tr), 2))
    yaw_sign = np.where(np.random.default_rng(seed_of("yawsign", seed)).random(len(tr)) < 0.5, -1.0, 1.0)
    return pd.DataFrame({"frame": tr.frame, "prior_x": tr.x + off[:, 0], "prior_y": tr.y + off[:, 1],
                         "baro_alt": baro, "yaw_sign": yaw_sign})


def negative_priors(frames, sim: pd.DataFrame, seed: int) -> dict:
    """For each frame: list of (map_key, prior_x, prior_y) negatives. Chosen on the 1 m rasters:
    Chebyshev >= 300 m from the TRUE nadir (same map) and >= 50 % valid reference window."""
    tr = G.truth_xy().set_index("frame")
    maps = {k: G.MapRaster(G.map_path(k, 1.0)) for k in ("main", "y2021", "negs")}
    half = 150    # m (= px at 1 m/px): half side of a square approximating a reference window
    cands = {}
    for k, m in maps.items():
        H, W = m.valid.shape
        ii = np.pad(m.valid.astype(np.int32).cumsum(0).cumsum(1), ((1, 0), (1, 0)))
        pts = []
        for r in range(0, H, 10):
            for c in range(0, W, 10):
                r0, r1, c0, c1 = max(0, r - half), min(H, r + half), max(0, c - half), min(W, c + half)
                frac = (ii[r1, c1] - ii[r0, c1] - ii[r1, c0] + ii[r0, c0]) / float(4 * half * half)
                if frac >= NEG_MIN_VALID:
                    pts.append(m.to_xy(c, r))
        cands[k] = np.array(pts, float).reshape(-1, 2)
    out = {}
    for f in frames:
        t = tr.loc[f]
        rng = np.random.default_rng(seed_of("neg", seed, f))
        negs = []
        for k in ("main", "y2021"):
            c = cands[k]
            far = c[np.maximum(abs(c[:, 0] - t.x), abs(c[:, 1] - t.y)) >= NEG_MIN_CHEB_M]
            if len(far):
                negs.append((k, "neg_same", *far[rng.integers(len(far))]))
        c = cands["negs"]
        for i in rng.choice(len(c), size=2, replace=False):
            negs.append(("negs", "neg_other", *c[i]))
        out[f] = negs
    return out


# ----------------------------------------------------------------------------- worker

_W: dict = {}


def _worker_init():
    import cv2
    cv2.setNumThreads(1)
    try:
        import torch
        torch.set_num_threads(1)
    except ImportError:
        pass


def _map(key, res):
    k = (key, res)
    if k not in _W:
        for old in [x for x in _W if isinstance(x, tuple) and x[1] != res]:
            del _W[old]
        _W[k] = G.MapRaster(G.map_path(key, res))
    return _W[k]


def run_unit(u: dict) -> list[dict]:
    import x_tuniu_match as M
    cam = _W.setdefault("cam", G.Camera())
    ph = _W.setdefault("ph", G.photos())
    cal = _W.setdefault("cal", json.loads((G.XOUT / "stage1_calibration.json").read_text()))
    att = ph.iloc[u["frame"] - 1]
    cfg = M.QueryConfig(**u["cfg"])
    factor = G.reduce_factor(u["res"])
    t0 = time.perf_counter()
    img = G.load_gray(att.path, factor)
    yaw_err = {"yaw3": 3.0, "yaw7": 7.0}.get(cfg.attitude, 0.0) * u["yaw_sign"]
    q = M.build_query(img, factor, att, cam, u["res"], cfg, u["baro_alt"], (u["prior_x"], u["prior_y"]),
                      u.get("oracle_alt"), cal["takeoff_alt_ell_m"], yaw_err, cal["boresight_deg"])
    t_rect = time.perf_counter() - t0
    rows = []
    for (key, kind, px, py) in [(u["map"], "pos", u["prior_x"], u["prior_y"])] + u["negs"]:
        ref = M.reference_for(_map(key, u["res"]), q, (px, py))
        ref_valid = float(ref["valid"].mean())
        for method in u["methods"]:
            r = M.run_method(method, q, ref)
            fix = r.pop("fix")
            row = dict(frame=u["frame"], split=u["split"], seed=u["seed"], cfg=cfg.name, res=u["res"],
                       map=u["map"], method=method, kind=kind, ref_map=key, neg_prior_x=px, neg_prior_y=py,
                       fix_x=np.nan if fix is None else fix[0], fix_y=np.nan if fix is None else fix[1],
                       ref_valid=ref_valid, rect_s=t_rect, height_used=q["height_used"])
            row.update({k: r.get(k, np.nan) for k in FIELDS_KEEP})
            rows.append(row)
    return rows


def build_units(args) -> list[dict]:
    prot = G.protocol()
    frames = prot["precut_frames"] if args.split == "precut" else prot["test_frames"]
    if args.frames:
        frames = [f for f in frames if f in set(args.frames)]
    units = []
    tr = G.truth_xy().set_index("frame")
    for seed in args.seeds:
        sim = simulated_inputs(seed).set_index("frame")
        negs = negative_priors(frames, sim, seed) if not args.no_negatives else {f: [] for f in frames}
        for cfg in args.cfgs:
            for res in args.res:
                for mp in args.maps:
                    for f in frames:
                        s = sim.loc[f]
                        own = [n for n in negs[f] if n[0] in (mp, "negs")]
                        units.append(dict(frame=f, split=args.split, seed=seed, cfg=cfg, res=res, map=mp,
                                          methods=args.methods, prior_x=s.prior_x, prior_y=s.prior_y,
                                          baro_alt=s.baro_alt, yaw_sign=s.yaw_sign,
                                          oracle_alt=float(tr.loc[f].alt_ell) if cfg.get("height") == "rtk_dem" else None,
                                          negs=own))
    return units


def parse_cfg(s: str) -> dict:
    d = dict(kv.split("=") for kv in s.split(",")) if s else {}
    return {"height": d.get("h", "baro_dem"), "attitude": d.get("att", "dji"), "ground": d.get("gnd", "dem_prior")}


def cmd_run(args):
    args.cfgs = [parse_cfg(c) for c in args.cfg]
    units = build_units(args)
    out = G.XOUT / f"stage3_rows_{args.tag}_{args.split}.csv"
    print(f"{len(units)} units ({args.split}, tag {args.tag}), methods {args.methods}, res {args.res}, "
          f"maps {args.maps}, cfgs {[M for M in args.cfg]}, seeds {args.seeds}, workers {args.workers}", flush=True)
    t0 = time.time()
    rows = []
    # resolution-major order keeps one map resolution per worker cache
    units.sort(key=lambda u: (u["res"], u["map"], u["frame"]))
    ctx = get_context("spawn")
    with ctx.Pool(args.workers, initializer=_worker_init, maxtasksperchild=200) as pool:
        partial = out.with_suffix(".partial.csv")
        for i, r in enumerate(pool.imap(run_unit, units, chunksize=1)):
            rows.extend(r)
            if (i + 1) % max(1, len(units) // 20) == 0:
                print(f"  {i + 1}/{len(units)} units, {time.time() - t0:.0f} s", flush=True)
                pd.DataFrame(rows).to_csv(partial, index=False)   # survives a kill; not read back
        partial.unlink(missing_ok=True)
    df = score(pd.DataFrame(rows))
    if out.exists() and args.append:
        df = pd.concat([pd.read_csv(out), df], ignore_index=True)
    df.to_csv(out, index=False)
    print(f"wrote {out} ({len(df)} rows) in {time.time() - t0:.0f} s")


def score(df: pd.DataFrame) -> pd.DataFrame:
    """Evaluator: map-offset correction (pre-cut calibration) and error vs RTK."""
    cal = json.loads((G.XOUT / "stage1_calibration.json").read_text())
    tr = G.truth_xy().set_index("frame")
    off = {k: (v["de_m"], v["dn_m"]) for k, v in cal["map_offsets"].items()}
    off["negs"] = (0.0, 0.0)
    de = df.ref_map.map(lambda k: off[k][0])
    dn = df.ref_map.map(lambda k: off[k][1])
    df["fixc_x"], df["fixc_y"] = df.fix_x - de, df.fix_y - dn
    df["err_m"] = np.hypot(df.fixc_x - tr.loc[df.frame, "x"].to_numpy(), df.fixc_y - tr.loc[df.frame, "y"].to_numpy())
    df["present"] = df.kind == "pos"
    return df


# ----------------------------------------------------------------------------- summary

def upper95(k, n):
    from scipy.stats import beta
    return float(beta.ppf(.95, k + 1, n - k)) if n else np.nan


def rule_list(methods):
    out = [("zncc", "zncc", "score"), ("zncc+quad>=3", "zncc", "quad")] if "zncc" in methods else []
    return out + [(m, m, "inliers") for m in methods if m != "zncc"]


def fit_threshold(pre: pd.DataFrame, feature: str) -> float:
    """Smallest value above every pre-cut false fix (negative with a fix, or positive > 10 m off)."""
    has = pre.fix_x.notna()
    false = has & (~pre.present | (pre.err_m > GOOD_M))
    if feature == "inliers":
        false &= pre.gate == "ok"
    vals = pre.loc[false, feature].dropna()
    return float(np.nextafter(vals.max(), np.inf)) if len(vals) else -np.inf


def accepted(d: pd.DataFrame, kind: str, thr: float) -> pd.Series:
    has = d.fix_x.notna()
    if kind == "quad":
        return has & (d.quad_n >= 3)
    if kind == "inliers":
        return has & (d.gate == "ok") & (d.inliers >= thr)
    return has & (d.score >= thr)


ORIN = {  # INFERENCE from published numbers; see report
    "zncc": "yes (FFT/NPP template matching, few ms per hypothesis at <=0.5 m/px)",
    "zncc+quad>=3": "yes (4 extra template matches)",
    "xfeat": "yes (XFeat paper: real-time on embedded CPU; TensorRT ports run >30 fps on Orin)",
    "aliked-lightglue": "probably at ~1 Hz (LightGlue-ONNX/TensorRT ports report tens of ms per pair on Orin-class GPUs)",
    "disk-lightglue": "probably at ~1 Hz (heavier extractor than ALIKED)",
    "roma": "no (dense transformer, seconds per pair on desktop GPUs)",
}


def summarize(tag: str, write=True) -> pd.DataFrame:
    pre = pd.read_csv(G.XOUT / f"stage3_rows_{tag}_precut.csv")
    test = pd.read_csv(G.XOUT / f"stage3_rows_{tag}_test.csv")
    n_all = len(G.protocol()["test_frames"])
    rows = []
    for (cfg, res, mp, seed), tg in test.groupby(["cfg", "res", "map", "seed"]):
        pg = pre[(pre.cfg == cfg) & (pre.res == res) & (pre["map"] == mp) & (pre.seed == seed)]
        for name, method, kind in rule_list(sorted(tg.method.unique())):
            t, p = tg[tg.method == method], pg[pg.method == method]
            if t.empty:
                continue
            thr = np.nan if kind == "quad" else fit_threshold(p, kind if kind != "score" else "score")
            pos, neg = t[t.present], t[~t.present]
            n_test_photos = pos.frame.nunique()   # 225, or 75 for the SUBSET runs
            acc_pos = accepted(pos, kind, thr)
            acc_neg = accepted(neg, kind, thr)
            ea = pos.err_m[acc_pos]
            kneg, nneg = int(acc_neg.sum()), len(neg)
            rows.append(dict(cfg=cfg, res=res, map=mp, seed=seed, method=name, gate=kind,
                             threshold=thr, test_photos=n_test_photos, subset=n_test_photos < n_all, attempts=len(pos),
                             raw_within5_pct=100 * (pos.err_m <= 5).sum() / n_test_photos,
                             raw_within10_pct=100 * (pos.err_m <= 10).sum() / n_test_photos,
                             accepted=int(acc_pos.sum()), accepted_pct=100 * acc_pos.sum() / n_test_photos,
                             accepted_wrong_gt10=int((ea > GOOD_M).sum()),
                             neg_accepted=kneg, negatives=nneg, neg_accepted_pct=100 * kneg / max(1, nneg),
                             neg_upper95_pct=100 * upper95(kneg, nneg),
                             median_err_acc_m=float(ea.median()) if len(ea) else np.nan,
                             p90_err_acc_m=float(ea.quantile(.9)) if len(ea) else np.nan,
                             max_err_acc_m=float(ea.max()) if len(ea) else np.nan,
                             match_s_median=float(t.latency_s.median()),
                             orin_realtime_INFERENCE=ORIN.get(name, ORIN.get(method, "?"))))
    s = pd.DataFrame(rows)
    s["pass_level1"] = (s["map"] == "main") & (s.accepted_pct >= 30) & (s.accepted_wrong_gt10 == 0) & \
                       (s.neg_accepted_pct <= 1.0)
    if write:
        s.to_csv(G.XOUT / f"stage3_summary_{tag}.csv", index=False)
    return s


def cmd_summarize(args):
    s = summarize(args.tag)
    pd.set_option("display.width", 250)
    cols = ["cfg", "map", "res", "seed", "method", "threshold", "raw_within5_pct", "raw_within10_pct", "accepted",
            "accepted_pct", "accepted_wrong_gt10", "neg_accepted", "negatives", "neg_upper95_pct",
            "median_err_acc_m", "p90_err_acc_m", "match_s_median", "pass_level1"]
    print(s[cols].round(2).to_string(index=False))
    if s.seed.nunique() > 1:
        print("\nacross seeds:")
        print(seed_table(s).to_string(index=False))


def seed_table(s: pd.DataFrame) -> pd.DataFrame:
    """Per config x method: accepted % (mean/min/max), seeds passing, pooled wrong and negative counts."""
    rows = []
    for (cfg, mp, res, method), g in s.groupby(["cfg", "map", "res", "method"]):
        kneg, nneg = int(g.neg_accepted.sum()), int(g.negatives.sum())
        rows.append(dict(cfg=cfg, map=mp, res=res, method=method, seeds=len(g),
                         acc_pct_mean=round(g.accepted_pct.mean(), 2), acc_pct_min=round(g.accepted_pct.min(), 2),
                         acc_pct_max=round(g.accepted_pct.max(), 2), seeds_ge30pct=int((g.accepted_pct >= 30).sum()),
                         wrong_gt10_total=int(g.accepted_wrong_gt10.sum()),
                         seeds_with_wrong=int((g.accepted_wrong_gt10 > 0).sum()),
                         neg_acc_total=kneg, negatives_total=nneg, neg_upper95_pct=round(100 * upper95(kneg, nneg), 3),
                         seeds_pass=int(g.pass_level1.sum()),
                         median_err_acc_m=round(g.median_err_acc_m.median(), 2)))
    return pd.DataFrame(rows)


# ----------------------------------------------------------------------------- latency

def cmd_latency(args):
    """Sequential, one process, one thread: decode+rectify and each method, 20 test photos (Mac, not Jetson)."""
    import cv2
    import torch
    import x_tuniu_match as M
    cv2.setNumThreads(1)
    torch.set_num_threads(1)
    cam, ph = G.Camera(), G.photos()
    cal = json.loads((G.XOUT / "stage1_calibration.json").read_text())
    sim = simulated_inputs(0).set_index("frame")
    frames = G.protocol()["test_frames"][::max(1, len(G.protocol()["test_frames"]) // args.n)][:args.n]
    rows = []
    for res in args.res:
        m = G.MapRaster(G.map_path("main", res))
        for method in args.methods:
            if method != "zncc":
                M.matcher(method)
            for i, f in enumerate(frames):
                s, att = sim.loc[f], ph.iloc[f - 1]
                fac = G.reduce_factor(res)
                t0 = time.perf_counter()
                img = G.load_gray(att.path, fac)
                q = M.build_query(img, fac, att, cam, res, M.QueryConfig(), s.baro_alt, (s.prior_x, s.prior_y),
                                  None, cal["takeoff_alt_ell_m"], 0.0, cal["boresight_deg"])
                t1 = time.perf_counter()
                ref = M.reference_for(m, q, (s.prior_x, s.prior_y))
                t2 = time.perf_counter()
                M.run_method(method, q, ref)
                t3 = time.perf_counter()
                if i == 0:
                    continue   # warm-up
                rows.append(dict(res=res, method=method, frame=f, decode_rectify_s=t1 - t0, map_window_s=t2 - t1,
                                 match_s=t3 - t2, total_s=t3 - t0, query_px=q["patch"].size, ref_px=ref["gray"].size))
            print(f"  {method} {res} m/px done", flush=True)
    df = pd.DataFrame(rows)
    out = G.XOUT / "stage3_latency_mac_1thread.csv"
    if out.exists():   # several invocations (method/resolution groups) accumulate
        old = pd.read_csv(out)
        old = old[~old.set_index(["method", "res"]).index.isin(df.set_index(["method", "res"]).index)]
        df_all = pd.concat([old, df], ignore_index=True)
    else:
        df_all = df
    df_all.to_csv(out, index=False)
    print("Mac (Apple M-series), 1 thread, NOT Jetson; median seconds over", len(frames) - 1, "photos")
    print(df.groupby(["method", "res"])[["decode_rectify_s", "match_s", "total_s"]].median().round(3).to_string())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--split", choices=["precut", "test"], required=True)
    r.add_argument("--tag", default="main")
    r.add_argument("--maps", nargs="+", default=["main", "y2021"])
    r.add_argument("--res", nargs="+", type=float, default=list(G.RESOLUTIONS))
    r.add_argument("--methods", nargs="+", default=METHODS_DEFAULT)
    r.add_argument("--cfg", nargs="+", default=[""], help="e.g. h=const100,att=dji,gnd=dem_prior")
    r.add_argument("--seeds", nargs="+", type=int, default=[0])
    r.add_argument("--frames", nargs="*", type=int)
    r.add_argument("--workers", type=int, default=10)
    r.add_argument("--no-negatives", action="store_true")
    r.add_argument("--append", action="store_true")
    s = sub.add_parser("summarize")
    s.add_argument("--tag", default="main")
    lt = sub.add_parser("latency")
    lt.add_argument("--res", nargs="+", type=float, default=list(G.RESOLUTIONS))
    lt.add_argument("--methods", nargs="+", default=METHODS_DEFAULT + ["roma"])
    lt.add_argument("--n", type=int, default=21)
    args = ap.parse_args()
    {"run": cmd_run, "summarize": cmd_summarize, "latency": cmd_latency}[args.cmd](args)


if __name__ == "__main__":
    main()

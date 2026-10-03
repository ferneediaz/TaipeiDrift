#!/usr/bin/env python3
"""Stage 4: ground speed from consecutive photos (no map), against RTK speed.

Both photos are rectified to nadir-anchored north-up ground patches (DJI fused attitude + pre-cut
boresight; height = SIMULATED baro - DEM under the SIMULATED prior). A ground point g sits at offset
g - n_i in patch i, so the template of patch i+1 found in patch i gives the nadir displacement
D = n_{i+1} - n_i = o_i - o_{i+1}. Speed = |D| / dt with dt from the MRK photo timestamps.
ZNCC with sub-pixel parabola refinement; the far 30 % of patch i+1 (not seen in patch i) is excluded.
RTK speed (reference) = |RTK_{i+1} - RTK_i| / dt. Test pairs only (both photos after the cut).

  .venv/bin/python experiments/x4_tuniu_speed.py --seeds 0 1 ... 19 --res 0.25
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from multiprocessing import get_context

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(__file__))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import x_tuniu_geo as G  # noqa: E402
import x_tuniu_match as M  # noqa: E402
from x3_tuniu_fix import simulated_inputs  # noqa: E402


def _subpix(s, loc):
    x, y = int(loc[0]), int(loc[1])
    dx = dy = 0.0
    if 0 < x < s.shape[1] - 1:
        a, b, c = s[y, x - 1], s[y, x], s[y, x + 1]
        den = a - 2 * b + c
        dx = 0.5 * (a - c) / den if abs(den) > 1e-9 else 0.0
    if 0 < y < s.shape[0] - 1:
        a, b, c = s[y - 1, x], s[y, x], s[y + 1, x]
        den = a - 2 * b + c
        dy = 0.5 * (a - c) / den if abs(den) > 1e-9 else 0.0
    return np.array([x + np.clip(dx, -1, 1), y + np.clip(dy, -1, 1)])


def displacement(qa: dict, qb: dict):
    """Nadir displacement (E, N) metres from patch a (earlier) to patch b (later)."""
    res = qa["res"]
    vb = qb["valid"].copy()
    # drop the far 30 % of b along its forward direction: ground not seen in a
    rows, cols = vb.shape
    nb = qb["nadir_px"]
    yy, xx = np.mgrid[0:rows, 0:cols]
    fwd = qb["fwd"]
    along = (xx - nb[0]) * fwd[0] - (yy - nb[1]) * fwd[1]
    vb &= along * res <= 0.7 * along[qb["valid"]].max() * res
    r0, c0, r1, c1 = M.template_rect(vb, nb, (0.0,), (1.0,))
    if r1 - r0 < 16 or c1 - c0 < 16:
        return None, np.nan
    t = qb["patch"][r0:r1, c0:c1]
    ok = M._valid_scores(qa["valid"], r1 - r0, c1 - c0)
    if not ok.any() or t.std() < 2:
        return None, np.nan
    s, peak, loc = M._match(qa["patch"], t, ok)
    loc = _subpix(s, loc)
    o_a = np.array([qa["x0"] + loc[0] * res, qa["y0"] - loc[1] * res])
    o_b = np.array([qb["x0"] + c0 * res, qb["y0"] - r0 * res])
    return o_a - o_b, peak


def run_pair(u):
    cv2.setNumThreads(1)
    cam, ph = G.Camera(), G.photos()
    cal = json.loads((G.XOUT / "stage1_calibration.json").read_text())
    fac = G.reduce_factor(u["res"])
    qs = []
    cfg = M.QueryConfig(ground="dem_lifted" if u["variant"] == "lifted" else "dem_prior")
    for f, prior, baro in ((u["fa"], u["prior_a"], u["baro_a"]), (u["fb"], u["prior_b"], u["baro_b"])):
        if u["variant"] == "shared_prior":
            prior = u["prior_a"]          # one flat ground plane for both photos of the pair
        att = ph.iloc[f - 1]
        q = M.build_query(G.load_gray(att.path, fac), fac, att, cam, u["res"], cfg, baro, prior,
                          None, cal["takeoff_alt_ell_m"], 0.0, cal["boresight_deg"])
        yaw = np.radians(att.gimbal_yaw_deg + cal["boresight_deg"]["yaw"])
        q["fwd"] = np.array([np.sin(yaw), np.cos(yaw)])
        qs.append(q)
    t0 = time.perf_counter()
    d, peak = displacement(*qs)
    return dict(fa=u["fa"], fb=u["fb"], seed=u["seed"], res=u["res"], de=np.nan if d is None else d[0],
                dn=np.nan if d is None else d[1], peak=peak, match_s=time.perf_counter() - t0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", nargs="+", type=int, default=list(range(20)))
    ap.add_argument("--res", type=float, default=0.25)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--variant", choices=["default", "shared_prior", "lifted"], default="default",
                    help="default = pre-registered (flat plane at the DEM under each photo's own prior); "
                         "shared_prior = one plane (photo a's prior) for both; lifted = DEM-lifted ground (exploratory)")
    args = ap.parse_args()
    test = G.protocol()["test_frames"]
    tr = G.truth_xy().set_index("frame")
    units = []
    for seed in args.seeds:
        sim = simulated_inputs(seed).set_index("frame")
        for fa, fb in zip(test[:-1], test[1:]):
            units.append(dict(fa=fa, fb=fb, seed=seed, res=args.res, variant=args.variant,
                              prior_a=(sim.loc[fa].prior_x, sim.loc[fa].prior_y), baro_a=sim.loc[fa].baro_alt,
                              prior_b=(sim.loc[fb].prior_x, sim.loc[fb].prior_y), baro_b=sim.loc[fb].baro_alt))
    t0 = time.time()
    with get_context("spawn").Pool(args.workers) as pool:
        df = pd.DataFrame(pool.map(run_pair, units, chunksize=4))
    # evaluator
    dt = tr.loc[df.fb, "t_s"].to_numpy() - tr.loc[df.fa, "t_s"].to_numpy()
    rtk = np.c_[tr.loc[df.fb, "x"].to_numpy() - tr.loc[df.fa, "x"].to_numpy(),
                tr.loc[df.fb, "y"].to_numpy() - tr.loc[df.fa, "y"].to_numpy()]
    leg = np.array(G.protocol()["leg_per_frame"])
    df["dt_s"] = dt
    df["rtk_speed"] = np.hypot(*rtk.T) / dt
    df["cam_speed"] = np.hypot(df.de, df.dn) / dt
    df["err_mps"] = df.cam_speed - df.rtk_speed
    df["err_pct"] = 100 * df.err_mps / df.rtk_speed
    df["vec_err_mps"] = np.hypot(df.de / dt - rtk[:, 0] / dt, df.dn / dt - rtk[:, 1] / dt)
    df["straight"] = (leg[df.fa - 1] >= 0) & (leg[df.fa - 1] == leg[df.fb - 1])
    suffix = "" if args.variant == "default" else f"_{args.variant}"
    df.to_csv(G.XOUT / f"stage4_speed_rows{suffix}.csv", index=False)
    print(f"{len(df)} pairs ({len(args.seeds)} seeds x {len(units) // len(args.seeds)} test pairs), "
          f"res {args.res} m/px, {time.time() - t0:.0f} s")
    for name, g in (("all pairs", df), ("straight-leg pairs", df[df.straight]), ("turn pairs", df[~df.straight])):
        ok = g.dropna(subset=["err_mps"])
        print(f"{name}: n={len(g)} measured={len(ok)}; RTK speed median {ok.rtk_speed.median():.2f} m/s; "
              f"speed error median {ok.err_mps.median():+.3f} m/s, |err| median {ok.err_mps.abs().median():.3f} m/s, "
              f"p90 {ok.err_mps.abs().quantile(.9):.3f} m/s; |err| % median {ok.err_pct.abs().median():.2f} %, "
              f"p90 {ok.err_pct.abs().quantile(.9):.2f} %; velocity-vector error median {ok.vec_err_mps.median():.3f} m/s")
    per_seed = df[df.straight].groupby("seed").err_pct.agg(lambda s: s.abs().median())
    print(f"straight legs, median |err %| per seed: min {per_seed.min():.2f} max {per_seed.max():.2f}; "
          f"match time median {df.match_s.median() * 1000:.0f} ms (Mac, 1 thread, not Jetson)")


if __name__ == "__main__":
    main()

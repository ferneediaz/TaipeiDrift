#!/usr/bin/env python3
"""Visual evidence for the Tuniu step-1 fixes: is the rectified photo really aligned with the map?

For each selected test photo, three checkerboards (photo squares alternate with map squares):
  1. at the FIX found by the matcher        -> roads / river / field edges should continue across squares
  2. at the TRUE position (RTK + map offset) -> should look like 1 when the fix is right
  3. at the PRIOR (what the matcher started from, ~30 m off) -> visibly broken, for contrast
Sampling is fixed before looking: rng seed 2026, 8 random ACCEPTED + 4 random REJECTED photos of the
chosen run, plus EVERY accepted fix wrong by > 10 m (none hidden).

Photo-derived images stay in data/processed (photo licence unknown: share privately, do not commit).

Run: .venv/bin/python experiments/x_gallery_tuniu.py --tag main --method xfeat --res 0.25 --seed 0
     .venv/bin/python experiments/x_gallery_tuniu.py --tag seeds --method zncc+quad>=3 --res 1.0 \
         --cfg h=baro_dem,att=dji,gnd=dem_lifted --wrong-only
"""
from __future__ import annotations

import argparse
import json

import cv2
import numpy as np
import pandas as pd

import x_tuniu_geo as G
import x_tuniu_match as M
from x3_tuniu_fix import parse_cfg, simulated_inputs

BLOCK_M = 8.0
DEFAULT_CFG = "h=baro_dem,att=dji,gnd=dem_prior"


def checker(photo: np.ndarray, pvalid: np.ndarray, gray: np.ndarray, block: int) -> np.ndarray:
    rr, cc = np.indices(photo.shape)
    use_photo = (((rr // block) + (cc // block)) % 2 == 0) & pvalid
    out = np.where(use_photo, photo, gray).astype(np.uint8)
    out[~pvalid] = (gray[~pvalid] * 0.4).astype(np.uint8)
    return out


def norm8(a: np.ndarray, valid: np.ndarray) -> np.ndarray:
    v = a[valid]
    lo, hi = np.percentile(v, 1), np.percentile(v, 99)
    return np.clip((a - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(np.uint8)


def panel(q, m: G.MapRaster, nadir_xy, block: int) -> np.ndarray:
    rows, cols = q["patch"].shape
    gray, valid, _ = m.window(nadir_xy[0] + q["x0"], nadir_xy[1] + q["y0"], cols, rows)
    photo = norm8(q["patch"].astype(float), q["valid"])
    mapg = norm8(gray.astype(float), valid) if valid.any() else np.zeros_like(photo)
    return checker(photo, q["valid"] & valid, mapg, block)


def label(img: np.ndarray, text: str) -> np.ndarray:
    out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(out, (0, 0), (out.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(out, text, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", default="main")
    ap.add_argument("--method", default="xfeat")
    ap.add_argument("--res", type=float, default=0.25)
    ap.add_argument("--seed", type=int, default=None, help="default: every seed in the run")
    ap.add_argument("--cfg", default=DEFAULT_CFG)
    ap.add_argument("--map", default="main")
    ap.add_argument("--wrong-only", action="store_true")
    args = ap.parse_args()

    summ = pd.read_csv(G.XOUT / f"stage3_summary_{args.tag}.csv")
    rows = pd.read_csv(G.XOUT / f"stage3_rows_{args.tag}_test.csv")
    base = "zncc" if args.method.startswith("zncc") else args.method
    sel = (rows["method"] == base) & (rows["res"] == args.res) & (rows["map"] == args.map) & \
          (rows["cfg"] == args.cfg) & (rows["kind"] == "pos")
    if args.seed is not None:
        sel &= rows["seed"] == args.seed
    pos = rows[sel].copy()
    s = summ[(summ["method"] == args.method) & (summ["res"] == args.res) & (summ["map"] == args.map)
             & (summ["cfg"] == args.cfg)]
    thr = {int(k): v for k, v in s.set_index("seed")["threshold"].items()}
    if args.method == "zncc+quad>=3":
        pos["acc"] = pos.fix_x.notna() & (pos.quad_n >= 3)
    elif args.method == "zncc":
        pos["acc"] = pos.fix_x.notna() & (pos.score >= pos.seed.map(thr))
    else:
        pos["acc"] = pos.fix_x.notna() & (pos.gate == "ok") & (pos.inliers >= pos.seed.map(thr))
    wrong = pos[pos.acc & (pos.err_m > 10)]
    rng = np.random.default_rng(2026)
    picks = [wrong]
    if not args.wrong_only:
        acc, rej = pos[pos.acc & (pos.err_m <= 10)], pos[~pos.acc]
        picks += [acc.iloc[rng.choice(len(acc), min(8, len(acc)), replace=False)],
                  rej.iloc[rng.choice(len(rej), min(4, len(rej)), replace=False)]]
    chosen = pd.concat(picks)

    cal = json.loads((G.XOUT / "stage1_calibration.json").read_text())
    off = cal["map_offsets"][args.map]
    truth = G.truth_xy().set_index("frame")
    cam, ph = G.Camera(), G.photos()
    m = G.MapRaster(G.map_path(args.map, args.res))
    cfg = M.QueryConfig(**parse_cfg(args.cfg))
    block = max(4, int(BLOCK_M / args.res))
    out_dir = G.XOUT / "gallery" / f"{args.tag}_{args.method.replace('>=', 'ge')}_{args.res:g}m"
    out_dir.mkdir(parents=True, exist_ok=True)
    sims = {}
    index = []
    for _, r in chosen.iterrows():
        seed = int(r.seed)
        sim = sims.setdefault(seed, simulated_inputs(seed).set_index("frame")).loc[int(r.frame)]
        att = ph.iloc[int(r.frame) - 1]
        fac = G.reduce_factor(args.res)
        yaw_err = {"yaw3": 3.0, "yaw7": 7.0}.get(cfg.attitude, 0.0) * sim.yaw_sign
        q = M.build_query(G.load_gray(att.path, fac), fac, att, cam, args.res, cfg, sim.baro_alt,
                          (sim.prior_x, sim.prior_y), None, cal["takeoff_alt_ell_m"], yaw_err, cal["boresight_deg"])
        t = truth.loc[int(r.frame)]
        status = "REJECTED" if not r.acc else ("WRONG >10 m" if r.err_m > 10 else "ACCEPTED")
        err = "no fix" if pd.isna(r.err_m) else f"err {r.err_m:.1f} m"
        tiles = []
        if pd.notna(r.fix_x):
            tiles.append(label(panel(q, m, (r.fix_x, r.fix_y), block), f"at FIX ({err})"))
        tiles.append(label(panel(q, m, (t.x + off["de_m"], t.y + off["dn_m"]), block), "at TRUE position"))
        prior_err = float(np.hypot(sim.prior_x - t.x - off["de_m"], sim.prior_y - t.y - off["dn_m"]))
        tiles.append(label(panel(q, m, (sim.prior_x, sim.prior_y), block), f"at PRIOR ({prior_err:.0f} m off)"))
        h = max(x.shape[0] for x in tiles)
        tiles = [cv2.copyMakeBorder(x, 0, h - x.shape[0], 0, 6, cv2.BORDER_CONSTANT) for x in tiles]
        img = np.hstack(tiles)
        name = f"{status.split()[0].lower()}_f{int(r.frame):03d}_s{seed}.png"
        cv2.imwrite(str(out_dir / name), img)
        index.append(dict(file=name, frame=int(r.frame), seed=seed, status=status, err_m=r.err_m,
                          prior_err_m=round(prior_err, 1)))
    pd.DataFrame(index).to_csv(out_dir / "index.csv", index=False)
    print(out_dir)
    print(pd.DataFrame(index).to_string(index=False))


if __name__ == "__main__":
    main()

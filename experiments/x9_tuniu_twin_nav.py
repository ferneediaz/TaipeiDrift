#!/usr/bin/env python3
"""Run the UNCHANGED Tuniu navigation pipelines on a digital-twin replay folder (x9_tuniu_twin.py).

Level 1: x3_tuniu_fix.py run (test photos, main map 2019-12, 0.5 m, ZNCC + XFeat, default query config,
         negatives as in the real run) scored with the consensus rule of x_consensus_tuniu.py (both fixes
         within 4 m, position = ZNCC fix).
Level 2: x5_tuniu_closed_loop.py calibrate (pre-cut photos of the twin itself) + run --mode loop dr
         --heading dji --ground dem_lifted (main map, stage-1 map offset, Copernicus terrain).

Nothing in those scripts is edited. This module points them at another replay folder by changing module
globals of x_tuniu_geo at import time (also in the spawned worker processes, which import this file as
__mp_main__ and read the variant from the environment variable X9_TWIN_VARIANT):
  G.SEQ          -> data/processed/t_replay/tuniu_tw_1_<variant> (images.csv, attitude.csv, truth.csv, meta.json)
  G.XOUT         -> data/processed/x9_tuniu_twin/nav/<variant> (copies of stage1_calibration.json and the
                    variant's protocol; Level-1 rows are written here)
  G.reduce_factor-> the real decode reduction scaled by the twin image width (quarter size at 0.5 m: 8 -> 2),
                    so the matcher sees the same pixel count as with the real photos
  G.load_gray    -> area-averaged decode of the render + its mask (pixels with no mesh)
  G.rectify      -> as before, and pixels whose mask is not fully valid become invalid
x5_tuniu_closed_loop.OUT -> data/processed/x9_tuniu_twin/nav/<variant>/l2 (calibration and runs).
Variant `real` = the real photos (only XOUT redirected), used to re-run Level 1 on seeds 0-4.

  .venv/bin/python experiments/x9_tuniu_twin_nav.py l1 twin_att_rc --seeds 0 1 2 3 4
  .venv/bin/python experiments/x9_tuniu_twin_nav.py l2 twin_att_rc --seeds 0 1 2 3 4 --modes loop dr
  .venv/bin/python experiments/x9_tuniu_twin_nav.py summary
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import x_tuniu_geo as G  # noqa: E402
import x9_tuniu_twin as T  # noqa: E402

ENV = "X9_TWIN_VARIANT"
REAL_XOUT = G.XOUT
NAV = T.WORK / "nav"
REAL_L2 = G.ROOT / "data/processed/x_tuniu_l2"


class TwinImage(np.ndarray):
    """Decoded render that carries its validity mask (uint8, 255 = mesh seen) to the patched rectify."""


def nav_dir(variant: str) -> Path:
    return NAV / variant


def patch(variant: str) -> None:
    G.XOUT = nav_dir(variant)
    if variant == "real":
        return
    G.SEQ = T.replay_dir(variant)
    ratio = json.loads((G.SEQ / "meta.json").read_text())["camera"]["image_width_px"] / T.FULL_W
    orig_reduce, orig_rectify = G.reduce_factor, G.rectify

    def reduce_factor(res: float) -> int:
        return max(1, int(round(orig_reduce(res) * ratio)))

    def load_gray(path: str, factor: int):
        img, m = T.twin_gray(path, factor)
        out = img.view(TwinImage)
        out.twin_mask = m
        return out

    def rectify(img, factor, cam, R, height, res, far_m=100.0, ground=None):
        m = getattr(img, "twin_mask", None)
        q = orig_rectify(np.asarray(img), factor, cam, R, height, res, far_m=far_m, ground=ground)
        if m is not None:
            qm = orig_rectify(m, factor, cam, R, height, res, far_m=far_m, ground=ground)
            q["valid"] = q["valid"] & (qm["patch"] >= 254)
        return q

    G.reduce_factor, G.load_gray, G.rectify = reduce_factor, load_gray, rectify


if os.environ.get(ENV):          # parent and spawned workers alike
    patch(os.environ[ENV])


def setup(variant: str) -> None:
    """Parent only: protocol + stage-1 calibration in the variant's XOUT, then patch this process."""
    out = nav_dir(variant)
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(REAL_XOUT / "stage1_calibration.json", out / "stage1_calibration.json")
    prot = REAL_XOUT / "stage0_protocol.json" if variant == "real" else T.replay_dir(variant) / "twin_protocol.json"
    shutil.copyfile(prot, out / "stage0_protocol.json")
    os.environ[ENV] = variant
    patch(variant)


# ----------------------------------------------------------------------------- Level 1

def cmd_l1(args) -> None:
    import x3_tuniu_fix as X3
    setup(args.variant)
    ns = argparse.Namespace(split=args.split, tag="cons", maps=["main"], res=[0.5], methods=["zncc", "xfeat"], cfg=[""],
                            seeds=args.seeds, frames=args.frames, workers=args.workers, no_negatives=False,
                            append=False)
    X3.cmd_run(ns)
    if args.split == "test":
        print(json.dumps(l1_eval(args.variant, args.seeds), indent=1))


def l1_eval(variant: str, seeds) -> dict:
    import x_consensus_tuniu as C
    C.SEEDS = list(seeds)
    path = nav_dir(variant) / "stage3_rows_cons_test.csv"
    r = C.evaluate(0.5, C.rows(path, "zncc", 0.5), C.rows(path, "xfeat", 0.5))
    r["variant"] = variant
    return r


# ----------------------------------------------------------------------------- Level 2

def l2_out(variant: str) -> Path:
    return REAL_L2 if variant == "real" else nav_dir(variant) / "l2"


def cmd_l2(args) -> None:
    if args.variant == "real":
        raise SystemExit("real Level-2 runs already exist in data/processed/x_tuniu_l2/runs/main")
    setup(args.variant)
    import x5_tuniu_closed_loop as X5
    X5.OUT = l2_out(args.variant)
    if not (X5.OUT / "calibration_main.json").exists():
        X5.cmd_calibrate(argparse.Namespace(map="main", terrain="copernicus", terrain_dz=0.0, tag="main",
                                            verbose=False))
    if args.calibrate_only:
        return
    X5.cmd_run(argparse.Namespace(map="main", terrain="copernicus", terrain_dz=0.0, map_offset="stage1",
                                  calib_tag="main", tag="main", mode=args.modes, heading=["dji"],
                                  ground=["dem_lifted"], seeds=args.seeds, workers=args.workers))


def l2_eval(variant: str, seeds, mode="loop") -> dict | None:
    import x5_tuniu_l2_report as R
    runs = []
    for s in seeds:
        p = l2_out(variant) / "runs" / "main" / f"{mode}_dji_dem_lifted_s{s}.csv"
        if not p.exists():
            return None
        runs.append(pd.read_csv(p, low_memory=False))
    per = pd.DataFrame([R.run_metrics(g) for g in runs])
    e = pd.concat(runs)
    out = dict(variant=variant, mode=mode, seeds=len(runs), photos_per_seed=int(per.photos.median()),
               err_median_m=float(e.err_m.median()), err_p90_m=float(e.err_m.quantile(0.9)),
               err_max_m=float(e.err_m.max()), pct_lt10=100 * float((e.err_m < 10).mean()),
               odo_ok_pct=float(per.odo_ok_pct.mean()))
    if mode == "loop":
        out.update(accepted_pct=100 * float(per.accepted.sum() / per.photos.sum()),
                   accepted_per_min=float(per.accepted.sum() / sum((g.t_s.iloc[-1] - g.t_s.iloc[0] + g.dt.iloc[0])
                                                                    for g in runs) * 60),
                   agreed_pct=100 * float(per.agreed.sum() / per.photos.sum()),
                   wrong_gt10=int(per.accepted_wrong_gt10.sum()),
                   accepted_fix_err_median_m=float(e.fix_err_m[e.fix_status == "accepted"].median()),
                   seeds_without_lol=int((per.lol_photos == 0).sum()),
                   longest_no_fix_s_max=float(per.longest_no_fix_s.max()),
                   per_seed_median_m=[round(float(v), 2) for v in per.err_median_m])
    return out


# ----------------------------------------------------------------------------- summary

def cmd_summary(args) -> None:
    rows1, rows2 = [], []
    for v in ["real"] + [p.name for p in sorted(NAV.iterdir()) if p.is_dir() and p.name != "real"]:
        if (nav_dir(v) / "stage3_rows_cons_test.csv").exists():
            rows1.append(l1_eval(v, args.l1_seeds))
        for seeds in (args.l2_seeds, args.sweep_seeds):
            for mode in ("loop", "dr"):
                r = l2_eval(v, seeds, mode)
                if r:
                    rows2.append(r | dict(seed_set="-".join(map(str, seeds))))
    T.WORK.mkdir(parents=True, exist_ok=True)
    d1, d2 = pd.DataFrame(rows1), pd.DataFrame(rows2)
    d1.to_csv(T.WORK / "summary_l1.csv", index=False)
    d2.to_csv(T.WORK / "summary_l2.csv", index=False)
    pd.set_option("display.width", 250)
    if len(d1):
        print(d1[["variant", "seeds", "accepted_pct_mean", "accepted_pct_min", "accepted_pct_max", "median_err_m",
                  "p90_err_m", "wrong_gt10", "neg_accepted"]].round(2).to_string(index=False))
    if len(d2):
        print(d2.drop(columns=["per_seed_median_m"], errors="ignore").round(2).to_string(index=False))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("l1")
    a.add_argument("variant")
    a.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4])
    a.add_argument("--frames", nargs="*", type=int)
    a.add_argument("--split", default="test", choices=["test", "precut"], help="precut = smoke test only")
    a.add_argument("--workers", type=int, default=2)
    b = sub.add_parser("l2")
    b.add_argument("variant")
    b.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4])
    b.add_argument("--modes", nargs="+", default=["loop", "dr"], choices=["loop", "dr"])
    b.add_argument("--workers", type=int, default=2)
    b.add_argument("--calibrate-only", action="store_true", help="pre-cut calibration only")
    s = sub.add_parser("summary")
    s.add_argument("--l1-seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4])
    s.add_argument("--l2-seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4])
    s.add_argument("--sweep-seeds", nargs="+", type=int, default=[0, 1, 2])
    args = ap.parse_args()
    {"l1": cmd_l1, "l2": cmd_l2, "summary": cmd_summary}[args.cmd](args)


if __name__ == "__main__":
    main()

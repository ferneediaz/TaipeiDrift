#!/usr/bin/env python3
"""Anti-cheat tests for the Tuniu Level 2 closed loop (x5_tuniu_closed_loop.py): would the result survive if
the estimator could not see RTK, or if the map were wrong? Config: heading dji, ground dem_lifted, closed loop,
seeds 0-4. Writes to data/processed/x_tuniu_l2/anti_cheat/ and runs/anticheat_*/; doc:
docs/research/tuniu-anti-cheat.md. Existing scripts and runs are not modified.

  precompute   parent, RTK allowed: estimator inputs (state at the cut, SIMULATED baro, pre-cut calibration,
               map offset) -> anti_cheat/hidden_inputs.json; replay copy WITHOUT truth.csv / gnss.csv and with
               metadata-stripped photo copies (EXIF/XMP GPS removed, pixels unchanged) -> anti_cheat/replay_notruth/
  hidden-run   runs the unchanged x5.run_sequence with G.SEQ -> replay_notruth, G.truth_xy raising, and an audit
               hook that refuses (PermissionError) any open() of truth.csv / gnss.csv / the original replay folder,
               in this process and in every worker. Raw rows + list of opened data files -> anti_cheat/hidden_raw/
  run-maps     unchanged x5.run_sequence on those maps, map offset = stage1 (as the normal run); one guard: XFeat
               on a texture-less window raises (0 keypoints) -> counted as "no fix" (see _map_worker_init)
  summary      scoring (parent, after the runs) + comparisons -> anti_cheat/summary.json, evidence/errors_*.png
  visual       seed-0 checkerboards at the FILTER / dead-reckoning / true position + trajectory overview
               -> data/processed/x_tuniu_l2/evidence/ (photo-derived: never committed)

Run (in order): .venv/bin/python experiments/x7_tuniu_anti_cheat.py precompute | hidden-run | maps | run-maps
                | summary | visual
"""
from __future__ import annotations

import os
import sys

# ----------------------------------------------------------------------------- truth firewall (before any import
# that could read data). Active in hidden-run and, through the environment, in its spawned workers.
HIDDEN_ENV = "X7_HIDDEN_SEQ"
_OPENED: set = set()
_ORIG_SEQ_PARTS = ("data/processed/t_replay/tuniu_tw_1/",)
_FORBIDDEN_NAMES = ("truth.csv", "gnss.csv")


def _audit(event, args):
    if event != "open" or not args or not isinstance(args[0], (str, bytes, os.PathLike)):
        return
    p = os.fsdecode(args[0])
    ap = os.path.abspath(p)
    if os.path.basename(ap) in _FORBIDDEN_NAMES or any(s in ap for s in _ORIG_SEQ_PARTS):
        raise PermissionError(f"anti-cheat: estimator tried to open {ap}")
    _OPENED.add(ap)


if os.environ.get(HIDDEN_ENV):
    sys.addaudithook(_audit)

import argparse  # noqa: E402
import json  # noqa: E402
import shutil  # noqa: E402
import time  # noqa: E402
from multiprocessing import get_context  # noqa: E402
from pathlib import Path  # noqa: E402

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import x_tuniu_geo as G  # noqa: E402
import x5_tuniu_closed_loop as X5  # noqa: E402


def _truth_hidden(*_a, **_k):
    raise PermissionError("anti-cheat: truth_xy() called on the estimator side")


if os.environ.get(HIDDEN_ENV):
    G.SEQ = Path(os.environ[HIDDEN_ENV])
    G.truth_xy = _truth_hidden

OUT = X5.OUT
AC = OUT / "anti_cheat"
EVID = OUT / "evidence"
SEEDS = (0, 1, 2, 3, 4)
HEADING, GROUND = "dji", "dem_lifted"
HIDDEN_SEQ = AC / "replay_notruth"
SHIFT_E = 30.0
WRONG_SHIFT_E = 250.0   # main map moved 250 m east: still covers the flight, content from 250 m west
VARIANTS = {            # tag -> (map file, description)
    "shift30": (AC / "maps/main_shift_e30_0.5m.tif", "main map, georeference +30 m east"),
    "main_e250": (AC / "maps/main_shift_e250_0.5m.tif", "main map, georeference +250 m east (wrong content)"),
    "negs_here": (AC / "maps/negs_at_flight_0.5m.tif", "OAM 'negs' scene (Sanwan, ~0.9 km away) moved onto the flight"),
}


def stage1_offset() -> tuple[float, float]:
    return X5.map_offset("main", "stage1", None)


# ----------------------------------------------------------------------------- 1. truth hidden

def strip_jpeg(src: Path, dst: Path) -> None:
    """Lossless metadata strip: drop APP1-APP13, APP15 and COM segments (EXIF, XMP with DJI GPS/RTK, MPF) and
    everything after the main image's EOI (embedded preview). Entropy-coded data copied byte for byte."""
    b = src.read_bytes()
    assert b[:2] == b"\xff\xd8", src
    out, i = [b[:2]], 2
    while True:
        assert b[i] == 0xFF, (src, i)
        mk = b[i + 1]
        if mk == 0xDA:
            eoi = b.index(b"\xff\xd9", i + 2)
            out.append(b[i:eoi + 2])
            break
        n = int.from_bytes(b[i + 2:i + 4], "big")
        if not ((0xE1 <= mk <= 0xEF and mk != 0xEE) or mk == 0xFE):
            out.append(b[i:i + 2 + n])
        i += 2 + n
    dst.write_bytes(b"".join(out))


def cmd_precompute(args):
    calib = json.loads((OUT / "calibration_main.json").read_text())
    off = stage1_offset()
    ns = argparse.Namespace(map="main", terrain="copernicus", terrain_dz=0.0)
    units = [X5.unit_for(s, HEADING, GROUND, "loop", ns, calib, off) for s in SEEDS]
    AC.mkdir(parents=True, exist_ok=True)
    (AC / "hidden_inputs.json").write_text(json.dumps(units))
    # replay folder without RTK: estimator-side tables only + stripped copies of the photos it uses
    if HIDDEN_SEQ.exists():
        shutil.rmtree(HIDDEN_SEQ)
    (HIDDEN_SEQ / "photos").mkdir(parents=True)
    for f in ("images.csv", "attitude.csv", "meta.json"):
        shutil.copy2(G.SEQ / f, HIDDEN_SEQ / f)
    img = pd.read_csv(G.SEQ / "images.csv")
    frames = sorted({f for u in units for f in u["frames"]})
    leaks, same = 0, 0
    keys = (b"Latitude", b"Longitude", b"drone-dji", b"Exif", b"RtkFlag", b"xmpmeta")
    for f in frames:
        rel = img.path[f - 1]
        src, dst = G.SEQ / rel, HIDDEN_SEQ / rel
        strip_jpeg(src, dst)
        raw = dst.read_bytes()
        hdr = raw[:raw.index(b"\xff\xda")]     # segments before the image data: no metadata segment left
        leaks += b"GPS" in hdr or any(k in raw for k in keys)
        if f % 25 == 0:      # spot check: decoded pixels identical at the factor the estimator uses
            fac = G.reduce_factor(X5.RES)
            same += int(np.array_equal(G.load_gray(str(src), fac), G.load_gray(str(dst), fac)))
    print(f"{len(units)} units -> {AC / 'hidden_inputs.json'}; {len(frames)} photos stripped -> {HIDDEN_SEQ}; "
          f"files still containing EXIF/XMP/GPS metadata: {leaks}; pixel-identical spot checks: "
          f"{same}/{sum(f % 25 == 0 for f in frames)}")


def hidden_run(u: dict) -> dict:
    rows = X5.run_sequence(u)
    return dict(seed=u["seed"], rows=rows, opened=sorted(_OPENED))


def _probe_truth() -> dict:
    """Proves the firewall is active in a worker: both access paths must fail."""
    res = {}
    for name, fn in (("truth_xy", lambda: G.truth_xy()),
                     ("open truth.csv", lambda: open(G.ROOT / "data/processed/t_replay/tuniu_tw_1/truth.csv"))):
        try:
            fn()
            res[name] = "READABLE"
        except PermissionError as e:
            res[name] = f"blocked: {e}"
    res["seq"] = str(G.SEQ)
    res["truth_in_seq"] = (G.SEQ / "truth.csv").exists()
    return res


def cmd_hidden_run(args):
    if not os.environ.get(HIDDEN_ENV):       # re-exec with the firewall active from the first import
        env = dict(os.environ, **{HIDDEN_ENV: str(HIDDEN_SEQ)})
        os.execve(sys.executable, [sys.executable, *sys.argv], env)
    units = json.loads((AC / "hidden_inputs.json").read_text())
    raw = AC / "hidden_raw"
    raw.mkdir(parents=True, exist_ok=True)
    print(f"hidden run: G.SEQ = {G.SEQ}, truth.csv there: {(G.SEQ / 'truth.csv').exists()}", flush=True)
    t0 = time.time()
    with get_context("spawn").Pool(args.workers, initializer=X5._worker_init, maxtasksperchild=8) as pool:
        probe = pool.apply(_probe_truth)
        print("worker probe:", probe, flush=True)
        (raw / "worker_probe.json").write_text(json.dumps(probe, indent=1))
        for r in pool.imap(hidden_run, units, chunksize=1):
            (raw / f"s{r['seed']}.json").write_text(json.dumps(r, default=float))
            print(f"  seed {r['seed']} done, {time.time() - t0:.0f} s", flush=True)


# ----------------------------------------------------------------------------- 2-3. shifted / wrong maps

def write_shifted(src: Path, dst: Path, de: float, dn: float) -> None:
    import rasterio
    from affine import Affine
    with rasterio.open(src) as ds:
        prof, data, t = ds.profile.copy(), ds.read(), ds.transform
    prof["transform"] = Affine(t.a, t.b, t.c + de, t.d, t.e, t.f + dn)
    dst.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(dst, "w", **prof) as o:
        o.write(data)
    print(f"wrote {dst} (shift {de:+.0f} E, {dn:+.0f} N)")


def cmd_maps(args):
    main = G.map_path("main", X5.RES)
    write_shifted(main, VARIANTS["shift30"][0], SHIFT_E, 0.0)
    write_shifted(main, VARIANTS["main_e250"][0], WRONG_SHIFT_E, 0.0)
    # negs: moved so that its centre sits on the centre of the RTK track's bounding box (truth used only to
    # place this test map, never by the estimator)
    import rasterio
    tr = G.truth_xy()
    cx, cy = (tr.x.min() + tr.x.max()) / 2, (tr.y.min() + tr.y.max()) / 2
    with rasterio.open(G.map_path("negs", X5.RES)) as ds:
        b = ds.bounds
    de, dn = round(cx - (b.left + b.right) / 2), round(cy - (b.bottom + b.top) / 2)
    write_shifted(G.map_path("negs", X5.RES), VARIANTS["negs_here"][0], de, dn)


_GUARD = {"empty_ref": 0}


def _map_worker_init():
    """x5 worker init + one guard: on a map window with no texture at all (e.g. outside a wrong map), XFeat finds
    0 keypoints and vismatch raises IndexError. Never happens on the real map; treated as what learned_fix returns
    for < 8 matches (no fix). Counted and reported."""
    X5._worker_init()
    import x_tuniu_match as M
    orig = M.learned_fix

    def guarded(name, q, ref):
        try:
            return orig(name, q, ref)
        except IndexError:
            _GUARD["empty_ref"] += 1
            return dict(fix=None, matches=0, inliers=0, inlier_ratio=0.0, h_scale=np.nan, h_rot=np.nan, gate="few")
    M.learned_fix = guarded


def map_run(u: dict) -> dict:
    n0 = _GUARD["empty_ref"]
    rows = X5.run_sequence(u)
    return dict(rows=rows, empty_ref=_GUARD["empty_ref"] - n0)


def cmd_run_maps(args):
    """Same estimator (x5.run_sequence, unchanged) and inputs as the normal run; only --map changes. The map
    offset stays the stage-1 offset of the real main map."""
    calib = json.loads((OUT / "calibration_main.json").read_text())
    off = stage1_offset()
    units = []
    for tag in args.variants:
        ns = argparse.Namespace(map=str(VARIANTS[tag][0]), terrain="copernicus", terrain_dz=0.0)
        X5.load_map(ns.map)            # same EPSG:3826 / 0.5 m check as x5 run
        (OUT / "runs" / f"anticheat_{tag}").mkdir(parents=True, exist_ok=True)
        for s in SEEDS:
            if not X5.run_path(f"anticheat_{tag}", "loop", HEADING, GROUND, s).exists():
                units.append(dict(X5.unit_for(s, HEADING, GROUND, "loop", ns, calib, off), tag=tag))
    print(f"{len(units)} runs to do, offset {off}, workers {args.workers}", flush=True)
    t0 = time.time()
    guard = {}
    with get_context("spawn").Pool(args.workers, initializer=_map_worker_init, maxtasksperchild=8) as pool:
        for u, r in zip(units, pool.imap(map_run, units, chunksize=1)):
            df = X5.score(r["rows"], u)
            df.to_csv(X5.run_path(f"anticheat_{u['tag']}", "loop", HEADING, GROUND, u["seed"]), index=False)
            guard[f"{u['tag']}_s{u['seed']}"] = r["empty_ref"]
            print(f"  {u['tag']} seed {u['seed']}: median {df.err_m.median():.1f} m, max {df.err_m.max():.1f} m, "
                  f"accepted {int((df.fix_status == 'accepted').sum())}, empty-reference guard {r['empty_ref']}, "
                  f"{time.time() - t0:.0f} s", flush=True)
    p = AC / "empty_ref_guard.json"
    old = json.loads(p.read_text()) if p.exists() else {}
    p.write_text(json.dumps(old | guard, indent=1))


# ----------------------------------------------------------------------------- scoring

def load(tag: str, mode: str = "loop") -> dict[int, pd.DataFrame]:
    return {s: pd.read_csv(X5.run_path(tag, mode, HEADING, GROUND, s)) for s in SEEDS}


def metrics(df: pd.DataFrame) -> dict:
    st = df.fix_status if "fix_status" in df else pd.Series("", index=df.index)
    acc = st == "accepted"
    out = dict(err_median_m=float(df.err_m.median()), err_max_m=float(df.err_m.max()),
               err_final_m=float(df.err_m.iloc[-1]), accepted=int(acc.sum()), gated=int((st == "gated").sum()),
               agree=int(st.isin(["accepted", "gated"]).sum()),
               ref_valid_median=float(df.ref_valid.median()) if "ref_valid" in df else np.nan)
    if acc.any():
        out["first_accepted_frame"] = int(df.frame[acc].iloc[0])
        out["accepted_fix_dE_median_m"] = float((df.z_x - df.true_x)[acc].median())
        out["accepted_fix_dN_median_m"] = float((df.z_y - df.true_y)[acc].median())
        out["accepted_fix_err_median_m"] = float(df.fix_err_m[acc].median())
    if (st == "gated").any():
        out["gated_fix_dE_median_m"] = float((df.z_x - df.true_x)[st == "gated"].median())
    last = df.iloc[len(df) // 2:]
    out["est_dE_median_2nd_half_m"] = float((last.post_x - last.true_x).median())
    out["est_dN_median_2nd_half_m"] = float((last.post_y - last.true_y).median())
    return out


def pooled(per: list[dict]) -> dict:
    p = pd.DataFrame(per)
    return {k: float(p[k].median()) for k in p.columns if k != "seed" and p[k].dtype.kind in "fi"} | \
        dict(accepted_total=int(p.accepted.sum()), gated_total=int(p.gated.sum()))


def cmd_summary(args):
    summ = {}
    main, dr = load("main"), load("main", "dr")
    summ["normal_loop"] = [dict(seed=s, **metrics(main[s])) for s in SEEDS]
    summ["dead_reckoning"] = [dict(seed=s, **metrics(dr[s])) for s in SEEDS]
    # 1. hidden truth: score here (parent), compare with the normal run
    units = {u["seed"]: u for u in json.loads((AC / "hidden_inputs.json").read_text())}
    hid, diffs, opened = {}, [], set()
    cols_num = ["pred_x", "pred_y", "post_x", "post_y", "post_sd_e", "post_sd_n", "b_hat_deg", "half_m"]
    for s in SEEDS:
        r = json.loads((AC / "hidden_raw" / f"s{s}.json").read_text())
        opened |= set(r["opened"])
        df = X5.score(r["rows"], units[s])
        df.to_csv(AC / "hidden_raw" / f"loop_hidden_s{s}.csv", index=False)
        hid[s] = df
        ref = main[s]
        d = {c: float(np.nanmax(np.abs(df[c].to_numpy() - ref[c].to_numpy()))) for c in cols_num}
        d["fix_status_mismatch"] = int((df.fix_status.fillna("") != ref.fix_status.fillna("")).sum())
        d["odo_status_mismatch"] = int((df.odo_status != ref.odo_status).sum())
        d["rows"] = len(df)
        diffs.append(dict(seed=s, **d))
    venv = str(G.ROOT / ".venv")
    data_files = sorted(p for p in opened if not p.startswith(venv) and not p.endswith((".py", ".pyc", ".so"))
                        and "/lib/python" not in p)
    summ["hidden"] = dict(per_seed=[dict(seed=s, **metrics(hid[s])) for s in SEEDS], diff_vs_normal=diffs,
                          max_abs_post_diff_m=max(max(d["post_x"], d["post_y"]) for d in diffs),
                          worker_probe=json.loads((AC / "hidden_raw/worker_probe.json").read_text()),
                          data_files_opened_by_estimator=[os.path.relpath(p, G.ROOT) for p in data_files])
    for tag in VARIANTS:
        try:
            runs = load(f"anticheat_{tag}")
        except FileNotFoundError:
            continue
        summ[tag] = [dict(seed=s, **metrics(runs[s])) for s in SEEDS]
    summ["pooled"] = {k: pooled(v) for k, v in summ.items() if isinstance(v, list)}
    summ["pooled"]["hidden"] = pooled(summ["hidden"]["per_seed"])
    (AC / "summary.json").write_text(json.dumps(summ, indent=1, default=float))
    print(json.dumps(summ["pooled"], indent=1, default=float))
    print(json.dumps({k: v for k, v in summ["hidden"].items() if k != "per_seed"}, indent=1, default=float))
    error_figure(summ)


def error_figure(summ):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    EVID.mkdir(parents=True, exist_ok=True)
    panels = [("main", "loop", "normal closed loop"), ("main", "dr", "dead reckoning (no map)")] + \
             [(f"anticheat_{t}", "loop", d) for t, (_, d) in VARIANTS.items()
              if X5.run_path(f"anticheat_{t}", "loop", HEADING, GROUND, 0).exists()]
    fig, axs = plt.subplots(len(panels), 1, figsize=(10, 2.3 * len(panels)), sharex=True, sharey=True)
    for ax, (tag, mode, title) in zip(axs, panels):
        for s, df in load(tag, mode).items():
            ax.plot(df.frame, df.err_m, lw=1, label=f"seed {s}")
            if "fix_status" in df:
                a = df.fix_status == "accepted"
                ax.plot(df.frame[a], df.err_m[a], "k.", ms=2)
        ax.set_title(f"{title}  (median {pd.concat(load(tag, mode).values()).err_m.median():.1f} m)", fontsize=9)
        ax.set_ylabel("error (m)")
        ax.set_yscale("symlog", linthresh=10)
        ax.grid(alpha=.3)
    axs[0].legend(fontsize=7, ncol=5)
    axs[-1].set_xlabel("photo (cut after 46); black dots = accepted map fixes")
    fig.tight_layout()
    fig.savefig(EVID / "errors_anti_cheat.png", dpi=110)
    print(EVID / "errors_anti_cheat.png")


# ----------------------------------------------------------------------------- 4. visuals

def query_at(att, est_xy, baro, res, terrain, cam, bs):
    """x5.make_query geometry (DJI attitude + boresight, SIMULATED baro - terrain under est, DEM-lifted) at
    display resolution."""
    fac = G.reduce_factor(res)
    R = G.rot_enu_cam(att.gimbal_yaw_deg + bs["yaw"], att.gimbal_pitch_deg + bs["pitch"],
                      att.gimbal_roll_deg + bs["roll"])
    z0 = float(terrain(est_xy[0], est_xy[1]))

    def gfun(X, Y):
        return terrain(est_xy[0] + X, est_xy[1] + Y) - z0
    return G.rectify(G.load_gray(att.path, fac), fac, cam, R, baro - z0, res, far_m=X5.FAR_M, ground=gfun)


def cmd_visual(args):
    import cv2
    from x_gallery_tuniu import label, panel
    res = 0.25
    seed = 0
    u = {x["seed"]: x for x in json.loads((AC / "hidden_inputs.json").read_text())}[seed]
    baro = dict(zip(u["frames"], u["baro"]))
    loop, dr = load("main")[seed].set_index("frame"), load("main", "dr")[seed].set_index("frame")
    off = np.array(stage1_offset())
    cal = json.loads((G.XOUT / "stage1_calibration.json").read_text())
    cam, ph, bs, terrain = G.Camera(), G.photos(), cal["boresight_deg"], X5.Terrain()
    m = G.MapRaster(G.map_path("main", res))
    block = int(8.0 / res)
    frames = list(loop.index[::10])
    worst = list(loop.err_m.sort_values(ascending=False).index[:3])
    out = EVID / "checker_s0"
    out.mkdir(parents=True, exist_ok=True)
    index = []
    for f in frames + [w for w in worst if w not in frames]:
        att = ph.iloc[f - 1]
        L, D = loop.loc[f], dr.loc[f]
        tiles = []
        for name, xy, err, extra in (
                ("FILTER", (L.post_x, L.post_y), L.err_m, f" fix:{L.fix_status if isinstance(L.fix_status, str) else '-'}"),
                ("DEAD RECKONING", (D.post_x, D.post_y), D.err_m, ""),
                ("TRUE (RTK, reference)", (L.true_x, L.true_y), 0.0, "")):
            q = query_at(att, xy, baro[f], res, terrain, cam, bs)
            img = panel(q, m, np.asarray(xy) + off, block)
            tiles.append(label(img, f"{name} est. err {err:.1f} m{extra}"))
        h = max(x.shape[0] for x in tiles)
        tiles = [cv2.copyMakeBorder(x, 0, h - x.shape[0], 0, 6, cv2.BORDER_CONSTANT) for x in tiles]
        img = np.hstack(tiles)
        tag = "worst_" if f in worst else ""
        name = f"{tag}f{f:03d}.png"
        cv2.imwrite(str(out / name), img)
        index.append(dict(file=name, frame=f, loop_err_m=round(float(L.err_m), 1), dr_err_m=round(float(D.err_m), 1),
                          fix_status=L.fix_status if isinstance(L.fix_status, str) else ""))
    pd.DataFrame(index).to_csv(out / "index.csv", index=False)
    print(pd.DataFrame(index).to_string(index=False))
    overview(seed, loop, dr, off)


def overview(seed, loop, dr, off):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import rasterio
    with rasterio.open(G.map_path("main", 1.0)) as ds:
        rgb = ds.read([1, 2, 3]).transpose(1, 2, 0)
        b = ds.bounds
    tr = G.truth_xy()
    cut = tr.iloc[G.protocol()["precut_frames"][-1] - 1]
    fig, ax = plt.subplots(figsize=(12, 9))
    # map drawn in the RTK frame: map = true + offset
    ax.imshow(rgb, extent=(b.left - off[0], b.right - off[0], b.bottom - off[1], b.top - off[1]), alpha=.75)
    t = tr[tr.frame >= cut.frame]
    ax.plot(t.x, t.y, "w-", lw=2.5, label="truth (RTK, scoring only)")
    ax.plot(t.x, t.y, "k-", lw=1)
    ax.plot(dr.post_x, dr.post_y, "-", color="tab:red", lw=1.5,
            label=f"dead reckoning (median {dr.err_m.median():.0f} m, final {dr.err_m.iloc[-1]:.0f} m)")
    ax.plot(loop.post_x, loop.post_y, "-", color="tab:cyan", lw=1.5,
            label=f"closed loop (median {loop.err_m.median():.1f} m)")
    a = loop.fix_status == "accepted"
    g = loop.fix_status == "gated"
    ax.plot(loop.z_x[a], loop.z_y[a], "o", mfc="none", mec="lime", ms=5, label=f"accepted map fixes ({a.sum()})")
    ax.plot(loop.z_x[g], loop.z_y[g], "x", color="orange", ms=6, label=f"fixes rejected by the gate ({g.sum()})")
    ax.plot(cut.x, cut.y, "*", color="yellow", mec="k", ms=16, label="cut (last RTK, photo 46)")
    pad = 120
    ax.set_xlim(min(t.x.min(), dr.post_x.min()) - pad, max(t.x.max(), dr.post_x.max()) + pad)
    ax.set_ylim(min(t.y.min(), dr.post_y.min()) - pad, max(t.y.max(), dr.post_y.max()) + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("EPSG:3826 east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title(f"Tuniu, GNSS-free after photo 46, seed {seed}, heading dji, ground dem_lifted")
    ax.legend(loc="lower left", fontsize=8, framealpha=.9)
    fig.tight_layout()
    p = EVID / f"overview_s{seed}.png"
    fig.savefig(p, dpi=110)
    print(p)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("precompute")
    h = sub.add_parser("hidden-run")
    h.add_argument("--workers", type=int, default=2)
    sub.add_parser("maps")
    r = sub.add_parser("run-maps")
    r.add_argument("--variants", nargs="+", default=list(VARIANTS), choices=list(VARIANTS))
    r.add_argument("--workers", type=int, default=2)
    sub.add_parser("summary")
    sub.add_parser("visual")
    args = ap.parse_args()
    {"precompute": cmd_precompute, "hidden-run": cmd_hidden_run, "maps": cmd_maps, "run-maps": cmd_run_maps,
     "summary": cmd_summary, "visual": cmd_visual}[args.cmd](args)


if __name__ == "__main__":
    main()

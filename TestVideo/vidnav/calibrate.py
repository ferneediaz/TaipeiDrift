"""Step 1 - camera calibration from the ChArUco video.

Board (inferred from the video itself): 7 x 5 squares, ArUco DICT_6X6 markers
with ids 0..16, marker side / square side = 0.71.  The physical square size
is NOT needed for the intrinsics (focal length, principal point and lens
distortion are scale free); pass --square-mm only if you want metric board poses.

Procedure
1. detect ChArUco corners in every 2nd frame (sub-pixel refined),
2. keep the sharpest frame of every short time window (removes motion blur and
   near-duplicate views),
3. choose the distortion model by 5-fold cross validation (hold-out
   reprojection error, so extra parameters must actually generalise),
4. final fit with iterative rejection of bad views,
5. write output/calibration.json and diagnostic figures.
"""
from __future__ import annotations

import argparse
import pickle

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import CALIB_VIDEO, OUT, ensure_dirs, iter_frames, save_calib, video_info

SQUARES = (7, 5)
MARKER_RATIO = 0.71
DICT = cv2.aruco.DICT_6X6_250

MODELS = {
    "k1k2p1p2k3 (5)": 0,
    "k1k2p1p2 (4)": cv2.CALIB_FIX_K3,
    "rational (8)": cv2.CALIB_RATIONAL_MODEL,
    "rational+prism (12)": cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_THIN_PRISM_MODEL,
}


def make_board(square=1.0):
    d = cv2.aruco.getPredefinedDictionary(DICT)
    return cv2.aruco.CharucoBoard(SQUARES, square, square * MARKER_RATIO, d)


def detect_all(board, step):
    cp = cv2.aruco.CharucoParameters()
    dp = cv2.aruco.DetectorParameters()
    dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.CharucoDetector(board, cp, dp)
    term = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-4)
    out = []
    for i, _, frame in iter_frames(CALIB_VIDEO, step=step):
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cc, cid, _, _ = det.detectBoard(g)
        if cid is None or len(cid) < 12:
            continue
        # half-window ~ 1/8 of the local square size, but at least 3 px
        cc = cc.astype(np.float32)
        span = np.ptp(cc.reshape(-1, 2), axis=0).max() / max(SQUARES)
        win = int(np.clip(span / 8, 3, 11))
        cv2.cornerSubPix(g, cc, (win, win), (-1, -1), term)
        x0, y0 = cc.reshape(-1, 2).min(0).astype(int)
        x1, y1 = cc.reshape(-1, 2).max(0).astype(int)
        sharp = cv2.Laplacian(g[y0:y1 + 1, x0:x1 + 1], cv2.CV_64F).var()
        out.append(dict(frame=i, corners=cc, ids=cid, sharp=sharp))
        if len(out) % 50 == 0:
            print(f"  frame {i}: {len(out)} detections")
    return out


def select_views(dets, window):
    """Sharpest detection per window of `window` frames, needs >=15 corners."""
    best = {}
    for d in dets:
        if len(d["ids"]) < 15:
            continue
        k = d["frame"] // window
        if k not in best or d["sharp"] * len(d["ids"]) > best[k]["sharp"] * len(best[k]["ids"]):
            best[k] = d
    return [best[k] for k in sorted(best)]


def to_points(board, views):
    obj, img = [], []
    for v in views:
        o, p = board.matchImagePoints(v["corners"], v["ids"])
        obj.append(o.astype(np.float32))
        img.append(p.astype(np.float32))
    return obj, img


def calibrate(obj, img, size, flags):
    crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-12)
    ndist = 12 if flags & cv2.CALIB_THIN_PRISM_MODEL else 8 if flags & cv2.CALIB_RATIONAL_MODEL else 5
    return cv2.calibrateCameraExtended(obj, img, size, None, np.zeros(ndist), flags=flags, criteria=crit)


def holdout_rms(K, dist, obj, img):
    errs = []
    for o, p in zip(obj, img):
        ok, r, t = cv2.solvePnP(o, p, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        pr, _ = cv2.projectPoints(o, r, t, K, dist)
        errs.append(np.sum((pr.reshape(-1, 2) - p.reshape(-1, 2)) ** 2, axis=1))
    e = np.concatenate(errs)
    return float(np.sqrt(e.mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=int, default=2, help="detect in every n-th frame")
    ap.add_argument("--window", type=int, default=10, help="frames per selection window")
    ap.add_argument("--square-mm", type=float, default=None)
    a = ap.parse_args()
    ensure_dirs()
    info = video_info(CALIB_VIDEO)
    size = (info["width"], info["height"])
    board = make_board(a.square_mm or 1.0)

    print(f"Detecting ChArUco corners in {CALIB_VIDEO.name} ({info['frames']} frames)...")
    cache = OUT / "cache" / f"charuco_step{a.step}.pkl"
    if cache.exists():
        dets = pickle.loads(cache.read_bytes())
    else:
        dets = detect_all(board, a.step)
        cache.write_bytes(pickle.dumps(dets))
    views = select_views(dets, a.window)
    print(f"{len(dets)} detections, {len(views)} selected views")
    obj, img = to_points(board, views)

    # --- model selection by 5-fold cross validation ---------------------------
    rng = np.random.default_rng(0)
    folds = rng.permutation(len(views)) % 5
    cv_res = {}
    for name, flags in MODELS.items():
        hold = []
        for f in range(5):
            tr = [j for j in range(len(views)) if folds[j] != f]
            te = [j for j in range(len(views)) if folds[j] == f]
            _, K, dist, *_ = calibrate([obj[j] for j in tr], [img[j] for j in tr], size, flags)
            hold.append(holdout_rms(K, dist, [obj[j] for j in te], [img[j] for j in te]))
        cv_res[name] = float(np.mean(hold))
        print(f"  model {name:22s} hold-out RMS {cv_res[name]:.4f} px")
    # pick the simplest model within 2 % of the best hold-out error
    best_err = min(cv_res.values())
    model = next(n for n in MODELS if cv_res[n] <= best_err * 1.02)
    flags = MODELS[model]
    print(f"chosen model: {model}")

    # --- final fit with view rejection -----------------------------------------
    keep = list(range(len(views)))
    for it in range(5):
        rms, K, dist, rv, tv, sdi, sde, pve = calibrate([obj[j] for j in keep], [img[j] for j in keep], size, flags)
        pve = pve.ravel()
        thr = max(3.0 * np.median(pve), 0.5)
        bad = pve > thr
        print(f"  iter {it}: RMS {rms:.4f} px, {len(keep)} views, rejecting {bad.sum()}")
        if not bad.any():
            break
        keep = [k for k, b in zip(keep, bad) if not b]

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    sd = sdi.ravel()
    dist = dist.ravel()
    # trim trailing unused coefficients for readability
    nd = {"k1k2p1p2k3 (5)": 5, "k1k2p1p2 (4)": 5, "rational (8)": 8, "rational+prism (12)": 12}[model]
    dist = dist[:nd]
    hfov = np.degrees(2 * np.arctan(size[0] / 2 / fx))
    vfov = np.degrees(2 * np.arctan(size[1] / 2 / fy))
    print(f"fx={fx:.2f}±{sd[0]:.2f} fy={fy:.2f}±{sd[1]:.2f} cx={cx:.2f}±{sd[2]:.2f} cy={cy:.2f}±{sd[3]:.2f}")
    print(f"dist={np.round(dist, 5)}  HFOV={hfov:.2f}°  VFOV={vfov:.2f}°")

    save_calib(OUT / "calibration.json", K, dist, size, dict(
        model=model, rms_px=float(rms), n_views=len(keep), n_detections=len(dets),
        std_fx=float(sd[0]), std_fy=float(sd[1]), std_cx=float(sd[2]), std_cy=float(sd[3]),
        std_dist=sd[4:4 + nd].tolist(), cv_holdout_rms=cv_res, hfov_deg=float(hfov), vfov_deg=float(vfov),
        board=dict(squares=SQUARES, marker_ratio=MARKER_RATIO, dictionary="DICT_6X6_250"),
        frames_used=[views[k]["frame"] for k in keep]))

    # --- figures ---------------------------------------------------------------
    fig, ax = plt.subplots(1, 3, figsize=(18, 5.2))
    allp = np.concatenate([img[k].reshape(-1, 2) for k in keep])
    resid = []
    for j, k in enumerate(keep):
        pr, _ = cv2.projectPoints(obj[k], rv[j], tv[j], K, dist)
        resid.append(pr.reshape(-1, 2) - img[k].reshape(-1, 2))
    resid = np.concatenate(resid)
    ax[0].scatter(allp[:, 0], allp[:, 1], s=1, c=np.linalg.norm(resid, axis=1), cmap="viridis", vmax=1.0)
    ax[0].set_xlim(0, size[0]); ax[0].set_ylim(size[1], 0); ax[0].set_aspect("equal")
    ax[0].set_title(f"Corner coverage ({len(allp)} corners, colour = residual px)")
    ax[1].hist(np.linalg.norm(resid, axis=1), bins=80)
    ax[1].set_title(f"Reprojection residuals, RMS {rms:.3f} px"); ax[1].set_xlabel("px")
    # distortion field: how far each pixel moves when undistorted
    gx, gy = np.meshgrid(np.linspace(0, size[0], 28), np.linspace(0, size[1], 16))
    pts = np.stack([gx.ravel(), gy.ravel()], 1)
    und = cv2.undistortPoints(pts.reshape(-1, 1, 2).astype(np.float64), K, dist, P=K).reshape(-1, 2)
    d = und - pts
    q = ax[2].quiver(pts[:, 0], pts[:, 1], d[:, 0], -d[:, 1], np.linalg.norm(d, axis=1))
    plt.colorbar(q, ax=ax[2], label="correction [px]")
    ax[2].set_xlim(0, size[0]); ax[2].set_ylim(size[1], 0); ax[2].set_aspect("equal")
    ax[2].set_title("Lens distortion correction field")
    fig.tight_layout(); fig.savefig(OUT / "calibration.png", dpi=110)

    # example detection overlay
    v = views[keep[len(keep) // 2]]
    for i, _, frame in iter_frames(CALIB_VIDEO, start=v["frame"], stop=v["frame"] + 1):
        for (x, y), cid in zip(v["corners"].reshape(-1, 2), v["ids"].ravel()):
            cv2.circle(frame, (int(round(x)), int(round(y))), 8, (0, 0, 255), 2)
            cv2.putText(frame, str(cid), (int(x) + 8, int(y) - 8), 0, 0.8, (0, 0, 255), 2)
        cv2.imwrite(str(OUT / "calibration_detection_example.jpg"), frame)
    print(f"wrote {OUT/'calibration.json'}")


if __name__ == "__main__":
    main()

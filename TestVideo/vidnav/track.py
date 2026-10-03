"""Step 2 - measure the ground surface in every video frame.

One pass over the flight video produces
* KLT feature tracks (sub-pixel optical flow, forward/backward checked) of the
  floor texture for every frame -> output/cache/tracks.npz
* SIFT keyframes (every --kf-step frames) for loop closure when the camera
  returns to an already seen area -> output/cache/keyframes.npz
* the image mask that is used (static rig parts and the part of the image
  that the calibration does not cover are excluded) -> output/mask.png
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from common import CACHE, FLIGHT_VIDEO, OUT, ensure_dirs, iter_frames, load_calib, video_info


def build_mask(size, calib_extra, step=10, scale=8):
    """Valid-pixel mask: floor texture inside the calibrated image region."""
    W, H = size
    w, h = W // scale, H // scale
    stack = []
    for _, _, f in iter_frames(FLIGHT_VIDEO, step=step):
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        stack.append(cv2.resize(g, (w, h), interpolation=cv2.INTER_AREA).astype(np.float32))
    stack = np.array(stack)
    motion = np.median(np.abs(np.diff(stack, axis=0)), axis=0)
    motion = cv2.GaussianBlur(motion, (0, 0), 3)
    static = (motion < 0.75 * np.median(motion)).astype(np.uint8)
    # swivelling casters move around: grow the static area a lot
    static = cv2.dilate(static, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
    rig = cv2.resize(static, (W, H), interpolation=cv2.INTER_NEAREST).astype(bool)

    # calibrated region: convex hull of all ChArUco corners used, grown by 4 %
    import pickle
    dets = pickle.loads((CACHE / "charuco_step2.pkl").read_bytes())
    pts = np.concatenate([d["corners"].reshape(-1, 2) for d in dets]).astype(np.float32)
    hull = cv2.convexHull(pts).reshape(-1, 2)
    c = hull.mean(0)
    hull = (c + (hull - c) * 1.04).astype(np.int32)
    cal = np.zeros((H, W), np.uint8)
    cv2.fillConvexPoly(cal, hull, 1)

    # 12 px border
    border = np.zeros((H, W), bool)
    border[12:-12, 12:-12] = True
    mask = (~rig) & cal.astype(bool) & border
    return mask.astype(np.uint8) * 255, motion


def rootsift(desc):
    desc = desc / (np.abs(desc).sum(1, keepdims=True) + 1e-7)
    desc = np.sqrt(desc)
    return np.clip(desc * 512, 0, 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-pts", type=int, default=1500)
    ap.add_argument("--kf-step", type=int, default=10)
    ap.add_argument("--stop", type=int, default=None)
    a = ap.parse_args()
    ensure_dirs()
    info = video_info(FLIGHT_VIDEO)
    size = (info["width"], info["height"])
    K, dist, cal = load_calib()

    print("building image mask ...")
    mask, motion = build_mask(size, cal)
    cv2.imwrite(str(OUT / "mask.png"), mask)
    print(f"  valid area {mask.mean() / 255 * 100:.1f} % of the image")

    lk = dict(winSize=(25, 25), maxLevel=5,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.005))
    sift = cv2.SIFT_create(nfeatures=4000, contrastThreshold=0.02)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    obs_f, obs_id, obs_uv = [], [], []
    kf_frames, kf_kp, kf_desc = [], [], []
    times, fb_stats, n_tracked = [], [], []
    prev = None
    pts = np.zeros((0, 2), np.float32)
    ids = np.zeros(0, np.int64)
    next_id = 0
    t0 = time.time()
    grid = 12  # cells for spatially uniform feature replenishment
    for k, t, frame in iter_frames(FLIGHT_VIDEO, stop=a.stop):
        g = clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        times.append(t)
        if prev is not None and len(pts):
            p1, st1, _ = cv2.calcOpticalFlowPyrLK(prev, g, pts, None, **lk)
            p0, st0, _ = cv2.calcOpticalFlowPyrLK(g, prev, p1, None, **lk)
            fb = np.linalg.norm(p0 - pts, axis=1)
            ok = (st1.ravel() == 1) & (st0.ravel() == 1) & (fb < 0.3)
            inside = (p1[:, 0] >= 0) & (p1[:, 1] >= 0) & (p1[:, 0] < size[0] - 1) & (p1[:, 1] < size[1] - 1)
            ok &= inside
            ok[inside] &= mask[p1[inside, 1].astype(int), p1[inside, 0].astype(int)] > 0
            fb_stats.append(np.median(fb[ok]) if ok.any() else np.nan)
            pts, ids = p1[ok], ids[ok]
        else:
            fb_stats.append(np.nan)
        n_tracked.append(len(pts))

        # replenish: new corners in cells that have few tracks
        if len(pts) < a.max_pts * 0.8:
            m = mask.copy()
            for p in pts:
                cv2.circle(m, (int(p[0]), int(p[1])), 18, 0, -1)
            new = cv2.goodFeaturesToTrack(g, a.max_pts - len(pts), 0.003, 18, mask=m, blockSize=7)
            if new is not None:
                new = new.reshape(-1, 2)
                term = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
                new = cv2.cornerSubPix(g, new.astype(np.float32), (5, 5), (-1, -1), term)
                pts = np.vstack([pts, new]).astype(np.float32)
                ids = np.concatenate([ids, np.arange(next_id, next_id + len(new))])
                next_id += len(new)

        obs_f.append(np.full(len(pts), k, np.int32))
        obs_id.append(ids.copy())
        obs_uv.append(pts.copy())

        if k % a.kf_step == 0:
            kp, desc = sift.detectAndCompute(g, mask)
            if desc is not None:
                kf_frames.append(k)
                kf_kp.append(np.array([p.pt for p in kp], np.float32))
                kf_desc.append(rootsift(desc))
        prev = g
        if k % 200 == 0:
            el = time.time() - t0
            print(f"  frame {k}/{info['frames']}  tracks {len(pts)}  fb-err {fb_stats[-1]:.3f}px  {el:.0f}s")

    np.savez_compressed(CACHE / "tracks.npz", frame=np.concatenate(obs_f), id=np.concatenate(obs_id),
                        uv=np.concatenate(obs_uv), times=np.array(times), fb=np.array(fb_stats),
                        n_tracked=np.array(n_tracked), size=np.array(size))
    off = np.cumsum([0] + [len(x) for x in kf_kp])
    np.savez_compressed(CACHE / "keyframes.npz", frames=np.array(kf_frames), offsets=off,
                        kp=np.concatenate(kf_kp), desc=np.concatenate(kf_desc))
    print(f"done: {len(times)} frames, {next_id} tracks, {len(kf_frames)} keyframes, {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()

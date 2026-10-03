"""Step 4 - visualise the result.

* output/floor_map.jpg      orthomosaic of the floor stitched with the estimated
                            poses (if the poses were wrong, tile joints would be
                            visibly broken / doubled - a strong visual check)
* output/trajectory.png     path, coordinates over time, speed, heading, altitude
* output/position_video.mp4 the camera video next to the map with the live position
"""
from __future__ import annotations

import argparse
import json

import cv2
import imageio_ffmpeg
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import CACHE, FLIGHT_VIDEO, OUT, iter_frames, load_calib
from navigate import rot_tilt


def ground_homography(K, R, h):
    """undistorted pixel -> floor coords (m, camera-nadir frame, x right / y down)."""
    return np.diag([h, h, 1.0]) @ R @ np.linalg.inv(K)


def pose_mat(x, y, th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mm-per-px", type=float, default=1.0)
    ap.add_argument("--mosaic-step", type=int, default=4)
    ap.add_argument("--video-step", type=int, default=2, help="60 fps / step = output fps")
    ap.add_argument("--no-video", action="store_true")
    a = ap.parse_args()
    K, dist, cal = load_calib()
    S = np.load(CACHE / "solution.npz")
    poses, odo, vo, hk, tilt, times = S["poses"], S["odo"], S["vo"], S["hk"], S["tilt"], S["times"]
    summ = json.loads((OUT / "summary.json").read_text())
    R = rot_tilt(*tilt)
    W, H = cal["image_size"]
    mask = cv2.imread(str(OUT / "mask.png"), cv2.IMREAD_GRAYSCALE)
    m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, K, (W, H), cv2.CV_32FC1)
    mask_u = cv2.remap(mask, m1, m2, cv2.INTER_NEAREST)
    feather = cv2.distanceTransform((mask_u > 0).astype(np.uint8), cv2.DIST_L2, 5).astype(np.float32)
    feather = np.minimum(feather / 150.0, 1.0) ** 2

    # map extent from frame footprints
    corners = np.array([[0, 0, 1], [W, 0, 1], [W, H, 1], [0, H, 1]], float).T
    foot = []
    for k in range(0, len(poses), 20):
        G = pose_mat(*poses[k]) @ ground_homography(K, R, hk[k])
        c = G @ corners
        foot.append((c[:2] / c[2]).T)
    foot = np.concatenate(foot)
    res = a.mm_per_px / 1000
    lo = foot.min(0) - 0.02
    hi = foot.max(0) + 0.02
    MW, MH = int((hi[0] - lo[0]) / res), int((hi[1] - lo[1]) / res)
    M = np.array([[1 / res, 0, -lo[0] / res], [0, 1 / res, -lo[1] / res], [0, 0, 1]])
    print(f"map {MW} x {MH} px at {a.mm_per_px} mm/px")

    acc = np.zeros((MH, MW, 3), np.float32)
    wacc = np.zeros((MH, MW), np.float32)
    for k, _, frame in iter_frames(FLIGHT_VIDEO, step=a.mosaic_step):
        und = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)
        Hm = M @ pose_mat(*poses[k]) @ ground_homography(K, R, hk[k])
        wimg = cv2.warpPerspective(und, Hm, (MW, MH), flags=cv2.INTER_LINEAR)
        ww = cv2.warpPerspective(feather, Hm, (MW, MH), flags=cv2.INTER_LINEAR)
        acc += wimg.astype(np.float32) * ww[..., None]
        wacc += ww
        if k % 400 == 0:
            print(f"  mosaic frame {k}")
    mosaic = (acc / np.maximum(wacc, 1e-6)[..., None]).astype(np.uint8)
    mosaic[wacc < 1e-3] = 255

    # trajectory overlay on the map
    def to_map(P):
        q = M @ np.vstack([P[:, 0], P[:, 1], np.ones(len(P))])
        return q[:2].T

    over = mosaic.copy()
    for P, col, th in ((odo, (0, 140, 255), 3), (poses, (0, 0, 220), 4)):
        pts = to_map(P[:, :2]).astype(np.int32)
        cv2.polylines(over, [pts], False, col, th, cv2.LINE_AA)
    st = to_map(poses[:1, :2])[0].astype(int)
    en = to_map(poses[-1:, :2])[0].astype(int)
    cv2.circle(over, tuple(st), 14, (0, 180, 0), -1)
    cv2.circle(over, tuple(en), 14, (200, 0, 0), -1)
    # 10 cm scale bar
    sb = int(0.1 / res)
    cv2.rectangle(over, (30, MH - 60), (30 + sb, MH - 45), (0, 0, 0), -1)
    cv2.putText(over, "10 cm", (30, MH - 70), 0, 1.2, (0, 0, 0), 3)
    cv2.imwrite(str(OUT / "floor_map.jpg"), mosaic, [cv2.IMWRITE_JPEG_QUALITY, 92])
    cv2.imwrite(str(OUT / "floor_map_with_path.jpg"), over, [cv2.IMWRITE_JPEG_QUALITY, 92])

    # ---- plots (convert to X right / Y up)
    X, Y = poses[:, 0], -poses[:, 1]
    fig = plt.figure(figsize=(16, 10))
    ax = fig.add_subplot(2, 3, (1, 4))
    ext = [lo[0], hi[0], -hi[1], -lo[1]]
    ax.imshow(cv2.cvtColor(mosaic, cv2.COLOR_BGR2RGB), extent=ext, alpha=0.75)
    ax.plot(odo[:, 0], -odo[:, 1], color="#ff8c00", lw=1.2, label="frame-to-frame dead reckoning")
    ax.plot(vo[:, 0], -vo[:, 1], color="#2a9d8f", lw=1.2, label="multi-span VO (no loop closure)")
    ax.plot(X, Y, color="#c1121f", lw=1.8, label="final (with loop closure)")
    lp = S["loops"]
    for k, j in lp[::max(1, len(lp) // 60)]:
        ax.plot([X[k], X[j]], [Y[k], Y[j]], color="k", lw=0.5, alpha=0.4)
    ax.plot(X[0], Y[0], "o", color="green", ms=9, label="start")
    ax.plot(X[-1], Y[-1], "s", color="blue", ms=8, label="end")
    for tt in range(0, int(times[-1]) + 1, 5):
        k = np.searchsorted(times, tt)
        ax.annotate(f"{tt}s", (X[k], Y[k]), fontsize=8, xytext=(4, 4), textcoords="offset points")
    ax.set_aspect("equal"); ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
    ax.set_title(f"Camera position over the floor (h = {summ['height_m']*100:.1f} cm)")
    ax.legend(loc="best", fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 3, 2)
    ax.plot(times, X, label="X"); ax.plot(times, Y, label="Y")
    ax.set_xlabel("time [s]"); ax.set_ylabel("position [m]"); ax.legend(); ax.grid(alpha=0.3)
    ax.set_title("Position vs. time")
    ax = fig.add_subplot(2, 3, 3)
    d = np.loadtxt(OUT / "trajectory.csv", delimiter=",", skiprows=1)
    ax.plot(times, d[:, 5] * 100); ax.set_xlabel("time [s]"); ax.set_ylabel("speed [cm/s]"); ax.grid(alpha=0.3)
    ax.set_title(f"Speed (path length {summ['path_length_m']:.3f} m)")
    ax = fig.add_subplot(2, 3, 5)
    ax.plot(times, d[:, 4]); ax.set_xlabel("time [s]"); ax.set_ylabel("heading [deg]"); ax.grid(alpha=0.3)
    ax.set_title("Heading (rotation about the vertical)")
    ax = fig.add_subplot(2, 3, 6)
    ax.plot(times, S["h_est"] * 100); ax.axhline(summ["height_m"] * 100, color="k", ls="--", lw=0.8)
    ax.set_xlabel("time [s]"); ax.set_ylabel("height [cm]"); ax.grid(alpha=0.3)
    ax.set_title("Altitude seen in image scale (diagnostic, smoothed)")
    fig.tight_layout(); fig.savefig(OUT / "trajectory.png", dpi=120)
    print("wrote trajectory.png, floor_map*.jpg")

    if a.no_video:
        return
    # ---- annotated video
    VW, VH = 1920, 1080
    fps = 60.0 / a.video_step
    writer = imageio_ffmpeg.write_frames(str(OUT / "position_video.mp4"), (VW, VH), fps=fps,
                                         codec="libx264", quality=7, macro_block_size=8)
    writer.send(None)
    small_map_scale = min(960 / MW, 1080 / MH)
    base = cv2.resize(over if False else mosaic, None, fx=small_map_scale, fy=small_map_scale, interpolation=cv2.INTER_AREA)
    oy, ox = (VH - base.shape[0]) // 2, 960 + (960 - base.shape[1]) // 2
    trail = to_map(poses[:, :2]) * small_map_scale
    for k, _, frame in iter_frames(FLIGHT_VIDEO, step=a.video_step):
        canvas = np.full((VH, VW, 3), 255, np.uint8)
        left = cv2.resize(frame, (960, 540), interpolation=cv2.INTER_AREA)
        cv2.drawMarker(left, (int(K[0, 2] / 2.25), int(K[1, 2] / 2.25)), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
        canvas[0:540, 0:960] = left
        mp = base.copy()
        cv2.polylines(mp, [trail[:k + 1].astype(np.int32)], False, (0, 0, 220), 2, cv2.LINE_AA)
        G = M @ pose_mat(*poses[k]) @ ground_homography(K, R, hk[k])
        c = G @ corners
        fp = ((c[:2] / c[2]).T * small_map_scale).astype(np.int32)
        cv2.polylines(mp, [fp], True, (255, 120, 0), 2, cv2.LINE_AA)
        cv2.circle(mp, tuple(trail[k].astype(int)), 6, (0, 0, 255), -1)
        canvas[oy:oy + mp.shape[0], ox:ox + mp.shape[1]] = mp
        X_, Y_ = poses[k, 0], -poses[k, 1]
        lines = [f"t = {times[k]:7.3f} s   frame {k}",
                 f"X = {X_ * 100:+8.2f} cm",
                 f"Y = {Y_ * 100:+8.2f} cm",
                 f"heading = {d[k, 4]:+7.2f} deg",
                 f"speed = {d[k, 5] * 100:5.1f} cm/s",
                 f"distance = {d[k, 6] * 100:7.1f} cm",
                 f"altitude = {hk[k] * 100:5.2f} cm (given)"]
        for i, s in enumerate(lines):
            cv2.putText(canvas, s, (30, 600 + i * 62), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (20, 20, 20), 3, cv2.LINE_AA)
        writer.send(np.ascontiguousarray(canvas[..., ::-1]))
        if k % 600 == 0:
            print(f"  video frame {k}")
    writer.close()
    print("wrote position_video.mp4")


if __name__ == "__main__":
    main()

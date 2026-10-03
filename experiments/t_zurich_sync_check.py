"""Check camera/IMU time alignment of a replay sequence (MEASURED on the Zurich AGZ window).

Horizontal image shift between consecutive frames (phase correlation, downsampled grey) is
mostly yaw for a forward/side camera: dx_px ~ fx * yaw_rate * dt. Cross-correlate the image
yaw rate with each gyro axis over lags; the best lag is the image-time offset vs the IMU clock.

  .venv/bin/python experiments/t_zurich_sync_check.py data/processed/t_replay/zurich_agz_1800_2400
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

root = Path(sys.argv[1])
meta = json.loads((root / "meta.json").read_text())
cam0 = meta["sensors"]["camera"]["cams"]["cam0"]
fx = cam0["K"][0][0] if cam0.get("K") else float(cam0["width"])  # unknown K: rate scale only, corr unaffected
images = pd.read_csv(root / "images.csv")
imu = pd.read_csv(root / "imu.csv")
scale = 0.25
win = None
prev = None
t_mid, rate = [], []
for t, p in zip(images.t_s, images.path):
    g = cv2.imread(str(root / p), cv2.IMREAD_GRAYSCALE)
    g = cv2.resize(g, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA).astype(np.float32)
    if win is None:
        win = cv2.createHanningWindow(g.shape[::-1], cv2.CV_32F)
    if prev is not None:
        (dx, _dy), resp = cv2.phaseCorrelate(prev[1], g, win)
        dt = t - prev[0]
        if 0 < dt < 0.15 and resp > 0.05:
            t_mid.append((t + prev[0]) / 2)
            rate.append(dx / scale / fx / dt)  # rad/s, sign arbitrary
    prev = (t, g)
t_mid, rate = np.array(t_mid), np.array(rate)
print(f"frame pairs used {len(rate)} of {len(images) - 1}")

grid = np.arange(t_mid.min() + 3, t_mid.max() - 3, 0.02)
cam = np.interp(grid, t_mid, rate)
out = {}
for ax in ("gx", "gy", "gz"):
    best = (0.0, 0.0)
    for lag in np.arange(-2.0, 2.0001, 0.02):  # gyro(t + lag) vs camera(t)
        g = np.interp(grid + lag, imu.t_s, imu[ax])
        c = np.corrcoef(cam, g)[0, 1]
        if abs(c) > abs(best[1]):
            best = (round(float(lag), 3), round(float(c), 3))
    c0 = float(np.corrcoef(cam, np.interp(grid, imu.t_s, imu[ax]))[0, 1])
    out[ax] = {"best_lag_s": best[0], "corr_at_best": best[1], "corr_at_0": round(c0, 3)}
    print(ax, out[ax])
# Stability: best gz lag per window (camera time offset may jump or drift)
W = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
windows = []
for w0 in np.arange(grid[0], grid[-1] - W, W):
    m = (grid >= w0) & (grid < w0 + W)
    lags = np.arange(-2.0, 2.0001, 0.02)
    cs = [np.corrcoef(cam[m], np.interp(grid[m] + lag, imu.t_s, imu.gz))[0, 1] for lag in lags]
    k = int(np.nanargmax(np.abs(cs)))
    windows.append({"t0_s": round(float(w0), 1), "best_lag_s": round(float(lags[k]), 2), "corr": round(float(cs[k]), 3)})
    print("window", windows[-1])
(root / "sync_check.json").write_text(json.dumps({"pairs": len(rate), **out, "window_s": W, "gz_windows": windows}, indent=2))

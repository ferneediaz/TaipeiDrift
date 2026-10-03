"""Shared helpers: paths, video reading, calibration I/O."""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FLIGHT_VIDEO = ROOT / "A001_10031533_C001.mov"
CALIB_VIDEO = ROOT / "A001_10031641_C002.mov"
OUT = ROOT / "output"
CACHE = OUT / "cache"


def ensure_dirs() -> None:
    OUT.mkdir(exist_ok=True)
    CACHE.mkdir(exist_ok=True)


def iter_frames(path: Path, step: int = 1, start: int = 0, stop: int | None = None):
    """Yield (index, time_s, BGR frame). OpenCV applies the rotation metadata
    automatically, so frames come out as 2160x1214 landscape images."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise FileNotFoundError(path)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    stop = n if stop is None else min(stop, n)
    if start:
        cap.set(cv2.CAP_PROP_POS_FRAMES, start)
    i = start
    while i < stop:
        if step > 1 and (i - start) % step:
            if not cap.grab():
                break
            i += 1
            continue
        ok, frame = cap.read()
        if not ok:
            break
        t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        yield i, t, frame
        i += 1
    cap.release()


def video_info(path: Path) -> dict:
    cap = cv2.VideoCapture(str(path))
    info = dict(
        frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
        fps=cap.get(cv2.CAP_PROP_FPS),
        width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    )
    cap.release()
    return info


def save_calib(path: Path, K, dist, size, extra: dict) -> None:
    d = dict(K=np.asarray(K).tolist(), dist=np.asarray(dist).ravel().tolist(),
             image_size=list(size), **extra)
    path.write_text(json.dumps(d, indent=2))


def load_calib(path: Path = None):
    path = path or OUT / "calibration.json"
    d = json.loads(Path(path).read_text())
    return np.array(d["K"]), np.array(d["dist"]), d


def undistort_norm(pts_px: np.ndarray, K, dist) -> np.ndarray:
    """Pixel coordinates (N,2) -> ideal normalized image coordinates (N,2)."""
    if len(pts_px) == 0:
        return np.zeros((0, 2))
    p = np.asarray(pts_px, np.float64).reshape(-1, 1, 2)
    crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-10)
    return cv2.undistortPoints(p, K, dist, R=None, P=None, criteria=crit).reshape(-1, 2)

"""Two questions about the ALTO chain that matter for a cheap drone.

1. What does one position fix cost in computing time, and does it still work on small images?
   A fix here is 105 comparisons: 7 reference images x 5 zooms x 3 rotations.
2. Where does the drift of camera dead reckoning come from: a wrong scale, or a wrong direction?
   A heading reference such as a sun sensor would remove only the second part.

Run from the repository root, after experiments/h_alto_end_to_end.py has run once:
    python experiments/l_alto_cost_and_drift.py
"""
import time
import zipfile

import cv2
import numpy as np
import pandas as pd

ZIP = "data/raw/alto/Val.zip"
FLOW_CACHE = "data/processed/alto_val_flow.npy"  # written by h_alto_end_to_end.py
REF_MPP = 0.60
JAM_AT_M = 300.0

z = zipfile.ZipFile(ZIP)
query = pd.read_csv(z.open("Val/query.csv"))
ref = pd.read_csv(z.open("Val/reference.csv"))
ref = ref[ref.name.str.startswith("offset_0_None")].reset_index(drop=True)
truth = query[["easting", "northing"]].to_numpy()
ref_xy = ref[["easting", "northing"]].to_numpy()
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


def load(name: str, scale: float) -> np.ndarray:
    raw = np.frombuffer(z.read(name), np.uint8)
    image = clahe.apply(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)).astype(np.float32)
    return image if scale == 1.0 else cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


def template(frame: np.ndarray, zoom: float, angle: float, keep: float = 0.8) -> np.ndarray:
    n = int(round(frame.shape[0] * zoom))
    t = cv2.resize(frame, (n, n), interpolation=cv2.INTER_AREA)
    t = cv2.warpAffine(t, cv2.getRotationMatrix2D((n / 2, n / 2), angle, 1.0), (n, n))
    c = int(n * keep)
    o = (n - c) // 2
    return t[o : o + c, o : o + c]


def one_fix(frame: np.ndarray, refs: list, near: np.ndarray, scale: float) -> np.ndarray:
    best = (-1.0, None)
    for zoom in (0.75, 0.80, 0.85, 0.90, 0.95):
        for angle in (5, 10, 15):
            t = template(frame, zoom, angle)
            for i, r in zip(near, refs):
                _, score, _, loc = cv2.minMaxLoc(cv2.matchTemplate(r, t, cv2.TM_CCOEFF_NORMED))
                if score > best[0]:
                    centre = np.array(loc) + t.shape[0] / 2
                    half = r.shape[0] / 2
                    mpp = REF_MPP / scale
                    best = (score, ref_xy[i] + [(centre[0] - half) * mpp, -(centre[1] - half) * mpp])
    return best[1]


print("1. Cost of one fix (105 comparisons), measured on this machine:")
for threads in (0, 1):  # 0: all processor cores, 1: a single core
    cv2.setNumThreads(threads)
    for scale in (1.0, 0.5, 0.25):
        times, errors = [], []
        for k in (200, 600, 1000, 1400):
            frame = load(f"Val/query_images/{query.name[k]}", scale)
            near = np.argsort(np.linalg.norm(ref_xy - truth[k], axis=1))[:7]
            refs = [load(f"Val/reference_images/{ref.name[i]}", scale) for i in near]
            t0 = time.perf_counter()
            position = one_fix(frame, refs, near, scale)
            times.append(time.perf_counter() - t0)
            errors.append(np.linalg.norm(position - truth[k]))
        cores = "all cores" if threads == 0 else "one core "
        print(f"   images at {int(500 * scale):3d} px, {cores}: {1000 * np.median(times):4.0f} ms per fix; fix error on 4 frames: {np.round(errors).astype(int).tolist()} m")

print("\n2. Camera dead reckoning with no fixes: scale or direction?")
flow = np.load(FLOW_CACHE)
travelled = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(truth, axis=0), axis=1))]
jam = int(np.searchsorted(travelled, JAM_AT_M))
A0, *_ = np.linalg.lstsq(flow[1 : jam + 1], np.diff(truth, axis=0)[:jam], rcond=None)
estimate = truth[jam] + np.cumsum(flow[jam + 1 :] @ A0, axis=0)
d_true, d_est = truth[-1] - truth[jam], estimate[-1] - truth[jam]
angle = np.degrees(np.arctan2(d_est[1], d_est[0]) - np.arctan2(d_true[1], d_true[0]))
along = d_true / np.linalg.norm(d_true)
across = np.array([-along[1], along[0]])
error = estimate[-1] - truth[-1]
print(f"   flown since the jam: {np.linalg.norm(d_true):.0f} m; estimated: {np.linalg.norm(d_est):.0f} m ({(np.linalg.norm(d_est) / np.linalg.norm(d_true) - 1) * 100:+.1f}%); direction off by {angle:+.1f} degrees")
print(f"   error at the end: {np.linalg.norm(error):.0f} m = {abs(error @ along):.0f} m along the route (scale) and {abs(error @ across):.0f} m across it (direction)")
course = np.degrees(np.arctan2(np.gradient(truth[:, 0]), np.gradient(truth[:, 1])))
print(f"   true course: {np.median(course[:jam]):.1f} degrees before the jam, between {course[jam:].min():.1f} and {course[jam:].max():.1f} degrees afterwards")

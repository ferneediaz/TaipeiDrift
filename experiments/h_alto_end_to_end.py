"""End to end on the real ALTO flight: GNSS, then jamming, then camera only.

1. While GNSS works (first 300 m): learn how image motion maps to ground motion,
   and the zoom, rotation and offset of the camera against the reference images.
2. After the jam: the position is carried forward by image motion alone (dead reckoning).
3. With fixes: every so many metres the camera frame is matched against reference images
   near the current estimate. A fix is used only if it passes the checks.

Three ways of taking fixes are compared:
  - fixed search: the 7 reference images nearest to the estimate, no score check
  - fixed search with a score check
  - search sized by the filter's uncertainty, with a score check

The script uses the camera frames, the reference images with their coordinates, and the
true position during the first 300 m. It does not use the altitude, the orientation or any
camera calibration. The true position after the jam is used only to measure the error.
Run from the repository root:  python experiments/h_alto_end_to_end.py
"""
import os
import time
import zipfile

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ZIP = "data/raw/alto/Val.zip"
FLOW_CACHE = "data/processed/alto_val_flow.npy"
FIGURE = "docs/figures/alto_end_to_end.png"
REF_MPP = 0.60  # metres per pixel of the reference images, measured in f_alto_matching.py
JAM_AT_M = 300.0
FIX_SIGMA = 15.0  # metres, accuracy of one fix
DRIFT_RATE = 0.10  # dead reckoning error assumed by the filter, as a share of distance since the last fix
KEEP = 0.8  # share of the camera frame used as the template
MIN_SCORE = 0.33  # from i_alto_zoom_check.py: no match at a wrong place scored above 0.32

z = zipfile.ZipFile(ZIP)
query = pd.read_csv(z.open("Val/query.csv"))
ref = pd.read_csv(z.open("Val/reference.csv"))
ref = ref[ref.name.str.startswith("offset_0_None")].reset_index(drop=True)
truth = query[["easting", "northing"]].to_numpy()
ref_xy = ref[["easting", "northing"]].to_numpy()
travelled = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(truth, axis=0), axis=1))]
jam = int(np.searchsorted(travelled, JAM_AT_M))
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
images: dict[str, np.ndarray] = {}


def load(name: str) -> np.ndarray:
    if name not in images:
        raw = np.frombuffer(z.read(name), np.uint8)
        images[name] = clahe.apply(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)).astype(np.float32)
    return images[name]


def image_motion() -> np.ndarray:
    """Median shift of the image content between consecutive camera frames, in pixels."""
    if os.path.exists(FLOW_CACHE):
        return np.load(FLOW_CACHE)
    flow = np.zeros((len(query), 2))
    previous = None
    for k, name in enumerate(query.name):
        raw = np.frombuffer(z.read(f"Val/query_images/{name}"), np.uint8)
        frame = cv2.resize(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE), (250, 250), interpolation=cv2.INTER_AREA)
        if previous is not None:
            f = cv2.calcOpticalFlowFarneback(previous, frame, None, 0.5, 4, 21, 3, 7, 1.5, 0)
            flow[k] = np.median(f[60:190, 60:190].reshape(-1, 2), axis=0) * 2
        previous = frame
    os.makedirs(os.path.dirname(FLOW_CACHE), exist_ok=True)
    np.save(FLOW_CACHE, flow)
    return flow


def template(frame: np.ndarray, zoom: float, angle: float) -> np.ndarray:
    n = int(round(500 * zoom))
    t = cv2.resize(frame, (n, n), interpolation=cv2.INTER_AREA)
    t = cv2.warpAffine(t, cv2.getRotationMatrix2D((n / 2, n / 2), angle, 1.0), (n, n))
    c = int(n * KEEP)
    o = (n - c) // 2
    return t[o : o + c, o : o + c]


def nearest(estimate: np.ndarray, n: int) -> np.ndarray:
    return np.argsort(np.linalg.norm(ref_xy - estimate, axis=1))[:n]


def fix(k: int, zooms: np.ndarray, angles: np.ndarray, candidates: np.ndarray) -> tuple:
    """Match camera frame k against the given reference images. Returns score, position, zoom, rotation."""
    frame = load(f"Val/query_images/{query.name[k]}")
    best = (-1.0, None, 0.0, 0.0)
    for zoom in zooms:
        for angle in angles:
            t = template(frame, zoom, angle)
            for ri in candidates:
                result = cv2.matchTemplate(load(f"Val/reference_images/{ref.name[ri]}"), t, cv2.TM_CCOEFF_NORMED)
                _, score, _, loc = cv2.minMaxLoc(result)
                if score > best[0]:
                    centre = np.array(loc) + t.shape[0] / 2
                    offset = np.array([(centre[0] - 250) * REF_MPP, -(centre[1] - 250) * REF_MPP])
                    best = (score, ref_xy[ri] + offset, zoom, angle)
    return best


flow = image_motion()

# --- while GNSS works: calibration -------------------------------------------------------
steps = np.diff(truth, axis=0)
A0, *_ = np.linalg.lstsq(flow[1 : jam + 1], steps[:jam], rcond=None)  # image shift -> ground step
calib_frames = (jam // 3, 2 * jam // 3, jam)
calib = [fix(k, np.arange(0.60, 1.01, 0.05), np.arange(-10, 36, 5), nearest(truth[k], 7)) for k in calib_frames]
zoom0 = float(np.median([c[2] for c in calib]))
angle0 = float(np.median([c[3] for c in calib]))
offset0 = np.median([c[1] - truth[k] for c, k in zip(calib, calib_frames)], axis=0)
print(f"jam after {travelled[jam]:.0f} m (frame {jam}). Learned before the jam: zoom {zoom0:.2f}, rotation {angle0:.0f} deg, fix offset {offset0.round(1)} m")


def run(fix_every_m: float | None, sized_search: bool = False, min_score: float = 0.0) -> dict:
    estimate = truth[jam].copy()
    path = [estimate.copy()]
    variance = 3.0**2
    since_fix = 0.0
    since_try = 0.0
    zoom = zoom0
    scale = 1.0
    log = []  # per attempted fix: score, error of the fix in metres, used or not
    for k in range(jam + 1, len(query)):
        step = flow[k] @ A0 * scale
        estimate = estimate + step
        since_fix += np.linalg.norm(step)
        since_try += np.linalg.norm(step)
        if fix_every_m and since_try >= fix_every_m:
            since_try = 0.0
            predicted_var = variance + (DRIFT_RATE * since_fix) ** 2
            zooms = np.clip(zoom + np.arange(-0.10, 0.11, 0.05), 0.5, 1.1)
            candidates = nearest(estimate, 7)
            if sized_search:
                radius = max(60.0, 3 * np.sqrt(predicted_var))
                inside = np.where(np.linalg.norm(ref_xy - estimate, axis=1) <= radius)[0]
                candidates = inside if len(inside) >= 7 else candidates
                if since_fix > 400:
                    zooms = np.arange(0.60, 1.101, 0.05)
            score, position, new_zoom, _ = fix(k, zooms, np.array([angle0 - 5, angle0, angle0 + 5]), candidates)
            position = position - offset0
            agrees = np.linalg.norm(position - estimate) <= 3 * np.sqrt(predicted_var + FIX_SIGMA**2)
            use = bool(agrees and score >= min_score)
            log.append((score, np.linalg.norm(position - truth[k]), use))
            if use:
                gain = predicted_var / (predicted_var + FIX_SIGMA**2)
                estimate = estimate + gain * (position - estimate)
                variance = (1 - gain) * predicted_var
                scale, zoom, since_fix = new_zoom / zoom0, new_zoom, 0.0
        path.append(estimate.copy())
    path = np.array(path)
    return {"path": path, "error": np.linalg.norm(path - truth[jam:], axis=1), "log": np.array(log).reshape(-1, 3)}


def report(label: str, r: dict) -> None:
    e, log = r["error"], r["log"]
    line = f"{label:44s} median {np.median(e):6.1f} m, 90% below {np.percentile(e, 90):6.1f}, worst {e.max():6.1f}, end {e[-1]:6.1f}"
    if len(log):
        used = log[:, 2] == 1
        line += f" | fixes used {used.sum():2d}, rejected {(~used).sum():2d}"
        line += f", used but wrong by over 50 m: {(log[used, 1] > 50).sum()}, rejected although within 30 m: {(log[~used, 1] <= 30).sum()}"
    print(line)


distance = travelled[jam:] - travelled[jam]
results = {}
t0 = time.time()
results["camera only"] = run(None)
report("camera only, no fixes", results["camera only"])
at = lambda m: results["camera only"]["error"][np.searchsorted(distance, m)]
print(f"  camera only, error at 300 / 500 / 1000 / 2000 m after the jam: {at(300):.0f} / {at(500):.0f} / {at(1000):.0f} / {at(2000):.0f} m")

print("fixed search (7 nearest reference images), no score check:")
for every in (100, 200, 300, 400, 500, 600, 800, 1000):
    results[f"fixed {every}"] = run(float(every))
    report(f"  fix every {every} m", results[f"fixed {every}"])

print(f"fixed search, fix used only if its score is at least {MIN_SCORE}:")
for every in (300, 1000):
    results[f"fixed checked {every}"] = run(float(every), min_score=MIN_SCORE)
    report(f"  fix every {every} m", results[f"fixed checked {every}"])

print(f"search sized by the uncertainty, fix used only if its score is at least {MIN_SCORE}:")
for every in (300, 1000, 2000):
    results[f"sized {every}"] = run(float(every), sized_search=True, min_score=MIN_SCORE)
    report(f"  fix every {every} m", results[f"sized {every}"])
print(f"run time: {time.time() - t0:.0f} s")

# --- plot -------------------------------------------------------------------------------
os.makedirs(os.path.dirname(FIGURE), exist_ok=True)
fig, (a, b) = plt.subplots(2, 1, figsize=(11, 8.5))
origin = truth[0]
a.plot(*(truth - origin).T, color="black", lw=2, label="true path")
for key, label, color in [("camera only", "camera only, no fixes", "tab:red"), ("fixed 300", "fix every 300 m", "tab:green")]:
    a.plot(*(results[key]["path"] - origin).T, color=color, lw=1.2, label=label)
a.axvline(truth[jam, 0] - origin[0], color="gray", ls=":")
a.set_xlabel("east (m)"), a.set_ylabel("north (m)"), a.set_aspect("equal"), a.legend()
a.set_title("ALTO helicopter flight: GNSS jammed after 300 m (dotted line)")
curves = [
    ("camera only", "camera only, no fixes", "tab:red"),
    ("fixed 1000", "fix every 1000 m, fixed search, no score check", "tab:orange"),
    ("sized 1000", "fix every 1000 m, search sized by uncertainty, score check", "tab:purple"),
    ("fixed 300", "fix every 300 m", "tab:green"),
    ("fixed 100", "fix every 100 m", "tab:blue"),
]
for key, label, color in curves:
    b.plot(distance, results[key]["error"], color=color, label=label)
b.set_xlabel("distance flown since the jam (m)"), b.set_ylabel("position error (m)"), b.set_yscale("log"), b.set_ylim(1, 2000)
b.grid(alpha=0.3), b.legend(fontsize=8)
fig.tight_layout()
fig.savefig(FIGURE, dpi=110)
print("figure:", FIGURE)

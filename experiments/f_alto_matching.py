"""Does classical matching give position fixes on the real ALTO images?

For a sample of camera frames, compare each with the reference images near the true
position (the prior) and measure how far the resulting position is from the truth.
Run from the repository root:  python experiments/f_alto_matching.py [prior in metres] [number of frames]

Method: SIFT keypoints, ratio test, and a RANSAC fit of shift, rotation and scale.
The score of a candidate is the number of keypoints that agree with the fit.
"""
import sys
import time
import zipfile

import cv2
import numpy as np
import pandas as pd

ZIP = "data/raw/alto/Val.zip"
PRIOR_M = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
N_FRAMES = int(sys.argv[2]) if len(sys.argv) > 2 else 100
MIN_AGREE = 12  # fewer agreeing keypoints than this: the fix is rejected

z = zipfile.ZipFile(ZIP)
query = pd.read_csv(z.open("Val/query.csv"))
ref = pd.read_csv(z.open("Val/reference.csv"))
ref = ref[ref.name.str.startswith("offset_0_None")].reset_index(drop=True)
q_xy = query[["easting", "northing"]].to_numpy()
r_xy = ref[["easting", "northing"]].to_numpy()

sift = cv2.SIFT_create(nfeatures=1500)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
matcher = cv2.BFMatcher(cv2.NORM_L2)
cache: dict[str, tuple] = {}


def features(name: str) -> tuple:
    if name not in cache:
        raw = np.frombuffer(z.read(name), np.uint8)
        gray = clahe.apply(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE))
        cache[name] = sift.detectAndCompute(gray, None)
    return cache[name]


def fit(name_a: str, name_b: str) -> tuple[int, np.ndarray | None]:
    """Shift, rotation and scale that map image a onto image b, and how many keypoints agree."""
    kp_a, des_a = features(name_a)
    kp_b, des_b = features(name_b)
    if des_a is None or des_b is None or len(kp_a) < 4 or len(kp_b) < 4:
        return 0, None
    good = [m for m, n in (p for p in matcher.knnMatch(des_a, des_b, k=2) if len(p) == 2) if m.distance < 0.75 * n.distance]
    if len(good) < 4:
        return 0, None
    pts_a = np.float32([kp_a[m.queryIdx].pt for m in good])
    pts_b = np.float32([kp_b[m.trainIdx].pt for m in good])
    matrix, mask = cv2.estimateAffinePartial2D(pts_a, pts_b, method=cv2.RANSAC, ransacReprojThreshold=4.0)
    if matrix is None:
        return 0, None
    return int(mask.sum()), matrix


# Step 1: metres per pixel of the reference images, from pairs 50 m apart.
mpp_values = []
for i in range(0, len(ref) - 5, 40):
    agree, m = fit(f"Val/reference_images/{ref.name[i]}", f"Val/reference_images/{ref.name[i + 5]}")
    if agree >= 30:
        shift_px = np.hypot(m[0, 2], m[1, 2])
        mpp_values.append(np.linalg.norm(r_xy[i + 5] - r_xy[i]) / shift_px)
MPP = float(np.median(mpp_values))
print(f"reference images: {MPP:.2f} m per pixel, so one image covers {MPP * 500:.0f} m (from {len(mpp_values)} pairs)")

# Step 2: fixes for a sample of camera frames.
rows = []
t0 = time.time()
for qi in np.linspace(0, len(query) - 1, N_FRAMES).astype(int):
    q_name = f"Val/query_images/{query.name[qi]}"
    near = np.where(np.linalg.norm(r_xy - q_xy[qi], axis=1) <= PRIOR_M)[0]
    results = [(*fit(q_name, f"Val/reference_images/{ref.name[ri]}"), ri) for ri in near]
    agree, m, ri = max(results, key=lambda r: r[0])
    second = sorted(r[0] for r in results)[-2] if len(results) > 1 else 0
    if m is None:
        rows.append([qi, 0, second, np.nan, np.nan, np.nan, len(near)])
        continue
    centre = m @ np.array([250.0, 250.0, 1.0])  # where the middle of the camera frame lands in the reference image
    east = (centre[0] - 250.0) * MPP
    north = -(centre[1] - 250.0) * MPP
    error = np.linalg.norm(r_xy[ri] + [east, north] - q_xy[qi])
    scale = np.hypot(m[0, 0], m[1, 0])
    angle = np.degrees(np.arctan2(m[1, 0], m[0, 0]))
    rows.append([qi, agree, second, error, scale, angle, len(near)])
elapsed = time.time() - t0

r = pd.DataFrame(rows, columns=["frame", "agree", "second", "error_m", "scale", "angle_deg", "candidates"])
ok = r[r.agree >= MIN_AGREE]
print(f"\nprior: within {PRIOR_M:.0f} m, {r.candidates.mean():.0f} candidates per frame, {len(r)} frames")
print(f"accepted (at least {MIN_AGREE} agreeing keypoints): {len(ok)} of {len(r)}")
if len(ok):
    e = ok.error_m
    print(f"position error of accepted fixes: median {e.median():.1f} m, 90% below {e.quantile(0.9):.1f} m, worst {e.max():.1f} m")
    print(f"accepted but wrong by more than 30 m: {(e > 30).sum()}")
    print(f"camera frame covers {ok.scale.median() * 500 * MPP:.0f} m; rotation against the reference: {ok.angle_deg.median():.0f} deg (from {ok.angle_deg.min():.0f} to {ok.angle_deg.max():.0f})")
print(f"agreeing keypoints: median {r.agree.median():.0f}, lowest {r.agree.min()}")
print(f"time: {1000 * elapsed / len(r):.0f} ms per fix, {1000 * elapsed / r.candidates.sum():.0f} ms per comparison")

# --- Part 2: brightness-pattern matching -------------------------------------------------
# Slide the camera frame over the reference image nearest to the true position and compare
# brightness patterns (normalised cross-correlation). Zoom and rotation are searched.
# The frame can shift by about 40 m inside the reference image, so this is a narrow search.
print("\nbrightness-pattern matching on 24 frames, zoom 0.60 to 1.00, rotation -10 to 35 degrees:")


def gray(name: str) -> np.ndarray:
    raw = np.frombuffer(z.read(name), np.uint8)
    return clahe.apply(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)).astype(np.float32)


def template(frame: np.ndarray, zoom: float, angle: float, keep: float = 0.8) -> np.ndarray:
    n = int(round(500 * zoom))
    t = cv2.resize(frame, (n, n), interpolation=cv2.INTER_AREA)
    t = cv2.warpAffine(t, cv2.getRotationMatrix2D((n / 2, n / 2), angle, 1.0), (n, n))
    c = int(n * keep)
    o = (n - c) // 2
    return t[o : o + c, o : o + c]


rows = []
for qi in np.linspace(20, len(query) - 20, 24).astype(int):
    ri = int(np.argmin(np.linalg.norm(r_xy - q_xy[qi], axis=1)))
    reference = gray(f"Val/reference_images/{ref.name[ri]}")
    frame = gray(f"Val/query_images/{query.name[qi]}")
    d = q_xy[qi] - r_xy[ri]
    expected = np.array([250 + d[0] / MPP, 250 - d[1] / MPP])  # where the frame centre truly lies in the reference image
    best = (-1.0, 0.0, 0.0, 0.0)
    for zoom in np.arange(0.60, 1.01, 0.05):
        for angle in range(-10, 36, 5):
            t = template(frame, zoom, angle)
            _, score, _, loc = cv2.minMaxLoc(cv2.matchTemplate(reference, t, cv2.TM_CCOEFF_NORMED))
            if score > best[0]:
                centre = np.array(loc) + t.shape[0] / 2
                best = (score, zoom, angle, np.linalg.norm(centre - expected) * MPP)
    rows.append(best)
b = np.array(rows)
good = b[:, 3] < 20
print(f"within 20 m: {good.sum()} of {len(b)}; median error of those {np.median(b[good, 3]):.1f} m; errors of the others: {np.sort(b[~good, 3]).round(0).tolist()} m")
print(f"zoom of the good ones: median {np.median(b[good, 1]):.2f}, from {b[good, 1].min():.2f} to {b[good, 1].max():.2f}; rotation: median {np.median(b[good, 2]):.0f} deg, from {b[good, 2].min():.0f} to {b[good, 2].max():.0f}")
print(f"score of the good ones: median {np.median(b[good, 0]):.2f}, lowest {b[good, 0].min():.2f}; score of the others: {b[~good, 0].round(2).tolist()}")

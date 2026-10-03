"""Can the zoom of an image match be used to check the fix against an elevation map?

Idea: the zoom that makes a camera frame fit the reference image is proportional to the
height above ground. The altitude (barometer) minus the ground height from an open
elevation model predicts that zoom for any claimed position. A fix whose zoom does not
fit the prediction is suspect.

Test on the real ALTO flight:
  1. Right place: measure the zoom for a sample of frames, compare with altitude minus ground.
  2. Wrong place: match each frame against a reference image 400 m away and see how often
     the zoom it finds still fits the prediction for that wrong place.
Run from the repository root:  python experiments/i_alto_zoom_check.py [number of frames]
"""
import os
import sys
import zipfile

import cv2
import numpy as np
import pandas as pd
import rasterio
from pyproj import Transformer
from rasterio.windows import from_bounds

ZIP = "data/raw/alto/Val.zip"
DEM_CACHE = "data/processed/alto_val_dem.npz"
N_FRAMES = int(sys.argv[1]) if len(sys.argv) > 1 else 60
ZOOMS = np.arange(0.50, 1.151, 0.025)
ANGLES = (5, 10, 15, 20)
KEEP = 0.8

z = zipfile.ZipFile(ZIP)
query = pd.read_csv(z.open("Val/query.csv"))
ref = pd.read_csv(z.open("Val/reference.csv"))
ref = ref[ref.name.str.startswith("offset_0_None")].reset_index(drop=True)
truth = query[["easting", "northing"]].to_numpy()
ref_xy = ref[["easting", "northing"]].to_numpy()
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
to_lonlat = Transformer.from_crs("EPSG:32617", "EPSG:4326", always_xy=True)


def ground_height(xy: np.ndarray) -> np.ndarray:
    """Ground height from the Copernicus 30 m elevation model, for UTM positions."""
    lon, lat = to_lonlat.transform(xy[:, 0], xy[:, 1])
    if not os.path.exists(DEM_CACHE):
        all_lon, all_lat = to_lonlat.transform(ref_xy[:, 0], ref_xy[:, 1])
        w, e, s, n = all_lon.min() - 0.02, all_lon.max() + 0.02, all_lat.min() - 0.02, all_lat.max() + 0.02
        tile = f"Copernicus_DSM_COG_10_N{int(np.floor(s)):02d}_00_W{int(-np.floor(w)):03d}_00_DEM"
        with rasterio.open(f"/vsicurl/https://copernicus-dem-30m.s3.amazonaws.com/{tile}/{tile}.tif") as ds:
            window = from_bounds(w, s, e, n, ds.transform)
            data = ds.read(1, window=window).astype(np.float32)
            t = ds.window_transform(window)
        os.makedirs(os.path.dirname(DEM_CACHE), exist_ok=True)
        np.savez(DEM_CACHE, data=data, t=np.array([t.a, t.b, t.c, t.d, t.e, t.f]))
    d = np.load(DEM_CACHE)
    a, _, c, _, e_, f = d["t"]
    col, row = (lon - c) / a - 0.5, (lat - f) / e_ - 0.5
    return cv2.remap(d["data"], col.astype(np.float32).reshape(-1, 1), row.astype(np.float32).reshape(-1, 1), cv2.INTER_LINEAR).ravel()


def load(name: str) -> np.ndarray:
    raw = np.frombuffer(z.read(name), np.uint8)
    return clahe.apply(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)).astype(np.float32)


def best_match(frame: np.ndarray, reference: np.ndarray) -> tuple[float, float]:
    """Zoom and score of the best fit of the camera frame inside one reference image."""
    best = (-1.0, 0.0)
    for zoom in ZOOMS:
        n = int(round(500 * zoom))
        scaled = cv2.resize(frame, (n, n), interpolation=cv2.INTER_AREA)
        c = int(n * KEEP)
        o = (n - c) // 2
        if c >= 500:
            continue
        for angle in ANGLES:
            t = cv2.warpAffine(scaled, cv2.getRotationMatrix2D((n / 2, n / 2), angle, 1.0), (n, n))[o : o + c, o : o + c]
            score = cv2.minMaxLoc(cv2.matchTemplate(reference, t, cv2.TM_CCOEFF_NORMED))[1]
            if score > best[0]:
                best = (score, zoom)
    return best


frames = np.linspace(10, len(query) - 10, N_FRAMES).astype(int)
right, wrong = [], []
for k in frames:
    frame = load(f"Val/query_images/{query.name[k]}")
    ri = int(np.argmin(np.linalg.norm(ref_xy - truth[k], axis=1)))
    wi = ri + 40 if ri + 40 < len(ref) else ri - 40  # a reference image 400 m along the route
    right.append((*best_match(frame, load(f"Val/reference_images/{ref.name[ri]}")), ri))
    wrong.append((*best_match(frame, load(f"Val/reference_images/{ref.name[wi]}")), wi))
right, wrong = np.array(right), np.array(wrong)

altitude = query.altitude.to_numpy()[frames]  # stands in for the barometer
above_right = altitude - ground_height(truth[frames])
above_wrong = altitude - ground_height(ref_xy[wrong[:, 2].astype(int)])

# zoom = slope * (altitude - ground) + intercept, fitted on the first half, tested on the second half
half = len(frames) // 2
slope, intercept = np.polyfit(above_right[:half], right[:half, 1], 1)
residual = right[half:, 1] - (slope * above_right[half:] + intercept)
tolerance = 2.5 * np.std(right[:half, 1] - (slope * above_right[:half] + intercept))
print(f"ground height along the route: {ground_height(truth[frames]).min():.0f} to {ground_height(truth[frames]).max():.0f} m; altitude minus ground: {above_right.min():.0f} to {above_right.max():.0f} m")
print(f"zoom at the right place: {right[:, 1].min():.2f} to {right[:, 1].max():.2f}; correlation with altitude minus ground: {np.corrcoef(above_right, right[:, 1])[0, 1]:.2f}")
print(f"fit on the first half: zoom = {slope:.5f} x height + {intercept:.2f}; zoom reaches zero at {-intercept / slope:.0f} m")
print(f"second half, not used for the fit: zoom error median {np.median(np.abs(residual)):.3f}, which is {np.median(np.abs(residual)) / slope:.0f} m of height")
print(f"tolerance for the check: zoom within {tolerance:.3f} of the prediction")

pass_right = np.abs(right[:, 1] - (slope * above_right + intercept)) <= tolerance
pass_wrong = np.abs(wrong[:, 1] - (slope * above_wrong + intercept)) <= tolerance
print(f"\nright fixes that pass the zoom check: {pass_right.sum()} of {len(frames)}")
print(f"wrong fixes (400 m off) that pass the zoom check: {pass_wrong.sum()} of {len(frames)}")
print(f"match score, right place: median {np.median(right[:, 0]):.2f}; wrong place: median {np.median(wrong[:, 0]):.2f}; wrong place scores higher than right place in {(wrong[:, 0] > right[:, 0]).sum()} of {len(frames)} frames")
both = pass_wrong & (wrong[:, 0] > np.percentile(right[:, 0], 10))
print(f"wrong fixes that pass both the zoom check and a score threshold: {both.sum()} of {len(frames)}")

# Diagnostics: is the zoom check physics, or a side effect of how wrong matches behave?
confident = right[:, 0] >= np.median(right[:, 0])
print(f"\nconfident right matches only ({confident.sum()}): correlation of zoom with altitude minus ground {np.corrcoef(above_right[confident], right[confident, 1])[0, 1]:.2f}")
print(f"right matches at the edge of the zoom range (0.50 or 1.15): {np.isin(right[:, 1].round(3), [0.5, 1.15]).sum()} of {len(frames)}")
print("zoom found at the wrong place, counts per value:", dict(zip(*[v.tolist() for v in np.unique(wrong[:, 1].round(3), return_counts=True)])))
print(f"score: lowest right {right[:, 0].min():.2f}, 10% of right below {np.percentile(right[:, 0], 10):.2f}; highest wrong {wrong[:, 0].max():.2f}, 90% of wrong below {np.percentile(wrong[:, 0], 90):.2f}")
threshold = np.percentile(wrong[:, 0], 100) + 0.01
print(f"a plain score threshold of {threshold:.2f} rejects every wrong fix and keeps {(right[:, 0] > threshold).sum()} of {len(frames)} right ones")

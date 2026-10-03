"""Test a nadir-image shadow compass on ALTO validation and Wufeng orthophotos.

Run from the repository root with .venv/bin/python. The detector uses only image
pixels. Its sole ALTO calibration parameter is a circular angular offset learned
from GNSS course before the 300 m cut; quaternion headings are opened only for
post-cut scoring.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
import zipfile

import cv2
import numpy as np
import pandas as pd
import rasterio
from pyproj import Transformer
from scipy.spatial.transform import Rotation

ZIP = "data/raw/alto/Val.zip"
WUFENG = (
    "data/raw/aerial/wufeng_2018-05-03_x4.tif",
    "data/raw/aerial/wufeng_2020-03-23_x4.tif",
)
OUT = "data/processed/u1_shadow_compass"
JAM_AT_M = 300.0
R_MIN = 0.025
MIN_EDGES = 120
SAMPLE_DISTANCE = 5.0


def circular_delta(a: np.ndarray | float, b: np.ndarray | float) -> np.ndarray:
    """Signed shortest angular difference a-b in degrees."""
    return (np.asarray(a) - np.asarray(b) + 180.0) % 360.0 - 180.0


def circ_mean_deg(values: np.ndarray) -> float:
    radians = np.radians(np.asarray(values, dtype=float))
    return float(np.degrees(np.arctan2(np.sin(radians).mean(), np.cos(radians).mean())) % 360.0)


def circ_std_deg(values: np.ndarray) -> float:
    radians = np.radians(np.asarray(values, dtype=float))
    resultant = np.hypot(np.cos(radians).mean(), np.sin(radians).mean())
    if resultant <= 0.0:
        return 180.0
    return float(np.degrees(np.sqrt(max(0.0, -2.0 * np.log(resultant)))))


def illumination_vector(image: np.ndarray, *, rgb: bool = False) -> tuple[float, float, int]:
    """Return signed dark-to-light edge direction, resultant confidence, and support.

    The selected edges have a dark-side sample in the lower half of the local
    luminance distribution and a brighter sample five pixels along the gradient.
    The signed resultant rejects frames whose edge normals have no dominant axis.
    """
    if image.ndim == 3:
        code = cv2.COLOR_RGB2GRAY if rgb else cv2.COLOR_BGR2GRAY
        gray = cv2.cvtColor(image, code)
    else:
        gray = image
    gray = np.asarray(gray, dtype=np.uint8)
    smooth = cv2.GaussianBlur(gray, (0, 0), 1.25).astype(np.float32)
    gx = cv2.Sobel(smooth, cv2.CV_32F, 1, 0, ksize=3, scale=1 / 8)
    gy = cv2.Sobel(smooth, cv2.CV_32F, 0, 1, ksize=3, scale=1 / 8)
    magnitude = cv2.magnitude(gx, gy)
    norm = magnitude + 1e-6
    ux, uy = gx / norm, gy / norm
    h, w = gray.shape
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    plus = cv2.remap(smooth, xx + SAMPLE_DISTANCE * ux, yy + SAMPLE_DISTANCE * uy,
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT101)
    minus = cv2.remap(smooth, xx - SAMPLE_DISTANCE * ux, yy - SAMPLE_DISTANCE * uy,
                      cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT101)
    contrast = plus - minus
    dark_limit = float(np.percentile(smooth, 50))
    mag_limit = max(1.5, float(np.percentile(magnitude, 65)))
    mask = (magnitude >= mag_limit) & (contrast >= 8.0) & (minus <= dark_limit)
    weights = np.maximum(contrast - 6.0, 0.0) * np.maximum(1.0 - minus / 255.0, 0.0)
    weights *= mask
    support = int(np.count_nonzero(mask))
    total = float(weights.sum())
    if support < MIN_EDGES or total <= 1e-6:
        return float("nan"), 0.0, support
    sx = float(np.sum(weights * ux))
    sy = float(np.sum(weights * uy))
    resultant = min(1.0, math.hypot(sx, sy) / total)
    angle = float(np.degrees(np.arctan2(sy, sx)) % 360.0)
    return angle, resultant, support


def shadow_silhouette_vector(image: np.ndarray, *, rgb: bool = False) -> tuple[float, float, int]:
    """Infer shadow direction from paired dark silhouettes and adjacent bright objects."""
    if image.ndim == 3:
        code = cv2.COLOR_RGB2GRAY if rgb else cv2.COLOR_BGR2GRAY
        gray = cv2.cvtColor(image, code)
    else:
        gray = image
    gray = cv2.GaussianBlur(np.asarray(gray, dtype=np.uint8), (0, 0), 1.0)
    local = cv2.GaussianBlur(gray, (0, 0), 18.0).astype(np.float32)
    residual = gray.astype(np.float32) - local
    dark_strength = -residual
    positive_dark = dark_strength[dark_strength > 0]
    positive_bright = residual[residual > 0]
    if not len(positive_dark) or not len(positive_bright):
        return float("nan"), 0.0, 0
    dark_threshold = max(8.0, float(np.percentile(positive_dark, 78)))
    bright_threshold = max(8.0, float(np.percentile(positive_bright, 78)))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    dark_mask = cv2.morphologyEx((dark_strength >= dark_threshold).astype(np.uint8), cv2.MORPH_OPEN, kernel)
    bright_mask = cv2.morphologyEx((residual >= bright_threshold).astype(np.uint8), cv2.MORPH_OPEN, kernel)
    nd, dark_labels, dark_stats, dark_centers = cv2.connectedComponentsWithStats(dark_mask, 8)
    nb, bright_labels, bright_stats, bright_centers = cv2.connectedComponentsWithStats(bright_mask, 8)
    del nb
    h, w = gray.shape
    vectors, weights = [], []
    for label in range(1, nd):
        x, y, bw, bh, area = dark_stats[label]
        if area < 18 or area > 1800 or min(bw, bh) < 3:
            continue
        ys, xs = np.where(dark_labels[y : y + bh, x : x + bw] == label)
        xs = xs + x
        ys = ys + y
        if len(xs) < 3:
            continue
        coords = np.column_stack((xs, ys)).astype(np.float32)
        centered = coords - coords.mean(axis=0)
        covariance = centered.T @ centered / len(coords)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        if eigenvalues[0] <= 0 or eigenvalues[1] / eigenvalues[0] < 1.25:
            continue
        axis = eigenvectors[:, 1]
        cx, cy = dark_centers[label]
        margin = 5
        x0, x1 = max(0, x - margin), min(w, x + bw + margin)
        y0, y1 = max(0, y - margin), min(h, y + bh + margin)
        region = (dark_labels[y0:y1, x0:x1] == label).astype(np.uint8)
        expanded = cv2.dilate(region, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)))
        adjacent = np.unique(bright_labels[y0:y1, x0:x1][(expanded > 0)])
        best = None
        for bright_label in adjacent:
            if bright_label == 0:
                continue
            _, _, _, _, bright_area = bright_stats[bright_label]
            if bright_area < 12 or bright_area > 5000:
                continue
            bx, by = bright_centers[bright_label]
            dx, dy = cx - bx, cy - by
            length = math.hypot(dx, dy)
            if length < 4.0 or length > 100.0:
                continue
            alignment = abs((dx * axis[0] + dy * axis[1]) / length)
            if alignment < 0.65:
                continue
            candidate = (alignment, dx, dy, length, bright_label)
            if best is None or candidate[0] > best[0]:
                best = candidate
        if best is None:
            continue
        alignment, dx, dy, length, bright_label = best
        dark_contrast = float(dark_strength[ys, xs].mean())
        bx, by, bbw, bbh, _ = bright_stats[bright_label]
        bright_region = (bright_labels[by : by + bbh, bx : bx + bbw] == bright_label)
        bright_contrast = float(residual[by : by + bbh, bx : bx + bbw][bright_region].mean())
        weight = math.sqrt(float(area)) * max(1.0, dark_contrast + bright_contrast) * alignment
        vectors.append((dx / length, dy / length))
        weights.append(weight)
    if len(vectors) < 2:
        return float("nan"), 0.0, len(vectors)
    vector_array = np.asarray(vectors)
    weight_array = np.asarray(weights)
    sx = float(np.sum(vector_array[:, 0] * weight_array))
    sy = float(np.sum(vector_array[:, 1] * weight_array))
    confidence = min(1.0, math.hypot(sx, sy) / float(weight_array.sum()))
    angle = float(np.degrees(np.arctan2(sy, sx)) % 360.0)
    return angle, confidence, len(vectors)


def load_alto() -> tuple[pd.DataFrame, zipfile.ZipFile]:
    archive = zipfile.ZipFile(ZIP)
    query = pd.read_csv(archive.open("Val/query.csv"))
    return query, archive


def travelled_and_cut(query: pd.DataFrame) -> tuple[np.ndarray, int]:
    xy = query[["easting", "northing"]].to_numpy(dtype=float)
    travelled = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    jam = int(np.searchsorted(travelled, JAM_AT_M))
    return travelled, jam


def gnss_course(query: pd.DataFrame, stop: int) -> np.ndarray:
    """Course over ground only through the GNSS calibration interval."""
    xy = query.loc[:stop, ["easting", "northing"]].to_numpy(dtype=float)
    steps = np.diff(xy, axis=0)
    bearings = np.degrees(np.arctan2(steps[:, 0], steps[:, 1])) % 360.0
    course = np.full(stop + 1, np.nan)
    if len(bearings):
        course[1:] = bearings
        course[0] = bearings[0]
    return course


def decode_heading_truth(query: pd.DataFrame) -> np.ndarray:
    """Exact ECEF-quaternion to local compass heading conversion from m_alto_orientation."""
    xy = query[["easting", "northing"]].to_numpy(dtype=float)
    lon, lat = Transformer.from_crs("EPSG:32617", "EPSG:4326", always_xy=True).transform(
        xy[:, 0], xy[:, 1]
    )
    la, lo = np.radians(lat), np.radians(lon)
    east = np.stack([-np.sin(lo), np.cos(lo), np.zeros_like(lo)], axis=1)
    north = np.stack([-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)], axis=1)
    rotation = Rotation.from_quat(
        query[["orient_x", "orient_y", "orient_z", "orient_w"]].to_numpy(dtype=float)
    )
    forward = rotation.apply([1, 0, 0])
    return np.degrees(np.arctan2((forward * east).sum(1), (forward * north).sum(1))) % 360.0


def evaluate_kill_test(count: int = 100) -> dict:
    """First 100-frame kill gate; no quaternion truth is decoded or scored here."""
    query, archive = load_alto()
    n = min(count, len(query))
    course = gnss_course(query, n - 1)
    raw_compass = np.full(n, np.nan)
    confidence = np.zeros(n)
    supports = np.zeros(n, dtype=int)
    for k in range(n):
        data = np.frombuffer(archive.read(f"Val/query_images/{query.name[k]}"), dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        angle, confidence[k], supports[k] = illumination_vector(frame)
        if np.isfinite(angle) and k > 0:
            raw_compass[k] = (course[k] + angle) % 360.0
    valid = np.isfinite(raw_compass) & (confidence >= R_MIN) & (supports >= MIN_EDGES)
    dispersion = circ_std_deg(raw_compass[valid]) if valid.any() else 180.0
    fired = bool(dispersion > 10.0)
    result = {
        "label": "MEASURED",
        "frames": int(n),
        "valid_frames": int(valid.sum()),
        "valid_fraction": float(valid.mean()),
        "confidence_threshold": R_MIN,
        "edge_support_minimum": MIN_EDGES,
        "raw_course_plus_image_circular_std_deg": float(dispersion),
        "kill_threshold_deg": 10.0,
        "kill_fired": fired,
        "interpretation": "kill if circular standard deviation of image azimuth + GNSS course exceeds 10 degrees",
    }
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "kill_test.json"), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    print(
        f"KILL TEST (first {n} ALTO frames; MEASURED): valid {valid.sum()}/{n} "
        f"({valid.mean():.1%}), circular dispersion {dispersion:.2f} deg "
        f"(kill > 10 deg): {'FIRED' if fired else 'not fired'}"
    )
    return result


def run_explicit_shadow_test(count: int = 100, patches_per_ortho: int = 30) -> dict:
    """Score explicit object-shadow pairs on ALTO and north-up Wufeng patch grids."""
    query, archive = load_alto()
    n = min(count, len(query))
    heading_truth = decode_heading_truth(query.iloc[:n])
    alto_world_shadow = np.full(n, np.nan)
    confidence = np.zeros(n)
    pairs = np.zeros(n, dtype=int)
    for k in range(n):
        encoded = np.frombuffer(archive.read(f"Val/query_images/{query.name[k]}"), dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        angle, confidence[k], pairs[k] = shadow_silhouette_vector(frame)
        if np.isfinite(angle):
            # The image x-axis points along aircraft forward: world bearing is image azimuth + heading.
            alto_world_shadow[k] = (angle + heading_truth[k]) % 360.0
    valid = np.isfinite(alto_world_shadow) & (confidence >= 0.25) & (pairs >= 3)
    alto_angles = alto_world_shadow[valid]
    alto_std = circ_std_deg(alto_angles) if len(alto_angles) >= 2 else None
    alto_center = circ_mean_deg(alto_angles) if len(alto_angles) else None
    alto_p95 = (
        float(np.percentile(np.abs(circular_delta(alto_angles, alto_center)), 95))
        if len(alto_angles) else None
    )
    alto = {
        "label": "MEASURED",
        "frames_tested": int(n),
        "frames_valid": int(valid.sum()),
        "valid_fraction": float(valid.mean()),
        "min_confidence": 0.25,
        "min_shadow_object_pairs": 3,
        "world_shadow_bearing_mean_deg": alto_center,
        "circular_std_deg": alto_std,
        "p95_absolute_deviation_deg": alto_p95,
        "kill_threshold_deg": 10.0,
        "kill_fired": bool(alto_std is None or alto_std > 10.0 or valid.mean() < 0.20),
    }
    wufeng = []
    for path in WUFENG:
        with rasterio.open(path) as ds:
            tile_size = 512
            rows = np.linspace(tile_size // 2, ds.height - tile_size // 2, 5).round().astype(int)
            cols = np.linspace(tile_size // 2, ds.width - tile_size // 2, 6).round().astype(int)
            angles = []
            confidences = []
            supports = []
            for cy in rows:
                for cx in cols:
                    top, left = int(cy - tile_size // 2), int(cx - tile_size // 2)
                    rgb = np.moveaxis(
                        ds.read(
                            (1, 2, 3),
                            window=((top, top + tile_size), (left, left + tile_size)),
                        ),
                        0,
                        -1,
                    )
                    angle, conf, support = shadow_silhouette_vector(rgb, rgb=True)
                    if np.isfinite(angle) and conf >= 0.25 and support >= 3:
                        angles.append(raster_world_bearing(angle, ds.transform))
                        confidences.append(conf)
                        supports.append(support)
            center = circ_mean_deg(np.asarray(angles)) if angles else None
            spread = circ_std_deg(np.asarray(angles)) if len(angles) >= 2 else None
            p95 = (
                float(np.percentile(np.abs(circular_delta(np.asarray(angles), center)), 95))
                if angles else None
            )
            wufeng.append({
                "label": "MEASURED",
                "file": path,
                "patches_tested": int(patches_per_ortho),
                "patches_valid": len(angles),
                "valid_fraction": float(len(angles) / patches_per_ortho),
                "world_shadow_bearing_mean_deg": center,
                "circular_std_deg": spread,
                "p95_absolute_patch_deviation_deg": p95,
                "pass_3_deg": bool(spread is not None and spread <= 3.0),
            })
    result = {
        "experiment": "X1 explicit shadow silhouettes",
        "method": "dark connected regions paired with adjacent bright components; shadow vector from bright-object centroid to dark-region centroid",
        "alto": alto,
        "wufeng": wufeng,
        "kill_fired": alto["kill_fired"],
    }
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "explicit_shadow_test.json"), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    print(
        f"EXPLICIT SHADOW ALTO (MEASURED): valid {valid.sum()}/{n} ({valid.mean():.1%}); "
        f"image azimuth + quaternion heading circular std {alto_std} deg; "
        f"p95 deviation {alto_p95} deg; kill >10 deg: "
        f"{'FIRED' if alto['kill_fired'] else 'not fired'}"
    )
    for row in wufeng:
        print(
            f"EXPLICIT SHADOW WUFENG (MEASURED) {os.path.basename(row['file'])}: "
            f"valid patches {row['patches_valid']}/{row['patches_tested']} "
            f"({row['valid_fraction']:.1%}); circular std {row['circular_std_deg']} deg; "
            f"p95 deviation {row['p95_absolute_patch_deviation_deg']} deg; "
            f"criterion <=3 deg: {'PASS' if row['pass_3_deg'] else 'FAIL'}"
        )
    return result


def smooth_circular(values: np.ndarray, valid: np.ndarray, window: int = 10) -> tuple[np.ndarray, np.ndarray]:
    result = np.full(len(values), np.nan)
    counts = np.zeros(len(values), dtype=int)
    for k in range(len(values)):
        start = max(0, k - window + 1)
        sample = values[start : k + 1]
        keep = valid[start : k + 1] & np.isfinite(sample)
        counts[k] = int(keep.sum())
        if counts[k] >= max(5, window // 2):
            result[k] = circ_mean_deg(sample[keep])
    return result, counts


def error_stats(errors: np.ndarray) -> dict:
    errors = np.asarray(errors, dtype=float)
    errors = errors[np.isfinite(errors)]
    if not len(errors):
        return {"count": 0, "p50_deg": None, "p95_deg": None}
    return {
        "count": int(len(errors)),
        "p50_deg": float(np.percentile(errors, 50)),
        "p95_deg": float(np.percentile(errors, 95)),
    }


def run_alto() -> dict:
    query, archive = load_alto()
    travelled, jam = travelled_and_cut(query)
    course = gnss_course(query, jam)
    n = len(query)
    image_angle = np.full(n, np.nan)
    confidence = np.zeros(n)
    support = np.zeros(n, dtype=int)
    started = time.time()
    for k, name in enumerate(query.name):
        data = np.frombuffer(archive.read(f"Val/query_images/{name}"), dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        image_angle[k], confidence[k], support[k] = illumination_vector(frame)
    valid = np.isfinite(image_angle) & (confidence >= R_MIN) & (support >= MIN_EDGES)
    calib = (np.arange(n) > 0) & (np.arange(n) <= jam) & valid
    if not calib.any():
        raise RuntimeError("No valid shadow direction in the 300 m GNSS calibration interval")
    # Dark-to-light gradient points toward illumination: image angle = sun azimuth - heading.
    # Therefore heading estimate is offset - image angle; offset is the sole fitted quantity.
    offset = circ_mean_deg((course[calib] + image_angle[calib]) % 360.0)
    heading_est = (offset - image_angle) % 360.0
    # Quaternion headings are decoded only after estimates and calibration are fixed,
    # and used only to score the post-cut section.
    heading_truth = decode_heading_truth(query)
    post = np.arange(n) > jam
    scored = post & valid
    heading_error = circular_delta(heading_est, heading_truth)
    abs_error = np.abs(heading_error[scored])
    smoothed, smooth_count = smooth_circular(heading_est, valid, 10)
    smooth_scored = post & np.isfinite(smoothed)
    smooth_error = np.abs(circular_delta(smoothed[smooth_scored], heading_truth[smooth_scored]))
    valid_errors = heading_error[scored]
    if len(valid_errors):
        count = max(1, int(math.ceil(len(valid_errors) * 0.10)))
        early_bias = circ_mean_deg(valid_errors[:count])
        late_bias = circ_mean_deg(valid_errors[-count:])
        drift = float(circular_delta(late_bias, early_bias))
    else:
        early_bias = late_bias = drift = None
    out = {
        "label": "MEASURED",
        "frames_total": int(n),
        "cut_distance_m": float(travelled[jam]),
        "cut_frame": int(jam),
        "calibration_valid_frames": int(calib.sum()),
        "offset_deg": float(offset),
        "post_cut_frames": int(post.sum()),
        "valid_frames": int(scored.sum()),
        "valid_fraction": float(scored.sum() / post.sum()),
        "heading_error": error_stats(abs_error),
        "10_frame_smoothed_heading_error": error_stats(smooth_error),
        "drift": {
            "definition": "circular change in mean signed heading error between first and last 10% of valid post-cut frames",
            "early_bias_deg": early_bias,
            "late_bias_deg": late_bias,
            "net_deg": drift,
        },
        "kill_criteria": {
            "p50_over_5_deg": bool(len(abs_error) and np.percentile(abs_error, 50) > 5.0),
            "valid_fraction_below_20_percent": bool(scored.sum() / post.sum() < 0.20),
            "initial_100_frame_dispersion_over_10_deg": None,
        },
        "detector": {"resultant_min": R_MIN, "edge_support_min": MIN_EDGES},
        "runtime_seconds": float(time.time() - started),
    }
    os.makedirs(OUT, exist_ok=True)
    table = pd.DataFrame({
        "frame": np.arange(n),
        "distance_m": travelled,
        "image_direction_deg": image_angle,
        "resultant": confidence,
        "edge_support": support,
        "valid": valid,
        "heading_est_deg": heading_est,
        "heading_truth_deg_scoring_only": np.where(post, heading_truth, np.nan),
        "heading_error_deg_scoring_only": np.where(post, heading_error, np.nan),
        "smoothed_heading_est_deg": smoothed,
        "smoothed_valid_samples": smooth_count,
    })
    table.to_csv(os.path.join(OUT, "alto_frames.csv"), index=False)
    with open(os.path.join(OUT, "alto_summary.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print(
        f"ALTO (MEASURED): cut {travelled[jam]:.1f} m/frame {jam}; offset {offset:.2f} deg; "
        f"valid {scored.sum()}/{post.sum()} ({out['valid_fraction']:.1%}); "
        f"heading p50/p95 {out['heading_error']['p50_deg']} / {out['heading_error']['p95_deg']} deg; "
        f"10-frame smoothed p50/p95 {out['10_frame_smoothed_heading_error']['p50_deg']} / "
        f"{out['10_frame_smoothed_heading_error']['p95_deg']} deg; drift {drift} deg; "
        f"runtime {out['runtime_seconds']:.1f} s"
    )
    return out


def raster_world_bearing(angle_deg: float, transform) -> float:
    angle = math.radians(angle_deg)
    col, row = math.cos(angle), math.sin(angle)
    east = transform.a * col + transform.b * row
    north = transform.d * col + transform.e * row
    return float(np.degrees(np.arctan2(east, north)) % 360.0)


def run_wufeng(tile_size: int = 512, stride: int = 1024) -> list[dict]:
    results = []
    for path in WUFENG:
        with rasterio.open(path) as ds:
            tile_angles, tile_conf, tile_support = [], [], []
            for top in range(0, ds.height - tile_size + 1, stride):
                for left in range(0, ds.width - tile_size + 1, stride):
                    rgb = np.moveaxis(ds.read((1, 2, 3), window=((top, top + tile_size),
                                                                   (left, left + tile_size))), 0, -1)
                    angle, confidence, support = illumination_vector(rgb, rgb=True)
                    if np.isfinite(angle) and confidence >= R_MIN and support >= MIN_EDGES:
                        tile_angles.append(raster_world_bearing(angle, ds.transform))
                        tile_conf.append(confidence)
                        tile_support.append(support)
            if tile_angles:
                center = circ_mean_deg(np.asarray(tile_angles))
                residuals = circular_delta(np.asarray(tile_angles), center)
                spread = circ_std_deg(np.asarray(tile_angles))
                p95 = float(np.percentile(np.abs(residuals), 95))
            else:
                center, spread, p95 = None, None, None
            result = {
                "label": "MEASURED",
                "file": path,
                "tile_size_px": tile_size,
                "stride_px": stride,
                "tiles_valid": len(tile_angles),
                "tiles_tested": int(max(0, ((ds.height - tile_size) // stride + 1) *
                                             ((ds.width - tile_size) // stride + 1))),
                "valid_fraction": float(len(tile_angles) / max(1, ((ds.height - tile_size) // stride + 1) *
                                                        ((ds.width - tile_size) // stride + 1))),
                "mean_world_illumination_bearing_deg": center,
                "circular_std_deg": spread,
                "p95_absolute_patch_deviation_deg": p95,
                "pass_3_deg": bool(spread is not None and spread <= 3.0),
            }
            results.append(result)
            print(
                f"WUFENG (MEASURED) {os.path.basename(path)}: valid patches "
                f"{len(tile_angles)}/{result['tiles_tested']} ({result['valid_fraction']:.1%}); "
                f"circular std {spread} deg; p95 patch deviation {p95} deg; "
                f"criterion <= 3 deg: {'PASS' if result['pass_3_deg'] else 'FAIL'}"
            )
    with open(os.path.join(OUT, "wufeng_summary.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    pd.DataFrame(results).to_csv(os.path.join(OUT, "wufeng_summary.csv"), index=False)
    return results


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kill-test", action="store_true", help="run only the first 100-frame gradient kill gate")
    parser.add_argument("--explicit-shadow-test", action="store_true", help="score silhouette shadows on ALTO and Wufeng patches")
    args = parser.parse_args()
    if args.explicit_shadow_test:
        result = run_explicit_shadow_test()
        return 2 if result["kill_fired"] else 0
    if args.kill_test:
        result = evaluate_kill_test()
        return 2 if result["kill_fired"] else 0
    kill = evaluate_kill_test()
    if kill["kill_fired"]:
        print("STOP: the 20-minute kill criterion fired; full experiment not run.")
        return 2
    alto = run_alto()
    wufeng = run_wufeng()
    kill["full_experiment_kill_fired"] = bool(
        alto["kill_criteria"]["p50_over_5_deg"]
        or alto["kill_criteria"]["valid_fraction_below_20_percent"]
    )
    with open(os.path.join(OUT, "kill_test.json"), "w", encoding="utf-8") as f:
        json.dump(kill, f, indent=2)
    combined = {
        "experiment": "X1 shadow/illumination compass",
        "alto": alto,
        "wufeng": wufeng,
        "kill_test": kill,
    }
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(combined, f, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

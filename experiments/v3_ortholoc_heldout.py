#!/usr/bin/env python3
"""Clean held-out OrthoLoC camera-to-DOP localization benchmark.

Run from the repository root:
  .venv/bin/python experiments/v3_ortholoc_heldout.py

The real query images and their paired DOP/DSM are held out from method and
threshold development. Query rectification and prior errors are declared
SIMULATED; ground truth is used only to generate noisy pose inputs, choose the
prior-centred search window, and score the resulting map fix.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from scipy.stats import beta

ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "data/raw/ortholoc/test_outPlace_L08_xDOP"
OUT_DIR = ROOT / "data/processed/v3_ortholoc_heldout"
MODELS_DIR = ROOT / "data/raw/models/accelerated_features"
sys.path.insert(0, str(ROOT / "experiments"))
import r_integrity as integrity  # noqa: E402
import r_map_benchmark as track_a  # noqa: E402

SEED = 20261003
PRIOR_HALF_RANGE_M = 40.0
ROTATION_NOISE_STD_DEG = 1.0
HEIGHT_NOISE_STD_FRACTION = 0.03
QUAD_ACCEPT_N = 3
UNION_TOL_M = 10.0
XFEAT_TOP_K = 2048
MATCH_METHODS = ("zncc", "zncc_yaw_scale", "xfeat_affine")
VARIANTS = ("rectified_perturbed_pose", "identity_warp")


@dataclass
class OrthoSample:
    order: int
    path: Path
    sample_id: str
    query_gray: np.ndarray
    map_gray: np.ndarray
    valid: np.ndarray
    pixel_to_xy: np.ndarray
    camera_xy: np.ndarray
    camera_z: float
    plane_z: float
    height_m: float
    rotation_world_to_camera: np.ndarray
    intrinsics: np.ndarray
    off_nadir_deg: float
    map_gsd_m_px: float
    projection_error_median_px: float
    projection_error_p95_px: float
    sha256: str

    def xy_to_pixel(self, xy: np.ndarray) -> np.ndarray:
        matrix = self.pixel_to_xy[:2, :].T
        return np.linalg.solve(matrix, np.asarray(xy, dtype=float) - self.pixel_to_xy[2, :])

    def pixel_to_world(self, uv: np.ndarray) -> np.ndarray:
        matrix = self.pixel_to_xy[:2, :].T
        return matrix @ np.asarray(uv, dtype=float) + self.pixel_to_xy[2, :]

    def search_size_px(self) -> int:
        """Cover +/-40 m prior error and the full 96 px ZNCC template."""
        matrix = self.pixel_to_xy[:2, :].T
        xy_to_pixel = np.linalg.inv(matrix)
        corners = np.array(
            [(-1, -1), (-1, 1), (1, -1), (1, 1)], dtype=float
        ) * PRIOR_HALF_RANGE_M
        max_prior_px = np.max(np.abs((xy_to_pixel @ corners.T).T), axis=0)
        half_px = float(np.max(max_prior_px)) + track_a.TEMPLATE / 2.0 + 2.0
        size = int(math.ceil(2.0 * half_px))
        return size + (size % 2)


@dataclass
class FeatureResult:
    features: dict | None
    elapsed_ms: float


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _projection_errors(
    point_map: np.ndarray, rotation: np.ndarray, translation: np.ndarray, intrinsics: np.ndarray
) -> np.ndarray:
    finite = np.isfinite(point_map).all(axis=2)
    ys, xs = np.where(finite)
    if len(xs) > 1024:
        chosen = np.linspace(0, len(xs) - 1, 1024).astype(int)
        xs, ys = xs[chosen], ys[chosen]
    points = point_map[ys, xs].astype(np.float64)
    camera = (rotation @ points.T).T + translation
    projected = (intrinsics @ camera.T).T
    good = camera[:, 2] > 0
    pixels = projected[good, :2] / projected[good, 2:3]
    expected = np.column_stack((xs[good], ys[good])).astype(np.float64)
    return np.linalg.norm(pixels - expected, axis=1)


def load_sample(path: Path, order: int) -> OrthoSample:
    with np.load(path, allow_pickle=False) as data:
        required = {"sample_id", "image_query", "image_dop", "dsm", "point_map", "extrinsics", "intrinsics"}
        missing = required.difference(data.files)
        if missing:
            raise ValueError(f"{path}: missing fields {sorted(missing)}")
        query_rgb = data["image_query"]
        map_rgb = data["image_dop"]
        dsm = data["dsm"].astype(np.float64)
        point_map = data["point_map"]
        extrinsics = data["extrinsics"].astype(np.float64)
        intrinsics = data["intrinsics"].astype(np.float64)
        sample_id = str(data["sample_id"].item())

    if query_rgb.ndim != 3 or map_rgb.ndim != 3 or dsm.ndim != 3 or dsm.shape[2] < 3:
        raise ValueError(f"{path}: unexpected image/DSM dimensions")
    if extrinsics.shape != (3, 4) or intrinsics.shape != (3, 3):
        raise ValueError(f"{path}: unexpected calibration dimensions")

    rotation = extrinsics[:, :3]
    translation = extrinsics[:, 3]
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=2e-4):
        raise ValueError(f"{path}: extrinsic rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=2e-4):
        raise ValueError(f"{path}: extrinsic rotation determinant is not +1")

    # E=[R|t] maps local-frame 3-D points into camera coordinates. This is
    # verified against the released query-pixel point_map below.
    camera_position = -rotation.T @ translation
    forward_world = rotation.T @ np.array([0.0, 0.0, 1.0])
    off_nadir = float(np.degrees(np.arccos(np.clip(-forward_world[2], -1.0, 1.0))))

    valid = np.isfinite(dsm).all(axis=2)
    ys, xs = np.where(valid)
    design = np.column_stack((xs, ys, np.ones(len(xs), dtype=np.float64)))
    pixel_to_xy, *_ = np.linalg.lstsq(design, dsm[valid, :2], rcond=None)
    xy_matrix = pixel_to_xy[:2, :].T
    map_gsd = float(np.sqrt(abs(np.linalg.det(xy_matrix))))
    if not np.isfinite(map_gsd) or map_gsd <= 0:
        raise ValueError(f"{path}: invalid DOP pixel-to-local-coordinate transform")

    plane_z = float(np.nanmedian(dsm[:, :, 2]))
    height = float(camera_position[2] - plane_z)
    if height <= 0:
        raise ValueError(f"{path}: non-positive camera height above median DSM")
    reprojection_error = _projection_errors(point_map, rotation, translation, intrinsics)
    if not len(reprojection_error):
        raise ValueError(f"{path}: point_map has no projectable finite points")

    query_gray = cv2.cvtColor(query_rgb, cv2.COLOR_RGB2GRAY)
    map_gray = cv2.cvtColor(map_rgb, cv2.COLOR_RGB2GRAY)
    return OrthoSample(
        order=order,
        path=path,
        sample_id=sample_id,
        query_gray=np.ascontiguousarray(query_gray),
        map_gray=np.ascontiguousarray(map_gray),
        valid=np.ascontiguousarray(valid.astype(np.uint8)),
        pixel_to_xy=pixel_to_xy,
        camera_xy=camera_position[:2].copy(),
        camera_z=float(camera_position[2]),
        plane_z=plane_z,
        height_m=height,
        rotation_world_to_camera=rotation,
        intrinsics=intrinsics,
        off_nadir_deg=off_nadir,
        map_gsd_m_px=map_gsd,
        projection_error_median_px=float(np.median(reprojection_error)),
        projection_error_p95_px=float(np.percentile(reprojection_error, 95)),
        sha256=_sha256(path),
    )


def _rectified_query(
    sample: OrthoSample, rotation_noise_deg: np.ndarray, estimated_height_m: float
) -> tuple[np.ndarray, float, float]:
    # The horizontal origin is the camera's vertical projection; translation
    # is removed, so absolute truth position is never embedded in the query.
    rotation_camera_to_world = sample.rotation_world_to_camera.T
    noise_rotation = Rotation.from_euler("xyz", rotation_noise_deg, degrees=True).as_matrix()
    estimated_world_to_camera = (rotation_camera_to_world @ noise_rotation).T
    camera_relative = np.array([0.0, 0.0, estimated_height_m])
    translation_relative = -estimated_world_to_camera @ camera_relative
    plane_to_image = sample.intrinsics @ np.column_stack(
        (estimated_world_to_camera[:, 0], estimated_world_to_camera[:, 1], translation_relative)
    )
    image_to_plane = np.linalg.inv(plane_to_image)

    xy_to_pixel = np.linalg.inv(sample.pixel_to_xy[:2, :].T)
    xy_to_pixel_h = np.eye(3, dtype=np.float64)
    xy_to_pixel_h[:2, :2] = xy_to_pixel
    center = track_a.QUERY / 2.0
    output_shift = np.array([[1.0, 0.0, center], [0.0, 1.0, center], [0.0, 0.0, 1.0]])
    image_to_query = output_shift @ xy_to_pixel_h @ image_to_plane

    size = track_a.QUERY
    query = cv2.warpPerspective(
        sample.query_gray,
        image_to_query,
        (size, size),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    source_mask = np.full(sample.query_gray.shape, 255, dtype=np.uint8)
    coverage = cv2.warpPerspective(
        source_mask,
        image_to_query,
        (size, size),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    ) > 0
    fill = int(np.median(query[coverage])) if coverage.any() else 0
    query[~coverage] = fill
    core = coverage[track_a.QUERY // 2 - track_a.TEMPLATE // 2:track_a.QUERY // 2 + track_a.TEMPLATE // 2,
                     track_a.QUERY // 2 - track_a.TEMPLATE // 2:track_a.QUERY // 2 + track_a.TEMPLATE // 2]
    return np.ascontiguousarray(query), float(coverage.mean()), float(core.mean())


def _identity_query(sample: OrthoSample, estimated_height_m: float) -> tuple[np.ndarray, float, float]:
    """Scale-only principal-point crop: no attitude/perspective correction."""
    size = track_a.QUERY
    fx, fy = float(sample.intrinsics[0, 0]), float(sample.intrinsics[1, 1])
    cx, cy = float(sample.intrinsics[0, 2]), float(sample.intrinsics[1, 2])
    sx = size * sample.map_gsd_m_px * fx / estimated_height_m
    sy = size * sample.map_gsd_m_px * fy / estimated_height_m
    transform = np.array(
        [[size / sx, 0.0, size / 2.0 - (size / sx) * cx],
         [0.0, size / sy, size / 2.0 - (size / sy) * cy],
         [0.0, 0.0, 1.0]], dtype=np.float64,
    )
    query = cv2.warpPerspective(
        sample.query_gray,
        transform,
        (size, size),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    coverage = cv2.warpPerspective(
        np.full(sample.query_gray.shape, 255, dtype=np.uint8),
        transform,
        (size, size),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    ) > 0
    fill = int(np.median(query[coverage])) if coverage.any() else 0
    query[~coverage] = fill
    return np.ascontiguousarray(query), float(coverage.mean()), float(coverage.mean())


def _make_windows(
    query: OrthoSample, references: list[OrthoSample], prior_error_xy: np.ndarray
) -> list[dict]:
    windows = []
    for reference in references:
        size = reference.search_size_px()
        height, width = reference.map_gray.shape
        if size > min(height, width):
            raise ValueError(
                f"{reference.sample_id}: {size}px search window exceeds {width}x{height} DOP"
            )
        prior_xy = reference.camera_xy + prior_error_xy
        prior_uv = reference.xy_to_pixel(prior_xy)
        x0 = int(np.rint(prior_uv[0] - size / 2.0))
        y0 = int(np.rint(prior_uv[1] - size / 2.0))
        x0 = min(max(0, x0), width - size)
        y0 = min(max(0, y0), height - size)
        image = np.ascontiguousarray(reference.map_gray[y0:y0 + size, x0:x0 + size])
        valid = np.ascontiguousarray(reference.valid[y0:y0 + size, x0:x0 + size])
        windows.append(
            {
                "reference": reference,
                "image": image,
                "valid": valid,
                "x0": x0,
                "y0": y0,
                "size": size,
                "prior_xy": prior_xy,
            }
        )
    return windows


def _extract_xfeat(model, gray: np.ndarray) -> FeatureResult:
    started = time.perf_counter()
    try:
        rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        tensor = model.parse_input(rgb)
        features = model.detectAndCompute(tensor, top_k=XFEAT_TOP_K)[0]
    except (IndexError, RuntimeError):
        features = None
    return FeatureResult(features, (time.perf_counter() - started) * 1000.0)


def _cached_xfeat_fix(model, query: FeatureResult, reference: FeatureResult, shape: tuple[int, ...]):
    started = time.perf_counter()
    empty = np.zeros((0, 2), dtype=np.float32)
    if query.features is None or reference.features is None:
        point, stats = track_a.geometric_fix(empty, empty, "affine", shape)
    else:
        try:
            i0, i1 = model.match(
                query.features["descriptors"], reference.features["descriptors"], min_cossim=-1
            )
            points_query = query.features["keypoints"][i0].cpu().numpy().astype(np.float32)
            points_reference = reference.features["keypoints"][i1].cpu().numpy().astype(np.float32)
        except (IndexError, RuntimeError):
            points_query, points_reference = empty, empty
        point, stats = track_a.geometric_fix(points_query, points_reference, "affine", shape)
    match_geometry_ms = (time.perf_counter() - started) * 1000.0
    estimated_direct_latency_ms = query.elapsed_ms + reference.elapsed_ms + match_geometry_ms
    return point, stats, estimated_direct_latency_ms


def _check_cached_xfeat_parity(
    model, query_gray: np.ndarray, reference_gray: np.ndarray, valid: np.ndarray, seed: int
) -> dict:
    query_features = _extract_xfeat(model, query_gray)
    reference_features = _extract_xfeat(model, reference_gray)
    cv2.setRNGSeed(seed)
    direct_point, direct_stats = track_a.run_method("xfeat_affine", query_gray, reference_gray, valid)
    cv2.setRNGSeed(seed)
    cached_point, cached_stats, _ = _cached_xfeat_fix(
        model, query_features, reference_features, reference_gray.shape
    )
    same_points = np.allclose(direct_point, cached_point, equal_nan=True, atol=1e-5)
    same_stats = all(
        (not isinstance(direct_stats.get(key), (int, float, np.number)))
        or np.isclose(direct_stats[key], cached_stats.get(key, np.nan), equal_nan=True, atol=1e-5)
        for key in ("score", "matches", "inliers", "inlier_ratio", "spread", "fit_scale")
    )
    if not same_points or not same_stats:
        raise RuntimeError("Cached XFeat path does not reproduce the unchanged track-A matcher")
    cv2.setRNGSeed(seed)
    return {"checked": True, "point_equal": bool(same_points), "geometry_stats_equal": bool(same_stats)}


def _matcher_row(
    query: OrthoSample,
    reference: OrthoSample,
    variant: str,
    present: bool,
    prior_error_xy: np.ndarray,
    noise_xyz_deg: np.ndarray,
    height_error_fraction: float,
    query_coverages: tuple[float, float],
    window: dict,
    method: str,
    point: np.ndarray,
    stats: dict,
    latency_ms: float,
) -> dict:
    point = np.asarray(point, dtype=float).reshape(-1)
    finite = point.size >= 2 and np.isfinite(point[:2]).all()
    if finite:
        map_uv = np.array([window["x0"], window["y0"]], dtype=float) + point[:2]
        estimate_xy = reference.pixel_to_world(map_uv)
        error = float(np.linalg.norm(estimate_xy - query.camera_xy)) if present else math.nan
    else:
        estimate_xy = np.array([math.nan, math.nan])
        error = math.nan
    quad_n = float(stats.get("quad_n", math.nan))
    if method.startswith("zncc"):
        accepted = bool(finite and np.isfinite(quad_n) and quad_n >= QUAD_ACCEPT_N)
    else:
        accepted = bool(finite)
    return {
        "site": "OrthoLoC_L08_outPlace",
        "pair": reference.sample_id,
        "centre_id": query.order,
        "condition": variant,
        "kind": "positive" if present else "negative",
        "query_site": query.sample_id,
        "present": present,
        "query_sample_id": query.sample_id,
        "map_sample_id": reference.sample_id,
        "query_index": query.order,
        "prior_error_x_m": float(prior_error_xy[0]),
        "prior_error_y_m": float(prior_error_xy[1]),
        "rotation_noise_x_deg": float(noise_xyz_deg[0]),
        "rotation_noise_y_deg": float(noise_xyz_deg[1]),
        "rotation_noise_z_deg": float(noise_xyz_deg[2]),
        "height_error_fraction": float(height_error_fraction),
        "off_nadir_deg": query.off_nadir_deg,
        "query_coverage_fraction": query_coverages[0],
        "query_core_coverage_fraction": query_coverages[1],
        "map_gsd_m_px": reference.map_gsd_m_px,
        "search_window_px": window["size"],
        "search_window_m": float(window["size"] * reference.map_gsd_m_px),
        "search_x0_px": window["x0"],
        "search_y0_px": window["y0"],
        "truth_x": float(query.camera_xy[0]) if present else float(reference.camera_xy[0]),
        "truth_y": float(query.camera_xy[1]) if present else float(reference.camera_xy[1]),
        "method": method,
        "estimate_x": float(estimate_xy[0]),
        "estimate_y": float(estimate_xy[1]),
        "error_m": error,
        "accepted": accepted,
        "fixed": accepted,
        "score": float(stats.get("score", math.nan)),
        "quad_n": quad_n,
        "inliers": float(stats.get("inliers", math.nan)),
        "inlier_ratio": float(stats.get("inlier_ratio", math.nan)),
        "gate": str(stats.get("gate", "")),
        "latency_ms": float(latency_ms),
    }


def _exact_upper95(k: int, n: int) -> float | None:
    if n <= 0:
        return None
    if k >= n:
        return 1.0
    return float(beta.ppf(0.95, k + 1, n - k))


def _summaries(rows: list[dict], query_count: int) -> list[dict]:
    frame = pd.DataFrame(rows)
    output = []
    for variant in VARIANTS:
        for method in (*MATCH_METHODS, "UNION"):
            group = frame[(frame.condition == variant) & (frame.method == method)]
            positives = group[group.present]
            negatives = group[~group.present]
            error = positives.error_m.to_numpy(dtype=float)
            accepted_pos = positives.accepted.to_numpy(dtype=bool)
            accepted_neg = negatives.accepted.to_numpy(dtype=bool)
            wrong_pos = accepted_pos & (error > 10.0)
            neg_k = int(accepted_neg.sum())
            neg_per_query = negatives.assign(accepted=accepted_neg).groupby("query_index").accepted.any()
            neg_query_k = int(neg_per_query.sum())
            false_queries = set(positives.loc[wrong_pos, "query_index"].astype(int))
            false_queries.update(neg_per_query[neg_per_query].index.astype(int))
            false_k = int(wrong_pos.sum() + neg_k)
            n_total = int(len(group))
            latency = group.latency_ms.to_numpy(dtype=float)
            accepted_correct = int((accepted_pos & (error <= 10.0)).sum())
            output.append(
                {
                    "variant": variant,
                    "method": method,
                    "positives_n": int(len(positives)),
                    "raw_success_within_5m_n": int(np.sum(error <= 5.0)),
                    "raw_success_within_5m_rate": float(np.sum(error <= 5.0) / max(1, len(positives))),
                    "raw_success_within_10m_n": int(np.sum(error <= 10.0)),
                    "raw_success_within_10m_rate": float(np.sum(error <= 10.0) / max(1, len(positives))),
                    "accepted_positive_n": int(accepted_pos.sum()),
                    "accepted_positive_rate": float(accepted_pos.mean()) if len(accepted_pos) else math.nan,
                    "accepted_correct_within_10m_n": accepted_correct,
                    "accepted_wrong_over_10m_n": int(wrong_pos.sum()),
                    "negative_pairs_n": int(len(negatives)),
                    "negative_accepted_n": neg_k,
                    "negative_accept_rate": float(neg_k / len(negatives)) if len(negatives) else math.nan,
                    "negative_accept_upper95_exact_pairwise": _exact_upper95(neg_k, int(len(negatives))),
                    "queries_with_any_negative_accept_n": neg_query_k,
                    "negative_any_map_upper95_exact_per_query": _exact_upper95(neg_query_k, query_count),
                    "all_trials_n": n_total,
                    "false_accepts_n": false_k,
                    "false_accept_upper95_exact_pairwise": _exact_upper95(false_k, n_total),
                    "queries_with_any_false_accept_n": len(false_queries),
                    "false_accept_upper95_exact_per_query": _exact_upper95(len(false_queries), query_count),
                    "latency_p50_ms": float(np.median(latency)) if len(latency) else math.nan,
                    "latency_p95_ms": float(np.percentile(latency, 95)) if len(latency) else math.nan,
                    "latency_note": (
                        "XFeat per-pair direct-equivalent latency: measured query-feature extraction + "
                        "measured reference-feature extraction + measured MNN/RANSAC; cached features "
                        "are charged on every pair. Other methods are directly timed. UNION sums all three."
                    ),
                }
            )
    return output


def _distribution(values: list[float]) -> dict:
    array = np.asarray(values, dtype=float)
    return {
        "n": int(len(array)),
        "min": float(np.min(array)),
        "p05": float(np.percentile(array, 5)),
        "p25": float(np.percentile(array, 25)),
        "median": float(np.median(array)),
        "p75": float(np.percentile(array, 75)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    return value


def run(args: argparse.Namespace) -> dict:
    cv2.setNumThreads(1)
    cv2.setRNGSeed(args.seed)
    rng = np.random.default_rng(args.seed)
    paths = sorted(DATA_DIR.glob("L08_xDOP*.npz"))
    if not paths:
        raise FileNotFoundError(f"No OrthoLoC samples under {DATA_DIR}")
    samples = [load_sample(path, index) for index, path in enumerate(paths)]
    query_count = min(len(samples), args.max_queries or len(samples))
    reference_count = min(len(samples), args.max_reference_maps or len(samples))
    queries = samples[:query_count]
    references = samples[:reference_count]
    if any(query not in references for query in queries):
        raise ValueError("The reference-map limit must include every query's paired positive DOP")

    query_data = []
    frame_rows = []
    for query in queries:
        rotation_noise = rng.normal(0.0, ROTATION_NOISE_STD_DEG, 3)
        height_error_fraction = float(rng.normal(0.0, HEIGHT_NOISE_STD_FRACTION))
        estimated_height = query.height_m * (1.0 + height_error_fraction)
        prior_error = rng.uniform(-PRIOR_HALF_RANGE_M, PRIOR_HALF_RANGE_M, 2)
        rectified, rect_cov, rect_core = _rectified_query(query, rotation_noise, estimated_height)
        identity, identity_cov, identity_core = _identity_query(query, estimated_height)
        query_data.append(
            {
                "sample": query,
                "rotation_noise_deg": rotation_noise,
                "height_error_fraction": height_error_fraction,
                "estimated_height_m": estimated_height,
                "prior_error_xy": prior_error,
                "variants": {
                    "rectified_perturbed_pose": (rectified, (rect_cov, rect_core)),
                    "identity_warp": (identity, (identity_cov, identity_core)),
                },
            }
        )
        frame_rows.append(
            {
                "query_index": query.order,
                "sample_id": query.sample_id,
                "off_nadir_deg": query.off_nadir_deg,
                "camera_x_local_m": float(query.camera_xy[0]),
                "camera_y_local_m": float(query.camera_xy[1]),
                "camera_z_local_m": query.camera_z,
                "median_dsm_z_local_m": query.plane_z,
                "true_height_above_median_dsm_m": query.height_m,
                "map_gsd_m_px": query.map_gsd_m_px,
                "rotation_noise_x_deg": float(rotation_noise[0]),
                "rotation_noise_y_deg": float(rotation_noise[1]),
                "rotation_noise_z_deg": float(rotation_noise[2]),
                "height_error_fraction": height_error_fraction,
                "height_error_percent": 100.0 * height_error_fraction,
                "prior_error_x_m": float(prior_error[0]),
                "prior_error_y_m": float(prior_error[1]),
                "rectified_query_valid_fraction": rect_cov,
                "rectified_core_valid_fraction": rect_core,
                "identity_query_valid_fraction": identity_cov,
                "identity_core_valid_fraction": identity_core,
                "point_map_reprojection_median_px": query.projection_error_median_px,
                "point_map_reprojection_p95_px": query.projection_error_p95_px,
                "source_npz_sha256": query.sha256,
            }
        )

    # Load and warm the frozen Track-A XFeat model once. Feature caching only
    # avoids repeating identical extraction; MNN and the affine/RANSAC gates
    # below are the same unchanged code path and are parity-checked.
    model = track_a.xfeat()
    raw_rows: list[dict] = []
    parity = None
    for qdata in query_data:
        query = qdata["sample"]
        prior_error = qdata["prior_error_xy"]
        windows = _make_windows(query, references, prior_error)
        query_features: dict[str, FeatureResult] = {}
        reference_features: dict[int, FeatureResult] = {}
        for variant, (query_image, coverages) in qdata["variants"].items():
            qfeatures = _extract_xfeat(model, query_image)
            query_features[variant] = qfeatures
            if parity is None:
                positive_window = next(w for w in windows if w["reference"].sample_id == query.sample_id)
                ref = positive_window["reference"]
                cv2.setRNGSeed(args.seed)
                parity = _check_cached_xfeat_parity(
                    model, query_image, positive_window["image"], positive_window["valid"], args.seed
                )
                # Keep the already measured query extraction for this variant.
                query_features[variant] = qfeatures
                del ref

            for window in windows:
                reference = window["reference"]
                present = reference.sample_id == query.sample_id
                for method in MATCH_METHODS:
                    if method == "xfeat_affine":
                        if reference.order not in reference_features:
                            reference_features[reference.order] = _extract_xfeat(model, window["image"])
                        point, stats, latency_ms = _cached_xfeat_fix(
                            model,
                            qfeatures,
                            reference_features[reference.order],
                            window["image"].shape,
                        )
                    else:
                        started = time.perf_counter()
                        point, stats = track_a.run_method(
                            method, query_image, window["image"], window["valid"]
                        )
                        latency_ms = (time.perf_counter() - started) * 1000.0
                    raw_rows.append(
                        _matcher_row(
                            query,
                            reference,
                            variant,
                            present,
                            prior_error,
                            qdata["rotation_noise_deg"],
                            qdata["height_error_fraction"],
                            coverages,
                            window,
                            method,
                            point,
                            stats,
                            latency_ms,
                        )
                    )
        print(f"{query.sample_id}: completed {len(windows)} map comparisons x 2 query variants", flush=True)

    raw = pd.DataFrame(raw_rows)
    # integrity.union_rule needs coordinate columns even when a method has no fix;
    # fixed=False masks these placeholders out of every candidate set.
    raw["est_x"] = raw["estimate_x"].fillna(0.0)
    raw["est_y"] = raw["estimate_y"].fillna(0.0)
    union = integrity.union_rule(raw, tol=UNION_TOL_M, features=("xfeat_affine",), name="UNION")
    union_rows = []
    for row in union.to_dict("records"):
        union_rows.append(
            {
                **row,
                "method": "UNION",
                "estimate_x": float(row["est_x"]),
                "estimate_y": float(row["est_y"]),
                "accepted": bool(row["accept"]),
                "fixed": bool(row["accept"]),
                "score": math.nan,
                "quad_n": math.nan,
                "inliers": math.nan,
                "inlier_ratio": math.nan,
                "gate": "",
            }
        )
    complete_rows = raw_rows + union_rows
    summary_rows = _summaries(complete_rows, query_count)

    out_dir = args.output
    out_dir.mkdir(parents=True, exist_ok=True)
    matches_path = out_dir / "matches.csv"
    frames_path = out_dir / "frames.csv"
    summary_path = out_dir / "summary.json"
    pd.DataFrame(complete_rows).to_csv(matches_path, index=False)
    pd.DataFrame(frame_rows).to_csv(frames_path, index=False)

    off_nadir = [sample.off_nadir_deg for sample in samples]
    gsd = [sample.map_gsd_m_px for sample in samples]
    reproj_median = [sample.projection_error_median_px for sample in samples]
    reproj_p95 = [sample.projection_error_p95_px for sample in samples]
    report = {
        "evidence": {
            "label": "MEASURED real query/map data; SIMULATED pose, height, and prior perturbations",
            "dataset": "OrthoLoC test_outPlace_L08_xDOP, released NPZ samples",
            "license": "CC BY-NC-SA 4.0 (dataset metadata recorded for this run)",
            "held_out_site_used_for_tuning": False,
            "parameters_fitted_on_held_out_site": [],
            "truth_use": "Only pose-noise query generation, prior-centered window selection, and scoring; never matcher input or acceptance decision.",
        },
        "protocol": {
            "seed": args.seed,
            "frames_available": len(samples),
            "queries_run": query_count,
            "reference_maps_run": reference_count,
            "positive_pairs_per_variant": query_count,
            "negative_maps_per_query": max(0, reference_count - 1),
            "negative_pairs_per_variant": query_count * max(0, reference_count - 1),
            "full_dataset_run": query_count == 60 and reference_count == 60,
            "negative_definition": "Each query is matched against every other NPZ DOP (all 59 in the full run); negative map window is centred on that DOP's paired camera position plus the query's same simulated prior error.",
            "coordinates": "DOP pixels are mapped to each NPZ's scene-local XYZ using its DSM X/Y channels; no global latitude/longitude is present.",
            "extrinsics": "Released [R|t] maps local-frame points to camera coordinates; camera centre is -R.T @ t. Verified by query point_map reprojection.",
            "plane": "Median valid DSM elevation per NPZ; DOP-grid X/Y affine fitted from finite DSM X/Y samples.",
            "rectification": "Real image warped to a 160x160 DOP-grid query using K and extrinsics with independent XYZ Euler perturbations N(0,1 deg) and camera-height perturbation N(0,3%). The local horizontal origin is the camera's vertical projection; absolute position is not written into the query.",
            "identity_variant": "No attitude/perspective correction: principal-point crop, scale-only resampling using the same noisy height and DOP GSD. It is the no-rectification control, not a literal same-size copy of the 1024x767 input.",
            "prior": "Independent uniform +/-40 m per local X/Y axis, sampled once per query and used for both variants and all its map comparisons.",
            "search_window": "Per-map square sized to contain the +/-40 m prior support plus the full 96 px ZNCC template; window is centred on prior and clipped to the DOP bounds. This changes only search extent, not matcher/acceptance settings.",
            "methods": {
                "matcher_source": "Unmodified experiments/r_map_benchmark.py run_method, zncc, quad_consensus, xfeat_points, geometric_fix.",
                "zncc": "Track-A default translation ZNCC.",
                "zncc_yaw_scale": "Track-A yaw hypotheses [-20,-10,0,10,20] deg and scales [0.9,1.0,1.1].",
                "xfeat_affine": "Track-A XFeat top_k=2048, affine RANSAC, 3 px reprojection threshold, 3000 iterations, 0.995 confidence, unchanged geometric gates; weights data/raw/models/accelerated_features.",
                "quad_acceptance": "Finite ZNCC fix and unchanged quad_consensus quad_n >= 3; four disjoint 56 px quadrants must agree within 4 px of the full fix.",
                "xfeat_acceptance": "Finite fix after unchanged Track-A geometric gates.",
                "union": "Unmodified r_integrity.union_rule, tol=10 m, XFeat feature set {xfeat_affine}; accepts consistent quad>=3 ZNCC candidates or ZNCC/XFeat agreements. No fitted score threshold.",
                "search_template_px": track_a.TEMPLATE,
                "query_px": track_a.QUERY,
                "xfeat_top_k": XFEAT_TOP_K,
                "xfeat_cache_parity": parity,
            },
            "false_accept_bounds": {
                "method": "One-sided exact Clopper-Pearson 95% upper bound (Beta quantile).",
                "reported_scopes": ["negative query-map pair", "any negative map per query cluster", "all false accepts per query cluster"],
                "caveat": "Pairwise trials share queries/maps; frame clustering is reported, but the exact binomial calculation does not model dependence between the 60 released frames.",
                "pointwise": "Bounds are pointwise per method and query variant; not adjusted for multiple comparisons.",
            },
            "latency": "Single-thread CPU wall-clock. ZNCC methods timed directly. XFeat uses exact cached feature extraction/MNN/RANSAC behavior parity-checked against Track-A; direct-equivalent pair time charges measured query and reference extraction on every pair, even when extraction was reused during this exhaustive evaluation. UNION sums all three method times.",
        },
        "geometry": {
            "off_nadir_angle_deg": _distribution(off_nadir),
            "camera_height_above_median_dsm_m": _distribution([sample.height_m for sample in samples]),
            "dop_gsd_m_per_pixel": _distribution(gsd),
            "point_map_reprojection_error_px_median_across_samples": _distribution(reproj_median),
            "point_map_reprojection_error_px_p95_across_samples": _distribution(reproj_p95),
            "sample_ids": [sample.sample_id for sample in samples],
        },
        "simulation_draws": {
            "rotation_noise_xyz_euler_std_deg_per_axis": ROTATION_NOISE_STD_DEG,
            "height_noise_std_fraction": HEIGHT_NOISE_STD_FRACTION,
            "prior_uniform_each_xy_axis_m": [-PRIOR_HALF_RANGE_M, PRIOR_HALF_RANGE_M],
            "observed_height_error_percent": _distribution([100.0 * row["height_error_fraction"] for row in frame_rows]),
        },
        "results": summary_rows,
        "outputs": [str(matches_path), str(frames_path), str(summary_path)],
        "source_sha256": {sample.sample_id: sample.sha256 for sample in samples},
        "runtime": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "opencv": cv2.__version__,
            "platform": platform.platform(),
            "model_weights": str(MODELS_DIR / "weights/xfeat.pt"),
        },
        "limits": [
            "Single held-out site/sample family: OrthoLoC L08 outPlace; no claim of cross-site transfer.",
            "60 released frames and their associated DOP/DSM only; this is not an independent flight-level population.",
            "Scene-local coordinates only; no global latitude/longitude or geodetic navigation error can be reported.",
            "Real imagery with simulated pose/barometer/prior noise; not an onboard closed-loop flight test.",
        ],
    }
    summary_path.write_text(json.dumps(_json_safe(report), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(pd.DataFrame(summary_rows).to_string(index=False), flush=True)
    print(f"Wrote {matches_path}, {frames_path}, and {summary_path}", flush=True)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=SEED)
    parser.add_argument("--output", type=Path, default=OUT_DIR)
    parser.add_argument("--max-queries", type=int, help="Debug only; default runs every query")
    parser.add_argument("--max-reference-maps", type=int, help="Debug only; default tests every DOP")
    args = parser.parse_args()
    if args.max_queries is not None and args.max_queries < 1:
        parser.error("--max-queries must be positive")
    if args.max_reference_maps is not None and args.max_reference_maps < 1:
        parser.error("--max-reference-maps must be positive")
    run(args)


if __name__ == "__main__":
    main()

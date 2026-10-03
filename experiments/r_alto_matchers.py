"""Compare map-matching methods on held-out ALTO validation frames.

Run from the repository root with ``.venv/bin/python experiments/r_alto_matchers.py``.
The script extracts only sampled query frames and the nearest main-reference tiles
under ``data/raw/alto/r_alto_matchers/``. Truth is used to generate the labelled
prior/negative windows and to score fixes; it is never passed to a matcher.
"""
from __future__ import annotations

import csv
import importlib
import json
import math
import multiprocessing as mp
import os
import platform
import sys
import time
import zipfile
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyproj import Transformer
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
ZIP_PATH = ROOT / "data/raw/alto/Val.zip"
RAW_CACHE = ROOT / "data/raw/alto/r_alto_matchers"
OUT_DIR = ROOT / "data/processed/r_alto_matchers"
N_SAMPLES = 300
MAP_MPP = 0.60
QUERY_ZOOM = 0.85  # ALTO scale learned before the GNSS cut by h_alto_end_to_end.py
QUERY_SIZE = 160
REFERENCE_SIZE = 384
WORKERS = 3
PRIOR_LIMIT_M = 60
NEGATIVE_DISTANCE_M = 600.0
HEADING_NOISE_STD_DEG = 3.0
REFERENCE_FOLDER = "offset_0_None"
METHODS = [
    "zncc",
    "zncc_yaw_scale",
    "xfeat_affine",
    "xfeat_homography",
    "zncc_heading",
    "xfeat_rot4",
]

sys.path.insert(0, str(ROOT / "experiments"))
import r_map_benchmark as B  # noqa: E402


def _member_cache_path(member: str) -> Path:
    parts = PurePosixPath(member).parts
    if not parts or parts[0] != "Val":
        raise ValueError(f"Unexpected ALTO member: {member}")
    return RAW_CACHE.joinpath(*parts)


def _extract_needed(archive: zipfile.ZipFile, members: set[str]) -> None:
    """Extract a bounded member set, retaining only complete cached members."""
    for member in sorted(members):
        target = _member_cache_path(member)
        info = archive.getinfo(member)
        if target.is_file() and target.stat().st_size == info.file_size:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        archive.extract(member, RAW_CACHE)


def _heading_degrees(query: pd.DataFrame) -> np.ndarray | None:
    cols = ["orient_x", "orient_y", "orient_z", "orient_w"]
    if not all(col in query.columns for col in cols):
        return None
    xy = query[["easting", "northing"]].to_numpy(dtype=float)
    to_lonlat = Transformer.from_crs("EPSG:32617", "EPSG:4326", always_xy=True)
    lon, lat = to_lonlat.transform(xy[:, 0], xy[:, 1])
    lat, lon = np.radians(lat), np.radians(lon)
    east = np.stack([-np.sin(lon), np.cos(lon), np.zeros_like(lon)], axis=1)
    north = np.stack([-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)], axis=1)
    rotation = Rotation.from_quat(query[cols].to_numpy(dtype=float))
    forward = rotation.apply([1.0, 0.0, 0.0])
    return np.degrees(np.arctan2(np.sum(forward * east, axis=1), np.sum(forward * north, axis=1)))


def _negative_index(travelled: np.ndarray, frame_index: int) -> int:
    """Choose a route location at least 600 m along-track from this frame."""
    distance = travelled[frame_index]
    if travelled[-1] < NEGATIVE_DISTANCE_M:
        raise ValueError("ALTO flight is shorter than the required negative offset")
    target_forward = distance + NEGATIVE_DISTANCE_M
    if target_forward <= travelled[-1]:
        return int(np.searchsorted(travelled, target_forward, side="left"))
    target_backward = distance - NEGATIVE_DISTANCE_M
    return int(np.searchsorted(travelled, target_backward, side="right") - 1)


def _nearest_reference(centres: np.ndarray, position: np.ndarray) -> int:
    return int(np.argmin(np.sum((centres - position) ** 2, axis=1)))


def _scene_metadata(query: pd.DataFrame, reference: pd.DataFrame) -> tuple[list[dict], np.ndarray | None]:
    truth = query[["easting", "northing"]].to_numpy(dtype=float)
    ref_centres = reference[["easting", "northing"]].to_numpy(dtype=float)
    travelled = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(truth, axis=0), axis=1))]
    headings = _heading_degrees(query)
    indices = np.linspace(0, len(query) - 1, N_SAMPLES).astype(np.int64)
    if len(np.unique(indices)) != N_SAMPLES:
        raise ValueError(f"Cannot select {N_SAMPLES} unique frames from {len(query)} records")

    scenes = []
    for order, frame_index in enumerate(indices):
        frame_index = int(frame_index)
        rng = np.random.default_rng(frame_index)
        offset_en = rng.integers(-PRIOR_LIMIT_M, PRIOR_LIMIT_M + 1, size=2)
        heading_noise = float(rng.normal(0.0, HEADING_NOISE_STD_DEG)) if headings is not None else 0.0
        prior = truth[frame_index] + offset_en
        neg_index = _negative_index(travelled, frame_index)
        negative_centre = truth[neg_index]
        query_name = str(query.iloc[frame_index]["name"])
        scenes.append({
            "sample_order": order,
            "frame_index": frame_index,
            "frame_name": query_name,
            "truth_easting_m": float(truth[frame_index, 0]),
            "truth_northing_m": float(truth[frame_index, 1]),
            "prior_easting_m": float(prior[0]),
            "prior_northing_m": float(prior[1]),
            "prior_offset_easting_m": int(offset_en[0]),
            "prior_offset_northing_m": int(offset_en[1]),
            "heading_deg": None if headings is None else float(headings[frame_index]),
            "heading_noise_deg": heading_noise,
            "derotation_deg": 0.0 if headings is None else float(90.0 - headings[frame_index] + heading_noise),
            "negative_frame_index": neg_index,
            "negative_along_flight_m": float(travelled[neg_index] - travelled[frame_index]),
            "positive_reference_index": _nearest_reference(ref_centres, prior),
            "negative_reference_index": _nearest_reference(ref_centres, negative_centre),
            "positive_centre_easting_m": float(prior[0]),
            "positive_centre_northing_m": float(prior[1]),
            "negative_centre_easting_m": float(negative_centre[0]),
            "negative_centre_northing_m": float(negative_centre[1]),
        })
    return scenes, headings


def _prepare_query(image_path: Path, derotation_deg: float, clahe: cv2.CLAHE) -> np.ndarray:
    gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(f"Could not decode query image: {image_path}")
    gray = clahe.apply(gray)
    width = int(round(gray.shape[1] * QUERY_ZOOM))
    height = int(round(gray.shape[0] * QUERY_ZOOM))
    scaled = cv2.resize(gray, (width, height), interpolation=cv2.INTER_AREA)
    matrix = cv2.getRotationMatrix2D((width / 2.0, height / 2.0), derotation_deg, 1.0)
    north_up = cv2.warpAffine(scaled, matrix, (width, height), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    top = (height - QUERY_SIZE) // 2
    left = (width - QUERY_SIZE) // 2
    query = north_up[top:top + QUERY_SIZE, left:left + QUERY_SIZE]
    if query.shape != (QUERY_SIZE, QUERY_SIZE) or query.dtype != np.uint8:
        raise ValueError(f"Prepared query has invalid format: {query.shape}, {query.dtype}")
    return np.ascontiguousarray(query)


def _prepare_reference_tile(path: Path, clahe: cv2.CLAHE) -> tuple[np.ndarray, np.ndarray]:
    raw = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if raw is None:
        raise FileNotFoundError(f"Could not decode reference image: {path}")
    if raw.shape != (500, 500):
        raise ValueError(f"Expected 500x500 ALTO reference tile, got {raw.shape}: {path}")
    valid = (raw != 0).astype(np.uint8)
    gray = clahe.apply(raw)
    gray[valid == 0] = 0
    return np.ascontiguousarray(gray), np.ascontiguousarray(valid)


def _reference_window(
    scene_centre: np.ndarray,
    ref_index: int,
    reference: pd.DataFrame,
    map_cache: dict[str, tuple[np.ndarray, np.ndarray]],
    clahe: cv2.CLAHE,
) -> tuple[np.ndarray, np.ndarray]:
    name = str(reference.iloc[ref_index]["name"])
    member = f"Val/reference_images/{name}"
    path = _member_cache_path(member)
    if name not in map_cache:
        map_cache[name] = _prepare_reference_tile(path, clahe)
    tile, tile_valid = map_cache[name]
    tile_centre = reference.iloc[ref_index][["easting", "northing"]].to_numpy(dtype=float)
    px = tile.shape[1] / 2.0 + (scene_centre[0] - tile_centre[0]) / MAP_MPP
    py = tile.shape[0] / 2.0 - (scene_centre[1] - tile_centre[1]) / MAP_MPP
    matrix = np.array([[1.0, 0.0, REFERENCE_SIZE / 2.0 - px],
                       [0.0, 1.0, REFERENCE_SIZE / 2.0 - py]], dtype=np.float32)
    window = cv2.warpAffine(tile, matrix, (REFERENCE_SIZE, REFERENCE_SIZE), flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    valid = cv2.warpAffine(tile_valid, matrix, (REFERENCE_SIZE, REFERENCE_SIZE), flags=cv2.INTER_NEAREST,
                           borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    if window.shape != (REFERENCE_SIZE, REFERENCE_SIZE) or valid.dtype != np.uint8:
        raise ValueError("ALTO reference window has invalid shape or mask")
    return np.ascontiguousarray(window), np.ascontiguousarray(valid)


def _optional_methods() -> tuple[list[str], str]:
    """Use the optional library only if its advertised available() API exists."""
    try:
        module = importlib.import_module("r_matchers")
        available = getattr(module, "available", None)
        if not callable(available):
            return [], "r_matchers.available() API absent"
        names = available()
        if isinstance(names, dict):
            names = names.keys()
        methods = [str(name) if str(name).startswith("lib:") else f"lib:{name}" for name in names]
        return methods, "r_matchers.available()"
    except Exception:
        return [], "optional matcher import/API unavailable"


def _worker_init() -> None:
    cv2.setNumThreads(1)


def _float_or_nan(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _run_method_set(payload: dict) -> list[dict]:
    query = payload["query"]
    truth = np.array([payload["truth_easting_m"], payload["truth_northing_m"]], dtype=float)
    results: list[dict] = []
    for case in payload["cases"]:
        reference = case["reference"]
        valid = case["valid"]
        centre = np.array([case["centre_easting_m"], case["centre_northing_m"]], dtype=float)
        case_results: list[dict] = []
        for method in payload["methods"]:
            started = time.perf_counter()
            point, stats = B.run_method(method, query, reference, valid)
            latency_ms = (time.perf_counter() - started) * 1000.0
            point = np.asarray(point, dtype=float).reshape(-1)
            finite = point.size >= 2 and bool(np.isfinite(point[:2]).all())
            if finite:
                estimate = np.array([
                    centre[0] + (point[0] - REFERENCE_SIZE / 2.0) * MAP_MPP,
                    centre[1] - (point[1] - REFERENCE_SIZE / 2.0) * MAP_MPP,
                ])
                error_m = float(np.linalg.norm(estimate - truth))
            else:
                estimate = np.array([math.nan, math.nan])
                error_m = math.nan
            score = _float_or_nan(stats.get("score", math.nan))
            quad = stats.get("quad_n")
            quad_n = math.nan if quad is None else _float_or_nan(quad)
            if method.startswith("zncc"):
                accepted = bool(finite and np.isfinite(quad_n) and quad_n >= 3)
            else:
                accepted = bool(finite)
            row = {
                "frame_index": payload["frame_index"],
                "sample_order": payload["sample_order"],
                "frame_name": payload["frame_name"],
                "case": case["case"],
                "reference_centre_easting_m": float(centre[0]),
                "reference_centre_northing_m": float(centre[1]),
                "truth_easting_m": float(truth[0]),
                "truth_northing_m": float(truth[1]),
                "positive_prior_offset_easting_m": payload["prior_offset_easting_m"],
                "positive_prior_offset_northing_m": payload["prior_offset_northing_m"],
                "negative_along_flight_m": payload["negative_along_flight_m"],
                "heading_deg": payload["heading_deg"],
                "heading_noise_deg": payload["heading_noise_deg"],
                "derotation_deg": payload["derotation_deg"],
                "method": method,
                "estimate_easting_m": float(estimate[0]),
                "estimate_northing_m": float(estimate[1]),
                "error_m": error_m,
                "accepted": accepted,
                "score": score,
                "quad_n": quad_n,
                "latency_ms": float(latency_ms),
            }
            case_results.append(row)
            results.append(row)

        zncc_rows = [r for r in case_results if r["method"].startswith("zncc") and
                     np.isfinite(r["estimate_easting_m"]) and np.isfinite(r["estimate_northing_m"])]
        xfeat_rows = [r for r in case_results if r["method"].startswith("xfeat") and
                      np.isfinite(r["estimate_easting_m"]) and np.isfinite(r["estimate_northing_m"])]
        candidates: dict[str, np.ndarray] = {}
        for row in zncc_rows:
            if row["accepted"]:
                candidates[row["method"]] = np.array([row["estimate_easting_m"], row["estimate_northing_m"]])
        for zrow in zncc_rows:
            zpoint = np.array([zrow["estimate_easting_m"], zrow["estimate_northing_m"]])
            for xrow in xfeat_rows:
                xpoint = np.array([xrow["estimate_easting_m"], xrow["estimate_northing_m"]])
                if np.linalg.norm(zpoint - xpoint) <= 10.0:
                    candidates[zrow["method"]] = zpoint
                    candidates[xrow["method"]] = xpoint
        union_accepted = False
        union_estimate = np.array([math.nan, math.nan])
        candidate_points = list(candidates.values())
        if candidate_points:
            points = np.stack(candidate_points)
            pairwise = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
            union_accepted = bool(pairwise.max() <= 10.0)
            union_estimate = points.mean(axis=0)
        union_error = float(np.linalg.norm(union_estimate - truth)) if np.isfinite(union_estimate).all() else math.nan
        results.append({
            "frame_index": payload["frame_index"],
            "sample_order": payload["sample_order"],
            "frame_name": payload["frame_name"],
            "case": case["case"],
            "reference_centre_easting_m": float(centre[0]),
            "reference_centre_northing_m": float(centre[1]),
            "truth_easting_m": float(truth[0]),
            "truth_northing_m": float(truth[1]),
            "positive_prior_offset_easting_m": payload["prior_offset_easting_m"],
            "positive_prior_offset_northing_m": payload["prior_offset_northing_m"],
            "negative_along_flight_m": payload["negative_along_flight_m"],
            "heading_deg": payload["heading_deg"],
            "heading_noise_deg": payload["heading_noise_deg"],
            "derotation_deg": payload["derotation_deg"],
            "method": "union",
            "estimate_easting_m": float(union_estimate[0]),
            "estimate_northing_m": float(union_estimate[1]),
            "error_m": union_error,
            "accepted": union_accepted,
            "score": math.nan,
            "quad_n": math.nan,
            "latency_ms": math.nan,
        })
    return results


def _payloads(scenes: list[dict], query: pd.DataFrame, reference: pd.DataFrame,
              methods: list[str]):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    map_cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for scene in scenes:
        query_path = _member_cache_path(f"Val/query_images/{scene['frame_name']}")
        query_image = _prepare_query(query_path, scene["derotation_deg"], clahe)
        positive_centre = np.array([scene["positive_centre_easting_m"], scene["positive_centre_northing_m"]])
        negative_centre = np.array([scene["negative_centre_easting_m"], scene["negative_centre_northing_m"]])
        pos_ref, pos_valid = _reference_window(positive_centre, scene["positive_reference_index"], reference, map_cache, clahe)
        neg_ref, neg_valid = _reference_window(negative_centre, scene["negative_reference_index"], reference, map_cache, clahe)
        yield {
            "sample_order": scene["sample_order"],
            "frame_index": scene["frame_index"],
            "frame_name": scene["frame_name"],
            "truth_easting_m": scene["truth_easting_m"],
            "truth_northing_m": scene["truth_northing_m"],
            "prior_offset_easting_m": scene["prior_offset_easting_m"],
            "prior_offset_northing_m": scene["prior_offset_northing_m"],
            "negative_along_flight_m": scene["negative_along_flight_m"],
            "heading_deg": scene["heading_deg"],
            "heading_noise_deg": scene["heading_noise_deg"],
            "derotation_deg": scene["derotation_deg"],
            "query": query_image,
            "methods": methods,
            "cases": [
                {"case": "positive", "centre_easting_m": scene["positive_centre_easting_m"],
                 "centre_northing_m": scene["positive_centre_northing_m"], "reference": pos_ref, "valid": pos_valid},
                {"case": "negative", "centre_easting_m": scene["negative_centre_easting_m"],
                 "centre_northing_m": scene["negative_centre_northing_m"], "reference": neg_ref, "valid": neg_valid},
            ],
        }


def _summary(rows: list[dict], methods: list[str]) -> list[dict]:
    summary = []
    for method in methods + ["union"]:
        method_rows = [row for row in rows if row["method"] == method]
        positives = [row for row in method_rows if row["case"] == "positive"]
        negatives = [row for row in method_rows if row["case"] == "negative"]
        n = len(positives)
        e = np.array([row["error_m"] for row in positives], dtype=float)
        accepted = np.array([row["accepted"] for row in positives], dtype=bool)
        negative_accepted = sum(bool(row["accepted"]) for row in negatives)
        latencies = np.array([row["latency_ms"] for row in method_rows], dtype=float)
        raw10 = int(np.sum(e <= 10.0))
        raw25 = int(np.sum(e <= 25.0))
        accepted10 = int(np.sum(accepted & (e <= 10.0)))
        accepted_wrong25 = int(np.sum(accepted & (e > 25.0)))
        summary.append({
            "method": method,
            "positives_n": n,
            "raw_success_10m_n": raw10,
            "raw_success_10m_rate": raw10 / n if n else math.nan,
            "raw_success_25m_n": raw25,
            "raw_success_25m_rate": raw25 / n if n else math.nan,
            "accepted_correct_10m_n": accepted10,
            "accepted_correct_10m_rate": accepted10 / n if n else math.nan,
            "accepted_wrong_over_25m_n": accepted_wrong25,
            "accepted_wrong_over_25m_rate": accepted_wrong25 / n if n else math.nan,
            "negatives_accepted_n": negative_accepted,
            "negatives_accepted_rate": negative_accepted / len(negatives) if negatives else math.nan,
            "latency_p50_ms": float(np.median(latencies)) if len(latencies) else math.nan,
        })
    return summary


def _write_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _examples(scenes: list[dict], query: pd.DataFrame, reference: pd.DataFrame,
              rows: list[dict], example_orders: list[int]) -> None:
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    map_cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    fig, axes = plt.subplots(len(example_orders), 2, figsize=(10, 4.2 * len(example_orders)))
    if len(example_orders) == 1:
        axes = np.asarray([axes])
    for axrow, order in zip(axes, example_orders):
        scene = scenes[order]
        q = _prepare_query(_member_cache_path(f"Val/query_images/{scene['frame_name']}"), scene["derotation_deg"], clahe)
        centre = np.array([scene["positive_centre_easting_m"], scene["positive_centre_northing_m"]])
        reference_image, _ = _reference_window(centre, scene["positive_reference_index"], reference, map_cache, clahe)
        query_ax, ref_ax = axrow
        query_ax.imshow(q, cmap="gray", vmin=0, vmax=255)
        query_ax.set_title(f"Query frame {scene['frame_index']} — 160×160 px")
        query_ax.axis("off")
        ref_ax.imshow(reference_image, cmap="gray", vmin=0, vmax=255)
        truth_px = np.array([REFERENCE_SIZE / 2 + (scene["truth_easting_m"] - centre[0]) / MAP_MPP,
                             REFERENCE_SIZE / 2 - (scene["truth_northing_m"] - centre[1]) / MAP_MPP])
        chosen = next((row for row in rows if row["sample_order"] == order and row["case"] == "positive" and
                       row["method"] == "zncc_yaw_scale"), None)
        if chosen is not None and np.isfinite(chosen["estimate_easting_m"]):
            estimate_px = np.array([REFERENCE_SIZE / 2 + (chosen["estimate_easting_m"] - centre[0]) / MAP_MPP,
                                    REFERENCE_SIZE / 2 - (chosen["estimate_northing_m"] - centre[1]) / MAP_MPP])
            ref_ax.scatter(estimate_px[0], estimate_px[1], marker="x", s=110, linewidths=2.5, color="red", label="zncc_yaw_scale estimate")
            ref_ax.legend(loc="lower right", fontsize=8)
        ref_ax.scatter(truth_px[0], truth_px[1], marker="o", s=95, facecolors="none", linewidths=2.0, edgecolors="lime", label="truth")
        if chosen is None or not np.isfinite(chosen["estimate_easting_m"]):
            ref_ax.legend(loc="lower right", fontsize=8)
        error_text = "no finite estimate" if chosen is None or not np.isfinite(chosen["error_m"]) else f"error {chosen['error_m']:.1f} m"
        ref_ax.set_title(f"384×384 px reference centred on prior — {error_text}\nGreen circle: truth; red ×: zncc_yaw_scale estimate")
        ref_ax.axis("off")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "examples.png", dpi=150)
    plt.close(fig)


def main() -> None:
    if not ZIP_PATH.is_file():
        raise FileNotFoundError(f"ALTO validation archive is missing: {ZIP_PATH}")
    cv2.setNumThreads(1)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    RAW_CACHE.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(ZIP_PATH) as archive:
        _extract_needed(archive, {"Val/query.csv", "Val/reference.csv"})
        query = pd.read_csv(_member_cache_path("Val/query.csv"))
        reference_all = pd.read_csv(_member_cache_path("Val/reference.csv"))
        reference = reference_all[reference_all.name.astype(str).str.startswith(REFERENCE_FOLDER)].reset_index(drop=True)
        if not len(reference):
            raise ValueError(f"No reference images under {REFERENCE_FOLDER}")
        scenes, headings = _scene_metadata(query, reference)
        members = {f"Val/query_images/{scene['frame_name']}" for scene in scenes}
        for scene in scenes:
            members.add(f"Val/reference_images/{reference.iloc[scene['positive_reference_index']]['name']}")
            members.add(f"Val/reference_images/{reference.iloc[scene['negative_reference_index']]['name']}")
        _extract_needed(archive, members)

    optional, optional_status = _optional_methods()
    methods = METHODS + optional
    print(f"ALTO Val: {len(query)} frames; selected {len(scenes)} evenly spaced; main map tiles {len(reference)}")
    print(f"Heading: {'quaternion available; per-frame de-rotation plus N(0, 3 deg) noise' if headings is not None else 'not available; using all six specified methods without de-rotation'}")
    print(f"Optional matchers: {len(optional)} ({optional_status}); workers: {min(WORKERS, os.cpu_count() or 1)}")

    rows: list[dict] = []
    workers = max(1, min(WORKERS, os.cpu_count() or 1))
    started = time.perf_counter()
    payload_iter = _payloads(scenes, query, reference, methods)
    context = mp.get_context("spawn")
    with context.Pool(processes=workers, initializer=_worker_init) as pool:
        for completed, result in enumerate(pool.imap_unordered(_run_method_set, payload_iter, chunksize=1), start=1):
            rows.extend(result)
            if completed % 25 == 0 or completed == len(scenes):
                print(f"Processed {completed}/{len(scenes)} query frames", flush=True)
    elapsed_s = time.perf_counter() - started
    rows.sort(key=lambda row: (row["sample_order"], row["case"], methods.index(row["method"]) if row["method"] in methods else len(methods)))

    frame_fields = [
        "frame_index", "sample_order", "frame_name", "case",
        "reference_centre_easting_m", "reference_centre_northing_m", "truth_easting_m", "truth_northing_m",
        "positive_prior_offset_easting_m", "positive_prior_offset_northing_m", "negative_along_flight_m",
        "heading_deg", "heading_noise_deg", "derotation_deg", "method",
        "estimate_easting_m", "estimate_northing_m", "error_m", "accepted", "score", "quad_n", "latency_ms",
    ]
    _write_csv(OUT_DIR / "frames.csv", rows, frame_fields)
    summary = _summary(rows, methods)
    _write_csv(OUT_DIR / "summary.csv", summary, list(summary[0].keys()))
    example_orders = np.linspace(0, len(scenes) - 1, 4).round().astype(int).tolist()
    _examples(scenes, query, reference, rows, example_orders)

    positive_offsets = np.array([[scene["prior_offset_easting_m"], scene["prior_offset_northing_m"]] for scene in scenes])
    negative_distances = np.array([abs(scene["negative_along_flight_m"]) for scene in scenes])
    run = {
        "experiment": "ALTO camera-to-reference matcher comparison",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "archive": str(ZIP_PATH.relative_to(ROOT)),
        "archive_listing": "/tmp/alto_Val_listing.txt",
        "archive_summary": {"listed_members": 3990, "query_frames": int(len(query)), "main_reference_tiles": int(len(reference)),
                            "main_reference_folder": REFERENCE_FOLDER, "query_csv": "Val/query.csv", "reference_csv": "Val/reference.csv"},
        "data_format": {
            "query_images": "500x500 RGB PNG; grayscale conversion, CLAHE, scale resize, heading de-rotation, central crop",
            "query_metadata": "UTM zone 17N EPSG:32617 easting/northing (m), WGS84 ellipsoid altitude, orient_x/y/z/w quaternion, name",
            "reference_images": "500x500 north-up grayscale orthophoto tiles; each centre at reference.csv easting/northing",
            "reference_resolution_m_per_px": MAP_MPP,
            "truth_to_reference_pixel": "tile/window x = 250 or 192 + (truth_easting - centre_easting)/0.60; y = 250 or 192 - (truth_northing - centre_northing)/0.60",
            "map_window": f"{REFERENCE_SIZE}x{REFERENCE_SIZE} px, centred at generated prior or negative route point; valid mask is raw map pixel != 0",
        },
        "labels": {"images_and_telemetry": "MEASURED", "reference_tiles_and_coordinates": "MEASURED",
                   "prior_offset": "SIMULATED uniform integer [-60,60] m per axis, rng seed=frame index",
                   "heading_noise": "SIMULATED N(0,3 deg), sampled after prior offset from same frame-index-seeded RNG" if headings is not None else "not applied; no heading data",
                   "negative": "SIMULATED evaluation case centred at least 600 m along-track from query truth"},
        "query_resampling": {"team_scale_logic": "ALTO zoom calibration learned before GNSS cut in h_alto_end_to_end.py",
                             "zoom": QUERY_ZOOM, "resized_dimension_px": int(round(500 * QUERY_ZOOM)),
                             "map_mpp": MAP_MPP, "output_query_shape_px": [QUERY_SIZE, QUERY_SIZE]},
        "heading": {"available": headings is not None,
                    "source": "query.csv orient_x/y/z/w; quaternion camera axes to ECEF; heading from local east/north projection" if headings is not None else "no orientation columns",
                    "derotation": "OpenCV rotation by 90 deg - heading + noise; top of image aligns north" if headings is not None else "none; all specified methods run"},
        "generator": {"selected_frames": int(len(scenes)), "sampling": "300 evenly spaced indices from first through last query frame",
                      "prior_offset_axis_min_max_m": [-PRIOR_LIMIT_M, PRIOR_LIMIT_M],
                      "prior_offset_axis_observed_min_max_m": [int(positive_offsets.min()), int(positive_offsets.max())],
                      "negative_along_track_abs_min_max_m": [float(negative_distances.min()), float(negative_distances.max())],
                      "truth_policy": "truth used for prior/negative window generation and scoring only; matcher receives only query, map window, and valid mask"},
        "methods": methods,
        "acceptance": {"zncc": "finite point and quad_n >= 3", "other_matchers": "finite point",
                       "union": "accept integrity-passing zncc fixes and each finite xfeat/zncc pair within 10 m; accept only if all candidate estimates are mutually within 10 m; estimate is their mean"},
        "optional_matchers": {"status": optional_status, "methods": optional},
        "execution": {"workers": workers, "opencv_threads_per_process": 1,
                      "latency_ms": "B.run_method call time; summary p50 includes positive and negative calls; union is a rule with no matcher call and its latency is not timed"},
        "elapsed_s": elapsed_s,
        "outputs": ["frames.csv", "summary.csv", "examples.png", "run.json"],
    }
    (OUT_DIR / "run.json").write_text(json.dumps(run, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Wrote {OUT_DIR.relative_to(ROOT)} ({len(rows)} result rows) in {elapsed_s:.1f} s")
    for row in summary:
        print(f"{row['method']:18s} raw<=10 {row['raw_success_10m_n']:3d}/{row['positives_n']} "
              f"raw<=25 {row['raw_success_25m_n']:3d}/{row['positives_n']} "
              f"accepted<=10 {row['accepted_correct_10m_n']:3d}/{row['positives_n']} "
              f"accepted-wrong>25 {row['accepted_wrong_over_25m_n']:3d} "
              f"negative accepted {row['negatives_accepted_n']:3d} "
              f"p50 {row['latency_p50_ms']:.2f} ms")


if __name__ == "__main__":
    main()

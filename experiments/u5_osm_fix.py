"""OSM-road oriented-chamfer localization against the Wufeng orthophotos.

This is a synthetic nadir-query experiment, not a real-flight validation.
Run from the repository root with .venv/bin/python.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import rasterio
import shapefile
from pyproj import Transformer
from rasterio.features import rasterize
from rasterio.transform import Affine
from shapely.geometry import box, shape as make_geometry
from shapely.ops import transform as transform_geometry

EXPERIMENTS = Path(__file__).resolve().parent
if str(EXPERIMENTS) not in sys.path:
    sys.path.insert(0, str(EXPERIMENTS))
import o_map_benchmark as benchmark  # reuse its unchanged data, split, render, and crop contract

QUERY = benchmark.QUERY
SEARCH = benchmark.SEARCH
BINS = 8
MAX_DISTANCE_M = 64.0
ANGLE_HYPOTHESES = (-20.0, -10.0, 0.0, 10.0, 20.0)
SCALE_HYPOTHESES = (0.9, 1.0, 1.1)
CONDITIONS = {
    "aligned": (0.0, 1.0, False),
    "degraded": (15.0, 1.07, True),
}
ROAD_WIDTH_M = {
    "motorway": 24.0,
    "trunk": 16.0,
    "primary": 12.0,
    "secondary": 9.0,
    "tertiary": 7.0,
    "unclassified": 6.0,
    "residential": 5.5,
    "living_street": 4.0,
    "service": 4.0,
    "road": 6.0,
    "track": 3.0,
    "pedestrian": 3.0,
    "cycleway": 2.0,
    "footway": 1.8,
    "path": 1.8,
    "steps": 1.5,
}


def road_width(fclass: str) -> float:
    """Approximate total paved width in metres; link classes inherit their parent."""
    cls = fclass.strip().lower()
    if cls.endswith("_link"):
        return max(4.0, ROAD_WIDTH_M.get(cls[:-5], 6.0) * 0.65)
    return ROAD_WIDTH_M.get(cls, 5.0)


def rasterize_osm_roads(roads_path: Path, shape_yx: tuple[int, int], affine: Affine):
    """Project Geofabrik WGS84 road lines to a 1 m EPSG:3826 road-mask raster."""
    height, width = shape_yx
    left, top = affine.c, affine.f
    right = left + affine.a * width
    bottom = top + affine.e * height
    projected_bounds = box(min(left, right), min(bottom, top), max(left, right), max(bottom, top))

    to_geo = Transformer.from_crs("EPSG:3826", "EPSG:4326", always_xy=True)
    to_projected = Transformer.from_crs("EPSG:4326", "EPSG:3826", always_xy=True)
    corners = [to_geo.transform(x, y) for x, y in ((left, bottom), (left, top), (right, bottom), (right, top))]
    lon_min = min(x for x, _ in corners)
    lon_max = max(x for x, _ in corners)
    lat_min = min(y for _, y in corners)
    lat_max = max(y for _, y in corners)

    reader = shapefile.Reader(str(roads_path), encoding="utf-8", encodingErrors="replace")
    projected_shapes = []
    classes: Counter[str] = Counter()
    for item in reader.iterShapeRecords(fields=["fclass"], bbox=(lon_min, lat_min, lon_max, lat_max)):
        fclass = str(item.record["fclass"])
        geom = make_geometry(item.shape.__geo_interface__)
        geom = transform_geometry(to_projected.transform, geom).intersection(projected_bounds)
        if geom.is_empty:
            continue
        pixel_geom = transform_geometry(lambda x, y, z=None: (x - left, top - y), geom)
        paved_area = pixel_geom.buffer(road_width(fclass) / 2, cap_style="flat", join_style="mitre", mitre_limit=2.0)
        if not paved_area.is_empty:
            projected_shapes.append((paved_area, 255))
            classes[fclass] += 1
    reader.close()

    road_mask = rasterize(
        projected_shapes,
        out_shape=(height, width),
        transform=Affine.identity(),
        fill=0,
        all_touched=False,
        dtype="uint8",
    )
    return road_mask, sum(classes.values()), dict(sorted(classes.items()))


def quantized_edge_masks(gray: np.ndarray) -> tuple[list[np.ndarray], int]:
    """Canny image edges grouped by unsigned gradient orientation for chamfer scoring."""
    softened = cv2.GaussianBlur(gray, (3, 3), 0)
    median = float(np.median(softened))
    low = max(20, int(0.66 * median))
    high = max(low + 1, min(240, int(1.33 * median)))
    edges = cv2.Canny(softened, low, high, L2gradient=True)

    # Suppress isolated texture speckles without discarding long road/building edges.
    count, labels, stats, _ = cv2.connectedComponentsWithStats((edges > 0).astype(np.uint8), 8)
    if count > 1:
        keep = np.zeros(count, dtype=np.uint8)
        keep[1:] = (stats[1:, cv2.CC_STAT_AREA] >= 6).astype(np.uint8)
        edges = (keep[labels] * 255).astype(np.uint8)

    gx = cv2.Sobel(softened, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(softened, cv2.CV_32F, 0, 1, ksize=3)
    orientation = np.mod(np.arctan2(gy, gx), np.pi)
    bins = np.minimum((orientation * (BINS / np.pi)).astype(np.int32), BINS - 1)
    edge_on = edges != 0
    masks = [((edge_on & (bins == index)).astype(np.uint8)) for index in range(BINS)]
    return masks, int(edge_on.sum())


def road_distance_fields(road_crop: np.ndarray) -> tuple[list[np.ndarray], int]:
    """Distance-to-road-boundary maps separated by unsigned edge orientation."""
    boundaries = cv2.Canny(road_crop, 50, 100, L2gradient=True)
    gx = cv2.Sobel(road_crop, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(road_crop, cv2.CV_32F, 0, 1, ksize=3)
    orientation = np.mod(np.arctan2(gy, gx), np.pi)
    bins = np.minimum((orientation * (BINS / np.pi)).astype(np.int32), BINS - 1)
    boundary_on = boundaries != 0
    fields: list[np.ndarray] = []
    for index in range(BINS):
        selected = (boundary_on & (bins == index)).astype(np.uint8)
        if selected.any():
            distances = cv2.distanceTransform(255 - selected * 255, cv2.DIST_L2, 3)
            fields.append(np.minimum(distances, MAX_DISTANCE_M).astype(np.float32))
        else:
            fields.append(np.full(road_crop.shape, MAX_DISTANCE_M, dtype=np.float32))
    return fields, int(boundary_on.sum())


def oriented_chamfer_surface(query: np.ndarray, fields: list[np.ndarray], map_edge_count: int):
    """Return the best chamfer cost at every candidate translation and its global minimum."""
    output_shape = (fields[0].shape[0] - QUERY + 1, fields[0].shape[1] - QUERY + 1)
    surface = np.full(output_shape, np.inf, dtype=np.float32)
    best_cost = float("inf")
    best_location = None
    best_angle = np.nan
    best_scale = np.nan
    best_edge_count = 0
    if map_edge_count == 0:
        surface.fill(MAX_DISTANCE_M)
        return surface, best_cost, best_location, best_angle, best_scale, best_edge_count

    for angle in ANGLE_HYPOTHESES:
        for scale in SCALE_HYPOTHESES:
            matrix = cv2.getRotationMatrix2D((QUERY / 2, QUERY / 2), angle, scale)
            warped = cv2.warpAffine(query, matrix, (QUERY, QUERY), flags=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            masks, edge_count = quantized_edge_masks(warped)
            if edge_count < 80:
                continue
            score_map = np.zeros(output_shape, dtype=np.float32)
            for index, mask in enumerate(masks):
                if not mask.any():
                    continue
                # Adjacent bins allow small gradient/rasterization orientation error.
                distance = np.minimum(fields[(index - 1) % BINS], fields[index])
                distance = np.minimum(distance, fields[(index + 1) % BINS])
                score_map += cv2.matchTemplate(distance, mask.astype(np.float32), cv2.TM_CCORR)
            score_map /= float(edge_count)
            np.minimum(surface, score_map, out=surface)
            cost, _, min_location, _ = cv2.minMaxLoc(score_map)
            if cost < best_cost:
                best_cost = float(cost)
                best_location = min_location
                best_angle, best_scale, best_edge_count = angle, scale, edge_count
    return surface, best_cost, best_location, best_angle, best_scale, best_edge_count


def oriented_chamfer_match(query: np.ndarray, fields: list[np.ndarray], map_edge_count: int):
    """Return the best location, robust per-scene peak z-score, and chamfer cost."""
    surface, best_cost, location, angle, scale, edge_count = oriented_chamfer_surface(
        query, fields, map_edge_count)
    if not np.isfinite(best_cost) or location is None:
        return np.array([np.nan, np.nan]), -MAX_DISTANCE_M, MAX_DISTANCE_M, np.nan, np.nan, 0
    peak_score, _, _ = peak_zscore(surface)
    point = np.array((location[0] + QUERY / 2, location[1] + QUERY / 2), dtype=float)
    return point, peak_score, best_cost, angle, scale, edge_count


def make_samples(groups, rng):
    """Use o_map_benchmark's exact center groups and its prior/negative construction."""
    samples = []
    for split, centres in groups.items():
        for index, centre_tuple in enumerate(centres):
            centre = np.array(centre_tuple, dtype=float)
            prior = centre + rng.integers(-60, 61, 2)
            alternatives = [p for p in centres if np.linalg.norm(np.array(p) - prior) > SEARCH]
            if not alternatives:
                raise ValueError(f"No spatially distinct negative exists for {split} centre {index}")
            negative = np.array(alternatives[rng.integers(len(alternatives))], dtype=float)
            samples.append({
                "split": split,
                "centre_id": index,
                "query_id": f"{split}_{index:02d}",
                "centre": centre,
                "prior": prior.astype(float),
                "negative": negative,
            })
    return samples


def build_threshold(rows: pd.DataFrame, method: str) -> float:
    calibration = rows[(rows["split"] == "calibration") & (rows["method"] == method)]
    false_or_bad = (~calibration["present"]) | (calibration["error_m"] > 25.0) | calibration["error_m"].isna()
    false_scores = calibration.loc[false_or_bad, "score"].astype(float)
    if false_scores.empty:
        raise ValueError(f"No calibration false score available for {method}")
    return float(np.nextafter(false_scores.max(), np.inf))


def add_agreement_gate(rows: pd.DataFrame, thresholds: dict[str, float]) -> pd.DataFrame:
    by_query: dict[str, dict[str, dict]] = {}
    for item in rows.to_dict("records"):
        by_query.setdefault(item["query_id"], {}).setdefault(item["condition"], {})[item["method"]] = item
    gates = []
    for query_id, conditions in by_query.items():
        for condition, methods in conditions.items():
            osm = methods["osm"]
            xfeat = methods["xfeat"]
            both_have_position = np.isfinite(osm["estimate_x"]) and np.isfinite(xfeat["estimate_x"])
            agreement = (float(np.hypot(osm["estimate_x"] - xfeat["estimate_x"],
                                        osm["estimate_y"] - xfeat["estimate_y"]))
                         if both_have_position else None)
            accepted = bool(osm["accepted"] and xfeat["accepted"] and agreement is not None and agreement <= 10.0)
            gates.append({
                **{key: xfeat[key] for key in ("query_id", "split", "centre_id", "condition", "present",
                                                "truth_x", "truth_y", "prior_x", "prior_y")},
                "method": "osm_and_xfeat_agreement",
                "score": float(min(osm["score"], xfeat["score"])),
                "estimate_x": xfeat["estimate_x"],
                "estimate_y": xfeat["estimate_y"],
                "error_m": xfeat["error_m"],
                "accepted": accepted,
                "latency_ms": osm["latency_ms"] + xfeat["latency_ms"],
                "osm_xfeat_agreement_m": agreement,
                "osm_threshold": thresholds["osm"],
                "xfeat_threshold": thresholds["xfeat"],
            })
    return pd.concat([rows, pd.DataFrame(gates)], ignore_index=True)


def summarize(rows: pd.DataFrame, thresholds: dict[str, float], count: int) -> dict:
    results = []
    condition_map = {"aligned": "aligned", "degraded": "degraded"}
    for method in ("osm", "xfeat", "osm_and_xfeat_agreement"):
        for condition, label in condition_map.items():
            test = rows[(rows["split"] == "test") & (rows["method"] == method) & (rows["condition"] == condition)]
            pos = test[test["present"]]
            neg = test[~test["present"]]
            accepted = pos[pos["accepted"]]
            results.append({
                "method": method,
                "condition": label,
                "positive_n": len(pos),
                "negative_n": len(neg),
                "accepted_positive": int(len(accepted)),
                "correct_accepted_10m": int((accepted["error_m"] <= 10.0).sum()),
                "false_accepts_negative": int(neg["accepted"].sum()),
                "accepted_wrong_over_25m": int((accepted["error_m"] > 25.0).sum()),
                "raw_correct_10m": int((pos["error_m"] <= 10.0).sum()),
                "median_error_m": float(pos["error_m"].median()) if pos["error_m"].notna().any() else None,
                "latency_p50_ms": float(test["latency_ms"].median()) if len(test) else None,
                "latency_p95_ms": float(test["latency_ms"].quantile(0.95)) if len(test) else None,
            })

    # Easting/northing residual on accepted, reasonably localized aligned OSM fixes.
    aligned = rows[(rows["split"] == "test") & (rows["method"] == "osm") &
                   (rows["condition"] == "aligned") & rows["present"] & rows["accepted"] &
                   (rows["error_m"] <= 25.0)]
    offsets = []
    if len(aligned):
        dx_east = aligned["estimate_x"].to_numpy(float) - aligned["truth_x"].to_numpy(float)
        dy_north = -(aligned["estimate_y"].to_numpy(float) - aligned["truth_y"].to_numpy(float))
        median_east = float(np.median(dx_east))
        median_north = float(np.median(dy_north))
        offsets = {"n": int(len(aligned)), "median_east_m": median_east,
                   "median_north_m": median_north,
                   "median_vector_magnitude": float(np.hypot(median_east, median_north)),
                   "interpretation": "apparent OSM-to-orthophoto translation; also includes road-width, image-edge, and matcher bias"}
    else:
        offsets = {"n": 0, "median_east_m": None, "median_north_m": None,
                   "median_vector_magnitude": None,
                   "interpretation": "not estimable: no accepted aligned OSM positive within 25 m"}

    calibration_false_accepts = {}
    for method in ("osm", "xfeat", "osm_and_xfeat_agreement"):
        cal = rows[(rows["split"] == "calibration") & (rows["method"] == method)]
        bad = (~cal["present"]) | (cal["error_m"] > 25.0) | cal["error_m"].isna()
        calibration_false_accepts[method] = int(cal.loc[bad, "accepted"].sum())
    return {
        "count_per_split": count,
        "thresholds_strictly_above_calibration_false_scores": thresholds,
        "calibration_false_accepts": calibration_false_accepts,
        "test_results": results,
        "apparent_osm_orthophoto_offset": offsets,
    }


def print_summary(summary: dict) -> None:
    print("method                         condition  accepted/30  correct<=10m  false_accepts/30  raw_correct/30")
    for row in summary["test_results"]:
        print(f"{row['method']:<30} {row['condition']:<10} "
              f"{row['accepted_positive']:>2}/{row['positive_n']:<2}          "
              f"{row['correct_accepted_10m']:>2}            "
              f"{row['false_accepts_negative']:>2}/{row['negative_n']:<2}               "
              f"{row['raw_correct_10m']:>2}/{row['positive_n']}")
    offset = summary["apparent_osm_orthophoto_offset"]
    print("OSM/orthophoto offset (accepted aligned OSM fixes <=25 m): "
          f"n={offset['n']}, east={offset['median_east_m']}, north={offset['median_north_m']}, "
          f"vector={offset['median_vector_magnitude']} m")
    print("Calibration false accepts:", summary["calibration_false_accepts"])


def peak_zscore(surface: np.ndarray) -> tuple[float, float, float]:
    finite = surface[np.isfinite(surface)]
    if not finite.size:
        return float("-inf"), float("nan"), float("nan")
    median = float(np.median(finite))
    mad = float(np.median(np.abs(finite - median)))
    scale = 1.4826 * mad
    if scale <= 1e-6:
        scale = float(np.std(finite))
    if scale <= 1e-6:
        return 0.0, median, scale
    return float((median - float(finite.min())) / scale), median, scale


def image_road_likelihood(gray: np.ndarray) -> np.ndarray:
    """Soft road-surface cue: moderate luminance and low 11x11 local texture."""
    values = gray.astype(np.float32)
    mean = cv2.blur(values, (11, 11))
    mean_square = cv2.blur(values * values, (11, 11))
    texture = np.sqrt(np.maximum(mean_square - mean * mean, 0.0))
    luminance = np.exp(-0.5 * ((mean - 120.0) / 70.0) ** 2)
    smoothness = np.exp(-texture / 20.0)
    return (luminance * smoothness).astype(np.float32)


def road_mask_cross_correlation(roads: np.ndarray, likelihood: np.ndarray, centre: np.ndarray) -> dict:
    road_window = benchmark.crop(roads, centre, SEARCH)
    image_window = benchmark.crop(likelihood, centre, SEARCH)
    template = benchmark.crop(road_window, (SEARCH / 2, SEARCH / 2), benchmark.SOURCE).astype(np.float32) / 255.0
    surface = cv2.matchTemplate(image_window, template, cv2.TM_CCORR_NORMED)
    expected_x = (SEARCH - benchmark.SOURCE) // 2
    expected_y = expected_x
    low_x, high_x = max(0, expected_x - 60), min(surface.shape[1] - 1, expected_x + 60)
    low_y, high_y = max(0, expected_y - 60), min(surface.shape[0] - 1, expected_y + 60)
    restricted = np.zeros_like(surface)
    restricted[low_y:high_y + 1, low_x:high_x + 1] = surface[low_y:high_y + 1, low_x:high_x + 1]
    _, best_score, _, best_location = cv2.minMaxLoc(restricted)
    zero_score = float(surface[expected_y, expected_x])
    dx_east = int(best_location[0] - expected_x)
    dy_north = -int(best_location[1] - expected_y)
    return {
        "best_score": float(best_score),
        "score_at_zero_offset": zero_score,
        "offset_east_m": dx_east,
        "offset_north_m": dy_north,
        "offset_magnitude_m": float(np.hypot(dx_east, dy_north)),
        "search_limit_m": 60,
    }


def write_diagnostic_overlay(output: Path, maps: list[np.ndarray], roads: np.ndarray, test_samples: list[dict]):
    tile_h = benchmark.SOURCE + 30
    canvas = np.full((len(test_samples) * tile_h, 2 * benchmark.SOURCE, 3), 255, dtype=np.uint8)
    for row, sample in enumerate(test_samples):
        centre = sample["centre"]
        for column, (image, date) in enumerate(zip(maps, ("2018", "2020"))):
            gray = benchmark.crop(image, centre, benchmark.SOURCE)
            road_mask = benchmark.crop(roads, centre, benchmark.SOURCE) != 0
            overlay = np.repeat(gray[:, :, None], 3, axis=2)
            red = np.array((0.0, 0.0, 255.0), dtype=np.float32)
            overlay[road_mask] = (overlay[road_mask].astype(np.float32) * 0.4 + red * 0.6).astype(np.uint8)
            x0 = column * benchmark.SOURCE
            y0 = row * tile_h
            canvas[y0 + 30:y0 + tile_h, x0:x0 + benchmark.SOURCE] = overlay
            cv2.putText(canvas, f"test {sample['centre_id']} - {date}", (x0 + 5, y0 + 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    path = output / "diag_overlay.png"
    if not cv2.imwrite(str(path), canvas):
        raise OSError(f"Failed to write diagnostic overlay: {path}")
    return path


def run_diagnostics(output: Path, maps: list[np.ndarray], affine: Affine, image_paths: list[Path],
                    samples: list[dict], roads: np.ndarray, raster_path: Path,
                    road_feature_count: int, road_classes: dict[str, int]) -> dict:
    started = time.perf_counter()
    test_samples = [sample for sample in samples if sample["split"] == "test"][:3]
    overlays = write_diagnostic_overlay(output, maps, roads, test_samples)
    likelihoods = [image_road_likelihood(image) for image in maps]
    correlations = []
    for sample in test_samples:
        for image, date, likelihood in zip(maps, ("2018", "2020"), likelihoods):
            correlations.append({
                "centre_id": sample["centre_id"],
                "date": date,
                "centre_pixel_x": int(sample["centre"][0]),
                "centre_pixel_y_row_down": int(sample["centre"][1]),
                **road_mask_cross_correlation(roads, likelihood, sample["centre"]),
            })

    offset_rows = []
    normalized_rows = []
    for sample in samples:
        prior = sample["prior"]
        fields, map_edge_count = road_distance_fields(benchmark.crop(roads, prior, SEARCH))
        for present, source in ((True, sample["centre"]), (False, sample["negative"])):
            query = benchmark.render(maps[1], source, 0.0, 1.0, False)
            surface, best_cost, location, angle, scale, edge_count = oriented_chamfer_surface(
                query, fields, map_edge_count)
            if location is None or not np.isfinite(best_cost):
                estimate = np.array((np.nan, np.nan))
                error_m = float("nan")
                best_dx = best_dy = None
                true_cost = float("nan")
                local_minimum = False
                true_is_global_best = False
            else:
                best_col, best_row = location
                point = np.array((best_col + QUERY / 2, best_row + QUERY / 2), dtype=float)
                estimate = point + prior - SEARCH / 2
                error_m = float(np.linalg.norm(estimate - source))
                true_point = np.asarray(source, dtype=float) - prior + SEARCH / 2
                true_col = int(round(true_point[0] - QUERY / 2))
                true_row = int(round(true_point[1] - QUERY / 2))
                if 0 <= true_col < surface.shape[1] and 0 <= true_row < surface.shape[0]:
                    true_cost = float(surface[true_row, true_col])
                    neighborhood = surface[max(0, true_row - 1):true_row + 2,
                                            max(0, true_col - 1):true_col + 2]
                    local_minimum = bool(true_cost <= float(np.min(neighborhood)) + 1e-5)
                    true_is_global_best = bool(true_row == best_row and true_col == best_col)
                else:
                    true_cost = float("nan")
                    local_minimum = False
                    true_is_global_best = False
                best_dx = float(estimate[0] - source[0])
                best_dy = float(-(estimate[1] - source[1]))

            z_score, surface_median, surface_scale = peak_zscore(surface)
            normalized_rows.append({
                "query_id": sample["query_id"],
                "split": sample["split"],
                "centre_id": sample["centre_id"],
                "present": present,
                "best_score_raw": -best_cost if np.isfinite(best_cost) else -MAX_DISTANCE_M,
                "best_chamfer_m": best_cost if np.isfinite(best_cost) else MAX_DISTANCE_M,
                "score_z": z_score,
                "surface_median_chamfer_m": surface_median,
                "surface_robust_scale_m": surface_scale,
                "error_m": error_m,
                "estimate_x": float(estimate[0]),
                "estimate_y": float(estimate[1]),
            })
            if sample["split"] == "test" and present:
                offset_rows.append({
                    "centre_id": sample["centre_id"],
                    "truth_x": int(source[0]),
                    "truth_y_row_down": int(source[1]),
                    "prior_x": int(prior[0]),
                    "prior_y_row_down": int(prior[1]),
                    "best_error_m": error_m,
                    "truth_chamfer_m": true_cost,
                    "best_chamfer_m": best_cost if np.isfinite(best_cost) else MAX_DISTANCE_M,
                    "truth_minus_best_chamfer_m": true_cost - best_cost if np.isfinite(true_cost) and np.isfinite(best_cost) else None,
                    "best_offset_east_m": best_dx,
                    "best_offset_north_m": best_dy,
                    "truth_is_local_minimum": local_minimum,
                    "truth_is_global_best": true_is_global_best,
                    "best_yaw_correction_deg": angle,
                    "best_scale": scale,
                    "camera_edge_count": edge_count,
                    "map_edge_count": map_edge_count,
                })

    frame = pd.DataFrame(normalized_rows)
    calibration = frame[frame["split"] == "calibration"]
    bad = (~calibration["present"]) | (calibration["error_m"] > 25.0) | calibration["error_m"].isna()
    raw_threshold = float(np.nextafter(calibration.loc[bad, "best_score_raw"].max(), np.inf))
    z_threshold = float(np.nextafter(calibration.loc[bad, "score_z"].max(), np.inf))
    negative_only = calibration[~calibration["present"]]
    pilot_threshold = float(np.nextafter(negative_only["best_score_raw"].max(), np.inf))
    test = frame[frame["split"] == "test"]
    test_summary = {}
    for name, threshold, column in (
        ("raw_all_calibration_false_scores", raw_threshold, "best_score_raw"),
        ("per_scene_zscore", z_threshold, "score_z"),
    ):
        accepted = test[column] >= threshold
        pos = test[test["present"]]
        neg = test[~test["present"]]
        pos_accepted = pos[column] >= threshold
        neg_accepted = neg[column] >= threshold
        test_summary[name] = {
            "threshold_strictly_above_calibration_false_scores": threshold,
            "accepted_positive": int(pos_accepted.sum()),
            "correct_accepted_10m": int(((pos["error_m"] <= 10.0) & pos_accepted).sum()),
            "false_accepts_negative": int(neg_accepted.sum()),
            "raw_correct_10m": int((pos["error_m"] <= 10.0).sum()),
            "calibration_false_accepts": int((calibration.loc[bad, column] >= threshold).sum()),
        }
    for item in offset_rows:
        item["raw_pilot_accepts_negative_only_threshold"] = bool(
            -item["best_chamfer_m"] >= pilot_threshold)
        item["normalized_accepts"] = bool(
            frame.loc[(frame["split"] == "test") & (frame["centre_id"] == item["centre_id"]) &
                      frame["present"], "score_z"].iloc[0] >= z_threshold)

    with rasterio.open(raster_path) as dataset:
        raster_metadata = {
            "crs": dataset.crs.to_string(),
            "epsg": dataset.crs.to_epsg(),
            "width": dataset.width,
            "height": dataset.height,
            "transform_gdal": list(dataset.transform.to_gdal()),
            "matches_load_maps_transform": dataset.transform == affine,
        }
    col_step = affine.a
    row_step_north = affine.e
    geometry_check = {
        "load_maps_transform_gdal": list(affine.to_gdal()),
        "orthophotos_share_load_maps_transform": True,
        "osm_raster_transform_exact_match": raster_metadata["matches_load_maps_transform"],
        "osm_raster_crs": raster_metadata["crs"],
        "east_increment_for_column_plus_one_m": col_step,
        "north_increment_for_row_plus_one_m": row_step_north,
        "row_axis": "row increases south/down; EPSG:3826 northing decreases by 1 m per row",
        "raster_dimensions": [raster_metadata["height"], raster_metadata["width"]],
        "raster_road_pixels": int(np.count_nonzero(roads)),
        "raster_road_fraction": float(np.count_nonzero(roads) / roads.size),
        "road_features": road_feature_count,
        "road_classes": road_classes,
        "rasterization": "buffered projected road lines in pixel coordinates (col=easting-left, row=top-northing); 1 m pixels",
        "assumed_total_road_width_m": ROAD_WIDTH_M,
    }
    positive_df = pd.DataFrame(offset_rows)
    truth_audit = {
        "positive_test_n": int(len(positive_df)),
        "truth_is_local_minimum_n": int(positive_df["truth_is_local_minimum"].sum()),
        "truth_is_global_best_n": int(positive_df["truth_is_global_best"].sum()),
        "median_truth_chamfer_m": float(positive_df["truth_chamfer_m"].median()),
        "median_best_chamfer_m": float(positive_df["best_chamfer_m"].median()),
        "median_truth_minus_best_chamfer_m": float(positive_df["truth_minus_best_chamfer_m"].median()),
        "per_positive": offset_rows,
    }
    result = {
        "purpose": "diagnostic only; truth is used to score/review matches, never as estimator input",
        "cross_correlation_likelihood": {
            "definition": "exp(-0.5*((11x11 mean gray-120)/70)^2) * exp(-11x11 gray std/20); OSM 240m road-mask patch vs image likelihood, ±60m search",
            "per_centre_and_date": correlations,
        },
        "chamfer_truth_audit": truth_audit,
        "threshold_recalibration": {
            "raw_pilot_threshold_negative_calibration_only": pilot_threshold,
            "raw_threshold_all_calibration_false_scores": raw_threshold,
            "normalized_threshold_all_calibration_false_scores": z_threshold,
            "normalization": "z=(median(candidate chamfer surface)-best chamfer)/robust scale; robust scale=1.4826*MAD, std fallback",
            "test_results": test_summary,
        },
        "geometry_and_rasterization": geometry_check,
        "overlay_png": str(overlays),
        "orthophotos": [str(path) for path in image_paths],
        "elapsed_s": time.perf_counter() - started,
    }
    path = output / "diag.json"
    path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({
        "overlay": str(overlays),
        "cross_correlation": correlations,
        "truth_audit": {key: value for key, value in truth_audit.items() if key != "per_positive"},
        "threshold_recalibration": test_summary,
        "geometry": geometry_check,
        "diag_json": str(path),
    }, indent=2, allow_nan=False), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("data/raw/aerial"))
    parser.add_argument("--osm-roads", type=Path, default=Path("data/raw/osm/gis_osm_roads_free_1.shp"))
    parser.add_argument("--xfeat-root", type=Path, default=Path("data/raw/models/accelerated_features"))
    parser.add_argument("--output", type=Path, default=Path("data/processed/u5_osm_fix"))
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument("--kill-test", action="store_true", help="OSM-only aligned kill test before full experiment")
    parser.add_argument("--diagnostic", action="store_true", help="write OSM/orthophoto alignment and score-normalization diagnostics")
    args = parser.parse_args()
    if args.kill_test and args.diagnostic:
        parser.error("--kill-test and --diagnostic are separate runs")
    if args.count < 1:
        parser.error("--count must be positive")
    if not args.osm_roads.is_file():
        parser.error(f"OSM roads shapefile not found: {args.osm_roads}")
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    cv2.setRNGSeed(args.seed)
    rng = np.random.default_rng(args.seed)

    maps, masks, affine, image_paths = benchmark.load_maps(args.data)
    groups = benchmark.select_centres(masks, args.count, rng)
    samples = make_samples(groups, rng)
    print(f"centres calibration={len(groups['calibration'])}, test={len(groups['test'])}; "
          f"camera=2020, map=Geofabrik roads; area={maps[1].shape} px at 1 m/px", flush=True)

    start = time.perf_counter()
    roads, road_feature_count, road_classes = rasterize_osm_roads(args.osm_roads, maps[1].shape, affine)
    raster_path = args.output / "osm_roads_1m.tif"
    with rasterio.open(raster_path, "w", driver="GTiff", height=roads.shape[0], width=roads.shape[1],
                       count=1, dtype="uint8", crs="EPSG:3826", transform=affine,
                       compress="deflate", nodata=0) as dataset:
        dataset.write(roads, 1)
        dataset.update_tags(source="Geofabrik Taiwan shapefile extract (OpenStreetMap)",
                            license="ODbL-1.0", resolution="1 m/pixel",
                            road_widths_m=json.dumps(ROAD_WIDTH_M, sort_keys=True))
    cv2.imwrite(str(args.output / "osm_roads_preview.png"), roads)
    raster_seconds = time.perf_counter() - start
    print(f"OSM raster: {road_feature_count} road features, {int(np.count_nonzero(roads))} road pixels; "
          f"rasterization {raster_seconds:.1f} s", flush=True)
    if args.diagnostic:
        run_diagnostics(args.output, maps, affine, image_paths, samples, roads, raster_path,
                        road_feature_count, road_classes)
        return

    cache = {}
    for sample in samples:
        prior = sample["prior"]
        road_crop = benchmark.crop(roads, prior, SEARCH)
        fields, map_edge_count = road_distance_fields(road_crop)
        reference = benchmark.crop(maps[0], prior, SEARCH)
        cache[sample["query_id"]] = (fields, map_edge_count, reference)

    # Deterministic positives and the spatially separate negatives used by the source benchmark.
    query_tasks = []
    for sample in samples:
        for condition, (angle, scale, degraded) in CONDITIONS.items():
            for present, source in ((True, sample["centre"]), (False, sample["negative"])):
                query = benchmark.render(maps[1], source, angle, scale, degraded)
                query_tasks.append((sample, condition, present, source, query))

    rows = []
    model = None
    if not args.kill_test:
        model = benchmark.load_xfeat(args.xfeat_root)

    if args.kill_test:
        # Calibrate on all calibration aligned positives/negatives; evaluate all 30 aligned test pairs.
        selected_tasks = [task for task in query_tasks if task[1] == "aligned"]
        methods = ("osm",)
    else:
        selected_tasks = query_tasks
        methods = ("osm", "xfeat")

    for sample, condition, present, source, query in selected_tasks:
        fields, map_edge_count, ortho_reference = cache[sample["query_id"]]
        truth = np.asarray(source, dtype=float)
        prior = sample["prior"]
        for method in methods:
            started = time.perf_counter()
            if method == "osm":
                point, score, chamfer_m, angle, scale, edge_count = oriented_chamfer_match(query, fields, map_edge_count)
            else:
                point, score = benchmark.learned_match(model, query, ortho_reference)
                chamfer_m, angle, scale, edge_count = None, None, None, None
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            estimate = point + prior - SEARCH / 2 if np.isfinite(point).all() else np.array([np.nan, np.nan])
            error = float(np.linalg.norm(estimate - truth)) if np.isfinite(estimate).all() else np.nan
            rows.append({
                "query_id": sample["query_id"], "split": sample["split"], "centre_id": sample["centre_id"],
                "condition": condition, "present": present, "method": method,
                "score": float(score), "error_m": error, "estimate_x": float(estimate[0]), "estimate_y": float(estimate[1]),
                "truth_x": float(truth[0]), "truth_y": float(truth[1]),
                "prior_x": float(prior[0]), "prior_y": float(prior[1]),
                "accepted": False, "latency_ms": elapsed_ms, "chamfer_m": chamfer_m,
                "searched_angle_deg": angle, "searched_scale": scale,
                "query_edge_count": edge_count, "map_road_edge_count": map_edge_count,
            })
        if len(rows) % (10 * len(methods)) == 0:
            print(f"matched {len(rows)} method/query rows", flush=True)

    df = pd.DataFrame(rows)
    if args.kill_test:
        calibration = df[df["split"] == "calibration"]
        bad = (~calibration["present"]) | (calibration["error_m"] > 25.0) | calibration["error_m"].isna()
        false_scores = calibration.loc[bad, "score"]
        threshold = float(np.nextafter(false_scores.max(), np.inf))
        df["accepted"] = np.isfinite(df["error_m"]) & (df["score"] >= threshold)
        thresholds = {"osm": threshold}
        summary_rows = []
        test = df[df["split"] == "test"]
        for condition in ("aligned",):
            rows_condition = test[test["condition"] == condition]
            pos = rows_condition[rows_condition["present"]]
            neg = rows_condition[~rows_condition["present"]]
            accepted = pos[pos["accepted"]]
            summary_rows.append({
                "method": "osm", "condition": condition, "positive_n": len(pos), "negative_n": len(neg),
                "accepted_positive": int(len(accepted)),
                "correct_accepted_10m": int((accepted["error_m"] <= 10).sum()),
                "false_accepts_negative": int(neg["accepted"].sum()),
            })
        good = test[(test["present"]) & test["accepted"] & (test["error_m"] <= 25)]
        if len(good):
            dx = good["estimate_x"].to_numpy(float) - good["truth_x"].to_numpy(float)
            dy = -(good["estimate_y"].to_numpy(float) - good["truth_y"].to_numpy(float))
            median_dx, median_dy = float(np.median(dx)), float(np.median(dy))
            offset_m = float(np.hypot(median_dx, median_dy))
        else:
            median_dx = median_dy = offset_m = None
        pilot = {
            "verdict": "kill" if summary_rows[0]["correct_accepted_10m"] < 5 or (len(good) >= 5 and offset_m > 10) else "continue",
            "count_per_split": args.count,
            "threshold_strictly_above_all_calibration_false_scores": thresholds,
            "test_results": summary_rows,
            "apparent_osm_orthophoto_offset": {
                "n": int(len(good)), "median_east_m": median_dx, "median_north_m": median_dy,
                "median_vector_magnitude": offset_m,
                "systematic_offset_evaluable": bool(len(good) >= 5),
                "interpretation": "apparent OSM-to-orthophoto translation; includes road-width, image-edge, and matcher bias",
            },
            "kill_rule": "kill if fewer than 5/30 correct aligned fixes at 10 m or systematic apparent offset >10 m with at least 5 matched positives",
            "seconds": time.perf_counter() - start,
        }
        df.to_csv(args.output / "kill_test_matches.csv", index=False)
        (args.output / "kill_test_summary.json").write_text(json.dumps(pilot, indent=2, allow_nan=False) + "\n")
        print(json.dumps(pilot, indent=2, allow_nan=False), flush=True)
        print("KILL TEST VERDICT:", pilot["verdict"].upper(), flush=True)
        return

    thresholds = {method: build_threshold(df, method) for method in methods}
    for method, threshold in thresholds.items():
        method_rows = df[df["method"] == method]
        df.loc[df["method"] == method, "accepted"] = np.isfinite(method_rows["error_m"].to_numpy(float)) & \
            (method_rows["score"].to_numpy(float) >= threshold)
    all_rows = add_agreement_gate(df, thresholds)
    summary = summarize(all_rows, thresholds, args.count)
    summary.update({
        "protocol": "2020 orthophoto synthetic nadir queries; OSM roads vs 2018 orthophoto XFeat; no flight validation",
        "data": {
            "osm_source": "Geofabrik Taiwan shapefile extract: https://download.geofabrik.de/asia/taiwan-latest-free.shp.zip",
            "osm_snapshot": "2026-10-01T20:22:06Z",
            "osm_license": "Open Database License 1.0 (ODbL-1.0); attribution: © OpenStreetMap contributors, Geofabrik download",
            "osm_road_layer": str(args.osm_roads),
            "raster_path": str(raster_path),
            "raster_crs": "EPSG:3826",
            "raster_resolution_m": 1,
            "road_features_rasterized": road_feature_count,
            "road_pixels": int(np.count_nonzero(roads)),
            "road_classes": road_classes,
            "assumed_road_widths_m": ROAD_WIDTH_M,
            "camera_image": str(image_paths[1]),
            "map_image": str(image_paths[0]),
            "xfeat_weights": str(args.xfeat_root / "weights/xfeat.pt"),
        },
        "method": {
            "osm": "Canny camera edges vs OSM buffered-road boundaries; 8 unsigned orientation bins, ±1-bin tolerance; distance transform; min mean chamfer over yaw ±20° (10° steps), scales 0.9/1.0/1.1; confidence=(surface median-best)/robust scale with std fallback; 1 m pixel units",
            "xfeat": "unchanged o_map_benchmark.load_xfeat/learned_match (2018 orthophoto reference)",
            "agreement_gate": "accept XFeat estimate only if both calibrated methods accept and their centers are within 10 m",
            "threshold_rule": "per-method score threshold is strictly above every calibration negative or positive error >25 m",
            "negative_rule": "same spatially distinct alternative-centre construction as o_map_benchmark.py",
        },
        "source_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in image_paths},
        "seed": args.seed,
        "counts": {key: len(value) for key, value in groups.items()},
        "opencv": cv2.__version__,
        "numpy": np.__version__,
        "python": platform.python_version(),
        "seconds": time.perf_counter() - start,
    })
    all_rows.to_csv(args.output / "matches.csv", index=False)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print_summary(summary)
    print("Thresholds strictly above calibration false scores:", thresholds)
    print(f"Full run finished in {summary['seconds']:.1f} s; outputs: {args.output}", flush=True)


if __name__ == "__main__":
    main()

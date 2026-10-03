#!/usr/bin/env python3
"""X3: Sentinel-2/OSM coastline matching, with an RGB-NDWI kill gate.

The input imagery is limited to 3 km x 3 km windows read from public Sentinel-2
COGs with GDAL /vsicurl range requests. The fix is a 2-D translation of OSM
coastline (or mapped breakwaters/groynes) against an observed water boundary.
It is a satellite proxy experiment, not a drone-flight result.
"""
from __future__ import annotations

import argparse
import json
import math
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
import time
import http.cookiejar
import urllib.error
from datetime import date
from html.parser import HTMLParser
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rasterio
from pyproj import CRS, Transformer
from rasterio.transform import Affine
from rasterio.windows import from_bounds
from scipy.ndimage import distance_transform_edt

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / "data/raw/sentinel2"
OUT = ROOT / "data/processed/u3_coastline"
STAC = "https://earth-search.aws.element84.com/v1/search"
OSM_API = "https://api.openstreetmap.org/api/0.6/map"
OVERPASS_ENDPOINTS = (
    "https://overpass-api.de/api/interpreter",
    "https://lz4.overpass-api.de/api/interpreter",
)
CWA_HISTORY = "https://opendata.cwa.gov.tw/historyapi/v1/getDataId/O-B0076-001"
CWA_PORTAL = "https://ocean.cwa.gov.tw/V2/"
CWA_FUXING_FORECAST = "https://www.cwa.gov.tw/V8/E/M/Fishery/tide_30day_MOD/T000706.html"
SCENE_ID = "S2B_50QRM_20260603_1_L2A"
WINDOW_M = 1500.0
PIXEL_M = 10.0
COAST_BUFFER_M = 500.0
MAP_RADIUS_M = 13500.0
SEARCH_RADIUS_PX = 150
RGB_IOU_KILL = 0.70
NEGATIVE_COUNT = 30
MIN_COAST_PIXELS = 12
SITES = (
    {"name": "changhua_mudflats", "lon": 120.30, "lat": 23.95},
    {"name": "taichung_port", "lon": 120.50, "lat": 24.28},
    {"name": "hualien_cliffs", "lon": 121.62, "lat": 23.98},
    {"name": "magong_penghu", "lon": 119.58, "lat": 23.57},
)


def fetch_json(url: str, payload: dict | None = None) -> dict:
    headers = {"User-Agent": "TaipeiDrift/1.0 coastline research"}
    if payload is None:
        request = urllib.request.Request(url, headers=headers)
    else:
        request = urllib.request.Request(
            url,
            data=json.dumps(payload).encode(),
            headers={**headers, "Content-Type": "application/json"},
        )
    with urllib.request.urlopen(request, timeout=60) as response:
        return json.load(response)


def get_kill_scene() -> dict:
    query = {
        "collections": ["sentinel-2-l2a"],
        "bbox": [120.27, 23.93, 120.33, 23.97],
        "limit": 100,
        "query": {"eo:cloud_cover": {"lt": 10}},
    }
    features = fetch_json(STAC, query)["features"]
    for feature in features:
        if feature["id"] == SCENE_ID:
            return feature
    raise RuntimeError(f"Low-cloud Sentinel item {SCENE_ID} was not returned by Earth Search")


def read_kill_window(scene: dict) -> tuple[dict[str, np.ndarray], Affine, CRS]:
    site = SITES[0]
    arrays, transform, crs = read_scene_window(scene, site, use_cache=False)
    return arrays, transform, crs


def fetch_osm_xml(site: dict, full_search: bool) -> bytes:
    span = 0.12 if full_search else 0.03
    west, east = site["lon"] - span, site["lon"] + span
    south, north = site["lat"] - span, site["lat"] + span
    params = urllib.parse.urlencode({"bbox": f"{west},{south},{east},{north}"})
    request = urllib.request.Request(
        f"{OSM_API}?{params}",
        headers={"User-Agent": "TaipeiDrift/1.0 coastline research"},
    )
    with urllib.request.urlopen(request, timeout=90) as response:
        return response.read()

def fetch_osm_overpass(site: dict) -> dict:
    lat_span = MAP_RADIUS_M / 111_000.0 + 0.003
    lon_span = MAP_RADIUS_M / (111_000.0 * math.cos(math.radians(site["lat"]))) + 0.003
    west, east = site["lon"] - lon_span, site["lon"] + lon_span
    south, north = site["lat"] - lat_span, site["lat"] + lat_span

    def request_box(
        box_south: float, box_west: float, box_north: float, box_east: float, depth: int = 0
    ) -> list[dict]:
        bbox = f"{box_south},{box_west},{box_north},{box_east}"
        query = (
            "[out:json][timeout:35];("
            f'way["natural"="coastline"]({bbox});'
            f'way["man_made"~"^(breakwater|groyne)$"]({bbox});'
            f");out geom({bbox});"
        )
        last_error = None
        for attempt in range(2):
            for endpoint in OVERPASS_ENDPOINTS:
                request = urllib.request.Request(
                    endpoint,
                    data=urllib.parse.urlencode({"data": query}).encode(),
                    headers={
                        "User-Agent": "TaipeiDrift/1.0 coastline research",
                        "Content-Type": "application/x-www-form-urlencoded",
                        "Accept": "application/json",
                    },
                )
                try:
                    with urllib.request.urlopen(request, timeout=55) as response:
                        return json.load(response).get("elements", [])
                except urllib.error.HTTPError as exc:
                    last_error = exc
                    if exc.code == 400:
                        break
                except (TimeoutError, urllib.error.URLError) as exc:
                    last_error = exc
            if attempt == 0:
                time.sleep(1)
        if depth >= 2:
            raise last_error
        middle_lat = (box_south + box_north) / 2.0
        middle_lon = (box_west + box_east) / 2.0
        elements = []
        for child in (
            (box_south, box_west, middle_lat, middle_lon),
            (box_south, middle_lon, middle_lat, box_east),
            (middle_lat, box_west, box_north, middle_lon),
            (middle_lat, middle_lon, box_north, box_east),
        ):
            elements.extend(request_box(*child, depth + 1))
        return elements

    middle_lon = site["lon"]
    middle_lat = site["lat"]
    elements = []
    for bounds in (
        (south, west, middle_lat, middle_lon),
        (south, middle_lon, middle_lat, east),
        (middle_lat, west, north, middle_lon),
        (middle_lat, middle_lon, north, east),
    ):
        elements.extend(request_box(*bounds))
    return {"elements": elements}


def parse_overpass_lines(data: dict, crs: CRS) -> tuple[list[np.ndarray], list[np.ndarray], int]:
    to_map = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    coast: list[np.ndarray] = []
    structures: list[np.ndarray] = []
    coast_nodes = 0
    for element in data.get("elements", []):
        geometry = [
            point for point in element.get("geometry", [])
            if isinstance(point, dict) and "lon" in point and "lat" in point
        ]
        if len(geometry) < 2:
            continue
        projected = np.asarray(
            [to_map.transform(point["lon"], point["lat"]) for point in geometry],
        )
        tags = element.get("tags", {})
        if tags.get("natural") == "coastline":
            coast.append(projected)
            coast_nodes += len(projected)
        if tags.get("man_made") in {"breakwater", "groyne"}:
            structures.append(projected)
    return coast, structures, coast_nodes


def parse_osm_lines(xml_bytes: bytes, crs: CRS) -> tuple[list[np.ndarray], list[np.ndarray], int]:
    root = ET.fromstring(xml_bytes)
    nodes = {
        node.attrib["id"]: (float(node.attrib["lon"]), float(node.attrib["lat"]))
        for node in root.findall("node")
    }
    to_map = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    coast: list[np.ndarray] = []
    structures: list[np.ndarray] = []
    coast_nodes = 0
    for way in root.findall("way"):
        tags = {tag.attrib.get("k"): tag.attrib.get("v") for tag in way.findall("tag")}
        refs = [node.attrib["ref"] for node in way.findall("nd")]
        lonlat = [nodes[ref] for ref in refs if ref in nodes]
        if len(lonlat) < 2:
            continue
        projected = np.asarray([to_map.transform(*point) for point in lonlat], dtype=np.float64)
        if tags.get("natural") == "coastline":
            coast.append(projected)
            coast_nodes += len(projected)
        if tags.get("man_made") in {"breakwater", "groyne"}:
            structures.append(projected)
    return coast, structures, coast_nodes


def rasterize_lines(
    lines: list[np.ndarray], center_xy: tuple[float, float], radius_m: float = MAP_RADIUS_M
) -> tuple[np.ndarray, Affine]:
    cx, cy = center_xy
    left = math.floor((cx - radius_m) / PIXEL_M) * PIXEL_M
    top = math.ceil((cy + radius_m) / PIXEL_M) * PIXEL_M
    width = int(math.ceil((cx + radius_m - left) / PIXEL_M))
    height = int(math.ceil((top - (cy - radius_m)) / PIXEL_M))
    transform = Affine(PIXEL_M, 0.0, left, 0.0, -PIXEL_M, top)
    raster = np.zeros((height, width), dtype=np.uint8)
    inv = ~transform
    for line in lines:
        pixels = []
        for x, y in line:
            col, row = inv * (x, y)
            pixels.append((int(round(col)), int(round(row))))
        if len(pixels) > 1:
            cv2.polylines(raster, [np.asarray(pixels, dtype=np.int32)], False, 1, 1)
    return raster, transform


def crop_map_mask(
    map_mask: np.ndarray, map_transform: Affine, image_transform: Affine, shape: tuple[int, int]
) -> np.ndarray:
    height, width = shape
    col0 = int(round((image_transform.c - map_transform.c) / PIXEL_M))
    row0 = int(round((map_transform.f - image_transform.f) / PIXEL_M))
    result = np.zeros(shape, dtype=np.uint8)
    src_c0, src_r0 = max(0, col0), max(0, row0)
    src_c1 = min(map_mask.shape[1], col0 + width)
    src_r1 = min(map_mask.shape[0], row0 + height)
    if src_c0 < src_c1 and src_r0 < src_r1:
        dst_c0, dst_r0 = src_c0 - col0, src_r0 - row0
        result[dst_r0 : dst_r0 + src_r1 - src_r0, dst_c0 : dst_c0 + src_c1 - src_c0] = map_mask[
            src_r0:src_r1, src_c0:src_c1
        ]
    return result


def stac_site_scenes(site: dict, count: int) -> list[dict]:
    lat_delta = WINDOW_M / 111_000.0 + 0.001
    lon_delta = WINDOW_M / (111_000.0 * math.cos(math.radians(site["lat"]))) + 0.001
    bbox = [
        site["lon"] - lon_delta,
        site["lat"] - lat_delta,
        site["lon"] + lon_delta,
        site["lat"] + lat_delta,
    ]
    query = {
        "collections": ["sentinel-2-l2a"],
        "bbox": [site["lon"] - 0.06, site["lat"] - 0.05, site["lon"] + 0.06, site["lat"] + 0.05],
        "limit": 100,
        "sortby": [{"field": "properties.datetime", "direction": "desc"}],
        "query": {"eo:cloud_cover": {"lt": 10}},
    }
    features = fetch_json(STAC, query)["features"]
    covering = []
    west, south, east, north = bbox
    for feature in features:
        item_west, item_south, item_east, item_north = feature["bbox"]
        if (
            item_west <= west
            and item_south <= south
            and item_east >= east
            and item_north >= north
        ):
            covering.append(feature)
    covering.sort(key=lambda item: item["properties"]["datetime"], reverse=True)
    if not covering:
        return []
    # Adjacent MGRS tiles can use different UTM zones near the zone boundary.
    # Keep one zone so every date shares the same metric grid.
    zone = covering[0]["id"].split("_")[1][:2]
    by_day: dict[str, dict] = {}
    for feature in covering:
        if feature["id"].split("_")[1][:2] != zone:
            continue
        day = feature["properties"]["datetime"][:10]
        prior = by_day.get(day)
        if prior is None or feature["properties"]["eo:cloud_cover"] < prior["properties"]["eo:cloud_cover"]:
            by_day[day] = feature
    candidates = sorted(by_day.values(), key=lambda item: item["properties"]["datetime"], reverse=True)
    if not candidates:
        return []
    selected = [candidates[0]]
    for feature in candidates[1:]:
        day = date.fromisoformat(feature["properties"]["datetime"][:10])
        if all(abs((day - date.fromisoformat(item["properties"]["datetime"][:10])).days) >= 5 for item in selected):
            selected.append(feature)
        if len(selected) >= count:
            break
    return selected


def read_scene_window(
    scene: dict, site: dict, use_cache: bool = True
) -> tuple[dict[str, np.ndarray], Affine, CRS]:
    scene_date = scene["properties"]["datetime"][:10]
    cache_path = RAW / f"u3_{site['name']}_{scene_date}_10m_window.npz"
    if use_cache and cache_path.exists():
        with np.load(cache_path, allow_pickle=False) as saved:
            if str(saved["scene_id"].item()) == scene["id"]:
                arrays = {key: saved[key] for key in ("visual", "green", "nir", "scl", "valid")}
                transform = Affine(*saved["transform"].tolist())
                crs = CRS.from_string(str(saved["crs"].item()))
                return arrays, transform, crs
    with rasterio.Env(
        GDAL_DISABLE_READDIR_ON_OPEN="EMPTY_DIR",
        CPL_VSIL_CURL_ALLOWED_EXTENSIONS=".tif",
        GDAL_HTTP_MULTIRANGE="YES",
    ):
        with rasterio.open("/vsicurl/" + scene["assets"]["visual"]["href"]) as visual_ds:
            crs = CRS.from_user_input(visual_ds.crs)
            if visual_ds.res != (PIXEL_M, PIXEL_M):
                raise RuntimeError(f"Unexpected visual COG pixel size {visual_ds.res}")
            to_map = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
            cx, cy = to_map.transform(site["lon"], site["lat"])
            bounds = (cx - WINDOW_M, cy - WINDOW_M, cx + WINDOW_M, cy + WINDOW_M)
            window = from_bounds(*bounds, transform=visual_ds.transform).round_offsets().round_lengths()
            visual = visual_ds.read(window=window)
            image_transform = visual_ds.window_transform(window)
        if visual.shape != (3, 300, 300):
            raise RuntimeError(f"Expected 3x300x300 visual COG window, got {visual.shape} ({scene['id']})")
        arrays: dict[str, np.ndarray] = {"visual": visual}
        for band in ("green", "nir"):
            with rasterio.open("/vsicurl/" + scene["assets"][band]["href"]) as dataset:
                band_window = from_bounds(*bounds, transform=dataset.transform).round_offsets().round_lengths()
                arrays[band] = dataset.read(1, window=band_window)[None, ...]
        with rasterio.open("/vsicurl/" + scene["assets"]["scl"]["href"]) as scl_ds:
            scl_window = from_bounds(*bounds, transform=scl_ds.transform).round_offsets().round_lengths()
            scl = scl_ds.read(1, window=scl_window)
        if arrays["green"].shape != (1, 300, 300) or arrays["nir"].shape != (1, 300, 300):
            raise RuntimeError(f"Sentinel 10m bands do not align for {scene['id']}")
        scl_10m = cv2.resize(scl, (300, 300), interpolation=cv2.INTER_NEAREST)
        # SCL invalid / cloud / shadow / cirrus / snow classes: 0, 1, 3, 8-11.
        valid = ~np.isin(scl_10m, [0, 1, 3, 8, 9, 10, 11])
        arrays["scl"] = scl[None, ...]
        arrays["valid"] = valid
    if use_cache:
        RAW.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            cache_path,
            **arrays,
            transform=np.asarray(tuple(image_transform)[:6], dtype=np.float64),
            crs=crs.to_string(),
            scene_id=scene["id"],
        )
    return arrays, image_transform, crs


def water_masks(arrays: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rgb = arrays["visual"][:3].astype(np.float32)
    green = arrays["green"][0].astype(np.float32)
    nir = arrays["nir"][0].astype(np.float32)
    valid = arrays["valid"].astype(bool)
    ndwi = (green - nir) / (green + nir + 1e-6)
    ndwi_water = (ndwi > 0.0) & valid
    rgb_water = (
        (rgb[2] > 1.05 * rgb[0])
        & (rgb[1] > 1.02 * rgb[0])
        & (rgb.max(axis=0) < 220)
        & valid
    )
    rgb_loose = ((rgb[2] > 1.03 * rgb[0]) & (rgb[1] > 0.95 * rgb[0])) & valid
    return rgb_water, rgb_loose, ndwi_water, valid


def calc_iou(left: np.ndarray, right: np.ndarray, roi: np.ndarray, valid: np.ndarray) -> float:
    region = roi & valid
    a = left & region
    b = right & region
    union = int(np.count_nonzero(a | b))
    return float(np.count_nonzero(a & b) / union) if union else float("nan")


def open_water_boundary(mask: np.ndarray, valid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    water = (mask & valid).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(water, connectivity=8)
    if count <= 1:
        raise RuntimeError("Water mask has no connected components")
    border_labels = np.unique(np.concatenate((labels[0], labels[-1], labels[:, 0], labels[:, -1])))
    options = [int(label) for label in border_labels if label > 0]
    if not options:
        options = list(range(1, count))
    chosen = max(options, key=lambda label: int(stats[label, cv2.CC_STAT_AREA]))
    ocean = (labels == chosen).astype(np.uint8)
    edge = cv2.morphologyEx(ocean, cv2.MORPH_GRADIENT, np.ones((3, 3), dtype=np.uint8)) > 0
    good = cv2.erode(valid.astype(np.uint8), np.ones((3, 3), dtype=np.uint8)) > 0
    edge &= good
    edge[[0, -1], :] = False
    edge[:, [0, -1]] = False
    if np.count_nonzero(edge) < 30:
        raise RuntimeError("Observed open-water boundary has fewer than 30 pixels")
    distance_m = np.minimum(distance_transform_edt(~edge) * PIXEL_M, 500.0).astype(np.float32)
    return edge, distance_m


def match_response(map_mask: np.ndarray, distance_m: np.ndarray, valid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source = map_mask.astype(np.float32)
    valid_f = valid.astype(np.float32)
    distance_template = distance_m * valid_f
    summed_distance = cv2.matchTemplate(source, distance_template, cv2.TM_CCORR)
    coast_pixels = cv2.matchTemplate(source, valid_f, cv2.TM_CCORR)
    score = np.full(summed_distance.shape, np.nan, dtype=np.float32)
    enough = coast_pixels >= MIN_COAST_PIXELS
    score[enough] = -summed_distance[enough] / coast_pixels[enough]
    return score, coast_pixels


def base_placement(map_transform: Affine, image_transform: Affine) -> tuple[int, int]:
    col = int(round((image_transform.c - map_transform.c) / PIXEL_M))
    row = int(round((map_transform.f - image_transform.f) / PIXEL_M))
    return col, row


def best_match(
    response: np.ndarray,
    coast_pixels: np.ndarray,
    base: tuple[int, int],
    prior_east_m: float = 0.0,
    prior_north_m: float = 0.0,
) -> dict[str, float] | None:
    base_col, base_row = base
    center_col = base_col + int(round(prior_east_m / PIXEL_M))
    center_row = base_row - int(round(prior_north_m / PIXEL_M))
    col0 = max(0, center_col - SEARCH_RADIUS_PX)
    col1 = min(response.shape[1], center_col + SEARCH_RADIUS_PX + 1)
    row0 = max(0, center_row - SEARCH_RADIUS_PX)
    row1 = min(response.shape[0], center_row + SEARCH_RADIUS_PX + 1)
    local = response[row0:row1, col0:col1].copy()
    local_rows = np.arange(row0, row1) - center_row
    local_cols = np.arange(col0, col1) - center_col
    outside_search_radius = (
        local_rows[:, None] ** 2 + local_cols[None, :] ** 2
    ) * PIXEL_M**2 > WINDOW_M**2
    local[outside_search_radius] = np.nan
    if local.size == 0 or not np.any(np.isfinite(local)):
        return None
    local_index = int(np.nanargmax(local))
    iy, ix = np.unravel_index(local_index, local.shape)
    row, col = row0 + int(iy), col0 + int(ix)
    score = float(response[row, col])
    return {
        "score": score,
        "mean_chamfer_m": -score,
        "map_to_observation_east_m": (col - base_col) * PIXEL_M,
        "map_to_observation_north_m": (base_row - row) * PIXEL_M,
        "coast_pixels_scored": float(coast_pixels[row, col]),
        "prior_east_m": prior_east_m,
        "prior_north_m": prior_north_m,
    }


def shifted_offsets(site_seed: int, cohort: int, count: int = NEGATIVE_COUNT) -> list[tuple[float, float]]:
    offsets = []
    phase = (site_seed * 29.0 + cohort * 47.0) % 360.0
    for index in range(count):
        radius = 3000.0 + 7000.0 * ((index + 0.5) / count)
        angle = math.radians(phase + index * 137.507764)
        offsets.append((radius * math.cos(angle), radius * math.sin(angle)))
    return offsets


def score_offsets(
    response: np.ndarray,
    coast_pixels: np.ndarray,
    base: tuple[int, int],
    offsets: list[tuple[float, float]],
) -> list[dict[str, Any]]:
    results = []
    for east, north in offsets:
        result = best_match(response, coast_pixels, base, east, north)
        if result is not None:
            result["shift_distance_m"] = math.hypot(east, north)
            result["shifted_prior_east_m"] = east
            result["shifted_prior_north_m"] = north
            results.append(result)
    return results


def local_basis(coast_lines: list[np.ndarray], center_xy: tuple[float, float]) -> dict[str, Any] | None:
    cx, cy = center_xy
    nearest: tuple[float, np.ndarray] | None = None
    for line in coast_lines:
        for first, second in zip(line[:-1], line[1:]):
            segment = second - first
            length2 = float(np.dot(segment, segment))
            if length2 <= 0:
                continue
            fraction = float(np.clip(np.dot(np.array([cx, cy]) - first, segment) / length2, 0.0, 1.0))
            point = first + fraction * segment
            distance = float(np.linalg.norm(point - np.array([cx, cy])))
            if nearest is None or distance < nearest[0]:
                nearest = (distance, segment)
    if nearest is None:
        return None
    tangent = nearest[1] / np.linalg.norm(nearest[1])
    if tangent[0] < 0 or (abs(tangent[0]) < 1e-9 and tangent[1] < 0):
        tangent = -tangent
    normal = np.array([-tangent[1], tangent[0]])
    return {
        "nearest_osm_coast_distance_m": nearest[0],
        "tangent_east_north": tangent.tolist(),
        "normal_east_north": normal.tolist(),
    }


def annotate_errors(match: dict[str, Any], basis: dict[str, Any] | None) -> dict[str, Any]:
    if basis is None:
        match["cross_shore_error_m"] = None
        match["along_shore_error_m"] = None
        return match
    # Vehicle correction is the opposite of the map-to-observation translation.
    correction = np.array(
        [-match["map_to_observation_east_m"], -match["map_to_observation_north_m"]]
    )
    tangent = np.asarray(basis["tangent_east_north"])
    normal = np.asarray(basis["normal_east_north"])
    match["position_correction_east_m"] = float(correction[0])
    match["position_correction_north_m"] = float(correction[1])
    match["cross_shore_error_m"] = float(np.dot(correction, normal))
    match["along_shore_error_m"] = float(np.dot(correction, tangent))
    return match


def distribution(matches: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [item for item in matches if item.get("cross_shore_error_m") is not None]
    result = {"n": len(valid), "accepted_n": sum(bool(item.get("accepted")) for item in valid)}
    for name in ("cross_shore_error_m", "along_shore_error_m"):
        values = np.asarray([abs(float(item[name])) for item in valid], dtype=np.float64)
        result[name + "_abs_p50_m"] = float(np.percentile(values, 50)) if values.size else None
        result[name + "_abs_p95_m"] = float(np.percentile(values, 95)) if values.size else None
        signed = np.asarray([float(item[name]) for item in valid], dtype=np.float64)
        result[name + "_signed_median_m"] = float(np.median(signed)) if signed.size else None
    return result


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(json_safe(value), indent=2, allow_nan=False) + "\n")


def load_osm_for_site(site: dict, crs: CRS) -> tuple[list[np.ndarray], list[np.ndarray], int, str]:
    RAW.mkdir(parents=True, exist_ok=True)
    path = RAW / f"u3_{site['name']}_osm_search_r13p8km.json"
    if path.exists():
        data = json.loads(path.read_text())
    else:
        data = fetch_osm_overpass(site)
        path.write_text(json.dumps(data))
    coast, structures, coast_nodes = parse_overpass_lines(data, crs)
    return coast, structures, coast_nodes, str(path.relative_to(ROOT))


class TideTableParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.rows: list[list[str]] = []
        self.row: list[str] | None = None
        self.cell: list[str] | None = None

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag == "tr":
            self.row = []
        elif tag in {"td", "th"}:
            self.cell = []

    def handle_data(self, data: str) -> None:
        if self.cell is not None:
            value = data.strip()
            if value:
                self.cell.append(value)

    def handle_endtag(self, tag: str) -> None:
        if tag in {"td", "th"} and self.cell is not None:
            if self.row is not None:
                self.row.append(" ".join(self.cell))
            self.cell = None
        elif tag == "tr" and self.row is not None:
            self.rows.append(self.row)
            self.row = None


def read_fuxing_prediction() -> dict[str, Any]:
    with urllib.request.urlopen(CWA_FUXING_FORECAST, timeout=30) as response:
        parser = TideTableParser()
        parser.feed(response.read().decode("utf-8", errors="replace"))
    levels_cm = []
    for row in parser.rows:
        try:
            # Date rows have a leading date cell; continuation rows omit it.
            level_index = 3 if len(row) >= 6 else 2 if len(row) >= 5 else -1
            if level_index >= 0:
                levels_cm.append(float(row[level_index]))
        except ValueError:
            continue
    if not levels_cm:
        raise RuntimeError("CWA Fuxing 30-day page contained no tide-height rows")
    low, high = min(levels_cm), max(levels_cm)
    return {
        "label": "PUBLISHED",
        "source": CWA_FUXING_FORECAST,
        "station": "Changhua County Fuxing (T000706)",
        "forecast_window": "next 30 days at fetch time; not scene-date historical data",
        "vertical_datum_min_cm": low,
        "vertical_datum_max_cm": high,
        "vertical_datum_range_m": (high - low) / 100.0,
        "height_rows": len(levels_cm),
    }
def tide_access_check() -> dict[str, Any]:
    status: dict[str, Any] = {
        "dataset": "CWA O-B0076-001 tide observations / Tide-his annual history",
        "historical_api_url": CWA_HISTORY,
        "label": "PUBLISHED access attempt",
    }
    try:
        with urllib.request.urlopen(CWA_HISTORY, timeout=25) as response:
            status["historical_api_http_status"] = response.status
            status["historical_api_content_type"] = response.headers.get("Content-Type")
    except Exception as exc:
        status["historical_api_error"] = f"{type(exc).__name__}: {exc}"
    try:
        opener = urllib.request.build_opener(
            urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar())
        )
        opener.open(CWA_PORTAL + "data_interface/datasets", timeout=30).read(1)
        request = urllib.request.Request(
            CWA_PORTAL + "data_interface/datasets/get_api_dataprovider",
            data=b"",
            headers={"User-Agent": "TaipeiDrift/1.0", "X-Requested-With": "XMLHttpRequest"},
            method="POST",
        )
        catalog = json.load(opener.open(request, timeout=30))
        item = next(row for row in catalog["tableForm"] if row.get("dataset") == "Tide-his")
        token = next(row["token"] for row in catalog["dataset_order"] if row["longname"] == "Tide-his")
        params = {
            "dataset": "Tide-his",
            "title": item["title"],
            "format": "CSV",
            "all_formats": ",".join(item["mothods"]),
            "token": token,
        }
        url = CWA_PORTAL + "data_interface/download?" + urllib.parse.urlencode(params)
        response = opener.open(urllib.request.Request(url, headers={"User-Agent": "TaipeiDrift/1.0"}), timeout=40)
        content_type = response.headers.get("Content-Type", "")
        status["portal_catalog_available"] = True
        status["portal_history_download_http_status"] = response.status
        status["portal_history_download_content_type"] = content_type
        if "csv" in content_type.lower() or "octet-stream" in content_type.lower():
            data = response.read()
            tide_path = RAW / "u3_cwa_tide_history.csv"
            tide_path.write_bytes(data)
            status["historical_csv_downloaded"] = str(tide_path.relative_to(ROOT))
            status["historical_csv_bytes"] = len(data)
        else:
            status["historical_csv_downloaded"] = False
            status["portal_note"] = "Download route returned HTML, not the tide CSV."
            response.close()
    except Exception as exc:
        status["portal_error"] = f"{type(exc).__name__}: {exc}"
    status["fuxing_30day_prediction_url"] = CWA_FUXING_FORECAST
    try:
        status["fuxing_30day_prediction"] = read_fuxing_prediction()
    except Exception as exc:
        status["fuxing_30day_prediction_error"] = f"{type(exc).__name__}: {exc}"
    status["scene_date_tide_levels_available"] = bool(status.get("historical_csv_downloaded"))
    return status


def run_kill_test() -> dict[str, Any]:
    site = SITES[0]
    scene = get_kill_scene()
    arrays, transform, crs = read_kill_window(scene)
    osm_xml = fetch_osm_xml(site, full_search=False)
    coast_lines, _, coast_nodes = parse_osm_lines(osm_xml, crs)
    center = Transformer.from_crs("EPSG:4326", crs, always_xy=True).transform(site["lon"], site["lat"])
    local_map, map_transform = rasterize_lines(coast_lines, center, radius_m=2500.0)
    coast = crop_map_mask(local_map, map_transform, transform, (300, 300))
    roi = distance_transform_edt(coast == 0) * PIXEL_M <= COAST_BUFFER_M
    rgb, rgb_loose, ndwi, valid = water_masks(arrays)
    main_iou = calc_iou(rgb, ndwi, roi, valid)
    loose_iou = calc_iou(rgb_loose, ndwi, roi, valid)
    cloud = float(scene["properties"]["eo:cloud_cover"])
    scene_date = scene["properties"]["datetime"]
    raw_path = RAW / f"u3_changhua_{scene_date[:10]}_10m_window.npz"
    if not raw_path.exists():
        arrays_to_save = {**arrays, "transform": np.asarray(tuple(transform)[:6], dtype=np.float64), "crs": crs.to_string(), "scene_id": scene["id"]}
        np.savez_compressed(raw_path, **arrays_to_save)
    osm_path = RAW / "u3_changhua_osm_coastline.xml"
    osm_path.write_bytes(osm_xml)
    result = {
        "label": "MEASURED",
        "experiment": "X3 coastline RGB-NDWI 20-minute kill test",
        "site": site["name"],
        "stac_collection": "sentinel-2-l2a",
        "stac_item": scene["id"],
        "acquired_utc": scene_date,
        "eo_cloud_cover_percent": cloud,
        "sentinel_asset_resolution_m": PIXEL_M,
        "window_shape_px": [300, 300],
        "window_size_km": 3.0,
        "window_crs": crs.to_string(),
        "cog_assets_range_read": ["visual", "green", "nir", "scl"],
        "rgb_water_rule": "B > 1.05*R, G > 1.02*R, max(R,G,B) < 220; visual COG uint8",
        "ndwi_rule": "(green - nir)/(green + nir) > 0; L2A COG reflectance bands",
        "osm_source": "OpenStreetMap API map bbox; natural=coastline; ODbL attribution required",
        "osm_coastline_ways": len(coast_lines),
        "osm_coastline_nodes": coast_nodes,
        "coast_roi_buffer_m": COAST_BUFFER_M,
        "coast_roi_pixels": int(np.count_nonzero(roi)),
        "ndwi_water_pixels_in_coast_roi": int(np.count_nonzero(ndwi & roi & valid)),
        "rgb_water_pixels_in_coast_roi": int(np.count_nonzero(rgb & roi & valid)),
        "rgb_ndwi_iou_coast_roi": main_iou,
        "rgb_ndwi_iou_looser_rgb_rule_coast_roi": loose_iou,
        "rgb_ndwi_iou_full_window": calc_iou(rgb, ndwi, np.ones_like(roi), valid),
        "kill_threshold": RGB_IOU_KILL,
        "decision": "KILL" if main_iou < RGB_IOU_KILL else "CONTINUE",
        "raw_window": str(raw_path.relative_to(ROOT)),
        "osm_cache": str(osm_path.relative_to(ROOT)),
        "limitations": [
            "One low-cloud date and one site; multi-site results are stored separately.",
            "NDWI is a comparator, not ground truth; the IoU is a segmentation agreement gate.",
            "The RGB threshold is fixed and declared, not tuned against NDWI.",
            "Satellite geolocation and OSM shoreline accuracy are not ground-truth navigation errors.",
        ],
    }
    (OUT / "kill_test.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"kill test: {site['name']} {scene_date[:10]} IoU={main_iou:.3f} "
        f"(looser RGB={loose_iou:.3f}), threshold={RGB_IOU_KILL:.2f}: {result['decision']}"
    )
    return result


def match_feature(
    scene_records: list[dict[str, Any]],
    feature_mask: np.ndarray,
    map_transform: Affine,
    basis: dict[str, Any] | None,
    seed: int,
) -> dict[str, Any]:
    if not np.any(feature_mask):
        return {"status": "NO_OSM_FEATURES", "calibration_negative_n": 0, "test_negative_n": 0}
    prepared: list[dict[str, Any]] = []
    for record in scene_records:
        try:
            boundary, distance_m = open_water_boundary(record["chosen_water"], record["valid"])
            response, coast_pixels = match_response(feature_mask, distance_m, record["valid"])
            base = base_placement(map_transform, record["transform"])
            prepared.append({**record, "boundary_pixels": int(boundary.sum()), "response": response, "coast_pixels": coast_pixels, "base": base})
        except RuntimeError as exc:
            record["match_error"] = str(exc)
    if not prepared:
        return {"status": "NO_VALID_WATER_BOUNDARIES", "calibration_negative_n": 0, "test_negative_n": 0}
    method_counts: dict[str, int] = {}
    for record in prepared:
        method = record["water_mask_method"]
        method_counts[method] = method_counts.get(method, 0) + 1
    calibration_method = max(method_counts, key=method_counts.get)
    comparable_records = [
        record for record in prepared if record["water_mask_method"] == calibration_method
    ]
    calibration_scene = comparable_records[0]
    validation_scene = comparable_records[1] if len(comparable_records) > 1 else comparable_records[0]
    calibration_scores = score_offsets(
        calibration_scene["response"],
        calibration_scene["coast_pixels"],
        calibration_scene["base"],
        shifted_offsets(seed, 0),
    )
    if not calibration_scores:
        threshold = None
    else:
        threshold = float(np.nextafter(max(item["score"] for item in calibration_scores), math.inf))
    positive_matches = []
    for record in prepared:
        match = best_match(record["response"], record["coast_pixels"], record["base"])
        if match is None:
            record["positive_match"] = None
            continue
        annotate_errors(match, basis)
        comparable = record["water_mask_method"] == calibration_method
        match["confidence_comparable"] = comparable
        match["accepted"] = comparable and threshold is not None and match["score"] > threshold
        record["positive_match"] = match
        positive_matches.append(match)
    test_scores = score_offsets(
        validation_scene["response"],
        validation_scene["coast_pixels"],
        validation_scene["base"],
        shifted_offsets(seed, 1),
    )
    for item in test_scores:
        item["accepted"] = threshold is not None and item["score"] > threshold
    false_accepts = sum(bool(item["accepted"]) for item in test_scores)
    return {
        "status": "MEASURED",
        "score_definition": "negative mean OSM-line-to-observed-water-boundary chamfer distance in metres; higher is better",
        "search_radius_m": WINDOW_M,
        "grid_resolution_m": PIXEL_M,
        "calibration_scene": calibration_scene["scene_id"],
        "calibration_water_mask_method": calibration_scene["water_mask_method"],
        "calibration_mask_population_n": len(comparable_records),
        "calibration_test_scene_independent": calibration_scene["scene_id"] != validation_scene["scene_id"],
        "calibration_negative_attempted": NEGATIVE_COUNT,
        "calibration_negative_scored": len(calibration_scores),
        "calibration_negative_unscored": NEGATIVE_COUNT - len(calibration_scores),
        "acceptance_threshold_strictly_above_calibration_false_scores": threshold,
        "test_negative_scene": validation_scene["scene_id"],
        "test_negative_water_mask_method": validation_scene["water_mask_method"],
        "calibration_test_mask_methods_match": calibration_scene["water_mask_method"] == validation_scene["water_mask_method"],
        "test_negative_attempted": NEGATIVE_COUNT,
        "test_negative_scored": len(test_scores),
        "test_negative_unscored": NEGATIVE_COUNT - len(test_scores),
        "false_accepts": int(false_accepts),
        "unscored_negative_policy": f"auto-reject when fewer than {MIN_COAST_PIXELS} OSM feature pixels overlap the 3 km window",
        "test_negative_scores": test_scores,
        "positive_matches": [
            {"scene_id": item["scene_id"], "date": item["date"], **item["positive_match"]}
            for item in prepared
            if item.get("positive_match") is not None
        ],
        "error_distribution_all_scene_fixes": distribution(positive_matches),
        "error_distribution_accepted_scene_fixes": distribution(
            [item for item in positive_matches if item.get("accepted")]
        ),
    }


def fix_verdict(result: dict[str, Any]) -> str:
    if result.get("status") != "MEASURED":
        return result.get("status", "NO_RESULT")
    accepted = result["error_distribution_accepted_scene_fixes"]
    if accepted["n"] == 0:
        return "INCONCLUSIVE_NO_POSITIVE_FIX_ACCEPTED"
    passed = (
        accepted["cross_shore_error_m_abs_p50_m"] <= 30.0
        and accepted["cross_shore_error_m_abs_p95_m"] <= 100.0
        and result["false_accepts"] <= 1
    )
    return "PASS" if passed else "FAIL_ERROR_OR_FALSE_ACCEPT_LIMIT"


def site_experiment(site: dict, scene_count: int) -> dict[str, Any]:
    print(f"site {site['name']}: querying low-cloud Sentinel scenes", flush=True)
    scenes = stac_site_scenes(site, scene_count)
    if not scenes:
        return {"label": "MEASURED", "site": site["name"], "verdict": "NO_COVERING_LOW_CLOUD_SCENES"}
    arrays_by_scene = []
    for scene in scenes:
        arrays, image_transform, crs = read_scene_window(scene, site)
        arrays_by_scene.append((scene, arrays, image_transform, crs))
    crs = arrays_by_scene[0][3]
    if any(item[3] != crs for item in arrays_by_scene):
        raise RuntimeError(f"Sentinel scenes for {site['name']} use inconsistent projections")
    coast_lines, structure_lines, coast_nodes, osm_path = load_osm_for_site(site, crs)
    center_xy = Transformer.from_crs("EPSG:4326", crs, always_xy=True).transform(site["lon"], site["lat"])
    coast_mask, map_transform = rasterize_lines(coast_lines, center_xy)
    structure_mask, _ = rasterize_lines(structure_lines, center_xy)
    basis = local_basis(coast_lines, center_xy)

    scene_records: list[dict[str, Any]] = []
    per_scene_iou = []
    for scene, arrays, image_transform, _ in arrays_by_scene:
        coast_local = crop_map_mask(coast_mask, map_transform, image_transform, (300, 300))
        coast_roi = distance_transform_edt(coast_local == 0) * PIXEL_M <= COAST_BUFFER_M
        rgb, rgb_loose, ndwi, valid = water_masks(arrays)
        rgb_iou = calc_iou(rgb, ndwi, coast_roi, valid)
        loose_iou = calc_iou(rgb_loose, ndwi, coast_roi, valid)
        scene_date = scene["properties"]["datetime"][:10]
        raw_path = RAW / f"u3_{site['name']}_{scene_date}_10m_window.npz"
        per_scene_iou.append(
            {
                "scene_id": scene["id"],
                "date": scene_date,
                "acquired_utc": scene["properties"]["datetime"],
                "eo_cloud_cover_percent": float(scene["properties"]["eo:cloud_cover"]),
                "rgb_ndwi_iou_coastal_roi": rgb_iou,
                "looser_rgb_iou_coastal_roi": loose_iou,
                "coastal_roi_pixels": int(np.count_nonzero(coast_roi & valid)),
                "local_valid_fraction": float(valid.mean()),
                "raw_window": str(raw_path.relative_to(ROOT)),
            }
        )
        scene_records.append(
            {
                "scene_id": scene["id"],
                "date": scene_date,
                "arrays": arrays,
                "transform": image_transform,
                "rgb_water": rgb,
                "ndwi_water": ndwi,
                "valid": valid,
                "rgb_iou": rgb_iou,
                "chosen_water": None,
            }
        )
    valid_ious = [row["rgb_ndwi_iou_coastal_roi"] for row in per_scene_iou if math.isfinite(row["rgb_ndwi_iou_coastal_roi"])]
    site_median_iou = float(np.median(valid_ious)) if valid_ious else float("nan")
    rgb_pass = math.isfinite(site_median_iou) and site_median_iou >= RGB_IOU_KILL
    chosen_method_count = {"RGB": 0, "NDWI": 0}
    for record, scene_metrics in zip(scene_records, per_scene_iou):
        scene_iou = scene_metrics["rgb_ndwi_iou_coastal_roi"]
        scene_rgb_pass = math.isfinite(scene_iou) and scene_iou >= RGB_IOU_KILL
        method = "RGB" if scene_rgb_pass else "NDWI"
        record["chosen_water"] = record["rgb_water"] if scene_rgb_pass else record["ndwi_water"]
        record.pop("arrays")
        record["water_mask_method"] = method
        scene_metrics["selected_water_mask"] = method
        chosen_method_count[method] += 1
    chosen_method = f"per-scene RGB={chosen_method_count['RGB']}, NDWI fallback={chosen_method_count['NDWI']}"
    coast_result = match_feature(scene_records, coast_mask, map_transform, basis, seed=len(site["name"]))
    structures_result = None
    if site["name"] in {"changhua_mudflats", "taichung_port"}:
        structures_result = match_feature(
            scene_records, structure_mask, map_transform, basis, seed=len(site["name"]) + 13
        )
    coast_result["map_feature"] = "natural=coastline"
    coast_result["fix_verdict"] = fix_verdict(coast_result)
    if structures_result is not None:
        structures_result["map_feature"] = "man_made=breakwater|groyne"
        structures_result["fix_verdict"] = fix_verdict(structures_result)
    segmentation_verdict = "RGB_IOU_PASS" if rgb_pass else "NDWI_FALLBACK"
    verdict = f"{segmentation_verdict}; coastline={coast_result['fix_verdict']}"
    report = {
        "label": "MEASURED",
        "site": site["name"],
        "site_center_wgs84": [site["lon"], site["lat"]],
        "stac_collection": "sentinel-2-l2a",
        "selected_scene_count": len(scenes),
        "all_selected_scenes_below_10_percent_cloud": all(
            float(scene["properties"]["eo:cloud_cover"]) < 10 for scene in scenes
        ),
        "scene_dates": [scene["properties"]["datetime"] for scene in scenes],
        "site_rgb_ndwi_iou_median": site_median_iou if math.isfinite(site_median_iou) else None,
        "rgb_iou_threshold": RGB_IOU_KILL,
        "rgb_iou_pass": rgb_pass,
        "water_mask_used_for_fixes": chosen_method,
        "rgb_iou_verdict": segmentation_verdict,
        "coastline_fix_verdict": coast_result["fix_verdict"],
        "per_scene_iou": per_scene_iou,
        "coastline_local_basis": basis,
        "osm_coastline_ways": len(coast_lines),
        "osm_coastline_nodes": coast_nodes,
        "osm_breakwater_groyne_ways": len(structure_lines),
        "osm_cache": osm_path,
        "coastline_fix": coast_result,
        "breakwater_groyne_fix": structures_result,
        "limits": [
            "Sentinel-2 is a 10 m satellite proxy, not drone-camera imagery.",
            "Translation errors are relative to Sentinel georeferencing and OSM; neither is ground-truth navigation truth.",
            "Three dates per site give only a small multi-tide sample; scene-date tide observations were unavailable.",
        ],
    }
    print(
        f"  {site['name']}: scenes={len(scenes)} IoU median="
        f"{site_median_iou:.3f} mask={chosen_method} "
        f"coast accepted={coast_result.get('error_distribution_all_scene_fixes', {}).get('accepted_n', 0)}"
        f"/{coast_result.get('error_distribution_all_scene_fixes', {}).get('n', 0)} "
        f"false_accepts={coast_result.get('false_accepts', 'n/a')}/"
        f"{coast_result.get('test_negative_attempted', 0)}",
        flush=True,
    )
    return report


def mudflat_tide_explanation(
    site_results: list[dict[str, Any]], tide_access: dict[str, Any]
) -> dict[str, Any]:
    changhua = next(
        (site for site in site_results if site.get("site") == "changhua_mudflats"), {}
    )
    matches = changhua.get("coastline_fix", {}).get("positive_matches", [])
    return {
        "label": "INFERENCE",
        "site": "changhua_mudflats",
        "measured_scene_dates": [item.get("date") for item in matches],
        "measured_cross_shore_candidate_errors_m": [
            item.get("cross_shore_error_m") for item in matches
        ],
        "measured_along_shore_candidate_errors_m": [
            item.get("along_shore_error_m") for item in matches
        ],
        "measured_accepted_coastline_fixes": changhua.get("coastline_fix", {}).get(
            "error_distribution_accepted_scene_fixes", {}
        ).get("accepted_n", 0),
        "scene_date_tide_data_available": tide_access.get(
            "scene_date_tide_levels_available", False
        ),
        "published_cwa_next_30_day_fuxing_prediction": tide_access.get(
            "fuxing_30day_prediction"
        ),
        "explanation": (
            "An intertidal mudflat has no fixed optical waterline: changing water level moves "
            "the wet/dry boundary horizontally, while OSM natural=coastline is a fixed mapped "
            "line. CWA's reachable next-30-day Fuxing prediction provides tidal context, but "
            "the historical API returned 401 and the archive download returned HTML, so these "
            "scene offsets cannot be correlated with scene-time tide or converted using a "
            "measured local mudflat slope. Tide is a plausible confounder, not a demonstrated "
            "cause of these measured offsets."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kill-only", action="store_true", help="run only the initial Changhua RGB-NDWI gate")
    parser.add_argument("--scenes-per-site", type=int, default=3)
    args = parser.parse_args()
    RAW.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    if args.kill_only:
        result = run_kill_test()
        return 2 if result["decision"] == "KILL" else 0

    results = []
    for site in SITES:
        report = site_experiment(site, args.scenes_per_site)
        results.append(report)
        write_json(OUT / f"site_{site['name']}.json", report)
        write_json(OUT / "site_results.partial.json", {"label": "MEASURED", "sites": results})
    tide = tide_access_check()
    mudflat_tide = mudflat_tide_explanation(results, tide)
    coastline_verdicts = [site.get("coastline_fix_verdict") for site in results]
    if all(verdict == "INCONCLUSIVE_NO_POSITIVE_FIX_ACCEPTED" for verdict in coastline_verdicts):
        overall_verdict = "INCONCLUSIVE_NO_ACCEPTED_COASTLINE_FIX"
    elif all(verdict == "PASS" for verdict in coastline_verdicts):
        overall_verdict = "PASS"
    else:
        overall_verdict = "FAIL_SITE_CRITERIA_NOT_MET"
    payload = {
        "label": "MEASURED",
        "overall_verdict": overall_verdict,
        "experiment": "X3 coastline fix after sea crossing, Sentinel-2 L2A proxy",
        "method": {
            "rgb_mask": "B > 1.05*R, G > 1.02*R, max(R,G,B) < 220",
            "ndwi_mask": "(green - nir)/(green + nir) > 0",
            "rgb_iou_gate": RGB_IOU_KILL,
            "water_mask_fallback": "Use NDWI on any date whose coastal RGB-NDWI IoU is below 0.70.",
            "map_features": ["natural=coastline", "man_made=breakwater|groyne"],
            "search": {"radius_m": WINDOW_M, "resolution_m": PIXEL_M},
            "negative_protocol": "30 calibration priors at 3-10 km; threshold is strictly above their maximum score; 30 distinct shifted test priors at 3-10 km",
            "false_accept_score": "negative mean chamfer distance; higher is better",
            "osm_attribution": "© OpenStreetMap contributors, ODbL 1.0",
            "sentinel_source": "Earth Search STAC sentinel-2-l2a public COGs; only small windows read via /vsicurl range requests",
        },
        "sites": results,
        "tide_access": tide,
        "mudflat_tide_explanation": mudflat_tide,
        "overall_limits": [
            "Satellite geolocation and OSM are comparison references, not independent ground truth.",
            "10 m pixels limit shoreline localization; along-shore ambiguity is expected on straight coasts.",
            "CWA historical tide levels for Sentinel scene timestamps were unavailable; the reachable next-30-day forecast is context only.",
        ],
    }
    result_path = OUT / "site_results.json"
    write_json(result_path, payload)
    print(f"saved {result_path.relative_to(ROOT)}")
    print(
        "tide archive: historical API="
        f"{tide.get('historical_api_http_status', tide.get('historical_api_error'))}; "
        f"downloaded={tide.get('historical_csv_downloaded', False)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

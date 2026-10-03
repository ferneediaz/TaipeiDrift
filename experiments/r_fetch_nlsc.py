#!/usr/bin/env python3
"""Inspect NLSC WMTS metadata and build a small, local research sample.

The ordinary WMTS terms prohibit bulk downloads and bulk caching. This
research-only run is deliberately bounded to 600 tile responses total and
one request per second. Offline retention is not expressly licensed; see
``data/raw/aerial_pairs/nlsc_rights.md`` and obtain NLSC confirmation before
expanding or redistributing these data.

Run from the repository root:
    .venv/bin/python experiments/r_fetch_nlsc.py

Use ``--inspect-only`` to print cached orthophoto layer metadata without
requesting imagery. Existing tile responses are reused and never refetched.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import date, datetime, timezone
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rasterio
from affine import Affine
from pyproj import Transformer
from rasterio.enums import Resampling
from rasterio.warp import reproject

ROOT = Path(__file__).resolve().parents[1]
PAIR_DIR = ROOT / "data" / "raw" / "aerial_pairs"
CACHE_DIR = ROOT / "data" / "raw" / "nlsc_tiles"
CAPABILITIES = CACHE_DIR / "capabilities.xml"
MANIFEST = PAIR_DIR / "manifest_nlsc.json"
RIGHTS_URL = "https://maps.nlsc.gov.tw/pro/use_clause.jsp"
FAQ_URL = "https://maps.nlsc.gov.tw/S09SOA/pro/faq_ajax_list.jsp"
LICENSE_URL = "https://data.gov.tw/license"
CAPABILITIES_URL = (
    "https://wmts.nlsc.gov.tw/wmts?SERVICE=WMTS&REQUEST=GetCapabilities"
)
USER_AGENT = (
    "TaipeiDrift-r_fetch_nlsc/1.0 "
    "(GNSS-denied aerial-map research; local limited sample)"
)
MAX_TILES = 600
MIN_REQUEST_INTERVAL_S = 1.0
ZOOM = "17"
TARGET_CRS = "EPSG:3826"
SOURCE_CRS = "EPSG:3857"
SITE_SIZE_M = 1600
TILE_SIZE = 256

SITES = [
    {
        "site_id": "nlsc_taipei_urban",
        "land_cover": "urban",
        "lat": 25.026,
        "lon": 121.543,
    },
    {
        "site_id": "nlsc_changhua_rice",
        "land_cover": "rice_paddy",
        "lat": 23.95,
        "lon": 120.45,
    },
    {
        "site_id": "nlsc_zhuoshui_river",
        "land_cover": "river",
        "lat": 23.83,
        "lon": 120.45,
    },
    {
        "site_id": "nlsc_guanyin_coast",
        "land_cover": "coast",
        "lat": 25.05,
        "lon": 121.08,
    },
    {
        "site_id": "nlsc_nantou_hills",
        "land_cover": "hills_forest",
        "lat": 23.85,
        "lon": 120.85,
    },
    {
        "site_id": "nlsc_kaohsiung_port",
        "land_cover": "industrial",
        "lat": 22.60,
        "lon": 120.29,
    },
]
PREFERRED_YEARS = (2015, 2023)


class FetchStopped(RuntimeError):
    """A polite stop caused by a service response or configured limit."""


class NoCoverage(RuntimeError):
    """A layer/year has no non-uniform imagery at this site."""


@dataclass(frozen=True)
class Layer:
    identifier: str
    title: str
    style: str
    matrix_sets: tuple[str, ...]
    templates: tuple[str, ...]
    formats: tuple[str, ...]


@dataclass(frozen=True)
class Matrix:
    identifier: str
    scale_denominator: float
    top_left_x: float
    top_left_y: float
    tile_width: int
    tile_height: int
    matrix_width: int
    matrix_height: int

    @property
    def resolution(self) -> float:
        return self.scale_denominator * 0.00028

    @property
    def tile_span_x(self) -> float:
        return self.resolution * self.tile_width

    @property
    def tile_span_y(self) -> float:
        return self.resolution * self.tile_height


class TileClient:
    def __init__(self, max_tiles: int) -> None:
        self.max_tiles = max_tiles
        self.last_request_started: float | None = None
        self.cache_entries = sum(
            1
            for path in CACHE_DIR.rglob("*")
            if path.is_file() and (path.suffix == ".jpg" or path.name.endswith(".http.json"))
        )
        self.network_requests = 0
        self.network_images = 0

    def _wait_for_slot(self) -> None:
        if self.last_request_started is not None:
            delay = MIN_REQUEST_INTERVAL_S - (time.monotonic() - self.last_request_started)
            if delay > 0:
                time.sleep(delay)
        self.last_request_started = time.monotonic()

    @staticmethod
    def _paths(layer: str, row: int, col: int) -> tuple[Path, Path]:
        folder = CACHE_DIR / layer / ZOOM
        image_path = folder / f"{row}_{col}.jpg"
        status_path = folder / f"{row}_{col}.http.json"
        return image_path, status_path

    def get(self, layer: Layer, year_layer: str, row: int, col: int) -> tuple[bytes | None, int | None, bool]:
        image_path, status_path = self._paths(year_layer, row, col)
        if image_path.exists():
            return image_path.read_bytes(), 200, True
        if status_path.exists():
            status = json.loads(status_path.read_text(encoding="utf-8"))
            return None, status.get("http_status"), True
        if self.cache_entries >= self.max_tiles:
            raise FetchStopped(
                f"tile cache cap reached ({self.cache_entries}/{self.max_tiles}); stopping without more requests"
            )
        if not layer.templates:
            raise FetchStopped(f"no ResourceURL for layer {year_layer}")
        template = layer.templates[0]
        url = template.format(
            Style=layer.style,
            TileMatrixSet="GoogleMapsCompatible",
            TileMatrix=ZOOM,
            TileRow=row,
            TileCol=col,
        )
        request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        self._wait_for_slot()
        self.network_requests += 1
        try:
            with urllib.request.urlopen(request, timeout=45) as response:
                status_code = response.status
                payload = response.read()
        except urllib.error.HTTPError as exc:
            status_code = exc.code
            payload = b""
            self._save_status(status_path, status_code, f"HTTP {status_code}")
            if status_code in (401, 403, 407, 429, 451):
                raise FetchStopped(f"NLSC blocked/rate-limited tile request: HTTP {status_code}") from exc
            if status_code == 404:
                return None, status_code, False
            raise FetchStopped(f"NLSC tile request failed: HTTP {status_code}") from exc
        except (TimeoutError, urllib.error.URLError, OSError) as exc:
            self._save_status(status_path, None, repr(exc))
            raise FetchStopped(f"NLSC tile request failed before an HTTP response: {exc!r}") from exc
        if status_code in (401, 403, 407, 429, 451):
            self._save_status(status_path, status_code, f"HTTP {status_code}")
            raise FetchStopped(f"NLSC blocked/rate-limited tile request: HTTP {status_code}")
        if status_code != 200:
            self._save_status(status_path, status_code, f"HTTP {status_code}")
            raise FetchStopped(f"NLSC tile request failed: HTTP {status_code}")
        decoded = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is None or decoded.shape[:2] != (TILE_SIZE, TILE_SIZE):
            self._save_status(status_path, status_code, "HTTP 200 response was not a 256x256 image")
            raise FetchStopped(
                f"NLSC returned HTTP 200 but not a readable 256x256 tile for {year_layer}/{row}/{col}"
            )
        image_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with image_path.open("xb") as stream:
                stream.write(payload)
        except FileExistsError:
            return image_path.read_bytes(), 200, True
        self.cache_entries += 1
        self.network_images += 1
        return payload, 200, False

    def _save_status(self, path: Path, status_code: int | None, detail: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        record = {
            "http_status": status_code,
            "detail": detail,
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        try:
            with path.open("x", encoding="utf-8") as stream:
                json.dump(record, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
        except FileExistsError:
            pass
        self.cache_entries += 1


def local_name(element: ET.Element) -> str:
    return element.tag.rsplit("}", 1)[-1]


def first_child_text(element: ET.Element, name: str, default: str = "") -> str:
    for child in element.iter():
        if local_name(child) == name and child.text:
            return child.text.strip()
    return default


def child_text_direct(element: ET.Element, name: str, default: str = "") -> str:
    for child in element:
        if local_name(child) == name and child.text:
            return child.text.strip()
    return default


def get_capabilities(client: TileClient) -> ET.Element:
    if not CAPABILITIES.exists():
        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        request = urllib.request.Request(CAPABILITIES_URL, headers={"User-Agent": USER_AGENT})
        client._wait_for_slot()
        client.network_requests += 1
        try:
            with urllib.request.urlopen(request, timeout=45) as response:
                if response.status != 200:
                    raise FetchStopped(f"GetCapabilities failed: HTTP {response.status}")
                payload = response.read()
        except urllib.error.HTTPError as exc:
            raise FetchStopped(f"GetCapabilities failed: HTTP {exc.code}") from exc
        CAPABILITIES.write_bytes(payload)
    return ET.parse(CAPABILITIES).getroot()


def parse_capabilities(root: ET.Element) -> tuple[dict[str, Layer], dict[str, Matrix]]:
    layers: dict[str, Layer] = {}
    for element in root.iter():
        if local_name(element) != "Layer":
            continue
        identifier = first_child_text(element, "Identifier")
        if not identifier.startswith("PHOTO"):
            continue
        styles: list[tuple[bool, str]] = []
        matrix_sets: list[str] = []
        templates: list[str] = []
        formats: list[str] = []
        for child in element:
            kind = local_name(child)
            if kind == "Style":
                styles.append((child.attrib.get("isDefault", "false").lower() == "true", first_child_text(child, "Identifier", "default")))
            elif kind == "Format" and child.text:
                formats.append(child.text.strip())
            elif kind == "TileMatrixSetLink":
                matrix_set = first_child_text(child, "TileMatrixSet")
                if matrix_set:
                    matrix_sets.append(matrix_set)
            elif kind == "ResourceURL" and child.attrib.get("resourceType") == "tile":
                templates.append(child.attrib.get("template", ""))
        title = first_child_text(element, "Title", identifier)
        layers[identifier] = Layer(
            identifier=identifier,
            title=title,
            style=next((style for is_default, style in styles if is_default), styles[0][1] if styles else "default"),
            matrix_sets=tuple(dict.fromkeys(matrix_sets)),
            templates=tuple(template for template in templates if template),
            formats=tuple(dict.fromkeys(formats)),
        )
    matrices: dict[str, Matrix] = {}
    for matrix_set in root.iter():
        if local_name(matrix_set) != "TileMatrixSet":
            continue
        set_id = first_child_text(matrix_set, "Identifier")
        for matrix_element in matrix_set:
            if local_name(matrix_element) != "TileMatrix":
                continue
            identifier = first_child_text(matrix_element, "Identifier")
            if not identifier:
                continue
            corners = child_text_direct(matrix_element, "TopLeftCorner").split()
            if len(corners) != 2:
                continue
            matrices[f"{set_id}/{identifier}"] = Matrix(
                identifier=identifier,
                scale_denominator=float(first_child_text(matrix_element, "ScaleDenominator")),
                top_left_x=float(corners[0]),
                top_left_y=float(corners[1]),
                tile_width=int(first_child_text(matrix_element, "TileWidth")),
                tile_height=int(first_child_text(matrix_element, "TileHeight")),
                matrix_width=int(first_child_text(matrix_element, "MatrixWidth")),
                matrix_height=int(first_child_text(matrix_element, "MatrixHeight")),
            )
    return layers, matrices


def print_layer_summary(layers: dict[str, Layer], matrices: dict[str, Matrix]) -> None:
    orthos = sorted(layers, key=lambda name: (0 if name == "PHOTO1" else 1, name))
    sets = sorted({matrix_set for layer in layers.values() for matrix_set in layer.matrix_sets})
    normalized_templates = {
        template.replace(f"/{layer.identifier}/", "/{PHOTO_LAYER}/")
        for layer in layers.values()
        for template in layer.templates
    }
    layer_text = ", ".join(orthos)
    resolution = matrices.get("GoogleMapsCompatible/17")
    gsd_text = f"; zoom 17={resolution.resolution:.3f} m/px at EPSG:3857" if resolution else ""
    print(f"Orthophoto layers ({len(orthos)}): {layer_text}")
    print(f"Tile matrix set(s): {', '.join(sets)}{gsd_text}")
    print(f"ResourceURL template(s): {'; '.join(sorted(normalized_templates))}")


def is_blank_tile(image: np.ndarray) -> bool:
    values = image.reshape(-1, 3)
    low = np.percentile(values, 1, axis=0)
    high = np.percentile(values, 99, axis=0)
    return bool(np.max(high - low) <= 4)


def site_grid(site: dict[str, Any]) -> tuple[tuple[int, int, int, int], Affine, tuple[int, int]]:
    to_target = Transformer.from_crs("EPSG:4326", TARGET_CRS, always_xy=True)
    center_x, center_y = to_target.transform(site["lon"], site["lat"])
    left = math.floor(center_x - SITE_SIZE_M / 2)
    top = math.ceil(center_y + SITE_SIZE_M / 2)
    width = SITE_SIZE_M
    height = SITE_SIZE_M
    transform = Affine(1.0, 0.0, left, 0.0, -1.0, top)
    return (left, top - height, left + width, top), transform, (width, height)


def tile_extent(site: dict[str, Any], matrix: Matrix) -> list[tuple[int, int]]:
    bounds, _, _ = site_grid(site)
    left, bottom, right, top = bounds
    to_3857 = Transformer.from_crs(TARGET_CRS, SOURCE_CRS, always_xy=True)
    corners = [
        to_3857.transform(left, bottom),
        to_3857.transform(left, top),
        to_3857.transform(right, bottom),
        to_3857.transform(right, top),
    ]
    min_x = min(point[0] for point in corners)
    max_x = max(point[0] for point in corners)
    min_y = min(point[1] for point in corners)
    max_y = max(point[1] for point in corners)
    tile_span_x, tile_span_y = matrix.tile_span_x, matrix.tile_span_y
    min_col = math.floor((min_x - matrix.top_left_x) / tile_span_x)
    max_col = math.floor((math.nextafter(max_x, -math.inf) - matrix.top_left_x) / tile_span_x)
    min_row = math.floor((matrix.top_left_y - max_y) / tile_span_y)
    max_row = math.floor((matrix.top_left_y - math.nextafter(min_y, math.inf)) / tile_span_y)
    if min_row < 0 or min_col < 0 or max_row >= matrix.matrix_height or max_col >= matrix.matrix_width:
        raise FetchStopped(f"site {site['site_id']} falls outside zoom-{ZOOM} matrix bounds")
    return [(row, col) for row in range(min_row, max_row + 1) for col in range(min_col, max_col + 1)]


def center_tile(site: dict[str, Any], matrix: Matrix) -> tuple[int, int]:
    to_3857 = Transformer.from_crs("EPSG:4326", SOURCE_CRS, always_xy=True)
    x, y = to_3857.transform(site["lon"], site["lat"])
    col = math.floor((x - matrix.top_left_x) / matrix.tile_span_x)
    row = math.floor((matrix.top_left_y - y) / matrix.tile_span_y)
    return row, col


def decode_tile(payload: bytes | None) -> np.ndarray | None:
    if payload is None:
        return None
    bgr = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def probe_year(
    client: TileClient,
    layer: Layer,
    year: int,
    site: dict[str, Any],
    matrix: Matrix,
) -> bool:
    row, col = center_tile(site, matrix)
    payload, status, _ = client.get(layer, layer.identifier, row, col)
    image = decode_tile(payload)
    return image is not None and not is_blank_tile(image)


def create_mosaic(
    client: TileClient,
    layer: Layer,
    year: int,
    site: dict[str, Any],
    matrix: Matrix,
) -> tuple[np.ndarray, Affine, float]:
    tile_ids = tile_extent(site, matrix)
    rows = [row for row, _ in tile_ids]
    cols = [col for _, col in tile_ids]
    min_row, max_row, min_col, max_col = min(rows), max(rows), min(cols), max(cols)
    mosaic = np.zeros(
        ((max_row - min_row + 1) * matrix.tile_height,
         (max_col - min_col + 1) * matrix.tile_width,
         3),
        dtype=np.uint8,
    )
    valid_tiles = 0
    for row, col in tile_ids:
        payload, status, _ = client.get(layer, layer.identifier, row, col)
        image = decode_tile(payload)
        if image is None or is_blank_tile(image):
            continue
        valid_tiles += 1
        y0 = (row - min_row) * matrix.tile_height
        x0 = (col - min_col) * matrix.tile_width
        mosaic[y0 : y0 + matrix.tile_height, x0 : x0 + matrix.tile_width] = image
    if valid_tiles == 0:
        raise NoCoverage(f"{layer.identifier}: all {len(tile_ids)} requested tiles are blank or unavailable")
    source_transform = Affine(
        matrix.resolution,
        0.0,
        matrix.top_left_x + min_col * matrix.tile_span_x,
        0.0,
        -matrix.resolution,
        matrix.top_left_y - min_row * matrix.tile_span_y,
    )
    bounds, target_transform, (width, height) = site_grid(site)
    destination = np.zeros((3, height, width), dtype=np.uint8)
    reproject(
        source=mosaic.transpose(2, 0, 1),
        destination=destination,
        src_transform=source_transform,
        src_crs=SOURCE_CRS,
        src_nodata=0,
        dst_transform=target_transform,
        dst_crs=TARGET_CRS,
        dst_nodata=0,
        resampling=Resampling.average,
        num_threads=1,
    )
    valid_fraction = float(np.any(destination != 0, axis=0).mean())
    if valid_fraction <= 0:
        raise NoCoverage(f"{layer.identifier}: reprojection produced no valid pixels")
    return destination, target_transform, valid_fraction


def save_raster_and_preview(
    site: dict[str, Any], year: int, array: np.ndarray, transform: Affine
) -> tuple[Path, Path, int, int, float]:
    output_dir = PAIR_DIR / site["site_id"]
    output_dir.mkdir(parents=True, exist_ok=True)
    raster_path = output_dir / f"{year}_nlsc.tif"
    preview_path = output_dir / f"preview_{year}.png"
    height, width = array.shape[1:]
    if raster_path.exists():
        with rasterio.open(raster_path) as dataset:
            valid_fraction = float(np.any(dataset.read() != 0, axis=0).mean())
            width, height = dataset.width, dataset.height
            existing = dataset.read()
        if not preview_path.exists():
            preview_rgb = cv2.resize(existing.transpose(1, 2, 0), (512, 512), interpolation=cv2.INTER_AREA)
            cv2.imwrite(str(preview_path), cv2.cvtColor(preview_rgb, cv2.COLOR_RGB2BGR))
        return raster_path, preview_path, width, height, valid_fraction
    profile = {
        "driver": "GTiff",
        "width": width,
        "height": height,
        "count": 3,
        "dtype": "uint8",
        "crs": TARGET_CRS,
        "transform": transform,
        "nodata": 0,
        "compress": "deflate",
        "predictor": 2,
        "tiled": True,
        "blockxsize": 256,
        "blockysize": 256,
        "interleave": "pixel",
    }
    with rasterio.open(raster_path, "w", **profile) as dataset:
        dataset.colorinterp = (rasterio.enums.ColorInterp.red, rasterio.enums.ColorInterp.green, rasterio.enums.ColorInterp.blue)
        dataset.write(array)
    preview_rgb = cv2.resize(array.transpose(1, 2, 0), (512, 512), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(preview_path), cv2.cvtColor(preview_rgb, cv2.COLOR_RGB2BGR))
    return raster_path, preview_path, width, height, float(np.any(array != 0, axis=0).mean())



def manifest_record(
    site: dict[str, Any], year: int, layer: Layer, raster_path: Path,
    width: int, height: int, valid_fraction: float,
) -> dict[str, Any]:
    template = layer.templates[0] if layer.templates else ""
    return {
        "site_id": site["site_id"],
        "land_cover": site["land_cover"],
        "lat": site["lat"],
        "lon": site["lon"],
        "date": str(year),
        "source": "nlsc",
        "source_url_or_layer": f"{layer.identifier}: {template}",
        "native_gsd_m": 0.25,
        "licence": "NLSC WMTS service terms; offline retention/use UNCLEAR; research-only, no redistribution",
        "licence_url": RIGHTS_URL,
        "attribution": (
            f"Recommended (not prescribed): Source: National Land Surveying and Mapping Center "
            f"(NLSC), Taiwan, {layer.identifier} WMTS; accessed {date.today().isoformat()}."
        ),
        "file": raster_path.relative_to(ROOT).as_posix(),
        "width": width,
        "height": height,
        "valid_fraction": round(valid_fraction, 6),
        "notes": (
            f"{layer.title} ({layer.identifier}), layer year {year}; zoom {ZOOM} "
            f"GoogleMapsCompatible source pixels are {_ZOOM17_RESOLUTION:.3f} m/px at the equator. "
            "NLSC describes 0.25 m as the normal ground pixel size for local orthophoto updates "
            "(https://www.nlsc.gov.tw/cp.aspx?n=13659); actual per-site/year GSD and capture date "
            "are not stated in capabilities. At 512x512, no overlay was visible in Taipei urban 2015 "
            "or 2023; faint repeated Chinese text over water was illegible in Guanyin coast 2023. "
            "No standalone logo was identified; other previews were not individually inspected. "
            "See nlsc_rights.md."
        ),
    }




_ZOOM17_RESOLUTION = 559082264.0287178 / (2**17) * 0.00028


def write_manifest(records: list[dict[str, Any]]) -> None:
    PAIR_DIR.mkdir(parents=True, exist_ok=True)
    records.sort(key=lambda item: (item["site_id"], item["date"]))
    MANIFEST.write_text(json.dumps(records, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def candidate_years(available: set[int], accepted: list[int], tried: set[int]) -> list[int]:
    remaining = available - tried
    if len(accepted) == 1:
        anchor = accepted[0]
        return sorted(remaining, key=lambda year: (-abs(year - anchor), -year))
    return sorted(remaining, key=lambda year: (-abs(year - 2019), -year))


def fetch_site_year(
    site: dict[str, Any],
    year: int,
    layers: dict[str, Layer],
    matrix: Matrix,
    client: TileClient,
    max_tiles: int,
) -> dict[str, Any] | None:
    layer = layers.get(f"PHOTO{year}")
    if layer is None:
        return None
    if not probe_year(client, layer, year, site, matrix):
        print(f"{site['site_id']} {year}: center tile blank; trying another year", flush=True)
        return None
    try:
        print(f"{site['site_id']} {year}: mosaicking zoom-{ZOOM} tiles", flush=True)
        array, transform, valid_fraction = create_mosaic(client, layer, year, site, matrix)
    except NoCoverage as exc:
        print(f"{site['site_id']} {year}: {exc}; trying another year", flush=True)
        return None
    raster_path, preview_path, width, height, valid_fraction = save_raster_and_preview(
        site, year, array, transform
    )
    record = manifest_record(site, year, layer, raster_path, width, height, valid_fraction)
    print(
        f"  saved {raster_path.relative_to(ROOT)}; {width}x{height}; "
        f"valid_fraction={valid_fraction:.6f}; preview={preview_path.relative_to(ROOT)}",
        flush=True,
    )
    return record


def run_fetch(layers: dict[str, Layer], matrices: dict[str, Matrix], client: TileClient, max_tiles: int) -> list[dict[str, Any]]:
    matrix = matrices.get("GoogleMapsCompatible/17")
    if matrix is None:
        raise FetchStopped("capabilities do not contain GoogleMapsCompatible zoom 17")
    available_years = {
        int(identifier.removeprefix("PHOTO"))
        for identifier in layers
        if identifier.startswith("PHOTO") and identifier.removeprefix("PHOTO").isdigit()
    }
    if not available_years:
        raise FetchStopped("capabilities contain no PHOTOYYYY year layers")
    records: list[dict[str, Any]] = []
    for site in SITES:
        accepted: list[int] = []
        tried: set[int] = set()
        for year in PREFERRED_YEARS:
            if year not in available_years:
                continue
            tried.add(year)
            record = fetch_site_year(site, year, layers, matrix, client, max_tiles)
            if record is not None:
                accepted.append(year)
                records.append(record)
            if len(accepted) == 2:
                break
        while len(accepted) < 2:
            options = candidate_years(available_years, accepted, tried)
            if not options:
                break
            year = options[0]
            tried.add(year)
            record = fetch_site_year(site, year, layers, matrix, client, max_tiles)
            if record is not None:
                accepted.append(year)
                records.append(record)
        if len(accepted) < 2:
            print(f"{site['site_id']}: only {len(accepted)} covered year(s) found", flush=True)
    return records


def records_from_existing(layers: dict[str, Layer]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for site in SITES:
        site_dir = PAIR_DIR / site["site_id"]
        if not site_dir.is_dir():
            continue
        for raster_path in sorted(site_dir.glob("*_nlsc.tif")):
            try:
                year = int(raster_path.stem.split("_", 1)[0])
            except ValueError:
                continue
            layer = layers.get(f"PHOTO{year}")
            if layer is None:
                continue
            with rasterio.open(raster_path) as dataset:
                fraction = float(np.any(dataset.read() != 0, axis=0).mean())
                records.append(manifest_record(site, year, layer, raster_path, dataset.width, dataset.height, fraction))
    return records

def verify_outputs(records: list[dict[str, Any]]) -> None:
    by_site: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        by_site.setdefault(record["site_id"], []).append(record)
    expected_crs = rasterio.crs.CRS.from_string(TARGET_CRS)
    for site_id, site_records in sorted(by_site.items()):
        reference_grid: tuple[Any, ...] | None = None
        for record in sorted(site_records, key=lambda item: item["date"]):
            raster_path = ROOT / record["file"]
            with rasterio.open(raster_path) as dataset:
                if (
                    dataset.crs != expected_crs
                    or dataset.width != SITE_SIZE_M
                    or dataset.height != SITE_SIZE_M
                    or dataset.count != 3
                    or dataset.dtypes != ("uint8", "uint8", "uint8")
                    or dataset.nodata != 0
                    or dataset.compression is None
                    or dataset.compression.value.lower() != "deflate"
                ):
                    raise FetchStopped(f"{raster_path}: raster contract check failed")
                grid = (dataset.crs, dataset.width, dataset.height, tuple(dataset.transform))
                if reference_grid is not None and grid != reference_grid:
                    raise FetchStopped(f"{site_id}: yearly rasters do not share an identical grid")
                reference_grid = grid
                pixels = dataset.read()
                valid_fraction = float(np.any(pixels != 0, axis=0).mean())
                preview = raster_path.with_name(f"preview_{record['date']}.png")
                preview_image = cv2.imread(str(preview), cv2.IMREAD_UNCHANGED)
                if preview_image is None or preview_image.shape[:2] != (512, 512):
                    raise FetchStopped(f"{preview}: missing or not 512x512")
                print(
                    f"VERIFY {site_id} {record['date']} {dataset.width}x{dataset.height} "
                    f"transform={dataset.transform} valid_fraction={valid_fraction:.6f}",
                    flush=True,
                )
        if len(site_records) > 1:
            print(f"VERIFY {site_id}: all years share one identical grid", flush=True)



def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspect-only", action="store_true", help="read cached capabilities and print a short layer summary")
    parser.add_argument("--max-tiles", type=int, default=MAX_TILES, help=f"total tile cache limit (1..{MAX_TILES})")
    args = parser.parse_args()
    if not 1 <= args.max_tiles <= MAX_TILES:
        parser.error(f"--max-tiles must be between 1 and {MAX_TILES}")
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    client = TileClient(args.max_tiles)
    try:
        root = get_capabilities(client)
        layers, matrices = parse_capabilities(root)
        print_layer_summary(layers, matrices)
        if args.inspect_only:
            return 0
        run_fetch(layers, matrices, client, args.max_tiles)
        records = records_from_existing(layers)
        verify_outputs(records)
        write_manifest(records)
    except FetchStopped as exc:
        records = records_from_existing(layers) if "layers" in locals() else []
        if records:
            try:
                verify_outputs(records)
            except FetchStopped as verification_error:
                print(f"VERIFY FAILED: {verification_error}", file=sys.stderr, flush=True)
        write_manifest(records)
        print(f"STOPPED: {exc}", file=sys.stderr, flush=True)
        print(
            f"Tile HTTP requests={client.network_requests}; successful image responses={client.network_images}; "
            f"cached tile results={client.cache_entries}/{args.max_tiles}",
            file=sys.stderr,
            flush=True,
        )
        return 2
    print(
        f"Complete: tile HTTP requests={client.network_requests}; successful image responses={client.network_images}; "
        f"cached tile results={client.cache_entries}/{args.max_tiles}; manifest={MANIFEST.relative_to(ROOT)}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""X8: Taiwan-wide terrain-relative-navigation and land-cover coverage map.

Run from any directory with `.venv/bin/python experiments/u8_taiwan_coverage.py`.

Evidence and assumptions
------------------------
Terrain observability is calculated from the real Copernicus DEM GLO-30 surface
model (DSM), not a bare-earth DTM. The assumed vertical/AGL measurement error is
sigma_h=4 m (barometer + stereo/laser), as specified for this experiment; it is
an assumption, not a measured Taiwan sensor result. For each square footprint,
RMS slope is sqrt(mean(gx**2 + gy**2)) and sigma_pos=sigma_h/RMS slope.
The 170/420/1000 m footprint stencils are nearest odd-sized windows on a 60 m
projected terrain grid. Results are resampled to a 150 m analysis grid.

"Map fix likely" is a land-cover proxy, not a tested image matcher: WorldCover
2021 v200 classes other than permanent water (80), tree cover (10), and
mangroves (95) are treated as textured land. TRN-informative means sigma_pos
<50 m for the 420 m footprint. The four 420 m combined classes are the
mutually-exclusive combinations of those two proxies.

Inputs and attribution
----------------------
- Copernicus DEM GLO-30 Public is free for public use under its Copernicus DEM
  licence: https://dataspace.copernicus.eu/explore-data/data-collections/copernicus-contributing-missions/collections-description/COP-DEM
  DSM tiles are read via GDAL range requests; no full DEM tiles are downloaded.
- ESA WorldCover 2021 v200, 10 m COGs, CC BY 4.0:
  https://esa-worldcover.org/en/data-access
  https://doi.org/10.5281/zenodo.7254221
  Attribution: © ESA WorldCover project 2021 / Contains modified Copernicus
  Sentinel data (2021) processed by ESA WorldCover consortium.
- Natural Earth 10m Admin-0 countries boundary, public domain:
  https://www.naturalearthdata.com/downloads/10m-cultural-vectors/10m-admin-0-countries/
  The largest Taiwan polygon is used as the main island; offshore islands are
  excluded. Raster fractions use 150 m UTM cell centres and equal-sized
  projected cells (area estimates are map-plane, not geodesic).
"""
from __future__ import annotations

import csv
import json
import math
import shutil
import urllib.request
import zipfile
from io import BytesIO
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.patches import Patch, Rectangle
import numpy as np
import rasterio
import shapefile
from pyproj import Transformer
from rasterio.enums import Resampling
from rasterio.features import geometry_mask
from rasterio.transform import from_origin
from rasterio.warp import reproject, transform_bounds
from rasterio.windows import Window, transform as window_transform
from rasterio.vrt import WarpedVRT
from scipy.ndimage import uniform_filter
from shapely.geometry import Polygon, box, mapping, shape
from shapely.ops import transform as transform_geometry

ROOT = Path(__file__).resolve().parents[1]
RAW_DIR = ROOT / "data" / "raw"
OUTPUT_DIR = ROOT / "data" / "processed" / "u8_taiwan_coverage"
BOUNDARY_PATH = RAW_DIR / "natural_earth" / "ne_10m_admin_0_countries.zip"
BOUNDARY_URL = "https://naturalearth.s3.amazonaws.com/10m_cultural/ne_10m_admin_0_countries.zip"
WORLD_COVER_ROOT = "https://esa-worldcover.s3.eu-central-1.amazonaws.com/v200/2021/map"
TARGET_CRS = "EPSG:32651"  # UTM zone 51N, Taiwan main island
ANALYSIS_RES_M = 150.0
DEM_RES_M = 60.0
DEM_NODATA = -9999.0
SIGMA_H_M = 4.0
TRN_LIMIT_M = 50.0
FOOTPRINTS_M = (170, 420, 1000)
DEM_PADDING_M = 1400.0

WORLD_COVER_CLASSES = {
    10: "tree_cover",
    20: "shrubland",
    30: "grassland",
    40: "cropland",
    50: "built_up",
    60: "bare_sparse_vegetation",
    70: "snow_ice",
    80: "permanent_water",
    90: "herbaceous_wetland",
    95: "mangroves",
    100: "moss_lichen",
}
MAP_FIX_CLASSES = {20, 30, 40, 50, 60, 70, 90, 100}
COVERAGE_CLASSES = {
    1: "neither (water/dense tree and TRN-flat proxy)",
    2: "map fix likely only",
    3: "TRN informative only",
    4: "both",
}

# Rectangular analysis extents in WGS84 (west, south, east, north). These are
# explicit study boxes, not claimed administrative metro boundaries.
METRO_BOXES = {
    "Taipei metro box": (121.30, 24.80, 121.75, 25.15),
    "Taichung metro box": (120.40, 24.00, 120.95, 24.40),
    "Kaohsiung metro box": (120.10, 22.40, 120.60, 22.90),
}


def fetch_if_missing(url: str, path: Path) -> None:
    """Fetch a small reference vector to data/raw, without leaving partial files."""
    if path.exists() and path.stat().st_size:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".part")
    request = urllib.request.Request(url, headers={"User-Agent": "TaipeiDrift-X8/1.0"})
    print(f"Fetching boundary: {url}")
    try:
        with urllib.request.urlopen(request, timeout=120) as response, temporary.open("wb") as dst:
            shutil.copyfileobj(response, dst)
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def largest_taiwan_polygon(path: Path) -> Polygon:
    """Return the largest polygon component of the Natural Earth Taiwan feature."""
    with zipfile.ZipFile(path) as archive:
        shp_member = next(name for name in archive.namelist() if name.endswith("ne_10m_admin_0_countries.shp"))
        stem = shp_member[:-4]
        parts = {Path(name).suffix.lower(): name for name in archive.namelist() if name.startswith(stem)}
        reader = shapefile.Reader(
            shp=BytesIO(archive.read(parts[".shp"])),
            shx=BytesIO(archive.read(parts[".shx"])),
            dbf=BytesIO(archive.read(parts[".dbf"])),
            encoding="utf-8",
        )
        field_names = [field[0] for field in reader.fields[1:]]
        taiwan = None
        for record in reader.iterShapeRecords():
            attributes = dict(zip(field_names, record.record))
            if attributes.get("ADMIN") == "Taiwan":
                taiwan = shape(record.shape.__geo_interface__)
                break
    if taiwan is None:
        raise RuntimeError("Natural Earth boundary has no ADMIN='Taiwan' feature")
    if not taiwan.is_valid:
        from shapely import make_valid
        taiwan = make_valid(taiwan)
    to_utm = Transformer.from_crs("EPSG:4326", TARGET_CRS, always_xy=True).transform
    components = list(taiwan.geoms) if hasattr(taiwan, "geoms") else [taiwan]
    projected = [transform_geometry(to_utm, component) for component in components if component.geom_type == "Polygon"]
    if not projected:
        raise RuntimeError("Taiwan boundary contains no polygon components")
    main_island = max(projected, key=lambda geometry: geometry.area)
    if main_island.is_empty or main_island.area < 1e9:
        raise RuntimeError("Largest Taiwan boundary component is implausibly small")
    return main_island


def make_grid(bounds: tuple[float, float, float, float], resolution: float, padding: float = 0.0):
    """Snap a north-up metric grid to resolution; bounds are left,bottom,right,top."""
    left = math.floor((bounds[0] - padding) / resolution) * resolution
    bottom = math.floor((bounds[1] - padding) / resolution) * resolution
    right = math.ceil((bounds[2] + padding) / resolution) * resolution
    top = math.ceil((bounds[3] + padding) / resolution) * resolution
    width = int(round((right - left) / resolution))
    height = int(round((top - bottom) / resolution))
    return from_origin(left, top, resolution, resolution), width, height


def copernicus_tile_url(lat: int, lon: int) -> str:
    tile = f"Copernicus_DSM_COG_10_N{lat:02d}_00_E{lon:03d}_00_DEM"
    return f"https://copernicus-dem-30m.s3.amazonaws.com/{tile}/{tile}.tif"


def candidate_dem_tiles(coverage_wgs84) -> list[tuple[int, int]]:
    west, south, east, north = coverage_wgs84.bounds
    tiles = []
    for lat in range(math.floor(south), math.ceil(north)):
        for lon in range(math.floor(west), math.ceil(east)):
            if coverage_wgs84.intersects(box(lon, lat, lon + 1, lat + 1)):
                tiles.append((lat, lon))
    return tiles


def load_dem_60m(main_island: Polygon):
    """Warp intersecting public Copernicus COGs into one padded 60 m UTM grid."""
    to_wgs84 = Transformer.from_crs(TARGET_CRS, "EPSG:4326", always_xy=True).transform
    island_wgs84 = transform_geometry(to_wgs84, main_island)
    coverage_wgs84 = transform_geometry(to_wgs84, main_island.buffer(DEM_PADDING_M))
    dem_transform, width, height = make_grid(main_island.bounds, DEM_RES_M, DEM_PADDING_M)
    dem = np.full((height, width), DEM_NODATA, dtype=np.float32)
    tiles = candidate_dem_tiles(coverage_wgs84)
    if not tiles:
        raise RuntimeError("No Copernicus DEM tiles intersect the padded Taiwan island")

    raster_env = {
        "GDAL_DISABLE_READDIR_ON_OPEN": "EMPTY_DIR",
        "CPL_VSIL_CURL_ALLOWED_EXTENSIONS": ".tif",
        "GDAL_HTTP_MAX_RETRY": "5",
        "GDAL_HTTP_RETRY_DELAY": "1",
        "GDAL_HTTP_TIMEOUT": "60",
        "GDAL_CACHEMAX": 128 * 1024 * 1024,
    }
    used_tiles = []
    with rasterio.Env(**raster_env):
        for lat, lon in tiles:
            url = copernicus_tile_url(lat, lon)
            vsi_url = "/vsicurl/" + url
            try:
                src = rasterio.open(vsi_url)
            except rasterio.errors.RasterioIOError as exc:
                if not island_wgs84.intersects(box(lon, lat, lon + 1, lat + 1)):
                    print(f"  Copernicus DEM N{lat:02d}E{lon:03d} absent outside island; treating padded water as 0 m")
                    continue
                raise RuntimeError(f"Cannot open required Copernicus DEM tile {url}: {exc}") from exc
            with src:
                left, bottom, right, top = transform_bounds(src.crs, TARGET_CRS, *src.bounds, densify_pts=21)
                col0 = max(0, int(math.floor((left - dem_transform.c) / DEM_RES_M)))
                col1 = min(width, int(math.ceil((right - dem_transform.c) / DEM_RES_M)))
                row0 = max(0, int(math.floor((dem_transform.f - top) / DEM_RES_M)))
                row1 = min(height, int(math.ceil((dem_transform.f - bottom) / DEM_RES_M)))
                if row1 <= row0 or col1 <= col0:
                    continue
                window = Window(col0, row0, col1 - col0, row1 - row0)
                reproject(
                    source=rasterio.band(src, 1),
                    destination=dem[row0:row1, col0:col1],
                    src_transform=src.transform,
                    src_crs=src.crs,
                    src_nodata=src.nodata,
                    dst_transform=window_transform(window, dem_transform),
                    dst_crs=TARGET_CRS,
                    dst_nodata=DEM_NODATA,
                    resampling=Resampling.bilinear,
                    init_dest_nodata=False,
                    num_threads=2,
                )
                used_tiles.append(f"N{lat:02d}E{lon:03d}")
                print(f"  Copernicus DEM {used_tiles[-1]}")
    land_cells = geometry_mask([mapping(main_island)], (height, width), dem_transform, invert=True)
    invalid_dem = (
        (dem == DEM_NODATA)
        | ~np.isfinite(dem)
        | (dem < -1000.0)
        | (dem > 9000.0)
    )
    missing_land = land_cells & invalid_dem
    if missing_land.any():
        raise RuntimeError(f"Copernicus DEM has no finite elevation for {int(missing_land.sum())} 60 m cells on the Taiwan island mask")
    # After verifying all mainland cells, treat remaining offshore nodata as
    # sea level so slope windows at the coast have finite input elevations.
    dem[invalid_dem] = 0.0
    return dem, dem_transform, used_tiles


def worldcover_tile_names(coverage_wgs84) -> list[str]:
    west, south, east, north = coverage_wgs84.bounds
    lat0 = math.floor(south / 3) * 3
    lon0 = math.floor(west / 3) * 3
    tiles = []
    for lat in range(lat0, math.ceil(north / 3) * 3, 3):
        for lon in range(lon0, math.ceil(east / 3) * 3, 3):
            if coverage_wgs84.intersects(box(lon, lat, lon + 3, lat + 3)):
                tiles.append(f"N{lat:02d}E{lon:03d}")
    return tiles


def load_worldcover(width: int, height: int, transform, main_island: Polygon) -> tuple[np.ndarray, list[str]]:
    """Majority-resample WorldCover 2021 v200 tiles to the 150 m UTM grid."""
    to_wgs84 = Transformer.from_crs(TARGET_CRS, "EPSG:4326", always_xy=True).transform
    coverage_wgs84 = transform_geometry(to_wgs84, main_island)
    tile_names = worldcover_tile_names(coverage_wgs84)
    classes = np.zeros((height, width), dtype=np.uint8)
    raster_env = {
        "GDAL_DISABLE_READDIR_ON_OPEN": "EMPTY_DIR",
        "CPL_VSIL_CURL_ALLOWED_EXTENSIONS": ".tif",
        "GDAL_HTTP_MAX_RETRY": "5",
        "GDAL_HTTP_RETRY_DELAY": "1",
        "GDAL_HTTP_TIMEOUT": "60",
        "GDAL_CACHEMAX": 128 * 1024 * 1024,
    }
    with rasterio.Env(**raster_env):
        for tile_name in tile_names:
            url = f"{WORLD_COVER_ROOT}/ESA_WorldCover_10m_2021_v200_{tile_name}_Map.tif"
            try:
                src = rasterio.open("/vsicurl/" + url)
            except rasterio.errors.RasterioIOError as exc:
                raise RuntimeError(f"Cannot open required ESA WorldCover tile {url}: {exc}") from exc
            with src:
                with WarpedVRT(
                    src,
                    crs=TARGET_CRS,
                    transform=transform,
                    width=width,
                    height=height,
                    resampling=Resampling.mode,
                    src_nodata=0,
                    nodata=0,
                ) as vrt:
                    tile_classes = vrt.read(1)
            valid = tile_classes != 0
            classes[valid] = tile_classes[valid]
            print(f"  WorldCover 2021 v200 {tile_name}")
    return classes, tile_names


def terrain_sigma_maps(dem: np.ndarray, dem_transform, analysis_shape: tuple[int, int], analysis_transform):
    """Calculate sigma_pos for all footprints without retaining full-size layers."""
    if not np.isfinite(dem).all():
        raise RuntimeError("Non-finite DEM values remain after the source-coverage check")
    dem = dem.astype(np.float64, copy=False)
    gx = np.gradient(dem, DEM_RES_M, axis=1)
    gy = np.gradient(dem, DEM_RES_M, axis=0)
    grad2 = gx * gx
    grad2 += gy * gy
    if not np.isfinite(grad2).all():
        raise RuntimeError("Non-finite terrain gradients remain after the DEM range check")
    del gx, gy, dem
    sigma_maps = {}
    effective = {}
    for footprint in FOOTPRINTS_M:
        kernel = max(1, int(round(footprint / DEM_RES_M)))
        if kernel % 2 == 0:
            kernel += 1
        effective[footprint] = kernel * DEM_RES_M
        local_rms = uniform_filter(grad2, size=kernel, mode="reflect", output=np.float64)
        # The RMS is non-negative by definition; remove any negative round-off
        # before sqrt so invalid DEM arithmetic cannot become a false sigma.
        np.maximum(local_rms, 0.0, out=local_rms)
        np.sqrt(local_rms, out=local_rms)
        rms_analysis = np.zeros(analysis_shape, dtype=np.float64)
        reproject(
            source=local_rms,
            destination=rms_analysis,
            src_transform=dem_transform,
            src_crs=TARGET_CRS,
            dst_transform=analysis_transform,
            dst_crs=TARGET_CRS,
            resampling=Resampling.bilinear,
            src_nodata=None,
            dst_nodata=0,
        )
        sigma = np.full(analysis_shape, np.inf, dtype=np.float32)
        representable_rms = np.isfinite(rms_analysis) & (rms_analysis >= SIGMA_H_M / np.finfo(np.float32).max)
        np.divide(SIGMA_H_M, rms_analysis, out=sigma, where=representable_rms)
        sigma_maps[footprint] = sigma
        del local_rms, rms_analysis
    del grad2
    return sigma_maps, effective


def metro_masks(width: int, height: int, transform, main_island: Polygon):
    to_utm = Transformer.from_crs("EPSG:4326", TARGET_CRS, always_xy=True).transform
    masks = {}
    for name, extent in METRO_BOXES.items():
        west, south, east, north = extent
        geometry = transform_geometry(to_utm, box(west, south, east, north)).intersection(main_island)
        masks[name] = geometry_mask([mapping(geometry)], (height, width), transform, invert=True)
    return masks


def calculate_summary(classes, sigma_maps, main_mask, coastal_mask, city_masks, resolution):
    """Return area-weighted-by-equal-UTM-cell fraction rows for all requested ROIs."""
    map_fix = np.isin(classes, tuple(MAP_FIX_CLASSES))
    trn = {fp: sigma < TRN_LIMIT_M for fp, sigma in sigma_maps.items()}
    coverage_420 = np.zeros(classes.shape, dtype=np.uint8)
    coverage_420[main_mask & ~map_fix & ~trn[420]] = 1
    coverage_420[main_mask & map_fix & ~trn[420]] = 2
    coverage_420[main_mask & ~map_fix & trn[420]] = 3
    coverage_420[main_mask & map_fix & trn[420]] = 4

    zones = {"Taiwan main island": main_mask, "20 km coastal strip": coastal_mask, **city_masks}
    rows: list[dict[str, Any]] = []
    cell_km2 = (resolution * resolution) / 1_000_000.0
    for scope, zone in zones.items():
        valid = zone & (classes != 0)
        denominator = int(np.count_nonzero(valid))
        if denominator == 0:
            raise RuntimeError(f"No valid analysis cells for {scope}")

        def append_row(metric: str, label: str, selected: np.ndarray) -> None:
            cells = int(np.count_nonzero(valid & selected))
            rows.append({
                "scope": scope,
                "metric": metric,
                "class": label,
                "cells": cells,
                "area_km2": cells * cell_km2,
                "fraction_pct": 100.0 * cells / denominator,
                "valid_denominator_cells": denominator,
            })

        append_row("map_fix_likely", "textured land-cover proxy", map_fix)
        for footprint, is_trn in trn.items():
            append_row(f"TRN_informative_sigma_pos_lt_50m_{footprint}m", "sigma_pos < 50 m", is_trn)
        append_row("both_at_420m", "map fix likely + TRN informative", map_fix & trn[420])
        append_row("map_fix_only_at_420m", "map fix likely only", map_fix & ~trn[420])
        append_row("TRN_only_at_420m", "TRN informative only", ~map_fix & trn[420])
        append_row("neither_at_420m", COVERAGE_CLASSES[1], ~map_fix & ~trn[420])
        for code, label in WORLD_COVER_CLASSES.items():
            append_row(f"WorldCover_{code}", label, classes == code)
    return rows, coverage_420


def write_outputs(classes, sigma_maps, coverage, island_mask, transform, rows, main_island, effective, dem_tiles, wc_tiles):
    height, width = classes.shape
    terrain_path = OUTPUT_DIR / "terrain_sigma_pos_150m.tif"
    classes_path = OUTPUT_DIR / "landcover_coverage_150m.tif"
    nodata = -9999.0
    sigma_bands = [sigma_maps[fp].copy() for fp in FOOTPRINTS_M]
    for band in sigma_bands:
        band[~island_mask] = nodata
    terrain_profile = {
        "driver": "GTiff",
        "height": height,
        "width": width,
        "count": 3,
        "dtype": "float32",
        "nodata": nodata,
        "crs": TARGET_CRS,
        "transform": transform,
        "tiled": True,
        "blockxsize": 256,
        "blockysize": 256,
        "compress": "deflate",
        "predictor": 3,
    }
    with rasterio.open(terrain_path, "w", **terrain_profile) as dst:
        for index, (footprint, band) in enumerate(zip(FOOTPRINTS_M, sigma_bands), start=1):
            dst.write(band, index)
            dst.set_band_description(index, f"sigma_pos at {footprint} m footprint, metres")
        dst.update_tags(
            sigma_h_m=str(SIGMA_H_M),
            terrain_formula="sigma_pos = sigma_h / RMS(sqrt(gx^2 + gy^2))",
            dem_source="Copernicus DEM GLO-30 DSM, public AWS COGs",
            analysis_resolution_m=str(ANALYSIS_RES_M),
            terrain_grid_resolution_m=str(DEM_RES_M),
            terrain_effective_footprints_m=json.dumps(effective, sort_keys=True),
            nodata_outside_island=str(nodata),
            dem_tiles=",".join(dem_tiles),
        )
    classes_out = classes.copy()
    classes_out[~island_mask] = 0
    coverage_out = coverage.astype(np.uint8, copy=True)
    coverage_out[~island_mask] = 0
    classes_profile = {
        "driver": "GTiff",
        "height": height,
        "width": width,
        "count": 2,
        "dtype": "uint8",
        "nodata": 0,
        "crs": TARGET_CRS,
        "transform": transform,
        "tiled": True,
        "blockxsize": 256,
        "blockysize": 256,
        "compress": "deflate",
        "predictor": 2,
    }
    with rasterio.open(classes_path, "w", **classes_profile) as dst:
        dst.write(classes_out, 1)
        dst.write(coverage_out, 2)
        dst.set_band_description(1, "ESA WorldCover 2021 v200 majority class")
        dst.set_band_description(2, "combined coverage class at 420 m footprint")
        dst.update_tags(
            worldcover_source="ESA WorldCover 2021 v200, CC BY 4.0",
            worldcover_classes=json.dumps(WORLD_COVER_CLASSES, sort_keys=True),
            coverage_classes=json.dumps(COVERAGE_CLASSES, sort_keys=True),
            trn_informative_rule=f"sigma_pos < {TRN_LIMIT_M} m at 420 m footprint",
            map_fix_classes="WorldCover excluding 10 tree cover, 80 permanent water, 95 mangroves",
            natural_earth_boundary="10m Admin-0; largest Taiwan polygon only; public domain",
            analysis_resolution_m=str(ANALYSIS_RES_M),
            worldcover_tiles=",".join(wc_tiles),
        )
    with (OUTPUT_DIR / "area_fractions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    report = OUTPUT_DIR / "method_and_sources.txt"
    with report.open("w", encoding="utf-8") as stream:
        stream.write("X8 Taiwan coverage map\n")
        stream.write("MEASURED: terrain-slope and land-cover area fractions calculated from real public raster data.\n")
        stream.write("INFERENCE: map-fix-likely is a land-cover proxy; sigma_h=4 m and the sigma_pos<50 m criterion are assumptions, not flight-validated performance.\n")
        stream.write("Copernicus DEM GLO-30 Public is available free for public use under its Copernicus DEM licence: https://dataspace.copernicus.eu/explore-data/data-collections/copernicus-contributing-missions/collections-description/COP-DEM.\n")
        stream.write(f"Copernicus GLO-30 is a DSM; terrain cells: {DEM_RES_M:g} m UTM; output grid: {ANALYSIS_RES_M:g} m UTM ({TARGET_CRS}).\n")
        stream.write(f"Effective square terrain stencil widths (m): {json.dumps(effective, sort_keys=True)}.\n")
        stream.write("Formula: sigma_pos = sigma_h / sqrt(mean(gx^2 + gy^2)); TRN informative iff sigma_pos < 50 m at 420 m footprint.\n")
        stream.write("Map-fix proxy: WorldCover 2021 v200 classes except tree cover (10), permanent water (80), mangroves (95); this is not measured matcher success.\n")
        stream.write("Copernicus DEM nodata or out-of-range elevations outside the Taiwan main-island mask are filled with 0 m sea level for coast-window gradients; all mainland mask cells must have plausible DSM values.\n")
        stream.write("WorldCover license: CC BY 4.0. Attribution: © ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data (2021) processed by ESA WorldCover consortium.\n")
        stream.write("WorldCover citation: Zanaga et al. (2022), ESA WorldCover 10 m 2021 v200, doi:10.5281/zenodo.7254221. Copernicus DEM: public AWS COGs, https://registry.opendata.aws/copernicus-dem/. Natural Earth boundary: public domain.\n")
        stream.write("Natural Earth source: https://www.naturalearthdata.com/downloads/10m-cultural-vectors/10m-admin-0-countries/ (public domain).\n")
        stream.write("Area fractions use 150 m projected cell centres; area_km2 is projected-grid area (not geodesic area). See area_fractions.csv.\n")
        stream.write("Metro analysis boxes (WGS84):\n")
        for name, extent in METRO_BOXES.items():
            stream.write(f"  {name}: {extent}\n")

    save_map(sigma_maps, coverage, island_mask, transform, main_island, effective)
    return terrain_path, classes_path, report


def save_map(sigma_maps, coverage, island_mask, transform, main_island, effective):
    height, width = island_mask.shape
    left = transform.c
    top = transform.f
    right = left + width * ANALYSIS_RES_M
    bottom = top - height * ANALYSIS_RES_M
    extent = (left / 1000, right / 1000, bottom / 1000, top / 1000)
    to_utm = Transformer.from_crs("EPSG:4326", TARGET_CRS, always_xy=True).transform
    city_geometries = {
        name: transform_geometry(to_utm, box(*bounds)).intersection(main_island)
        for name, bounds in METRO_BOXES.items()
    }
    coverage_colors = ["#858585", "#f3bd55", "#4a90c2", "#62ae6a"]
    coverage_cmap = ListedColormap(coverage_colors)
    coverage_norm = BoundaryNorm([0.5, 1.5, 2.5, 3.5, 4.5], coverage_cmap.N)
    fig, axes = plt.subplots(2, 2, figsize=(10, 14), constrained_layout=True)
    fig.suptitle("Taiwan GNSS-free cue coverage · 150 m cells · WorldCover 2021\nRed boxes: Taipei, Taichung and Kaohsiung study extents", fontsize=14)

    combined = np.ma.masked_where(~island_mask, coverage)
    ax = axes[0, 0]
    ax.imshow(combined, extent=extent, origin="upper", cmap=coverage_cmap, norm=coverage_norm, interpolation="nearest")
    ax.set_title("Combined classes · map-fix proxy · TRN < 50 m / 420 m")
    legend = [Patch(facecolor=coverage_colors[i - 1], label=COVERAGE_CLASSES[i]) for i in range(1, 5)]
    ax.legend(handles=legend, loc="lower left", fontsize=7, framealpha=0.9)

    for ax, footprint in zip(axes.flat[1:], FOOTPRINTS_M):
        sigma = sigma_maps[footprint]
        masked = np.ma.masked_where((~island_mask) | (sigma < 0), np.log10(np.maximum(sigma, 1.0)))
        image = ax.imshow(masked, extent=extent, origin="upper", cmap="viridis", vmin=0, vmax=3, interpolation="nearest")
        ax.set_title(f"log10 sigma_pos · {footprint} m footprint")
        fig.colorbar(image, ax=ax, shrink=0.76, label="log10(sigma_pos / m); cap 1000 m")

    for ax in axes.flat:
        outline_x, outline_y = main_island.exterior.xy
        ax.plot(np.asarray(outline_x) / 1000, np.asarray(outline_y) / 1000, color="black", linewidth=0.55, alpha=0.85)
        for name, geometry in city_geometries.items():
            if geometry.is_empty:
                continue
            west, south, east, north = box(*METRO_BOXES[name]).bounds
            projected_box = transform_geometry(to_utm, box(west, south, east, north))
            minx, miny, maxx, maxy = projected_box.bounds
            ax.add_patch(Rectangle((minx / 1000, miny / 1000), (maxx - minx) / 1000, (maxy - miny) / 1000,
                                   fill=False, edgecolor="#e13b2f", linewidth=0.8))
        ax.set_xlim(left / 1000, right / 1000)
        ax.set_ylim(bottom / 1000, top / 1000)
        ax.set_xlabel("UTM 51N easting (km)")
        ax.set_ylabel("UTM 51N northing (km)")
        ax.grid(color="white", alpha=0.25, linewidth=0.35)
    fig.savefig(OUTPUT_DIR / "coverage_map.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def print_summary(rows, classes, main_mask, sigma_maps, tile_counts, width, height):
    valid_main = main_mask & (classes != 0)
    print("\nX8 Taiwan coverage summary")
    print(f"  Analysis grid: {width} x {height}, {ANALYSIS_RES_M:g} m UTM cells")
    print(f"  Valid main-island cells: {int(valid_main.sum()):,}")
    print(f"  Public raster tiles read: {tile_counts[0]} Copernicus DEM, {tile_counts[1]} WorldCover")
    print(f"  Assumed sigma_h={SIGMA_H_M:g} m; threshold sigma_pos < {TRN_LIMIT_M:g} m at 420 m footprint")
    scopes = ["Taiwan main island", "20 km coastal strip", *METRO_BOXES]
    metrics = ["map_fix_likely", "TRN_informative_sigma_pos_lt_50m_170m", "TRN_informative_sigma_pos_lt_50m_420m",
               "TRN_informative_sigma_pos_lt_50m_1000m", "both_at_420m", "map_fix_only_at_420m",
               "TRN_only_at_420m", "neither_at_420m"]
    lookup = {(row["scope"], row["metric"]): row["fraction_pct"] for row in rows}
    for scope in scopes:
        print(f"  {scope}:")
        for metric in metrics:
            print(f"    {metric}: {lookup[(scope, metric)]:.1f}%")
    finite_land = int(np.count_nonzero(valid_main & np.isfinite(sigma_maps[420])))
    infinite_land = int(np.count_nonzero(valid_main & np.isinf(sigma_maps[420])))
    print(f"  420 m sigma_pos: {finite_land:,} finite / {infinite_land:,} infinite over valid island cells (infinite means zero or sub-float32 RMS slope)")
    print(f"  Outputs: {OUTPUT_DIR.relative_to(ROOT)}/coverage_map.png, terrain_sigma_pos_150m.tif, landcover_coverage_150m.tif, area_fractions.csv, method_and_sources.txt")
    print("  Evidence: MEASURED map-derived fractions; INFERENCE for the land-cover map-fix proxy and assumed sigma_h/threshold.")


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    fetch_if_missing(BOUNDARY_URL, BOUNDARY_PATH)
    main_island = largest_taiwan_polygon(BOUNDARY_PATH)
    inward = main_island.buffer(-20_000)
    if inward.is_empty:
        raise RuntimeError("20 km inward buffer unexpectedly erased Taiwan main island")
    coastal_strip = main_island.difference(inward)

    analysis_transform, width, height = make_grid(main_island.bounds, ANALYSIS_RES_M)
    island_mask = geometry_mask([mapping(main_island)], (height, width), analysis_transform, invert=True)
    coastal_mask = geometry_mask([mapping(coastal_strip)], (height, width), analysis_transform, invert=True)
    coastal_mask &= island_mask
    city_masks = metro_masks(width, height, analysis_transform, main_island)

    print(f"Taiwan main-island mask: {main_island.area / 1e6:,.0f} km2 (Natural Earth largest polygon)")
    print(f"Analysis grid: {width} x {height} cells at {ANALYSIS_RES_M:g} m; DEM terrain working grid: {DEM_RES_M:g} m")
    dem, dem_transform, dem_tiles = load_dem_60m(main_island)
    sigma_maps, effective = terrain_sigma_maps(dem, dem_transform, (height, width), analysis_transform)
    del dem
    classes, wc_tiles = load_worldcover(width, height, analysis_transform, main_island)
    missing_landcover = int(np.count_nonzero(island_mask & (classes == 0)))
    total_land_cells = int(island_mask.sum())
    if missing_landcover:
        print(f"  WARNING: WorldCover missing {missing_landcover:,}/{total_land_cells:,} main-island cells; fractions use valid cover cells only")
    rows, coverage = calculate_summary(classes, sigma_maps, island_mask, coastal_mask, city_masks, ANALYSIS_RES_M)
    terrain_path, classes_path, report = write_outputs(classes, sigma_maps, coverage, island_mask, analysis_transform, rows,
                                                       main_island, effective, dem_tiles, wc_tiles)
    print_summary(rows, classes, island_mask, sigma_maps, (len(dem_tiles), len(wc_tiles)), width, height)
    print(f"  Written: {terrain_path.relative_to(ROOT)}")
    print(f"  Written: {classes_path.relative_to(ROOT)}")
    print(f"  Written: {report.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

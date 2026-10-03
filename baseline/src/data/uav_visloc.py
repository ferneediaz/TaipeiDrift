"""UAV-VisLoc adapter: real drone photos and a satellite map taken years later.

Xu et al. 2024, https://github.com/IntelliSensing/UAV-VisLoc. One flight is one numbered folder:

    <root>/03/03.csv               one row per photo: lat, lon (centre of the photo on the ground),
                                   height, date, Omega, Kappa, Phi1, Phi2
    <root>/03/drone/03_0001.JPG    3976 x 2652 px, looking straight down
    <root>/03/satellite03.tif      the satellite map (Google Earth, 0.3 m), latitude/longitude grid

Facts checked on flight 03 (Taizhou, October 2018; map from April 2021), ``docs/findings.md`` 3.9:

- 768 photos, one every 95 m, 74 km in a lawnmower pattern with 14 turns of more than 45 degrees.
- ``Phi2`` is the course as a bearing from north, clockwise: it agrees with the direction between
  neighbouring photos to within 2 degrees.
- ``Phi1`` is the heading, where the nose and the camera point. It differs from the course by the
  crab angle against the wind, 4 to 13 degrees, with opposite signs on opposite legs. Photos turned
  north up with ``Phi1`` need the same small turn on every leg; with ``Phi2`` they do not.
- A photo covers about 500 by 330 m.
- The matched position lies about 13 m ahead of the recorded one in the direction of flight, on
  every leg: a fixed offset in the drone's own frame (camera tilt or trigger delay).

How it is turned into a ``CameraFlight``:

- Positions are (north, east) in metres from the first photo, on a flat local grid. Over the
  7 km of a map the error of that grid is below a metre.
- The map is resampled to a square metric grid (``metres_per_pixel``) and cached as a PNG.
- Every photo is turned so that north is up, using a heading the caller supplies (in a test, the
  recorded heading plus the error of a chosen heading sensor). The largest square that stays inside
  the turned photo for any angle is kept, so the frame has no black corners.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from src.data.camera_flight import CameraFlight, ReferenceMap
from src.data.ground_map import GroundMap

FRAME_PX = 400  # side of the north-up frames handed to the navigator


class VisLocDataNotFound(FileNotFoundError):
    """The UAV-VisLoc flight folder is not where it was expected."""


@dataclass
class VisLocConfig:
    data_root: str = "data/raw/uav_visloc"
    flight: str = "03"
    metres_per_pixel: float = 1.0  # of the resampled map
    cache_dir: str | None = None  # map and frames are kept here


def metres_per_degree(latitude_deg: float) -> tuple[float, float]:
    """Length of one degree of latitude and of longitude, in metres, at a latitude (WGS84 series)."""
    phi = np.radians(latitude_deg)
    lat = 111132.954 - 559.822 * np.cos(2 * phi) + 1.175 * np.cos(4 * phi)
    lon = 111412.84 * np.cos(phi) - 93.5 * np.cos(3 * phi)
    return float(lat), float(lon)


def load_visloc_table(cfg: VisLocConfig) -> pd.DataFrame:
    folder = Path(cfg.data_root).expanduser() / cfg.flight
    table = folder / f"{cfg.flight}.csv"
    if not table.is_file():
        raise VisLocDataNotFound(f"{table} not found; unzip the UAV-VisLoc flight into {cfg.data_root}/ (data/README.md)")
    return pd.read_csv(table)


def load_visloc_flight(cfg: VisLocConfig, heading_deg: np.ndarray | None = None) -> CameraFlight:
    """Load one flight. ``heading_deg`` is the drone's heading sensor, used to turn each photo north up.

    Without it the recorded heading (``Phi1``) is used, which is the truth; a fair test passes the
    output of a heading sensor model instead (``src.sensors.heading``), or calls ``with_heading``.
    """
    folder = Path(cfg.data_root).expanduser() / cfg.flight
    table = load_visloc_table(cfg)
    ranges = pd.read_csv(Path(cfg.data_root).expanduser() / "satellite_ coordinates_range.csv")
    # the example archive names the map "03.tif", the full archive "satellite03.tif"
    box = ranges[ranges.mapname.isin([f"{cfg.flight}.tif", f"satellite{cfg.flight}.tif"])].iloc[0]

    lat0, lon0 = float(table.lat[0]), float(table.lon[0])
    per_lat, per_lon = metres_per_degree(0.5 * (box.LT_lat_map + box.RB_lat_map))
    position = np.column_stack([(table.lat - lat0) * per_lat, (table.lon - lon0) * per_lon])
    corner = np.array([(box.LT_lat_map - lat0) * per_lat, (box.LT_lon_map - lon0) * per_lon])
    size_m = np.array([(box.LT_lat_map - box.RB_lat_map) * per_lat, (box.RB_lon_map - box.LT_lon_map) * per_lon])

    cache = Path(cfg.cache_dir).expanduser() / f"visloc_{cfg.flight}" if cfg.cache_dir else None
    ground = _ground_map(folder / f"satellite{cfg.flight}.tif", corner, size_m, cfg.metres_per_pixel, cache)

    photos = [str(folder / "drone" / name) for name in table.filename]
    squares = _square_photos(photos, cache)

    seconds = pd.to_datetime(table.date).astype("int64").to_numpy() / 1e9
    flight = CameraFlight(
        name=f"visloc_{cfg.flight}",
        timestamp=seconds - seconds[0],
        position_gt=position,
        load_frame=lambda i: np.zeros((FRAME_PX, FRAME_PX), np.uint8),
        reference=ReferenceMap(np.zeros((0, 2)), cfg.metres_per_pixel, lambda i: np.zeros((1, 1), np.uint8)),
        metadata={
            "source": "uav_visloc",
            "flight": cfg.flight,
            "origin_lat_lon": [lat0, lon0],
            "lat_lon": table[["lat", "lon"]].to_numpy(),
            "true_heading_deg": table.Phi1.to_numpy(),
            "course_deg": table.Phi2.to_numpy(),
            "height_m": table.height.to_numpy(),
            "date": table.date.tolist(),
            "photo_squares": squares,
        },
        ground_map=ground,
    )
    return with_heading(flight, table.Phi1.to_numpy() if heading_deg is None else heading_deg)


def with_heading(flight: CameraFlight, heading_deg: np.ndarray) -> CameraFlight:
    """The same flight, with its photos turned north up by another heading sensor's readings."""
    heading = np.asarray(heading_deg, dtype=float)
    squares = flight.metadata["photo_squares"]
    return replace(flight, load_frame=lambda i: north_up(squares[i], heading[i]), heading_deg=heading)


def north_up(square: np.ndarray, heading_deg: float) -> np.ndarray:
    """Turn a square photo so that north is up, and keep the largest square without black corners.

    The top edge of the photo points along the heading. A drone heading east (90 degrees) has north
    on the left of its photo, so the photo is turned clockwise by the heading; OpenCV counts
    counter-clockwise, hence the minus sign. What is left over, the camera's mounting and the error
    of the heading sensor, is the small turn the navigator learns before the jam, as on ALTO.
    """
    n = square.shape[0]
    turn = cv2.getRotationMatrix2D((n / 2, n / 2), -float(heading_deg), 1.0)
    turned = cv2.warpAffine(square, turn, (n, n), flags=cv2.INTER_LINEAR)
    side = int(n / np.sqrt(2)) - 2
    start = (n - side) // 2
    inner = turned[start : start + side, start : start + side]
    return cv2.resize(inner, (FRAME_PX, FRAME_PX), interpolation=cv2.INTER_AREA)


def _square_photos(paths: list[str], cache: Path | None, side: int = 800) -> np.ndarray:
    """The central square of every photo, grey, ``side`` pixels (cached as one array)."""
    target = cache / f"photos_{side}.npy" if cache else None
    if target is not None and target.is_file():
        stack = np.load(target, mmap_mode="r")
        if len(stack) == len(paths):
            return stack
    stack = np.zeros((len(paths), side, side), np.uint8)
    for i, path in enumerate(paths):
        photo = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if photo is None:
            raise VisLocDataNotFound(f"cannot read {path}")
        h, w = photo.shape
        s = min(h, w)
        top, left = (h - s) // 2, (w - s) // 2
        stack[i] = cv2.resize(photo[top : top + s, left : left + s], (side, side), interpolation=cv2.INTER_AREA)
    if target is not None:
        target.parent.mkdir(parents=True, exist_ok=True)
        np.save(target, stack)
    return stack


def _ground_map(tif: Path, corner: np.ndarray, size_m: np.ndarray, metres_per_pixel: float, cache: Path | None) -> GroundMap:
    """The satellite map on a square metric grid, north up, grey."""
    if cache is not None and (cache / "map_info.json").is_file():
        info = json.loads((cache / "map_info.json").read_text())
        image = cv2.imread(str(cache / "map.png"), cv2.IMREAD_GRAYSCALE)
        if image is not None and info["metres_per_pixel"] == metres_per_pixel:
            return GroundMap(image, metres_per_pixel, np.array(info["origin_north_east"]), image > 0, zoom_unit_px=info["zoom_unit_px"])
    import rasterio
    from rasterio.enums import Resampling

    height, width = (int(round(s / metres_per_pixel)) for s in size_m)
    with rasterio.open(tif) as src:
        rgb = src.read(out_shape=(src.count, height, width), resampling=Resampling.average)
    image = cv2.cvtColor(np.ascontiguousarray(np.transpose(rgb[:3], (1, 2, 0))), cv2.COLOR_RGB2GRAY)
    zoom_unit_px = 300
    if cache is not None:
        cache.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(cache / "map.png"), image)
        (cache / "map_info.json").write_text(json.dumps(
            {"origin_north_east": corner.tolist(), "metres_per_pixel": metres_per_pixel, "zoom_unit_px": zoom_unit_px, "source": tif.name},
            indent=2))
    return GroundMap(image, metres_per_pixel, corner, image > 0, zoom_unit_px=zoom_unit_px)

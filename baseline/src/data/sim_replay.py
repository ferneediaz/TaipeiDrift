"""Simulator recordings (``taipeidrift-replay/1``, written by ``sim/nodes/recorder.py``) as a CameraFlight.

The demo flight: Gazebo with the Wufeng 2020 aerial image as the ground, flown along the corridor
planned by ``sim/scripts/plan_route.py``. The navigator's map is the 2018 aerial image of the same
place, two years older than the ground, as a real map would be. Passing the 2020 image instead gives
the navigator a map as fresh as the ground (what Ukraine's Eagle Eyes has, from recent
reconnaissance flights); the difference between the two runs is the price of an old map.

Frames in the simulator, from the recording's ``meta.json``:
- Truth: ENU (x east, y north, z up) in metres from the world origin, the centre of the 2020 image;
  ``q`` turns the body (forward, left, up) into ENU.
- The down camera: 512 x 512 px, 90 degrees wide, looking straight down; the top of the image points
  forward. At 100 m it sees 200 by 200 m, 0.39 m per pixel.

How it is turned into a ``CameraFlight`` (conventions of ``camera_flight.py``):
- One frame per recorded image (5 per second). The flight starts at the first image at flight height
  (after the climb) and ends at the last image before the drone stops at the end of the route.
- Positions are (north, east) in metres from the first frame.
- Heading: a bearing from north, clockwise. ENU yaw is counted counter-clockwise from east, so
  heading = 90 degrees - yaw. Each image is turned north up with a heading the caller supplies (in a
  test, the true heading plus the error of a chosen sensor model, ``src.sensors.heading``), and the
  largest square without black corners is kept, as for UAV-VisLoc.
- The map: the aerial image resampled to ``map_metres_per_pixel``, grey, north up; where it holds no
  imagery (black) is marked as not covered.
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

FRAME_PX = 360  # int(512 / sqrt(2)) - 2: the largest square inside a 512 px image at any angle


@dataclass
class SimReplayConfig:
    recording: str = "recordings/wufeng_corridor_100m"
    map_tif: str = "data/raw/aerial/wufeng_2018-05-03_x4.tif"  # the 2020 image gives a map as fresh as the ground
    route: str = "sim/scenarios/wufeng_corridor.json"  # holds the world origin in EPSG:3826
    map_metres_per_pixel: float = 0.5
    start_height_m: float = 95.0  # the flight starts at the first image at least this high
    stop_speed_mps: float = 2.0  # and ends before the drone slows below this at the end of the route
    cache_dir: str | None = "data/processed"


def heading_from_quaternion(qw: np.ndarray, qx: np.ndarray, qy: np.ndarray, qz: np.ndarray) -> np.ndarray:
    """Heading in degrees (bearing from north, clockwise) from a body-to-ENU quaternion.

    Example: a drone flying east has ENU yaw 0 and heading 90; flying north, yaw 90 and heading 0.
    """
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
    return np.mod(90.0 - np.degrees(yaw), 360.0)


def north_up(image: np.ndarray, heading_deg: float) -> np.ndarray:
    """Turn a down-camera image so that north is up; keep the largest square without black corners.

    The top of the image points along the heading. A drone heading east has north on the left of its
    image, so the image is turned clockwise by the heading (OpenCV counts counter-clockwise).
    """
    n = image.shape[0]
    turn = cv2.getRotationMatrix2D((n / 2, n / 2), -float(heading_deg), 1.0)
    turned = cv2.warpAffine(image, turn, (n, n), flags=cv2.INTER_LINEAR)
    start = (n - FRAME_PX) // 2
    return turned[start : start + FRAME_PX, start : start + FRAME_PX]


def load_sim_flight(cfg: SimReplayConfig, heading_deg: np.ndarray | None = None) -> CameraFlight:
    """Load one recording. ``heading_deg`` is the drone's heading sensor, one value per frame of the flight.

    Without it the true heading is used; a fair test passes a sensor model's output, or calls ``with_heading``.
    """
    rec = Path(cfg.recording).expanduser()
    truth = pd.read_csv(rec / "truth.csv")
    images = pd.read_csv(rec / "images.csv")
    images = images[images.cam == "cam0"].reset_index(drop=True)
    t = images.t_s.to_numpy()

    east = np.interp(t, truth.t_s, truth.e_m)
    north = np.interp(t, truth.t_s, truth.n_m)
    up = np.interp(t, truth.t_s, truth.u_m)
    nearest = np.clip(np.searchsorted(truth.t_s.to_numpy(), t), 0, len(truth) - 1)
    q = truth[["qw", "qx", "qy", "qz"]].to_numpy()[nearest]
    true_heading = heading_from_quaternion(*q.T)

    speed = np.r_[0.0, np.hypot(np.diff(east), np.diff(north)) / np.maximum(np.diff(t), 1e-6)]
    high = np.nonzero(up >= cfg.start_height_m)[0]
    if len(high) == 0:
        raise ValueError(f"the drone never reaches {cfg.start_height_m} m in {rec}")
    first = int(high[0])
    moving = np.nonzero(speed[first:] >= cfg.stop_speed_mps)[0]
    last = first + int(moving[-1]) if len(moving) else len(t) - 1
    keep = slice(first, last + 1)

    origin = np.array([north[first], east[first]])
    position = np.column_stack([north[keep], east[keep]]) - origin
    route = json.loads(Path(cfg.route).read_text())
    ground = _ground_map(Path(cfg.map_tif), (route["origin_easting_m"], route["origin_northing_m"]), origin,
                         cfg.map_metres_per_pixel, Path(cfg.cache_dir).expanduser() if cfg.cache_dir else None)

    paths = [str(rec / p) for p in images.path[keep]]
    flight = CameraFlight(
        name=f"sim_{rec.name}",
        timestamp=t[keep] - t[first],
        position_gt=position,
        load_frame=lambda i: np.zeros((FRAME_PX, FRAME_PX), np.uint8),
        reference=ReferenceMap(np.zeros((0, 2)), cfg.map_metres_per_pixel, lambda i: np.zeros((1, 1), np.uint8)),
        metadata={
            "source": "simulator",
            "recording": str(rec),
            "map": Path(cfg.map_tif).name,
            "origin_enu_m": [float(east[first]), float(north[first])],
            "true_heading_deg": true_heading[keep],
            "height_m": up[keep],
            "recording_t_s": t[keep],
            "image_paths": paths,
        },
        ground_map=ground,
    )
    return with_heading(flight, true_heading[keep] if heading_deg is None else heading_deg)


def with_heading(flight: CameraFlight, heading_deg: np.ndarray) -> CameraFlight:
    """The same flight, with its images turned north up by another heading sensor's readings."""
    heading = np.asarray(heading_deg, dtype=float)
    if heading.shape != (len(flight),):
        raise ValueError(f"heading has shape {heading.shape}, expected ({len(flight)},)")
    paths = flight.metadata["image_paths"]
    return replace(flight, load_frame=lambda i: north_up(cv2.imread(paths[i], cv2.IMREAD_GRAYSCALE), heading[i]), heading_deg=heading)


def _ground_map(tif: Path, origin_en: tuple[float, float], start_ne: np.ndarray, metres_per_pixel: float,
                cache: Path | None) -> GroundMap:
    """The aerial image as the navigator's map: grey, north up, ``metres_per_pixel``, in the flight's frame.

    The image is georeferenced in EPSG:3826 (true metres). Its top-left corner, minus the world origin
    (``origin_en``, east and north) and minus the first frame's position (``start_ne``), is the map's origin.
    """
    import rasterio
    from rasterio.enums import Resampling

    name = f"sim_map_{tif.stem}_{metres_per_pixel:g}"
    with rasterio.open(tif) as src:
        bounds = src.bounds
        corner = np.array([bounds.top - origin_en[1], bounds.left - origin_en[0]]) - start_ne
        if cache is not None and (cache / f"{name}.png").is_file():
            image = cv2.imread(str(cache / f"{name}.png"), cv2.IMREAD_GRAYSCALE)
        else:
            height = int(round((bounds.top - bounds.bottom) / metres_per_pixel))
            width = int(round((bounds.right - bounds.left) / metres_per_pixel))
            rgb = src.read([1, 2, 3], out_shape=(3, height, width), resampling=Resampling.average)
            image = cv2.cvtColor(np.ascontiguousarray(np.transpose(rgb, (1, 2, 0))), cv2.COLOR_RGB2GRAY)
            if cache is not None:
                cache.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(cache / f"{name}.png"), image)
    empty = cv2.morphologyEx((image < 8).astype(np.uint8), cv2.MORPH_OPEN, np.ones((7, 7), np.uint8)).astype(bool)
    return GroundMap(image, metres_per_pixel, corner, ~empty, zoom_unit_px=FRAME_PX)

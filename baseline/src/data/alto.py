"""ALTO adapter: the competition sample of the ALTO helicopter dataset.

One section (``Val`` or ``Train``) is read straight from its zip file, without unpacking:

    <root>/Val.zip
        Val/query_images/000000.png ...        camera frames, 500 x 500 px
        Val/reference_images/offset_0_None/    reference images centred on the true route, one every 10 m
        Val/reference_images/offset_20_North/  ... and 20 and 40 m north and south of it (four more folders)
        Val/query.csv                          easting, northing, altitude, orientation per frame
        Val/reference.csv                      easting, northing per reference image

What the sample does not contain: raw IMU readings and the height above ground.
Facts about the files were measured in ``experiments/f_alto_matching.py`` and
``experiments/m_alto_orientation.py``; see ``docs/findings.md``, section 3.
"""
from __future__ import annotations

import json
import zipfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from src.data.camera_flight import CameraFlight, ReferenceMap
from src.data.ground_map import GroundMap, mosaic

FRAME_RATE_HZ = 20.0  # stated in the dataset readme
REFERENCE_METRES_PER_PIXEL = 0.60  # measured by matching reference images against each other
MAIN_REFERENCE_FOLDER = "offset_0_None"  # the reference images on the route itself


class AltoDataNotFound(FileNotFoundError):
    """The ALTO zip file is not where it was expected."""


@dataclass
class AltoConfig:
    data_root: str = "data/raw/alto"
    section: str = "Val"  # "Val" or "Train"
    metres_per_pixel: float = REFERENCE_METRES_PER_PIXEL
    ground_map: bool = False  # also build one map from all reference folders, for the search around the estimate
    map_cache_dir: str | None = None  # where the built map is kept; None builds it every time


def load_alto_flight(cfg: AltoConfig) -> CameraFlight:
    """Load one ALTO section as a ``CameraFlight``. Images are read on demand."""
    path = Path(cfg.data_root).expanduser() / f"{cfg.section}.zip"
    if not path.is_file():
        raise AltoDataNotFound(
            f"{path} not found. Download {cfg.section}.zip as described in data/README.md "
            f"and put it into {cfg.data_root}/."
        )
    archive = zipfile.ZipFile(path)
    section = cfg.section
    query = pd.read_csv(archive.open(f"{section}/query.csv"))
    all_references = pd.read_csv(archive.open(f"{section}/reference.csv"))
    reference = all_references[all_references.name.str.startswith(MAIN_REFERENCE_FOLDER)].reset_index(drop=True)

    origin = query[["northing", "easting"]].to_numpy()[0]

    def decode(member: str) -> np.ndarray:
        raw = np.frombuffer(archive.read(member), np.uint8)
        return cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)

    ground_map = None
    if cfg.ground_map:
        ground_map = _ground_map(section, all_references, origin, decode, cfg)

    reference_map = ReferenceMap(
        position=reference[["northing", "easting"]].to_numpy() - origin,
        metres_per_pixel=cfg.metres_per_pixel,
        load=lambda i: decode(f"{section}/reference_images/{reference.name[i]}"),
    )
    return CameraFlight(
        name=f"alto_{section.lower()}",
        timestamp=np.arange(len(query)) / FRAME_RATE_HZ,
        position_gt=query[["northing", "easting"]].to_numpy() - origin,
        load_frame=lambda i: decode(f"{section}/query_images/{query.name[i]}"),
        reference=reference_map,
        metadata={
            "source": "alto",
            "file": str(path),
            "section": section,
            "origin_northing_easting": origin.tolist(),
            "altitude_ellipsoid_m": query.altitude.to_numpy(),
        },
        ground_map=ground_map,
    )


def _ground_map(section: str, references: pd.DataFrame, origin: np.ndarray, decode, cfg: AltoConfig) -> GroundMap:
    """One map from the reference images of all folders: the route and 20 and 40 m to either side.

    The images of all folders are crops of the same aerial photos. Their coordinates agree with
    each other to within one pixel (checked by phase correlation of overlapping images, findings 3.8).
    Together they cover a strip about 380 m wide along the route.
    """
    cache = Path(cfg.map_cache_dir).expanduser() / f"alto_{section.lower()}_map" if cfg.map_cache_dir else None
    if cache is not None and (cache / "info.json").is_file():
        info = json.loads((cache / "info.json").read_text())
        image = cv2.imread(str(cache / "map.png"), cv2.IMREAD_GRAYSCALE)
        covered = cv2.imread(str(cache / "covered.png"), cv2.IMREAD_GRAYSCALE) > 0
        if info.get("images") == len(references) and image is not None:
            return GroundMap(image, info["metres_per_pixel"], np.array(info["origin_north_east"]), covered, info["zoom_unit_px"])
    centres = references[["northing", "easting"]].to_numpy() - origin
    names = references.name.tolist()
    ground = mosaic(centres, lambda i: decode(f"{section}/reference_images/{names[i]}"), cfg.metres_per_pixel, zoom_unit_px=500)
    if cache is not None:
        cache.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(cache / "map.png"), ground.image)
        cv2.imwrite(str(cache / "covered.png"), ground.covered.astype(np.uint8) * 255)
        (cache / "info.json").write_text(json.dumps({
            "section": section,
            "images": len(references),
            "folders": sorted({n.split("/")[0] for n in names}),
            "metres_per_pixel": cfg.metres_per_pixel,
            "origin_north_east": ground.origin.tolist(),
            "zoom_unit_px": ground.zoom_unit_px,
        }, indent=2))
    return ground

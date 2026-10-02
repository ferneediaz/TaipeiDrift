"""ALTO adapter: the competition sample of the ALTO helicopter dataset.

One section (``Val`` or ``Train``) is read straight from its zip file, without unpacking:

    <root>/Val.zip
        Val/query_images/000000.png ...        camera frames, 500 x 500 px
        Val/reference_images/offset_0_None/    reference images along the route, one every 10 m
        Val/query.csv                          easting, northing, altitude, orientation per frame
        Val/reference.csv                      easting, northing per reference image

What the sample does not contain: raw IMU readings and the height above ground.
Facts about the files were measured in ``experiments/f_alto_matching.py`` and
``experiments/m_alto_orientation.py``; see ``docs/findings.md``, section 3.
"""
from __future__ import annotations

import zipfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from src.data.camera_flight import CameraFlight, ReferenceMap

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
    reference = pd.read_csv(archive.open(f"{section}/reference.csv"))
    reference = reference[reference.name.str.startswith(MAIN_REFERENCE_FOLDER)].reset_index(drop=True)

    origin = query[["northing", "easting"]].to_numpy()[0]

    def decode(member: str) -> np.ndarray:
        raw = np.frombuffer(archive.read(member), np.uint8)
        return cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)

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
    )

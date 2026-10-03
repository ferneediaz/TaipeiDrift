"""A flight seen through a downward camera, with reference images that have coordinates.

This is the camera counterpart of ``Trajectory``: it carries no IMU. The ALTO loader and the
synthetic generator both produce it, so the navigator never needs to know where the data came from.

Conventions
-----------
- positions: (north, east) in metres, origin at the first camera frame.
- images: grayscale. A reference image has north at the top and east to the right.
- ``metres_per_pixel``: ground size of one pixel of a reference image.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

import cv2
import numpy as np

from src.data.ground_map import GroundMap

_CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


def prepare(gray: np.ndarray) -> np.ndarray:
    """Even out the local contrast and return float32, the form used for matching."""
    return _CLAHE.apply(gray).astype(np.float32)


@dataclass
class ReferenceMap:
    """Reference images along the route, each with the position of its centre."""

    position: np.ndarray  # (M, 2) north, east of each image centre, m
    metres_per_pixel: float
    load: Callable[[int], np.ndarray]  # index -> grayscale uint8 image
    _prepared: dict[int, np.ndarray] = field(default_factory=dict, repr=False)

    def __len__(self) -> int:
        return len(self.position)

    def image(self, i: int) -> np.ndarray:
        """Reference image ``i``, prepared for matching (cached)."""
        i = int(i)
        if i not in self._prepared:
            self._prepared[i] = prepare(self.load(i))
        return self._prepared[i]

    def nearest(self, position: np.ndarray, n: int) -> np.ndarray:
        """Indices of the ``n`` reference images closest to a position."""
        return np.argsort(np.linalg.norm(self.position - position, axis=1))[:n]

    def within(self, position: np.ndarray, radius: float) -> np.ndarray:
        """Indices of all reference images within ``radius`` metres of a position."""
        return np.nonzero(np.linalg.norm(self.position - position, axis=1) <= radius)[0]


@dataclass
class CameraFlight:
    """Camera frames with their true positions, plus the reference images to match against."""

    name: str
    timestamp: np.ndarray  # (N,) s
    position_gt: np.ndarray  # (N, 2) north, east in m, origin at the first frame
    load_frame: Callable[[int], np.ndarray]  # index -> grayscale uint8 camera frame
    reference: ReferenceMap
    metadata: dict[str, Any] = field(default_factory=dict)
    ground_map: GroundMap | None = None  # one map of the whole area, for the search around the estimate
    heading_deg: np.ndarray | None = None  # (N,) heading from the drone's own sensor (bearing from north), not the truth

    def __post_init__(self) -> None:
        self.timestamp = np.asarray(self.timestamp, dtype=float)
        self.position_gt = np.asarray(self.position_gt, dtype=float)
        if self.position_gt.shape != (len(self.timestamp), 2):
            raise ValueError(f"position_gt has shape {self.position_gt.shape}, expected ({len(self.timestamp)}, 2)")

    def __len__(self) -> int:
        return len(self.timestamp)

    def frame(self, i: int) -> np.ndarray:
        """Camera frame ``i`` as grayscale uint8."""
        return self.load_frame(int(i))

    @property
    def travelled(self) -> np.ndarray:
        """True distance flown up to each frame, in metres."""
        steps = np.linalg.norm(np.diff(self.position_gt, axis=0), axis=1)
        return np.concatenate([[0.0], np.cumsum(steps)])

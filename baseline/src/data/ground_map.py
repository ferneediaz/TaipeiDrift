"""One north-up aerial image of the whole area, with coordinates: the map the navigator searches.

Positions are (north, east) in metres, in the same frame as the flight. Pixel coordinates follow
OpenCV: x to the right (east), y downwards (south), both measured from the top-left corner of the
map, which lies at ``origin``. A point at pixel coordinates (x, y) lies at

    north = origin_north - y * metres_per_pixel
    east  = origin_east  + x * metres_per_pixel

Example with 0.5 m per pixel and the top-left corner at north 100 m, east 0 m: the point at
x = 20, y = 40 lies at north 100 - 40 * 0.5 = 80 m and east 0 + 20 * 0.5 = 10 m.

Why one map and not the reference images of a dataset: ALTO's reference images are centred on the
true flight path. Searching only those tells the navigator where the path is (findings, section 3.8).
A map of the whole area, searched around the estimate, does not.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import cv2
import numpy as np


@dataclass
class GroundMap:
    """A north-up map with coordinates, and where it holds imagery."""

    image: np.ndarray  # (H, W) uint8 grayscale, north at the top
    metres_per_pixel: float
    origin: np.ndarray  # (north, east) of the top-left corner, m
    covered: np.ndarray | None = None  # (H, W) bool, True where the map holds imagery; None: everywhere
    zoom_unit_px: int = 500  # a frame with zoom 1 is this many map pixels wide
    _prepared: np.ndarray | None = field(default=None, repr=False)
    _covered_float: np.ndarray | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        self.origin = np.asarray(self.origin, dtype=float)
        if self.covered is None:
            self.covered = np.ones(self.image.shape[:2], dtype=bool)
        if self.covered.shape != self.image.shape[:2]:
            raise ValueError(f"covered has shape {self.covered.shape}, the image {self.image.shape[:2]}")

    @property
    def shape(self) -> tuple[int, int]:
        return self.image.shape[:2]

    def to_pixel(self, position: np.ndarray) -> np.ndarray:
        """(north, east) in m to pixel coordinates (x, y)."""
        north, east = np.asarray(position, dtype=float)
        return np.array([(east - self.origin[1]), (self.origin[0] - north)]) / self.metres_per_pixel

    def to_position(self, x: float, y: float) -> np.ndarray:
        """Pixel coordinates (x, y) to (north, east) in m."""
        return np.array([self.origin[0] - y * self.metres_per_pixel, self.origin[1] + x * self.metres_per_pixel])

    def prepared(self) -> np.ndarray:
        """The map with its local contrast evened out, as float32 (cached).

        The tiles of the contrast step are one eighth of ``zoom_unit_px``, the same ground size as
        when a single reference image of that width is prepared. Empty parts of the map are filled
        with the median brightness first, so that they do not distort the tiles at the edge.
        """
        if self._prepared is None:
            filled = self.image.copy()
            if not self.covered.all():
                filled[~self.covered] = int(np.median(self.image[self.covered])) if self.covered.any() else 0
            tile = self.zoom_unit_px / 8
            grid = (max(1, round(filled.shape[1] / tile)), max(1, round(filled.shape[0] / tile)))
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=grid)
            self._prepared = clahe.apply(filled).astype(np.float32)
        return self._prepared

    def covered_float(self) -> np.ndarray:
        """``covered`` as float32 0 and 1, for counting covered pixels under a template (cached)."""
        if self._covered_float is None:
            self._covered_float = self.covered.astype(np.float32)
        return self._covered_float


def mosaic(
    centres: np.ndarray,
    load: Callable[[int], np.ndarray],
    metres_per_pixel: float,
    zoom_unit_px: int = 500,
) -> GroundMap:
    """Paste north-up images, each with the (north, east) of its centre, into one map.

    Where images overlap, their brightness is averaged. Each image is placed to the nearest pixel,
    so a place is off by at most half a pixel.
    """
    centres = np.asarray(centres, dtype=float)
    first = load(0)
    h, w = first.shape[:2]
    half = np.array([h, w]) / 2 * metres_per_pixel  # m from the centre to the top and left edges
    origin = np.array([centres[:, 0].max() + half[0], centres[:, 1].min() - half[1]])
    height = int(np.ceil((centres[:, 0].max() - centres[:, 0].min()) / metres_per_pixel)) + h + 1
    width = int(np.ceil((centres[:, 1].max() - centres[:, 1].min()) / metres_per_pixel)) + w + 1
    total = np.zeros((height, width), np.float32)
    count = np.zeros((height, width), np.uint16)
    for i, (north, east) in enumerate(centres):
        image = first if i == 0 else load(i)
        x0 = int(round((east - origin[1]) / metres_per_pixel - w / 2))
        y0 = int(round((origin[0] - north) / metres_per_pixel - h / 2))
        total[y0 : y0 + h, x0 : x0 + w] += image
        count[y0 : y0 + h, x0 : x0 + w] += 1
    covered = count > 0
    image = np.zeros((height, width), np.uint8)
    image[covered] = np.clip(np.round(total[covered] / count[covered]), 0, 255).astype(np.uint8)
    return GroundMap(image, metres_per_pixel, origin, covered, zoom_unit_px)

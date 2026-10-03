"""Position fixes: find a camera frame inside reference images that have coordinates.

The frame is brought to the scale and orientation of the reference images, its centre is cut
out as a template, and the template is slid over each candidate reference image. The place
with the highest normalised correlation is the fix. The correlation compares brightness
patterns after removing the mean and the contrast of each patch, so a brighter or darker
frame scores the same.

Two numbers describe how a frame relates to the reference images:

- ``zoom``: the frame shows ``zoom`` times the width of a reference image. It grows with the
  height above ground.
- ``angle``: the frame has to be turned by this many degrees, counter-clockwise, to put north
  at the top.

Why this method and not keypoints: see ``docs/findings.md``, section 3.2.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np

from src.data.camera_flight import ReferenceMap
from src.data.ground_map import GroundMap

KEEP = 0.8  # share of the scaled and turned frame used as the template; the corners are cut off


@dataclass
class Match:
    """The best place found for one camera frame."""

    score: float  # normalised correlation at that place, between -1 and 1
    position: np.ndarray  # (2,) north, east of the centre of the camera frame, m
    zoom: float
    angle: float  # degrees
    reference_index: int  # the reference image in which the place was found


def make_template(
    frame: np.ndarray, zoom: float, angle: float, reference_side: int, keep: float = KEEP, inscribed: bool = False
) -> np.ndarray:
    """Bring a prepared camera frame to the scale and orientation of the reference images.

    Args:
        frame: camera frame, prepared with ``camera_flight.prepare``.
        zoom: the frame shows ``zoom`` times the width of a reference image.
        angle: degrees to turn the frame, counter-clockwise.
        reference_side: width of a reference image in pixels.
        keep: share of the result that is kept, cut from its centre.
        inscribed: cut no more than the largest square that lies inside the turned frame. A frame
            turned by 45 degrees leaves black corners in a cut of 0.8; the largest clean square is
            then 1 / (cos 45 + sin 45) = 0.71 of the side.
    """
    n = int(round(reference_side * zoom))
    scaled = cv2.resize(frame, (n, n), interpolation=cv2.INTER_AREA)
    turned = cv2.warpAffine(scaled, cv2.getRotationMatrix2D((n / 2, n / 2), angle, 1.0), (n, n))
    if inscribed:
        a = np.radians(angle)
        keep = min(keep, 1.0 / (abs(np.cos(a)) + abs(np.sin(a))) - 0.01)
    side = int(n * keep)
    start = (n - side) // 2
    return turned[start : start + side, start : start + side]


def match(
    frame: np.ndarray,
    reference_map: ReferenceMap,
    candidates: Iterable[int],
    zooms: Iterable[float],
    angles: Iterable[float],
    keep: float = KEEP,
) -> Match:
    """Find a prepared camera frame in the candidate reference images.

    Every combination of zoom and angle is tried against every candidate. The place with the
    highest score wins.
    """
    candidates = [int(i) for i in candidates]
    if not candidates:
        raise ValueError("match needs at least one candidate reference image")
    metres_per_pixel = reference_map.metres_per_pixel
    reference_side = reference_map.image(candidates[0]).shape[1]
    best: Match | None = None
    for zoom in zooms:
        for angle in angles:
            template = make_template(frame, float(zoom), float(angle), reference_side, keep)
            for index in candidates:
                reference = reference_map.image(index)
                if template.shape[0] > reference.shape[0] or template.shape[1] > reference.shape[1]:
                    continue
                scores = cv2.matchTemplate(reference, template, cv2.TM_CCOEFF_NORMED)
                _, score, _, corner = cv2.minMaxLoc(scores)
                if best is None or score > best.score:
                    centre_x = corner[0] + template.shape[1] / 2
                    centre_y = corner[1] + template.shape[0] / 2
                    height, width = reference.shape[:2]
                    # image y points down, north points up
                    offset = np.array([-(centre_y - height / 2), centre_x - width / 2]) * metres_per_pixel
                    best = Match(float(score), reference_map.position[index] + offset, float(zoom), float(angle), index)
    if best is None:
        raise ValueError("no template fitted into the reference images; lower the zoom")
    return best


def search_area(
    frame: np.ndarray,
    ground: GroundMap,
    centre: np.ndarray,
    radius_m: float,
    zooms: Iterable[float],
    angles: Iterable[float],
    keep: float = KEEP,
    min_covered: float = 0.98,
) -> Match | None:
    """Find a prepared camera frame in one map, within ``radius_m`` of ``centre``.

    The search area is a circle around the navigator's own estimate, so it knows nothing about
    where the true path runs. Only places where the frame would lie fully on imagery count
    (``min_covered`` of its pixels). Returns ``None`` if no such place lies in the circle.

    Example with 0.6 m per pixel: a radius of 60 m is 100 pixels, so the centre of the frame may
    land anywhere in a circle 200 pixels across around the estimate.
    """
    image = ground.prepared()
    covered = ground.covered_float()
    cx, cy = ground.to_pixel(centre)
    reach = radius_m / ground.metres_per_pixel
    best: Match | None = None
    for zoom in zooms:
        for angle in angles:
            template = make_template(frame, float(zoom), float(angle), ground.zoom_unit_px, keep, inscribed=True)
            th, tw = template.shape
            left = max(0, int(np.floor(cx - reach - tw / 2)))
            top = max(0, int(np.floor(cy - reach - th / 2)))
            right = min(image.shape[1], int(np.ceil(cx + reach + tw / 2)) + 1)
            bottom = min(image.shape[0], int(np.ceil(cy + reach + th / 2)) + 1)
            if right - left < tw or bottom - top < th:
                continue
            scores = cv2.matchTemplate(image[top:bottom, left:right], template, cv2.TM_CCOEFF_NORMED)
            on_map = cv2.matchTemplate(covered[top:bottom, left:right], np.ones_like(template), cv2.TM_CCORR) / template.size
            xs = left + np.arange(scores.shape[1]) + tw / 2  # centre of the template, map pixels
            ys = top + np.arange(scores.shape[0]) + th / 2
            outside = np.hypot(xs[None, :] - cx, ys[:, None] - cy) > reach
            scores = np.where(outside | (on_map < min_covered) | ~np.isfinite(scores), -np.inf, scores)
            iy, ix = np.unravel_index(int(np.argmax(scores)), scores.shape)
            score = float(scores[iy, ix])
            if np.isfinite(score) and (best is None or score > best.score):
                best = Match(score, ground.to_position(xs[ix], ys[iy]), float(zoom), float(angle), -1)
    return best

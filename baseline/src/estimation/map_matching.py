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

KEEP = 0.8  # share of the scaled and turned frame used as the template; the corners are cut off


@dataclass
class Match:
    """The best place found for one camera frame."""

    score: float  # normalised correlation at that place, between -1 and 1
    position: np.ndarray  # (2,) north, east of the centre of the camera frame, m
    zoom: float
    angle: float  # degrees
    reference_index: int  # the reference image in which the place was found


def make_template(frame: np.ndarray, zoom: float, angle: float, reference_side: int, keep: float = KEEP) -> np.ndarray:
    """Bring a prepared camera frame to the scale and orientation of the reference images.

    Args:
        frame: camera frame, prepared with ``camera_flight.prepare``.
        zoom: the frame shows ``zoom`` times the width of a reference image.
        angle: degrees to turn the frame, counter-clockwise.
        reference_side: width of a reference image in pixels.
        keep: share of the result that is kept, cut from its centre.
    """
    n = int(round(reference_side * zoom))
    scaled = cv2.resize(frame, (n, n), interpolation=cv2.INTER_AREA)
    turned = cv2.warpAffine(scaled, cv2.getRotationMatrix2D((n / 2, n / 2), angle, 1.0), (n, n))
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

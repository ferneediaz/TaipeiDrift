"""A phone camera as a sun compass: the mentor's suggestion, to try the sun sensor's idea on real hardware.

Setup: the phone lies face down on a level surface, so its back camera looks straight up. Its
photo shows the sun somewhere in the sky. Three steps turn that into a heading:

1. **Calibration** (OpenCV, photos of a chessboard): the camera's focal length, centre and lens
   distortion, so that every pixel can be turned into a direction.
2. **Where the sun appears:** the centre of the brightest patch in the photo, turned into a
   direction with the calibration.
3. **Where the sun truly stands:** from the photo's time and GPS position (``heading.sun_position``).
   The difference between the two is the phone's heading.

The geometry, for a camera looking straight up. Let psi be the bearing of the image's up direction
(the heading we want), and the sun stand at azimuth A and elevation e. In normalised image
coordinates (x to the right, y down, divided by the focal length) the sun appears at

    x = -cot(e) * sin(A - psi)
    y = -cot(e) * cos(A - psi)

so psi = A - atan2(-x, -y), and the elevation can be read back as e = atan(1 / sqrt(x^2 + y^2)).

Example: image up pointing north (psi = 0), sun due south (A = 180) at 45 degrees: x = 0, y = 1,
the sun appears straight below the centre. With the image up pointing east (psi = 90), the same sun
is at x = -1, y = 0, left of the centre. Looking up flips east and west compared with a map.

The measured elevation is a free check: if it disagrees with the true one, the phone was not level
or the calibration is off.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

CHESSBOARD_INNER_CORNERS = (9, 6)


@dataclass
class CameraModel:
    """Result of the calibration: pixel coordinates to directions."""

    matrix: np.ndarray  # (3, 3) focal lengths and centre in pixels
    distortion: np.ndarray  # lens distortion coefficients
    image_size: tuple[int, int]  # (width, height) in pixels
    rms_px: float  # how far the chessboard corners land from the model, pixels

    def normalised(self, x_px: float, y_px: float) -> tuple[float, float]:
        """A pixel to normalised coordinates (x right, y down, per unit of focal length), distortion removed."""
        point = np.array([[[x_px, y_px]]], np.float64)
        x, y = cv2.undistortPoints(point, self.matrix, self.distortion)[0, 0]
        return float(x), float(y)


def calibrate(images: list[np.ndarray], inner_corners: tuple[int, int] = CHESSBOARD_INNER_CORNERS) -> tuple[CameraModel, int]:
    """Calibrate from greyscale photos of a chessboard. Returns the model and how many photos were usable."""
    pattern = np.zeros((inner_corners[0] * inner_corners[1], 3), np.float32)
    pattern[:, :2] = np.mgrid[0 : inner_corners[0], 0 : inner_corners[1]].T.reshape(-1, 2)
    object_points, image_points, size = [], [], None
    for gray in images:
        size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCorners(gray, inner_corners, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3))
        object_points.append(pattern)
        image_points.append(corners)
    if len(object_points) < 5:
        raise ValueError(f"the chessboard was found in {len(object_points)} photos; at least 5 are needed")
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(object_points, image_points, size, None, None)
    return CameraModel(matrix, distortion, size, float(rms)), len(object_points)


def find_sun(gray: np.ndarray) -> tuple[float, float, int]:
    """Centre (x, y) in pixels of the largest saturated patch, and its size in pixels.

    The sun saturates the sensor even with the exposure turned down. Lens flares make smaller
    bright patches; the largest one is the sun.
    """
    level = max(int(gray.max()) - 5, 200)
    _, mask = cv2.threshold(gray, level, 255, cv2.THRESH_BINARY)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        raise ValueError("no bright patch found; is the sun in the picture?")
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y = centroids[largest]
    return float(x), float(y), int(stats[largest, cv2.CC_STAT_AREA])


def heading_from_sun(x: float, y: float, sun_azimuth_deg: float) -> tuple[float, float]:
    """Bearing of the image's up direction and the sun's elevation as measured, both in degrees.

    ``x, y``: the sun in normalised coordinates of a camera looking straight up. See the module
    docstring for the geometry and an example.
    """
    relative = np.degrees(np.arctan2(-x, -y))  # A - psi
    heading = (sun_azimuth_deg - relative) % 360.0
    elevation = float(np.degrees(np.arctan(1.0 / max(np.hypot(x, y), 1e-9))))
    return float(heading), elevation


def read_gray(path: Path) -> np.ndarray:
    """A photo as greyscale pixels in the sensor's own orientation (the phone's rotation flag is ignored).

    A phone lying flat sets that flag at random; following it would add false quarter turns.
    """
    gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE | cv2.IMREAD_IGNORE_ORIENTATION)
    if gray is None:
        raise ValueError(f"cannot read {path}")
    return gray

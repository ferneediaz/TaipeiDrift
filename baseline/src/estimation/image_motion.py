"""Image motion between consecutive camera frames.

The ground slides through the image as the aircraft moves. The median shift of the image
content, in pixels, is what the camera measures of that motion. It says nothing about metres
yet: ``navigator_core.fit_motion_matrix`` learns that link while GNSS still works.

The shift is computed on reduced frames (about 250 pixels wide) with dense optical flow, and
only the centre of the frame is used, where the lens distorts least.
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from src.data.camera_flight import CameraFlight

WORK_SIDE_PX = 250  # frames are reduced to this size before the flow is computed
CENTRE = (0.24, 0.76)  # part of the reduced frame used for the median: 60 to 190 of 250 px


def _reduce(frame: np.ndarray) -> tuple[np.ndarray, float]:
    """Shrink a frame for the flow computation. Returns the small frame and the factor back to full size."""
    height, width = frame.shape[:2]
    factor = max(height, width) / WORK_SIDE_PX
    if factor <= 1.0:
        return frame, 1.0
    size = (int(round(width / factor)), int(round(height / factor)))
    return cv2.resize(frame, size, interpolation=cv2.INTER_AREA), factor


def _median_shift(previous: np.ndarray, current: np.ndarray, factor: float) -> np.ndarray:
    """Median optical flow over the centre of two reduced frames, in full-size pixels."""
    flow = cv2.calcOpticalFlowFarneback(previous, current, None, 0.5, 4, 21, 3, 7, 1.5, 0)
    height, width = flow.shape[:2]
    rows = slice(int(round(CENTRE[0] * height)), int(round(CENTRE[1] * height)))
    cols = slice(int(round(CENTRE[0] * width)), int(round(CENTRE[1] * width)))
    return np.median(flow[rows, cols].reshape(-1, 2), axis=0) * factor


def image_shift(previous: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Shift of the image content from one grayscale frame to the next.

    Returns:
        (2,) pixels of the full-size frame: x to the right, y downwards.
    """
    small_previous, _ = _reduce(previous)
    small_current, factor = _reduce(current)
    return _median_shift(small_previous, small_current, factor)


def shifts_for_flight(flight: CameraFlight, cache_path: str | Path | None = None) -> np.ndarray:
    """Image shift for every frame of a flight.

    Row ``k`` is the shift from frame ``k - 1`` to frame ``k``; row 0 is zero. The result is
    stored at ``cache_path`` and read from there the next time, since it only depends on the frames.

    Returns:
        (N, 2) pixels.
    """
    if cache_path is not None and Path(cache_path).is_file():
        cached = np.load(cache_path)
        if cached.shape == (len(flight), 2):
            return cached
    shifts = np.zeros((len(flight), 2))
    previous = None
    for k in range(len(flight)):
        current, factor = _reduce(flight.frame(k))
        if previous is not None:
            shifts[k] = _median_shift(previous, current, factor)
        previous = current
    if cache_path is not None:
        Path(cache_path).parent.mkdir(parents=True, exist_ok=True)
        np.save(cache_path, shifts)
    return shifts


def picture_detail(frame: np.ndarray) -> float:
    """How much fine detail a frame holds: the spread of its texture 1 to 4 pixels across, over its mean brightness.

    Fog, haze and blur wash this detail out; a frame that is only darker or brighter keeps it. Measured on the
    simulated flight over Wufeng with the realistic camera: about 0.071 in clear air, 0.038 in fog with 1 km of
    visibility (54 percent of it), 0.015 at 300 m (21 percent).
    """
    g = frame.astype(np.float32)
    band = cv2.GaussianBlur(g, (0, 0), 1.0) - cv2.GaussianBlur(g, (0, 0), 4.0)
    return float(np.std(band) / max(float(np.mean(g)), 1.0))


def detail_for_flight(flight: CameraFlight, cache_path: str | Path | None = None) -> np.ndarray:
    """Picture detail of every frame of a flight (see ``picture_detail``), stored at ``cache_path`` like the shifts."""
    if cache_path is not None and Path(cache_path).is_file():
        cached = np.load(cache_path)
        if cached.shape == (len(flight),):
            return cached
    detail = np.array([picture_detail(flight.frame(k)) for k in range(len(flight))])
    if cache_path is not None:
        Path(cache_path).parent.mkdir(parents=True, exist_ok=True)
        np.save(cache_path, detail)
    return detail

"""A small synthetic camera flight over a generated ground texture.

It has the same shape as the ALTO data, so the navigator can be tested without the 1.7 GB
download: reference images with north at the top, one every 10 m along the route, and camera
frames that are rotated, show less ground and have a different brightness.
"""
from __future__ import annotations

import cv2
import numpy as np

from src.data.camera_flight import CameraFlight, ReferenceMap

MAP_METRES_PER_PIXEL = 0.6
IMAGE_PX = 160  # side of reference images and camera frames


def _ground_texture(rng: np.random.Generator, height: int, width: int) -> np.ndarray:
    """Noise at several scales plus a few sharp shapes, so that places can be told apart."""
    texture = np.zeros((height, width), np.float32)
    for cells in (6, 14, 40, 110, 300):
        layer = rng.random((max(2, cells * height // width), cells), dtype=np.float32)
        texture += cv2.resize(layer, (width, height), interpolation=cv2.INTER_CUBIC)
    texture = (texture - texture.min()) / (texture.max() - texture.min())
    image = (texture * 200 + 20).astype(np.uint8)
    for _ in range(120):  # "roads" and "buildings"
        x, y = int(rng.integers(0, width)), int(rng.integers(0, height))
        shade = int(rng.integers(0, 256))
        if rng.random() < 0.5:
            x2, y2 = x + int(rng.integers(-150, 150)), y + int(rng.integers(-150, 150))
            cv2.line(image, (x, y), (x2, y2), shade, int(rng.integers(1, 4)))
        else:
            w, h = int(rng.integers(5, 25)), int(rng.integers(5, 25))
            cv2.rectangle(image, (x, y), (x + w, y + h), shade, -1)
    return image


def make_synthetic_camera_flight(
    length_m: float = 900.0,
    step_m: float = 3.0,
    zoom: float = 0.85,
    rotation_deg: float = 15.0,
    seed: int = 0,
) -> CameraFlight:
    """Build a camera flight heading roughly east, with a gentle sideways curve.

    Args:
        length_m: distance flown.
        step_m: distance between camera frames.
        zoom: a camera frame shows ``zoom`` times the width of a reference image.
        rotation_deg: the frame has to be turned by this angle (counter-clockwise) to put north at the top.
        seed: seed of the ground texture.
    """
    rng = np.random.default_rng(seed)
    mpp = MAP_METRES_PER_PIXEL
    margin = 150.0  # m of ground around the route
    width = int((length_m + 2 * margin) / mpp)
    height = int((2 * margin + 60.0) / mpp)
    ground = _ground_texture(rng, height, width)

    def to_pixel(position: np.ndarray) -> tuple[float, float]:
        north, east = position
        return (east + margin) / mpp, height / 2 - north / mpp  # x to the right, y downwards

    def crop_north_up(position: np.ndarray, side: int) -> np.ndarray:
        x, y = to_pixel(position)
        return cv2.getRectSubPix(ground, (side, side), (x, y))

    east = np.arange(0.0, length_m + 1e-6, step_m)
    north = 25.0 * np.sin(east / length_m * np.pi)  # a shallow arc
    position = np.column_stack([north, east])

    reference_east = np.arange(0.0, length_m + 1e-6, 10.0)
    reference_position = np.column_stack([25.0 * np.sin(reference_east / length_m * np.pi), reference_east])

    def load_reference(i: int) -> np.ndarray:
        return crop_north_up(reference_position[i], IMAGE_PX)

    def load_frame(i: int) -> np.ndarray:
        side = int(np.ceil(IMAGE_PX * zoom * 1.5)) | 1
        patch = crop_north_up(position[i], side)
        turn = cv2.getRotationMatrix2D((side / 2, side / 2), -rotation_deg, 1.0)
        patch = cv2.warpAffine(patch, turn, (side, side))
        inner = int(round(IMAGE_PX * zoom))
        o = (side - inner) // 2
        frame = cv2.resize(patch[o : o + inner, o : o + inner], (IMAGE_PX, IMAGE_PX), interpolation=cv2.INTER_LINEAR)
        frame = np.clip(frame.astype(np.float32) * 1.15 + 12.0, 0, 255)  # brighter, as if overexposed
        return frame.astype(np.uint8)

    return CameraFlight(
        name="synthetic_camera",
        timestamp=np.arange(len(position)) / 20.0,
        position_gt=position - position[0],
        load_frame=load_frame,
        reference=ReferenceMap(position=reference_position - position[0], metres_per_pixel=mpp, load=load_reference),
        metadata={"source": "synthetic", "zoom": zoom, "rotation_deg": rotation_deg, "seed": seed},
    )

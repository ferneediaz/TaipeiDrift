"""A realistic camera for the simulated flights: what the simulator's perfect pinhole camera leaves out.

The simulator renders each down-camera frame as an ideal pinhole would see a sunlit, still world: no
noise, no lens faults, no weather. A real camera on a real drone records something worse. This module
turns a recorded frame into such a recording, in the order the light travels:

1. **Cloud shadows on the ground.** Clouds cover a share of the sky; their shadows darken the ground in
   patches a few hundred metres across and drift with the wind. The old map has none of them, or other
   ones. The shadows are drawn in ground coordinates, so a field stays in the same shadow in
   consecutive frames, as in reality. Example: a shadow that removes 35 percent of the light turns a
   field of brightness 120 into 78.
2. **Haze.** Bright air between the drone and the ground leaves only a share of the contrast (see degrade.py).
3. **The lens.** Vignetting: the corners of the image receive less light; here the corner of the full frame
   loses ``vignetting`` of it, and points in between lose in proportion to their squared distance from the
   centre. A small distortion stays after calibration: a point at the image edge (256 pixels from the
   centre) lands ``distortion_k1 * 256`` pixels further out, 1 pixel for 0.004. Vibration and focus
   blur the image by ``blur_px``.
4. **Auto-exposure.** The camera keeps the mean brightness near a target: when a dark cloud shadow fills
   the view, it raises the gain, so the parts in the sun get brighter too; and the gain wobbles a little
   from frame to frame. (A real camera adapts with a lag of a few frames; left out, so that every frame
   can be made on its own, in any order.)
5. **Sensor noise**, as in degrade.py: the count of light particles wobbles by its square root.
6. **Compression.** The frame is stored as JPEG at ``jpeg_quality``, as a small on-board camera would.

Left out: rolling shutter (rows read one after another while the drone moves), raindrops and glare.

Everything is drawn from ``seed`` and the frame index, so a frame read twice is identical and a
different seed is another flight under other clouds.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass

import cv2
import numpy as np

from src.data.degrade import AIRLIGHT, less_light

CLOUD_GRID_M = 10.0  # resolution of the cloud shadow field
CLOUD_FIELD_M = 12_000.0  # side of the (repeating) cloud field: covers a flight and the drift of the clouds


@dataclass(frozen=True)
class CameraModel:
    cloud_cover: float = 0.3  # share of the ground in cloud shadow
    cloud_shadow: float = 0.35  # share of the light a cloud shadow takes away
    cloud_size_m: float = 400.0  # typical size of a cloud shadow
    wind_east_mps: float = 4.0  # the clouds drift with the wind
    wind_north_mps: float = 2.0
    haze_contrast: float = 0.9  # share of the contrast left by the haze
    vignetting: float = 0.4  # share of the light lost in the corners of the full frame
    distortion_k1: float = 0.004  # radial distortion left after calibration
    blur_px: float = 0.6  # vibration and focus, Gaussian sigma in pixels
    gain_wobble: float = 0.05  # frame-to-frame wobble of the auto-exposure, as a share of the gain
    light: float = 1.0  # light relative to a bright day (sets the noise; 0.25 is dusk)
    jpeg_quality: int = 80
    seed: int = 0

    def key(self) -> str:
        """A short fingerprint of the settings, for cache file names."""
        import hashlib

        return hashlib.sha1(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()[:10]


def cloud_field(model: CameraModel) -> np.ndarray:
    """Shadow strength on a repeating grid of ground cells: 0 in the sun, up to ``cloud_shadow`` in full shade."""
    n = int(CLOUD_FIELD_M / CLOUD_GRID_M)
    rng = np.random.default_rng((model.seed, 1))
    # white noise blurred to cloud-sized blobs; blurred in the frequency domain, so the field repeats without a seam
    sigma = model.cloud_size_m / CLOUD_GRID_M / 3.0
    f = np.fft.fftfreq(n)
    transfer = np.exp(-2.0 * (np.pi * sigma) ** 2 * (f[:, None] ** 2 + f[None, :] ** 2))
    noise = np.real(np.fft.ifft2(np.fft.fft2(rng.standard_normal((n, n))) * transfer)).astype(np.float32)
    threshold = np.quantile(noise, 1.0 - model.cloud_cover)
    edge = 0.25 * float(np.std(noise))  # soft edges: half shade over a few tens of metres
    shade = np.clip((noise - threshold) / edge + 0.5, 0.0, 1.0)
    return (model.cloud_shadow * shade).astype(np.float32)


class RealisticCamera:
    """Turns ideal frames of one flight into realistic ones, each frame on its own."""

    def __init__(self, model: CameraModel, timestamp: np.ndarray, position_ne: np.ndarray, heading_deg: np.ndarray,
                 height_m: np.ndarray, size_px: int = 512, focal_px: float = 256.0):
        self.model = model
        self.t, self.position, self.heading, self.height = timestamp, position_ne, heading_deg, height_m
        self.size, self.focal = size_px, focal_px
        self.clouds = cloud_field(model)
        c = (size_px - 1) / 2.0
        v, u = np.mgrid[0:size_px, 0:size_px].astype(np.float32)
        self.forward_px, self.right_px = c - v, u - c  # image top points forward, right is the body's right
        r2 = (self.forward_px**2 + self.right_px**2) / (c * c)  # 1 at the middle of an edge, 2 in a corner
        self.vignette = (1.0 - model.vignetting * r2 / 2.0).astype(np.float32)
        # distortion: the recorded pixel at radius r shows what the ideal camera saw at r * (1 + k1 r^2) / ... inverted
        # to first order: sample the ideal image at r / (1 + k1 r^2)
        scale = 1.0 / (1.0 + model.distortion_k1 * r2)
        self.map_x = (c + self.right_px * scale).astype(np.float32)
        self.map_y = (c - self.forward_px * scale).astype(np.float32)

    def shadow(self, i: int) -> np.ndarray:
        """Cloud shadow strength under every pixel of frame ``i``."""
        m = self.model
        metres_per_px = self.height[i] / self.focal
        a = np.radians(self.heading[i])
        fwd, right = self.forward_px * metres_per_px, self.right_px * metres_per_px
        north = self.position[i, 0] + fwd * np.cos(a) - right * np.sin(a) - m.wind_north_mps * self.t[i]
        east = self.position[i, 1] + fwd * np.sin(a) + right * np.cos(a) - m.wind_east_mps * self.t[i]
        n = self.clouds.shape[0]
        col = np.mod(east / CLOUD_GRID_M, n).astype(np.float32)
        row = np.mod(-north / CLOUD_GRID_M, n).astype(np.float32)
        return cv2.remap(self.clouds, col, row, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)

    def frame(self, i: int, ideal: np.ndarray) -> np.ndarray:
        """Frame ``i`` as the realistic camera records it, from the simulator's ideal grey frame."""
        m = self.model
        rng = np.random.default_rng((m.seed, 2, i))
        light = ideal.astype(np.float32) * (1.0 - self.shadow(i))
        light = m.haze_contrast * light + (1.0 - m.haze_contrast) * AIRLIGHT
        light = cv2.remap(light, self.map_x, self.map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        light *= self.vignette
        if m.blur_px > 0:
            light = cv2.GaussianBlur(light, (0, 0), m.blur_px)
        gain = 115.0 / max(float(np.mean(light)), 1.0) * float(np.exp(rng.normal(0.0, m.gain_wobble)))
        exposed = np.clip(light * gain, 0, 255).astype(np.uint8)
        noisy = less_light(exposed, m.light, rng)
        _, jpeg = cv2.imencode(".jpg", noisy, [cv2.IMWRITE_JPEG_QUALITY, m.jpeg_quality])
        return cv2.imdecode(jpeg, cv2.IMREAD_GRAYSCALE)

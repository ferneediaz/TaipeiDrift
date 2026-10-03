"""Make camera frames worse, the way bad light, blur and haze would, for the limits test.

The matcher compares brightness patterns after removing each patch's own brightness and contrast,
so a frame that is only darker or only paler matches exactly as well. What hurts is noise: a
camera in dim light turns up its gain, and the picture gets grainy. Each degradation therefore
ends with the noise a real sensor would add.

**Less light.** A pixel counts light particles. Say a mid-grey pixel collects 1,000 of them in the
recorded flight. The count wobbles by about its square root: 1,000 plus or minus 32, about 3
percent. With 64 times less light it collects about 16, plus or minus 4: 25 percent. The camera
brightens the picture back, but the wobble stays. Modelled with 2,000 particles for a white pixel
at the recorded light and a read noise of 2 particles.

**Blur.** Defocus, vibration or long exposure, as a Gaussian blur given in metres on the ground.

**Haze.** A veil of bright air: each pixel becomes t * (its brightness) + (1 - t) * 230, so only
the share t of the contrast is left; then the sensor noise of the recorded light is added.
"""
from __future__ import annotations

import cv2
import numpy as np

FULL_WELL = 2000.0  # light particles collected by a white pixel at the recorded light
READ_NOISE = 2.0  # particles
AIRLIGHT = 230.0  # brightness of the haze veil


def less_light(frame: np.ndarray, light: float, rng: np.random.Generator) -> np.ndarray:
    """The frame as a camera would record it with ``light`` times the light (1 = as recorded)."""
    particles = frame.astype(np.float64) / 255.0 * FULL_WELL * light
    counted = rng.poisson(particles) + rng.normal(0.0, READ_NOISE, frame.shape)
    return np.clip(counted / (FULL_WELL * light) * 255.0, 0, 255).astype(np.uint8)


def blur(frame: np.ndarray, sigma_m: float, metres_per_pixel: float) -> np.ndarray:
    """Gaussian blur of ``sigma_m`` metres on the ground."""
    if sigma_m <= 0:
        return frame
    sigma_px = sigma_m / metres_per_pixel
    return cv2.GaussianBlur(frame, (0, 0), sigma_px)


def haze(frame: np.ndarray, contrast_left: float, rng: np.random.Generator) -> np.ndarray:
    """A bright veil that leaves ``contrast_left`` of the contrast, plus the sensor noise of the recorded light."""
    veiled = contrast_left * frame.astype(np.float64) + (1.0 - contrast_left) * AIRLIGHT
    return less_light(np.clip(veiled, 0, 255).astype(np.uint8), 1.0, rng)


def degrade(frame: np.ndarray, kind: str, level: float, index: int, metres_per_pixel: float, seed: int = 0) -> np.ndarray:
    """One frame, made worse; the noise depends only on the frame index, so a frame read twice is identical."""
    rng = np.random.default_rng((seed, index))
    if kind == "light":
        return less_light(frame, level, rng)
    if kind == "blur":
        return less_light(blur(frame, level, metres_per_pixel), 1.0, rng)
    if kind == "haze":
        return haze(frame, level, rng)
    raise ValueError(f"unknown degradation {kind!r}")

"""Simple, explainable sun detector for a forward RGB camera, plus the pixel -> ray geometry.

Pipeline (BGR image):
1. sky mask: bright, blue-dominant or near-white pixels connected to the top of the image;
2. saturated mask: min(B, G, R) >= SAT (the sun disk clips every channel);
3. candidates: connected saturated regions with
     area within [MIN_AREA, MAX_AREA] px,
     circularity 4 pi A / perimeter^2 >= MIN_CIRCULARITY (the disk is round; clouds are not),
     mean brightness of a surrounding ring >= MIN_RING (the sun blooms; a white rock does not),
     fraction of the ring that is sky or saturated >= MIN_SKY_ADJ (it sits in the sky);
4. the best candidate by confidence = circularity * ring/255 * sky adjacency.
No learning; the brightest pixel alone is never trusted.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

SAT = 250
MIN_AREA, MAX_AREA = 150, 30000  # the sun shows as a large clipped bloom; small clipped blobs are clouds
MIN_CIRCULARITY = 0.3  # bloom merges with nearby bright sky
MIN_RING = 215.0
MIN_SKY_ADJ = 0.5


@dataclass
class SunDetection:
    u: float
    v: float
    confidence: float
    area: float
    circularity: float
    ring_brightness: float
    sky_adjacency: float


def sky_mask(bgr: np.ndarray) -> np.ndarray:
    b, g, r = [bgr[..., i].astype(np.int16) for i in range(3)]
    v = bgr.max(axis=2)
    candidate = ((v >= 120) & (b >= r) & (b >= g - 10)) | (bgr.min(axis=2) >= 200)
    n, lab = cv2.connectedComponents(candidate.astype(np.uint8))
    top = np.unique(lab[0][candidate[0]])
    return np.isin(lab, top[top > 0])


def detect_sun(bgr: np.ndarray) -> tuple[SunDetection | None, float, int]:
    """Return (best detection or None, sky fraction, number of saturated candidates examined)."""
    sky = sky_mask(bgr)
    sat = (bgr.min(axis=2) >= SAT).astype(np.uint8)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(sat)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    best = None
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if not MIN_AREA <= area <= MAX_AREA:
            continue
        comp = (lab == i).astype(np.uint8)
        cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        per = max(cv2.arcLength(cnts[0], True), 1.0)
        circ = float(min(1.0, 4 * np.pi * area / per**2))
        if circ < MIN_CIRCULARITY:
            continue
        rad = max(3, int(np.sqrt(area / np.pi)))
        ring = cv2.dilate(comp, np.ones((2 * rad + 1, 2 * rad + 1), np.uint8)).astype(bool) & ~comp.astype(bool)
        if not ring.any():
            continue
        ring_b = float(gray[ring].mean())
        adj = float((sky[ring] | (sat[ring] > 0)).mean())
        if ring_b < MIN_RING or adj < MIN_SKY_ADJ:
            continue
        conf = circ * ring_b / 255.0 * adj
        if best is None or conf > best.confidence:
            best = SunDetection(float(cents[i][0]), float(cents[i][1]), conf, float(area), circ, ring_b, adj)
    return best, float(sky.mean()), int(n - 1)


def pixel_to_camera_ray(u: float, v: float, K: np.ndarray) -> np.ndarray:
    """Unit ray in camera axes (x right, y down, z optical axis) through pixel (u, v)."""
    z = np.linalg.solve(K, np.array([u, v, 1.0]))
    return z / np.linalg.norm(z)


def camera_to_body_ray(s_cam: np.ndarray, R_bc: np.ndarray) -> np.ndarray:
    return np.asarray(R_bc, float) @ np.asarray(s_cam, float)


def sun_vector_ned(azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    """Unit vector TOWARDS the sun in NED: azimuth from north towards east, elevation above the horizon."""
    a, e = np.radians(azimuth_deg), np.radians(elevation_deg)
    return np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), -np.sin(e)])


def azimuth_elevation_ned(s: np.ndarray) -> tuple[float, float]:
    s = np.asarray(s, float) / np.linalg.norm(s)
    return float(np.degrees(np.arctan2(s[1], s[0])) % 360.0), float(np.degrees(np.arcsin(-s[2])))

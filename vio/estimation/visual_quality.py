"""Quality-conditioned noise for relative visual rotations. One rule for every camera.

Only the quality signals that transferred between Mid-Air and NTU are used (visual-rotation
diagnostic): image motion, RANSAC inlier ratio and track count. Image motion is expressed as an
ANGLE (median pixel flow / focal length in pixels), so the rule does not depend on resolution.

    q = 1 + alpha * flow_rad / FLOW_REF
          + beta  * max(0, INLIER_REF - inlier_ratio) / 0.1
          + gamma * max(0, sqrt(TRACKS_REF / tracks) - 1)
    sigma = max(SIGMA_FLOOR_DEG, sigma0 * q)          (per axis, per interval)

Every term is non-negative and monotone: worse quality can only raise sigma. The reference
constants are fixed physical values; sigma0, alpha, beta, gamma are fitted once on Mid-Air
development flights (non-negative least squares) and frozen before any NTU run.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from scipy.optimize import nnls

FLOW_REF_RAD = 0.05  # ~3 deg of image motion per interval
INLIER_REF = 0.95
TRACKS_REF = 200.0
SIGMA_FLOOR_DEG = 0.1
CHI3_MEAN = 1.5958  # mean norm of a 3-D standard normal vector: E|e| = 1.596 sigma


def quality_terms(flow_rad, inlier_ratio, tracks) -> np.ndarray:
    flow_rad, inlier_ratio, tracks = np.broadcast_arrays(np.asarray(flow_rad, float), np.asarray(inlier_ratio, float),
                                                         np.asarray(tracks, float))
    return np.stack([np.maximum(flow_rad, 0.0) / FLOW_REF_RAD,
                     np.maximum(0.0, INLIER_REF - inlier_ratio) / 0.1,
                     np.maximum(0.0, np.sqrt(TRACKS_REF / np.maximum(tracks, 1.0)) - 1.0)], axis=-1)


@dataclass(frozen=True)
class QualityModel:
    sigma0_deg: float
    alpha_flow: float
    beta_inlier: float
    gamma_tracks: float

    def scale(self, flow_rad, inlier_ratio, tracks):
        t = quality_terms(flow_rad, inlier_ratio, tracks)
        return 1.0 + t @ np.array([self.alpha_flow, self.beta_inlier, self.gamma_tracks])

    def sigma_deg(self, flow_px, focal_px, inlier_ratio, tracks):
        """Per-axis noise for one interval. ``flow_px / focal_px`` makes it camera-independent."""
        return np.maximum(SIGMA_FLOOR_DEG, self.sigma0_deg * self.scale(np.asarray(flow_px) / focal_px, inlier_ratio, tracks))

    def as_dict(self) -> dict:
        return {**asdict(self), "flow_ref_rad": FLOW_REF_RAD, "inlier_ref": INLIER_REF, "tracks_ref": TRACKS_REF,
                "sigma_floor_deg": SIGMA_FLOOR_DEG}


def fit_quality_model(err_norm_deg, flow_rad, inlier_ratio, tracks) -> QualityModel:
    """Fit E|e| = 1.596 sigma0 (1 + alpha t1 + beta t2 + gamma t3) by non-negative least squares."""
    t = quality_terms(flow_rad, inlier_ratio, tracks)
    X = np.column_stack([np.ones(len(t)), t])
    c, _ = nnls(X, np.asarray(err_norm_deg, float) / CHI3_MEAN)
    s0 = max(c[0], 1e-6)
    return QualityModel(float(s0), float(c[1] / s0), float(c[2] / s0), float(c[3] / s0))

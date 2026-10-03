"""Relative camera motion from pixel correspondences (essential matrix + RANSAC).

Convention
----------
OpenCV's ``recoverPose`` returns (R, t) with ``X_b = R X_a + t`` for a 3D point
expressed in camera a (earlier frame) and camera b (later frame). This module
returns the orientation of camera b expressed in camera a,

    R_ab = R^T        (camera b axes -> camera a axes, i.e. R_wc_a^T R_wc_b)

and the direction of camera b's centre in camera a, ``-R^T t``, normalised.

Monocular scale
---------------
A single camera fixes translation only up to scale: ``translation_dir`` is a
unit vector, never a distance. Metric scale must come from elsewhere (the
inertial state in this project), never from ground truth after GNSS loss.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class PoseConfig:
    """RANSAC and validity thresholds."""

    ransac_threshold_px: float = 1.0
    ransac_confidence: float = 0.999
    min_correspondences: int = 30
    min_inliers: int = 30
    min_inlier_ratio: float = 0.5
    min_median_flow_px: float = 0.0  # below this, treat the pair as having no measurable motion


@dataclass
class RelativePose:
    """Relative motion between two frames, or the reason it is not available."""

    valid: bool
    rotation: np.ndarray  # (3, 3) R_ab, orientation of camera b in camera a; identity when invalid
    translation_dir: np.ndarray | None  # (3,) unit vector in camera a; scale unknown
    n_correspondences: int
    n_inliers: int
    inlier_ratio: float
    median_flow_px: float
    reason: str = ""  # why the pose is invalid; empty when valid


def _invalid(n: int, reason: str, inliers: int = 0, flow: float = 0.0) -> RelativePose:
    return RelativePose(False, np.eye(3), None, n, inliers, inliers / n if n else 0.0, flow, reason)


def estimate_relative_pose(pts_a: np.ndarray, pts_b: np.ndarray, K: np.ndarray, cfg: PoseConfig | None = None) -> RelativePose:
    """Estimate the motion of the camera from frame a to frame b.

    Args:
        pts_a, pts_b: (N, 2) matching pixel coordinates in frames a and b.
        K: (3, 3) intrinsic matrix of the (processed) image.
        cfg: thresholds.

    Never raises on degenerate input: it returns ``valid=False`` with a reason.
    """
    cfg = cfg or PoseConfig()
    n = len(pts_a)
    if n < max(cfg.min_correspondences, 5):
        return _invalid(n, "too few correspondences")
    pts_a = np.ascontiguousarray(pts_a, dtype=np.float64)
    pts_b = np.ascontiguousarray(pts_b, dtype=np.float64)
    flow = float(np.median(np.linalg.norm(pts_b - pts_a, axis=1)))
    if flow < cfg.min_median_flow_px:
        return _invalid(n, "no measurable motion", flow=flow)

    E, mask = cv2.findEssentialMat(pts_a, pts_b, K, method=cv2.RANSAC,
                                   prob=cfg.ransac_confidence, threshold=cfg.ransac_threshold_px)
    if E is None or E.shape != (3, 3):  # several solutions come back stacked; treat as ambiguous
        return _invalid(n, "essential matrix failed or ambiguous", flow=flow)
    n_inl = int(mask.sum())
    if n_inl < cfg.min_inliers or n_inl / n < cfg.min_inlier_ratio:
        return _invalid(n, "too few RANSAC inliers", n_inl, flow)

    good, R, t, _ = cv2.recoverPose(E, pts_a, pts_b, K, mask=mask.copy())
    if good < cfg.min_inliers // 2:
        return _invalid(n, "cheirality check failed", n_inl, flow)
    t_dir = (-R.T @ t).ravel()
    t_dir = t_dir / np.linalg.norm(t_dir)
    return RelativePose(True, R.T, t_dir, n, n_inl, n_inl / n, flow)


def homography_rotation_candidates(pts_a: np.ndarray, pts_b: np.ndarray, K: np.ndarray,
                                   cfg: PoseConfig | None = None) -> tuple[list[np.ndarray], RelativePose]:
    """Rotation hypotheses from a plane-induced homography (for a downward camera over flat ground).

    ``decomposeHomographyMat`` returns up to four (R, t, n) solutions. They are
    returned as candidate ``R_ab`` rotations; the caller picks one using its
    own prediction (the gyro), which needs no ground truth. The returned
    ``RelativePose`` carries the statistics and validity; its ``rotation`` is
    the identity and ``translation_dir`` is None because the choice is deferred.
    """
    cfg = cfg or PoseConfig()
    n = len(pts_a)
    if n < max(cfg.min_correspondences, 4):
        return [], _invalid(n, "too few correspondences")
    pts_a = np.ascontiguousarray(pts_a, dtype=np.float64)
    pts_b = np.ascontiguousarray(pts_b, dtype=np.float64)
    flow = float(np.median(np.linalg.norm(pts_b - pts_a, axis=1)))
    if flow < cfg.min_median_flow_px:
        return [], _invalid(n, "no measurable motion", flow=flow)
    H, mask = cv2.findHomography(pts_a, pts_b, cv2.RANSAC, cfg.ransac_threshold_px,
                                 confidence=cfg.ransac_confidence)
    if H is None:
        return [], _invalid(n, "homography failed", flow=flow)
    n_inl = int(mask.sum())
    if n_inl < cfg.min_inliers or n_inl / n < cfg.min_inlier_ratio:
        return [], _invalid(n, "too few RANSAC inliers", n_inl, flow)
    _, rotations, _, _ = cv2.decomposeHomographyMat(H, K)
    candidates = [R.T for R in rotations]  # same convention as estimate_relative_pose: R_ab
    return candidates, RelativePose(True, np.eye(3), None, n, n_inl, n_inl / n, flow)

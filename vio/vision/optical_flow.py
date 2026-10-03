"""Downward-camera optical flow -> camera velocity, given rotation and height above ground.

Front end (no estimator state, no ground truth): consecutive down-camera frames
are tracked (Shi-Tomasi + Lucas-Kanade, ``max_keyframe_age = 1``), a homography
is fitted with RANSAC, and only its inliers are kept, in normalised image
coordinates. Quality checks: number of tracks and RANSAC inlier ratio.

Measurement (needs the estimator's rotation, so it is evaluated inside the filter):

1. Derotation. With the relative camera rotation R_ab (camera b in camera a),
   a point seen along ray x_a would appear at pi(R_ab^T x_a) after a pure
   rotation. The translational flow is f = x_b - pi(R_ab^T x_a). R_ab comes
   from the filter's bias-corrected attitudes, i.e. from w_m - b_g.
2. Frames. After derotation everything is expressed in camera b's orientation:
   the derotated ray x' = pi(R_ab^T x_a), the plane normal n_b = R_ab^T n_a and
   the translation t_b = R_ab^T t_a. Mixing camera a and b axes would leak
   rotation x speed into the result.
3. Depth. The ground is a plane at height h below the camera with normal n_b
   (world down). A feature on ray x' has depth Z = h / (n_b . x'), which
   accounts for camera tilt.
4. Translation. For small motion,  f_x Z = -t_x + x' t_z,  f_y Z = -t_y + y' t_z.
   t_b is solved by least squares over all features (one outlier-trimming
   pass). v_cam = t_b / dt, in camera b axes, matching the filter's
   measurement model at the time of frame b. Only the lateral components
   (camera x, y) are used.

Lever arm: Mid-Air gives ``local_position = (0, 0, 0)`` for color_down, so the
camera sits at the IMU origin and no w x r term is needed.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from vio.vision.feature_tracker import TrackResult


@dataclass
class FlowConfig:
    min_tracks: int = 40
    min_inlier_ratio: float = 0.5  # homography inliers: the share of points on the dominant ground plane
    ransac_threshold_px: float = 1.0
    max_residual_px: float = 1.0  # reject if the translational-flow fit leaves more than this (rms)
    min_ray_cos: float = 0.2  # ignore rays nearly parallel to the ground plane


@dataclass
class FlowPair:
    """Inlier correspondences between two consecutive down-camera frames."""

    frame_index: int
    imu_index: int
    prev_imu_index: int
    xa: np.ndarray  # (N, 2) normalised coordinates in the earlier frame
    xb: np.ndarray  # (N, 2) normalised coordinates in the later frame
    n_tracked: int
    inlier_ratio: float
    valid: bool
    reason: str = ""


@dataclass
class FlowVelocity:
    v_cam: np.ndarray  # (3,) camera velocity in camera a axes, m/s
    cov_xy: np.ndarray  # (2, 2) covariance of the lateral components from the fit alone
    residual_px: float
    n_used: int


def flow_pairs_from_tracks(tracks: list[TrackResult], K: np.ndarray, frame_to_imu, cfg: FlowConfig | None = None) -> list[FlowPair]:
    """Quality-check consecutive-frame tracks and convert inliers to normalised coordinates."""
    cfg = cfg or FlowConfig()
    Kinv = np.linalg.inv(K)
    out = []
    for tr in tracks:
        if tr.new_keyframe:
            continue
        imu, prev = int(frame_to_imu(tr.frame_index)), int(frame_to_imu(tr.keyframe_index))
        n = len(tr.current_points)
        empty = np.empty((0, 2))
        if n < cfg.min_tracks:
            out.append(FlowPair(tr.frame_index, imu, prev, empty, empty, n, 0.0, False, "too few tracks"))
            continue
        pa, pb = tr.keyframe_points.astype(np.float64), tr.current_points.astype(np.float64)
        H, mask = cv2.findHomography(pa, pb, cv2.RANSAC, cfg.ransac_threshold_px)
        if H is None:
            out.append(FlowPair(tr.frame_index, imu, prev, empty, empty, n, 0.0, False, "homography failed"))
            continue
        inl = mask.ravel().astype(bool)
        ratio = float(inl.mean())
        if ratio < cfg.min_inlier_ratio:
            out.append(FlowPair(tr.frame_index, imu, prev, empty, empty, n, ratio, False, "low inlier ratio"))
            continue
        to_norm = lambda p: (np.c_[p, np.ones(len(p))] @ Kinv.T)[:, :2]  # noqa: E731
        out.append(FlowPair(tr.frame_index, imu, prev, to_norm(pa[inl]), to_norm(pb[inl]), n, ratio, True))
    return out


def _rotate_rays(xa: np.ndarray, R_ab_cam: np.ndarray) -> np.ndarray:
    """Where a pure rotation moves each point: pi(R_ab^T x_a), in camera b axes."""
    rays = np.c_[xa, np.ones(len(xa))] @ R_ab_cam  # rows: (R_ab^T x_a)^T
    return rays[:, :2] / rays[:, 2:3]


def derotate(xa: np.ndarray, xb: np.ndarray, R_ab_cam: np.ndarray) -> np.ndarray:
    """Translational flow: observed position minus where a pure rotation would have moved the point."""
    return xb - _rotate_rays(xa, R_ab_cam)


def camera_velocity_from_flow(xa: np.ndarray, xb: np.ndarray, R_ab_cam: np.ndarray, n_cam: np.ndarray,
                              height: float, dt: float, focal_px: float,
                              cfg: FlowConfig | None = None) -> FlowVelocity | None:
    """Solve for the camera velocity (camera b axes) from derotated flow over a ground plane.

    ``n_cam`` is the world-down direction in camera a. Returns None if unusable.
    """
    cfg = cfg or FlowConfig()
    if height <= 0 or dt <= 0:
        return None
    xr = _rotate_rays(xa, R_ab_cam)
    f = xb - xr
    n_b = R_ab_cam.T @ n_cam
    cosr = np.c_[xr, np.ones(len(xr))] @ n_b
    keep = cosr > cfg.min_ray_cos
    if keep.sum() < 3:
        return None
    x, y, f, Z = xr[keep, 0], xr[keep, 1], f[keep], height / cosr[keep]
    A = np.zeros((2 * len(x), 3))
    A[0::2, 0], A[0::2, 2] = -1.0, x
    A[1::2, 1], A[1::2, 2] = -1.0, y
    b = (f * Z[:, None]).ravel()
    w = 1.0 / Z.repeat(2)  # residuals in flow units (normalised coordinates)
    for _ in range(2):
        t, *_ = np.linalg.lstsq(A * w[:, None], b * w, rcond=None)
        res = (A @ t - b) * w
        per = np.hypot(res[0::2], res[1::2])
        thr = 3.0 * max(np.median(per), 1e-9)
        good = (per <= thr).repeat(2)
        if good.all():
            break
        A, b, w = A[good], b[good], w[good]
    res = (A @ t - b) * w
    sigma = float(np.sqrt(np.mean(res**2)))
    residual_px = sigma * focal_px
    if residual_px > cfg.max_residual_px:
        return None
    Aw = A * w[:, None]
    cov_t = np.linalg.inv(Aw.T @ Aw) * max(sigma, 1e-12) ** 2
    return FlowVelocity(t / dt, cov_t[:2, :2] / dt**2, residual_px, len(A) // 2)


def height_from_known_velocity(xa: np.ndarray, xb: np.ndarray, R_ab_cam: np.ndarray, n_cam: np.ndarray,
                               t_cam: np.ndarray, cfg: FlowConfig | None = None) -> float | None:
    """Height above the ground plane that best explains the flow for a KNOWN translation.

    Used before the GNSS cutoff, with the GNSS velocity, to learn the starting height.
    ``n_cam`` (world down) and ``t_cam`` (camera displacement) are given in camera a;
    both are rotated into camera b axes as in ``camera_velocity_from_flow``.
    f = (n . x') A t / h  ->  1/h = (g . f) / (g . g),  g = (n . x') A t.
    """
    cfg = cfg or FlowConfig()
    xr = _rotate_rays(xa, R_ab_cam)
    f = xb - xr
    n_b, t_b = R_ab_cam.T @ n_cam, R_ab_cam.T @ t_cam
    cosr = np.c_[xr, np.ones(len(xr))] @ n_b
    keep = cosr > cfg.min_ray_cos
    if keep.sum() < 3:
        return None
    x, y = xr[keep, 0], xr[keep, 1]
    g = np.column_stack([-t_b[0] + x * t_b[2], -t_b[1] + y * t_b[2]]) * cosr[keep, None]
    gg = float(np.sum(g * g))
    if gg < 1e-12:
        return None
    inv_h = float(np.sum(g * f[keep])) / gg
    return 1.0 / inv_h if inv_h > 1e-6 else None

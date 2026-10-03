"""Live consecutive-frame down-camera flow front end (no GT access)."""
from __future__ import annotations

import cv2
import numpy as np

from vio.vision.optical_flow import FlowConfig, FlowPair, camera_velocity_from_flow


def range_jump_detected(previous_m, current_m, threshold_m):
    return (previous_m is not None and np.isfinite(current_m)
            and abs(float(current_m) - float(previous_m)) > float(threshold_m))


def flow_update_due(frame_index, every_n):
    """Decimate overlapping image-pair measurements to reduce temporal correlation."""
    return int(every_n) <= 1 or int(frame_index) % int(every_n) == 0


def track_pair(gray_a, gray_b, K, frame_index, prev_imu_index, imu_index, cfg=None):
    """Shi-Tomasi + forward/backward LK + homography RANSAC, normalized rays."""
    cfg = cfg or FlowConfig()
    pa = cv2.goodFeaturesToTrack(gray_a, maxCorners=500, qualityLevel=0.01,
                                 minDistance=8, blockSize=7)
    empty = np.empty((0, 2), dtype=float)
    if pa is None or len(pa) < cfg.min_tracks:
        return FlowPair(frame_index, imu_index, prev_imu_index, empty, empty,
                        0 if pa is None else len(pa), 0.0, False, "too few tracks")
    pb, st1, _ = cv2.calcOpticalFlowPyrLK(gray_a, gray_b, pa, None, winSize=(21, 21), maxLevel=3)
    if pb is None:
        return FlowPair(frame_index, imu_index, prev_imu_index, empty, empty, len(pa), 0.0, False, "LK failed")
    back, st2, _ = cv2.calcOpticalFlowPyrLK(gray_b, gray_a, pb, None, winSize=(21, 21), maxLevel=3)
    if back is None:
        return FlowPair(frame_index, imu_index, prev_imu_index, empty, empty, len(pa), 0.0, False, "backward LK failed")
    a, b = pa.reshape(-1, 2), pb.reshape(-1, 2)
    keep = (st1.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm(a - back.reshape(-1, 2), axis=1) < 1.0)
    a, b = a[keep], b[keep]
    if len(a) < cfg.min_tracks:
        return FlowPair(frame_index, imu_index, prev_imu_index, empty, empty, len(a), 0.0, False, "too few LK tracks")
    _, mask = cv2.findHomography(a, b, cv2.RANSAC, cfg.ransac_threshold_px)
    if mask is None:
        return FlowPair(frame_index, imu_index, prev_imu_index, empty, empty, len(a), 0.0, False, "RANSAC failed")
    inliers = mask.ravel().astype(bool)
    ratio = float(inliers.mean())
    if ratio < cfg.min_inlier_ratio or inliers.sum() < cfg.min_tracks:
        return FlowPair(frame_index, imu_index, prev_imu_index, empty, empty, len(a), ratio, False, "low RANSAC support")
    Kinv = np.linalg.inv(np.asarray(K, float))
    norm = lambda p: (np.c_[p, np.ones(len(p))] @ Kinv.T)[:, :2]
    return FlowPair(frame_index, imu_index, prev_imu_index, norm(a[inliers]), norm(b[inliers]), len(a), ratio, True)


def estimate_metric_velocity(pair, R_ab_cam, n_cam_a, range_m, dt, fx, flow_cfg=None,
                             range_std_m=0.02, min_range=0.20, max_range=100.0, max_dt=0.20):
    """Return diagnostic record + metric camera velocity; never reads estimator GT."""
    out = {"valid": False, "reason": "", "flow_u_px_s": np.nan, "flow_v_px_s": np.nan,
           "flow_spread_px_s": np.nan, "velocity": None, "covariance": None, "inliers": 0}
    if not pair.valid:
        out["reason"] = pair.reason
        return out
    if not np.isfinite(range_m) or not min_range <= range_m <= max_range:
        out["reason"] = "invalid range"
        return out
    if not np.isfinite(dt) or dt < 0.01 or dt > max_dt:
        out["reason"] = "invalid image dt"
        return out
    # Median image displacement is logged only as a quality diagnostic; metric
    # velocity is solved from derotated rays and a planar ground model below.
    disp = (pair.xb - pair.xa) * np.array([fx, fx]) / dt
    med = np.median(disp, axis=0)
    out["flow_u_px_s"], out["flow_v_px_s"] = med.tolist()
    out["flow_spread_px_s"] = float(np.median(np.linalg.norm(disp - med, axis=1)))
    fit = camera_velocity_from_flow(pair.xa, pair.xb, R_ab_cam, n_cam_a,
                                    range_m, dt, fx, flow_cfg)
    out["inliers"] = int(fit.n_used) if fit is not None else 0
    if fit is None:
        out["reason"] = "flow geometry/residual rejected"
        return out
    # Range finder and camera share the sensor_link origin. Range-noise scale
    # propagates along velocity; per-axis fit scatter comes from the LK residual.
    speed = max(float(np.linalg.norm(fit.v_cam[:2])), 0.1)
    u = fit.v_cam[:2] / speed
    cov = fit.cov_xy + np.outer(u, u) * (range_std_m * speed / range_m) ** 2
    out.update(valid=True, reason="ok", velocity=fit.v_cam[:2], covariance=cov,
               residual_px=fit.residual_px)
    return out

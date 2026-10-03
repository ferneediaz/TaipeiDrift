"""Turn camera frames into visual rotation measurements for the fusion.

The front end sees only images, the camera model and the frame-to-IMU index
mapping. It never sees ground truth or the estimator state, so it cannot leak
either into the measurements.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

from vio.vision.camera import camera_to_body
from vio.vision.feature_tracker import KeyframeTracker, TrackerConfig, TrackResult
from vio.vision.relative_pose import (
    PoseConfig,
    estimate_relative_pose,
    homography_rotation_candidates,
)

POSE_MODELS = ("essential", "homography")


@dataclass
class VisualMeasurement:
    """Relative body rotation between a keyframe and the current frame, as seen by the camera."""

    frame_index: int
    keyframe_index: int
    imu_index: int  # IMU sample at which this frame was taken
    keyframe_imu_index: int
    valid: bool
    body_rotation_candidates: list[np.ndarray] = field(default_factory=list)  # R_ab in the body frame
    translation_dir_cam: np.ndarray | None = None  # unit vector, camera a frame; scale unknown
    n_detected: int = 0
    n_correspondences: int = 0
    n_inliers: int = 0
    inlier_ratio: float = 0.0
    new_keyframe: bool = False  # tracker re-anchored on this frame, nothing measured
    reason: str = ""
    end_of_span: bool = False  # last measurement against this keyframe: the next keyframe is this frame


def track_frames(frames: Iterable[tuple[int, np.ndarray]], tracker_cfg: TrackerConfig) -> list[TrackResult]:
    """Run the keyframe tracker over (frame index, grayscale image) pairs."""
    tracker = KeyframeTracker(tracker_cfg)
    return [tracker.process(img, i) for i, img in frames]


def measurements_from_tracks(
    tracks: list[TrackResult],
    K: np.ndarray,
    R_bc: np.ndarray,
    frame_to_imu: callable,
    pose_model: str = "essential",
    pose_cfg: PoseConfig | None = None,
) -> list[VisualMeasurement]:
    """Estimate the relative rotation for each tracked frame and express it in the body frame.

    Args:
        tracks: tracker output in frame order.
        K: intrinsics of the processed image.
        R_bc: camera-to-body rotation.
        frame_to_imu: maps a frame index to its IMU sample index.
        pose_model: "essential" (one rotation) or "homography" (up to four
            candidates, resolved later by the fusion with its gyro prediction).
    """
    if pose_model not in POSE_MODELS:
        raise ValueError(f"pose_model must be one of {POSE_MODELS}, got {pose_model!r}")
    out: list[VisualMeasurement] = []
    for tr in tracks:
        m = VisualMeasurement(
            frame_index=tr.frame_index,
            keyframe_index=tr.keyframe_index,
            imu_index=int(frame_to_imu(tr.frame_index)),
            keyframe_imu_index=int(frame_to_imu(tr.keyframe_index)),
            valid=False,
            n_detected=tr.n_detected,
            n_correspondences=len(tr.current_points),
            new_keyframe=tr.new_keyframe,
            end_of_span=bool(getattr(tr, "reanchor", "")),
        )
        if tr.new_keyframe:
            m.reason = "new keyframe"
            out.append(m)
            continue
        try:
            if pose_model == "essential":
                pose = estimate_relative_pose(tr.keyframe_points, tr.current_points, K, pose_cfg)
                cands = [pose.rotation] if pose.valid else []
                m.translation_dir_cam = pose.translation_dir
            else:
                cands, pose = homography_rotation_candidates(tr.keyframe_points, tr.current_points, K, pose_cfg)
        except Exception as exc:  # a bad frame must never stop navigation
            m.reason = f"front end error: {exc}"
            out.append(m)
            continue
        m.valid = pose.valid
        m.reason = pose.reason
        m.n_inliers, m.inlier_ratio = pose.n_inliers, pose.inlier_ratio
        m.body_rotation_candidates = [camera_to_body(R_bc, R) for R in cands]
        out.append(m)
    return out

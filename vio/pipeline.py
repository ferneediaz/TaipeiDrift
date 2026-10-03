"""Glue between the Mid-Air data, the visual front end and the estimators."""
from __future__ import annotations

from dataclasses import dataclass, fields
from itertools import islice
from pathlib import Path

import numpy as np

from src.data.trajectory import Trajectory
from src.estimation.inertial_dead_reckoning import DeadReckoningResult, NavState
from vio.data.midair_camera import FrameSource, frame_to_imu_index, open_frames
from vio.estimation.visual_attitude_fusion import FusionConfig, FusionEvent, run_visual_attitude_fusion
from vio.vision.camera import check_rotation_matrix, pinhole_intrinsics
from vio.vision.feature_tracker import TrackerConfig, TrackResult
from vio.vision.measurements import VisualMeasurement, measurements_from_tracks, track_frames
from vio.vision.relative_pose import PoseConfig


def _dataclass_from(cls, d: dict | None):
    names = {f.name for f in fields(cls)}
    unknown = set(d or {}) - names
    if unknown:
        raise ValueError(f"unknown {cls.__name__} keys: {sorted(unknown)}")
    return cls(**(d or {}))


@dataclass
class CameraSetup:
    """Everything the front end needs to know about one camera."""

    name: str
    stream: str
    hfov_deg: float
    rate_hz: float
    frame_offset_samples: int
    R_bc: np.ndarray
    pose_model: str
    downscale: int

    @classmethod
    def from_config(cls, cfg: dict, camera: str) -> "CameraSetup":
        if camera not in cfg["cameras"]:
            raise ValueError(f"camera {camera!r} not configured; choose from {sorted(cfg['cameras'])}")
        c = cfg["cameras"][camera]
        return cls(camera, c["stream"], float(c["hfov_deg"]), float(c["rate_hz"]), int(c["frame_offset_samples"]),
                   check_rotation_matrix(c["R_bc"]), c["pose_model"], int(cfg.get("downscale", 1)))

    def intrinsics(self, width: int, height: int) -> np.ndarray:
        """Intrinsics for the processed (downscaled) image size."""
        return pinhole_intrinsics(width, height, self.hfov_deg)


def tracker_config(cfg: dict) -> TrackerConfig:
    return _dataclass_from(TrackerConfig, cfg.get("tracker"))


def pose_config(cfg: dict) -> PoseConfig:
    return _dataclass_from(PoseConfig, cfg.get("pose"))


def fusion_config(cfg: dict) -> FusionConfig:
    return _dataclass_from(FusionConfig, cfg.get("fusion"))


def frame_range(frames: FrameSource, setup: CameraSetup, imu_rate_hz: float, start_index: int, n_imu: int) -> range:
    """Frames taken at or after the GNSS cutoff and inside the IMU record."""
    idx = frame_to_imu_index(np.arange(len(frames)), imu_rate_hz, setup.rate_hz, setup.frame_offset_samples)
    ok = np.nonzero((idx >= start_index) & (idx < n_imu))[0]
    if ok.size == 0:
        raise ValueError("no camera frames after the GNSS cutoff")
    return range(int(ok[0]), int(ok[-1]) + 1)


def run_front_end(traj: Trajectory, setup: CameraSetup, tracker_cfg: TrackerConfig, start_index: int,
                  imu_rate_hz: float) -> tuple[list[TrackResult], np.ndarray, FrameSource, range]:
    """Track features from the first frame after the cutoff to the end. Returns tracks and intrinsics.

    Only images are read here: no ground truth and no estimator state.
    """
    frames = open_frames(Path(traj.metadata["file"]), traj.metadata["trajectory"], setup.stream)
    rng = frame_range(frames, setup, imu_rate_hz, start_index, len(traj))
    first = frames.read_gray(rng.start, setup.downscale)
    K = setup.intrinsics(first.shape[1], first.shape[0])
    gen = islice(frames.iter_gray(rng.start, setup.downscale), len(rng))
    return track_frames(gen, tracker_cfg), K, frames, rng


def build_measurements(tracks: list[TrackResult], K: np.ndarray, setup: CameraSetup, pose_cfg: PoseConfig,
                       imu_rate_hz: float) -> list[VisualMeasurement]:
    to_imu = lambda i: frame_to_imu_index(i, imu_rate_hz, setup.rate_hz, setup.frame_offset_samples)  # noqa: E731
    return measurements_from_tracks(tracks, K, setup.R_bc, to_imu, setup.pose_model, pose_cfg)


def run_fusion_on_trajectory(traj: Trajectory, gnss_cutoff: float, measurements: list[VisualMeasurement],
                             fusion_cfg: FusionConfig) -> tuple[DeadReckoningResult, list[FusionEvent]]:
    """Hand the deployable estimator only what it is allowed to see.

    Ground truth enters once, as the initial state at the cutoff. Everything
    after the cutoff passed to the estimator is IMU and camera data.
    """
    k0 = traj.index_at(gnss_cutoff)
    initial = NavState(traj.position_gt[k0].copy(), traj.velocity_gt[k0].copy(), traj.attitude_gt[k0].copy())
    return run_visual_attitude_fusion(
        traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], initial, traj.gravity_world,
        traj.gyroscope_frame, measurements, k0, fusion_cfg,
    )

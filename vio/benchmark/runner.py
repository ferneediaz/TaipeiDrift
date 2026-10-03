"""Run the three methods on one discovered trajectory and compute its metrics.

Methods (unchanged; only called here):
  imu_only                       baseline strapdown dead reckoning (baseline/)
  visual                         forward-camera complementary attitude correction (vio/, frozen settings)
  ground_truth_attitude_oracle   measured accelerometer + TRUE attitude after the cutoff. NOT an estimator.

Ground truth after the cutoff is used only for metrics and by the oracle.
"""
from __future__ import annotations

import hashlib
import json
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path

import numpy as np

from src.data.midair import MidAirConfig, load_midair_trajectory
from src.estimation.inertial_dead_reckoning import DeadReckoningResult, run_dead_reckoning
from src.evaluation.trajectory_metrics import error_series
from vio.benchmark.discovery import TrajectoryEntry
from vio.estimation.oracle import oracle_ground_truth_attitude
from vio.evaluation.attitude_metrics import attitude_error_series
from vio.pipeline import CameraSetup, build_measurements, fusion_config, pose_config, run_front_end, run_fusion_on_trajectory, tracker_config

METHODS = ("imu_only", "visual", "ground_truth_attitude_oracle")
BENCHMARK_VERSION = 1  # bump when the metric definitions change, so --resume recomputes


def fingerprint(entry: TrajectoryEntry, bench_cfg: dict, vio_cfg: dict, camera: str, cutoff: float) -> str:
    """Hash of everything a stored result depends on: settings and the data files' size and mtime."""
    blob = json.dumps({"v": BENCHMARK_VERSION, "bench": bench_cfg, "vio": {k: vio_cfg[k] for k in ("cameras", "downscale", "tracker", "pose", "fusion")},
                       "camera": camera, "cutoff": cutoff, "visual_valid": entry.visual_valid,
                       "sig": entry.signature}, sort_keys=True, default=str)
    return hashlib.sha1(blob.encode()).hexdigest()


def _at(t: np.ndarray, y: np.ndarray, h: float) -> float:
    return float(np.interp(h, t, y)) if t.size and h <= t[-1] + 1e-9 else float("nan")


def series_metrics(t: np.ndarray, e: np.ndarray, horizons: list[float], rmse: bool = True) -> dict:
    """Error at fixed horizons (NaN beyond the flight), final, mean, median, RMSE, max."""
    m = {f"{h:g}s": _at(t, e, h) for h in horizons}
    m.update({"final": float(e[-1]), "mean": float(np.mean(e)), "median": float(np.median(e)), "max": float(np.max(e))})
    if rmse:
        m["rmse"] = float(np.sqrt(np.mean(e**2)))
    return m


def method_metrics(res: DeadReckoningResult, traj, horizons: list[float]) -> dict:
    pos = error_series(res, traj)
    att = attitude_error_series(res, traj)
    return {"position_m": series_metrics(pos.time_since_loss, pos.error, horizons),
            "attitude_deg": series_metrics(att.time_since_loss, att.error_deg, horizons, rmse=False)}


def visual_statistics(tracks, measurements, events) -> dict:
    """Front-end and fusion statistics of the visual estimator."""
    meas = [m for m in measurements if not m.new_keyframe]
    valid = np.array([m.valid for m in meas], dtype=bool)
    accepted_by = {ev.frame_index: ev.accepted for ev in events}
    accepted = np.array([accepted_by.get(m.frame_index, False) for m in meas], dtype=bool)
    tracks_n = np.array([m.n_correspondences for m in meas], dtype=float)
    inl = np.array([m.n_inliers for m in meas], dtype=float)
    ratio = np.array([m.inlier_ratio for m in meas], dtype=float)
    stat = lambda x, f: float(f(x)) if x.size else float("nan")  # noqa: E731
    return {
        "frames_processed": len(tracks),
        "attempted_updates": len(meas),
        "valid_measurements": int(valid.sum()),
        "accepted_updates": int(accepted.sum()),
        "valid_fraction": stat(valid, np.mean),
        "accepted_fraction": stat(accepted, np.mean),
        "rejected_updates": int(len(meas) - accepted.sum()),
        "visual_failures": int((~valid).sum()),
        "tracks_mean": stat(tracks_n, np.mean), "tracks_median": stat(tracks_n, np.median),
        "inliers_mean": stat(inl, np.mean), "inliers_median": stat(inl, np.median),
        "inlier_ratio_mean": stat(ratio, np.mean), "inlier_ratio_median": stat(ratio, np.median),
        "keyframe_resets_track_loss": sum(1 for t in tracks if getattr(t, "reanchor", "") == "few tracks"),
        "keyframes_by_age": sum(1 for t in tracks if getattr(t, "reanchor", "") == "age"),
        "invalid_reasons": dict(sorted(Counter(m.reason for m in meas if not m.valid).items())),
        "rejection_reasons": dict(sorted(Counter(ev.reason for ev in events if not ev.accepted).items())),
    }


def load_entry(entry: TrajectoryEntry, base_midair_cfg: dict):
    """Load a discovered flight through the baseline loader (same keys and conventions)."""
    m = dict(base_midair_cfg)
    m.update({"environment": entry.environment, "condition": entry.condition, "trajectory": entry.trajectory, "data_root": None})
    cfg = MidAirConfig.from_dict(m)
    root = Path(entry.sensor_file).parent.parent.parent  # <root>/<env>/<cond>/sensor_records.hdf5
    traj = load_midair_trajectory(cfg, data_root=str(root))
    traj.metadata["frames_dir"] = entry.frames_dir  # images stay in the dataset folder
    return traj, cfg


def run_trajectory(entry: TrajectoryEntry, base_midair_cfg: dict, vio_cfg: dict, camera: str, cutoff: float,
                   horizons: list[float], return_estimates: bool = False) -> dict:
    """All methods on one flight. A failure in one method is recorded, not raised.

    ``return_estimates`` adds the raw estimated positions under ``_estimates`` (for tests; not saved).
    """
    t0 = time.time()
    traj, mcfg = load_entry(entry, base_midair_cfg)
    k0 = traj.index_at(cutoff)
    out = {"key": entry.key, "environment": entry.environment, "condition": entry.condition, "trajectory": entry.trajectory,
           "duration_s": traj.duration, "gnss_cutoff_s": float(traj.timestamp[k0]),
           "evaluated_s": float(traj.timestamp[-1] - traj.timestamp[k0]), "methods": {}, "visual_statistics": None}

    est = {}
    est["imu_only"] = run_dead_reckoning(traj, cutoff)
    est["ground_truth_attitude_oracle"] = oracle_ground_truth_attitude(traj, cutoff)
    out["methods"]["imu_only"] = method_metrics(est["imu_only"], traj, horizons)
    out["methods"]["ground_truth_attitude_oracle"] = method_metrics(est["ground_truth_attitude_oracle"], traj, horizons)

    if not entry.visual_valid:
        out["methods"]["visual"] = None
        out["visual_unavailable_reason"] = entry.visual_reason
    else:
        try:
            setup = CameraSetup.from_config(vio_cfg, camera)
            tracks, K, _, _ = run_front_end(traj, setup, tracker_config(vio_cfg), k0, mcfg.imu_rate_hz)
            meas = build_measurements(tracks, K, setup, pose_config(vio_cfg), mcfg.imu_rate_hz)
            res, events = run_fusion_on_trajectory(traj, cutoff, meas, fusion_config(vio_cfg))
            est["visual"] = res
            out["methods"]["visual"] = method_metrics(res, traj, horizons)
            out["visual_statistics"] = visual_statistics(tracks, meas, events)
        except Exception as e:  # e.g. a frame that fails to decode mid-archive
            out["methods"]["visual"] = None
            out["visual_unavailable_reason"] = f"visual run failed: {type(e).__name__}: {e}"
    out["runtime_s"] = round(time.time() - t0, 1)
    if return_estimates:
        out["_estimates"] = {k: v.position.copy() for k, v in est.items()}
    return out

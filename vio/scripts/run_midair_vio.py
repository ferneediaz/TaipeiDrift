"""Compare IMU-only dead reckoning with visual-attitude-assisted dead reckoning on Mid-Air.

From the repository root:

    python vio/scripts/run_midair_vio.py --data-root data/MidAir --condition sunny \
        --trajectory 0 --camera left --gnss-cutoff 5.0

Three runs from the same GNSS cutoff:
  1. IMU only               the baseline estimator, unchanged (baseline/)
  2. IMU + visual attitude  deployable: IMU and camera only after the cutoff
  3. ORACLE                 ground-truth attitude + measured accelerometer. NOT an estimator:
                            it shows how much drift would remain if the attitude were perfect.

Results go to outputs/midair_vio/<run_name>/.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path

import numpy as np
import yaml

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))

from src.data.midair import MidAirConfig, MidAirDataNotFound, load_midair_trajectory  # noqa: E402
from src.estimation.inertial_dead_reckoning import run_dead_reckoning  # noqa: E402
from src.evaluation.trajectory_metrics import error_at_horizons, error_series, summarize, time_to_exceed  # noqa: E402
from vio.estimation.oracle import ORACLE_LABEL, oracle_ground_truth_attitude  # noqa: E402
from vio.evaluation.attitude_metrics import attitude_at_horizons, attitude_error_series, summarize_attitude  # noqa: E402
from vio.pipeline import (  # noqa: E402
    CameraSetup,
    build_measurements,
    fusion_config,
    pose_config,
    run_front_end,
    run_fusion_on_trajectory,
    tracker_config,
)
from vio.visualization.vio_plots import draw_tracks, plot_series, plot_trajectory, plot_visual_tracking  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=VIO_DIR / "configs" / "midair_vio.yaml")
    p.add_argument("--data-root", help="Mid-Air root (overrides MID_AIR_ROOT and the baseline config)")
    p.add_argument("--environment")
    p.add_argument("--condition")
    p.add_argument("--trajectory", help="e.g. 0 or trajectory_0000")
    p.add_argument("--camera", help="down or left")
    p.add_argument("--gnss-cutoff", type=float)
    p.add_argument("--gain", type=float, help="override fusion.gain")
    p.add_argument("--run-name")
    p.add_argument("--output-dir", type=Path)
    return p.parse_args()


def position_block(series, horizons, thresholds) -> dict:
    s = summarize(series)
    return {
        "error_at_time_since_loss_m": {f"{h:g}s": v for h, v in error_at_horizons(series, horizons).items()},
        "final_m": s.final, "rmse_m": s.rmse, "mean_m": s.mean, "max_m": s.max,
        "time_to_exceed_s": {f"{th:g}m": v for th, v in time_to_exceed(series, thresholds).items()},
    }


def attitude_block(series, horizons) -> dict:
    s = summarize_attitude(series)
    return {"error_at_time_since_loss_deg": {f"{h:g}s": v for h, v in attitude_at_horizons(series, horizons).items()},
            **s.as_dict()}


def main() -> int:
    args = parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    base_cfg = yaml.safe_load((REPO_ROOT / cfg["baseline_config"]).read_text())
    cutoff = args.gnss_cutoff if args.gnss_cutoff is not None else cfg["gnss_cutoff_s"]
    camera = args.camera or cfg["camera"]
    if args.gain is not None:
        cfg["fusion"]["gain"] = args.gain

    m = dict(base_cfg["midair"])
    for key in ("environment", "condition", "trajectory"):
        if getattr(args, key):
            m[key] = getattr(args, key)
    mcfg = MidAirConfig.from_dict(m)
    if mcfg.data_root and not Path(mcfg.data_root).is_absolute():
        mcfg.data_root = str(REPO_ROOT / mcfg.data_root)
    try:
        traj = load_midair_trajectory(mcfg, data_root=args.data_root)
    except MidAirDataNotFound as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    setup = CameraSetup.from_config(cfg, camera)
    run_name = args.run_name or f"{traj.name}_{camera}_cutoff{cutoff:g}s"
    out = Path(args.output_dir or REPO_ROOT / cfg["output_dir"]) / run_name
    out.mkdir(parents=True, exist_ok=True)
    horizons, thresholds = cfg["error_horizons_s"], cfg["error_thresholds_m"]
    k0 = traj.index_at(cutoff)
    print(f"flight {traj.name}, camera {setup.stream}, GNSS lost at {traj.timestamp[k0]:.2f} s")

    # 1. IMU only (baseline, unchanged)
    imu = run_dead_reckoning(traj, cutoff)
    # 2. IMU + visual attitude (deployable)
    t = time.time()
    try:
        tracks, K, frames, frange = run_front_end(traj, setup, tracker_config(cfg), k0, mcfg.imu_rate_hz)
    except FileNotFoundError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    measurements = build_measurements(tracks, K, setup, pose_config(cfg), mcfg.imu_rate_hz)
    print(f"visual front end: {len(tracks)} frames in {time.time() - t:.0f} s")
    visual, events = run_fusion_on_trajectory(traj, cutoff, measurements, fusion_config(cfg))
    # 3. ORACLE (not an estimator)
    oracle = oracle_ground_truth_attitude(traj, cutoff)

    runs = {"imu": imu, "visual": visual, "oracle": oracle}
    pos = {k: error_series(r, traj) for k, r in runs.items()}
    att = {k: attitude_error_series(r, traj) for k, r in runs.items() if k != "oracle"}

    # visual statistics
    meas_frames = [mm for mm in measurements if not mm.new_keyframe]
    valid = np.array([mm.valid for mm in meas_frames])
    accepted_by_frame = {ev.frame_index: ev.accepted for ev in events}
    accepted = np.array([accepted_by_frame.get(mm.frame_index, False) for mm in meas_frames])
    n_corr = np.array([mm.n_correspondences for mm in meas_frames])
    n_inl = np.array([mm.n_inliers for mm in meas_frames])
    ratio = np.array([mm.inlier_ratio for mm in meas_frames])
    resets = sum(1 for tr in tracks if tr.reanchor == "few tracks")  # track loss forced a new keyframe
    scheduled = sum(1 for tr in tracks if tr.reanchor == "age")
    reasons = Counter(mm.reason for mm in meas_frames if not mm.valid)
    rejections = Counter(ev.reason for ev in events if not ev.accepted)
    visual_stats = {
        "camera_stream": setup.stream, "pose_model": setup.pose_model,
        "frames_processed": len(tracks), "measurements": len(meas_frames),
        "valid_percent": float(100 * valid.mean()) if len(valid) else 0.0,
        "accepted_percent": float(100 * accepted.mean()) if len(accepted) else 0.0,
        "median_tracked_features": float(np.median(n_corr)), "median_inliers": float(np.median(n_inl)),
        "median_inlier_ratio": float(np.median(ratio)),
        "keyframe_resets_track_loss": resets, "keyframes_by_age": scheduled,
        "invalid_reasons": dict(reasons), "fusion_rejections": dict(rejections),
        "median_camera_gyro_disagreement_deg": float(np.nanmedian([ev.disagreement_deg for ev in events])) if events else None,
    }

    metrics = {
        "run_name": run_name, "trajectory": traj.name, "metadata": traj.metadata,
        "gnss_cutoff_s": float(traj.timestamp[k0]),
        "config": {"tracker": cfg["tracker"], "pose": cfg["pose"], "fusion": cfg["fusion"],
                   "camera": {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in asdict(setup).items()}},
        "imu_only": {"position": position_block(pos["imu"], horizons, thresholds),
                     "attitude": attitude_block(att["imu"], horizons)},
        "visual_attitude": {"position": position_block(pos["visual"], horizons, thresholds),
                            "attitude": attitude_block(att["visual"], horizons), "visual_front_end": visual_stats},
        "oracle_ground_truth_attitude": {"label": ORACLE_LABEL,
                                         "position": position_block(pos["oracle"], horizons, thresholds)},
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
    np.savetxt(out / "errors.csv", np.column_stack([
        pos["imu"].time_since_loss, pos["imu"].error, pos["visual"].error, pos["oracle"].error,
        att["imu"].error_deg, att["visual"].error_deg]), delimiter=",", comments="", fmt="%.6f",
        header="time_since_loss_s,pos_err_imu_m,pos_err_visual_m,pos_err_ORACLE_true_attitude_m,att_err_imu_deg,att_err_visual_deg")

    plot_trajectory(traj, runs, out / "trajectory.png")
    plot_series({k: s.time_since_loss for k, s in pos.items()}, {k: s.error for k, s in pos.items()},
                "Position error (m)", f"Position error after GNSS loss, {traj.name}", out / "position_error.png")
    plot_series({k: s.time_since_loss for k, s in att.items()}, {k: s.error_deg for k, s in att.items()},
                "Attitude error (deg)", f"Attitude error after GNSS loss, {traj.name}", out / "attitude_error.png")
    t_meas = np.array([traj.timestamp[mm.imu_index] for mm in meas_frames]) - traj.timestamp[k0]
    plot_visual_tracking(t_meas, n_corr, n_inl, valid, accepted, out / "visual_tracking.png",
                         f"Visual tracking, {setup.stream}, {traj.name}")
    # debug image: a valid measurement near the middle of the run
    mid = [i for i, mm in enumerate(meas_frames) if mm.valid]
    if mid:
        j = mid[len(mid) // 2]
        tr = next(x for x in tracks if x.frame_index == meas_frames[j].frame_index)
        import cv2
        _, mask = cv2.findEssentialMat(tr.keyframe_points.astype(np.float64), tr.current_points.astype(np.float64), K,
                                       cv2.RANSAC, 0.999, cfg["pose"]["ransac_threshold_px"])
        inl = mask.ravel().astype(bool) if mask is not None else np.ones(len(tr.current_points), bool)
        draw_tracks(frames.read_gray(tr.frame_index, setup.downscale), tr.keyframe_points, tr.current_points, inl,
                    out / f"tracks_frame{tr.frame_index:06d}.png")

    def row(label, p, a=None):
        h = error_at_horizons(p, horizons)
        s = summarize(p)
        txt = f"  {label:34s} pos " + " ".join(f"{k:g}s {v:7.1f}" if v is not None else f"{k:g}s    -  " for k, v in h.items())
        txt += f" | final {s.final:7.1f} RMSE {s.rmse:7.1f} max {s.max:7.1f} m"
        if a is not None:
            sa = summarize_attitude(a)
            txt += f" | att final {sa.final_deg:5.2f} max {sa.max_deg:5.2f} deg"
        print(txt)
    row("IMU only", pos["imu"], att["imu"])
    row("IMU + visual attitude", pos["visual"], att["visual"])
    row("ORACLE true attitude (not estimator)", pos["oracle"])
    print(f"  visual: {visual_stats['valid_percent']:.1f}% valid, {visual_stats['accepted_percent']:.1f}% accepted, "
          f"median {visual_stats['median_tracked_features']:.0f} tracks / {visual_stats['median_inliers']:.0f} inliers, "
          f"{resets} track-loss resets, {scheduled} scheduled keyframes")
    print(f"results in {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

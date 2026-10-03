"""Prepare ESKF inputs from a Mid-Air flight and evaluate the outputs.

Separation of what is allowed where:
- BEFORE the cutoff (GNSS available): GNSS velocity and the GNSS-aided attitude
  may be used, here to learn the starting height above ground.
- AT the cutoff: the true state initialises the filter, standing in for the
  GNSS-aided navigation state at that moment.
- AFTER the cutoff: the filter gets IMU, camera measurements and the simulated
  barometer only. Ground truth is used for evaluation and to SIMULATE the
  barometer (vio/sensors/simulated.py), never passed to the filter.
"""
from __future__ import annotations

import hashlib
import json
import pickle
from dataclasses import dataclass, replace
from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

from src.data.trajectory import Trajectory, quat_wxyz_to_rotation
from src.estimation.inertial_dead_reckoning import NavState
from vio.data.midair_camera import frame_to_imu_index, open_frames
from vio.estimation.eskf import ImuNoiseModel
from vio.estimation.eskf_runner import (
    BaroUpdateConfig,
    DirectionUpdateConfig,
    EskfInputs,
    EskfOutput,
    FlowUpdateConfig,
    RotationUpdateConfig,
    run_eskf,
)
from vio.evaluation.attitude_metrics import attitude_error_series
from vio.evaluation.consistency import ate_rmse, native_imu_bias, nees, rpe, state_error, summarize_chi2
from vio.pipeline import CameraSetup, build_measurements, frame_range, pose_config, tracker_config
from vio.sensors.simulated import BarometerConfig, simulate_barometer
from vio.vision.feature_tracker import TrackResult
from vio.vision.measurements import track_frames
from vio.vision.optical_flow import FlowConfig, flow_pairs_from_tracks, height_from_known_velocity

BIAS_SCORE_WINDOW_S = 20.0

# name: (forward relative rotation, down-camera flow, barometer altitude[, forward translation direction])
# The assumed platform carries a barometer (docs/PLAN.md), so every visual configuration is
# compared against "imu_baro". "imu_only" is the pure IMU-only baseline, kept for continuity.
ABLATIONS = {
    "imu_only": (False, False, False),
    "imu_baro": (False, False, True),
    "forward_rotation": (True, False, True),
    "down_flow": (False, True, True),
    "both": (True, True, True),
    "forward_direction": (True, False, True, True),
    "both_direction": (True, True, True, True),
}


def noise_model(cfg: dict) -> ImuNoiseModel:
    c = cfg["imu_noise"]
    sq = np.sqrt(0.01)
    return ImuNoiseModel(
        gyro_noise=c["gyro_noise_per_sample"] * sq, accel_noise=c["accel_noise_per_sample"] * sq,
        gyro_bias_walk=c["gyro_bias_change_per_flight"] / np.sqrt(80.0),
        accel_bias_walk=c["accel_bias_change_per_flight"] / np.sqrt(80.0),
        init_pos_std=c["init_pos_std"], init_vel_std=c["init_vel_std"], init_att_std_deg=c["init_att_std_deg"],
        init_gyro_bias_std=c["init_gyro_bias_std"], init_accel_bias_std=c["init_accel_bias_std"],
    )


# ------------------------------------------------------------ front ends (cached)
def _cached_tracks(traj: Trajectory, setup: CameraSetup, tracker, first_frame: int, last_frame: int,
                   cache_dir: Path | None) -> tuple[list[TrackResult], tuple[int, int]]:
    frames = open_frames(Path(traj.metadata["file"]), traj.metadata["trajectory"], setup.stream, traj.metadata.get("frames_dir"))
    shape = frames.read_gray(first_frame, setup.downscale).shape
    key = hashlib.sha1(json.dumps([traj.metadata["file"], traj.metadata["trajectory"], setup.stream, setup.downscale,
                                   first_frame, last_frame, tracker.__dict__], sort_keys=True).encode()).hexdigest()[:16]
    path = cache_dir / f"tracks_{traj.metadata['trajectory']}_{setup.stream}_{key}.pkl" if cache_dir else None
    if path is not None and path.is_file():
        return pickle.loads(path.read_bytes()), shape
    from itertools import islice
    tracks = track_frames(islice(frames.iter_gray(first_frame, setup.downscale), last_frame - first_frame + 1), tracker)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(pickle.dumps(tracks))
    return tracks, shape


@dataclass
class VisualInputs:
    rotation_measurements: list | None
    R_bc_forward: np.ndarray | None
    flow_pairs: list | None
    R_bc_down: np.ndarray | None
    focal_px_down: float
    height_above_ground: np.ndarray | None  # (n,) from the cutoff
    height_info: dict
    baro_altitude_change: np.ndarray | None = None  # (n,) metres up since the cutoff


def prepare_visual_inputs(traj: Trajectory, cfg: dict, vio_cfg: dict, k0: int, imu_rate: float,
                          cache_dir: Path | None, want_forward: bool = True, want_down: bool = True) -> VisualInputs:
    rot_meas = R_fwd = pairs = R_down = hag = None
    baro = simulate_barometer(traj, BarometerConfig(**{k: v for k, v in cfg["barometer"].items() if k in BarometerConfig.__dataclass_fields__}))
    baro_change = baro.altitude_m[k0:] - baro.altitude_m[k0]
    focal = 1.0
    info: dict = {}
    if want_forward:
        fc = cfg["forward_camera"]
        setup = CameraSetup.from_config(vio_cfg, fc["camera"])
        tr_cfg = replace(tracker_config(vio_cfg), max_keyframe_age=int(fc["max_keyframe_age"]))
        frames = open_frames(Path(traj.metadata["file"]), traj.metadata["trajectory"], setup.stream, traj.metadata.get("frames_dir"))
        rng = frame_range(frames, setup, imu_rate, k0, len(traj))
        tracks, shape = _cached_tracks(traj, setup, tr_cfg, rng.start, rng.stop - 1, cache_dir)
        K = setup.intrinsics(shape[1], shape[0])
        rot_meas = build_measurements(tracks, K, setup, pose_config(vio_cfg), imu_rate)
        R_fwd = setup.R_bc
    if want_down:
        dc = cfg["down_camera"]
        setup = CameraSetup.from_config(vio_cfg, dc["camera"])
        tr_cfg = replace(tracker_config(vio_cfg), max_keyframe_age=1)
        frames = open_frames(Path(traj.metadata["file"]), traj.metadata["trajectory"], setup.stream, traj.metadata.get("frames_dir"))
        rng = frame_range(frames, setup, imu_rate, 0, len(traj))
        to_imu = lambda i: frame_to_imu_index(i, imu_rate, setup.rate_hz, setup.frame_offset_samples)  # noqa: E731
        learn_frames = int(round(dc["height_learning_window_s"] * setup.rate_hz))
        first_after = next(i for i in rng if to_imu(i) >= k0)
        first = max(rng.start, first_after - learn_frames)
        tracks, shape = _cached_tracks(traj, setup, tr_cfg, first, rng.stop - 1, cache_dir)
        K = setup.intrinsics(shape[1], shape[0])
        fcfg = FlowConfig(min_tracks=dc["min_tracks"], min_inlier_ratio=dc["min_inlier_ratio"],
                          max_residual_px=dc["max_residual_px"])
        all_pairs = flow_pairs_from_tracks(tracks, K, to_imu, fcfg)
        before = [p for p in all_pairs if p.imu_index <= k0 and p.valid]
        pairs = [p for p in all_pairs if p.prev_imu_index >= k0]
        R_down, focal = setup.R_bc, float(K[0, 0])
        h0, info = learn_height_before_cutoff(traj, before, R_down, imu_rate, fcfg)
        win = slice(min(p.prev_imu_index for p in before), k0 + 1) if before else slice(k0, k0 + 1)
        ref = float(np.mean(baro.altitude_m[win]))
        hag = h0 + (baro.altitude_m[k0:] - ref)
        info.update({"baro_reference_altitude_m": ref})
    return VisualInputs(rot_meas, R_fwd, pairs, R_down, focal, hag, info, baro_change)


def load_gnss_velocity(traj: Trajectory) -> tuple[np.ndarray, np.ndarray]:
    """Mid-Air's simulated GNSS velocity (NED, 1 Hz). Checked: same frame as ground truth, ~1 mm/s noise."""
    with h5py.File(traj.metadata["file"], "r") as f:
        d = f[traj.metadata["trajectory"]]["gps/velocity"]
        rate = float(d.attrs.get("sampling_frequency", 1.0))
        v = d[:]
    return np.arange(len(v)) / rate, v


def learn_height_before_cutoff(traj: Trajectory, pairs: list, R_bc: np.ndarray, imu_rate: float,
                               fcfg: FlowConfig) -> tuple[float, dict]:
    """Height above ground at the cutoff, from down-camera flow and GNSS velocity BEFORE the cutoff.

    The attitude before the cutoff is the GNSS-aided one, represented by ground truth,
    exactly as for the initial state at the cutoff.
    """
    tg, vg = load_gnss_velocity(traj)
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    hs = []
    for p in pairs:
        ta, tb = p.prev_imu_index / imu_rate, p.imu_index / imu_rate
        v = np.array([np.interp(0.5 * (ta + tb), tg, vg[:, i]) for i in range(3)])
        Ra, Rb = R[p.prev_imu_index].as_matrix(), R[p.imu_index].as_matrix()
        R_ab_cam = R_bc.T @ (Ra.T @ Rb) @ R_bc
        n_cam = (Ra @ R_bc).T @ np.array([0.0, 0.0, 1.0])
        t_cam = (Ra @ R_bc).T @ v * (tb - ta)
        h = height_from_known_velocity(p.xa, p.xb, R_ab_cam, n_cam, t_cam, fcfg)
        if h is not None and 1.0 < h < 500.0:
            hs.append(h)
    if not hs:
        raise RuntimeError("could not learn the height above ground before the cutoff (no usable down-camera flow)")
    return float(np.median(hs)), {"height_learned_m": float(np.median(hs)), "height_learning_pairs": len(hs),
                                  "height_learning_spread_m": float(np.percentile(hs, 75) - np.percentile(hs, 25))}


# ------------------------------------------------------------------ running
def run_ablation(traj: Trajectory, k0: int, vis: VisualInputs, cfg: dict, name: str) -> EskfOutput:
    use_rot, use_flow, use_baro, use_dir = (ABLATIONS[name] + (False,))[:4]
    fc, dc = cfg["forward_camera"], cfg["down_camera"]
    inp = EskfInputs(
        traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], traj.gyroscope_frame, traj.gravity_world,
        NavState(traj.position_gt[k0].copy(), traj.velocity_gt[k0].copy(), traj.attitude_gt[k0].copy()), k0,
        rotation_measurements=vis.rotation_measurements if (use_rot or use_dir) else None, R_bc_forward=vis.R_bc_forward,
        flow_pairs=vis.flow_pairs if use_flow else None, R_bc_down=vis.R_bc_down, focal_px_down=vis.focal_px_down,
        height_above_ground=vis.height_above_ground if use_flow else None,
        baro_altitude_change=vis.baro_altitude_change if use_baro else None,
    )
    bc = cfg["barometer"]
    baro_cfg = BaroUpdateConfig(enabled=use_baro, white_std_m=bc["white_std_m"],
                                bias_walk_m_per_sqrt_s=bc["bias_walk_m_per_sqrt_s"],
                                drift_sigma_m_per_s=bc.get("drift_sigma_m_per_s", 0.0024),
                                scale_error_rms=bc.get("scale_error_rms", 0.052),
                                every_n_samples=int(bc.get("every_n_samples", 20)))
    rot_cfg = RotationUpdateConfig(enabled=use_rot, sigma_deg=fc["sigma_deg"], gate_prob=fc["gate_prob"])
    flow_cfg = FlowUpdateConfig(enabled=use_flow, rel_height_std=dc["rel_height_std"], min_sigma_mps=dc["min_sigma_mps"],
                                dir_sigma_deg=dc["dir_sigma_deg"], every_n_frames=int(dc["every_n_frames"]),
                                gate_prob=dc["gate_prob"],
                                flow=FlowConfig(min_tracks=dc["min_tracks"], min_inlier_ratio=dc["min_inlier_ratio"],
                                                max_residual_px=dc["max_residual_px"]))
    dc = cfg.get("direction", {})
    dir_cfg = DirectionUpdateConfig(enabled=use_dir, sigma_deg=dc.get("sigma_deg", 1.5), min_speed_mps=dc.get("min_speed_mps", 0.5),
                                    gate_prob=dc.get("gate_prob", 0.99))
    return run_eskf(inp, noise_model(cfg), rot_cfg, flow_cfg, baro_cfg=baro_cfg, dir_cfg=dir_cfg)


# --------------------------------------------------------------- evaluation
def evaluate(out: EskfOutput, traj: Trajectory, cfg: dict, injected: dict | None) -> dict:
    from src.evaluation.trajectory_metrics import error_at_horizons, error_series, summarize
    res, k0 = out.result, out.result.start_index
    n = len(res.timestamp)
    pos = error_series(res, traj)
    att = attitude_error_series(res, traj)
    s = summarize(pos)
    m = {
        "position": {"error_at_s": {f"{h:g}": v for h, v in error_at_horizons(pos, cfg["error_horizons_s"]).items()},
                     "final_m": s.final, "ate_rmse_m": ate_rmse(res, traj), "mean_m": s.mean, "max_m": s.max,
                     "final_horizontal_m": s.final_horizontal, "final_vertical_m": s.final_vertical},
        "rpe": rpe(res, traj, cfg["rpe_delta_s"]),
        "attitude_deg": {"final": float(att.error_deg[-1]), "max": float(att.error_deg.max()),
                         "mean": float(att.error_deg.mean()),
                         "at_s": {f"{h:g}": float(np.interp(h, att.time_since_loss, att.error_deg))
                                  for h in cfg["error_horizons_s"] if h <= att.time_since_loss[-1]}},
    }
    # updates and NIS
    upd = {}
    for kind in ("rotation", "flow", "baro"):
        L = [u for u in out.updates if u.kind == kind]
        if not L:
            continue
        acc = [u for u in L if u.accepted]
        nis_acc = np.array([u.nis for u in acc])
        nis_all = np.array([u.nis for u in L if np.isfinite(u.nis)])
        reasons: dict[str, int] = {}
        for u in L:
            if not u.accepted:
                reasons[u.reason] = reasons.get(u.reason, 0) + 1
        summ = summarize_chi2(nis_acc, L[0].dof)
        upd[kind] = {"attempted": len(L), "accepted": len(acc), "accepted_percent": 100.0 * len(acc) / len(L),
                     "rejections": reasons, "nis_accepted": summ.as_dict() if summ else None,
                     "nis_all_evaluated_median": float(np.median(nis_all)) if nis_all.size else None}
    m["updates"] = upd
    # NEES on position, velocity, attitude (9 states) at the stored covariance samples
    idx = out.nav_cov_index
    e = state_error(res.position[idx], res.velocity[idx], res.attitude[idx], traj.position_gt[k0 + idx],
                    traj.velocity_gt[k0 + idx], traj.attitude_gt[k0 + idx])
    nees9 = np.array([nees(e[i], out.nav_cov[i][:9, :9]) for i in range(len(idx))])
    nees_att = np.array([nees(e[i, 6:9], out.nav_cov[i][6:9, 6:9]) for i in range(len(idx))])
    m["nees"] = {"nav9": summarize_chi2(nees9[1:], 9).as_dict(), "attitude3": summarize_chi2(nees_att[1:], 3).as_dict()}
    # biases
    nb_g, nb_a = native_imu_bias(traj)
    true_g, true_a = nb_g[k0:k0 + n], nb_a[k0:k0 + n]  # includes any injected bias (it is in the data)
    last = res.timestamp >= res.timestamp[-1] - BIAS_SCORE_WINDOW_S  # score biases over the last 20 s
    err = lambda est, ref: float(np.mean(np.linalg.norm(est[last] - ref[last] if np.ndim(ref) > 1 else est[last] - ref, axis=1)))  # noqa: E731
    m["bias"] = {
        "score_window_s": BIAS_SCORE_WINDOW_S,
        "gyro_est_mean_last": out.gyro_bias[last].mean(0).tolist(), "accel_est_mean_last": out.accel_bias[last].mean(0).tolist(),
        "gyro_total_true_mean_last": true_g[last].mean(0).tolist(), "accel_total_true_mean_last": true_a[last].mean(0).tolist(),
        "gyro_err_vs_total": err(out.gyro_bias, true_g), "accel_err_vs_total": err(out.accel_bias, true_a),
        "gyro_err_vs_total_if_zero": err(np.zeros_like(true_g), true_g),
        "accel_err_vs_total_if_zero": err(np.zeros_like(true_a), true_a),
    }
    if injected:
        bg_inj, ba_inj = np.asarray(injected["gyro"]), np.asarray(injected["accel"])
        m["bias"].update({
            "gyro_injected": bg_inj.tolist(), "accel_injected": ba_inj.tolist(),
            "gyro_err_vs_injected": err(out.gyro_bias, bg_inj), "accel_err_vs_injected": err(out.accel_bias, ba_inj),
            "gyro_err_vs_injected_if_zero": float(np.linalg.norm(bg_inj)),
            "accel_err_vs_injected_if_zero": float(np.linalg.norm(ba_inj)),
        })
    m["_series"] = {"t": pos.time_since_loss, "pos_err": pos.error, "att_err": att.error_deg,
                    "nees9_t": res.timestamp[idx] - res.t0, "nees9": nees9,
                    "bias_true_g": true_g, "bias_true_a": true_a}
    return m

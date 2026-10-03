"""Gyro-bias experiment: can relative visual rotations estimate the Mid-Air gyro bias
well enough to slow FUTURE attitude drift?

    python vio/scripts/run_bias_experiment.py synthetic
    python vio/scripts/run_bias_experiment.py tune          # sunny/trajectory_0001 only
    python vio/scripts/run_bias_experiment.py real          # the four diagnostic flights, frozen settings

Real-data methods: imu_only (baseline), complementary (current visual estimator, frozen),
bias_kf (new: bias-only filter, see vio/estimation/gyro_bias_kf.py), oracle (true attitude,
NOT an estimator). Same cutoff, loader, conventions, camera front end and metrics.

Images for the four diagnostic flights are no longer on disk. Their color_left feature
tracks, computed earlier by the same unchanged front end (same tracker settings, keyframe
age 25, from the first frame after the 5 s cutoff), are read from outputs/midair_eskf/cache/.
If frames.zip is present, tracks are recomputed instead.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import pickle
import sys
from pathlib import Path

import numpy as np
import yaml

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src.data.midair import MidAirConfig, load_midair_trajectory  # noqa: E402
from src.estimation.inertial_dead_reckoning import run_dead_reckoning  # noqa: E402
from src.evaluation.trajectory_metrics import error_series  # noqa: E402
from vio.benchmark.discovery import resolve_sensor_file  # noqa: E402
from vio.bias_experiment import DEG, growth_rates, intervals_from_measurements, run_on_trajectory, synthetic_case  # noqa: E402
from vio.estimation.gyro_bias_kf import BiasKFConfig  # noqa: E402
from vio.estimation.oracle import oracle_ground_truth_attitude  # noqa: E402
from vio.evaluation.attitude_metrics import attitude_error_series  # noqa: E402
from vio.evaluation.consistency import native_imu_bias  # noqa: E402
from vio.pipeline import CameraSetup, build_measurements, fusion_config, pose_config, run_front_end, run_fusion_on_trajectory, tracker_config  # noqa: E402

CFG_PATH = VIO_DIR / "configs" / "midair_bias_kf.yaml"
OUT = REPO_ROOT / "outputs" / "midair_bias_eskf"
FLIGHTS = [("sunny", "trajectory_0000"), ("sunny", "trajectory_0001"), ("cloudy", "trajectory_3000"), ("cloudy", "trajectory_3001")]
HORIZONS = [10.0, 30.0, 60.0]
STYLE = {"imu_only": ("#eb6834", "IMU only"), "complementary": ("#eda100", "Complementary (current)"),
         "bias_kf": ("#2a78d6", "Visual gyro-bias KF (new)"), "oracle": ("#1baf7a", "Oracle: true attitude")}


def kf_config(c: dict) -> BiasKFConfig:
    return BiasKFConfig(sigma_vis_deg=c["sigma_vis_deg"], bias_walk=c["bias_walk"], gyro_noise_density=c["gyro_noise_density"],
                        init_bias_std=c["init_bias_std"], init_att_std_deg=c["init_att_std_deg"],
                        consider_attitude=c["consider_attitude"], gate_prob=c["gate_prob"])


# ------------------------------------------------------------------ synthetic
def synthetic(cfg: dict) -> None:
    rows = []
    cases = [[0.02, 0, 0], [0, 0.02, 0], [0, 0, 0.02], [0.02, -0.01, 0.015], [0, 0, 0], [0.2, -0.1, 0.15]]
    for vis, gw in ((0.005, 0.0), (0.2, 1.2)):  # ideal camera; Mid-Air-like camera and gyro noise
        for bias in cases:
            traj, ivs, _ = synthetic_case(bias, vis_sigma_deg=vis, gyro_white_deg_s=gw, interval_s=cfg["interval_s"])
            kc = kf_config(cfg)
            kc.sigma_vis_deg, kc.gyro_noise_density = vis, gw * DEG * 0.1
            out = run_on_trajectory(traj, 5.0, ivs, kc)
            imu = attitude_error_series(run_dead_reckoning(traj, 5.0), traj)
            kf = attitude_error_series(out.result, traj)
            est = out.bias[-1] / DEG
            rows.append({"vis_sigma_deg": vis, "gyro_white_deg_s": gw, "injected_deg_s": bias, "estimated_deg_s": est.round(4).tolist(),
                         "error_deg_s": float(np.linalg.norm(est - bias)), "est_1sigma_deg_s": float(np.linalg.norm(out.bias_std[-1] / DEG)),
                         "att_final_imu_deg": float(imu.error_deg[-1]), "att_final_kf_deg": float(kf.error_deg[-1]),
                         "slope_60_end_imu": growth_rates(imu.time_since_loss, imu.error_deg)["60-ends"],
                         "slope_60_end_kf": growth_rates(kf.time_since_loss, kf.error_deg)["60-ends"]})
            r = rows[-1]
            print(f"vis {vis:5.3f} gyro {gw:3.1f} | injected {str(bias):20s} est {r['estimated_deg_s']} err {r['error_deg_s']:.4f} "
                  f"(1sd {r['est_1sigma_deg_s']:.4f}) deg/s | att final IMU {r['att_final_imu_deg']:.2f} KF {r['att_final_kf_deg']:.2f} deg")
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "synthetic_summary.json").write_text(json.dumps(rows, indent=2))


# ------------------------------------------------------------------ real data
def load_flight(cond: str, name: str):
    root = REPO_ROOT / "data" / "MidAir"
    cache = REPO_ROOT / "outputs" / "midair_benchmark" / "cache" / "sensor_records"
    sensor, _, problem = resolve_sensor_file(root / "Kite_training" / cond, cache, "Kite_training", cond)
    if sensor is None:
        raise FileNotFoundError(problem)
    base = yaml.safe_load((REPO_ROOT / "baseline/configs/midair_baseline.yaml").read_text())["midair"]
    m = {**base, "condition": cond, "trajectory": name, "data_root": None}
    traj = load_midair_trajectory(MidAirConfig.from_dict(m), data_root=str(sensor.parent.parent.parent))
    traj.metadata["frames_dir"] = str(root / "Kite_training" / cond)
    return traj


def visual_measurements(traj, vio_cfg: dict, k0: int):
    setup = CameraSetup.from_config(vio_cfg, "left")
    frames = Path(traj.metadata["frames_dir"]) / setup.stream / traj.metadata["trajectory"] / "frames.zip"
    if frames.is_file():
        tracks, K, _, _ = run_front_end(traj, setup, tracker_config(vio_cfg), k0, 100.0)
        source = "recomputed from frames.zip"
    else:
        hits = sorted(glob.glob(str(REPO_ROOT / "outputs" / "midair_eskf" / "cache" / f"tracks_{traj.metadata['trajectory']}_color_left_*.pkl")))
        if not hits:
            raise FileNotFoundError(f"no images and no cached tracks for {traj.metadata['trajectory']}")
        tracks = pickle.loads(Path(hits[0]).read_bytes())
        if tracks[0].frame_index * 4 != k0:
            raise ValueError("cached tracks do not start at the cutoff frame")
        K = setup.intrinsics(512, 512)  # 1024 px frames at downscale 2
        source = f"cached tracks {Path(hits[0]).name} (same front end, keyframe age 25)"
    return build_measurements(tracks, K, setup, pose_config(vio_cfg), 100.0), source


def horizon_values(t, e):
    return {**{f"{h:g}s": float(np.interp(h, t, e)) if h <= t[-1] else None for h in HORIZONS}, "final": float(e[-1])}


def evaluate_flight(cond: str, name: str, cfg: dict, vio_cfg: dict, write: bool = True) -> dict:
    traj = load_flight(cond, name)
    k0 = traj.index_at(5.0)
    meas, source = visual_measurements(traj, vio_cfg, k0)
    frames = int(round(cfg["interval_s"] * 25))
    intervals = intervals_from_measurements(meas, frames)
    runs = {"imu_only": run_dead_reckoning(traj, 5.0)}
    runs["complementary"], _ = run_fusion_on_trajectory(traj, 5.0, meas, fusion_config(vio_cfg))
    kf = run_on_trajectory(traj, 5.0, intervals, kf_config(cfg))
    runs["bias_kf"] = kf.result
    runs["oracle"] = oracle_ground_truth_attitude(traj, 5.0)
    pos = {k: error_series(r, traj) for k, r in runs.items()}
    att = {k: attitude_error_series(r, traj) for k, r in runs.items()}
    t = pos["imu_only"].time_since_loss
    gt_g, _ = native_imu_bias(traj, window_s=10.0)  # EVALUATION ONLY: measured gyro minus true rate, low-passed
    gt_g = gt_g[k0:]
    last = t >= t[-1] - 30.0
    acc = [u for u in kf.updates if u.accepted]
    m = {
        "flight": f"{cond}/{name}", "visual_source": source, "settings": cfg,
        "methods": {k: {"position_m": horizon_values(t, pos[k].error), "attitude_deg": horizon_values(t, att[k].error_deg),
                        "attitude_growth_deg_per_s": growth_rates(t, att[k].error_deg)} for k in runs},
        "bias_kf": {
            "final_bias_deg_s": (kf.bias[-1] / DEG).tolist(), "final_bias_1sigma_deg_s": (kf.bias_std[-1] / DEG).tolist(),
            "gt_diagnostic_bias_last30s_deg_s": (gt_g[last].mean(0) / DEG).tolist(),
            "bias_error_vs_gt_diag_last30s_deg_s": float(np.mean(np.linalg.norm(kf.bias[last] - gt_g[last], axis=1)) / DEG),
            "gt_diag_magnitude_last30s_deg_s": float(np.mean(np.linalg.norm(gt_g[last], axis=1)) / DEG),
            "updates_attempted": len(kf.updates), "updates_accepted": len(acc),
            "interval_s_median": float(np.median([u.interval_s for u in kf.updates])) if kf.updates else None,
            "visual_rotation_deg_median": float(np.median([u.visual_rotation_deg for u in acc])) if acc else None,
            "residual_deg_median": float(np.median([u.residual_deg for u in acc])) if acc else None,
            "nis_mean": float(np.mean([u.nis for u in acc])) if acc else None,
            "rejections": {r: sum(1 for u in kf.updates if not u.accepted and u.reason == r) for r in {u.reason for u in kf.updates if not u.accepted}},
        },
    }
    if write:
        d = OUT / f"{cond}_{name}"
        d.mkdir(parents=True, exist_ok=True)
        (d / "metrics.json").write_text(json.dumps(m, indent=2))
        with open(d / "errors.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_since_loss_s"] + [f"pos_err_{k}_m" for k in runs] + [f"att_err_{k}_deg" for k in runs if k != "oracle"])
            for i in range(len(t)):
                w.writerow([f"{t[i]:.2f}"] + [f"{pos[k].error[i]:.4f}" for k in runs] + [f"{att[k].error_deg[i]:.5f}" for k in runs if k != "oracle"])
        with open(d / "bias_estimate.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_since_loss_s", "bg_x_deg_s", "bg_y_deg_s", "bg_z_deg_s", "std_x", "std_y", "std_z",
                        "gt_diag_x_deg_s", "gt_diag_y_deg_s", "gt_diag_z_deg_s"])
            for i in range(0, len(t), 10):
                w.writerow([f"{t[i]:.2f}"] + [f"{x:.5f}" for x in np.r_[kf.bias[i], kf.bias_std[i], gt_g[i]] / DEG])
        plot(t, {k: att[k].error_deg for k in runs if k != "oracle"}, "Attitude error (deg)", f"Attitude error, {cond}/{name}", d / "attitude_error.png")
        plot(t, {k: pos[k].error for k in runs}, "Position error (m)", f"Position error, {cond}/{name}", d / "position_error.png")
        plot_bias(t, kf.bias / DEG, kf.bias_std / DEG, gt_g / DEG, f"Gyro bias (world frame), {cond}/{name}", d / "gyro_bias.png")
    return m


def plot(t, series, ylabel, title, path):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for k, y in series.items():
        ax.plot(t, y, color=STYLE[k][0], lw=2, label=STYLE[k][1])
    ax.set_xlabel("Time since GNSS loss (s)"), ax.set_ylabel(ylabel), ax.set_title(title)
    ax.set_xlim(left=0), ax.set_ylim(bottom=0), ax.grid(color="#e4e3df"), ax.legend(frameon=False)
    fig.tight_layout(), fig.savefig(path, dpi=120), plt.close(fig)


def plot_bias(t, b, s, gt, title, path):
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for i, ax in enumerate(axes):
        ax.fill_between(t, b[:, i] - s[:, i], b[:, i] + s[:, i], color="#2a78d6", alpha=0.15, lw=0)
        ax.plot(t, b[:, i], color="#2a78d6", lw=2, label="estimate ±1σ")
        ax.plot(t, gt[:, i], color="#0b0b0b", lw=1.2, ls="--", label="GT-derived gyro error, 10 s low-pass (evaluation only)")
        ax.set_ylabel(f"b_{'xyz'[i]} (deg/s)"), ax.grid(color="#e4e3df")
    axes[0].set_title(title), axes[0].legend(frameon=False, fontsize=8), axes[-1].set_xlabel("Time since GNSS loss (s)")
    fig.tight_layout(), fig.savefig(path, dpi=120), plt.close(fig)


def tune(cfg: dict, vio_cfg: dict) -> None:
    """Small grid on sunny/trajectory_0001 only. Rule: mean NIS closest to 3 (consistent noise)."""
    rows = []
    for interval in (0.5, 1.0):
        for sv in (0.2, 0.4):
            for bw in (1e-5, 1e-4):
                c = {**cfg, "interval_s": interval, "sigma_vis_deg": sv, "bias_walk": bw}
                m = evaluate_flight("sunny", "trajectory_0001", c, vio_cfg, write=False)
                b, a = m["bias_kf"], m["methods"]["bias_kf"]["attitude_deg"]
                rows.append({"interval_s": interval, "sigma_vis_deg": sv, "bias_walk": bw, "nis_mean": b["nis_mean"],
                             "accepted": b["updates_accepted"], "attempted": b["updates_attempted"], "att_final_deg": a["final"],
                             "att_final_imu_deg": m["methods"]["imu_only"]["attitude_deg"]["final"]})
                print(rows[-1])
    best = min(rows, key=lambda r: abs(np.log(r["nis_mean"] / 3.0)) if r["nis_mean"] else np.inf)
    print("chosen (NIS closest to 3):", best)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "tuning.json").write_text(json.dumps({"rule": "mean NIS of accepted updates closest to 3", "grid": rows, "chosen": best}, indent=2))


def real(cfg: dict, vio_cfg: dict) -> None:
    summary = []
    for cond, name in FLIGHTS:
        m = evaluate_flight(cond, name, cfg, vio_cfg)
        summary.append(m)
        mm = m["methods"]
        print(f"\n{cond}/{name}  ({m['visual_source']})")
        for k in ("imu_only", "complementary", "bias_kf", "oracle"):
            a, p, g = mm[k]["attitude_deg"], mm[k]["position_m"], mm[k]["attitude_growth_deg_per_s"]
            atxt = f"att 10/30/60/final {a['10s']:.2f} {a['30s']:.2f} {a['60s']:.2f} {a['final']:.2f} deg | slope " + \
                   " ".join(f"{v:+.3f}" for v in g.values()) if k != "oracle" else "att 0 (true attitude)".ljust(58)
            print(f"  {k:14s} {atxt} | pos 10/30/60/final {p['10s']:.1f} {p['30s']:.1f} {p['60s']:.1f} {p['final']:.1f} m")
        b = m["bias_kf"]
        print(f"  bias est {np.round(b['final_bias_deg_s'], 3)} deg/s (1sd {np.round(b['final_bias_1sigma_deg_s'], 3)}) | "
              f"GT diag last 30 s {np.round(b['gt_diagnostic_bias_last30s_deg_s'], 3)} | err {b['bias_error_vs_gt_diag_last30s_deg_s']:.3f} "
              f"(GT diag |b| {b['gt_diag_magnitude_last30s_deg_s']:.3f}) | updates {b['updates_accepted']}/{b['updates_attempted']} "
              f"NIS {b['nis_mean']:.2f} vis rot {b['visual_rotation_deg_median']:.2f} deg residual {b['residual_deg_median']:.2f} deg")
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("phase", choices=["synthetic", "tune", "real"])
    a = ap.parse_args()
    cfg = yaml.safe_load(CFG_PATH.read_text())
    vio_cfg = yaml.safe_load((VIO_DIR / "configs" / "midair_vio.yaml").read_text())
    {"synthetic": lambda: synthetic(cfg), "tune": lambda: tune(cfg, vio_cfg), "real": lambda: real(cfg, vio_cfg)}[a.phase]()
    return 0


if __name__ == "__main__":
    sys.exit(main())

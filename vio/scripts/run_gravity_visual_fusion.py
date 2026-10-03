"""Quality-weighted relative vision + absolute gravity (roll/pitch) on Mid-Air, then NTU unchanged.

    python vio/scripts/run_gravity_visual_fusion.py            # all phases

Phases (frozen in this order):
1. Fit the visual quality rule on the 16 Mid-Air diagnostic flights that are NOT test flights.
2. Tune gravity (norm tolerance, measurement noise) on sunny 0001 + cloudy 3000 only.
   Rule: minimise the mean of (final attitude error / IMU-only final attitude error) over the two.
3. Holdouts sunny 0000, cloudy 3001 with everything frozen.
4. NTU VIRAL with the same rule and gravity logic; only sensor constants change (gyro noise from
   the calibration file, camera calibration, gravity magnitude measured at rest before the cutoff).

Methods: imu_only, complementary (Mid-Air only), bias_kf_fixed (sigma 0.2 deg), quality_kf,
quality_gravity_kf, oracle (Mid-Air only). Same 0.5 s visual intervals for all bias filters.
"""
from __future__ import annotations

import copy
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(VIO_DIR / "scripts"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from run_bias_experiment import load_flight, visual_measurements  # noqa: E402
from src.estimation.inertial_dead_reckoning import NavState, run_dead_reckoning  # noqa: E402
from src.evaluation.trajectory_metrics import error_series  # noqa: E402
from vio.bias_experiment import growth_rates  # noqa: E402
from vio.diagnostics import sources  # noqa: E402
from vio.estimation.gyro_bias_kf import BiasInterval, BiasKFConfig, run_bias_kf  # noqa: E402
from vio.estimation.oracle import oracle_ground_truth_attitude  # noqa: E402
from vio.estimation.visual_quality import QualityModel, fit_quality_model  # noqa: E402
from vio.evaluation.attitude_metrics import geodesic_angle_deg, tilt_heading_error_deg  # noqa: E402
from vio.pipeline import fusion_config, run_fusion_on_trajectory  # noqa: E402
from vio.vision.camera import pinhole_intrinsics  # noqa: E402
from vio.vision.relative_pose import PoseConfig  # noqa: E402

OUT = REPO_ROOT / "outputs" / "gravity_visual_fusion"
DIAG = REPO_ROOT / "outputs" / "visual_rotation_diagnostic"
TEST = [("sunny", "trajectory_0000"), ("sunny", "trajectory_0001"), ("cloudy", "trajectory_3000"), ("cloudy", "trajectory_3001")]
TUNE = [("sunny", "trajectory_0001"), ("cloudy", "trajectory_3000")]
HOLD = [("sunny", "trajectory_0000"), ("cloudy", "trajectory_3001")]
GRID = [(nt, sg) for nt in (0.2, 0.5) for sg in (1.0, 3.0, 6.0)]
GRAVITY_STD_TOL = 0.5  # m/s^2, fixed
HORIZONS = (10.0, 30.0, 60.0)
STYLE = {"imu_only": "#eb6834", "complementary": "#eda100", "bias_kf_fixed": "#e87ba4", "quality_kf": "#2a78d6",
         "quality_gravity_kf": "#4a3aa7", "oracle": "#1baf7a"}


def base_cfg() -> dict:
    return yaml.safe_load((VIO_DIR / "configs" / "midair_bias_kf.yaml").read_text())


def kf(c: dict, **over) -> BiasKFConfig:
    k = BiasKFConfig(sigma_vis_deg=c["sigma_vis_deg"], bias_walk=c["bias_walk"], gyro_noise_density=c["gyro_noise_density"],
                     init_bias_std=c["init_bias_std"], init_att_std_deg=c["init_att_std_deg"],
                     consider_attitude=c["consider_attitude"], gate_prob=c["gate_prob"])
    for key, v in over.items():
        setattr(k, key, v)
    return k


# ------------------------------------------------------------------ phase 1
def fit_quality() -> QualityModel:
    rows = list(csv.DictReader(open(DIAG / "midair" / "intervals.csv")))
    dev = [r for r in rows if (r["condition"], r["sequence"]) not in set(TEST)]
    f = lambda k: np.array([float(r[k]) for r in dev])  # noqa: E731
    m = fit_quality_model(f("e_vis_norm_deg"), f("median_flow_px") / 256.0, f("inlier_ratio"), f("track_count"))
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "quality_model.json").write_text(json.dumps({**m.as_dict(), "fitted_on": sorted({f"{r['condition']}/{r['sequence']}" for r in dev}),
                                                        "n_intervals": len(dev)}, indent=2))
    return m


# ------------------------------------------------------------------ evaluation helpers
def att_metrics(q_est, q_ref, t, g_unit) -> dict:
    tot = geodesic_angle_deg(q_est, q_ref)
    tilt, head = tilt_heading_error_deg(q_est, q_ref, g_unit)
    at = lambda e, h: float(np.interp(h, t, e)) if h <= t[-1] else None  # noqa: E731
    out = {}
    for name, e in (("total", tot), ("tilt", tilt), ("heading", np.abs(head))):
        out[name] = {**{f"{h:g}s": at(e, h) for h in HORIZONS}, "final": float(e[-1]), "growth_deg_s": growth_rates(t, e)}
    return out, tot, tilt, np.abs(head)


def gravity_stats(glogs) -> dict:
    if not glogs:
        return {"opportunities": 0}
    acc = [g for g in glogs if g.accepted]
    rej = [g for g in glogs if not g.accepted]
    dev = np.array([g.norm_deviation for g in glogs])
    return {"opportunities": len(glogs), "accepted": len(acc), "rejected": len(rej), "accepted_percent": 100.0 * len(acc) / len(glogs),
            "norm_deviation_percentiles_m_s2": np.percentile(dev, [10, 50, 90]).tolist(),
            "residual_deg_accepted_mean": float(np.mean([g.residual_deg for g in acc])) if acc else None,
            "residual_deg_rejected_mean": float(np.mean([g.residual_deg for g in rej])) if rej else None,
            "rejection_reasons": {r: sum(1 for g in rej if g.reason == r) for r in {g.reason for g in rej}}}


def save_flight(d: Path, t, series: dict, pos: dict, quality_rows, glogs, metrics):
    d.mkdir(parents=True, exist_ok=True)
    (d / "metrics.json").write_text(json.dumps(metrics, indent=2, default=float))
    keys = list(series)
    with open(d / "errors.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_since_loss_s"] + [f"{k}_{q}" for k in keys for q in ("total_deg", "tilt_deg", "heading_deg")]
                   + [f"{k}_pos_m" for k in pos])
        for i in range(0, len(t), 10):
            w.writerow([f"{t[i]:.2f}"] + [f"{series[k][q][i]:.4f}" for k in keys for q in range(3)] + [f"{pos[k][i]:.3f}" for k in pos])
    with open(d / "visual_quality.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_start_s", "t_end_s", "median_flow_px", "flow_rad", "inlier_ratio", "track_count", "sigma_vis_deg"])
        w.writerows(quality_rows)
    with open(d / "gravity_updates.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["imu_index", "accepted", "norm_deviation_m_s2", "norm_std_m_s2", "residual_deg", "reason"])
        w.writerows([[g.imu_index, int(g.accepted), f"{g.norm_deviation:.4f}", f"{g.norm_std:.4f}", f"{g.residual_deg:.3f}", g.reason] for g in glogs])
    for qi, name in enumerate(("total_attitude_error", "tilt_error", "heading_error")):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for k in keys:
            ax.plot(t, series[k][qi], lw=1.8, color=STYLE.get(k, "#52514e"), label=k)
        ax.set_xlabel("Time since GNSS loss (s)"), ax.set_ylabel(f"{name.replace('_', ' ')} (deg)"), ax.set_title(f"{d.name}: {name.replace('_', ' ')}")
        ax.set_xlim(left=0), ax.set_ylim(bottom=0), ax.grid(color="#e4e3df"), ax.legend(frameon=False, fontsize=8)
        fig.tight_layout(), fig.savefig(d / f"{name}.png", dpi=110), plt.close(fig)
    if pos:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for k, e in pos.items():
            ax.plot(t, e, lw=1.8, color=STYLE.get(k, "#52514e"), label=k)
        ax.set_xlabel("Time since GNSS loss (s)"), ax.set_ylabel("Position error (m)"), ax.set_title(f"{d.name}: position error")
        ax.set_xlim(left=0), ax.set_ylim(bottom=0), ax.grid(color="#e4e3df"), ax.legend(frameon=False, fontsize=8)
        fig.tight_layout(), fig.savefig(d / "position_error.png", dpi=110), plt.close(fig)


# ------------------------------------------------------------------ Mid-Air
def midair_intervals(cond, name, qm: QualityModel, vio_cfg):
    import pickle
    tracks = pickle.loads((DIAG / "cache" / "midair" / f"{cond}_{name}.pkl").read_bytes())
    age = sources.frames_for(25.0)
    K = pinhole_intrinsics(512, 512, vio_cfg["cameras"]["left"]["hfov_deg"])
    R_bc = np.asarray(vio_cfg["cameras"]["left"]["R_bc"], float)
    ivs = sources.intervals_from_tracks(tracks, np.arange(len(tracks) + 1) * 0.04, K, PoseConfig(**vio_cfg["pose"]), age)
    out_fixed, out_q, qrows = [], [], []
    for v in ivs:
        i, j = int(round(v.t_i * 100)), int(round(v.t_j * 100))
        C = R_bc @ v.C_cam @ R_bc.T
        s = float(qm.sigma_deg(v.median_flow_px, K[0, 0], v.inlier_ratio, v.track_count))
        out_fixed.append(BiasInterval(i, j, C))
        out_q.append(BiasInterval(i, j, C, sigma_deg=s))
        qrows.append([f"{v.t_i:.2f}", f"{v.t_j:.2f}", f"{v.median_flow_px:.2f}", f"{v.median_flow_px / K[0, 0]:.4f}",
                      f"{v.inlier_ratio:.3f}", v.track_count, f"{s:.4f}"])
    return out_fixed, out_q, qrows


def midair_flight(cond, name, qm, vio_cfg, gravity_params, write=True, only_gravity=False):
    traj = load_flight(cond, name)
    k0 = traj.index_at(5.0)
    c = base_cfg()
    fixed, qual, qrows = midair_intervals(cond, name, qm, vio_cfg)
    run = lambda ivs, cfg: run_bias_kf(traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], "world", traj.gravity_world,  # noqa: E731
                                       NavState(traj.position_gt[k0].copy(), traj.velocity_gt[k0].copy(), traj.attitude_gt[k0].copy()), k0, ivs, cfg)
    nt, sg = gravity_params
    grav = run(qual, kf(c, gravity_update=True, gravity_norm_tol=nt, gravity_sigma_deg=sg, gravity_std_tol=GRAVITY_STD_TOL))
    res = {"quality_gravity_kf": grav.result}
    res["imu_only"] = run_dead_reckoning(traj, 5.0)
    if not only_gravity:
        meas, _ = visual_measurements(traj, vio_cfg, k0)
        res["complementary"], _ = run_fusion_on_trajectory(traj, 5.0, meas, fusion_config(vio_cfg))
        res["bias_kf_fixed"] = run(fixed, kf(c, sigma_vis_deg=0.2)).result
        res["quality_kf"] = run(qual, kf(c)).result
        res["oracle"] = oracle_ground_truth_attitude(traj, 5.0)
    order = [k for k in ("imu_only", "complementary", "bias_kf_fixed", "quality_kf", "quality_gravity_kf", "oracle") if k in res]
    g_unit = traj.gravity_world / np.linalg.norm(traj.gravity_world)
    t = res["imu_only"].timestamp - res["imu_only"].t0
    m = {"flight": f"{cond}/{name}", "gravity_params": {"norm_tol_m_s2": nt, "sigma_deg": sg, "std_tol_m_s2": GRAVITY_STD_TOL},
         "methods": {}, "gravity": gravity_stats(grav.gravity),
         "visual": {"n_intervals": len(qual), "sigma_vis_deg_percentiles": np.percentile([iv.sigma_deg for iv in qual], [10, 50, 90]).tolist() if qual else None,
                    "accepted_quality_kf": sum(u.accepted for u in grav.updates), "attempted": len(grav.updates)}}
    series, pos = {}, {}
    for k in order:
        r = res[k]
        a, tot, tilt, head = att_metrics(r.attitude, traj.attitude_gt[k0:], t, g_unit)
        pe = error_series(r, traj).error
        m["methods"][k] = {"attitude": a, "position_final_m": float(pe[-1]),
                           "position_m": {f"{h:g}s": float(np.interp(h, t, pe)) for h in HORIZONS}}
        if k != "oracle":
            series[k] = (tot, tilt, head)
        pos[k] = pe
    if write:
        save_flight(OUT / f"midair_{cond}_{name}", t, series, pos, qrows, grav.gravity, m)
    return m


# ------------------------------------------------------------------ NTU
def ntu_flight(seq, qm, vio_cfg, gravity_params):
    cam = {"rtp_01": "right"}.get(seq, "left")
    sdir = DIAG / "cache" / "ntu" / seq
    d = dict(np.load(DIAG / "cache" / "ntu" / f"{seq}_{cam}_messages_v2.npz"))
    cal = sources.ntu_calibration(sdir, cam)
    imu_yaml = sdir / "imu_v100.yaml" if (sdir / "imu_v100.yaml").is_file() else DIAG / "cache" / "ntu" / "eee_03" / "imu_v100.yaml"
    txt = "\n".join(l for l in imu_yaml.read_text().splitlines() if not l.startswith("%YAML")).replace("!!opencv-matrix", "")
    gyro_std = float(next(l.split(":")[1] for l in txt.splitlines() if l.strip().startswith("gyro_std")))  # first entry: white noise
    t, a, w = d["imu_t"], d["imu_a"], d["imu_w"]
    Rq = Rotation.from_quat(d["imu_q_xyzw"])
    dt = float(np.median(np.diff(t)))
    # gravity in the VN100 world frame, measured in the first 2 s (before the cutoff): g_w = -mean(R f)
    pre = t < t[0] + 2.0
    g_w = -np.mean(Rq[pre].apply(a[pre]), axis=0)
    k0 = int(np.searchsorted(t, t[0] + 5.0))
    ivs, rate, age, nfr = sources.ntu_visual_intervals({"img_t": d["img_t"], "imgs": d["imgs"]}, cal, vio_cfg)
    fixed, qual, qrows = [], [], []
    for v in ivs:
        i, j = int(np.searchsorted(t, v.t_i)), int(np.searchsorted(t, v.t_j))
        if i < k0 or j >= len(t):
            continue
        C = cal["R_bc"] @ v.C_cam @ cal["R_bc"].T
        s = float(qm.sigma_deg(v.median_flow_px, cal["K"][0, 0], v.inlier_ratio, v.track_count))
        fixed.append(BiasInterval(i, j, C))
        qual.append(BiasInterval(i, j, C, sigma_deg=s))
        qrows.append([f"{v.t_i - t[k0]:.2f}", f"{v.t_j - t[k0]:.2f}", f"{v.median_flow_px:.2f}", f"{v.median_flow_px / cal['K'][0, 0]:.4f}",
                      f"{v.inlier_ratio:.3f}", v.track_count, f"{s:.4f}"])
    c = base_cfg()
    c["gyro_noise_density"] = gyro_std * np.sqrt(dt)  # sensor constant from the NTU calibration file
    q0 = Rq[k0].as_quat()[[3, 0, 1, 2]]
    init = NavState(np.zeros(3), np.zeros(3), q0)
    run = lambda ivs_, cfg: run_bias_kf(t[k0:], a[k0:], w[k0:], "body", g_w, init, k0, ivs_, cfg)  # noqa: E731
    nt, sg = gravity_params
    res = {"imu_only": run([], kf(c)), "bias_kf_fixed": run(fixed, kf(c, sigma_vis_deg=0.2)), "quality_kf": run(qual, kf(c)),
           "quality_gravity_kf": run(qual, kf(c, gravity_update=True, gravity_norm_tol=nt, gravity_sigma_deg=sg, gravity_std_tol=GRAVITY_STD_TOL))}
    tt = t[k0:] - t[k0]
    ref = Rq[k0:].as_quat()[:, [3, 0, 1, 2]]
    g_unit = g_w / np.linalg.norm(g_w)
    m = {"sequence": seq, "camera": cam, "reference": "VN100 orientation output (no GT attitude in NTU VIRAL)",
         "gravity_world_measured": g_w.tolist(), "gyro_noise_density": c["gyro_noise_density"], "image_rate_hz": rate,
         "n_intervals": len(qual), "duration_s": float(tt[-1]),
         "sigma_vis_deg_percentiles": np.percentile([iv.sigma_deg for iv in qual], [10, 50, 90]).tolist() if qual else None,
         "gravity": gravity_stats(res["quality_gravity_kf"].gravity), "methods": {},
         "bias_final_deg_s": {k: np.degrees(r.bias[-1]).tolist() for k, r in res.items()}}
    series = {}
    for k, r in res.items():
        am, tot, tilt, head = att_metrics(r.result.attitude, ref, tt, g_unit)
        m["methods"][k] = {"attitude": am}
        series[k] = (tot, tilt, head)
    save_flight(OUT / f"ntu_{seq}", tt, series, {}, qrows, res["quality_gravity_kf"].gravity, m)
    return m


def show(m, keyname):
    print(f"\n{m.get(keyname)}  gravity {m.get('gravity', {}).get('accepted', '-')}/{m.get('gravity', {}).get('opportunities', '-')}")
    for k, v in m["methods"].items():
        a = v["attitude"]
        pos = f" | pos final {v['position_final_m']:7.1f} m" if "position_final_m" in v else ""
        print(f"  {k:20s} total {a['total']['final']:6.2f} | tilt 10/30/60/final {a['tilt']['10s']:.2f}/{a['tilt']['30s']:.2f}/"
              f"{a['tilt']['60s'] if a['tilt']['60s'] is not None else float('nan'):.2f}/{a['tilt']['final']:.2f} | heading final {a['heading']['final']:6.2f}{pos}")


def main() -> int:
    vio_cfg = yaml.safe_load((VIO_DIR / "configs" / "midair_vio.yaml").read_text())
    qm = fit_quality()
    print("quality model (frozen):", qm)
    # phase 2: tune gravity on the two tuning flights only
    tuning = []
    for nt, sg in GRID:
        ratios = []
        for cond, name in TUNE:
            m = midair_flight(cond, name, qm, vio_cfg, (nt, sg), write=False, only_gravity=True)
            ratios.append(m["methods"]["quality_gravity_kf"]["attitude"]["total"]["final"] / m["methods"]["imu_only"]["attitude"]["total"]["final"])
        tuning.append({"norm_tol": nt, "sigma_deg": sg, "ratios": ratios, "mean_ratio": float(np.mean(ratios))})
        print(f"tune norm_tol {nt} sigma {sg}: final attitude / IMU = {np.round(ratios, 3)} mean {np.mean(ratios):.3f}", flush=True)
    best = min(tuning, key=lambda r: r["mean_ratio"])
    params = (best["norm_tol"], best["sigma_deg"])
    print("chosen gravity params:", params)
    (OUT / "tuning.json").write_text(json.dumps({"rule": "min mean(final attitude / IMU final attitude) over sunny 0001 + cloudy 3000",
                                                 "grid": tuning, "chosen": {"norm_tol": params[0], "sigma_deg": params[1], "std_tol": GRAVITY_STD_TOL}}, indent=2))
    summary = {"quality_model": qm.as_dict(), "gravity_params": params, "midair": [], "ntu": []}
    for cond, name in TUNE + HOLD:
        m = midair_flight(cond, name, qm, vio_cfg, params)
        m["role"] = "tuning" if (cond, name) in TUNE else "holdout"
        summary["midair"].append(m)
        show(m, "flight")
    for seq in ("eee_03", "sbs_01", "rtp_01"):
        t0 = time.time()
        m = ntu_flight(seq, qm, vio_cfg, params)
        summary["ntu"].append(m)
        show(m, "sequence")
        print(f"  ({time.time() - t0:.0f} s, {m['n_intervals']} intervals, gravity world {np.round(m['gravity_world_measured'], 3)}, bias final {np.round(m['bias_final_deg_s']['quality_gravity_kf'], 3)} deg/s)")
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())

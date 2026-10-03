"""Sun-direction attitude update on Mid-Air (quality-weighted vision + gravity, frozen) .

    python vio/scripts/run_sun_feasibility.py <flights...>     # detections per frame (image only)
    python vio/scripts/run_sun_orientation.py

Mid-Air has no date/time/location metadata, so no solar ephemeris. The world sun direction is
taken from the simulator's (static) lighting, CALIBRATED LEAVE-ONE-OUT: the densest 3-degree
cluster of the OTHER flights' detections in the same condition, mapped to the world with their
ground-truth attitude. This stands in for "simulator-provided sun direction"; the flight under
test never contributes. At run time the filter only sees the image-derived body ray (5 Hz),
the detector confidence and its own prediction (chi-square gate). Gravity and the visual
quality rule are frozen from outputs/gravity_visual_fusion/.
"""
from __future__ import annotations

import csv
import glob
import json
import sys
from pathlib import Path

import numpy as np

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(VIO_DIR / "scripts"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

from run_bias_experiment import load_flight  # noqa: E402
from run_gravity_visual_fusion import att_metrics, base_cfg, kf, midair_intervals  # noqa: E402
from src.estimation.inertial_dead_reckoning import NavState, run_dead_reckoning  # noqa: E402
from vio.estimation.gyro_bias_kf import SunObservation, run_bias_kf  # noqa: E402
from vio.estimation.oracle import oracle_ground_truth_attitude  # noqa: E402
from vio.estimation.visual_quality import QualityModel  # noqa: E402
from vio.sun.detector import azimuth_elevation_ned  # noqa: E402

OUT = REPO_ROOT / "outputs" / "sun_orientation"
FEAS = OUT / "feasibility"
FLIGHTS = [("sunny", "trajectory_0000"), ("sunny", "trajectory_0001"), ("cloudy", "trajectory_3000"), ("cloudy", "trajectory_3001"),
           ("sunset", "trajectory_1010"), ("sunset", "trajectory_1013")]
SUN_SIGMA_DEG = 5.0  # from the 5-15 deg cross-flight consistency of the detected direction; not tuned
SUN_EVERY_FRAMES = 5  # 5 Hz
STYLE = {"imu_only": "#eb6834", "quality_kf": "#2a78d6", "quality_gravity": "#4a3aa7", "quality_sun": "#eda100",
         "quality_gravity_sun": "#1baf7a"}


def detections(cond, name):
    p = FEAS / f"{cond}_{name}.csv"
    return list(csv.DictReader(open(p))) if p.is_file() else []


def calibrate_sun_world(cond, exclude):
    """Densest 3-degree cluster of the other flights' world-frame detections (offline calibration)."""
    sw = []
    for p in glob.glob(str(FEAS / f"{cond}_trajectory_*.csv")):
        if Path(p).stem == f"{cond}_{exclude}":
            continue
        sw += [[float(r["sw_x"]), float(r["sw_y"]), float(r["sw_z"])] for r in csv.DictReader(open(p)) if r["detected"] == "1"]
    if len(sw) < 20:
        return None, 0
    sw = np.array(sw)
    c3 = np.cos(np.radians(3))
    counts = np.array([(sw @ s > c3).sum() for s in sw])
    k = int(np.argmax(counts))
    centre = sw[sw @ sw[k] > c3].mean(0)
    return centre / np.linalg.norm(centre), int(counts[k])


def longest_gap(times, total):
    if not len(times):
        return float(total)
    t = np.r_[0.0, np.sort(times), total]
    return float(np.diff(t).max())


def run_flight(cond, name, qm, grav, vio_cfg):
    traj = load_flight(cond, name)
    k0 = traj.index_at(5.0)
    _, qual, _ = midair_intervals(cond, name, qm, vio_cfg)
    rows = detections(cond, name)
    sun_w, support = calibrate_sun_world(cond, name)
    obs = [SunObservation(4 * int(r["frame"]), np.array([float(r["sb_x"]), float(r["sb_y"]), float(r["sb_z"])]), float(r["confidence"]))
           for r in rows if r["detected"] == "1" and int(r["frame"]) % SUN_EVERY_FRAMES == 0 and 4 * int(r["frame"]) > k0]
    c = base_cfg()
    init = NavState(traj.position_gt[k0].copy(), traj.velocity_gt[k0].copy(), traj.attitude_gt[k0].copy())
    nt, sg = grav
    G = dict(gravity_update=True, gravity_norm_tol=nt, gravity_sigma_deg=sg, gravity_std_tol=0.5)
    Sn = dict(sun_update=True, sun_sigma_deg=SUN_SIGMA_DEG)
    run = lambda cfg, use_sun: run_bias_kf(traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], "world", traj.gravity_world,  # noqa: E731
                                           init, k0, qual, cfg, sun_observations=obs if use_sun else None, sun_world=sun_w if use_sun else None)
    outs = {"quality_kf": run(kf(c), False), "quality_gravity": run(kf(c, **G), False),
            "quality_sun": run(kf(c, **Sn), True), "quality_gravity_sun": run(kf(c, **G, **Sn), True)}
    res = {"imu_only": run_dead_reckoning(traj, 5.0), **{k: o.result for k, o in outs.items()}, "oracle": oracle_ground_truth_attitude(traj, 5.0)}
    g_unit = traj.gravity_world / np.linalg.norm(traj.gravity_world)
    t = res["imu_only"].timestamp - res["imu_only"].t0
    slog = outs["quality_gravity_sun"].sun
    acc = [s for s in slog if s.accepted]
    conf = [float(r["confidence"]) for r in rows if r["detected"] == "1"]
    m = {"flight": f"{cond}/{name}", "sun_world": None if sun_w is None else {"vector_ned": sun_w.tolist(), "azimuth_elevation_deg": azimuth_elevation_ned(sun_w),
                                                                            "calibration_support_detections": support, "source": "leave-one-out cluster of other flights"},
         "sun": {"frames_examined": len(rows), "sky_visible_fraction": float(np.mean([float(r["sky_fraction"]) > 0.05 for r in rows])) if rows else 0.0,
                 "detected_fraction": float(np.mean([r["detected"] == "1" for r in rows])) if rows else 0.0,
                 "update_attempts": len(slog), "accepted": len(acc), "rejected": len(slog) - len(acc),
                 "accepted_percent": 100.0 * len(acc) / len(slog) if slog else None,
                 "residual_deg_mean": float(np.mean([s.residual_deg for s in slog])) if slog else None,
                 "residual_deg_median": float(np.median([s.residual_deg for s in slog])) if slog else None,
                 "residual_deg_accepted_median": float(np.median([s.residual_deg for s in acc])) if acc else None,
                 "confidence_percentiles": np.percentile(conf, [10, 50, 90]).tolist() if conf else None,
                 "longest_gap_without_accepted_sun_s": longest_gap(np.array([traj.timestamp[s.imu_index] - traj.timestamp[k0] for s in acc]), float(t[-1])),
                 "rejection_reasons": {r: sum(1 for s in slog if not s.accepted and s.reason == r) for r in {s.reason for s in slog if not s.accepted}}},
         "methods": {}}
    series = {}
    for k, r in res.items():
        a, tot, tilt, head = att_metrics(r.attitude, traj.attitude_gt[k0:], t, g_unit)
        m["methods"][k] = {"attitude": a}
        if k != "oracle":
            series[k] = (tot, tilt, head)
    d = OUT / f"{cond}_{name}"
    d.mkdir(parents=True, exist_ok=True)
    (d / "metrics.json").write_text(json.dumps(m, indent=2, default=float))
    with open(d / "attitude_errors.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_since_loss_s"] + [f"{k}_{q}_deg" for k in series for q in ("total", "tilt", "heading")])
        for i in range(0, len(t), 10):
            w.writerow([f"{t[i]:.2f}"] + [f"{series[k][q][i]:.4f}" for k in series for q in range(3)])
    with open(d / "sun_detections.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "detected", "u", "v", "confidence", "sky_fraction"])
        w.writerows([[r["frame"], r["detected"], r["u"], r["v"], r["confidence"], r["sky_fraction"]] for r in rows])
    with open(d / "sun_updates.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["imu_index", "accepted", "residual_deg", "nis", "reason"])
        w.writerows([[s.imu_index, int(s.accepted), f"{s.residual_deg:.3f}", f"{s.nis:.3f}", s.reason] for s in slog])
    for qi, nm in enumerate(("total_attitude_error", "tilt_error", "heading_error")):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for k, v in series.items():
            ax.plot(t, v[qi], lw=1.8, color=STYLE.get(k, "#52514e"), label=k)
        ax.set_xlabel("Time since GNSS loss (s)"), ax.set_ylabel(f"{nm.replace('_', ' ')} (deg)"), ax.set_title(f"{cond}/{name}: {nm.replace('_', ' ')}")
        ax.set_xlim(left=0), ax.set_ylim(bottom=0), ax.grid(color="#e4e3df"), ax.legend(frameon=False, fontsize=8)
        fig.tight_layout(), fig.savefig(d / f"{nm}.png", dpi=110), plt.close(fig)
    if slog:
        fig, ax = plt.subplots(figsize=(8, 4))
        ts = np.array([traj.timestamp[s.imu_index] - traj.timestamp[k0] for s in slog])
        ok = np.array([s.accepted for s in slog])
        r = np.array([s.residual_deg for s in slog])
        ax.scatter(ts[ok], r[ok], s=10, color="#2a78d6", label="accepted"), ax.scatter(ts[~ok], r[~ok], s=10, color="#eb6834", label="rejected")
        ax.set_yscale("log"), ax.set_xlabel("Time since GNSS loss (s)"), ax.set_ylabel("Sun residual (deg)"), ax.set_title(f"{cond}/{name}: sun residual")
        ax.grid(color="#e4e3df"), ax.legend(frameon=False), fig.tight_layout(), fig.savefig(d / "sun_residual.png", dpi=110), plt.close(fig)
    return m


def main() -> int:
    vio_cfg = yaml.safe_load((VIO_DIR / "configs" / "midair_vio.yaml").read_text())
    q = json.loads((REPO_ROOT / "outputs" / "gravity_visual_fusion" / "quality_model.json").read_text())
    qm = QualityModel(q["sigma0_deg"], q["alpha_flow"], q["beta_inlier"], q["gamma_tracks"])
    tj = json.loads((REPO_ROOT / "outputs" / "gravity_visual_fusion" / "tuning.json").read_text())["chosen"]
    grav = (tj["norm_tol"], tj["sigma_deg"])
    summary = []
    for cond, name in FLIGHTS:
        m = run_flight(cond, name, qm, grav, vio_cfg)
        summary.append(m)
        s = m["sun"]
        sw = m["sun_world"]["azimuth_elevation_deg"] if m["sun_world"] else None
        print(f"\n{cond}/{name}: sun world {np.round(sw, 1) if sw else 'n/a'} | detected {s['detected_fraction'] * 100:.1f}% of frames | "
              f"updates {s['accepted']}/{s['update_attempts']} | residual median {s['residual_deg_median']} | longest gap {s['longest_gap_without_accepted_sun_s']:.0f} s")
        for k, v in m["methods"].items():
            a = v["attitude"]
            print(f"  {k:20s} total {a['total']['final']:6.2f} | tilt 10/30/60/final {a['tilt']['10s']:.2f}/{a['tilt']['30s']:.2f}/{a['tilt']['60s']:.2f}/{a['tilt']['final']:.2f} "
                  f"| heading 10/30/60/final {a['heading']['10s']:.2f}/{a['heading']['30s']:.2f}/{a['heading']['60s']:.2f}/{a['heading']['final']:.2f}")
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())

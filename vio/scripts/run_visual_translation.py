"""Visual translational motion: direction quality, inertial-scale velocity, acceleration feasibility.

    python vio/scripts/run_visual_translation.py

Uses cached feature tracks only (no new feature extraction for Mid-Air):
  forward camera color_left: the 20 diagnostic flights, 0.52 s keyframe spans
      (outputs/visual_rotation_diagnostic/cache/midair)
  down camera color_down: consecutive 40 ms pairs for the four diagnostic flights
      (outputs/midair_eskf/cache)
Ground truth is used ONLY to score. The deployable quantities use the image, the IMU-only
dead-reckoning attitude and its speed.

Acceptance of a translation interval: valid essential matrix and translational parallax
>= PARALLAX_MIN_PX (flow left after removing the estimated rotation). Below that there is no
measurable translation and the direction is undefined (zero-motion handling).

Acceleration: causal moving linear fit over the last 5 intervals (~2.6 s); lag ~1.3 s.
NTU: Leica positions are in the tracker's own frame, not the VN100 world frame. One global
rotation between them is fitted on the FIRST half of each sequence and the direction error is
reported on the SECOND half only.
"""
from __future__ import annotations

import csv
import glob
import json
import pickle
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp
from scipy.stats import spearmanr

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(VIO_DIR / "scripts"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from run_bias_experiment import load_flight  # noqa: E402
from run_visual_rotation_diagnostic import MIDAIR_FLIGHTS, load_midair  # noqa: E402
from src.data.trajectory import quat_wxyz_to_rotation  # noqa: E402
from src.estimation.inertial_dead_reckoning import run_dead_reckoning  # noqa: E402
from vio.diagnostics import sources  # noqa: E402
from vio.diagnostics.translation import causal_slope, direction_error_deg, inertial_scaled_velocity, sign_agnostic  # noqa: E402
from vio.vision.camera import pinhole_intrinsics  # noqa: E402
from vio.vision.relative_pose import PoseConfig, estimate_relative_pose  # noqa: E402

OUT = REPO_ROOT / "outputs" / "visual_translation"
DIAG = REPO_ROOT / "outputs" / "visual_rotation_diagnostic"
PARALLAX_MIN_PX = 2.0
SLOPE_WINDOW = 5
FOUR = [("sunny", "trajectory_0000"), ("sunny", "trajectory_0001"), ("cloudy", "trajectory_3000"), ("cloudy", "trajectory_3001")]
G = 9.81


def cfg():
    return yaml.safe_load((VIO_DIR / "configs" / "midair_vio.yaml").read_text())


def left_intervals(cond, name, vcfg):
    tracks = pickle.loads((DIAG / "cache" / "midair" / f"{cond}_{name}.pkl").read_bytes())
    K = pinhole_intrinsics(512, 512, 90.0)
    ivs = sources.intervals_from_tracks(tracks, np.arange(len(tracks) + 1) * 0.04, K, PoseConfig(**vcfg["pose"]), sources.frames_for(25.0))
    spans = sum(1 for t in tracks if getattr(t, "reanchor", "") == "age" and t.frame_index - t.keyframe_index == sources.frames_for(25.0))
    return ivs, spans


def stats(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return {"n": int(x.size), "median": float(np.median(x)) if x.size else None, "p90": float(np.percentile(x, 90)) if x.size else None,
            "mean": float(x.mean()) if x.size else None}


# ------------------------------------------------------------------ 1. forward camera, 20 flights
def forward_direction(vcfg):
    R_bc = np.asarray(vcfg["cameras"]["left"]["R_bc"], float)
    allrows, per = [], {}
    for spec in MIDAIR_FLIGHTS:
        traj, cond, _ = load_midair(spec)
        name = traj.metadata["trajectory"]
        ivs, spans = left_intervals(cond, name, vcfg)
        R = quat_wxyz_to_rotation(traj.attitude_gt)
        rows = []
        for v in ivs:
            i, j = int(round(v.t_i * 100)), int(round(v.t_j * 100))
            if j >= len(traj) or v.t_dir_cam is None:
                continue
            t_gt = R[i].inv().apply(traj.position_gt[j] - traj.position_gt[i])  # EVALUATION
            t_vis = R_bc @ v.t_dir_cam
            e = float(direction_error_deg(t_vis, t_gt)[0])
            rows.append({"flight": f"{cond}/{name}", "t_i": v.t_i, "t_j": v.t_j, "accepted": int(v.parallax_px >= PARALLAX_MIN_PX),
                         "dir_err_deg": e, "dir_err_sign_agnostic_deg": float(sign_agnostic(np.array([e]))[0]),
                         "parallax_px": v.parallax_px, "true_translation_m": float(np.linalg.norm(t_gt)), "tracks": v.track_count,
                         "inliers": v.inliers, "inlier_ratio": v.inlier_ratio,
                         "rotation_deg": float(np.degrees(Rotation.from_matrix(v.C_cam).magnitude()))})
        acc = [r for r in rows if r["accepted"]]
        per[f"{cond}/{name}"] = {"spans": spans, "valid_pose": len(rows), "accepted": len(acc), "accepted_fraction_of_spans": len(acc) / max(spans, 1),
                                 "dir_err_deg": stats([r["dir_err_deg"] for r in acc])}
        allrows += rows
    acc = [r for r in allrows if r["accepted"]]
    e = np.array([r["dir_err_deg"] for r in acc])
    summary = {"camera": "color_left (forward)", "flights": len(per), "intervals_valid_pose": len(allrows), "accepted": len(acc),
               "accepted_fraction_of_spans": len(acc) / max(sum(p["spans"] for p in per.values()), 1),
               "dir_err_deg": stats(e), "dir_err_sign_agnostic_deg": stats([r["dir_err_sign_agnostic_deg"] for r in acc]),
               "share_flipped_over_90deg": float(np.mean(e > 90)) if e.size else None,
               "spearman_err_vs_parallax": float(spearmanr([r["parallax_px"] for r in acc], e).statistic) if len(acc) > 5 else None,
               "spearman_err_vs_true_translation": float(spearmanr([r["true_translation_m"] for r in acc], e).statistic) if len(acc) > 5 else None,
               "dir_err_by_parallax_quartile": [], "per_flight": per}
    par = np.array([r["parallax_px"] for r in acc])
    for lo, hi in zip(np.percentile(par, [0, 25, 50, 75]), np.percentile(par, [25, 50, 75, 100])):
        sel = (par >= lo) & (par <= hi)
        summary["dir_err_by_parallax_quartile"].append({"parallax_px": [float(lo), float(hi)], "median_err_deg": float(np.median(e[sel]))})
    return summary, allrows


# ------------------------------------------------------------------ 2. down camera, 4 flights
def down_camera(vcfg):
    R_bc = np.asarray(vcfg["cameras"]["down"]["R_bc"], float)
    K = pinhole_intrinsics(512, 512, 90.0)
    out = {}
    for cond, name in FOUR:
        hits = sorted(glob.glob(str(REPO_ROOT / "outputs" / "midair_eskf" / "cache" / f"tracks_{name}_color_down_*.pkl")))
        if not hits:
            continue
        tracks = pickle.loads(Path(hits[0]).read_bytes())
        traj = load_flight(cond, name)
        R = quat_wxyz_to_rotation(traj.attitude_gt)
        errs, flows, speeds = [], [], []
        for tr in tracks[::5]:
            if tr.new_keyframe or len(tr.current_points) < 30:
                continue
            i, j = 4 * tr.keyframe_index, 4 * tr.frame_index
            if j >= len(traj):
                continue
            v = traj.velocity_gt[j]
            flows.append(float(np.median(np.linalg.norm(tr.current_points - tr.keyframe_points, axis=1))))
            speeds.append(float(np.linalg.norm(v[:2])))
            p = estimate_relative_pose(tr.keyframe_points, tr.current_points, K, PoseConfig(**vcfg["pose"]))
            if p.valid:
                errs.append(float(direction_error_deg(R_bc @ p.translation_dir, R[i].inv().apply(traj.position_gt[j] - traj.position_gt[i]))[0]))
        out[f"{cond}/{name}"] = {"pairs": len(flows), "dir_err_deg_40ms_pairs": stats(errs),
                                 "spearman_flow_vs_true_horizontal_speed": float(spearmanr(flows, speeds).statistic) if len(flows) > 5 else None}
    return out


# ------------------------------------------------------------------ 3. velocity + acceleration, 4 flights
def velocity_flight(cond, name, vcfg):
    R_bc = np.asarray(vcfg["cameras"]["left"]["R_bc"], float)
    traj = load_flight(cond, name)
    k0 = traj.index_at(5.0)
    imu = run_dead_reckoning(traj, 5.0)  # deployable: IMU-only attitude, velocity and position
    Rh = quat_wxyz_to_rotation(imu.attitude)
    ivs, _ = left_intervals(cond, name, vcfg)
    rows = []
    for v in ivs:
        i, j = int(round(v.t_i * 100)), int(round(v.t_j * 100))
        if i < k0 or j >= len(traj) or v.t_dir_cam is None:
            continue
        dt = v.t_j - v.t_i
        v_imu = (imu.position[j - k0] - imu.position[i - k0]) / dt  # mean IMU velocity over the interval
        v_gt = (traj.position_gt[j] - traj.position_gt[i]) / dt  # EVALUATION
        ok = v.parallax_px >= PARALLAX_MIN_PX
        t_w = Rh[i - k0].apply(R_bc @ v.t_dir_cam)
        v_vis = inertial_scaled_velocity(t_w, np.array([np.linalg.norm(v_imu)]))[0] if ok else v_imu  # fall back to IMU when no parallax
        rows.append({"t_mid": 0.5 * (v.t_i + v.t_j) - traj.timestamp[k0], "accepted": int(ok),
                     "imu_dir_err_deg": float(direction_error_deg(v_imu, v_gt)[0]), "vis_dir_err_deg": float(direction_error_deg(v_vis, v_gt)[0]),
                     "imu_vec_err": float(np.linalg.norm(v_imu - v_gt)), "vis_vec_err": float(np.linalg.norm(v_vis - v_gt)),
                     "speed_err": float(abs(np.linalg.norm(v_imu) - np.linalg.norm(v_gt))), "v_imu": v_imu, "v_vis": v_vis, "v_gt": v_gt,
                     "a_gt_inst": np.mean(np.gradient(traj.velocity_gt[i:j + 1], 0.01, axis=0), axis=0)})
    t = np.array([r["t_mid"] for r in rows])
    a_vis = causal_slope(t, np.array([r["v_vis"] for r in rows]), SLOPE_WINDOW)
    a_imu = causal_slope(t, np.array([r["v_imu"] for r in rows]), SLOPE_WINDOW)
    a_gtv = causal_slope(t, np.array([r["v_gt"] for r in rows]), SLOPE_WINDOW)  # same smoother on true velocity: the best this window can do
    a_true = np.array([r["a_gt_inst"] for r in rows])
    ea_vis, ea_imu, ea_gtv = (np.linalg.norm(x - a_true, axis=1) for x in (a_vis, a_imu, a_gtv))
    zero = np.linalg.norm(a_true, axis=1)  # error of the gravity update's assumption a = 0
    horiz = lambda x: np.linalg.norm((x - a_true)[:, :2], axis=1)  # noqa: E731
    at = lambda key, h: rows[int(np.argmin(np.abs(t - h)))][key] if len(rows) and (h <= t[-1] + 1) else None  # noqa: E731
    m = {"flight": f"{cond}/{name}", "intervals": len(rows), "accepted": int(sum(r["accepted"] for r in rows)),
         "direction_err_deg": {"imu_only": stats([r["imu_dir_err_deg"] for r in rows]), "vision_dir_imu_speed": stats([r["vis_dir_err_deg"] for r in rows])},
         "velocity_vector_err_m_s": {"imu_only": stats([r["imu_vec_err"] for r in rows]), "vision_dir_imu_speed": stats([r["vis_vec_err"] for r in rows])},
         "speed_err_m_s": stats([r["speed_err"] for r in rows]),
         "at_horizons": {f"{h:g}s" if h != "final" else "final": {k: (rows[-1][k] if h == "final" else at(k, h)) for k in ("imu_dir_err_deg", "vis_dir_err_deg", "imu_vec_err", "vis_vec_err", "speed_err")}
                         for h in (10.0, 30.0, 60.0, "final")},
         "acceleration_err_m_s2": {"vision_dir_imu_speed": stats(ea_vis), "imu_only": stats(ea_imu), "true_velocity_same_smoother": stats(ea_gtv),
                                   "assume_zero_acceleration": stats(zero),
                                   "horizontal_vision": stats(horiz(a_vis)), "horizontal_zero_assumption": stats(np.linalg.norm(a_true[:, :2], axis=1))},
         "acceleration_smoother": f"causal least-squares slope over the last {SLOPE_WINDOW} intervals (~{SLOPE_WINDOW * 0.52:.1f} s)"}
    m["apparent_gravity_tilt_error_deg_median"] = {k: float(np.degrees(np.arctan(np.nanmedian(x) / G))) for k, x in
                                                   (("vision_dir_imu_speed", ea_vis), ("imu_only", ea_imu), ("assume_zero_acceleration", zero))}
    d = OUT / f"{cond}_{name}"
    d.mkdir(parents=True, exist_ok=True)
    (d / "metrics.json").write_text(json.dumps(m, indent=2, default=float))
    with open(d / "velocity_errors.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_since_loss_s", "accepted", "imu_dir_err_deg", "vis_dir_err_deg", "imu_vec_err_m_s", "vis_vec_err_m_s", "speed_err_m_s"])
        w.writerows([[f"{r['t_mid']:.2f}", r["accepted"], f"{r['imu_dir_err_deg']:.3f}", f"{r['vis_dir_err_deg']:.3f}", f"{r['imu_vec_err']:.3f}",
                      f"{r['vis_vec_err']:.3f}", f"{r['speed_err']:.3f}"] for r in rows])
    with open(d / "acceleration_errors.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_since_loss_s", "a_true_norm", "err_vision", "err_imu", "err_true_velocity_same_smoother"])
        w.writerows([[f"{t[k]:.2f}", f"{zero[k]:.3f}", f"{ea_vis[k]:.3f}", f"{ea_imu[k]:.3f}", f"{ea_gtv[k]:.3f}"] for k in range(len(t))])
    for key, ylab, fn in ((("imu_dir_err_deg", "vis_dir_err_deg"), "Velocity direction error (deg)", "velocity_direction_error.png"),
                          (("imu_vec_err", "vis_vec_err"), "Velocity vector error (m/s)", "velocity_error.png")):
        fig, ax = plt.subplots(figsize=(8, 4.2))
        ax.plot(t, [r[key[0]] for r in rows], color="#eb6834", lw=1.4, label="IMU only")
        ax.plot(t, [r[key[1]] for r in rows], color="#2a78d6", lw=1.4, label="vision direction x IMU speed")
        ax.set_xlabel("Time since GNSS loss (s)"), ax.set_ylabel(ylab), ax.set_title(f"{cond}/{name}"), ax.grid(color="#e4e3df"), ax.legend(frameon=False)
        fig.tight_layout(), fig.savefig(d / fn, dpi=110), plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(t, [r["speed_err"] for r in rows], color="#eb6834", lw=1.4)
    ax.set_xlabel("Time since GNSS loss (s)"), ax.set_ylabel("Speed error (m/s), IMU (shared by both)"), ax.set_title(f"{cond}/{name}"), ax.grid(color="#e4e3df")
    fig.tight_layout(), fig.savefig(d / "speed_error.png", dpi=110), plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(t, zero, color="#0b0b0b", lw=1.2, label="|a_true| (gravity update assumes 0)")
    ax.plot(t, ea_imu, color="#eb6834", lw=1.2, label="IMU-only velocity derivative")
    ax.plot(t, ea_vis, color="#2a78d6", lw=1.2, label="vision-direction velocity derivative")
    ax.plot(t, ea_gtv, color="#1baf7a", lw=1.2, label="true velocity, same smoother (floor)")
    ax.set_yscale("log"), ax.set_xlabel("Time since GNSS loss (s)"), ax.set_ylabel("Acceleration error (m/s²)"), ax.set_title(f"{cond}/{name}")
    ax.grid(color="#e4e3df"), ax.legend(frameon=False, fontsize=8), fig.tight_layout(), fig.savefig(d / "acceleration_error.png", dpi=110), plt.close(fig)
    return m, rows


# ------------------------------------------------------------------ 4. NTU
def ntu(vcfg):
    out = {}
    for seq, cam in (("eee_03", "left"), ("sbs_01", "left"), ("rtp_01", "right")):
        d = dict(np.load(DIAG / "cache" / "ntu" / f"{seq}_{cam}_messages_v2.npz"))
        cal = sources.ntu_calibration(DIAG / "cache" / "ntu" / seq, cam)
        ivs, _, _, _ = sources.ntu_visual_intervals({"img_t": d["img_t"], "imgs": d["imgs"]}, cal, vcfg)
        sl = Slerp(d["imu_t"], Rotation.from_quat(d["imu_q_xyzw"]))
        pt, P = d["pos_t"], d["pos"]
        tw, dl, par = [], [], []
        for v in ivs:
            if v.t_dir_cam is None or not (pt[0] <= v.t_i and v.t_j <= pt[-1] and d["imu_t"][0] <= v.t_i and v.t_j <= d["imu_t"][-1]):
                continue
            dp = np.array([np.interp(v.t_j, pt, P[:, k]) - np.interp(v.t_i, pt, P[:, k]) for k in range(3)])
            if np.linalg.norm(dp) < 0.05:  # < 5 cm: hovering, no direction to score
                continue
            tw.append(sl(v.t_i).apply(cal["R_bc"] @ v.t_dir_cam))
            dl.append(dp / np.linalg.norm(dp))
            par.append(v.parallax_px)
        tw, dl, par = np.array(tw), np.array(dl), np.array(par)
        if len(tw) < 20:
            out[seq] = {"intervals_scored": int(len(tw)), "note": "too few moving intervals"}
            continue
        h = len(tw) // 2
        U, _, Vt = np.linalg.svd(dl[:h].T @ tw[:h])  # rotation taking Leica-frame directions to VN100-world directions
        Ra = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
        e = direction_error_deg(tw[h:], dl[h:] @ Ra.T)
        acc = par[h:] >= PARALLAX_MIN_PX
        out[seq] = {"camera": cam, "intervals_scored_second_half": int(len(e)), "accepted": int(acc.sum()),
                    "dir_err_deg_accepted": stats(e[acc]), "dir_err_deg_all": stats(e)}
    return out


def main() -> int:
    vcfg = cfg()
    OUT.mkdir(parents=True, exist_ok=True)
    print("1. forward camera (color_left), 20 flights")
    fwd, rows = forward_direction(vcfg)
    with open(OUT / "translation_intervals.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(json.dumps({k: v for k, v in fwd.items() if k != "per_flight"}, indent=1, default=float))
    acc = [r for r in rows if r["accepted"]]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.scatter([r["parallax_px"] for r in acc], [r["dir_err_deg"] for r in acc], s=4, alpha=0.3, color="#2a78d6")
    ax.set_xscale("log"), ax.set_yscale("log"), ax.set_xlabel("Translational parallax (px)"), ax.set_ylabel("Translation-direction error (deg)")
    ax.set_title("Forward camera, 20 Mid-Air flights"), ax.grid(color="#e4e3df"), fig.tight_layout(), fig.savefig(OUT / "translation_direction_error.png", dpi=110), plt.close(fig)
    print("2. down camera (color_down), 40 ms pairs")
    down = down_camera(vcfg)
    print(json.dumps(down, indent=1, default=float))
    print("3. velocity and acceleration, four flights")
    vel = {}
    for cond, name in FOUR:
        m, _ = velocity_flight(cond, name, vcfg)
        vel[m["flight"]] = m
        dv, vv, a = m["direction_err_deg"], m["velocity_vector_err_m_s"], m["acceleration_err_m_s2"]
        print(f"  {m['flight']}: accepted {m['accepted']}/{m['intervals']} | direction err median IMU {dv['imu_only']['median']:.1f} vs vision {dv['vision_dir_imu_speed']['median']:.1f} deg | "
              f"vector err median IMU {vv['imu_only']['median']:.2f} vs vision {vv['vision_dir_imu_speed']['median']:.2f} m/s | accel err median vision {a['vision_dir_imu_speed']['median']:.2f}, "
              f"IMU {a['imu_only']['median']:.2f}, floor {a['true_velocity_same_smoother']['median']:.2f}, |a_true| {a['assume_zero_acceleration']['median']:.2f} m/s2")
    print("4. NTU transfer")
    nt = ntu(vcfg)
    print(json.dumps(nt, indent=1, default=float))
    (OUT / "summary.json").write_text(json.dumps({"forward": fwd, "down": down, "velocity": vel, "ntu": nt}, indent=2, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())

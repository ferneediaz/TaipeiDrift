"""Corrected ESKF baseline with the measured barometer, then the visual velocity-direction update.
Mid-Air (development) and NTU VIRAL (transfer, no retuning).

    python vio/scripts/run_eskf_direction.py

Mid-Air: the four diagnostic flights, 5 barometer seeds each (the barometer is the only random
input). Methods: imu_only, imu_baro (A), forward_rotation, both (B), forward_direction,
both_direction (C). One extra run per flight with the OLD barometer (0.30 m, 0.05 m/sqrt s, no drift,
no scale) shows how much the earlier 13-33 m numbers move.
NTU: imu_only, imu_baro, forward_rotation, forward_direction (no down camera, so no flow; the
barometer is SIMULATED from the Leica altitude with the measured model). Sensor constants (gyro
and accelerometer noise) come from NTU's calibration file; every other parameter is the Mid-Air one.
NTU positions are in the Leica tracker frame: for EVALUATION each method's trajectory is aligned
to it with one rotation (Kabsch, starts coincide at the cutoff); velocities use the same rotation.

Outputs: outputs/eskf_real_baro_baseline/ and outputs/eskf_visual_direction/.
"""
from __future__ import annotations

import copy
import csv
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src.data.midair import MidAirConfig, load_midair_trajectory  # noqa: E402
from src.estimation.inertial_dead_reckoning import NavState  # noqa: E402
from vio.diagnostics import sources  # noqa: E402
from vio.diagnostics.translation import causal_slope  # noqa: E402
from vio.eskf_pipeline import noise_model, prepare_visual_inputs, run_ablation  # noqa: E402
from vio.estimation.eskf_runner import BaroUpdateConfig, DirectionUpdateConfig, EskfInputs, RotationUpdateConfig, run_eskf  # noqa: E402
from vio.evaluation.attitude_metrics import geodesic_angle_deg, tilt_heading_error_deg  # noqa: E402
from vio.evaluation.consistency import native_imu_bias  # noqa: E402
from vio.sensors.simulated import BarometerConfig, generate_barometer, relative_altitude  # noqa: E402

OUT_BASE = REPO_ROOT / "outputs" / "eskf_real_baro_baseline"
OUT_DIR = REPO_ROOT / "outputs" / "eskf_visual_direction"
DIAG = REPO_ROOT / "outputs" / "visual_rotation_diagnostic"
FLIGHTS = [("sunny", "trajectory_0000"), ("sunny", "trajectory_0001"), ("cloudy", "trajectory_3000"), ("cloudy", "trajectory_3001")]
MIDAIR_METHODS = ["imu_only", "imu_baro", "forward_rotation", "both", "forward_direction", "both_direction"]
NTU_METHODS = ["imu_only", "imu_baro", "forward_rotation", "forward_direction"]
SEEDS = range(5)
HORIZONS = (10.0, 30.0, 60.0)
MIN_SPEED_EVAL = 0.3  # m/s: below this the true velocity direction is undefined for scoring
OLD_BARO = {"white_std_m": 0.30, "bias_walk_m_per_sqrt_s": 0.05, "drift_sigma_m_per_s": 0.0, "scale_error_min": 0.0, "scale_error_max": 0.0, "scale_error_rms": 0.0}


def at(t, e, h):
    return float(np.interp(h, t, e)) if h <= t[-1] else None


def evaluate(t, p, v, q, ba, bg, p_ref, v_ref, q_ref, g_unit, bias_truth=None) -> dict:
    pe = np.linalg.norm(p - p_ref, axis=1)
    spd = np.linalg.norm(v_ref, axis=1)
    mv = spd > MIN_SPEED_EVAL
    vdir = np.full(len(t), np.nan)
    vdir[mv] = np.degrees(np.arccos(np.clip(np.sum(v[mv] * v_ref[mv], 1) / np.maximum(np.linalg.norm(v[mv], axis=1), 1e-9) / spd[mv], -1, 1)))
    vvec = np.linalg.norm(v - v_ref, axis=1)
    sperr = np.abs(np.linalg.norm(v, axis=1) - spd)
    tot = geodesic_angle_deg(q, q_ref)
    tilt, head = tilt_heading_error_deg(q, q_ref, g_unit)
    # evaluation-only linear acceleration from the filter velocity (10 Hz, causal 1 s linear fit)
    sub = slice(None, None, max(1, int(round(0.1 / np.median(np.diff(t))))))
    a_est = causal_slope(t[sub], v[sub], 10)
    a_ref = causal_slope(t[sub], v_ref[sub], 10)
    ae = np.linalg.norm(a_est - a_ref, axis=1)
    out = {"position_m": {**{f"{h:g}s": at(t, pe, h) for h in HORIZONS}, "final": float(pe[-1]), "rmse": float(np.sqrt(np.mean(pe**2)))},
           "velocity": {"direction_err_deg_median": float(np.nanmedian(vdir)), "direction_err_deg_final": float(vdir[mv][-1]) if mv.any() else None,
                        "vector_err_m_s_median": float(np.median(vvec)), "vector_err_m_s_final": float(vvec[-1]),
                        "speed_err_m_s_median": float(np.median(sperr)), "speed_err_m_s_final": float(sperr[-1]),
                        "share_of_vector_err_from_speed_median": float(np.nanmedian(sperr / np.maximum(vvec, 1e-9)))},
           "attitude_deg": {"total_final": float(tot[-1]), "tilt_final": float(tilt[-1]), "heading_final": float(abs(head[-1]))},
           "bias_final": {"accel": ba[-1].tolist(), "gyro": bg[-1].tolist()},
           "accel_eval_only": {"err_m_s2_median": float(np.nanmedian(ae)), "ref_norm_m_s2_median": float(np.nanmedian(np.linalg.norm(a_ref, axis=1))),
                               "tilt_equivalent_err_deg_median": float(np.degrees(np.arctan(np.nanmedian(ae) / 9.81)))}}
    if bias_truth is not None:
        tb_g, tb_a = bias_truth
        out["bias_final"]["accel_err_vs_native"] = float(np.linalg.norm(ba[-1] - tb_a[-1]))
        out["bias_final"]["accel_err_if_zero"] = float(np.linalg.norm(tb_a[-1]))
        out["bias_final"]["gyro_err_vs_native"] = float(np.linalg.norm(bg[-1] - tb_g[-1]))
        out["bias_final"]["gyro_err_if_zero"] = float(np.linalg.norm(tb_g[-1]))
    out["_series"] = {"t": t, "pos": pe, "vdir": vdir, "vvec": vvec, "speed": sperr, "ba": ba, "bg": bg}
    return out


def median_over_seeds(ms: list[dict]) -> dict:
    def rec(objs):
        o0 = objs[0]
        if isinstance(o0, dict):
            return {k: rec([o[k] for o in objs]) for k in o0 if not str(k).startswith("_")}
        if isinstance(o0, (int, float)) and o0 is not None:
            vals = [o for o in objs if o is not None]
            return float(np.median(vals)) if vals else None
        if isinstance(o0, list):
            return np.median(np.array(objs), axis=0).tolist()
        return o0
    out = rec(ms)
    out["position_m"]["final_p95_over_seeds"] = float(np.percentile([m["position_m"]["final"] for m in ms], 95))
    return out


def save(d: Path, methods: dict, seed0: dict):
    d.mkdir(parents=True, exist_ok=True)
    (d / "metrics.json").write_text(json.dumps(methods, indent=2, default=float))
    t = next(iter(seed0.values()))["_series"]["t"]
    for fname, key in (("velocity_errors.csv", ("vdir", "vvec", "speed")), ("position_errors.csv", ("pos",)), ("bias_estimates.csv", ("ba", "bg"))):
        with open(d / fname, "w", newline="") as f:
            w = csv.writer(f)
            hdr = ["t_since_loss_s"]
            for m in seed0:
                for k in key:
                    hdr += [f"{m}_{k}_{c}" for c in "xyz"] if k in ("ba", "bg") else [f"{m}_{k}"]
            w.writerow(hdr)
            for i in range(0, len(t), 10):
                row = [f"{t[i]:.2f}"]
                for m, s in seed0.items():
                    for k in key:
                        row += [f"{x:.6f}" for x in s["_series"][k][i]] if k in ("ba", "bg") else [f"{s['_series'][k][i]:.4f}"]
                w.writerow(row)
    for key, ylab, fn in (("vdir", "Velocity direction error (deg)", "velocity_direction_error.png"), ("vvec", "Velocity vector error (m/s)", "velocity_vector_error.png"),
                          ("speed", "Speed error (m/s)", "speed_error.png"), ("pos", "Position error (m)", "position_error.png")):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for m, s in seed0.items():
            ax.plot(t, s["_series"][key], lw=1.4, label=m)
        ax.set_xlabel("Time since GNSS loss (s)"), ax.set_ylabel(ylab), ax.set_title(f"{d.name} (barometer seed 0)")
        ax.grid(color="#e4e3df"), ax.legend(frameon=False, fontsize=8), fig.tight_layout(), fig.savefig(d / fn, dpi=110), plt.close(fig)


# ------------------------------------------------------------------ Mid-Air
def midair(cfg, vio_cfg):
    base_mid = yaml.safe_load((REPO_ROOT / "baseline/configs/midair_baseline.yaml").read_text())["midair"]
    root = REPO_ROOT / "outputs" / "midair_benchmark" / "cache" / "sensor_records"
    results = {}
    for cond, name in FLIGHTS:
        traj = load_midair_trajectory(MidAirConfig.from_dict({**base_mid, "condition": cond, "trajectory": name, "data_root": None}), data_root=str(root))
        traj.metadata["frames_dir"] = str(REPO_ROOT / "data" / "MidAir_3_trajecotry" / "Kite_training" / cond)
        k0 = traj.index_at(cfg["gnss_cutoff_s"])
        t = traj.timestamp[k0:] - traj.timestamp[k0]
        g_unit = traj.gravity_world / np.linalg.norm(traj.gravity_world)
        nb_g, nb_a = native_imu_bias(traj)
        truth = (nb_g[k0:], nb_a[k0:])
        per_method = {m: [] for m in MIDAIR_METHODS}
        seed0 = {}
        for sd in SEEDS:
            c = copy.deepcopy(cfg)
            c["barometer"]["seed"] = sd
            vis = prepare_visual_inputs(traj, c, vio_cfg, k0, 100.0, REPO_ROOT / "outputs" / "midair_eskf" / "cache")
            for m in MIDAIR_METHODS:
                if m == "imu_only" and sd > 0:
                    per_method[m].append(per_method[m][0])
                    continue
                o = run_ablation(traj, k0, vis, c, m)
                ev = evaluate(t, o.result.position, o.result.velocity, o.result.attitude, o.accel_bias, o.gyro_bias,
                              traj.position_gt[k0:], traj.velocity_gt[k0:], traj.attitude_gt[k0:], g_unit, truth)
                ev["updates"] = {kind: f"{sum(u.accepted for u in o.updates if u.kind == kind)}/{sum(1 for u in o.updates if u.kind == kind)}"
                                 for kind in ("rotation", "flow", "baro", "direction") if any(u.kind == kind for u in o.updates)}
                per_method[m].append(ev)
                if sd == 0:
                    seed0[m] = ev
        # old barometer, seed 0, both cameras: how much the earlier numbers move
        c = copy.deepcopy(cfg)
        c["barometer"].update(OLD_BARO)
        vis_old = prepare_visual_inputs(traj, c, vio_cfg, k0, 100.0, REPO_ROOT / "outputs" / "midair_eskf" / "cache")
        old = {}
        for m in ("imu_baro", "both"):
            o = run_ablation(traj, k0, vis_old, c, m)
            old[m] = float(np.linalg.norm(o.result.position[-1] - traj.position_gt[-1]))
        summary = {m: median_over_seeds(v) for m, v in per_method.items()}
        summary["_old_barometer_final_position_m"] = old
        results[f"{cond}/{name}"] = summary
        save(OUT_BASE / f"midair_{cond}_{name}", {m: summary[m] for m in ("imu_only", "imu_baro", "forward_rotation", "both")} | {"_old": old},
             {m: seed0[m] for m in ("imu_only", "imu_baro", "forward_rotation", "both")})
        save(OUT_DIR / f"midair_{cond}_{name}", {m: summary[m] for m in ("imu_baro", "both", "forward_rotation", "forward_direction", "both_direction")},
             {m: seed0[m] for m in ("imu_baro", "forward_rotation", "both", "forward_direction", "both_direction")})
        show(f"{cond}/{name}", summary, MIDAIR_METHODS, old)
    return results


# ------------------------------------------------------------------ NTU
def ntu(cfg, vio_cfg):
    results = {}
    for seq, cam in (("eee_03", "left"), ("sbs_01", "left"), ("rtp_01", "right")):
        d = dict(np.load(DIAG / "cache" / "ntu" / f"{seq}_{cam}_messages_v2.npz"))
        sdir = DIAG / "cache" / "ntu" / seq
        cal = sources.ntu_calibration(sdir, cam)
        imu_yaml = sdir / "imu_v100.yaml" if (sdir / "imu_v100.yaml").is_file() else DIAG / "cache" / "ntu" / "eee_03" / "imu_v100.yaml"
        txt = imu_yaml.read_text().splitlines()
        gyro_std = float(next(l.split(":")[1] for l in txt if l.strip().startswith("gyro_std")))
        accel_std = float(next(l.split(":")[1] for l in txt if l.strip().startswith("accel_std")))
        t_all, a, w = d["imu_t"], d["imu_a"], d["imu_w"]
        Rq = Rotation.from_quat(d["imu_q_xyzw"])
        pt, P = d["pos_t"], d["pos"]
        keep = np.r_[True, np.diff(pt) > 1e-6]
        pt, P = pt[keep], P[keep]
        t0 = max(pt[0], t_all[0] + 2.0) + cfg["gnss_cutoff_s"]  # Leica may start before the IMU (rtp_01)
        k0 = int(np.searchsorted(t_all, t0))
        kend = int(np.searchsorted(t_all, pt[-1])) - 1
        tt = t_all[k0:kend]
        dt = float(np.median(np.diff(tt)))
        pre = (t_all >= t0 - 2.0) & (t_all < t0)
        g_w = -np.mean(Rq[pre].apply(a[pre]), axis=0)  # gravity in the VN100 world frame, measured before the cut
        P_ref = np.column_stack([np.interp(tt, pt, P[:, i]) for i in range(3)])
        P_ref -= P_ref[0]
        V_ref = np.gradient(P_ref, tt, axis=0)
        V_ref = np.column_stack([np.convolve(V_ref[:, i], np.ones(20) / 20, mode="same") for i in range(3)])  # 0.1 s smoothing (evaluation)
        meas, rate, age = sources.ntu_visual_measurements(d, cal, vio_cfg, t_all)
        meas = [m for m in meas if k0 <= m.keyframe_imu_index and m.imu_index < kend]
        c = copy.deepcopy(cfg)
        nm = noise_model(c)
        nm.gyro_noise, nm.accel_noise = gyro_std * np.sqrt(dt), accel_std * np.sqrt(dt)  # sensor constants from NTU's calibration file
        init = NavState(np.zeros(3), np.zeros(3), Rq[k0].as_quat()[[3, 0, 1, 2]])  # drone ~stationary at the cut (0.01-0.21 m/s)
        q_ref = Rq[k0:kend].as_quat()[:, [3, 0, 1, 2]]
        g_unit = g_w / np.linalg.norm(g_w)
        bc = c["barometer"]
        fc = c["forward_camera"]
        per_method = {m: [] for m in NTU_METHODS}
        seed0 = {}
        for sd in SEEDS:
            b = generate_barometer(t_all, np.interp(t_all, pt, P[:, 2]), BarometerConfig(**{k: bc[k] for k in ("white_std_m", "bias_walk_m_per_sqrt_s", "drift_sigma_m_per_s", "scale_error_min", "scale_error_max")}, seed=sd))
            dalt = relative_altitude(b, k0)[: kend - k0]
            for m in NTU_METHODS:
                if m == "imu_only" and sd > 0:
                    per_method[m].append(per_method[m][0])
                    continue
                use_baro = m != "imu_only"
                use_rot = m in ("forward_rotation", "forward_direction")
                use_dir = m == "forward_direction"
                inp = EskfInputs(tt, a[k0:kend], w[k0:kend], "body", g_w, init, k0, rotation_measurements=meas if (use_rot or use_dir) else None,
                                 R_bc_forward=cal["R_bc"], baro_altitude_change=dalt if use_baro else None)
                o = run_eskf(inp, nm, RotationUpdateConfig(enabled=use_rot, sigma_deg=fc["sigma_deg"], gate_prob=fc["gate_prob"]),
                             baro_cfg=BaroUpdateConfig(enabled=use_baro, every_n_samples=int(round(0.2 / dt))),
                             dir_cfg=DirectionUpdateConfig(enabled=use_dir, **{k: c["direction"][k] for k in ("sigma_deg", "min_speed_mps", "gate_prob")}))
                # EVALUATION-ONLY alignment: one rotation from the estimated (VN100 world) to the Leica frame
                U, _, Vt = np.linalg.svd(P_ref.T @ o.result.position)
                Ra = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
                ev = evaluate(tt - tt[0], o.result.position @ Ra.T, o.result.velocity @ Ra.T, o.result.attitude, o.accel_bias, o.gyro_bias,
                              P_ref, V_ref, q_ref, g_unit)
                ev["updates"] = {kind: f"{sum(u.accepted for u in o.updates if u.kind == kind)}/{sum(1 for u in o.updates if u.kind == kind)}"
                                 for kind in ("rotation", "baro", "direction") if any(u.kind == kind for u in o.updates)}
                ev["direction_rejections"] = {r: sum(1 for u in o.updates if u.kind == "direction" and not u.accepted and u.reason == r)
                                              for r in {u.reason for u in o.updates if u.kind == "direction" and not u.accepted}}
                per_method[m].append(ev)
                if sd == 0:
                    seed0[m] = ev
        summary = {m: median_over_seeds(v) for m, v in per_method.items()}
        summary["_info"] = {"camera": cam, "visual_spans": len(meas), "valid_spans": sum(m.valid for m in meas), "image_rate_hz": rate,
                            "frames_per_span": age, "duration_s": float(tt[-1] - tt[0]), "gravity_world": g_w.tolist(),
                            "evaluation": "Leica positions aligned by one rotation per method (evaluation only); attitude vs VN100 orientation"}
        results[seq] = summary
        save(OUT_BASE / f"ntu_{seq}", {m: summary[m] for m in ("imu_only", "imu_baro", "forward_rotation")}, {m: seed0[m] for m in ("imu_only", "imu_baro", "forward_rotation")})
        save(OUT_DIR / f"ntu_{seq}", {m: summary[m] for m in ("imu_baro", "forward_rotation", "forward_direction")}, {m: seed0[m] for m in ("imu_baro", "forward_rotation", "forward_direction")})
        show(f"NTU {seq} ({summary['_info']['valid_spans']}/{summary['_info']['visual_spans']} valid spans)", summary, NTU_METHODS)
    return results


def show(title, s, methods, old=None):
    print(f"\n{title}" + (f"  | OLD barometer final position: imu_baro {old['imu_baro']:.1f} m, both {old['both']:.1f} m" if old else ""))
    for m in methods:
        r = s[m]
        p, v, a = r["position_m"], r["velocity"], r["attitude_deg"]
        print(f"  {m:18s} pos 10/30/60/final {p['10s'] or float('nan'):6.1f} {p['30s'] or float('nan'):6.1f} {p['60s'] or float('nan'):6.1f} {p['final']:7.1f} "
              f"(p95 {p['final_p95_over_seeds']:6.1f}) RMSE {p['rmse']:6.1f} | vel dir {v['direction_err_deg_median']:5.1f} deg vec {v['vector_err_m_s_median']:5.2f} "
              f"speed {v['speed_err_m_s_median']:5.2f} m/s (speed share {v['share_of_vector_err_from_speed_median']:.2f}) | att tot/tilt/head {a['total_final']:5.2f}/{a['tilt_final']:5.2f}/{a['heading_final']:5.2f} "
              f"| acc tilt-eq {r['accel_eval_only']['tilt_equivalent_err_deg_median']:4.1f} deg | {r.get('updates', '')}")


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", choices=["midair", "ntu"], help="run one dataset only")
    args = ap.parse_args()
    cfg = yaml.safe_load((VIO_DIR / "configs" / "midair_eskf.yaml").read_text())
    vio_cfg = yaml.safe_load((REPO_ROOT / cfg["vio_config"]).read_text())
    for out in (OUT_BASE, OUT_DIR):
        out.mkdir(parents=True, exist_ok=True)
    if args.only != "ntu":
        print("=== MID-AIR (median over 5 barometer seeds) ===")
        mid = midair(cfg, vio_cfg)
        for out in (OUT_BASE, OUT_DIR):
            (out / "midair_summary.json").write_text(json.dumps(mid, indent=2, default=float))
    if args.only != "midair":
        print("\n=== NTU VIRAL (same parameters; sensor noise from the NTU calibration file) ===")
        nt = ntu(cfg, vio_cfg)
        for out in (OUT_BASE, OUT_DIR):
            (out / "ntu_summary.json").write_text(json.dumps(nt, indent=2, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())

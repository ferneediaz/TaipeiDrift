"""ESKF relative-navigation ablation on Mid-Air.

Five filters from the same GNSS cutoff, same IMU, same evaluation interval:
  imu_only          ESKF prediction only (reproduces the IMU-only baseline)
  imu_baro          + simulated barometer altitude (the platform has one; reference for the rest)
  forward_rotation  + barometer + forward-camera relative rotation (stochastic cloning)
  down_flow         + barometer + downward-camera flow velocity
  both              + barometer + both visual measurements

From the repository root:

    python vio/scripts/run_midair_eskf.py --data-root data/MidAir \
        --flights sunny:0 sunny:1 cloudy:3000 cloudy:3001 [--inject-bias]

Results: outputs/midair_eskf/<flight>[_injected]/ (metrics.json, plots) and
outputs/midair_eskf/summary[_injected].json / .md for all flights together.
"""
from __future__ import annotations

import argparse
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

from src.data.midair import MidAirConfig, MidAirDataNotFound, load_midair_trajectory  # noqa: E402
from vio.eskf_pipeline import ABLATIONS, evaluate, prepare_visual_inputs, run_ablation  # noqa: E402
from vio.sensors.simulated import InjectedBias, inject_imu_bias  # noqa: E402
from vio.visualization.eskf_plots import plot_biases, plot_nis, plot_series, plot_trajectories  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=VIO_DIR / "configs" / "midair_eskf.yaml")
    p.add_argument("--data-root", default="data/MidAir")
    p.add_argument("--environment", default="Kite_training")
    p.add_argument("--flights", nargs="+", default=["sunny:0", "sunny:1", "cloudy:3000", "cloudy:3001"],
                   help="condition:trajectory pairs")
    p.add_argument("--ablations", nargs="+", default=list(ABLATIONS), choices=list(ABLATIONS))
    p.add_argument("--inject-bias", action="store_true", help="add the configured known IMU biases")
    p.add_argument("--gnss-cutoff", type=float)
    p.add_argument("--output-dir", type=Path)
    p.add_argument("--no-cache", action="store_true", help="recompute feature tracks")
    return p.parse_args()


def _clean(x):
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items() if not str(k).startswith("_")}
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return x


def run_flight(spec: str, args, cfg: dict, vio_cfg: dict, out_root: Path) -> dict:
    condition, traj_id = spec.split(":")
    mcfg = MidAirConfig.from_dict({"environment": args.environment, "condition": condition, "trajectory": traj_id})
    traj = load_midair_trajectory(mcfg, data_root=args.data_root)
    injected = None
    if args.inject_bias:
        injected = cfg["bias_injection"]
        traj = inject_imu_bias(traj, InjectedBias(tuple(injected["gyro"]), tuple(injected["accel"])))
    cutoff = args.gnss_cutoff if args.gnss_cutoff is not None else cfg["gnss_cutoff_s"]
    k0 = traj.index_at(cutoff)
    tuning = cfg.get("tuning_trajectory", {})
    is_tuning = condition == tuning.get("condition") and mcfg.trajectory == tuning.get("trajectory")
    name = f"{condition}_{mcfg.trajectory}" + ("_injected" if injected else "")
    out = out_root / name
    out.mkdir(parents=True, exist_ok=True)

    t = time.time()
    need_fwd = any(ABLATIONS[a][0] for a in args.ablations)
    need_down = any(ABLATIONS[a][1] for a in args.ablations)
    vis = prepare_visual_inputs(traj, cfg, vio_cfg, k0, mcfg.imu_rate_hz,
                                None if args.no_cache else out_root / "cache", need_fwd, need_down)
    print(f"{name}: front ends {time.time() - t:.0f} s" + (" (TUNING flight)" if is_tuning else ""))

    results, outputs = {}, {}
    for ab in args.ablations:
        o = run_ablation(traj, k0, vis, cfg, ab)
        outputs[ab] = o
        results[ab] = evaluate(o, traj, cfg, injected)
        r = results[ab]
        print(f"  {ab:16s} final {r['position']['final_m']:7.1f} m  ATE {r['position']['ate_rmse_m']:6.1f}  "
              f"RPE10 {r['rpe']['trans_rmse_m']:6.1f}  att {r['attitude_deg']['final']:5.2f} deg  "
              f"NEES9 {r['nees']['nav9']['mean']:7.1f}  bg err {r['bias'].get('gyro_err_vs_injected', r['bias']['gyro_err_vs_total']) * 1e3:5.2f} mrad/s")

    metrics = {"flight": name, "tuning_flight": is_tuning, "gnss_cutoff_s": float(traj.timestamp[k0]),
               "height_above_ground": vis.height_info, "injected_bias": injected,
               "ablations": {k: _clean(v) for k, v in results.items()}}
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2, default=float))

    ts = results[args.ablations[0]]["_series"]["t"]
    plot_trajectories(traj.position_gt, k0, {k: o.result.position for k, o in outputs.items()}, f"Top view, {name}",
                      out / "trajectory.png")
    plot_series(ts, {k: r["_series"]["pos_err"] for k, r in results.items()}, "Position error (m)",
                f"Position error after GNSS loss, {name}", out / "position_error.png")
    plot_series(ts, {k: r["_series"]["att_err"] for k, r in results.items()}, "Attitude error (deg)",
                f"Attitude error after GNSS loss, {name}", out / "attitude_error.png")
    plot_series(results[args.ablations[0]]["_series"]["nees9_t"],
                {k: r["_series"]["nees9"] for k, r in results.items()}, "NEES (9 states)",
                f"NEES of position, velocity, attitude (expected 9), {name}", out / "nees.png", log=True)
    s0 = results[args.ablations[0]]["_series"]
    plot_biases(ts, {k: (o.gyro_bias, o.accel_bias) for k, o in outputs.items() if k != "imu_only"},
                s0["bias_true_g"], s0["bias_true_a"], f"Bias estimates, {name}", out / "bias.png")
    if "both" in outputs:
        o = outputs["both"]
        upd = {}
        for kind, dof in (("rotation", 3), ("flow", 2)):
            L = [u for u in o.updates if u.kind == kind and u.accepted]
            upd[kind] = (np.array([traj.timestamp[u.imu_index] - traj.timestamp[k0] for u in L]),
                         np.array([u.nis for u in L]), dof)
        plot_nis(upd, f"IMU + both, {name}", out / "nis.png")
    return metrics


def write_summary(all_metrics: list[dict], path_stem: Path) -> None:
    path_stem.with_suffix(".json").write_text(json.dumps(all_metrics, indent=2, default=float))
    lines = ["| Flight | Filter | Final pos (m) | ATE (m) | RPE 10 s (m) | Att final (deg) | NEES9 mean | Gyro bias err (mrad/s) | Accel bias err (m/s²) | Updates accepted |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for m in all_metrics:
        for ab, r in m["ablations"].items():
            b = r["bias"]
            bg = b.get("gyro_err_vs_injected", b["gyro_err_vs_total"]) * 1e3
            ba = b.get("accel_err_vs_injected", b["accel_err_vs_total"])
            upd = ", ".join(f"{k} {v['accepted']}/{v['attempted']}" for k, v in r["updates"].items()) or "-"
            flight = m["flight"] + (" (tuning)" if m["tuning_flight"] else "")
            lines.append(f"| {flight} | {ab} | {r['position']['final_m']:.1f} | {r['position']['ate_rmse_m']:.1f} | "
                         f"{r['rpe']['trans_rmse_m']:.1f} | {r['attitude_deg']['final']:.2f} | {r['nees']['nav9']['mean']:.1f} | "
                         f"{bg:.2f} | {ba:.3f} | {upd} |")
    path_stem.with_suffix(".md").write_text("\n".join(lines) + "\n")


def main() -> int:
    args = parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    vio_cfg = yaml.safe_load((REPO_ROOT / cfg["vio_config"]).read_text())
    out_root = Path(args.output_dir or REPO_ROOT / cfg["output_dir"])
    all_metrics = []
    try:
        for spec in args.flights:
            all_metrics.append(run_flight(spec, args, cfg, vio_cfg, out_root))
    except MidAirDataNotFound as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    stem = out_root / ("summary_injected" if args.inject_bias else "summary")
    write_summary(all_metrics, stem)
    print(f"summary: {stem.with_suffix('.md')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Run the IMU-only dead-reckoning baseline and evaluate its drift after GNSS loss.

From the repository root:

    python baseline/scripts/run_midair_baseline.py --synthetic --gnss-cutoff 5.0
    python baseline/scripts/run_midair_baseline.py --data-root data/raw/midair/MidAir \
        --trajectory trajectory_0000 --gnss-cutoff 5.0

Results go to outputs/midair_baseline/<run_name>/: metrics.json, errors.csv and plots.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

BASELINE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = BASELINE_DIR.parent
sys.path.insert(0, str(BASELINE_DIR))

from src.data.midair import MidAirConfig, MidAirDataNotFound, load_midair_trajectory  # noqa: E402
from src.data.synthetic import SCENARIOS, ImuNoise, make_synthetic_trajectory  # noqa: E402
from src.data.trajectory import Trajectory, imu_consistency  # noqa: E402
from src.estimation.inertial_dead_reckoning import run_dead_reckoning  # noqa: E402
from src.evaluation.trajectory_metrics import (  # noqa: E402
    error_at_horizons,
    error_series,
    summarize,
    time_to_exceed,
)
from src.visualization.trajectory_plot import (  # noqa: E402
    plot_error,
    plot_trajectory_2d,
    plot_trajectory_3d,
)

# Residuals above these suggest a wrong frame, gravity sign or quaternion order, which give
# errors of order g. They sit well above Mid-Air's measured IMU noise (gyro up to ~0.07 rad/s).
ACCEL_RESIDUAL_WARN = 2.0  # m/s^2
GYRO_RESIDUAL_WARN = 0.5  # rad/s


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=BASELINE_DIR / "configs" / "midair_baseline.yaml")
    p.add_argument("--synthetic", action="store_true", help="use a synthetic flight instead of Mid-Air")
    p.add_argument("--scenario", choices=sorted(SCENARIOS), help="synthetic scenario")
    p.add_argument("--no-noise", action="store_true", help="synthetic: ideal IMU, no noise or bias")
    p.add_argument("--data-root", help="Mid-Air root folder (overrides MID_AIR_ROOT and the config)")
    p.add_argument("--environment", help="Mid-Air environment, e.g. Kite_training")
    p.add_argument("--condition", help="Mid-Air condition, e.g. sunny")
    p.add_argument("--trajectory", help="Mid-Air flight, e.g. trajectory_0000 or 0")
    p.add_argument("--gnss-cutoff", type=float, help="time of GNSS loss in seconds")
    p.add_argument("--run-name", help="output subfolder name")
    p.add_argument("--output-dir", type=Path, help="parent folder for run outputs")
    return p.parse_args()


def load_trajectory(args: argparse.Namespace, cfg: dict) -> Trajectory:
    if args.synthetic:
        s = cfg["synthetic"]
        n = dict(s.get("noise", {}))
        enabled = n.pop("enabled", False) and not args.no_noise
        noise = ImuNoise(**{k: tuple(v) if isinstance(v, list) else v for k, v in n.items()}) if enabled else None
        return make_synthetic_trajectory(
            scenario=args.scenario or s["scenario"],
            duration=s["duration_s"],
            rate_hz=s["rate_hz"],
            noise=noise,
        )
    m = dict(cfg["midair"])
    for key in ("environment", "condition", "trajectory"):
        if getattr(args, key):
            m[key] = getattr(args, key)
    mcfg = MidAirConfig.from_dict(m)
    if mcfg.data_root and not Path(mcfg.data_root).is_absolute():
        mcfg.data_root = str(REPO_ROOT / mcfg.data_root)
    return load_midair_trajectory(mcfg, data_root=args.data_root)


def main() -> int:
    args = parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    cutoff = args.gnss_cutoff if args.gnss_cutoff is not None else cfg["gnss_cutoff_s"]

    try:
        traj = load_trajectory(args, cfg)
    except MidAirDataNotFound as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    if args.synthetic and args.no_noise:
        traj.name += "_ideal"
    run_name = args.run_name or f"{traj.name}_cutoff{cutoff:g}s"
    out_dir = args.output_dir or REPO_ROOT / cfg["output_dir"]
    out = Path(out_dir) / run_name
    out.mkdir(parents=True, exist_ok=True)

    print(f"flight {traj.name}: {len(traj)} samples, {traj.duration:.1f} s, frame {traj.world_frame}")
    check = imu_consistency(traj)
    print(f"IMU vs ground truth, median residual: accelerometer {check['accel_residual_median']:.3f} m/s^2, "
          f"gyroscope {check['gyro_residual_median']:.4f} rad/s")
    if check["accel_residual_median"] > ACCEL_RESIDUAL_WARN or check["gyro_residual_median"] > GYRO_RESIDUAL_WARN:
        print("WARNING: the IMU does not match the ground truth. Check quaternion order, world frame "
              "and gravity sign in the config before trusting the results.")

    result = run_dead_reckoning(traj, cutoff)
    series = error_series(result, traj)
    summary = summarize(series)
    horizons = error_at_horizons(series, cfg["error_horizons_s"])
    exceed = time_to_exceed(series, cfg["error_thresholds_m"])

    metrics = {
        "run_name": run_name,
        "trajectory": traj.name,
        "metadata": traj.metadata,
        "gnss_cutoff_requested_s": cutoff,
        "gnss_cutoff_actual_s": result.t0,
        "imu_consistency": check,
        "position_error_m": summary.as_dict(),
        "error_at_time_since_loss_m": {f"{h:g}s": v for h, v in horizons.items()},
        "time_to_exceed_s": {f"{th:g}m": v for th, v in exceed.items()},
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
    np.savetxt(
        out / "errors.csv",
        np.column_stack([series.time_since_loss, series.error, series.horizontal, series.vertical]),
        delimiter=",", header="time_since_loss_s,error_m,horizontal_m,vertical_m", comments="", fmt="%.6f",
    )
    plot_trajectory_2d(result, traj, out / "trajectory_2d.png")
    plot_error(series, out / "error_vs_time.png", title=f"Position error after GNSS loss, {traj.name}")
    plot_trajectory_3d(result, traj, out / "trajectory_3d.png")

    print(f"GNSS lost at {result.t0:.2f} s, dead reckoning for {summary.duration_s:.1f} s")
    print(f"position error: final {summary.final:.2f} m, RMSE {summary.rmse:.2f} m, "
          f"mean {summary.mean:.2f} m, max {summary.max:.2f} m")
    print("error after loss: " + ", ".join(
        f"{h:g}s {v:.2f} m" for h, v in horizons.items() if v is not None))
    print("first exceeds:    " + ", ".join(
        f"{th:g} m at {v:.2f} s" if v is not None else f"{th:g} m never" for th, v in exceed.items()))
    print(f"results in {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Soft-threshold (shrinkage) version of the visual gyro-bias filter.

The Schmidt bias filter is unchanged; only the bias APPLIED to the gyro is shrunk per axis:
b_applied = sign(b_hat) * max(|b_hat| - f, 0). Floors f = 0 (unshrunk), 0.05, 0.10, 0.15 deg/s.

Selection rule (fixed before running): tune on sunny 0001 (low drift) and cloudy 3000 (high
drift). Pick the SMALLEST floor whose sunny-0001 degradation (final attitude minus IMU-only) is
at most 1.0 deg; if none qualifies, the floor with the least degradation. Cloudy 3000 reports
how much gain each floor retains. Position is reported only. The chosen floor is then run
unchanged on the holdouts sunny 0000 and cloudy 3001.

    python vio/scripts/run_bias_shrinkage.py
"""
from __future__ import annotations

import csv
import json
import sys
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

from run_bias_experiment import kf_config, load_flight, visual_measurements  # noqa: E402
from src.estimation.inertial_dead_reckoning import run_dead_reckoning  # noqa: E402
from src.evaluation.trajectory_metrics import error_series  # noqa: E402
from vio.bias_experiment import DEG, growth_rates, intervals_from_measurements, run_on_trajectory  # noqa: E402
from vio.evaluation.attitude_metrics import attitude_error_series  # noqa: E402

FLOORS = [0.0, 0.05, 0.10, 0.15]
TUNING = [("sunny", "trajectory_0001"), ("cloudy", "trajectory_3000")]
HOLDOUT = [("sunny", "trajectory_0000"), ("cloudy", "trajectory_3001")]
MAX_LOW_DRIFT_DEGRADATION_DEG = 1.0
OUT = REPO_ROOT / "outputs" / "midair_bias_shrinkage"


def at(t, e, h):
    return float(np.interp(h, t, e)) if h <= t[-1] else None


def run_flight(cond, name, floors, cfg, vio_cfg):
    traj = load_flight(cond, name)
    k0 = traj.index_at(5.0)
    meas, _ = visual_measurements(traj, vio_cfg, k0)
    ivs = intervals_from_measurements(meas, int(round(cfg["interval_s"] * 25)))
    imu = run_dead_reckoning(traj, 5.0)
    ai, pi = attitude_error_series(imu, traj), error_series(imu, traj)
    raw_final = None
    rows = []
    for f in floors:
        kc = kf_config(cfg)
        kc.apply_floor_deg_s = f
        out = run_on_trajectory(traj, 5.0, ivs, kc)
        a, p = attitude_error_series(out.result, traj), error_series(out.result, traj)
        if f == 0.0:
            raw_final = float(a.error_deg[-1])
        t = a.time_since_loss
        rows.append({
            "flight": f"{cond}/{name}", "floor_deg_s": f,
            "imu_final_attitude_deg": float(ai.error_deg[-1]), "raw_bias_final_attitude_deg": raw_final,
            "shrinkage_final_attitude_deg": float(a.error_deg[-1]),
            "attitude_10s_deg": at(t, a.error_deg, 10), "attitude_30s_deg": at(t, a.error_deg, 30), "attitude_60s_deg": at(t, a.error_deg, 60),
            **{f"growth_{k}_deg_s": v for k, v in growth_rates(t, a.error_deg).items()},
            **{f"imu_growth_{k}_deg_s": v for k, v in growth_rates(ai.time_since_loss, ai.error_deg).items()},
            "imu_final_position_m": float(pi.error[-1]), "shrinkage_final_position_m": float(p.error[-1]),
            **{f"raw_b_hat_{c}_deg_s": float(v) for c, v in zip("xyz", out.bias[-1] / DEG)},
            **{f"b_applied_{c}_deg_s": float(v) for c, v in zip("xyz", out.bias_applied[-1] / DEG)},
        })
    return rows


def show(rows):
    for r in rows:
        print(f"  {r['flight']:22s} f={r['floor_deg_s']:.2f} | att final {r['shrinkage_final_attitude_deg']:5.2f} (IMU {r['imu_final_attitude_deg']:5.2f}, raw {r['raw_bias_final_attitude_deg']:5.2f}) "
              f"10/30/60 {r['attitude_10s_deg']:.2f}/{r['attitude_30s_deg']:.2f}/{r['attitude_60s_deg']:.2f} | slope 5-30/30-60/60-end "
              f"{r['growth_5-30s_deg_s']:+.3f}/{r['growth_30-60s_deg_s']:+.3f}/{r['growth_60-ends_deg_s']:+.3f} | pos {r['shrinkage_final_position_m']:6.1f} (IMU {r['imu_final_position_m']:6.1f}) | "
              f"b_hat {[round(r[f'raw_b_hat_{c}_deg_s'], 3) for c in 'xyz']} applied {[round(r[f'b_applied_{c}_deg_s'], 3) for c in 'xyz']}")


def main() -> int:
    cfg = yaml.safe_load((VIO_DIR / "configs" / "midair_bias_kf.yaml").read_text())
    vio_cfg = yaml.safe_load((VIO_DIR / "configs" / "midair_vio.yaml").read_text())
    OUT.mkdir(parents=True, exist_ok=True)
    print("TUNING (sunny 0001 low drift, cloudy 3000 high drift)")
    tune = [r for c, n in TUNING for r in run_flight(c, n, FLOORS, cfg, vio_cfg)]
    show(tune)
    low = {r["floor_deg_s"]: r["shrinkage_final_attitude_deg"] - r["imu_final_attitude_deg"] for r in tune if r["flight"].endswith("0001")}
    ok = [f for f in FLOORS if low[f] <= MAX_LOW_DRIFT_DEGRADATION_DEG]
    chosen = min(ok) if ok else min(FLOORS, key=lambda f: low[f])
    print(f"chosen floor: {chosen} deg/s (rule: smallest floor with sunny-0001 degradation <= {MAX_LOW_DRIFT_DEGRADATION_DEG} deg; degradations {low})")
    print("HOLDOUT (frozen floor; f=0 shown for reference)")
    hold = [r for c, n in HOLDOUT for r in run_flight(c, n, [0.0, chosen] if chosen else [0.0], cfg, vio_cfg)]
    show(hold)
    rows = tune + hold
    with open(OUT / "summary.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    (OUT / "summary.json").write_text(json.dumps({"chosen_floor_deg_s": chosen, "rule": f"smallest floor with sunny-0001 degradation <= {MAX_LOW_DRIFT_DEGRADATION_DEG} deg",
                                                   "low_drift_degradation_deg": low, "rows": rows}, indent=2))
    # trade-off: low-drift degradation vs high-drift improvement
    hi = {r["floor_deg_s"]: r["imu_final_attitude_deg"] - r["shrinkage_final_attitude_deg"] for r in tune if r["flight"].endswith("3000")}
    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.plot([low[f] for f in FLOORS], [hi[f] for f in FLOORS], "-o", color="#2a78d6")
    for f in FLOORS:
        ax.annotate(f"f={f:.2f}", (low[f], hi[f]), textcoords="offset points", xytext=(6, 4))
    ax.axvline(0, color="#52514e", lw=1, ls="--")
    ax.set_xlabel("sunny 0001: final attitude minus IMU-only (deg)  [lower = less harm]")
    ax.set_ylabel("cloudy 3000: IMU-only minus final attitude (deg)  [higher = more gain]")
    ax.set_title("Shrinkage floor trade-off (tuning flights)")
    ax.grid(color="#e4e3df")
    fig.tight_layout()
    fig.savefig(OUT / "tradeoff.png", dpi=120)
    print(f"results in {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

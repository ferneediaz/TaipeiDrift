"""Validate the measured barometer model and its effect in the ESKF.

    python vio/scripts/run_barometer_validation.py

1. Monte Carlo (2000 flights, 10 Hz, 20 min): relative-altitude error |e(t0 + tau) - e(t0)| at
   10 s, 60 s, 2, 5, 10, 20 min, against the measured Zurich table
   (origin/research/offline-nav-evidence, docs/research/context-alessandro-en.md section 3.2).
   No altitude change, as in the branch's own generator check (the measured fit already contains
   the scale effect of that flight); a second run adds a +/-70 m vertical excursion (Mid-Air's
   median per-flight altitude range) to show what the 3-7 % scale error adds.
2. ESKF on the four Mid-Air diagnostic flights, IMU + barometer, 20 seeds each: vertical error
   with the previous arbitrary barometer (0.3 m, 0.05 m/sqrt(s)) and with the measured model.
All barometers are SIMULATED, calibrated on real logs.
"""
from __future__ import annotations

import csv
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

from vio.sensors.simulated import BarometerConfig, generate_barometer  # noqa: E402

OUT = REPO_ROOT / "outputs" / "barometer_real_model"
HORIZONS = [10, 60, 120, 300, 600, 1200]
MEASURED = {10: (0.41, 1.42), 60: (0.58, 2.12), 120: (0.82, 2.97), 300: (1.75, 4.04), 600: (2.49, 6.11), 1200: (4.50, 8.83)}


def monte_carlo(n=2000, excursion_m=0.0, rate=10.0, duration=1200.0):
    t = np.arange(0.0, duration + 1e-9, 1.0 / rate)
    idx = [int(round(h * rate)) for h in HORIZONS]
    rng = np.random.default_rng(12345)
    errs = np.empty((n, len(HORIZONS)))
    for i in range(n):
        alt = excursion_m * np.sin(2 * np.pi * t / rng.uniform(60, 300) + rng.uniform(0, 2 * np.pi)) if excursion_m else np.zeros_like(t)
        b = generate_barometer(t, alt, BarometerConfig(seed=i))
        rel_err = (b.altitude_m - b.altitude_m[0]) - (alt - alt[0])  # relative altitude after a cut at t0
        errs[i] = np.abs(rel_err[idx])
    return errs


def eskf_check(seeds=20):
    from run_bias_experiment import load_flight
    from src.estimation.inertial_dead_reckoning import NavState
    from vio.estimation.eskf_runner import BaroUpdateConfig, EskfInputs, run_eskf
    from vio.sensors.simulated import relative_altitude, simulate_barometer
    rows = []
    models = {"previous (0.30 m, 0.05 m/sqrt s)": (BarometerConfig(bias_walk_m_per_sqrt_s=0.05, drift_sigma_m_per_s=0.0, scale_error_min=0.0, scale_error_max=0.0),
                                                   BaroUpdateConfig(bias_walk_m_per_sqrt_s=0.05, drift_sigma_m_per_s=0.0, scale_error_rms=0.0)),
              "measured": (BarometerConfig(), BaroUpdateConfig())}
    for cond, name in (("sunny", "trajectory_0000"), ("sunny", "trajectory_0001"), ("cloudy", "trajectory_3000"), ("cloudy", "trajectory_3001")):
        traj = load_flight(cond, name)
        k0 = traj.index_at(5.0)
        base = dict(timestamp=traj.timestamp[k0:], accelerometer=traj.accelerometer[k0:], gyroscope=traj.gyroscope[k0:],
                    gyroscope_frame=traj.gyroscope_frame, gravity_world=traj.gravity_world,
                    initial=NavState(traj.position_gt[k0].copy(), traj.velocity_gt[k0].copy(), traj.attitude_gt[k0].copy()), start_index=k0)
        imu = run_eskf(EskfInputs(**base))
        vz_imu = abs(imu.result.position[-1, 2] - traj.position_gt[-1, 2])
        for label, (bcfg, ucfg) in models.items():
            finals, at60 = [], []
            for sd in range(seeds):
                b = simulate_barometer(traj, BarometerConfig(**{**bcfg.__dict__, "seed": sd}))
                out = run_eskf(EskfInputs(**base, baro_altitude_change=relative_altitude(b, k0)), baro_cfg=ucfg)
                ez = np.abs(out.result.position[:, 2] - traj.position_gt[k0:, 2])
                finals.append(ez[-1])
                at60.append(ez[int(60 * 100)])
            rows.append({"flight": f"{cond}/{name}", "model": label, "imu_only_final_vertical_m": float(vz_imu),
                         "final_vertical_median_m": float(np.median(finals)), "final_vertical_p95_m": float(np.percentile(finals, 95)),
                         "vertical_60s_median_m": float(np.median(at60)), "vertical_60s_p95_m": float(np.percentile(at60, 95))})
            print(f"  {cond}/{name} {label:34s}: vertical error at 60 s median {np.median(at60):.2f} p95 {np.percentile(at60, 95):.2f} | "
                  f"final median {np.median(finals):.2f} p95 {np.percentile(finals, 95):.2f} m (IMU only final {vz_imu:.1f} m)", flush=True)
    return rows


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    flat = monte_carlo()
    exc = monte_carlo(excursion_m=70.0)
    val = {"model": BarometerConfig().__dict__, "source": "origin/research/offline-nav-evidence docs/research/context-alessandro-en.md 3.2",
           "horizons_s": HORIZONS, "measured": {str(h): {"median": m, "p95": p} for h, (m, p) in MEASURED.items()}, "simulated_flat": {}, "simulated_70m_excursion": {}}
    with open(OUT / "monte_carlo.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["horizon_s", "measured_median_m", "measured_p95_m", "sim_median_m", "sim_p95_m", "sim_ratio_median", "sim_70m_median_m", "sim_70m_p95_m"])
        print("horizon | measured median / p95 | simulated (no altitude change) median / p95 | with +/-70 m excursion")
        for k, h in enumerate(HORIZONS):
            m, p = MEASURED[h]
            sm, sp = np.median(flat[:, k]), np.percentile(flat[:, k], 95)
            em, ep = np.median(exc[:, k]), np.percentile(exc[:, k], 95)
            val["simulated_flat"][str(h)] = {"median": float(sm), "p95": float(sp)}
            val["simulated_70m_excursion"][str(h)] = {"median": float(em), "p95": float(ep)}
            w.writerow([h, m, p, f"{sm:.3f}", f"{sp:.3f}", f"{sm / m:.2f}", f"{em:.3f}", f"{ep:.3f}"])
            print(f"  {h:5d} s | {m:5.2f} / {p:5.2f} | {sm:5.2f} / {sp:5.2f} (x{sm / m:.2f} / x{sp / p:.2f}) | {em:5.2f} / {ep:5.2f}")
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    h = np.array(HORIZONS) / 60
    ax.plot(h, [MEASURED[x][0] for x in HORIZONS], "o-", color="#0b0b0b", label="measured median (Zurich)")
    ax.plot(h, [MEASURED[x][1] for x in HORIZONS], "o--", color="#0b0b0b", label="measured p95")
    ax.plot(h, np.median(flat, 0), "s-", color="#2a78d6", label="simulated median")
    ax.plot(h, np.percentile(flat, 95, 0), "s--", color="#2a78d6", label="simulated p95")
    ax.plot(h, np.median(exc, 0), "^-", color="#eb6834", label="simulated median, +/-70 m excursion")
    ax.set_xscale("log"), ax.set_xlabel("Time after GNSS cut (min)"), ax.set_ylabel("Relative-altitude error (m)")
    ax.set_title("SIMULATED barometer, calibrated on real logs"), ax.grid(color="#e4e3df"), ax.legend(frameon=False, fontsize=8)
    fig.tight_layout(), fig.savefig(OUT / "relative_altitude_error.png", dpi=120), plt.close(fig)
    print("ESKF, IMU + barometer, 20 seeds per flight:")
    val["eskf"] = eskf_check()
    (OUT / "model_validation.json").write_text(json.dumps(val, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())

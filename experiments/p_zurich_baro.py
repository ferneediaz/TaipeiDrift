"""How well does a real on-board barometer hold relative altitude after GNSS is lost?

Data: Zurich Urban MAV sample logs (real Fotokite flight, Pixhawk barometer, 45 min).
Evaluation reference: Pix4D photogrammetric camera positions, used only to score.
The estimator sees only the barometer after the cut; GNSS altitude is kept as a comparison.

Run from the repository root after unzipping AGZ_subset.zip into data/raw/zurich_mav:
    python experiments/p_zurich_baro.py
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HORIZONS_S = (10, 30, 60, 120, 300, 600, 1200)


def load(log_dir: Path):
    baro = pd.read_csv(log_dir / "BarometricPressure.csv", skipinitialspace=True).iloc[:, :4]
    baro.columns = ["t_us", "pressure_hpa", "altitude_m", "temperature_c"]
    gps = pd.read_csv(log_dir / "OnboardGPS.csv", skipinitialspace=True).iloc[:, :5]
    gps.columns = ["t_us", "imgid", "lat_e7", "lon_e7", "alt_m"]
    truth = pd.read_csv(log_dir / "GroundTruthAGL.csv", skipinitialspace=True).iloc[:, :10]
    truth.columns = ["imgid", "x_gt", "y_gt", "z_gt", "omega", "phi", "kappa", "x_gps", "y_gps", "z_gps"]
    image_time = gps.groupby("imgid").t_us.first()
    truth = truth.join(image_time, on="imgid").dropna(subset=["t_us"]).sort_values("t_us")
    for frame in (baro, gps, truth):
        frame["t_s"] = frame.t_us / 1e6
    return baro.sort_values("t_s"), gps.sort_values("t_s"), truth


def pressure_altitude(pressure_hpa: np.ndarray, sea_level_hpa: float = 1013.25) -> np.ndarray:
    return 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** (1 / 5.255))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs", type=Path, default=Path("data/raw/zurich_mav/AGZ_subset/Log Files"))
    parser.add_argument("--output", type=Path, default=Path("data/processed/zurich_baro"))
    parser.add_argument("--start-step", type=float, default=30.0, help="Seconds between simulated GNSS cuts")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    baro, gps, truth = load(args.logs)

    # Provenance check: the logged altitude must be a function of the logged pressure alone.
    recomputed = pressure_altitude(baro.pressure_hpa.to_numpy())
    provenance_residual = baro.altitude_m.to_numpy() - recomputed
    provenance = dict(
        residual_median_m=float(np.median(provenance_residual)),
        residual_p99_abs_dev_m=float(np.quantile(np.abs(provenance_residual - np.median(provenance_residual)), .99)),
    )

    t = truth.t_s.to_numpy()
    z_truth = truth.z_gt.to_numpy()
    z_baro = np.interp(t, baro.t_s, baro.altitude_m)
    temp = np.interp(t, baro.t_s, baro.temperature_c)
    z_gnss = np.interp(t, gps.t_s, gps.alt_m)

    rows = []
    for cut in np.arange(t[0] + 60, t[-1], args.start_step):
        i0 = int(np.searchsorted(t, cut))
        if i0 >= len(t):
            break
        # Relative altitude since the cut: the estimator's offset cancels; truth only scores.
        for horizon in HORIZONS_S:
            i1 = int(np.searchsorted(t, t[i0] + horizon))
            if i1 >= len(t) or abs(t[i1] - t[i0] - horizon) > 2:
                continue
            truth_change = z_truth[i1] - z_truth[i0]
            rows.append(dict(cut_s=t[i0], horizon_s=horizon, truth_change_m=truth_change,
                             baro_error_m=(z_baro[i1] - z_baro[i0]) - truth_change,
                             gnss_error_m=(z_gnss[i1] - z_gnss[i0]) - truth_change,
                             temperature_change_c=temp[i1] - temp[i0]))
    errors = pd.DataFrame(rows)
    errors.to_csv(args.output / "relative_altitude_errors.csv", index=False)

    def stats(values: pd.Series) -> dict:
        a = np.abs(values.to_numpy())
        return dict(n=len(a), median_abs_m=float(np.median(a)), p95_abs_m=float(np.quantile(a, .95)), max_abs_m=float(a.max()))

    by_horizon = []
    for horizon, group in errors.groupby("horizon_s"):
        by_horizon.append(dict(horizon_s=int(horizon), baro=stats(group.baro_error_m), gnss=stats(group.gnss_error_m),
                               baro_error_temp_corr=float(np.corrcoef(group.baro_error_m, group.temperature_change_c)[0, 1])
                               if group.temperature_change_c.std() > 0 else None))

    # Whole-flight view with one offset fixed in the first minute (GNSS-initialised estimator).
    first = t < t[0] + 60
    baro_offset = float(np.median(z_gnss[first] - z_baro[first]))
    datum_gap = float(np.median(z_gnss - z_truth))
    report = dict(
        protocol=("Real barometer, cut every %.0f s; error = baro altitude change minus photogrammetric altitude change. "
                  "Windows overlap and come from one flight, so they are not independent samples." % args.start_step),
        files_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(args.logs.glob("*.csv"))},
        duration_s=float(t[-1] - t[0]), truth_samples=len(t), baro_samples=len(baro),
        truth_altitude_range_m=[float(z_truth.min()), float(z_truth.max())],
        baro_temperature_range_c=[float(baro.temperature_c.min()), float(baro.temperature_c.max())],
        pressure_altitude_provenance=provenance,
        gnss_minus_photogrammetry_median_m=datum_gap,
        results=by_horizon,
    )
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
    minutes = (t - t[0]) / 60
    ax1.plot(minutes, z_truth - z_truth[0], label="photogrammetry (reference)", lw=1.5)
    # Barometer mapped to the GNSS frame in the first minute, then to the reference frame for display only.
    ax1.plot(minutes, z_baro + baro_offset - datum_gap - z_truth[0],
             label="barometer, offset from first GNSS minute", lw=.8)
    ax1.plot(minutes, z_gnss - z_truth[0] - datum_gap, label="GNSS altitude", lw=.6, alpha=.6)
    ax1.set_ylabel("altitude change [m]"); ax1.set_xlabel("minutes"); ax1.legend(); ax1.grid(alpha=.3)
    horizons = [r["horizon_s"] for r in by_horizon]
    ax2.plot(horizons, [r["baro"]["median_abs_m"] for r in by_horizon], "o-", label="barometer median")
    ax2.plot(horizons, [r["baro"]["p95_abs_m"] for r in by_horizon], "o--", label="barometer p95")
    ax2.plot(horizons, [r["gnss"]["median_abs_m"] for r in by_horizon], "s-", label="GNSS median")
    ax2.set_xscale("log"); ax2.set_xlabel("seconds since cut"); ax2.set_ylabel("|relative altitude error| [m]")
    ax2.legend(); ax2.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(args.output / "baro_vs_photogrammetry.png", dpi=130)
    print(json.dumps({k: report[k] for k in ("duration_s", "truth_altitude_range_m", "pressure_altitude_provenance",
                                            "gnss_minus_photogrammetry_median_m")}, indent=2))
    print(pd.DataFrame([dict(horizon_s=r["horizon_s"], n=r["baro"]["n"], baro_med=r["baro"]["median_abs_m"],
                             baro_p95=r["baro"]["p95_abs_m"], gnss_med=r["gnss"]["median_abs_m"],
                             gnss_p95=r["gnss"]["p95_abs_m"], temp_corr=r["baro_error_temp_corr"])
                        for r in by_horizon]).round(2).to_string(index=False))


if __name__ == "__main__":
    main()

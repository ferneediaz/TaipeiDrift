"""Measure PX4 barometer drift against fixed RTK height in INSANE sensor archives.

Run from the repository root with ``.venv/bin/python experiments/s_insane_baro.py``.
Archives stay compressed; outputs are written to data/processed/insane_baro/.
"""
from __future__ import annotations

import json
import re
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import nnls

ROOT = Path(__file__).resolve().parents[1]
INPUT_DIR = ROOT / "data" / "raw" / "insane"
OUTPUT_DIR = ROOT / "data" / "processed" / "insane_baro"
HORIZONS_S = (1, 3, 10, 30, 60, 120, 300, 600)
PAIR_TOLERANCE_S = 0.5
BARO_REFERENCE_PRESSURE_PA = 101_325.0
ZURICH_MODEL = {"white_m": 0.30, "random_walk_m_sqrt_s": 0.112, "ramp_m_s": 0.0024}


def _member(names: list[str], suffix: str) -> str:
    matches = [name for name in names if name.endswith("/" + suffix)]
    if len(matches) != 1:
        raise ValueError(f"Expected one archive member ending in {suffix!r}; found {matches}")
    return matches[0]


def _read_csv(archive: zipfile.ZipFile, member: str) -> pd.DataFrame:
    with archive.open(member) as source:
        frame = pd.read_csv(source, skipinitialspace=True)
    frame.columns = [str(column).strip() for column in frame.columns]
    return frame


def _time_offset(archive: zipfile.ZipFile, names: list[str]) -> float:
    member = _member(names, "time_info.yaml")
    with archive.open(member) as source:
        contents = source.read().decode("utf-8", errors="replace")
    match = re.search(r"(?m)^\s*t_mag_gps:\s*([-+0-9.eE]+)", contents)
    if match is None:
        raise ValueError(f"No t_mag_gps clock offset in {member}")
    return float(match.group(1))


def _clean_stream(frame: pd.DataFrame, time_col: str) -> pd.DataFrame:
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(subset=[time_col])
    return frame.sort_values(time_col).drop_duplicates(time_col, keep="first").reset_index(drop=True)


def _structure_function(time_s: np.ndarray, error_m: np.ndarray) -> list[dict[str, float | int | None]]:
    rows: list[dict[str, float | int | None]] = []
    for horizon_s in HORIZONS_S:
        increments: list[np.ndarray] = []
        for i, start_s in enumerate(time_s):
            lo = max(i + 1, int(np.searchsorted(time_s, start_s + horizon_s - PAIR_TOLERANCE_S, side="left")))
            hi = int(np.searchsorted(time_s, start_s + horizon_s + PAIR_TOLERANCE_S, side="right"))
            if hi > lo:
                increments.append(np.abs(error_m[lo:hi] - error_m[i]))
        if increments:
            values = np.concatenate(increments)
            rows.append({
                "horizon_s": horizon_s,
                "median_abs_m": float(np.median(values)),
                "p95_abs_m": float(np.percentile(values, 95)),
                "rms_m": float(np.sqrt(np.mean(values * values))),
                "n_pairs": int(values.size),
            })
        else:
            rows.append({
                "horizon_s": horizon_s,
                "median_abs_m": None,
                "p95_abs_m": None,
                "rms_m": None,
                "n_pairs": 0,
            })
    return rows


def _fit_error_model(rows: list[dict[str, float | int | None]]) -> dict[str, float | int | list[int] | None]:
    usable = [row for row in rows if row["rms_m"] is not None and row["rms_m"] > 0 and row["n_pairs"]]
    if len(usable) < 3:
        return {"sw_m": None, "sqrt_q_m_sqrt_s": None, "d_m_s": None, "fit_horizons_s": [int(r["horizon_s"]) for r in usable]}
    tau = np.asarray([row["horizon_s"] for row in usable], dtype=float)
    rms = np.asarray([row["rms_m"] for row in usable], dtype=float)
    response = rms * rms
    design = np.column_stack((np.ones_like(tau), tau, tau * tau))
    # Weighted least squares with weight 1/RMS^2: multiply residuals by 1/RMS.
    coefficients, _ = nnls(design / rms[:, None], response / rms)
    return {
        "sw_m": float(np.sqrt(coefficients[0] / 2.0)),
        "sqrt_q_m_sqrt_s": float(np.sqrt(coefficients[1])),
        "d_m_s": float(np.sqrt(coefficients[2])),
        "fit_horizons_s": [int(row["horizon_s"]) for row in usable],
    }


def _analyze_archive(archive_path: Path) -> tuple[dict, list[dict]]:
    with zipfile.ZipFile(archive_path) as archive:
        names = archive.namelist()
        baro_member = _member(names, "px4_baro.csv")
        rtk_member = _member(names, "rtk_gps1.csv")
        baro = _clean_stream(_read_csv(archive, baro_member), "t")
        rtk = _clean_stream(_read_csv(archive, rtk_member), "t")
        clock_offset_s = _time_offset(archive, names)
        license_member = next((name for name in names if name.endswith("/LICENSE") or name.endswith("/LICENSE.txt")), None)
        license_text = ""
        if license_member is not None:
            with archive.open(license_member) as source:
                license_text = source.read().decode("utf-8", errors="replace")

    required_baro = {"t", "p"}
    required_rtk = {"t", "p_z", "v_x", "v_y", "rtk_status"}
    if not required_baro.issubset(baro.columns) or not required_rtk.issubset(rtk.columns):
        raise ValueError(f"Unexpected columns: baro={list(baro.columns)}, RTK={list(rtk.columns)}")

    pressure_pa = baro["p"].to_numpy(dtype=float)
    baro_time = baro["t"].to_numpy(dtype=float)
    pressure_ok = np.isfinite(pressure_pa) & (pressure_pa > 0)
    baro_time, pressure_pa = baro_time[pressure_ok], pressure_pa[pressure_ok]
    baro_alt_m = 44_330.0 * (1.0 - (pressure_pa / BARO_REFERENCE_PRESSURE_PA) ** (1.0 / 5.255))

    rtk_time_raw = rtk["t"].to_numpy(dtype=float)
    rtk_time = rtk_time_raw - clock_offset_s
    rtk_up = rtk["p_z"].to_numpy(dtype=float)
    rtk_status = rtk["rtk_status"].to_numpy(dtype=float)
    rtk_fixed = (rtk_status == 1.0) & np.isfinite(rtk_up) & np.isfinite(rtk_time)
    fixed_share = float(np.count_nonzero(rtk_status == 1.0) / len(rtk)) if len(rtk) else None

    # Retain fixed RTK samples only, within the measured barometer interval.
    in_overlap = rtk_fixed & (rtk_time >= baro_time[0]) & (rtk_time <= baro_time[-1])
    time = rtk_time[in_overlap]
    up = rtk_up[in_overlap]
    if time.size < 4:
        raise ValueError(f"{archive_path.name}: only {time.size} fixed RTK samples overlap barometer data")
    interpolated_baro_alt = np.interp(time, baro_time, baro_alt_m)
    raw_error = interpolated_baro_alt - up
    error = raw_error - raw_error[0]
    elapsed = time - time[0]

    speed_xy = np.hypot(
        rtk.loc[in_overlap, "v_x"].to_numpy(dtype=float),
        rtk.loc[in_overlap, "v_y"].to_numpy(dtype=float),
    )
    finite_speed = speed_xy[np.isfinite(speed_xy)]
    structure = _structure_function(elapsed, error)
    fit = _fit_error_model(structure)

    altitude_centered = up - float(np.mean(up))
    regression_design = np.column_stack((altitude_centered, np.ones_like(time), elapsed))
    regression_coefficients, _, _, _ = np.linalg.lstsq(regression_design, raw_error, rcond=None)
    duration_s = float(elapsed[-1])
    altitude_min = float(np.min(up))
    altitude_max = float(np.max(up))
    sequence = archive_path.name.removesuffix("_sensors.zip")
    license_lower = license_text.lower()
    license_note = "BSD-2-Clause with additional no-sell condition" if "bsd-2" in license_lower and "sell" in license_lower else "See bundled LICENSE"

    result = {
        "sequence": sequence,
        "archive": str(archive_path.relative_to(ROOT)),
        "archive_size_bytes": archive_path.stat().st_size,
        "license_member": license_member,
        "license_note": license_note,
        "streams": {
            "barometer_member": baro_member,
            "barometer_columns": ["t", "p"],
            "barometer_pressure_unit": "Pa",
            "rtk_member": rtk_member,
            "rtk_up_column": "p_z (local ENU up, m)",
            "rtk_fixed_rule": "rtk_status == 1 (RTK fixed)",
            "clock_alignment": "t_common = RTK t - time_info.t_mag_gps; barometer t is already on this common clock",
            "clock_offset_t_mag_gps_s": clock_offset_s,
        },
        "duration_s": duration_s,
        "duration_basis": "span of fixed RTK sample times overlapping the barometer stream (last minus first)",
        "rtk_altitude_min_m": altitude_min,
        "rtk_altitude_max_m": altitude_max,
        "rtk_altitude_range_m": altitude_max - altitude_min,
        "median_ground_speed_m_s": float(np.median(finite_speed)) if finite_speed.size else None,
        "rtk_fix_share": fixed_share,
        "rtk_fix_share_basis": "count(rtk_status == 1) / all rtk_gps1 rows; RTK status 1 is fixed",
        "rtk_samples_total": int(len(rtk)),
        "rtk_fixed_samples_total": int(np.count_nonzero(rtk_status == 1.0)),
        "analysis_samples": int(time.size),
        "fit": fit,
        "regression": {
            "altitude_scale_error_slope": float(regression_coefficients[0]),
            "altitude_scale_error_percent": float(regression_coefficients[0] * 100.0),
            "ramp_m_s": float(regression_coefficients[2]),
            "intercept_m": float(regression_coefficients[1]),
        },
        "structure_function": structure,
    }
    return result, structure


def main() -> None:
    archives = sorted(INPUT_DIR.glob("*_sensors.zip"))
    if not archives:
        raise FileNotFoundError(f"No *_sensors.zip archives found in {INPUT_DIR}")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    results = []
    structure_rows = []
    for archive_path in archives:
        result, structure = _analyze_archive(archive_path)
        results.append(result)
        structure_rows.extend({"sequence": result["sequence"], **row} for row in structure)

    payload = {
        "dataset": "INSANE outdoor and Mars-analog sensor data",
        "method": {
            "barometer_altitude": "h = 44330 * (1 - (p / 101325) ** (1 / 5.255)); p in Pa",
            "error": "e = barometer_altitude interpolated to fixed RTK times - RTK p_z, then subtract first e",
            "structure_function": "All pairs with |(t_j - t_i) - horizon| <= 0.5 s; report median absolute increment, p95, RMS, and pair count",
            "fit": "RMS(tau)^2 = 2*sw^2 + q*tau + (d*tau)^2; non-negative least squares weighted by 1/RMS^2",
            "regression": "barometer_altitude - RTK p_z on [RTK p_z - mean, 1, t - t0], fixed samples only",
        },
        "zurich_reference_model": ZURICH_MODEL,
        "sequences": results,
    }
    (OUTPUT_DIR / "summary.json").write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
    pd.DataFrame(structure_rows, columns=["sequence", "horizon_s", "median_abs_m", "p95_abs_m", "rms_m", "n_pairs"]).to_csv(
        OUTPUT_DIR / "structure_function.csv", index=False
    )

    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    for result in results:
        rows = [row for row in result["structure_function"] if row["rms_m"] is not None and row["rms_m"] > 0]
        if rows:
            ax.loglog(
                [row["horizon_s"] for row in rows],
                [row["rms_m"] for row in rows],
                marker="o",
                linewidth=1.6,
                label=result["sequence"],
            )
    tau_ref = np.geomspace(0.5, max(HORIZONS_S), 300)
    model = ZURICH_MODEL
    rms_ref = np.sqrt(
        2.0 * model["white_m"] ** 2
        + model["random_walk_m_sqrt_s"] ** 2 * tau_ref
        + (model["ramp_m_s"] * tau_ref) ** 2
    )
    ax.loglog(tau_ref, rms_ref, "k--", linewidth=1.8, label="Zurich fitted model")
    ax.set_xlabel("Time separation $\\tau$ (s)")
    ax.set_ylabel("RMS $|\\Delta e|$ (m)")
    ax.set_title("INSANE barometer altitude error vs fixed RTK")
    ax.grid(True, which="both", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "structure_function.png", dpi=150)
    plt.close(fig)

    print(f"Wrote {OUTPUT_DIR.relative_to(ROOT)} for {len(results)} sequences")
    for result in results:
        fit = result["fit"]
        regression = result["regression"]
        print(
            f"{result['sequence']}: duration={result['duration_s']:.1f}s, "
            f"altitude_range={result['rtk_altitude_range_m']:.2f}m, "
            f"fixed_share={result['rtk_fix_share']:.3f}, "
            f"sw={fit['sw_m']:.4f}m, sqrt(q)={fit['sqrt_q_m_sqrt_s']:.5f}m/sqrt(s), "
            f"d={fit['d_m_s']:.6f}m/s, scale={regression['altitude_scale_error_percent']:.3f}%, "
            f"ramp={regression['ramp_m_s']:.6f}m/s"
        )


if __name__ == "__main__":
    main()

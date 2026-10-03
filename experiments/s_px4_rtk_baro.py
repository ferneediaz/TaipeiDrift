"""Measure PX4 barometer error against RTK-fixed GNSS during airborne segments.

Run from the repository root:
    .venv/bin/python experiments/s_px4_rtk_baro.py

The exported CSVs are preferred; a missing log export is read directly from its
public ULog with pyulog. Sensor timestamps are PX4 microseconds; exported t_s is
that timestamp relative to the same ULog start timestamp.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import nnls

ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "data/processed/t_px4"
RAW_DIR = ROOT / "data/raw/px4_logs"
OUT_DIR = ROOT / "data/processed/px4_rtk_baro"
LOG_IDS = (
    "036fb3a7-f5d1-49ab-9f34-144f373a65ba",
    "53736001-8385-4656-aea5-5fd0894b1bbb",
    "a2a30b98-ee1b-4015-9516-d79005f2ad71",
)
HORIZONS_S = np.asarray([10, 30, 60, 120, 300, 600, 1200], dtype=np.float64)
AIRBORNE_GAP_S = 5.0
ZURICH_SW_M = 0.30
ZURICH_Q_M2_S = 0.112
ZURICH_DRIFT_M_S = 0.0024


def _numeric(frame: pd.DataFrame, column: str, default: float = np.nan) -> np.ndarray:
    if column not in frame:
        return np.full(len(frame), default, dtype=np.float64)
    return pd.to_numeric(frame[column], errors="coerce").to_numpy(dtype=np.float64)


def _first_field(data: dict, *names: str) -> np.ndarray:
    for name in names:
        if name in data:
            return np.asarray(data[name], dtype=np.float64)
    return np.asarray([], dtype=np.float64)


def _pressure_to_pa(pressure: np.ndarray) -> np.ndarray:
    pressure = np.asarray(pressure, dtype=np.float64).copy()
    finite = pressure[np.isfinite(pressure) & (pressure > 0)]
    if finite.size:
        median = float(np.median(finite))
        # Some PX4 logs store hPa despite the exported pressure_pa column name.
        if 200.0 < median < 2000.0:
            pressure *= 100.0
    return pressure


def _pressure_altitude(pressure_pa: np.ndarray) -> np.ndarray:
    altitude = np.full(np.shape(pressure_pa), np.nan, dtype=np.float64)
    valid = np.isfinite(pressure_pa) & (pressure_pa > 0)
    altitude[valid] = 44330.0 * (1.0 - np.power(pressure_pa[valid] / 101325.0, 1.0 / 5.255))
    return altitude


def _clean_stream(t: np.ndarray, *values: np.ndarray) -> tuple[np.ndarray, ...]:
    t = np.asarray(t, dtype=np.float64)
    arrays = [np.asarray(value, dtype=np.float64) for value in values]
    n = min([t.size, *(value.size for value in arrays)])
    t = t[:n]
    arrays = [value[:n] for value in arrays]
    keep = np.isfinite(t)
    t = t[keep]
    arrays = [value[keep] for value in arrays]
    if not t.size:
        return (t, *arrays)
    order = np.argsort(t, kind="stable")
    t = t[order]
    arrays = [value[order] for value in arrays]
    unique_t, first = np.unique(t, return_index=True)
    # Duplicate timestamps are unusual, but choosing the first stable sample
    # avoids inventing a measurement by averaging independent sensor instances.
    return (unique_t, *(value[first] for value in arrays))


def _csv_streams(log_id: str) -> tuple[dict, dict] | None:
    log_dir = DATA_DIR / log_id
    baro_path, gnss_path = log_dir / "baro.csv", log_dir / "gnss.csv"
    if not (baro_path.is_file() and gnss_path.is_file()):
        return None

    baro = pd.read_csv(baro_path)
    baro_t = _numeric(baro, "t_s")
    air_alt = _numeric(baro, "vehicle_air_data_baro_alt_meter_m")
    if np.isfinite(air_alt).any():
        baro_alt = air_alt
        baro_source = "vehicle_air_data.baro_alt_meter"
    else:
        pressure = _pressure_to_pa(_numeric(baro, "pressure_pa"))
        baro_alt = _pressure_altitude(pressure)
        baro_source = "sensor_baro pressure altitude (standard atmosphere)"
    baro_t, baro_alt = _clean_stream(baro_t, baro_alt)
    baro_keep = np.isfinite(baro_alt)
    baro_data = {"t": baro_t[baro_keep], "alt": baro_alt[baro_keep], "source": baro_source}

    gnss = pd.read_csv(gnss_path)
    gnss_t = _numeric(gnss, "t_s")
    gnss_alt = _numeric(gnss, "alt_msl_m")
    fix = _numeric(gnss, "fix_type")
    vn, ve = _numeric(gnss, "vel_n"), _numeric(gnss, "vel_e")
    speed_fallback = _numeric(gnss, "vel_m_s")
    speed = np.where(np.isfinite(vn) & np.isfinite(ve), np.hypot(vn, ve), speed_fallback)
    vd = _numeric(gnss, "vel_d")
    if not np.isfinite(vd).any():
        vd = _numeric(gnss, "vel_up_m_s") * -1.0
    gnss_t, gnss_alt, fix, speed, vd = _clean_stream(gnss_t, gnss_alt, fix, speed, vd)
    gnss_data = {
        "t": gnss_t,
        "alt": gnss_alt,
        "fix": fix,
        "speed": speed,
        "vertical_up": -vd,
    }
    return baro_data, gnss_data


def _ulog_streams(log_id: str) -> tuple[dict, dict]:
    from pyulog import ULog

    raw_path = RAW_DIR / f"{log_id}.ulg"
    if not raw_path.is_file():
        raise FileNotFoundError(f"neither exported CSVs nor raw ULog found for {log_id}")
    ulog = ULog(str(raw_path))

    air_rows: list[tuple[np.ndarray, np.ndarray]] = []
    for dataset in ulog.data_list:
        if dataset.name != "vehicle_air_data":
            continue
        data = dataset.data
        stamp = _first_field(data, "timestamp")
        altitude = _first_field(data, "baro_alt_meter")
        n = min(stamp.size, altitude.size)
        if n:
            air_rows.append(((stamp[:n] - float(ulog.start_timestamp)) / 1_000_000.0, altitude[:n]))
    if air_rows:
        air_t = np.concatenate([row[0] for row in air_rows])
        air_alt = np.concatenate([row[1] for row in air_rows])
        baro_t, baro_alt = _clean_stream(air_t, air_alt)
        baro_keep = np.isfinite(baro_alt)
        baro_data = {"t": baro_t[baro_keep], "alt": baro_alt[baro_keep], "source": "vehicle_air_data.baro_alt_meter"}
    else:
        pressure_rows: list[tuple[np.ndarray, np.ndarray]] = []
        for dataset in ulog.data_list:
            if dataset.name != "sensor_baro":
                continue
            data = dataset.data
            stamp = _first_field(data, "timestamp_sample", "timestamp")
            pressure = _first_field(data, "pressure")
            n = min(stamp.size, pressure.size)
            if n:
                pressure_rows.append(((stamp[:n] - float(ulog.start_timestamp)) / 1_000_000.0, _pressure_to_pa(pressure[:n])))
        if not pressure_rows:
            raise ValueError(f"no vehicle_air_data or sensor_baro samples in {log_id}")
        baro_t = np.concatenate([row[0] for row in pressure_rows])
        pressure = np.concatenate([row[1] for row in pressure_rows])
        baro_t, pressure = _clean_stream(baro_t, pressure)
        baro_alt = _pressure_altitude(pressure)
        baro_keep = np.isfinite(baro_alt)
        baro_data = {"t": baro_t[baro_keep], "alt": baro_alt[baro_keep], "source": "sensor_baro pressure altitude (standard atmosphere)"}

    gps_topics = ("vehicle_gps_position", "sensor_gps")
    gps_sets = [dataset for dataset in ulog.data_list if dataset.name == gps_topics[0]]
    if not gps_sets:
        gps_sets = [dataset for dataset in ulog.data_list if dataset.name == gps_topics[1]]
    if not gps_sets:
        raise ValueError(f"no GNSS topic in {log_id}")
    dataset = max(gps_sets, key=lambda item: len(item.data.get("timestamp", [])))
    data = dataset.data
    stamps = _first_field(data, "timestamp", "timestamp_sample")
    alt = _first_field(data, "altitude_msl_m", "alt")
    if "altitude_msl_m" not in data:
        finite_alt = np.abs(alt[np.isfinite(alt)])
        if finite_alt.size and float(np.max(finite_alt)) > 20_000.0:
            alt = alt / 1000.0
    fix = _first_field(data, "fix_type")
    vn = _first_field(data, "vel_n_m_s", "vel_n")
    ve = _first_field(data, "vel_e_m_s", "vel_e")
    speed_fallback = _first_field(data, "vel_m_s")
    n = min(stamps.size, alt.size, fix.size)
    stamps, alt, fix = stamps[:n], alt[:n], fix[:n]
    if vn.size >= n and ve.size >= n:
        speed = np.hypot(vn[:n], ve[:n])
    else:
        speed = speed_fallback[:n] if speed_fallback.size >= n else np.full(n, np.nan)
    vd = _first_field(data, "vel_d_m_s", "vel_d")
    if vd.size < n:
        vel_up = _first_field(data, "vel_up_m_s")
        vd = -vel_up[:n] if vel_up.size >= n else np.full(n, np.nan)
    else:
        vd = vd[:n]
    gps_t = (stamps - float(ulog.start_timestamp)) / 1_000_000.0
    gps_t, alt, fix, speed, vd = _clean_stream(gps_t, alt, fix, speed, vd)
    return baro_data, {"t": gps_t, "alt": alt, "fix": fix, "speed": speed, "vertical_up": -vd}


def load_streams(log_id: str) -> tuple[dict, dict]:
    exported = _csv_streams(log_id)
    return exported if exported is not None else _ulog_streams(log_id)


def _airborne_segments(t: np.ndarray, airborne: np.ndarray) -> list[np.ndarray]:
    active = np.flatnonzero(airborne)
    if not active.size:
        return []
    segments: list[np.ndarray] = []
    start = previous = int(active[0])
    for index in active[1:]:
        index = int(index)
        if t[index] - t[previous] >= AIRBORNE_GAP_S:
            segments.append(np.arange(start, previous + 1, dtype=np.int64))
            start = index
        previous = index
    segments.append(np.arange(start, previous + 1, dtype=np.int64))
    return segments


def _correlation(x: np.ndarray, y: np.ndarray) -> float | None:
    valid = np.isfinite(x) & np.isfinite(y)
    if valid.sum() < 2 or np.std(x[valid]) == 0.0 or np.std(y[valid]) == 0.0:
        return None
    return float(np.corrcoef(x[valid], y[valid])[0, 1])


def _structure_function(t: np.ndarray, error: np.ndarray, segments: list[np.ndarray]) -> list[dict]:
    values: list[list[float]] = [[] for _ in HORIZONS_S]
    for indices in segments:
        valid = indices[np.isfinite(error[indices])]
        if valid.size < 2:
            continue
        segment_t = t[valid]
        segment_e = error[valid]
        for h_index, horizon in enumerate(HORIZONS_S):
            deltas: list[np.ndarray] = []
            for i, start_t in enumerate(segment_t[:-1]):
                left = np.searchsorted(segment_t, start_t + horizon - 0.5, side="left")
                right = np.searchsorted(segment_t, start_t + horizon + 0.5, side="right")
                left = max(left, i + 1)
                if right > left:
                    deltas.append(np.abs(segment_e[left:right] - segment_e[i]))
            if deltas:
                values[h_index].extend(np.concatenate(deltas).tolist())

    rows = []
    for horizon, deltas in zip(HORIZONS_S, values):
        if deltas:
            array = np.asarray(deltas, dtype=np.float64)
            rows.append({
                "horizon_s": float(horizon),
                "median_abs_de_m": float(np.median(array)),
                "p95_abs_de_m": float(np.percentile(array, 95)),
                "rms_de_m": float(np.sqrt(np.mean(array * array))),
                "n_pairs": int(array.size),
            })
        else:
            rows.append({"horizon_s": float(horizon), "median_abs_de_m": None, "p95_abs_de_m": None, "rms_de_m": None, "n_pairs": 0})
    return rows


def _fit_structure(rows: list[dict]) -> dict:
    valid = [row for row in rows if row["rms_de_m"] is not None and row["rms_de_m"] > 0.0]
    if len(valid) < 3:
        return {"sw_m": None, "sqrt_q_m_sqrt_s": None, "d_m_s": None}
    tau = np.asarray([row["horizon_s"] for row in valid], dtype=np.float64)
    rms = np.asarray([row["rms_de_m"] for row in valid], dtype=np.float64)
    y = rms**2
    design = np.column_stack((np.ones(tau.size), tau, tau**2))
    weights = 1.0 / np.maximum(rms**2, 1e-24)
    coefficients, _ = nnls(design * weights[:, None], y * weights)
    return {
        "sw_m": float(math.sqrt(coefficients[0] / 2.0)),
        "sqrt_q_m_sqrt_s": float(math.sqrt(coefficients[1])),
        "d_m_s": float(math.sqrt(coefficients[2])),
    }


def _highpass_error(t: np.ndarray, error: np.ndarray, segments: list[np.ndarray]) -> np.ndarray:
    highpass = np.full(error.shape, np.nan, dtype=np.float64)
    for indices in segments:
        valid = indices[np.isfinite(error[indices])]
        if not valid.size:
            continue
        time_index = pd.to_timedelta(t[valid], unit="s")
        series = pd.Series(error[valid], index=time_index)
        rolling_median = series.rolling("60s", center=True, min_periods=1).median().to_numpy(dtype=np.float64)
        highpass[valid] = error[valid] - rolling_median
    return highpass


def _scan_airframes() -> dict[str, str]:
    scan_path = DATA_DIR / "scan.csv"
    if not scan_path.is_file():
        return {}
    with scan_path.open(newline="", encoding="utf-8") as stream:
        return {row.get("log_id", ""): row.get("airframe", "") for row in csv.DictReader(stream)}


def _analyze(log_id: str, airframe: str) -> tuple[dict, list[dict]]:
    baro, gnss = load_streams(log_id)
    rtk = np.isfinite(gnss["fix"]) & (gnss["fix"] == 6) & np.isfinite(gnss["alt"])
    t, altitude, speed, vertical_up = (gnss[key][rtk] for key in ("t", "alt", "speed", "vertical_up"))
    if t.size < 2:
        raise ValueError(f"fewer than two RTK-fixed GNSS samples for {log_id}")

    initial = (t >= 0.0) & (t <= 20.0) & np.isfinite(altitude)
    if not initial.any():
        initial = (t >= t[0]) & (t <= t[0] + 20.0) & np.isfinite(altitude)
    if not initial.any():
        raise ValueError(f"no RTK altitude samples in the initial 20 seconds for {log_id}")
    takeoff_reference = float(np.median(altitude[initial]))
    airborne = (altitude > takeoff_reference + 3.0) | (np.isfinite(speed) & (speed > 1.0))
    segments = _airborne_segments(t, airborne)
    segment_indices = np.concatenate(segments) if segments else np.asarray([], dtype=np.int64)
    if not segment_indices.size:
        raise ValueError(f"no airborne samples found for {log_id}")

    baro_t, baro_alt = baro["t"], baro["alt"]
    if baro_t.size < 2:
        raise ValueError(f"fewer than two barometer samples for {log_id}")
    order = np.argsort(baro_t, kind="stable")
    baro_t, baro_alt = baro_t[order], baro_alt[order]
    baro_t, baro_alt = _clean_stream(baro_t, baro_alt)
    interpolated_baro = np.full(t.shape, np.nan, dtype=np.float64)
    in_range = (t >= baro_t[0]) & (t <= baro_t[-1])
    interpolated_baro[in_range] = np.interp(t[in_range], baro_t, baro_alt)
    error = interpolated_baro - altitude
    error[~np.isin(np.arange(t.size), segment_indices)] = np.nan

    structure = _structure_function(t, error, segments)
    fit = _fit_structure(structure)
    valid_error = segment_indices[np.isfinite(error[segment_indices]) & np.isfinite(speed[segment_indices])]
    if not valid_error.size:
        raise ValueError(f"no overlapping airborne barometer and RTK samples for {log_id}")
    alt_center = float(np.mean(altitude[valid_error]))
    t0 = float(np.min(t[segment_indices]))
    regression = np.column_stack((
        altitude[valid_error] - alt_center,
        np.ones(valid_error.size),
        t[valid_error] - t0,
        speed[valid_error] ** 2,
    ))
    coefficients, _, rank, _ = np.linalg.lstsq(regression, error[valid_error], rcond=None)
    scale_error = float(coefficients[0])
    ramp = float(coefficients[2])
    speed_squared_coefficient = float(coefficients[3])

    highpass = _highpass_error(t, error, segments)
    highpass_valid = segment_indices[np.isfinite(highpass[segment_indices])]
    hp_speed2_corr = _correlation(highpass[highpass_valid], speed[highpass_valid] ** 2)
    hp_vertical_corr = _correlation(highpass[highpass_valid], vertical_up[highpass_valid])

    airborne_altitude = altitude[segment_indices] - takeoff_reference
    airborne_speed = speed[segment_indices]
    finite_speed = airborne_speed[np.isfinite(airborne_speed)]
    segment_details = []
    for number, indices in enumerate(segments, start=1):
        segment_details.append({
            "segment": number,
            "start_s": float(t[indices[0]]),
            "end_s": float(t[indices[-1]]),
            "duration_s": float(t[indices[-1]] - t[indices[0]]),
            "rtk_samples": int(indices.size),
        })
    structure_by_horizon = {str(int(row["horizon_s"])): row for row in structure}
    result = {
        "log_id": log_id,
        "airframe": airframe or "unknown",
        "barometer_source": baro["source"],
        "gnss_source": "RTK-fixed GNSS (fix_type == 6)",
        "rtk_samples": int(t.size),
        "airborne_segment_count": len(segments),
        "airborne_segments": segment_details,
        "airborne_duration_s": float(sum(item["duration_s"] for item in segment_details)),
        "takeoff_reference_rtk_altitude_msl_m": takeoff_reference,
        "altitude_above_takeoff_m": {
            "min_m": float(np.min(airborne_altitude)),
            "max_m": float(np.max(airborne_altitude)),
            "range_m": float(np.max(airborne_altitude) - np.min(airborne_altitude)),
        },
        "ground_speed_m_s": {
            "median": float(np.median(finite_speed)) if finite_speed.size else None,
            "p95": float(np.percentile(finite_speed, 95)) if finite_speed.size else None,
            "min": float(np.min(finite_speed)) if finite_speed.size else None,
            "max": float(np.max(finite_speed)) if finite_speed.size else None,
        },
        "structure_fit": fit,
        "regression": {
            "altitude_scale_error_m_per_m": scale_error,
            "altitude_scale_error_percent": 100.0 * scale_error,
            "ramp_m_s": ramp,
            "speed_squared_coefficient_m_per_m_s_squared": speed_squared_coefficient,
            "implied_error_at_15_m_s_m": speed_squared_coefficient * 15.0**2,
            "regression_rank": int(rank),
            "samples": int(valid_error.size),
        },
        "highpass_correlations": {
            "rolling_median_window_s": 60,
            "error_vs_ground_speed_squared": hp_speed2_corr,
            "error_vs_vertical_speed_up": hp_vertical_corr,
            "samples": int(highpass_valid.size),
        },
        "structure_function": structure_by_horizon,
    }
    return result, structure


def _json_safe(value):
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    return value


def _write_structure_csv(path: Path, rows: list[dict]) -> None:
    columns = ["log_id", "airframe", "horizon_s", "median_abs_de_m", "p95_abs_de_m", "rms_de_m", "n_pairs"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def _write_plot(path: Path, results: list[dict]) -> None:
    figure, axis = plt.subplots(figsize=(9, 5.5), constrained_layout=True)
    tau = np.geomspace(10.0, 1200.0, 300)
    zurich = np.sqrt(2.0 * ZURICH_SW_M**2 + ZURICH_Q_M2_S * tau + (ZURICH_DRIFT_M_S * tau) ** 2)
    axis.plot(tau, zurich, "k--", linewidth=1.8, label="Zurich reference")
    for result in results:
        values = list(result["structure_function"].values())
        x = np.asarray([float(row["horizon_s"]) for row in values if row["rms_de_m"] is not None])
        y = np.asarray([float(row["rms_de_m"]) for row in values if row["rms_de_m"] is not None])
        if x.size:
            axis.plot(x, y, marker="o", linewidth=1.5, label=result["log_id"][:8])
    axis.set_xscale("log")
    axis.set_yscale("log")
    axis.set_xlabel("Horizon τ (s)")
    axis.set_ylabel("RMS |Δ(baro − RTK altitude)| (m)")
    axis.set_title("PX4 RTK-fixed barometer structure function")
    axis.grid(True, which="both", alpha=0.25)
    axis.legend()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main() -> None:
    airframes = _scan_airframes()
    results = []
    structure_rows = []
    for log_id in LOG_IDS:
        result, structure = _analyze(log_id, airframes.get(log_id, ""))
        results.append(result)
        structure_rows.extend({"log_id": log_id, "airframe": result["airframe"], **row} for row in structure)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    _write_structure_csv(OUT_DIR / "structure_function.csv", structure_rows)
    _write_plot(OUT_DIR / "structure_function.png", results)
    summary = {
        "logs": results,
        "horizons_s": HORIZONS_S.astype(int).tolist(),
        "structure_fit_model": "RMS(tau)^2 = 2*sw^2 + q*tau + (d*tau)^2; NNLS, least-squares residual weights 1/RMS^2",
        "zurich_reference": {
            "formula": "sqrt(2*0.30^2 + 0.112^2*tau + (0.0024*tau)^2)",
            "sw_m": ZURICH_SW_M,
            "q_m2_s": ZURICH_Q_M2_S,
            "d_m_s": ZURICH_DRIFT_M_S,
        },
        "caveats": [
            "vehicle_air_data.baro_alt_meter is preferred when logged; PX4 may already apply an offset correction, so this is not necessarily raw pressure drift.",
            "RTK-fixed GNSS is used instead of PX4 fused truth, but is not an independent surveyed altitude reference; GNSS altitude and barometer reference datums may differ.",
            "The altitude-or-speed mask can include ground taxiing or stationary ground tests; the mask alone does not prove flight.",
            "Pressure fallback normalizes hPa-like values to Pa and uses the standard-atmosphere pressure-altitude equation.",
            "Barometer and GNSS samples are aligned by the shared PX4 timestamp clock; barometer interpolation is linear and is not extrapolated beyond its sample span.",
        ],
    }
    with (OUT_DIR / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(_json_safe(summary), stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(_json_safe(summary), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()

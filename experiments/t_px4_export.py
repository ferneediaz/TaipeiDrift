"""Export sensor streams from the three selected public PX4 ULogs.

Run from the repository root with:
    uv run --no-project --with pyulog --with pandas --with numpy python experiments/t_px4_export.py

The truth.csv output is the PX4 fused estimate, not independent ground truth.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import pandas as pd
from pyulog import ULog

ROOT = Path(__file__).resolve().parents[1]
RAW_DIR = ROOT / "data/raw/px4_logs"
PROCESSED_DIR = ROOT / "data/processed/t_px4"
SCAN_PATH = PROCESSED_DIR / "scan.csv"
METADATA_PATH = RAW_DIR / "selected_candidates.json"
LICENSE_NOTE = (
    'Flight Review Browse page: “Use this script for automated download of public log files '
    '(license: CC-BY PX4).” The page links that label to CC BY 4.0.'
)
TRUTH_LABEL = "PX4 fused estimate, NOT independent truth"
RTK_SUBSTANTIAL_FRACTION = 0.05


def load_metadata() -> dict[str, dict]:
    if not METADATA_PATH.exists():
        return {}
    payload = json.loads(METADATA_PATH.read_text(encoding="utf-8"))
    rows = payload.get("logs", []) if isinstance(payload, dict) else payload
    return {row["log_id"]: row for row in rows if row.get("log_id")}


def load_scan() -> list[dict]:
    with SCAN_PATH.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def topic_datasets(ulog: ULog, name: str) -> list:
    return [item for item in ulog.data_list if item.name == name]


def get_field(data: dict, *names: str, default=None):
    for name in names:
        if name in data:
            return data[name]
    return default


def numeric(data: dict, *names: str, default=np.nan) -> np.ndarray:
    value = get_field(data, *names)
    if value is None:
        return np.asarray([], dtype=np.float64)
    try:
        return np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return np.full(len(value), default, dtype=np.float64)


def relative_seconds(timestamps: np.ndarray, ulog: ULog) -> np.ndarray:
    return (np.asarray(timestamps, dtype=np.float64) - float(ulog.start_timestamp)) / 1_000_000.0


def coordinate_degrees(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    finite = np.abs(values[np.isfinite(values)])
    if finite.size and float(np.max(finite)) > 180.0:
        return values / 10_000_000.0
    return values


def altitude_meters(values: np.ndarray, already_meters: bool = False) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    if already_meters:
        return values
    finite = np.abs(values[np.isfinite(values)])
    if finite.size and float(np.max(finite)) > 20_000.0:
        return values / 1000.0
    return values


def topic_rate(datasets: list) -> float | None:
    rates = []
    for dataset in datasets:
        timestamps = numeric(dataset.data, "timestamp", "timestamp_sample")
        timestamps = timestamps[np.isfinite(timestamps)]
        timestamps = np.unique(np.sort(timestamps))
        if timestamps.size > 1 and timestamps[-1] > timestamps[0]:
            rates.append((timestamps.size - 1) * 1_000_000.0 / (timestamps[-1] - timestamps[0]))
    return float(np.median(rates)) if rates else None


def make_imu(ulog: ULog) -> pd.DataFrame:
    combined = topic_datasets(ulog, "sensor_combined")
    rows = []
    for dataset in combined:
        data = dataset.data
        times = numeric(data, "timestamp_sample", "timestamp")
        gyro_names = tuple(f"gyro_rad[{axis}]" for axis in range(3))
        accel_names = tuple(f"accelerometer_m_s2[{axis}]" for axis in range(3))
        if not all(name in data for name in gyro_names + accel_names):
            continue
        for index, timestamp in enumerate(times):
            rows.append({
                "t_s": (timestamp - ulog.start_timestamp) / 1_000_000.0,
                "gx": float(data[gyro_names[0]][index]),
                "gy": float(data[gyro_names[1]][index]),
                "gz": float(data[gyro_names[2]][index]),
                "ax": float(data[accel_names[0]][index]),
                "ay": float(data[accel_names[1]][index]),
                "az": float(data[accel_names[2]][index]),
            })
    if rows:
        return pd.DataFrame(rows).sort_values("t_s", kind="stable").reset_index(drop=True)

    accel_data = max(topic_datasets(ulog, "sensor_accel"), key=lambda d: len(d.data.get("timestamp", [])), default=None)
    gyro_data = max(topic_datasets(ulog, "sensor_gyro"), key=lambda d: len(d.data.get("timestamp", [])), default=None)
    if accel_data is None or gyro_data is None:
        return pd.DataFrame(columns=["t_s", "gx", "gy", "gz", "ax", "ay", "az"])
    accel = accel_data.data
    gyro = gyro_data.data
    accel_times = numeric(accel, "timestamp_sample", "timestamp")
    gyro_times = numeric(gyro, "timestamp_sample", "timestamp")
    left = pd.DataFrame({
        "timestamp_us": accel_times,
        "ax": numeric(accel, "x"), "ay": numeric(accel, "y"), "az": numeric(accel, "z"),
    }).dropna(subset=["timestamp_us"]).sort_values("timestamp_us")
    right = pd.DataFrame({
        "timestamp_us": gyro_times,
        "gx": numeric(gyro, "x"), "gy": numeric(gyro, "y"), "gz": numeric(gyro, "z"),
    }).dropna(subset=["timestamp_us"]).sort_values("timestamp_us")
    joined = pd.merge_asof(left, right, on="timestamp_us", direction="nearest", tolerance=25_000)
    joined["t_s"] = relative_seconds(joined["timestamp_us"].to_numpy(), ulog)
    return joined[["t_s", "gx", "gy", "gz", "ax", "ay", "az"]].reset_index(drop=True)


def make_baro(ulog: ULog) -> tuple[pd.DataFrame, dict[str, float]]:
    raw_rows = []
    scale_factors = {}
    for dataset in topic_datasets(ulog, "sensor_baro"):
        data = dataset.data
        timestamps = numeric(data, "timestamp_sample", "timestamp")
        raw_pressure = numeric(data, "pressure")
        finite_pressure = raw_pressure[np.isfinite(raw_pressure) & (raw_pressure > 0.0)]
        scale = 100.0 if finite_pressure.size and float(np.median(finite_pressure)) < 2000.0 else 1.0
        scale_factors[str(dataset.multi_id)] = scale
        pressure = raw_pressure * scale
        temperature = numeric(data, "temperature")
        for index, timestamp in enumerate(timestamps):
            raw_rows.append({
                "t_s": (timestamp - ulog.start_timestamp) / 1_000_000.0,
                "instance": dataset.multi_id,
                "pressure_pa": float(pressure[index]) if index < len(pressure) else np.nan,
                "temperature_c": float(temperature[index]) if index < len(temperature) else np.nan,
            })
    bars = pd.DataFrame(raw_rows, columns=["t_s", "instance", "pressure_pa", "temperature_c"])
    if bars.empty:
        return pd.DataFrame(columns=["t_s", "instance", "pressure_pa", "temperature_c", "baro_alt_m", "vehicle_air_data_baro_alt_meter_m"]), scale_factors
    pressure = bars["pressure_pa"].to_numpy(dtype=np.float64)
    standard_altitude = np.full(pressure.shape, np.nan, dtype=np.float64)
    valid = np.isfinite(pressure) & (pressure > 0.0)
    standard_altitude[valid] = 44330.0 * (1.0 - np.power(pressure[valid] / 101325.0, 1.0 / 5.255))
    bars["baro_alt_m"] = standard_altitude

    air_rows = []
    for dataset in topic_datasets(ulog, "vehicle_air_data"):
        data = dataset.data
        timestamps = numeric(data, "timestamp")
        altitudes = numeric(data, "baro_alt_meter")
        for index, timestamp in enumerate(timestamps):
            if index < len(altitudes):
                air_rows.append({
                    "t_s": (timestamp - ulog.start_timestamp) / 1_000_000.0,
                    "vehicle_air_data_baro_alt_meter_m": float(altitudes[index]),
                })
    if air_rows:
        air = pd.DataFrame(air_rows).sort_values("t_s", kind="stable")
        bars = pd.merge_asof(bars.sort_values("t_s", kind="stable"), air, on="t_s", direction="nearest", tolerance=1.0)
    else:
        bars["vehicle_air_data_baro_alt_meter_m"] = np.nan
    return bars[["t_s", "instance", "pressure_pa", "temperature_c", "baro_alt_m", "vehicle_air_data_baro_alt_meter_m"]].reset_index(drop=True), scale_factors


def make_gnss(ulog: ULog) -> pd.DataFrame:
    datasets = topic_datasets(ulog, "vehicle_gps_position")
    if not datasets:
        datasets = topic_datasets(ulog, "sensor_gps")
    if not datasets:
        return pd.DataFrame(columns=["t_s", "lat_deg", "lon_deg", "alt_msl_m", "fix_type", "eph_m", "epv_m", "vel_n", "vel_e", "vel_d", "satellites"])
    dataset = max(datasets, key=lambda d: len(d.data.get("timestamp", [])))
    data = dataset.data
    timestamps = numeric(data, "timestamp")
    lat = coordinate_degrees(numeric(data, "lat", "latitude_deg", "latitude"))
    lon = coordinate_degrees(numeric(data, "lon", "longitude_deg", "longitude"))
    if "altitude_msl_m" in data:
        alt = altitude_meters(numeric(data, "altitude_msl_m"), already_meters=True)
    else:
        alt = altitude_meters(numeric(data, "alt"))
    fields = {
        "fix_type": numeric(data, "fix_type"),
        "eph_m": numeric(data, "eph"),
        "epv_m": numeric(data, "epv"),
        "vel_n": numeric(data, "vel_n_m_s", "vel_n"),
        "vel_e": numeric(data, "vel_e_m_s", "vel_e"),
        "vel_d": numeric(data, "vel_d_m_s", "vel_d"),
        "satellites": numeric(data, "satellites_used", "satellites"),
    }
    rows = []
    for index, timestamp in enumerate(timestamps):
        def at(values):
            return float(values[index]) if index < len(values) else np.nan
        rows.append({
            "t_s": (timestamp - ulog.start_timestamp) / 1_000_000.0,
            "lat_deg": at(lat), "lon_deg": at(lon), "alt_msl_m": at(alt),
            **{name: at(values) for name, values in fields.items()},
        })
    return pd.DataFrame(rows).sort_values("t_s", kind="stable").reset_index(drop=True)


def make_camera(ulog: ULog) -> pd.DataFrame:
    rows = []
    for topic in ("camera_trigger", "camera_capture"):
        for dataset in topic_datasets(ulog, topic):
            data = dataset.data
            timestamps = numeric(data, "timestamp")
            lat = coordinate_degrees(numeric(data, "lat", "latitude_deg", "latitude"))
            lon = coordinate_degrees(numeric(data, "lon", "longitude_deg", "longitude"))
            if "altitude_msl_m" in data:
                alt = altitude_meters(numeric(data, "altitude_msl_m"), already_meters=True)
            else:
                alt = altitude_meters(numeric(data, "alt"))
            seq = numeric(data, "seq", "sequence", "frame_seq", "sequence_number")
            for index, timestamp in enumerate(timestamps):
                rows.append({
                    "t_s": (timestamp - ulog.start_timestamp) / 1_000_000.0,
                    "seq": float(seq[index]) if index < len(seq) else np.nan,
                    "lat": float(lat[index]) if index < len(lat) else np.nan,
                    "lon": float(lon[index]) if index < len(lon) else np.nan,
                    "alt": float(alt[index]) if index < len(alt) else np.nan,
                    "event_type": topic,
                })
    if not rows:
        return pd.DataFrame()
    return pd.DataFrame(rows).sort_values("t_s", kind="stable").reset_index(drop=True)


def make_truth(ulog: ULog) -> pd.DataFrame:
    rows = []
    for topic in ("vehicle_global_position", "vehicle_local_position"):
        for dataset in topic_datasets(ulog, topic):
            data = dataset.data
            timestamps = numeric(data, "timestamp")
            if topic == "vehicle_global_position":
                lat = coordinate_degrees(numeric(data, "lat"))
                lon = coordinate_degrees(numeric(data, "lon"))
                altitude = numeric(data, "alt")
                for index, timestamp in enumerate(timestamps):
                    rows.append({
                        "t_s": (timestamp - ulog.start_timestamp) / 1_000_000.0,
                        "source_topic": topic,
                        "lat_deg": float(lat[index]) if index < len(lat) else np.nan,
                        "lon_deg": float(lon[index]) if index < len(lon) else np.nan,
                        "alt_msl_m": float(altitude[index]) if index < len(altitude) else np.nan,
                        "x_ned_m": np.nan, "y_ned_m": np.nan, "z_ned_m": np.nan,
                        "estimate_label": TRUTH_LABEL,
                    })
            else:
                x, y, z = (numeric(data, field) for field in ("x", "y", "z"))
                ref_lat = coordinate_degrees(numeric(data, "ref_lat"))
                ref_lon = coordinate_degrees(numeric(data, "ref_lon"))
                for index, timestamp in enumerate(timestamps):
                    rows.append({
                        "t_s": (timestamp - ulog.start_timestamp) / 1_000_000.0,
                        "source_topic": topic,
                        "lat_deg": float(ref_lat[index]) if index < len(ref_lat) else np.nan,
                        "lon_deg": float(ref_lon[index]) if index < len(ref_lon) else np.nan,
                        "alt_msl_m": np.nan,
                        "x_ned_m": float(x[index]) if index < len(x) else np.nan,
                        "y_ned_m": float(y[index]) if index < len(y) else np.nan,
                        "z_ned_m": float(z[index]) if index < len(z) else np.nan,
                        "estimate_label": TRUTH_LABEL,
                    })
    columns = ["t_s", "source_topic", "lat_deg", "lon_deg", "alt_msl_m", "x_ned_m", "y_ned_m", "z_ned_m", "estimate_label"]
    return pd.DataFrame(rows, columns=columns).sort_values("t_s", kind="stable").reset_index(drop=True) if rows else pd.DataFrame(columns=columns)


def eligible(row: dict) -> bool:
    return (
        row.get("parse_error", "") == ""
        and row.get("sensor_baro_present", "").lower() == "true"
        and row.get("has_imu", "").lower() == "true"
        and int(float(row.get("gps_fixes_ge3") or 0)) > 0
    )


def select_best(rows: list[dict], count: int) -> list[dict]:
    candidates = [row for row in rows if eligible(row)]
    def key(row: dict) -> tuple:
        rtk_fraction = float(row.get("frac_fix_rtk_fixed") or 0)
        capture = int(float(row.get("camera_capture_messages") or 0)) > 0
        duration = float(row.get("duration_s") or 0)
        return (rtk_fraction >= RTK_SUBSTANTIAL_FRACTION, capture, duration, rtk_fraction)
    return sorted(candidates, key=key, reverse=True)[:count]


def write_frame(frame: pd.DataFrame, path: Path) -> None:
    frame.to_csv(path, index=False, na_rep="")


def export_one(log_id: str, scan_row: dict, metadata: dict) -> dict:
    source = RAW_DIR / f"{log_id}.ulg"
    if not source.exists():
        raise FileNotFoundError(source)
    ulog = ULog(str(source))
    destination = PROCESSED_DIR / log_id
    destination.mkdir(parents=True, exist_ok=True)
    imu = make_imu(ulog)
    baro, pressure_scale_factors = make_baro(ulog)
    gnss = make_gnss(ulog)
    camera = make_camera(ulog)
    truth = make_truth(ulog)
    write_frame(imu, destination / "imu.csv")
    write_frame(baro, destination / "baro.csv")
    write_frame(gnss, destination / "gnss.csv")
    if not camera.empty:
        write_frame(camera, destination / "camera.csv")
    write_frame(truth, destination / "truth.csv")

    topics = (
        "sensor_baro", "sensor_combined", "sensor_accel", "sensor_gyro", "vehicle_gps_position", "sensor_gps",
        "vehicle_air_data", "camera_trigger", "camera_capture", "distance_sensor", "optical_flow", "vehicle_global_position",
    )
    rates = {}
    for topic in topics:
        datasets = topic_datasets(ulog, topic)
        rates[topic] = {"present": bool(datasets), "rate_hz": topic_rate(datasets)}
    meta = {
        "log_id": log_id,
        "url": f"https://review.px4.io/download?log={log_id}",
        "airframe": scan_row.get("airframe") or metadata.get("airframe_name") or metadata.get("airframe_type") or metadata.get("mav_type", ""),
        "hardware": metadata.get("sys_hw") or scan_row.get("hardware", ""),
        "software_release": metadata.get("ver_sw_release") or scan_row.get("ver_sw_release", ""),
        "duration_s": (ulog.last_timestamp - ulog.start_timestamp) / 1_000_000.0,
        "sensor_rates": rates,
        "imu_frame": "PX4 body FRD",
        "imu_units": {"gyro": "rad/s", "accelerometer": "m/s^2"},
        "imu_data_source": "sensor_combined" if topic_datasets(ulog, "sensor_combined") else "sensor_accel + sensor_gyro",
        "sensor_baro_pressure_scale_to_pa": pressure_scale_factors,
        "sensor_baro_pressure_scale_note": "pressure_pa is normalized to Pa; per-instance scale factors of 100 indicate raw readings inferred to be hPa/mbar from their magnitude.",
        "license_note": LICENSE_NOTE,
        "rtk_fixed_fraction": float(scan_row.get("frac_fix_rtk_fixed") or 0),
        "rtk_float_fraction": float(scan_row.get("frac_fix_rtk_float") or 0),
        "max_fix_type": scan_row.get("max_fix_type", ""),
        "truth_note": TRUTH_LABEL,
    }
    (destination / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return {
        "log_id": log_id,
        "duration_s": round(meta["duration_s"], 3),
        "rtk_fixed_fraction": meta["rtk_fixed_fraction"],
        "camera_capture_messages": int(float(scan_row.get("camera_capture_messages") or 0)),
        "directory": str(destination.relative_to(ROOT)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-ids", nargs="+", help="explicit IDs; by default choose the best three from scan.csv")
    parser.add_argument("--count", type=int, default=3, help="number of logs to export (default: 3)")
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count must be positive")
    scans = load_scan()
    metadata = load_metadata()
    by_id = {row["log_id"]: row for row in scans}
    chosen = args.log_ids if args.log_ids else [row["log_id"] for row in select_best(scans, args.count)]
    if len(chosen) < args.count and not args.log_ids:
        raise RuntimeError(f"Only {len(chosen)} downloaded logs meet raw barometer + IMU + GPS fix>=3 criteria")
    exports = [export_one(log_id, by_id[log_id], metadata.get(log_id, {})) for log_id in chosen]
    print(json.dumps({
        "exported_count": len(exports),
        "rtk_fixed_logs": [row["log_id"] for row in exports if row["rtk_fixed_fraction"] > 0],
        "rtk_substantial_logs": [row["log_id"] for row in exports if row["rtk_fixed_fraction"] >= RTK_SUBSTANTIAL_FRACTION],
        "exports": exports,
    }, indent=2))


if __name__ == "__main__":
    main()

"""Summarize sensor availability and GNSS quality for downloaded PX4 ULogs.

Run from the repository root with:
    uv run --no-project --with pyulog --with pandas --with numpy python experiments/t_px4_scan.py
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

ROOT = Path(__file__).resolve().parents[1]
RAW_DIR = ROOT / "data/raw/px4_logs"
OUT_PATH = ROOT / "data/processed/t_px4/scan.csv"
METADATA_PATH = RAW_DIR / "selected_candidates.json"
TOPICS = (
    "sensor_baro",
    "sensor_combined",
    "sensor_accel",
    "sensor_gyro",
    "vehicle_gps_position",
    "sensor_gps",
    "vehicle_air_data",
    "camera_trigger",
    "camera_capture",
    "distance_sensor",
    "optical_flow",
    "vehicle_global_position",
)


def load_metadata() -> dict[str, dict]:
    if not METADATA_PATH.exists():
        return {}
    payload = json.loads(METADATA_PATH.read_text(encoding="utf-8"))
    rows = payload.get("logs", []) if isinstance(payload, dict) else payload
    return {row["log_id"]: row for row in rows if row.get("log_id")}


def topic_datasets(ulog: ULog, name: str) -> list:
    return [item for item in ulog.data_list if item.name == name]


def topic_rate(datasets: list) -> float | None:
    rates = []
    for dataset in datasets:
        data = dataset.data
        timestamp = data.get("timestamp", data.get("timestamp_sample"))
        if timestamp is None:
            continue
        values = np.asarray(timestamp, dtype=np.float64)
        values = values[np.isfinite(values)]
        if values.size < 2:
            continue
        values = np.unique(np.sort(values))
        if values.size > 1 and values[-1] > values[0]:
            rates.append((values.size - 1) * 1_000_000.0 / (values[-1] - values[0]))
    return float(np.median(rates)) if rates else None


def field_array(datasets: list, field_names: tuple[str, ...]) -> np.ndarray:
    arrays = []
    for dataset in datasets:
        for name in field_names:
            if name in dataset.data:
                arrays.append(np.asarray(dataset.data[name]))
                break
    if not arrays:
        return np.asarray([])
    return np.concatenate(arrays)


def gps_source(ulog: ULog) -> tuple[str | None, list]:
    for name in ("vehicle_gps_position", "sensor_gps"):
        datasets = topic_datasets(ulog, name)
        if datasets:
            # Multiple sensor_gps instances can duplicate one fix stream. Use
            # the most complete instance for fix and RTK statistics.
            return name, [max(datasets, key=lambda d: len(d.data.get("timestamp", [])))]
    return None, []


def altitude_above_takeoff(ulog: ULog) -> tuple[float | None, str]:
    air_data = topic_datasets(ulog, "vehicle_air_data")
    altitudes = field_array(air_data, ("baro_alt_meter",)).astype(np.float64, copy=False)
    timestamps = field_array(air_data, ("timestamp",)).astype(np.float64, copy=False)
    valid = np.isfinite(altitudes) & np.isfinite(timestamps)
    if not np.any(valid):
        return None, ""
    altitudes = altitudes[valid]
    timestamps = timestamps[valid]
    order = np.argsort(timestamps)
    timestamps, altitudes = timestamps[order], altitudes[order]

    baseline_index = 0
    method = "first_vehicle_air_data_sample"
    land_topics = topic_datasets(ulog, "vehicle_land_detected")
    airborne_times = []
    for dataset in land_topics:
        landed = dataset.data.get("landed")
        stamp = dataset.data.get("timestamp")
        if landed is None or stamp is None:
            continue
        landed = np.asarray(landed).astype(bool)
        stamp = np.asarray(stamp, dtype=np.float64)
        if np.any(landed):
            after_ground = np.flatnonzero(~landed & (np.arange(len(landed)) > np.flatnonzero(landed)[0]))
            if after_ground.size:
                airborne_times.append(float(stamp[after_ground[0]]))
    if airborne_times:
        takeoff_time = min(airborne_times)
        before = np.flatnonzero(timestamps <= takeoff_time)
        baseline_index = int(before[-1]) if before.size else int(np.searchsorted(timestamps, takeoff_time, side="left"))
        baseline_index = min(baseline_index, len(altitudes) - 1)
        method = "last_vehicle_air_data_sample_at_or_before_first_airborne"
    return float(np.nanmax(altitudes) - altitudes[baseline_index]), method


def gps_metrics(ulog: ULog) -> dict:
    source, datasets = gps_source(ulog)
    if not datasets:
        return {
            "gps_source_topic": "",
            "gps_fixes_ge3": 0,
            "gps_ever_lost": "",
            "max_fix_type": "",
            "frac_fix_rtk_fixed": "",
            "frac_fix_rtk_float": "",
            "median_epv_m": "",
        }
    fixes = field_array(datasets, ("fix_type",)).astype(np.float64, copy=False)
    fixes = fixes[np.isfinite(fixes)]
    epv = field_array(datasets, ("epv",)).astype(np.float64, copy=False)
    epv = epv[np.isfinite(epv)]
    timestamps = field_array(datasets, ("timestamp",)).astype(np.float64, copy=False)
    if fixes.size and timestamps.size == fixes.size:
        ordered = fixes[np.argsort(timestamps)]
    else:
        ordered = fixes
    valid_seen = False
    gps_ever_lost = False
    for fix in ordered:
        if fix >= 3:
            valid_seen = True
        elif valid_seen:
            gps_ever_lost = True
            break
    count = int(fixes.size)
    return {
        "gps_source_topic": source or "",
        "gps_fixes_ge3": int(np.count_nonzero(fixes >= 3)),
        "gps_ever_lost": gps_ever_lost if np.any(fixes >= 3) else "",
        "max_fix_type": int(np.max(fixes)) if count else "",
        "frac_fix_rtk_fixed": float(np.count_nonzero(fixes == 6) / count) if count else "",
        "frac_fix_rtk_float": float(np.count_nonzero(fixes == 5) / count) if count else "",
        "median_epv_m": float(np.median(epv)) if epv.size else "",
    }


def fallback_airframe(ulog: ULog) -> str:
    params = getattr(ulog, "initial_parameters", {})
    autostart = params.get("SYS_AUTOSTART") if isinstance(params, dict) else None
    return f"SYS_AUTOSTART={autostart}" if autostart is not None else ""


def display_airframe(metadata: dict, ulog: ULog) -> str:
    name = str(metadata.get("airframe_name") or "").strip()
    if name:
        return name
    airframe_type = metadata.get("airframe_type")
    if airframe_type not in (None, ""):
        value = str(airframe_type).strip()
        if value.isdigit():
            mav_type = str(metadata.get("mav_type") or "airframe").strip()
            return f"{mav_type} (SYS_AUTOSTART={value})"
        return value
    return str(metadata.get("mav_type") or fallback_airframe(ulog))


def scan_one(path: Path, metadata: dict) -> dict:
    log_id = path.stem
    try:
        ulog = ULog(str(path))
    except Exception as error:
        return {
            "log_id": log_id,
            "parse_error": f"{type(error).__name__}: {error}",
        }

    row = {
        "log_id": log_id,
        "duration_s": (ulog.last_timestamp - ulog.start_timestamp) / 1_000_000.0,
        "airframe": display_airframe(metadata, ulog),
        "hardware": metadata.get("sys_hw", ""),
        "ver_sw_release": metadata.get("ver_sw_release", ""),
        "rating": metadata.get("rating", ""),
        "parse_error": "",
    }
    for topic in TOPICS:
        datasets = topic_datasets(ulog, topic)
        row[f"{topic}_present"] = bool(datasets)
        row[f"{topic}_rate_hz"] = topic_rate(datasets)
    row["camera_trigger_messages"] = sum(len(item.data.get("timestamp", [])) for item in topic_datasets(ulog, "camera_trigger"))
    row["camera_capture_messages"] = sum(len(item.data.get("timestamp", [])) for item in topic_datasets(ulog, "camera_capture"))
    row["max_baro_alt_above_takeoff_m"], row["baro_alt_baseline_method"] = altitude_above_takeoff(ulog)
    row.update(gps_metrics(ulog))
    row["has_imu"] = bool(topic_datasets(ulog, "sensor_combined")) or (
        bool(topic_datasets(ulog, "sensor_accel")) and bool(topic_datasets(ulog, "sensor_gyro"))
    )
    return row


def main() -> None:
    files = sorted(RAW_DIR.glob("*.ulg"))
    if not files:
        raise FileNotFoundError(f"No downloaded ULogs found in {RAW_DIR}")
    metadata = load_metadata()
    rows = [scan_one(path, metadata.get(path.stem, {})) for path in files]
    columns = [
        "log_id", "duration_s", "airframe", "hardware", "ver_sw_release", "rating", "parse_error", "has_imu",
        *[column for topic in TOPICS for column in (f"{topic}_present", f"{topic}_rate_hz")],
        "camera_trigger_messages", "camera_capture_messages", "max_baro_alt_above_takeoff_m",
        "baro_alt_baseline_method", "gps_source_topic", "gps_fixes_ge3", "gps_ever_lost", "max_fix_type",
        "frac_fix_rtk_fixed", "frac_fix_rtk_float", "median_epv_m",
    ]
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with OUT_PATH.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    errors = sum(bool(row.get("parse_error")) for row in rows)
    print(json.dumps({"downloaded_logs": len(files), "rows_written": len(rows), "parse_errors": errors, "scan_csv": str(OUT_PATH.relative_to(ROOT))}, indent=2))


if __name__ == "__main__":
    main()

"""Replay high-rate INSANE IMU + barometer after fixed-RTK loss.

Run from the repository root with ``.venv/bin/python experiments/s_insane_vertical.py``.
Outputs are written to ``data/processed/insane_vertical/``.
"""
from __future__ import annotations

import json
import re
import sys
import zipfile
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
INPUT_DIR = ROOT / "data" / "raw" / "insane"
OUTPUT_DIR = ROOT / "data" / "processed" / "insane_vertical"
G = 9.80665
BARO_REFERENCE_PRESSURE_PA = 101_325.0
DT_S = 0.1
HORIZONS_S = (1, 3, 10, 30, 60)
VARIANTS = ("baro_only", "baro_lowpass_1s", "imu_baro_ekf", "imu_only_after_cut")

sys.path.insert(0, str(Path(__file__).resolve().parent))
from s_zurich_vertical import causal_lowpass, vertical_ekf  # noqa: E402


def _member(names: list[str], suffix: str) -> str:
    matches = [name for name in names if name.endswith("/" + suffix)]
    if len(matches) != 1:
        raise ValueError(f"Expected one archive member ending in {suffix!r}; found {matches}")
    return matches[0]


def _read_csv(archive: zipfile.ZipFile, member: str, **kwargs) -> pd.DataFrame:
    with archive.open(member) as source:
        frame = pd.read_csv(source, skipinitialspace=True, **kwargs)
    frame.columns = [str(column).strip() for column in frame.columns]
    return frame


def _clean(frame: pd.DataFrame, time_col: str = "t") -> pd.DataFrame:
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(subset=[time_col])
    return frame.sort_values(time_col).drop_duplicates(time_col, keep="first").reset_index(drop=True)


def _clock_offset(archive: zipfile.ZipFile, names: list[str]) -> float:
    member = _member(names, "time_info.yaml")
    with archive.open(member) as source:
        contents = source.read().decode("utf-8", errors="replace")
    match = re.search(r"(?m)^\s*t_mag_gps:\s*([-+0-9.eE]+)", contents)
    if match is None:
        raise ValueError(f"No t_mag_gps clock offset in {member}")
    return float(match.group(1))


def _sample_rate(archive: zipfile.ZipFile, member: str) -> dict[str, float | int]:
    times = _read_csv(archive, member, usecols=["t"])["t"].to_numpy(dtype=float)
    if times.size < 2:
        return {"samples": int(times.size), "rate_hz": 0.0}
    return {"samples": int(times.size), "rate_hz": float((times[-1] - times[0]) and (times.size - 1) / (times[-1] - times[0]))}


def _bin_average(times: np.ndarray, values: np.ndarray, anchor_s: float, first_bin: int, n_bins: int) -> np.ndarray:
    index = np.floor((times - anchor_s) / DT_S).astype(np.int64) - first_bin
    valid = np.isfinite(values) & (index >= 0) & (index < n_bins)
    counts = np.bincount(index[valid], minlength=n_bins)
    sums = np.bincount(index[valid], weights=values[valid], minlength=n_bins)
    result = np.full(n_bins, np.nan, dtype=float)
    populated = counts > 0
    result[populated] = sums[populated] / counts[populated]
    return result


def _fill_empty_bins(values: np.ndarray, grid_t: np.ndarray) -> np.ndarray:
    result = values.copy()
    ok = np.isfinite(result)
    if not ok.any():
        raise ValueError("No samples fell into the 10 Hz grid")
    if not ok.all():
        result[~ok] = np.interp(grid_t[~ok], grid_t[ok], result[ok])
    return result


def _complementary_tilt(
    times: np.ndarray, accel_body: np.ndarray, gyro_body: np.ndarray, tau_s: float = 2.0
) -> tuple[np.ndarray, np.ndarray]:
    """Causal roll/pitch complementary filter; yaw is not needed for vertical acceleration."""
    roll = np.empty(len(times), dtype=float)
    pitch = np.empty(len(times), dtype=float)
    roll[0] = np.arctan2(accel_body[0, 1], accel_body[0, 2])
    pitch[0] = np.arctan2(-accel_body[0, 0], np.hypot(accel_body[0, 1], accel_body[0, 2]))
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        p, q, r = gyro_body[i - 1]
        sr, cr = np.sin(roll[i - 1]), np.cos(roll[i - 1])
        cp = np.cos(pitch[i - 1])
        cp = np.copysign(max(abs(cp), 0.05), cp if cp else 1.0)
        roll_pred = roll[i - 1] + (
            p + q * sr * np.sin(pitch[i - 1]) / cp + r * cr * np.sin(pitch[i - 1]) / cp
        ) * dt
        pitch_pred = pitch[i - 1] + (q * cr - r * sr) * dt
        roll_acc = np.arctan2(accel_body[i, 1], accel_body[i, 2])
        pitch_acc = np.arctan2(-accel_body[i, 0], np.hypot(accel_body[i, 1], accel_body[i, 2]))
        correction = 1.0 - np.exp(-dt / tau_s)
        roll_error = np.arctan2(np.sin(roll_acc - roll_pred), np.cos(roll_acc - roll_pred))
        roll[i] = roll_pred + correction * roll_error
        pitch[i] = (1.0 - correction) * pitch_pred + correction * pitch_acc
    return roll, pitch


def _rtk_interval_supported(times: np.ndarray, start_s: float, end_s: float, max_gap_s: float = 1.0) -> bool:
    if times.size < 2 or start_s < times[0] or end_s > times[-1]:
        return False
    inside = times[(times > start_s) & (times < end_s)]
    points = np.concatenate(([start_s], inside, [end_s]))
    return bool(np.all(np.diff(points) <= max_gap_s + 1e-9))


def _decimate_rtk_1hz(fixed_t: np.ndarray, fixed_z: np.ndarray, t_end: float) -> tuple[np.ndarray, np.ndarray]:
    keep = fixed_t <= t_end
    times, heights = fixed_t[keep], fixed_z[keep]
    if not times.size:
        return np.empty(0), np.empty(0)
    bin_id = np.floor(times - times[0]).astype(np.int64)
    n_bins = int(bin_id[-1]) + 1
    count = np.bincount(bin_id, minlength=n_bins)
    time_sum = np.bincount(bin_id, weights=times, minlength=n_bins)
    z_sum = np.bincount(bin_id, weights=heights, minlength=n_bins)
    ok = count > 0
    return time_sum[ok] / count[ok], z_sum[ok] / count[ok]



def _analyze_archive(archive_path: Path) -> tuple[dict, list[dict]]:
    with zipfile.ZipFile(archive_path) as archive:
        names = archive.namelist()
        baro_member = _member(names, "px4_baro.csv")
        rtk_member = _member(names, "rtk_gps1.csv")
        imu_member = _member(names, "px4_imu.csv")
        clock_offset_s = _clock_offset(archive, names)

        baro = _clean(_read_csv(archive, baro_member))
        rtk = _clean(_read_csv(archive, rtk_member))
        imu = _clean(_read_csv(archive, imu_member))

        stream_inventory = {}
        for suffix in ("px4_imu.csv", "lsm_imu.csv", "rs_imu.csv"):
            candidates = [name for name in names if name.endswith("/" + suffix)]
            if candidates:
                stream_inventory[suffix] = _sample_rate(archive, candidates[0])

        attitude_tokens = ("px4_attitude", "vehicle_attitude", "estimator", "rs_odom", "quaternion")
        nontruth_attitude_streams = []
        for name in names:
            base = Path(name).name.lower()
            if base.endswith(".csv") and any(token in base for token in attitude_tokens):
                nontruth_attitude_streams.append({"member": name, **_sample_rate(archive, name)})


    required_baro = {"t", "p"}
    required_rtk = {"t", "p_z", "rtk_status"}
    required_imu = {"t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z"}
    for frame, required, label in (
        (baro, required_baro, "barometer"), (rtk, required_rtk, "RTK"),
        (imu, required_imu, "PX4 IMU"),
    ):
        if not required.issubset(frame.columns):
            raise ValueError(f"Unexpected {label} columns: {list(frame.columns)}")

    baro_t_raw = baro["t"].to_numpy(dtype=float)
    pressure = baro["p"].to_numpy(dtype=float)
    baro_ok = np.isfinite(pressure) & (pressure > 0)
    baro_t = baro_t_raw[baro_ok]
    baro_alt = 44_330.0 * (1.0 - (pressure[baro_ok] / BARO_REFERENCE_PRESSURE_PA) ** (1.0 / 5.255))

    rtk_t = rtk["t"].to_numpy(dtype=float) - clock_offset_s
    rtk_z = rtk["p_z"].to_numpy(dtype=float)
    status = rtk["rtk_status"].to_numpy(dtype=float)
    fixed = (status == 1.0) & np.isfinite(rtk_t) & np.isfinite(rtk_z)
    fixed_t = rtk_t[fixed]
    fixed_z = rtk_z[fixed]
    if fixed_t.size < 2:
        raise ValueError(f"{archive_path.name}: fewer than two fixed RTK samples")

    imu_t = imu["t"].to_numpy(dtype=float)
    accel_body = imu[["a_x", "a_y", "a_z"]].to_numpy(dtype=float)
    gyro_body = imu[["w_x", "w_y", "w_z"]].to_numpy(dtype=float)
    imu_ok = np.isfinite(accel_body).all(axis=1) & np.isfinite(gyro_body).all(axis=1)
    imu_t, accel_body, gyro_body = imu_t[imu_ok], accel_body[imu_ok], gyro_body[imu_ok]
    if imu_t.size < 2:
        raise ValueError(f"{archive_path.name}: not enough finite PX4 IMU samples")

    imu_rate_hz = float(1.0 / np.median(np.diff(imu_t)))
    if imu_rate_hz < 100.0:
        raise ValueError(f"{archive_path.name}: PX4 IMU only {imu_rate_hz:.2f} Hz")

    # Keep full 100 ms bins inside the overlapping barometer/IMU interval.
    overlap_start = max(float(baro_t[0]), float(imu_t[0]))
    overlap_end = min(float(baro_t[-1]), float(imu_t[-1]))
    anchor_s = float(baro_t[0])
    first_bin = int(np.ceil((overlap_start - anchor_s) / DT_S))
    stop_bin = int(np.floor((overlap_end - anchor_s) / DT_S))
    bin_ids = np.arange(first_bin, stop_bin, dtype=np.int64)
    if bin_ids.size < 2:
        raise ValueError(f"{archive_path.name}: insufficient shared barometer/IMU interval")
    t = anchor_s + bin_ids * DT_S

    imu_in_range = (imu_t >= t[0]) & (imu_t < t[-1] + DT_S)
    imu_t_use = imu_t[imu_in_range]
    accel_use = accel_body[imu_in_range]
    gyro_use = gyro_body[imu_in_range]
    roll, pitch = _complementary_tilt(imu_t_use, accel_use, gyro_use, tau_s=2.0)
    # Rz(yaw) Ry(pitch) Rx(roll) body->ENU; yaw cancels from its world-up row.
    specific_up = (
        -np.sin(pitch) * accel_use[:, 0]
        + np.sin(roll) * np.cos(pitch) * accel_use[:, 1]
        + np.cos(roll) * np.cos(pitch) * accel_use[:, 2]
    )
    a_up_raw = specific_up - G
    if not np.isfinite(a_up_raw).all():
        raise ValueError(f"{archive_path.name}: non-finite vertical acceleration")

    accel_10hz = _bin_average(imu_t_use, a_up_raw, anchor_s, first_bin, len(t))
    accel_10hz = _fill_empty_bins(accel_10hz, t)
    baro_10hz = _bin_average(baro_t, baro_alt, anchor_s, first_bin, len(t))
    baro_10hz = _fill_empty_bins(baro_10hz, t)

    first20 = imu_t_use < imu_t_use[0] + 20.0
    filter_window = max(3, int(round(2.0 * imu_rate_hz)) | 1)
    rolling_mean = pd.Series(a_up_raw).rolling(
        window=filter_window, center=True, min_periods=max(3, filter_window // 2)
    ).mean().to_numpy()
    highpassed = a_up_raw - rolling_mean
    acc_sigma = float(np.nanstd(highpassed[first20]))

    fixed_scoring = (fixed_t >= t[0]) & (fixed_t <= t[-1] + DT_S)
    score_t = fixed_t[fixed_scoring]
    score_z = fixed_z[fixed_scoring]
    if score_t.size < 2:
        raise ValueError(f"{archive_path.name}: fewer than two fixed RTK samples overlap the replay grid")

    cuts = np.arange(fixed_t[0] + 30.0, t[-1] - 10.0 + 1e-9, 5.0)
    k_cut = np.searchsorted(t, cuts, side="left")
    valid_cuts = (k_cut < len(t)) & ((t[np.minimum(k_cut, len(t) - 1)] - cuts) < DT_S) & ((t[-1] - t[np.minimum(k_cut, len(t) - 1)]) >= 10.0)
    cuts = cuts[valid_cuts]
    k_cut = k_cut[valid_cuts].astype(np.int64)
    cut_grid_t = t[k_cut]
    if not cuts.size:
        raise ValueError(f"{archive_path.name}: no replay cuts with at least 10 s remaining")

    scored_window_mask = np.zeros(len(rtk_t), dtype=bool)
    for cut_t in cut_grid_t:
        supported_horizons = [
            horizon for horizon in HORIZONS_S
            if cut_t + horizon <= min(t[-1], score_t[-1])
            and _rtk_interval_supported(score_t, cut_t, cut_t + horizon)
        ]
        if supported_horizons:
            window_end = cut_t + max(supported_horizons)
            scored_window_mask |= (rtk_t >= cut_t) & (rtk_t <= window_end)
    rtk_scored_window_samples = int(np.count_nonzero(scored_window_mask))
    rtk_fixed_scored_window_samples = int(np.count_nonzero(scored_window_mask & fixed))
    rtk_fixed_fraction_scored_windows = (
        rtk_fixed_scored_window_samples / rtk_scored_window_samples if rtk_scored_window_samples else None
    )


    gnss_t, gnss_z = _decimate_rtk_1hz(fixed_t, fixed_z, float(t[-1]))
    cfg = dict(
        baro_sigma=0.5,
        baro_bias_rw=0.12,
        gnss_sigma=0.05,
        acc_sigma=acc_sigma,
        acc_bias_rw=0.002,
        acc_bias_sigma0=1.0,
        manoeuvre_sigma=0.5,
    )
    z_imu_baro = vertical_ekf(t, baro_10hz, accel_10hz, gnss_t, gnss_z, k_cut, True, True, cfg)
    z_imu_only = vertical_ekf(t, baro_10hz, accel_10hz, gnss_t, gnss_z, k_cut, True, False, cfg)
    z_lowpass = causal_lowpass(baro_10hz.copy(), DT_S, 1.0)
    estimates = {
        "baro_only": np.broadcast_to(baro_10hz, (len(cuts), len(t))),
        "baro_lowpass_1s": np.broadcast_to(z_lowpass, (len(cuts), len(t))),
        "imu_baro_ekf": z_imu_baro,
        "imu_only_after_cut": z_imu_only,
    }

    horizon_errors = {name: {h: [] for h in HORIZONS_S} for name in VARIANTS}
    hf_errors = {name: [] for name in VARIANTS}
    for i, cut_t in enumerate(cut_grid_t):
        for name, estimate in estimates.items():
            z_cut = float(estimate[i, k_cut[i]])
            for horizon in HORIZONS_S:
                target_t = cut_t + horizon
                if target_t > min(t[-1], score_t[-1]):
                    continue
                if not _rtk_interval_supported(score_t, cut_t, target_t):
                    continue
                ref_cut = float(np.interp(cut_t, score_t, score_z))
                z_target = float(np.interp(target_t, t, estimate[i]))
                ref_target = float(np.interp(target_t, score_t, score_z))
                horizon_errors[name][horizon].append((z_target - z_cut) - (ref_target - ref_cut))

            hf_end = cut_t + 10.0
            if hf_end > min(t[-1], score_t[-1]) or not _rtk_interval_supported(score_t, cut_t, hf_end):
                continue
            in_first10 = (score_t >= cut_t) & (score_t < hf_end)
            if np.count_nonzero(in_first10) >= 2:
                z_score = np.interp(score_t[in_first10], t, estimate[i])
                residual = z_score - score_z[in_first10]
                hf_errors[name].append(float(np.std(residual - np.mean(residual))))

    rows = []
    by_horizon = []
    for name in VARIANTS:
        for horizon in HORIZONS_S:
            error = np.asarray(horizon_errors[name][horizon], dtype=float)
            row = {
                "sequence": archive_path.stem.removesuffix("_sensors"),
                "variant": name,
                "horizon_s": horizon,
                "median_abs_m": float(np.median(np.abs(error))) if error.size else None,
                "p95_abs_m": float(np.percentile(np.abs(error), 95)) if error.size else None,
                "n": int(error.size),
            }
            by_horizon.append(row)
            rows.append(row)
    high_frequency = {
        name: {
            "median_m": float(np.median(values)) if values else None,
            "n": len(values),
        }
        for name, values in hf_errors.items()
    }

    sequence = archive_path.stem.removesuffix("_sensors")
    result = {
        "sequence": sequence,
        "archive": archive_path.name,
        "imu_stream": imu_member,
        "imu_rate_hz": imu_rate_hz,
        "available_accel_streams": stream_inventory,
        "attitude_source": "PX4 IMU gyro + accelerometer complementary tilt filter (2 s time constant; yaw ignored)",
        "attitude_rate_hz": imu_rate_hz,
        "available_nontruth_attitude_streams": nontruth_attitude_streams,
        "rotation_convention": "Rz(yaw) Ry(pitch) Rx(roll), active body-to-ENU; yaw cancels from the up row; a_up = (R_body_to_ENU @ f_body).z - 9.80665 m/s^2",
        "mean_a_up_mps2": float(np.mean(a_up_raw)),
        "mean_a_up_10hz_mps2": float(np.mean(accel_10hz)),
        "mean_a_up_check_abs_le_0_3": bool(abs(float(np.mean(a_up_raw))) <= 0.3),
        "clock_offset_t_mag_gps_s": clock_offset_s,
        "shared_duration_s": float(t[-1] - t[0]),
        "n_fixed_rtk_scoring_samples": int(score_t.size),
        "n_cuts": int(len(cuts)),
        "rtk_scored_window_samples": rtk_scored_window_samples,
        "rtk_fixed_scored_window_samples": rtk_fixed_scored_window_samples,
        "rtk_fixed_fraction_scored_windows": rtk_fixed_fraction_scored_windows,
        "acc_sigma_highpass_first_20s_mps2": acc_sigma,
        "config": cfg,
        "by_horizon": by_horizon,
        "high_frequency_error_first_10s": high_frequency,
    }
    return result, rows


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    archives = sorted(INPUT_DIR.glob("*_sensors.zip"))
    expected = {"mars_1_sensors.zip", "mars_2_sensors.zip", "outdoor_1_sensors.zip"}
    found = {path.name for path in archives}
    if found != expected:
        raise FileNotFoundError(f"Expected INSANE archives {sorted(expected)}; found {sorted(found)}")

    sequence_results = []
    horizon_rows = []
    for archive_path in archives:
        result, rows = _analyze_archive(archive_path)
        sequence_results.append(result)
        horizon_rows.extend(rows)

    by_horizon = pd.DataFrame(horizon_rows)
    by_horizon.to_csv(OUTPUT_DIR / "by_horizon.csv", index=False)
    summary = {
        "kind": "MEASURED: PX4 barometer + PX4 IMU, fixed RTK scoring reference, causal complementary-filter attitude",
        "attitude_source": "PX4 IMU gyroscope integration + accelerometer gravity correction; roll/pitch only, 2 s time constant",
        "frame_convention": "Tilt-only active body-to-ENU rotation; a_up = (R_body_to_ENU @ f_body).z - 9.80665 m/s^2.",
        "horizons_s": list(HORIZONS_S),
        "sequences": sequence_results,
    }
    (OUTPUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    for result in sequence_results:
        seq = result["sequence"]
        seq_table = by_horizon[by_horizon.sequence == seq]
        print(f"\n{seq}: PX4 IMU {result['imu_rate_hz']:.2f} Hz; "
              f"mean a_up={result['mean_a_up_mps2']:+.4f} m/s^2; cuts={result['n_cuts']}; "
              f"RTK fixed fraction in scored windows={result['rtk_fixed_fraction_scored_windows']:.3f}")
        print(seq_table.pivot(index="horizon_s", columns="variant", values="median_abs_m").round(3).to_string())
        print("high-frequency error, first 10 s (median std, m):", json.dumps(result["high_frequency_error_first_10s"], sort_keys=True))

    print("\nMedian / p95 absolute altitude-change error (m) and n:")
    print(by_horizon.to_string(index=False, float_format=lambda value: f"{value:.3f}"))
    print(f"\nWrote {OUTPUT_DIR / 'summary.json'} and {OUTPUT_DIR / 'by_horizon.csv'}")


if __name__ == "__main__":
    main()

"""Evaluate PX4 multicopter drag + learned wind as a GNSS-free velocity floor.

Run from the repository root with .venv/bin/python experiments/u6_px4_drag.py.
Fit EKF2-style drag from body X/Y accelerometer specific force and GNSS body
velocity in the first 40%; after the cut, invert that force model and rotate
air-relative velocity to NED. Post-cut GNSS is scoring only.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
RAW_DIR = ROOT / "data/raw/px4_logs"
OUT_DIR = ROOT / "data/processed/u6_px4_drag"
METADATA_PATH = RAW_DIR / "selected_candidates.json"
MANIFEST_PATH = OUT_DIR / "download_manifest.json"
FIT_FRACTION = 0.40
POSITION_STATES = {2, 3, 4}  # PX4 POSCTL, AUTO_MISSION, AUTO_LOITER.
POSITION_MODE_MIN_S = 300.0
DRAG_BCOEF_MIN = 5.0
DRAG_BCOEF_MAX = 200.0
HOVER_SPEED_M_S = 0.5
CRUISE_SPEED_M_S = 3.0
MIN_TRAIN_SAMPLES = 30
MIN_GPS_AT_CUT = 20
RHO_AIR = 1.225


def metadata_by_id() -> dict[str, dict]:
    metadata = {}
    for path in (METADATA_PATH, MANIFEST_PATH):
        if not path.exists():
            continue
        payload = json.loads(path.read_text(encoding="utf-8"))
        rows = payload.get("logs", []) if isinstance(payload, dict) else payload
        metadata.update({row["log_id"]: row for row in rows if row.get("log_id")})
    return metadata


def topic(ulog: ULog, name: str):
    datasets = [dataset for dataset in ulog.data_list if dataset.name == name]
    return max(datasets, key=lambda item: len(item.data.get("timestamp", [])), default=None)


def array(data: dict, *names: str) -> np.ndarray:
    for name in names:
        if name in data:
            try:
                return np.asarray(data[name], dtype=np.float64)
            except (TypeError, ValueError):
                continue
    return np.asarray([], dtype=np.float64)


def relative_time(data: dict, ulog: ULog, *timestamp_names: str) -> np.ndarray:
    stamps = array(data, *timestamp_names)
    return (stamps - float(ulog.start_timestamp)) / 1_000_000.0


def quaternion_matrix(q: np.ndarray) -> np.ndarray:
    """Rotation from PX4 FRD body to NED, for w,x,y,z quaternions."""
    q = np.asarray(q, dtype=np.float64)
    norms = np.linalg.norm(q, axis=-1)
    safe = norms > 1e-9
    normalized = np.zeros_like(q)
    normalized[safe] = q[safe] / norms[safe, None]
    normalized[~safe, 0] = 1.0
    w, x, y, z = normalized.T
    matrices = np.empty((len(q), 3, 3), dtype=np.float64)
    matrices[:, 0, 0] = 1 - 2 * (y * y + z * z)
    matrices[:, 0, 1] = 2 * (x * y - z * w)
    matrices[:, 0, 2] = 2 * (x * z + y * w)
    matrices[:, 1, 0] = 2 * (x * y + z * w)
    matrices[:, 1, 1] = 1 - 2 * (x * x + z * z)
    matrices[:, 1, 2] = 2 * (y * z - x * w)
    matrices[:, 2, 0] = 2 * (x * z - y * w)
    matrices[:, 2, 1] = 2 * (y * z + x * w)
    matrices[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return matrices


def interp_columns(source_t: np.ndarray, values: np.ndarray, target_t: np.ndarray) -> np.ndarray:
    source_t = np.asarray(source_t, dtype=np.float64)
    values = np.asarray(values, dtype=np.float64)
    target_t = np.asarray(target_t, dtype=np.float64)
    if source_t.size == 0 or values.shape[0] != source_t.size:
        return np.full((target_t.size, values.shape[1] if values.ndim == 2 else 1), np.nan)
    order = np.argsort(source_t, kind="stable")
    source_t, values = source_t[order], values[order]
    keep = np.r_[True, np.diff(source_t) > 0]
    source_t, values = source_t[keep], values[keep]
    result = np.column_stack([np.interp(target_t, source_t, values[:, k]) for k in range(values.shape[1])])
    return result


def mode_and_armed(ulog: ULog) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    dataset = topic(ulog, "vehicle_status")
    if dataset is None:
        return np.asarray([]), np.asarray([], dtype=int), np.asarray([], dtype=bool)
    data = dataset.data
    t = relative_time(data, ulog, "timestamp")
    states = array(data, "nav_state").astype(int)
    armed_state = array(data, "arming_state")
    armed = (armed_state == 2) if armed_state.size == t.size else np.ones(t.size, dtype=bool)
    n = min(t.size, states.size, armed.size)
    return t[:n], states[:n], armed[:n]


def mode_at(mode_t: np.ndarray, states: np.ndarray, armed: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if mode_t.size == 0:
        return np.full(t.shape, -1, dtype=int), np.zeros(t.shape, dtype=bool)
    idx = np.searchsorted(mode_t, t, side="right") - 1
    idx = np.clip(idx, 0, len(mode_t) - 1)
    return states[idx], armed[idx]


def gps_data(ulog: ULog) -> dict | None:
    candidates = [(name, topic(ulog, name)) for name in ("vehicle_gps_position", "sensor_gps", "vehicle_gnss")]
    candidates = [(name, item) for name, item in candidates if item is not None]
    if not candidates:
        return None
    name, dataset = max(candidates, key=lambda pair: len(pair[1].data.get("timestamp", pair[1].data.get("timestamp_sample", []))))
    data = dataset.data
    t = relative_time(data, ulog, "timestamp_sample", "timestamp") if name == "vehicle_gnss" else relative_time(data, ulog, "timestamp")
    if name == "vehicle_gnss":
        vn, ve, vd = (array(data, f"receiver.{field}") for field in ("vel_north", "vel_east", "vel_down"))
        fix = array(data, "receiver.fix_type", "fix_type")
        vel_valid = array(data, "receiver.vel_ned_valid", "vel_ned_valid")
        lat, lon = array(data, "receiver.latitude", "latitude_deg"), array(data, "receiver.longitude", "longitude_deg")
    else:
        vn, ve, vd = (array(data, *names) for names in (("vel_n_m_s", "vel_n"), ("vel_e_m_s", "vel_e"), ("vel_d_m_s", "vel_d")))
        fix = array(data, "fix_type")
        vel_valid = array(data, "vel_ned_valid")
        lat, lon = array(data, "latitude_deg", "lat"), array(data, "longitude_deg", "lon")
    if lat.size and np.nanmax(np.abs(lat)) > 180:
        lat = lat / 1e7
    if lon.size and np.nanmax(np.abs(lon)) > 180:
        lon = lon / 1e7
    if fix.size == 0 and vel_valid.size:
        fix = np.where(vel_valid > 0, 3.0, 0.0)
    n = min(t.size, vn.size, ve.size, vd.size, fix.size, lat.size, lon.size)
    if n == 0:
        return None
    vel = np.column_stack((vn[:n], ve[:n], vd[:n]))
    valid = (fix[:n] >= 3) & np.all(np.isfinite(vel), axis=1) & np.isfinite(lat[:n]) & np.isfinite(lon[:n])
    if vel_valid.size:
        valid &= vel_valid[:n] > 0
    return {"t": t[:n], "vel": vel, "lat": lat[:n], "lon": lon[:n], "valid": valid, "fix": fix[:n]}


def attitude_data(ulog: ULog) -> tuple[np.ndarray, np.ndarray] | None:
    dataset = topic(ulog, "vehicle_attitude")
    if dataset is None:
        return None
    data = dataset.data
    t = relative_time(data, ulog, "timestamp")
    q = np.column_stack([array(data, f"q[{i}]") for i in range(4)])
    n = min(t.size, q.shape[0])
    if n == 0:
        return None
    order = np.argsort(t[:n], kind="stable")
    t, q = t[:n][order], q[:n][order]
    # Resolve q/-q sign flips before component interpolation.
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0:
            q[i] *= -1
    keep = np.r_[True, np.diff(t) > 0]
    return t[keep], q[keep]




def accel_data(ulog: ULog) -> tuple[np.ndarray, np.ndarray] | None:
    dataset = topic(ulog, "sensor_combined")
    if dataset is None:
        return None
    data = dataset.data
    t = relative_time(data, ulog, "timestamp_sample", "timestamp")
    accel = np.column_stack([array(data, f"accelerometer_m_s2[{i}]") for i in range(3)])
    n = min(t.size, accel.shape[0])
    if n == 0:
        return None
    good = np.isfinite(t[:n]) & np.all(np.isfinite(accel[:n]), axis=1)
    order = np.argsort(t[:n][good], kind="stable")
    t, accel = t[:n][good][order], accel[:n][good][order]
    keep = np.r_[True, np.diff(t) > 0]
    return t[keep], accel[keep]


def position_ne(lat: np.ndarray, lon: np.ndarray) -> np.ndarray:
    lat0 = np.deg2rad(float(lat[0]))
    north = np.deg2rad(lat - float(lat[0])) * 6_371_000.0
    east = np.deg2rad(lon - float(lon[0])) * 6_371_000.0 * np.cos(lat0)
    return np.column_stack((north, east))


def path_length(t: np.ndarray, lat: np.ndarray, lon: np.ndarray, valid: np.ndarray) -> float:
    idx = np.flatnonzero(valid)
    if idx.size < 2:
        return 0.0
    xy = position_ne(lat[idx], lon[idx])
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def inspect_log(path: Path, metadata: dict) -> dict:
    ulog = ULog(str(path))
    duration = (ulog.last_timestamp - ulog.start_timestamp) / 1_000_000.0
    gps = gps_data(ulog)
    mode_t, states, armed = mode_and_armed(ulog)
    if gps is None:
        return {"path": path, "duration_s": duration, "gps": None, "qualifies": False, "reason": "missing GNSS"}
    gps_modes, gps_armed = mode_at(mode_t, states, armed, gps["t"])
    in_mode = np.isin(gps_modes, tuple(POSITION_STATES)) & gps_armed & gps["valid"]
    mode_duration = 0.0
    if mode_t.size > 1:
        mode_duration = float(np.sum(np.diff(mode_t)[np.isin(states[:-1], tuple(POSITION_STATES)) & armed[:-1]]))
    cutoff = FIT_FRACTION * duration
    post = in_mode & (gps["t"] >= cutoff)
    speed = np.linalg.norm(gps["vel"][:, :2], axis=1)
    hil = topic(ulog, "vehicle_status")
    hil_values = array(hil.data, "hil_state") if hil is not None else np.asarray([])
    sys_hil = ulog.initial_parameters.get("SYS_HITL", 0)
    is_hil = bool(hil_values.size and np.any(hil_values != 0)) or float(sys_hil or 0) != 0.0
    mav_param = ulog.initial_parameters.get("MAV_TYPE")
    mav_type = str(metadata.get("mav_type", "")).lower()
    is_quad = int(float(mav_param)) == 2 if mav_param is not None else "quadrotor" in mav_type
    is_real_flightreport = metadata.get("type") == "flightreport"
    candidate = (
        is_quad and is_real_flightreport and not is_hil and duration >= 600 and mode_duration >= POSITION_MODE_MIN_S
        and int(np.count_nonzero(gps["valid"] & (gps["t"] <= cutoff))) >= MIN_GPS_AT_CUT
        and attitude_data(ulog) is not None and accel_data(ulog) is not None
    )
    return {
        "path": path,
        "ulog": ulog,
        "gps": gps,
        "mode_t": mode_t,
        "states": states,
        "armed": armed,
        "duration_s": duration,
        "mav_type": metadata.get("mav_type", ""),
        "airframe": metadata.get("airframe_name") or metadata.get("airframe_type", ""),
        "hardware": metadata.get("sys_hw", ""),
        "firmware": metadata.get("ver_sw_release", ""),
        "mode_duration_s": mode_duration,
        "post_mode_distance_m": path_length(gps["t"], gps["lat"], gps["lon"], post),
        "max_position_speed_m_s": float(np.nanmax(speed[in_mode])) if np.any(in_mode) else float("nan"),
        "gps_valid_n": int(np.count_nonzero(gps["valid"])),
        "post_gps_n": int(np.count_nonzero(gps["valid"] & (gps["t"] >= cutoff))),
        "states_seen": sorted(set(int(x) for x in states)),
        "hil": is_hil,
        "qualifies": candidate,
        "reason": "eligible" if candidate else "fails quad/HIL/duration/mode/sensor/GNSS filter",
    }


def training_samples(record: dict, cutoff: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    gps = record["gps"]
    gps_t, gps_vel = gps["t"], gps["vel"]
    mode_t, states, armed = record["mode_t"], record["states"], record["armed"]
    att = attitude_data(record["ulog"])
    acc = accel_data(record["ulog"])
    if att is None or acc is None:
        return np.asarray([]), np.empty((0, 3)), np.empty((0, 3, 3)), np.empty((0, 3))
    att_t, quats = att
    acc_t, acc_values = acc
    valid_idx = np.flatnonzero(gps["valid"] & (gps_t <= cutoff))
    if valid_idx.size < 2:
        return np.asarray([]), np.empty((0, 3)), np.empty((0, 3, 3)), np.empty((0, 3))
    target_t, target_vel, target_q, target_acc = [], [], [], []
    prev_gps_t = -np.inf
    for idx in valid_idx:
        t = gps_t[idx]
        if not np.isfinite(prev_gps_t):
            prev_gps_t = t
            continue
        state, is_armed = mode_at(mode_t, states, armed, np.asarray([t]))
        if int(state[0]) not in POSITION_STATES or not bool(is_armed[0]):
            prev_gps_t = t
            continue
        q = interp_columns(att_t, quats, np.asarray([t]))[0]
        qnorm = np.linalg.norm(q)
        imu_mask = (acc_t > prev_gps_t) & (acc_t <= t)
        if not np.isfinite(qnorm) or qnorm < 0.5 or not np.any(imu_mask):
            prev_gps_t = t
            continue
        target_t.append(t)
        target_vel.append(gps_vel[idx])
        target_q.append(q / qnorm)
        target_acc.append(np.mean(acc_values[imu_mask], axis=0))
        prev_gps_t = t
    if not target_t:
        return np.asarray([]), np.empty((0, 3)), np.empty((0, 3, 3)), np.empty((0, 3))
    target_t = np.asarray(target_t)
    matrices = quaternion_matrix(np.asarray(target_q))
    return target_t, np.asarray(target_vel), matrices, np.asarray(target_acc)


def drag_specific_force(v_air_body: np.ndarray, mcoef: float, bcoef_x: float, bcoef_y: float) -> np.ndarray:
    speed = np.linalg.norm(v_air_body, axis=1)
    return np.column_stack((
        -mcoef * v_air_body[:, 0] - 0.5 * RHO_AIR * v_air_body[:, 0] * speed / bcoef_x,
        -mcoef * v_air_body[:, 1] - 0.5 * RHO_AIR * v_air_body[:, 1] * speed / bcoef_y,
    ))


def fit_drag(record: dict, cutoff: float, estimate_wind: bool = True) -> dict:
    samples = training_samples(record, cutoff)
    if len(samples) != 4:
        raise ValueError("training streams are unavailable")
    _, velocity_ned, rotation_body_to_ned, accel_body = samples
    usable = np.all(np.isfinite(velocity_ned), axis=1) & np.all(np.isfinite(accel_body[:, :2]), axis=1)
    velocity_ned = velocity_ned[usable]
    rotation_body_to_ned = rotation_body_to_ned[usable]
    accel_body = accel_body[usable, :2]
    if len(velocity_ned) < MIN_TRAIN_SAMPLES:
        raise ValueError(f"only {len(velocity_ned)} aligned GNSS/IMU training samples")

    def residual(parameters: np.ndarray) -> np.ndarray:
        if estimate_wind:
            mcoef, inv_bx, inv_by, wind_n, wind_e = parameters
        else:
            mcoef, inv_bx, inv_by = parameters
            wind_n = wind_e = 0.0
        v_air_ned = velocity_ned - np.array([wind_n, wind_e, 0.0])
        v_air_body = np.einsum("nji,nj->ni", rotation_body_to_ned, v_air_ned)
        predicted = drag_specific_force(v_air_body, mcoef, 1.0 / inv_bx, 1.0 / inv_by)
        return (predicted - accel_body).ravel()

    initial = np.array([0.15, 0.01, 0.01, 0.0, 0.0]) if estimate_wind else np.array([0.15, 0.01, 0.01])
    lower = [0.0, 1.0 / DRAG_BCOEF_MAX, 1.0 / DRAG_BCOEF_MAX]
    upper = [1.0, 1.0 / DRAG_BCOEF_MIN, 1.0 / DRAG_BCOEF_MIN]
    if estimate_wind:
        lower.extend([-30.0, -30.0])
        upper.extend([30.0, 30.0])
    fit = least_squares(
        residual, initial, bounds=(lower, upper),
        loss="soft_l1", f_scale=0.75, max_nfev=1000, x_scale="jac",
    )
    mcoef, inv_bx, inv_by = fit.x[:3]
    wind_n, wind_e = fit.x[3:] if estimate_wind else (0.0, 0.0)
    residuals = residual(fit.x).reshape(-1, 2)
    return {
        "mcoef_1_s": float(mcoef),
        "bcoef_x_kg_m2": float(1.0 / inv_bx),
        "bcoef_y_kg_m2": float(1.0 / inv_by),
        "wind_n_m_s": float(wind_n),
        "wind_e_m_s": float(wind_e),
        "wind_fit_mode": "constant_wind" if estimate_wind else "wind_fixed_zero",
        "fit_accel_rmse_m_s2": float(np.sqrt(np.mean(residuals**2))),
        "train_samples": int(len(velocity_ned)),
        "fit_success": bool(fit.success),
        "fit_cost": float(fit.cost),
    }


def plot_drag_diagnostic(record: dict, cutoff: float, fit_zero: dict, fit_wind: dict) -> dict:
    _, velocity_ned, rotation, accel_body = training_samples(record, cutoff)
    wind_ned = np.array([fit_wind["wind_n_m_s"], fit_wind["wind_e_m_s"], 0.0])
    velocity_body_zero = np.einsum("nji,nj->ni", rotation, velocity_ned)
    velocity_body_wind = np.einsum("nji,nj->ni", rotation, velocity_ned - wind_ned)
    ground_speed = np.linalg.norm(velocity_ned[:, :2], axis=1)
    category = np.where(ground_speed < HOVER_SPEED_M_S, 0, np.where(ground_speed > CRUISE_SPEED_M_S, 2, 1))
    colors = np.asarray(["tab:blue", "tab:gray", "tab:orange"])[category]
    force_fits = (fit_zero, fit_wind)
    velocity_fits = (velocity_body_zero, velocity_body_wind)
    titles = ("Wind fixed to zero", "Constant wind fitted")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex="col")
    stats = {
        "hover_samples_speed_lt_0_5_m_s": int(np.count_nonzero(category == 0)),
        "cruise_samples_speed_gt_3_m_s": int(np.count_nonzero(category == 2)),
    }
    for col, (model, body_velocity, title) in enumerate(zip(force_fits, velocity_fits, titles)):
        predicted = drag_specific_force(body_velocity, model["mcoef_1_s"], model["bcoef_x_kg_m2"], model["bcoef_y_kg_m2"])
        for axis in range(2):
            axes[axis, col].scatter(body_velocity[:, axis], accel_body[:, axis], c=colors, s=9, alpha=0.55, label="IMU specific force")
            axes[axis, col].scatter(body_velocity[:, axis], predicted[:, axis], color="black", s=8, marker="x", alpha=0.55, label="drag model")
            axes[axis, col].axhline(0.0, color="0.6", linewidth=0.7)
            axes[axis, col].axvline(0.0, color="0.6", linewidth=0.7)
            axes[axis, col].set_title(f"{title}, body {'XY'[axis]}")
            axes[axis, col].set_ylabel("Specific force (m/s²)")
            x, y = body_velocity[:, axis], accel_body[:, axis]
            corr = None if np.std(x) < 1e-9 or np.std(y) < 1e-9 else float(np.corrcoef(x, y)[0, 1])
            stats[f"body_{'xy'[axis]}_force_velocity_corr_{'wind0' if col == 0 else 'windfit'}"] = corr
        axes[1, col].set_xlabel("GNSS-derived air-relative body velocity (m/s)")
    axes[0, 0].legend(loc="best", fontsize=8)
    fig.suptitle(f"{record['path'].stem}: training data only (first 40%)")
    fig.tight_layout()
    fig.savefig(OUT_DIR / f"drag_fit_{record['path'].stem}.png", dpi=140)
    plt.close(fig)
    return stats


def invert_drag_force(accel_xy: np.ndarray, fit: dict) -> np.ndarray:
    """Invert the EKF2 body-axis drag observation for horizontal air velocity."""
    accel_xy = np.asarray(accel_xy, dtype=np.float64)
    mcoef = max(float(fit["mcoef_1_s"]), 1e-4)
    cx = 0.5 * RHO_AIR / float(fit["bcoef_x_kg_m2"])
    cy = 0.5 * RHO_AIR / float(fit["bcoef_y_kg_m2"])
    velocity = -accel_xy / mcoef
    for _ in range(24):
        speed = np.linalg.norm(velocity, axis=1)
        next_velocity = np.column_stack((
            -accel_xy[:, 0] / (mcoef + cx * speed),
            -accel_xy[:, 1] / (mcoef + cy * speed),
        ))
        velocity = 0.5 * (velocity + next_velocity)
    return velocity


def mean_acceleration_on_grid(accel_t: np.ndarray, accel: np.ndarray, grid: np.ndarray) -> np.ndarray:
    edges = np.searchsorted(accel_t, grid, side="right")
    samples = np.full((grid.size, 3), np.nan, dtype=np.float64)
    for i in range(1, grid.size):
        if edges[i] > edges[i - 1]:
            samples[i] = np.mean(accel[edges[i - 1]:edges[i]], axis=0)
    if grid.size > 1:
        samples[0] = samples[1]
    return samples


def simulate(record: dict, fit: dict, cutoff: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    ulog, gps = record["ulog"], record["gps"]
    attitude, accelerometer = attitude_data(ulog), accel_data(ulog)
    if attitude is None or accelerometer is None:
        raise ValueError("missing attitude or accelerometer specific force")
    att_t, quats = attitude
    accel_t, accel_body = accelerometer
    valid_before_cut = np.flatnonzero(gps["valid"] & (gps["t"] <= cutoff))
    if valid_before_cut.size == 0:
        raise ValueError("no valid GNSS velocity at the 40% cutoff")
    last_idx = int(valid_before_cut[-1])
    start_t = float(gps["t"][last_idx])
    if cutoff - start_t > 5.0:
        raise ValueError(f"last pre-cut GNSS velocity is {cutoff-start_t:.1f}s before cutoff")
    hold_velocity = gps["vel"][last_idx].copy()
    grid = att_t[(att_t >= start_t) & (att_t <= float(record["duration_s"]))]
    if grid.size < 2:
        raise ValueError("insufficient attitude samples after cutoff")
    q_grid = interp_columns(att_t, quats, grid)
    qnorm = np.linalg.norm(q_grid, axis=1)
    q_grid = q_grid / np.maximum(qnorm[:, None], 1e-9)
    specific_force = mean_acceleration_on_grid(accel_t, accel_body, grid)
    good = np.all(np.isfinite(q_grid), axis=1) & np.all(np.isfinite(specific_force), axis=1)
    grid, q_grid, specific_force = grid[good], q_grid[good], specific_force[good]
    if grid.size < 2 or grid[0] > start_t + 1.0:
        raise ValueError("attitude/accelerometer stream does not cover the cutoff")
    rotation = quaternion_matrix(q_grid)
    v_air_body_xy = invert_drag_force(specific_force[:, :2], fit)
    v_air_body = np.column_stack((v_air_body_xy, np.zeros(grid.size)))
    velocity = np.einsum("nij,nj->ni", rotation, v_air_body)
    velocity += np.array([fit["wind_n_m_s"], fit["wind_e_m_s"], 0.0])
    position = np.zeros((grid.size, 2), dtype=np.float64)
    dt = np.diff(grid)
    if np.any(dt <= 0.0) or np.any(dt > 0.5):
        raise ValueError("attitude integration has a non-positive or >0.5s gap")
    position[1:] = np.cumsum(0.5 * (velocity[:-1, :2] + velocity[1:, :2]) * dt[:, None], axis=0)
    return grid, velocity, position, hold_velocity, np.asarray([start_t, cutoff, np.median(np.linalg.norm(v_air_body_xy, axis=1))])




def one_km_legs(gps: dict, cutoff: float, grid: np.ndarray, pred_pos: np.ndarray, hold_v: np.ndarray) -> list[dict]:
    valid = gps["valid"] & (gps["t"] >= cutoff)
    idx = np.flatnonzero(valid)
    if idx.size < 2:
        return []
    truth_t = gps["t"][idx]
    truth_xy = position_ne(gps["lat"][idx], gps["lon"][idx])
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(truth_xy, axis=0), axis=1))]
    if distance[-1] < 1000.0:
        return []
    pred_xy = interp_columns(grid, pred_pos, truth_t)
    hold_xy = (truth_t - cutoff)[:, None] * hold_v[None, :]
    legs = []
    for leg_start in np.arange(0.0, distance[-1] - 999.999, 1000.0):
        leg_end = leg_start + 1000.0
        start = np.array([np.interp(leg_start, distance, truth_xy[:, axis]) for axis in range(2)])
        end = np.array([np.interp(leg_end, distance, truth_xy[:, axis]) for axis in range(2)])
        pred_start = np.array([np.interp(leg_start, distance, pred_xy[:, axis]) for axis in range(2)])
        pred_end = np.array([np.interp(leg_end, distance, pred_xy[:, axis]) for axis in range(2)])
        hold_start = np.array([np.interp(leg_start, distance, hold_xy[:, axis]) for axis in range(2)])
        hold_end = np.array([np.interp(leg_end, distance, hold_xy[:, axis]) for axis in range(2)])
        legs.append({
            "leg_start_km": round(leg_start / 1000.0, 3),
            "leg_end_km": round(leg_end / 1000.0, 3),
            "drag_error_m_per_km": float(np.linalg.norm((pred_end - pred_start) - (end - start))),
            "hold_error_m_per_km": float(np.linalg.norm((hold_end - hold_start) - (end - start))),
        })
    return legs


def score(record: dict, fit: dict, fit_zero: dict, diagnostics: dict, cutoff: float) -> tuple[dict, list[dict]]:
    grid, pred_vel, pred_pos, v0, sim_meta = simulate(record, fit, cutoff)
    gps = record["gps"]
    score_mask = gps["valid"] & (gps["t"] >= cutoff)
    score_t = gps["t"][score_mask]
    truth_vel = gps["vel"][score_mask, :2]
    predicted = interp_columns(grid, pred_vel[:, :2], score_t)
    drag_error = predicted - truth_vel
    hold_error = v0[:2][None, :] - truth_vel
    drag_rms = float(np.sqrt(np.mean(np.sum(drag_error**2, axis=1))))
    hold_rms = float(np.sqrt(np.mean(np.sum(hold_error**2, axis=1))))
    legs = one_km_legs(gps, cutoff, grid, pred_pos, v0[:2])
    leg_drag = np.asarray([leg["drag_error_m_per_km"] for leg in legs], dtype=np.float64)
    leg_hold = np.asarray([leg["hold_error_m_per_km"] for leg in legs], dtype=np.float64)
    row = {
        "label": "MEASURED",
        "log_id": record["path"].stem,
        "mav_type": record["mav_type"],
        "airframe": record["airframe"],
        "hardware": record["hardware"],
        "firmware": record["firmware"],
        "duration_s": float(record["duration_s"]),
        "cutoff_s": float(cutoff),
        "position_mode_s": float(record["mode_duration_s"]),
        "postcut_mission_position_distance_m": float(record["post_mode_distance_m"]),
        "gps_train_n": int(np.count_nonzero(gps["valid"] & (gps["t"] <= cutoff))),
        "gps_test_n": int(score_t.size),
        "velocity_rms_m_s": drag_rms,
        "hold_last_velocity_rms_m_s": hold_rms,
        "velocity_rms_gain_pct": float(100.0 * (hold_rms - drag_rms) / hold_rms) if hold_rms > 0 else float("nan"),
        "one_km_legs": len(legs),
        "one_km_error_m_per_km_mean": float(np.mean(leg_drag)) if leg_drag.size else None,
        "one_km_error_m_per_km_median": float(np.median(leg_drag)) if leg_drag.size else None,
        "hold_one_km_error_m_per_km_median": float(np.median(leg_hold)) if leg_hold.size else None,
        "predicted_air_speed_m_s_median": float(sim_meta[2]),
        "wind0_fit_accel_rmse_m_s2": fit_zero["fit_accel_rmse_m_s2"],
        "wind0_mcoef_1_s": fit_zero["mcoef_1_s"],
        "wind0_bcoef_x_kg_m2": fit_zero["bcoef_x_kg_m2"],
        "wind0_bcoef_y_kg_m2": fit_zero["bcoef_y_kg_m2"],
        **diagnostics,
        **fit,
    }
    return row, legs


def eligible_files() -> list[Path]:
    paths = list(RAW_DIR.glob("*.ulg"))
    paths.extend((RAW_DIR / "u6").glob("*.ulg"))
    return sorted(paths)


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def screen(metadata: dict[str, dict], paths: list[Path] | None = None) -> list[dict]:
    result = []
    for path in (eligible_files() if paths is None else paths):
        try:
            record = inspect_log(path, metadata.get(path.stem, {}))
            record.pop("ulog", None)
            result.append(record)
            print(
                f"{path.stem} duration={record['duration_s']:.1f}s modes={record.get('states_seen', [])} "
                f"position_mode={record.get('mode_duration_s', 0):.1f}s gps={record.get('gps_valid_n', 0)} "
                f"postcut_distance={record.get('post_mode_distance_m', 0):.1f}m eligible={record['qualifies']} "
                f"reason={record['reason']}"
            )
        except Exception as exc:
            result.append({"path": path, "qualifies": False, "reason": f"{type(exc).__name__}: {exc}"})
            print(f"{path.stem} eligible=False reason={type(exc).__name__}: {exc}")
    n_eligible = sum(bool(item["qualifies"]) for item in result)
    print(f"SCREEN: candidates={len(result)} eligible={n_eligible}; PX4 modes 2/3/4=position/mission/loiter")
    return result


def run(metadata: dict[str, dict], log_ids: list[str] | None, limit: int | None, screen_only: bool) -> None:
    requested = set(log_ids or [])
    paths = [path for path in eligible_files() if path.stem in requested] if requested else None
    if requested and {path.stem for path in paths} != requested:
        raise SystemExit(f"Requested logs unavailable: {sorted(requested - {path.stem for path in paths})}")
    records = screen(metadata, paths)
    if screen_only:
        return
    selected = [item for item in records if item.get("qualifies")]
    selected.sort(key=lambda item: (item["post_mode_distance_m"] >= 1000.0, item["post_mode_distance_m"]), reverse=True)
    if requested:
        selected = [item for item in selected if item["path"].stem in requested]
        if {item["path"].stem for item in selected} != requested:
            raise SystemExit("One or more requested PX4 logs fail the real-quad/GNSS/mode filters")
    if limit is not None:
        selected = selected[:limit]
    elif not requested:
        if len(selected) < 8:
            raise SystemExit(f"Need 8–10 eligible logs; found {len(selected)}. See screen output; no experiment outputs written.")
        selected = selected[:10]
    if not selected:
        raise SystemExit("No eligible PX4 quadrotor logs found")
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    rows, legs_rows, failures = [], [], []
    for record in selected:
        cutoff = FIT_FRACTION * record["duration_s"]
        try:
            record["ulog"] = ULog(str(record["path"]))
            fit_zero = fit_drag(record, cutoff, estimate_wind=False)
            fit = fit_drag(record, cutoff, estimate_wind=True)
            diagnostics = plot_drag_diagnostic(record, cutoff, fit_zero, fit)
            row, legs = score(record, fit, fit_zero, diagnostics, cutoff)
            rows.append(row)
            legs_rows.extend({"log_id": record["path"].stem, **leg} for leg in legs)
            print(
                f"MEASURED {record['path'].stem}: vel_RMS={row['velocity_rms_m_s']:.3f} m/s "
                f"hold={row['hold_last_velocity_rms_m_s']:.3f} m/s legs_1km={len(legs)} "
                f"fit_accel_RMSE wind0={fit_zero['fit_accel_rmse_m_s2']:.3f} wind={fit['fit_accel_rmse_m_s2']:.3f} m/s² "
                f"wind=({fit['wind_n_m_s']:.2f},{fit['wind_e_m_s']:.2f}) m/s "
                f"B=({fit['bcoef_x_kg_m2']:.1f},{fit['bcoef_y_kg_m2']:.1f}) kg/m² M={fit['mcoef_1_s']:.3f} 1/s "
                f"hover_samples={diagnostics['hover_samples_speed_lt_0_5_m_s']} cruise_samples={diagnostics['cruise_samples_speed_gt_3_m_s']}"
            )
        except Exception as exc:
            failures.append({"log_id": record["path"].stem, "reason": f"{type(exc).__name__}: {exc}"})
            print(f"EXCLUDED {record['path'].stem}: {type(exc).__name__}: {exc}")
        finally:
            record.pop("ulog", None)
    if not rows:
        raise SystemExit("No logs produced score rows; inspect exclusion output above")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    write_csv(OUT_DIR / "per_log.csv", rows)
    write_csv(OUT_DIR / "legs_1km.csv", legs_rows)
    drag_rms = np.asarray([row["velocity_rms_m_s"] for row in rows])
    hold_rms = np.asarray([row["hold_last_velocity_rms_m_s"] for row in rows])
    drag_legs = np.asarray([leg["drag_error_m_per_km"] for leg in legs_rows], dtype=np.float64)
    hold_legs = np.asarray([leg["hold_error_m_per_km"] for leg in legs_rows], dtype=np.float64)
    mean_drag_rms, mean_hold_rms = float(np.mean(drag_rms)), float(np.mean(hold_rms))
    pass_velocity = int(np.count_nonzero(drag_rms <= 1.0)) >= 5
    pass_legs = bool(drag_legs.size and float(np.median(drag_legs)) <= 100.0)
    kill = mean_drag_rms > 2.0 or mean_drag_rms >= mean_hold_rms
    verdict = "kill" if kill else "pass" if pass_velocity and pass_legs else "inconclusive"
    summary = {
        "verdict": verdict,
        "decision_rule": {
            "pass": "at least 5 logs with horizontal velocity RMS <= 1 m/s and median 1 km-leg endpoint error <= 100 m",
            "kill": "mean horizontal velocity RMS > 2 m/s or not better than mean hold-last-GNSS-velocity RMS",
        },
        "previous_attempt": {
            "verdict": "INCONCLUSIVE — invalid thrust-setpoint propagation, not evidence against drag fusion",
            "log_id": "480ec791-3397-43a1-a7d2-6f2d839936cc",
            "velocity_rms_m_s": 117.61161933134996,
            "hold_last_velocity_rms_m_s": 6.604059766229457,
            "one_km_error_m": [14804.222017417487, 13576.610252595478],
            "fit_wind_n_e_m_s": [-18.615089908836193, -9.627063880279826],
            "fit_bcoef_x_y_kg_m2": [231.30195399498626, 9999.999999999998],
        },
        "label": "MEASURED",
        "description": "Real PX4 Flight Review ULogs. Fit raw GNSS body velocity against sensor_combined body X/Y specific force on t <= 40%; after the cut, invert the drag-specific-force model and rotate air-relative velocity with vehicle_attitude. No thrust setpoint or EKF-fused velocity/wind is an estimator input. Post-cut GNSS is scoring only.",
        "candidate_logs": len(records),
        "selected_logs": len(selected),
        "scored_logs": len(rows),
        "scored_log_ids": [row["log_id"] for row in rows],
        "excluded_after_selection": failures,
        "cut_fraction": FIT_FRACTION,
        "wind_zero_fit": "Comparator fit with wind fixed to zero; metadata wind_speed=-1 means calm conditions are not established.",
        "bcoef_bounds_kg_m2": [DRAG_BCOEF_MIN, DRAG_BCOEF_MAX],
        "drag_observation": "a_xy_body = -MCOEF*v_rel_xy - rho/(2*BCOEF_X/Y)*v_rel_xy*|v_rel|; horizontal post-cut inversion assumes v_rel_down=0.",
        "position_mode_states": {"2": "POSCTL", "3": "AUTO_MISSION", "4": "AUTO_LOITER"},
        "velocity_rms_m_s": {"drag_wind_mean": float(np.mean(drag_rms)), "drag_wind_median": float(np.median(drag_rms)), "hold_last_mean": float(np.mean(hold_rms)), "hold_last_median": float(np.median(hold_rms))},
        "velocity_rms_better_than_hold_logs": int(np.count_nonzero(drag_rms < hold_rms)),
        "one_km_leg_count": int(len(legs_rows)),
        "one_km_error_m_per_km": {
            "drag_wind_mean": float(np.mean(drag_legs)) if drag_legs.size else None,
            "drag_wind_median": float(np.median(drag_legs)) if drag_legs.size else None,
            "hold_last_mean": float(np.mean(hold_legs)) if hold_legs.size else None,
            "hold_last_median": float(np.median(hold_legs)) if hold_legs.size else None,
        },
        "model_reference": "PX4 EKF2 multicopter drag-specific-force guidance: https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf#mc_wind_estimation_using_drag",
        "exclusions": failures,
    }
    (OUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print("SUMMARY", json.dumps(summary, sort_keys=True))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--screen", action="store_true", help="inspect local candidate logs without fitting")
    parser.add_argument("--log-id", dest="log_ids", action="append", help="run one or more specific eligible logs; repeat the option")
    parser.add_argument("--limit", type=int, help="run the top N eligible logs (pilot/kill test only)")
    args = parser.parse_args()
    run(metadata_by_id(), args.log_ids, args.limit, args.screen)


if __name__ == "__main__":
    main()

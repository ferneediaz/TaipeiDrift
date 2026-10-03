"""Replay a recorded simulator flight through Alessandro's ESKF, fed step by step with what the camera navigator measures.

The recordings in recordings/ hold the IMU (100 Hz), the barometer, GNSS (1 Hz) and the truth. The filter is the one
of the live adapter (sim/nodes/eskf_ros_adapter.py), run offline in the same way: started on the pad from GNSS and
the IMU's gravity, then IMU prediction, barometer height and GNSS position (with the velocity fit) while GNSS lasts.

    python scripts/fused_replay.py --flight wufeng_corridor_100m                    # step 1: GNSS all the way
    python scripts/fused_replay.py --flight wufeng_corridor_100m --cut              # step 2: GNSS lost after 450 m
    python scripts/fused_replay.py --flight wufeng_corridor_100m --cut --camera     # step 3: plus camera speed and sun heading
    python scripts/fused_replay.py --flight wufeng_corridor_100m --cut --camera --fixes   # step 4: plus the map fixes
    python scripts/fused_replay.py --all [--camera-model realistic]                 # every development flight and draw

Never run on the sealed flights; the development flights are listed in baseline/configs/sim_navigator.yaml.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
for extra in (REPO / "baseline", REPO, REPO / "sim" / "nodes", REPO / "baseline" / "scripts", REPO / "scripts"):
    sys.path.insert(0, str(extra))

import run_sim_navigator as S  # noqa: E402
from gnss_projection import geodetic_to_enu  # noqa: E402
from gnss_velocity_fit import fit_position_velocity  # noqa: E402
from sim_dev_check import flight_cfg  # noqa: E402
from src.data.sim_replay import with_heading  # noqa: E402
from src.estimation.camera_navigator import NavigatorConfig, calibrate  # noqa: E402
from src.estimation.image_motion import shifts_for_flight  # noqa: E402
from vio.eskf_pipeline import noise_model  # noqa: E402
from vio.estimation.eskf import ESKF, P_, TH, V_  # noqa: E402

GRAVITY = 9.80665


def load_recording(folder: Path) -> dict:
    """The sensor files of one recording, as arrays, and its world origin (latitude, longitude, height)."""
    meta = json.loads((folder / "meta.json").read_text())
    origin = (meta["origin"]["lat_deg"], meta["origin"]["lon_deg"], meta["origin"]["alt_m"])
    imu = pd.read_csv(folder / "imu.csv")
    baro = pd.read_csv(folder / "baro.csv")
    gnss = pd.read_csv(folder / "gnss.csv")
    truth = pd.read_csv(folder / "truth.csv")
    fixes = np.array([geodetic_to_enu(lat, lon, alt, origin) for lat, lon, alt in zip(gnss.lat_deg, gnss.lon_deg, gnss.alt_m)])
    return {
        "origin": origin,
        "imu_t": imu.t_s.to_numpy(), "gyro": imu[["gx", "gy", "gz"]].to_numpy(), "accel": imu[["ax", "ay", "az"]].to_numpy(),
        "baro_t": baro.t_s.to_numpy(), "pressure": baro.pressure_pa.to_numpy(),
        "gnss_t": gnss.t_s.to_numpy(), "gnss_enu": fixes,
        "truth_t": truth.t_s.to_numpy(), "truth_enu": truth[["e_m", "n_m", "u_m"]].to_numpy(),
        "truth_q": truth[["qw", "qx", "qy", "qz"]].to_numpy(),
    }


def yaw_of(rotation: Rotation) -> float:
    """Yaw of a body-to-ENU rotation: the angle of the body's forward axis from east, counter-clockwise, in radians."""
    return float(rotation.as_euler("ZYX")[0])


def settled_start(rec: dict, samples: int) -> int:
    """First IMU sample that ends a settled window (the adapter's rule): gravity steady, hardly any turning."""
    accel, gyro, t = rec["accel"], rec["gyro"], rec["imu_t"]
    for k in range(samples, len(t)):
        if t[k] < 1.0:
            continue
        a, w = accel[k - samples:k], gyro[k - samples:k]
        mean = a.mean(axis=0)
        if np.std(np.linalg.norm(a, axis=1)) <= 0.25 and 9.0 <= np.linalg.norm(mean) <= 10.6 and np.linalg.norm(w.mean(axis=0)) <= 0.05:
            return k
    raise ValueError("the IMU never settles: the recording does not start on the ground")


def camera_measurements(cfg: dict, flight_name: str, camera: str, seed: int, heading_source: str | None = None) -> dict:
    """What the camera navigator measures on one flight, for the filter: per frame the sun sensor's heading and the
    ground speed from the down camera.

    The speed is the image shift between two north-up frames, turned into metres by the matrix the navigator
    learns while GNSS works (camera_navigator.calibrate), over the time between the frames. Example: a shift worth
    1.9 m north in 0.2 s is 9.5 m/s north.
    """
    if heading_source:  # another heading sensor of the config: compass, sun_digital or sun_photodiode
        cfg = {**cfg, "heading": {**cfg["heading"], "source": heading_source}}
    fc = flight_cfg(cfg, flight_name, cfg["flights"]["development"][flight_name], camera, {})
    flight0 = S.flight_for(fc, "2018")
    heading = S.heading_reading(fc, flight0, seed)
    flight = with_heading(flight0, heading)
    shifts = shifts_for_flight(flight, S.flow_path(fc, seed))
    settings = {k: v for k, v in cfg["runs"]["camera_alone"].items() if k != "map"}
    calibration = calibrate(flight, shifts, NavigatorConfig(**{**fc["navigator"], **settings}))
    frame_t = flight.metadata["recording_t_s"]
    step_ne = shifts @ calibration.motion_matrix  # (K, 2) north, east in metres, from the frame before
    dt = np.r_[np.nan, np.diff(frame_t)]
    return {"flight": flight, "calibration": calibration, "t": frame_t, "jam_s": float(frame_t[calibration.jam_index]),
            "yaw": np.radians(90.0 - heading),  # heading is a bearing from north, yaw an angle from east
            "v_en": np.column_stack([step_ne[:, 1], step_ne[:, 0]]) / dt[:, None], "step_ne": step_ne,
            "cruise_mps": calibration.cruise_speed_mps, "config": fc}


def navigator_fixes(cfg: dict, camera: dict, seed: int) -> dict:
    """The map fixes the frozen camera navigator used on this flight, for the filter: time, east, north, in the
    world's frame. The navigator runs as it was frozen (its own dead reckoning, search and checks); only the
    fixes it used are handed on, each with the accuracy it claims for one fix."""
    fc = camera["config"]
    out = S.one_run((fc, "map_2018", cfg["runs"]["map_2018"], seed, True))
    result, flight = out["_result"], camera["flight"]
    east0, north0 = flight.metadata["origin_enu_m"]
    used = [f for f in result.fixes if f.used]
    return {"t": np.array([camera["t"][f.frame] for f in used]),
            "en": np.array([[f.position[1] + east0, f.position[0] + north0] for f in used]).reshape(-1, 2),
            "sigma_m": fc["navigator"]["fix_sigma_m"], "navigator": out}


def soft_update(f: ESKF, residual: np.ndarray, rows: np.ndarray, noise: np.ndarray, limit: float) -> bool:
    """An update that an outlier can neither be locked out of nor take over: if the residual is larger than the
    filter expects (its squared size against the expected spread above ``limit``), the measurement's noise is
    raised until it sits at the limit, and the update is made with that.

    Example: a camera speed 6 m/s off where 1 m/s is expected has a squared size of about 36; with a limit of 9 its
    variance is taken four times larger, so it pulls the state a little instead of a lot. Returns True if the
    measurement was within the limit as it came.
    """
    spread = rows @ f.P @ rows.T + noise
    size = float(residual @ np.linalg.solve(spread, residual))
    if not np.isfinite(size):
        return False
    inside = size <= limit
    if not inside:
        # raise the noise so that the squared size becomes the limit: solve by scaling the whole spread
        noise = noise + spread * (size / limit - 1.0)
    f.update(residual, rows, noise, None)
    return inside


def run_filter(rec: dict, eskf_cfg: dict, start_yaw_rad: float, gnss_until_s: float = math.inf, sample_times: np.ndarray | None = None,
               camera: dict | None = None, yaw_sigma_deg: float = 1.5, speed_sigma_mps: float = 1.0, gate_prob: float = 0.99,
               fixes: dict | None = None, motion_floor: float | None = 0.3, drift_rate: float = 0.10, reanchor_after: int = 3,
               speed_gate_prob: float | None = None, soft_limit: float | None = 9.21, use_speed: bool = True) -> dict:
    """Run the ESKF over one recording and return its state at ``sample_times`` (default: every tenth IMU sample).

    GNSS position fixes (and Alessandro's velocity fit over 8 s of fixes) are used up to ``gnss_until_s``.
    ``start_yaw_rad`` is the yaw at the start; the live adapter assumes 0, a heading sensor can give it.
    With ``camera`` (camera_measurements): at every frame the sun heading is fused as a yaw, and once GNSS is gone
    the camera's ground speed as a horizontal velocity; each update passes the filter's own gate or is left out.

    The camera's step is corrected for tilt first. A camera fixed to the drone looks at the ground a little away from
    the point below it: height times the tangent of the tilt. When the tilt changes between two frames that point
    moves, and the picture shifts although the drone did not. Example: at 100 m a tilt change of 1 degree moves it
    1.75 m, which over 0.2 s would read as 8.7 m/s. The filter knows its own tilt from the IMU and takes that shift out.

    A camera step shorter than ``motion_floor`` of the cruising step is not fused (the navigator's rule: the camera
    may have lost track). The navigator then assumes the drone flies on; here the IMU carries the state, and it can
    tell a drone that stopped to turn from a camera that sees nothing.

    ``fixes`` (navigator_fixes) are fused as horizontal positions, like Dan's RF fix: behind the gate, and after
    ``reanchor_after`` rejections in a row the horizontal position is reset to the fix. So that the gate means what
    it means in the navigator, the horizontal position's uncertainty also grows as the navigator's does: by
    ``drift_rate`` of the distance flown since the last fix (the filter's own grows far too slowly: it takes the
    camera's speed errors for random, and they are mostly a scale and a heading that are a little off).
    """
    gnss_cfg, baro_cfg = eskf_cfg["sim_gnss"], eskf_cfg["barometer"]
    n = gnss_cfg["attitude_init_samples"]
    k0 = settled_start(rec, n)
    t, accel, gyro = rec["imu_t"], rec["accel"], rec["gyro"]

    # the start, as in the adapter: tilt and biases from gravity at rest, position from the first GNSS fix
    mean_accel, mean_gyro = accel[k0 - n:k0].mean(axis=0), gyro[k0 - n:k0].mean(axis=0)
    up_body = mean_accel / np.linalg.norm(mean_accel)
    cross = np.cross(up_body, [0.0, 0.0, 1.0])
    angle = math.acos(float(np.clip(up_body[2], -1.0, 1.0)))
    level = Rotation.from_rotvec(cross / max(float(np.linalg.norm(cross)), 1e-12) * angle)
    attitude = Rotation.from_euler("z", start_yaw_rad) * level
    first_fix = int(np.searchsorted(rec["gnss_t"], t[k0], side="right")) - 1
    if first_fix < 0:
        raise ValueError("no GNSS fix before the filter starts")
    q = attitude.as_quat()
    f = ESKF(rec["gnss_enu"][first_fix], [0.0, 0.0, 0.0], [q[3], q[0], q[1], q[2]], [0.0, 0.0, -GRAVITY], "body",
             noise_model(eskf_cfg), ba0=mean_accel - level.inv().apply([0.0, 0.0, GRAVITY]), bg0=mean_gyro)

    gnss_cov = np.diag(np.square(gnss_cfg["fallback_sigma_m"]))
    next_fix = first_fix + 1
    history, last_fit = [], t[k0]
    baro_index, baro0, baro_ref = 0, None, None
    every = int(baro_cfg["every_n_samples"])
    if sample_times is None:
        sample_times = t[k0::10]
    out_t, out_p, out_v, out_yaw, out_sigma = [], [], [], [], []
    next_sample = int(np.searchsorted(sample_times, t[k0]))
    used = rejected = fits = 0
    next_frame = int(np.searchsorted(camera["t"], t[k0])) if camera else 0
    counts = {"yaw_used": 0, "yaw_rejected": 0, "speed_used": 0, "speed_rejected": 0, "speed_too_short": 0,
              "fixes_used": 0, "fixes_rejected": 0, "reanchored": 0}
    next_map_fix, rejected_in_row = 0, 0
    since_fix = 0.0  # metres flown since GNSS or the last map fix, by the filter's own estimate
    position_rows = np.zeros((2, 15))
    position_rows[:, P_.start:P_.start + 2] = np.eye(2)
    yaw_row = np.zeros((1, 15))
    yaw_row[0, TH.start + 2] = 1.0  # the attitude error is on the world side: its z part is the yaw error
    speed_rows = np.zeros((2, 15))
    speed_rows[:, V_.start:V_.start + 2] = np.eye(2)
    ground_z = float(f.p[2])  # the pad; the ground of these worlds is flat
    looked_at = None  # where the camera's axis met the ground at the frame before, east and north of the point below

    def camera_offset() -> np.ndarray:
        axis = f.R.apply([0.0, 0.0, -1.0])  # the down camera's axis in the world
        return max(float(f.p[2]) - ground_z, 0.0) * axis[:2] / max(abs(float(axis[2])), 1e-6)

    for k in range(k0 + 1, len(t)):
        dt = t[k] - t[k - 1]
        if dt <= 0:
            continue
        f.predict(accel[k - 1], accel[k], gyro[k - 1], gyro[k], dt)

        if (k - k0) % every == 0:  # barometer height, relative to the pressure at the first update
            baro_index = int(np.searchsorted(rec["baro_t"], t[k], side="right")) - 1
            if baro_index >= 0:
                pressure = max(float(rec["pressure"][baro_index]), 1.0)
                if baro0 is None:
                    baro0, baro_ref = pressure, float(f.p[2])
                dz = 8434.5 * math.log(baro0 / pressure)
                elapsed = t[k] - t[k0]
                sigma = math.sqrt(baro_cfg["white_std_m"] ** 2 + baro_cfg["bias_walk_m_per_sqrt_s"] ** 2 * elapsed
                                  + (baro_cfg["drift_sigma_m_per_s"] * elapsed) ** 2 + (baro_cfg["scale_error_rms"] * dz) ** 2)
                f.update_altitude(baro_ref + dz, np.array([0.0, 0.0, 1.0]), sigma, eskf_cfg["forward_camera"]["gate_prob"])

        while next_fix < len(rec["gnss_t"]) and rec["gnss_t"][next_fix] <= t[k]:
            stamp, position = float(rec["gnss_t"][next_fix]), rec["gnss_enu"][next_fix]
            next_fix += 1
            if stamp > gnss_until_s:
                continue
            result = f.update_position(position, gnss_cov, gnss_cfg["gate_prob"])
            used += int(result.accepted)
            rejected += int(not result.accepted)
            history.append((stamp, position.copy(), gnss_cov.copy()))
            window = gnss_cfg["velocity_window_s"]
            history = [h for h in history if stamp - h[0] <= max(20.0, 2.0 * window)]
            if stamp - last_fit >= window:
                samples = [h for h in history if max(stamp - window, last_fit + 1e-6) <= h[0] <= stamp]
                fit = fit_position_velocity(samples, window, gnss_cfg["velocity_min_samples"], gnss_cfg["velocity_min_span_s"],
                                            gnss_cfg["velocity_outlier_mahalanobis_sq"])
                if fit.get("reason") == "ok":
                    f.update_velocity(fit["velocity"], fit["covariance"], gnss_cfg["gate_prob"])
                    last_fit = stamp
                    fits += 1

        while camera is not None and next_frame < len(camera["t"]) and camera["t"][next_frame] <= t[k]:
            j = next_frame
            next_frame += 1
            innovation = (camera["yaw"][j] - yaw_of(f.R) + math.pi) % (2 * math.pi) - math.pi
            result = f.update(np.array([innovation]), yaw_row, np.array([[math.radians(yaw_sigma_deg) ** 2]]), gate_prob)
            counts["yaw_used" if result.accepted else "yaw_rejected"] += 1
            offset, frame_dt = camera_offset(), camera["t"][j] - camera["t"][j - 1] if j > 0 else math.nan
            if camera["t"][j] > gnss_until_s and looked_at is not None and frame_dt > 0 and use_speed:
                raw = camera["step_ne"][j][::-1]
                if motion_floor is not None and np.linalg.norm(raw) < motion_floor * camera["cruise_mps"] * frame_dt:
                    counts["speed_too_short"] += 1  # stopped, or lost track: the IMU decides
                else:
                    step_en = raw - (offset - looked_at)  # the camera's step without the tilt's shift
                    residual, noise = step_en / frame_dt - f.v[:2], np.eye(2) * speed_sigma_mps ** 2
                    if soft_limit is not None:
                        inside = soft_update(f, residual, speed_rows, noise, soft_limit)
                    else:
                        inside = f.update(residual, speed_rows, noise, speed_gate_prob).accepted
                    counts["speed_used" if inside else "speed_rejected"] += 1
                # the navigator's drift budget on the horizontal position: sigma grows by drift_rate of the distance
                flown = float(np.linalg.norm(f.v[:2])) * frame_dt
                f.P[P_.start:P_.start + 2, P_.start:P_.start + 2] += np.eye(2) * drift_rate ** 2 * ((since_fix + flown) ** 2 - since_fix ** 2)
                since_fix += flown
            looked_at = offset

            while fixes is not None and next_map_fix < len(fixes["t"]) and fixes["t"][next_map_fix] <= camera["t"][j]:
                en = fixes["en"][next_map_fix]
                next_map_fix += 1
                cov = np.eye(2) * fixes["sigma_m"] ** 2
                result = f.update(en - f.p[:2], position_rows, cov, gate_prob)
                rejected_in_row = 0 if result.accepted else rejected_in_row + 1
                counts["fixes_used" if result.accepted else "fixes_rejected"] += 1
                if rejected_in_row >= reanchor_after:  # as Dan's RF fix: the filter has drifted, the fixes are the reference
                    f.p[:2] = en
                    f.P[P_.start:P_.start + 2, :] = 0.0
                    f.P[:, P_.start:P_.start + 2] = 0.0
                    f.P[P_.start:P_.start + 2, P_.start:P_.start + 2] = cov
                    rejected_in_row = 0
                    counts["reanchored"] += 1
                if result.accepted or rejected_in_row == 0:
                    since_fix = 0.0

        while next_sample < len(sample_times) and sample_times[next_sample] <= t[k]:
            out_t.append(t[k])
            out_p.append(f.p.copy())
            out_v.append(f.v.copy())
            out_yaw.append(yaw_of(f.R))
            out_sigma.append(float(np.sqrt(np.max(np.linalg.eigvalsh(f.P[:2, :2])))))
            next_sample += 1

    return {"t": np.array(out_t), "p": np.array(out_p), "v": np.array(out_v), "yaw": np.array(out_yaw), "sigma": np.array(out_sigma),
            "start_s": float(t[k0]), "gnss_used": used, "gnss_rejected": rejected, "velocity_fits": fits, **counts}


def against_truth(rec: dict, run: dict) -> dict:
    """Horizontal error in metres and yaw error in degrees of a filter run, at its sample times."""
    truth = np.column_stack([np.interp(run["t"], rec["truth_t"], rec["truth_enu"][:, i]) for i in range(3)])
    nearest = np.clip(np.searchsorted(rec["truth_t"], run["t"]), 0, len(rec["truth_t"]) - 1)
    q = rec["truth_q"][nearest]
    true_yaw = np.array([yaw_of(Rotation.from_quat([x, y, z, w])) for w, x, y, z in q])
    yaw_error = np.degrees((run["yaw"] - true_yaw + np.pi) % (2 * np.pi) - np.pi)
    return {"horizontal": np.hypot(*(run["p"][:, :2] - truth[:, :2]).T), "height": run["p"][:, 2] - truth[:, 2], "yaw_deg": yaw_error}


def one_flight(job: tuple) -> dict:
    """Steps 3 and 4 for one development flight, camera and draw of the heading sensor, against the frozen navigator."""
    flight_name, camera_model, seed, heading_source, yaw_sigma_deg = job
    cfg = yaml.safe_load((REPO / "baseline" / "configs" / "sim_navigator.yaml").read_text())
    eskf_cfg = yaml.safe_load((REPO / "vio" / "configs" / "midair_eskf.yaml").read_text())
    rec = load_recording(REPO / "recordings" / flight_name)
    camera = camera_measurements(cfg, flight_name, camera_model, seed, heading_source)
    flight, frame_t, jam = camera["flight"], camera["t"], camera["calibration"].jam_index
    k0 = settled_start(rec, eskf_cfg["sim_gnss"]["attitude_init_samples"])
    w, x, y, z = rec["truth_q"][int(np.searchsorted(rec["truth_t"], rec["imu_t"][k0]))]
    start_yaw = yaw_of(Rotation.from_quat([x, y, z, w])) + math.radians(np.random.default_rng(seed).normal(0.0, 1.0))
    fixes = navigator_fixes(cfg, camera, seed)
    out = {"flight": flight_name, "camera": camera_model, "seed": seed, "fixes": len(fixes["t"]),
           "navigator": {k: fixes["navigator"][k] for k in ("median", "p90", "worst", "end", "used_but_wrong", "within_3_sigma")}}
    alone = flight.position_gt[jam] + np.cumsum(camera["step_ne"][jam + 1:], axis=0)
    e = np.linalg.norm(alone - flight.position_gt[jam + 1:], axis=1)
    out["camera_steps"] = {"median": float(np.median(e)), "p90": float(np.percentile(e, 90)), "worst": float(e.max())}
    for label, with_fixes in (("filter", None), ("filter_and_fixes", fixes)):
        run = run_filter(rec, eskf_cfg, start_yaw, gnss_until_s=camera["jam_s"], camera=camera, fixes=with_fixes, sample_times=frame_t,
                         yaw_sigma_deg=yaw_sigma_deg)
        scored = against_truth(rec, run)
        error = scored["horizontal"]
        after = run["t"] > camera["jam_s"]
        e, sigma = error[after], run["sigma"][after]
        out[label] = {"median": float(np.median(e)), "p90": float(np.percentile(e, 90)), "worst": float(e.max()), "end": float(e[-1]),
                      "within_3_sigma": float(np.mean(e <= 3.0 * sigma)), "fixes_used": run["fixes_used"],
                      "fixes_rejected": run["fixes_rejected"], "resets": run["reanchored"], "sigma_median": float(np.median(sigma)),
                      "yaw_rms_deg": float(np.sqrt(np.mean(scored["yaw_deg"][after] ** 2)))}
    return out


def all_flights(camera_model: str, workers: int, heading_source: str | None = None, yaw_sigma_deg: float = 1.5) -> int:
    cfg = yaml.safe_load((REPO / "baseline" / "configs" / "sim_navigator.yaml").read_text())
    jobs = [(name, camera_model, seed, heading_source, yaw_sigma_deg) for name in cfg["flights"]["development"] for seed in cfg["seeds"]]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(one_flight, jobs))
    (REPO / "outputs").mkdir(exist_ok=True)
    source = heading_source or cfg["heading"].get("source", "compass")
    (REPO / "outputs" / f"fused_replay_{camera_model}_{source}.json").write_text(json.dumps(rows, indent=1))
    print(f"development flights, camera {camera_model}, heading from {source}, GNSS lost after 450 m; medians over the draws {cfg['seeds']} (metres)")
    for name in cfg["flights"]["development"]:
        mine = [r for r in rows if r["flight"] == name]
        med = lambda part, key: float(np.median([r[part][key] for r in mine]))  # noqa: E731
        print(f"\n{name}")
        print(f"  the camera's steps summed              median {med('camera_steps', 'median'):6.1f}  90% {med('camera_steps', 'p90'):6.1f}  worst {med('camera_steps', 'worst'):6.1f}")
        print(f"  filter: IMU, barometer, sun, camera    median {med('filter', 'median'):6.1f}  90% {med('filter', 'p90'):6.1f}  worst {med('filter', 'worst'):6.1f}"
              f"  | within its 3 sigma {min(r['filter']['within_3_sigma'] for r in mine):.1%} (lowest draw) | heading error {med('filter', 'yaw_rms_deg'):.1f} deg rms")
        print(f"  the frozen navigator (camera and map)  median {med('navigator', 'median'):6.1f}  90% {med('navigator', 'p90'):6.1f}  worst {med('navigator', 'worst'):6.1f}"
              f"  | within its 3 sigma {min(r['navigator']['within_3_sigma'] for r in mine):.1%}; wrong fixes {[int(r['navigator']['used_but_wrong']) for r in mine]}")
        print(f"  filter with the navigator's map fixes  median {med('filter_and_fixes', 'median'):6.1f}  90% {med('filter_and_fixes', 'p90'):6.1f}  worst {med('filter_and_fixes', 'worst'):6.1f}"
              f"  | within its 3 sigma {min(r['filter_and_fixes']['within_3_sigma'] for r in mine):.1%}; fixes fused {[r['filter_and_fixes']['fixes_used'] for r in mine]}, "
              f"rejected {[r['filter_and_fixes']['fixes_rejected'] for r in mine]}, resets {[r['filter_and_fixes']['resets'] for r in mine]}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--flight", default="wufeng_corridor_100m", help="a development flight")
    p.add_argument("--cut", action="store_true", help="GNSS is lost after jam_after_m of the camera flight, as for the camera navigator")
    p.add_argument("--yaw-error-deg", type=float, default=1.0, help="error of the heading reading the filter starts with")
    p.add_argument("--camera", action="store_true", help="fuse the sun heading, and after the GNSS loss the down camera's ground speed")
    p.add_argument("--camera-model", default="ideal", help="ideal (the simulator's frames) or realistic")
    p.add_argument("--seed", type=int, default=1, help="draw of the heading sensor")
    p.add_argument("--speed-sigma", type=float, default=1.0, help="m/s, noise the filter assumes for the camera's ground speed")
    p.add_argument("--fixes", action="store_true", help="fuse the map fixes the frozen navigator used on this flight")
    p.add_argument("--no-floor", action="store_true", help="fuse every camera step, however short")
    p.add_argument("--speed-gate", type=float, default=None, help="gate probability for the camera's speed, e.g. 0.99; default: no gate")
    p.add_argument("--hard", action="store_true", help="use the gate (or none) for the camera's speed, not the soft update")
    p.add_argument("--all", action="store_true", help="steps 3 and 4 on every development flight and draw, against the frozen navigator")
    p.add_argument("--workers", type=int, default=5)
    p.add_argument("--heading-source", help="compass, sun_digital or sun_photodiode (default: as configured, the sun)")
    p.add_argument("--yaw-sigma", type=float, default=1.5, help="degrees, noise the filter assumes for the heading sensor")
    p.add_argument("--no-speed", action="store_true", help="fuse the heading only, not the camera's ground speed")
    args = p.parse_args()
    if args.all:
        return all_flights(args.camera_model, args.workers, args.heading_source, args.yaw_sigma)

    cfg = yaml.safe_load((REPO / "baseline" / "configs" / "sim_navigator.yaml").read_text())
    if args.flight not in cfg["flights"]["development"]:
        raise SystemExit(f"{args.flight} is not a development flight; the sealed and held-out flights are not run here")
    eskf_cfg = yaml.safe_load((REPO / "vio" / "configs" / "midair_eskf.yaml").read_text())
    rec = load_recording(REPO / "recordings" / args.flight)

    fc = flight_cfg(cfg, args.flight, cfg["flights"]["development"][args.flight], "ideal", {})
    flight = S.flight_for(fc, "2018")
    frame_t = flight.metadata["recording_t_s"]
    jam = int(np.searchsorted(flight.travelled, cfg["navigator"]["jam_after_m"]))
    jam_s = float(frame_t[jam])

    # the heading the filter starts with: the true one on the pad, read with an error (a sun sensor's reading)
    k0 = settled_start(rec, eskf_cfg["sim_gnss"]["attitude_init_samples"])
    w, x, y, z = rec["truth_q"][int(np.searchsorted(rec["truth_t"], rec["imu_t"][k0]))]
    start_yaw = yaw_of(Rotation.from_quat([x, y, z, w])) + math.radians(args.yaw_error_deg)

    camera = camera_measurements(cfg, args.flight, args.camera_model, args.seed, args.heading_source) if args.camera else None
    fixes = navigator_fixes(cfg, camera, args.seed) if args.fixes and camera else None
    run = run_filter(rec, eskf_cfg, start_yaw, gnss_until_s=jam_s if args.cut else math.inf, camera=camera,
                     speed_sigma_mps=args.speed_sigma, fixes=fixes, motion_floor=None if args.no_floor else 0.3,
                     speed_gate_prob=args.speed_gate, soft_limit=None if args.hard else 9.21, use_speed=not args.no_speed,
                     yaw_sigma_deg=args.yaw_sigma)
    error = against_truth(rec, run)
    print(f"{args.flight}: filter started at {run['start_s']:.1f} s; the camera flight runs from {frame_t[0]:.0f} to {frame_t[-1]:.0f} s; "
          f"GNSS {'lost at ' + format(jam_s, '.0f') + ' s' if args.cut else 'all the way'}; "
          f"{run['gnss_used']} fixes used, {run['gnss_rejected']} rejected, {run['velocity_fits']} velocity fits")
    spans = [("on the pad and climbing", run["t"] < frame_t[0]), ("with GNSS, flying", (run["t"] >= frame_t[0]) & (run["t"] <= jam_s)),
             ("after the 450 m mark", (run["t"] > jam_s) & (run["t"] <= frame_t[-1]))]
    for label, m in spans:
        if not m.any():
            continue
        e = error["horizontal"][m]
        print(f"  {label:26s} horizontal error median {np.median(e):8.1f} m, 90% {np.percentile(e, 90):8.1f}, worst {e.max():8.1f}, at the end {e[-1]:8.1f} | "
              f"height {np.sqrt(np.mean(error['height'][m] ** 2)):5.1f} m rms | yaw {np.sqrt(np.mean(error['yaw_deg'][m] ** 2)):5.1f} deg rms | "
              f"filter's own sigma at the end {run['sigma'][m][-1]:7.1f} m")
    if camera is not None:
        print(f"  camera updates: yaw {run['yaw_used']} used, {run['yaw_rejected']} rejected; ground speed {run['speed_used']} as they came, "
              f"{run['speed_rejected']} weakened or rejected, {run['speed_too_short']} too short to fuse")
        if fixes is not None:
            nav = fixes["navigator"]
            print(f"  map fixes from the frozen navigator: {run['fixes_used']} fused, {run['fixes_rejected']} rejected by the filter's gate, "
                  f"{run['reanchored']} resets | the frozen navigator itself on this draw: median {nav['median']:.1f} m, 90% {nav['p90']:.1f}, "
                  f"worst {nav['worst']:.1f}, at the end {nav['end']:.1f}")
        # the camera's own dead reckoning from the 450 m mark, summing its steps and nothing else, for comparison
        k = camera["calibration"].jam_index
        alone = flight.position_gt[k] + np.cumsum(camera["step_ne"][k + 1:], axis=0)
        e = np.linalg.norm(alone - flight.position_gt[k + 1:], axis=1)
        print(f"  the camera's steps summed, no filter: median {np.median(e):.1f} m, 90% {np.percentile(e, 90):.1f}, worst {e.max():.1f}, at the end {e[-1]:.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

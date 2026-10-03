"""Generate a simulated, multi-sensor replay over real Wufeng orthophotography.

The 2020 orthophoto supplies stabilized-nadir camera pixels; the 2018 orthophoto
is used only when finding the shared valid-area mask. Ground is treated as flat at 30 m MSL
(INFERENCE). No guidance or control outputs are produced.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import rasterio
from pyproj import Transformer
from rasterio.enums import Resampling
from rasterio.transform import from_origin
from rasterio.vrt import WarpedVRT
from rasterio.windows import Window
from scipy import ndimage
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

from t_replay import SCHEMA, isa_altitude_m

CAMERA_RASTER = Path("data/raw/aerial/wufeng_2020-03-23_x4.tif")
MAP_RASTER = Path("data/raw/aerial/wufeng_2018-05-03_x4.tif")
OUT_ROOT = Path("data/processed/t_replay")
GROUND_MSL_M = 30.0  # INFERENCE: flat ground and constant MSL height.
INTERNAL_HZ = 200
GRAVITY_M_S2 = 9.80665
BARO_P0_PA = 101325.0
BARO_ISA_EXPONENT = 1.0 / 0.190263


def load_shared_valid_mask(res_m: float = 4.0):
    """Read both RGB rasters at 4 m/px and return their eroded overlap mask."""
    with rasterio.open(CAMERA_RASTER) as camera, rasterio.open(MAP_RASTER) as map_raster:
        left = max(camera.bounds.left, map_raster.bounds.left)
        right = min(camera.bounds.right, map_raster.bounds.right)
        bottom = max(camera.bounds.bottom, map_raster.bounds.bottom)
        top = min(camera.bounds.top, map_raster.bounds.top)
        width = int(math.ceil((right - left) / res_m))
        height = int(math.ceil((top - bottom) / res_m))
        transform = from_origin(left, top, res_m, res_m)
        masks = []
        for source in (camera, map_raster):
            with WarpedVRT(source, crs="EPSG:3826", transform=transform,
                           width=width, height=height, resampling=Resampling.average) as vrt:
                rgb = vrt.read(out_shape=(3, height, width))
            valid = np.all(rgb > 0, axis=0)
            masks.append(ndimage.binary_erosion(valid, iterations=2))
        common = masks[0] & masks[1]
        if not np.any(common):
            raise RuntimeError("the two Wufeng rasters have no shared valid pixels")
    return common, left, top, res_m


def fit_corridor_axis(mask: np.ndarray, left: float, top: float, res_m: float):
    rows, cols = np.where(mask)
    xy = np.column_stack((left + (cols + 0.5) * res_m,
                          top - (rows + 0.5) * res_m))
    center = xy.mean(axis=0)
    _, vectors = np.linalg.eigh(np.cov((xy - center).T))
    axis = vectors[:, -1]
    if axis[np.argmax(np.abs(axis))] < 0:
        axis = -axis
    normal = np.array([-axis[1], axis[0]])
    center += normal * np.median((xy - center) @ normal)
    along = (xy - center) @ axis
    lateral = (xy - center) @ normal
    s_min, s_max = float(along.min()), float(along.max())
    bin_width = 20.0
    edges = np.arange(s_min, s_max + bin_width, bin_width)
    if edges[-1] < s_max:
        edges = np.append(edges, edges[-1] + bin_width)
    centers = 0.5 * (edges[:-1] + edges[1:])
    bin_ids = np.clip(np.searchsorted(edges, along, side="right") - 1, 0, len(centers) - 1)
    profile = np.full(len(centers), np.nan)
    for i in range(len(centers)):
        values = lateral[bin_ids == i]
        if len(values):
            profile[i] = np.median(values)
    known = np.isfinite(profile)
    profile = np.interp(centers, centers[known], profile[known])
    profile = ndimage.gaussian_filter1d(profile, sigma=3.0, mode="nearest")
    return center, axis, normal, s_min, s_max, centers, profile


def make_turn(radius_m: float, ramp_m: float, ds_m: float):
    """Make a half-turn with zero curvature at both joins and min radius R."""
    length = math.pi * radius_m + ramp_m
    s = np.arange(0.0, length, ds_m)
    s = np.append(s, length)
    k = np.zeros_like(s)
    first = s < ramp_m
    k[first] = (s[first] / ramp_m) ** 2 * (3.0 - 2.0 * s[first] / ramp_m) / radius_m
    last = s > length - ramp_m
    z = (length - s[last]) / ramp_m
    k[last] = z**2 * (3.0 - 2.0 * z) / radius_m
    middle = ~(first | last)
    k[middle] = 1.0 / radius_m
    ds = np.diff(s)
    heading = np.zeros_like(s)
    heading[1:] = np.cumsum(0.5 * (k[:-1] + k[1:]) * ds)
    x = np.zeros_like(s)
    y = np.zeros_like(s)
    x[1:] = np.cumsum(np.cos(0.5 * (heading[:-1] + heading[1:])) * ds)
    y[1:] = np.cumsum(np.sin(0.5 * (heading[:-1] + heading[1:])) * ds)
    # Symmetry makes x_end zero; remove only numerical integration residue.
    x -= (s / length) * x[-1]
    return np.column_stack((x, y)), s, k


def build_route(center: np.ndarray, axis: np.ndarray, normal: np.ndarray,
                profile_s: np.ndarray, profile_l: np.ndarray,
                start_s: float, end_s: float, speed_m_s: float,
                dt_s: float):
    ds_m = speed_m_s * dt_s
    turn_xy, turn_s, turn_k = make_turn(60.0, 30.0, ds_m)
    separation = float(turn_xy[-1, 1])
    half_sep = separation / 2.0

    profile_spline = CubicSpline(profile_s, profile_l, bc_type="natural")

    def centerline(s):
        s = np.asarray(s, dtype=float)
        lateral = profile_spline(s)
        return center + s[..., None] * axis + lateral[..., None] * normal

    def lane(s, side):
        s = np.asarray(s, dtype=float)
        c = centerline(s)
        before, after = centerline(s - 1.0), centerline(s + 1.0)
        tangent = after - before
        tangent /= np.linalg.norm(tangent, axis=-1)[..., None]
        local_normal = np.stack((-tangent[..., 1], tangent[..., 0]), axis=-1)
        return c + side * half_sep * local_normal

    def lane_tangent(s, side):
        tangent = lane(np.array([s - 1.0, s + 1.0]), side)
        result = tangent[1] - tangent[0]
        return result / np.linalg.norm(result)

    def line(a: float, b: float, side: float):
        count = max(1, int(math.ceil(abs(b - a) / ds_m)))
        return lane(np.linspace(a, b, count + 1), side)

    def half_turn(s: float, side_start: float, side_end: float, heading: np.ndarray):
        left = np.array([-heading[1], heading[0]])
        start = lane(s, side_start)
        points = start + turn_xy[:, :1] * heading + turn_xy[:, 1:] * left
        target = lane(s, side_end)
        u = turn_s / turn_s[-1]
        smooth = u * u * (3.0 - 2.0 * u)
        points += smooth[:, None] * (target - points[-1])
        return points

    out = line(start_s, end_s, -1.0)
    far_turn = half_turn(end_s, -1.0, 1.0, lane_tangent(end_s, -1.0))
    back = line(end_s, start_s, 1.0)
    near_turn = half_turn(start_s, 1.0, -1.0, -lane_tangent(start_s, 1.0))
    points = np.vstack((out, far_turn[1:], back[1:], near_turn[1:]))
    arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))
    duration = float(arc[-1] / speed_m_s)
    times = np.arange(0.0, duration, dt_s)
    distance = times * speed_m_s
    xy = np.column_stack((np.interp(distance, arc, points[:, 0]),
                          np.interp(distance, arc, points[:, 1])))
    return xy, times, duration, float(arc[-1]), turn_s, turn_k




def ecef_enu_rotation(lat_deg: float, lon_deg: float) -> np.ndarray:
    lat, lon = np.radians([lat_deg, lon_deg])
    slat, clat, slon, clon = np.sin(lat), np.cos(lat), np.sin(lon), np.cos(lon)
    return np.array([[-slon, clon, 0.0],
                     [-slat * clon, -slat * slon, clat],
                     [clat * clon, clat * slon, slat]])


def projected_to_enu(xy: np.ndarray, altitude_m: np.ndarray | float):
    to_geo = Transformer.from_crs("EPSG:3826", "EPSG:4326", always_xy=True)
    to_ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
    lon, lat = to_geo.transform(xy[:, 0], xy[:, 1])
    alt = np.broadcast_to(np.asarray(altitude_m, dtype=float), len(xy))
    ex, ey, ez = to_ecef.transform(lon, lat, alt)
    ecef = np.column_stack((ex, ey, ez))
    r_enu = ecef_enu_rotation(float(lat[0]), float(lon[0]))
    enu = (ecef - ecef[0]) @ r_enu.T
    return enu, np.asarray(lat), np.asarray(lon), ecef, r_enu


def kinematics(path_xy: np.ndarray, times: np.ndarray, altitude_m: float):
    dt = 1.0 / INTERNAL_HZ
    enu, lat, lon, ecef, r_enu = projected_to_enu(path_xy, altitude_m)
    velocity = np.gradient(enu, dt, axis=0, edge_order=2)
    acceleration = np.gradient(velocity, dt, axis=0, edge_order=2)
    yaw = np.unwrap(np.arctan2(velocity[:, 1], velocity[:, 0]))
    camera_rotations = Rotation.from_euler("Z", yaw[:, None])

    b3 = np.column_stack((acceleration[:, 0], acceleration[:, 1],
                          np.full(len(times), GRAVITY_M_S2)))
    b3 /= np.linalg.norm(b3, axis=1)[:, None]
    c, s = np.cos(yaw), np.sin(yaw)
    bx = c * b3[:, 0] + s * b3[:, 1]
    by = -s * b3[:, 0] + c * b3[:, 1]
    pitch = np.arctan2(bx, b3[:, 2])
    roll = np.arcsin(np.clip(-by, -1.0, 1.0))
    wobble = np.radians(1.0) * np.sin(2.0 * np.pi * 0.7 * times)
    roll = roll + wobble
    pitch = pitch + np.radians(1.0) * np.sin(2.0 * np.pi * 0.7 * times + np.pi / 2.0)
    rotations = Rotation.from_euler("ZYX", np.column_stack((yaw, pitch, roll)))
    quat_xyzw = rotations.as_quat()
    if np.any(np.sum(quat_xyzw[1:] * quat_xyzw[:-1], axis=1) < 0):
        for i in range(1, len(quat_xyzw)):
            if np.dot(quat_xyzw[i], quat_xyzw[i - 1]) < 0:
                quat_xyzw[i] *= -1
        rotations = Rotation.from_quat(quat_xyzw)

    body_rate = np.empty((len(times), 3), dtype=float)
    body_rate[0] = (rotations[0].inv() * rotations[1]).as_rotvec() / dt
    body_rate[-1] = (rotations[-2].inv() * rotations[-1]).as_rotvec() / dt
    body_rate[1:-1] = (rotations[:-2].inv() * rotations[2:]).as_rotvec() / (2.0 * dt)
    specific_force = rotations.inv().apply(
        np.column_stack((acceleration[:, 0], acceleration[:, 1],
                         acceleration[:, 2] + GRAVITY_M_S2)))
    return {"enu": enu, "lat": lat, "lon": lon, "ecef": ecef,
            "r_enu": r_enu, "velocity": velocity, "acceleration": acceleration,
            "rotations": rotations, "camera_rotations": camera_rotations,
            "quat_xyzw": quat_xyzw, "body_rate": body_rate,
            "specific_force": specific_force}


def camera_ground_grid(xy_map: np.ndarray, rotation: Rotation, agl_m: float,
                       width: int, height: int, fx: float, fy: float,
                       grid_x: np.ndarray, grid_y: np.ndarray):
    xx, yy = np.meshgrid(grid_x, grid_y)
    # A yaw-only gimbal keeps this camera nadir while image top follows track.
    rays_body = np.column_stack((-(yy.ravel() - (height - 1) / 2.0) / fy,
                                 -(xx.ravel() - (width - 1) / 2.0) / fx,
                                 -np.ones(xx.size)))
    rays_enu = rotation.apply(rays_body)
    if np.any(rays_enu[:, 2] >= -1e-8):
        return None
    scale = -agl_m / rays_enu[:, 2]
    return xy_map + rays_enu[:, :2] * scale[:, None]


def camera_frame_safe(path_xy: np.ndarray, times: np.ndarray, poses: dict,
                      mask: np.ndarray, grid_left: float, grid_top: float,
                      grid_res: float, agl_m: float, image_rate_hz: float):
    width, height = 640, 480
    fx = width / (2.0 * math.tan(math.radians(70.0) / 2.0))
    fy = fx
    gx, gy = np.linspace(0, width - 1, 13), np.linspace(0, height - 1, 11)
    frame_times = np.arange(0.0, times[-1] + 1e-9, 1.0 / image_rate_hz)
    indices = np.clip(np.rint(frame_times * INTERNAL_HZ).astype(int), 0, len(times) - 1)
    for i in indices:
        ground = camera_ground_grid(path_xy[i], poses["camera_rotations"][i], agl_m,
                                    width, height, fx, fy, gx, gy)
        if ground is None:
            return False
        cols = np.floor((ground[:, 0] - grid_left) / grid_res).astype(int)
        rows = np.floor((grid_top - ground[:, 1]) / grid_res).astype(int)
        if (np.any(cols < 0) or np.any(rows < 0) or
                np.any(cols >= mask.shape[1]) or np.any(rows >= mask.shape[0]) or
                not np.all(mask[rows, cols])):
            return False
    return True


def choose_safe_route(mask: np.ndarray, grid_left: float, grid_top: float,
                      grid_res: float, speed: float, agl_m: float,
                      image_rate_hz: float, duration_cap_s: float):
    center, axis, normal, s_min, s_max, profile_s, profile_l = fit_corridor_axis(
        mask, grid_left, grid_top, grid_res)
    width, height = 640, 480
    fx = width / (2.0 * math.tan(math.radians(70.0) / 2.0))
    corner_angle = math.atan(math.hypot((width / 2.0) / fx, (height / 2.0) / fx))
    max_tilt = math.atan(speed * speed / (60.0 * GRAVITY_M_S2)) + math.radians(2.0)
    initial_margin = agl_m * math.tan(corner_angle + max_tilt) + 12.0
    lateral_shifts = [0.0, -20.0, 20.0, -40.0, 40.0, -60.0, 60.0]
    last_reason = ""
    for shift in lateral_shifts:
        shifted_center = center + shift * normal
        for extra_margin in range(0, 241, 20):
            margin = initial_margin + extra_margin
            start_s, end_s = s_min + margin, s_max - margin
            if end_s - start_s < 600.0:
                continue
            path_xy, times, full_duration, path_length, _, _ = build_route(
                shifted_center, axis, normal, profile_s, profile_l,
                start_s, end_s, speed, 1.0 / INTERNAL_HZ)
            run_duration = min(full_duration, duration_cap_s)
            keep = int(math.floor(run_duration * INTERNAL_HZ)) + 1
            path_xy, times = path_xy[:keep], times[:keep]
            poses = kinematics(path_xy, times, GROUND_MSL_M + agl_m)
            if camera_frame_safe(path_xy, times, poses, mask, grid_left, grid_top,
                                 grid_res, agl_m, image_rate_hz):
                return path_xy, times, poses, path_length, full_duration, shifted_center, axis, normal
            last_reason = f"unsafe image footprint (cross-track shift {shift:.0f} m, margin {margin:.0f} m)"
    raise RuntimeError(f"no full-footprint-safe path found in shared mask: {last_reason}")


def gm_noise(rng: np.random.Generator, n: int, dt: float, sigma: float, tau_s: float):
    result = np.zeros((n, 3), dtype=float)
    decay = math.exp(-dt / tau_s)
    innovation = sigma * math.sqrt(1.0 - decay * decay)
    for i in range(1, n):
        result[i] = decay * result[i - 1] + innovation * rng.standard_normal(3)
    return result


def sample_indices(duration_s: float, rate_hz: float):
    count = int(math.floor(duration_s * rate_hz + 1e-9)) + 1
    times = np.arange(count, dtype=float) / rate_hz
    indices = np.rint(times * INTERNAL_HZ).astype(int)
    return times, indices


def write_sequence(args, path_xy, times, poses, path_length, full_duration):
    duration_s = float(times[-1])
    out = Path(args.out) if args.out else OUT_ROOT / f"wufeng_sim_{args.variant}"
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"refusing to overwrite non-empty output directory: {out}")
    out.mkdir(parents=True, exist_ok=True)
    (out / "images" / "cam0").mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(args.seed)
    scale = args.noise
    dt_imu = 1.0 / 100.0
    imu_t, imu_i = sample_indices(duration_s, 100.0)
    gyro = poses["body_rate"][imu_i].copy()
    accel = poses["specific_force"][imu_i].copy()
    gyro_walk = np.zeros_like(gyro)
    gyro_walk[1:] = np.cumsum(
        rng.normal(0.0, 2e-5 * math.sqrt(dt_imu), (len(gyro) - 1, 3)), axis=0)
    accel_walk = np.zeros_like(accel)
    accel_walk[1:] = np.cumsum(
        rng.normal(0.0, 5e-4 * math.sqrt(dt_imu), (len(accel) - 1, 3)), axis=0)
    gyro += scale * (rng.normal(0.0, 0.005, 3)[None, :] + gyro_walk)
    gyro += scale * rng.normal(0.0, 0.003, gyro.shape)
    accel += scale * (rng.normal(0.0, 0.05, 3)[None, :] + accel_walk)
    accel += scale * rng.normal(0.0, 0.05, accel.shape)
    pd.DataFrame({"t_s": imu_t, "gx": gyro[:, 0], "gy": gyro[:, 1], "gz": gyro[:, 2],
                  "ax": accel[:, 0], "ay": accel[:, 1], "az": accel[:, 2]}).to_csv(
        out / "imu.csv", index=False, float_format="%.8f")

    baro_t, baro_i = sample_indices(duration_s, 25.0)
    true_alt = GROUND_MSL_M + args.agl
    true_pressure = BARO_P0_PA * (1.0 - true_alt / 44330.769) ** BARO_ISA_EXPONENT
    baro_drift = np.zeros(len(baro_t), dtype=float)
    for i in range(1, len(baro_t)):
        baro_drift[i] = baro_drift[i - 1] + rng.normal(0.0, 0.9 * math.sqrt(1.0 / 25.0))
    pressure = true_pressure + scale * (rng.normal(0.0, 8.0, len(baro_t)) + baro_drift)
    baro_alt = isa_altitude_m(pressure, p0=BARO_P0_PA)
    pd.DataFrame({"t_s": baro_t, "pressure_pa": pressure,
                  "temperature_c": [""] * len(baro_t), "alt_isa_m": baro_alt}).to_csv(
        out / "baro.csv", index=False, float_format="%.8f")

    gnss_t, gnss_i = sample_indices(duration_s, 5.0)
    gnss_dt = 1.0 / 5.0
    horiz_gm = gm_noise(rng, len(gnss_t), gnss_dt, 1.2, 60.0)
    position_error = np.column_stack((rng.normal(0.0, 0.8, (len(gnss_t), 2)) + horiz_gm[:, :2],
                                      rng.normal(0.0, 2.5, len(gnss_t))))
    noisy_enu = poses["enu"][gnss_i] + scale * position_error
    origin_ecef = poses["ecef"][0]
    noisy_ecef = origin_ecef + noisy_enu @ poses["r_enu"]
    to_geo3 = Transformer.from_crs("EPSG:4978", "EPSG:4979", always_xy=True)
    gnss_lon, gnss_lat, _ = to_geo3.transform(noisy_ecef[:, 0], noisy_ecef[:, 1], noisy_ecef[:, 2])
    velocity = poses["velocity"][gnss_i] + scale * rng.normal(0.0, 0.1, (len(gnss_t), 3))
    gnss_alt = true_alt + scale * position_error[:, 2]
    pd.DataFrame({"t_s": gnss_t, "lat_deg": gnss_lat, "lon_deg": gnss_lon,
                  "alt_m": gnss_alt, "fix_type": 3, "hacc_m": 1.5, "vacc_m": 3.0,
                  "ve_mps": velocity[:, 0], "vn_mps": velocity[:, 1], "vu_mps": velocity[:, 2],
                  "nsat": 12}).to_csv(out / "gnss.csv", index=False, float_format="%.8f")

    truth_t, truth_i = sample_indices(duration_s, 20.0)
    q = poses["quat_xyzw"][truth_i]
    pd.DataFrame({"t_s": truth_t, "e_m": poses["enu"][truth_i, 0],
                  "n_m": poses["enu"][truth_i, 1], "u_m": poses["enu"][truth_i, 2],
                  "lat_deg": poses["lat"][truth_i], "lon_deg": poses["lon"][truth_i],
                  "alt_m": true_alt, "qw": q[:, 3], "qx": q[:, 0],
                  "qy": q[:, 1], "qz": q[:, 2]}).to_csv(
        out / "truth.csv", index=False, float_format="%.8f")

    camera_t, camera_i = sample_indices(duration_s, args.image_rate)
    with rasterio.open(CAMERA_RASTER) as raster:
        fx = 640 / (2.0 * math.tan(math.radians(70.0) / 2.0))
        fy = fx
        decimation = max(1, int(round((args.agl / fx) / raster.res[0])))
        image_rows = []
        for frame_id, (t, idx) in enumerate(zip(camera_t, camera_i)):
            name = f"{frame_id:06d}.jpg"
            destination = out / "images" / "cam0" / name
            image = render_frame(raster, path_xy[idx], poses["camera_rotations"][idx],
                                 args.agl, fx, fy, decimation, args.blur)
            if not cv2.imwrite(str(destination), image, [cv2.IMWRITE_JPEG_QUALITY, 90]):
                raise OSError(f"could not write camera frame {destination}")
            image_rows.append((t, "cam0", f"images/cam0/{name}"))
    pd.DataFrame(image_rows, columns=("t_s", "cam", "path")).to_csv(
        out / "images.csv", index=False, float_format="%.8f")

    start_lat = float(poses["lat"][0])
    start_lon = float(poses["lon"][0])
    meta = {
        "schema": SCHEMA,
        "sequence": out.name,
        "evidence_label": "SIMULATED",
        "source": ("OpenAerialMap Wufeng, Taiwan orthophotography: 2020-03-23 camera source; "
                   "2018-05-03 image used only to bound the shared valid area"),
        "licence": "imagery CC BY 4.0 OpenAerialMap; generated data: team",
        "origin": {"lat_deg": start_lat, "lon_deg": start_lon, "alt_m": true_alt,
                   "crs": "EPSG:3826", "note": "first trajectory point; ENU tangent plane at path start; "
                   "u_m is relative to the start altitude MSL"},
        "map": {"path": str(MAP_RASTER), "crs": "EPSG:3826",
                "note": "different year from camera source; used for common-valid-area masking only"},
        "platform": "simulated multirotor; FLU body frame",
        "altitude_semantics": "simulated raw pressure (ISA of true MSL altitude + declared noise and drift)",
        "gnss_cut_s": 60.0,
        "trajectory": {"path_length_m": path_length, "duration_s": duration_s,
                       "full_loop_duration_s": full_duration, "speed_m_s": args.speed,
                       "agl_m": args.agl, "h_ground_msl_m": GROUND_MSL_M,
                       "h_ground_assumption": "flat ground, constant 30 m MSL (INFERENCE)",
                       "attitude_wobble_deg": 1.0, "attitude_wobble_hz": 0.7,
                       "turn_radius_m": 60.0},
        "generator": {"script": "experiments/t_gen_wufeng_replay.py",
                      "args": {"variant": args.variant, "speed": args.speed,
                               "agl": args.agl, "duration_cap": args.duration_cap,
                               "image_rate": args.image_rate, "noise": args.noise,
                               "blur": args.blur, "out": str(out)}, "seed": args.seed},
        "sensors": {
            "imu": {"rate_hz": 100, "file": "imu.csv", "frame": "body FLU",
                    "provenance": "simulated at 200 Hz from trajectory truth; sampled at 100 Hz",
                    "noise": {"scale": scale,
                              "gyro_white_std_rad_s_per_sample": 0.003,
                              "gyro_initial_bias_std_rad_s": 0.005,
                              "gyro_bias_random_walk_rad_s_sqrt_s": 2e-5,
                              "accel_white_std_m_s2_per_sample": 0.05,
                              "accel_initial_bias_std_m_s2": 0.05,
                              "accel_bias_random_walk_m_s2_sqrt_s": 5e-4,
                              "bias_model": "independent 3-axis Gaussian initial bias plus random walk"}},
            "baro": {"rate_hz": 25, "file": "baro.csv", "p0_pa": BARO_P0_PA,
                     "altitude_semantics": "simulated raw pressure (ISA of true MSL altitude + declared noise and drift)",
                     "noise": {"scale": scale, "white_std_pa": 8.0,
                               "random_walk_drift_pa_sqrt_s": 0.9,
                               "drift_note": "chosen for 600 s endpoint drift std ~22 Pa (~1.9 m), comparable to measured Zurich median 2.5 m at 600 s"}},
            "gnss": {"rate_hz": 5, "file": "gnss.csv", "alt_ref": "msl",
                     "noise": {"scale": scale, "horizontal_white_std_m_per_axis": 0.8,
                               "horizontal_gauss_markov_sigma_m_per_axis": 1.2,
                               "vertical_white_std_m": 2.5, "gauss_markov_tau_s": 60.0,
                               "velocity_white_std_m_s_per_axis": 0.1},
                     "fix_type": 3, "hacc_m": 1.5, "vacc_m": 3.0, "nsat": 12},
            "camera": {"file": "images.csv", "rate_hz": args.image_rate,
                       "cams": {"cam0": {"width": 640, "height": 480,
                                          "horizontal_fov_deg": 70.0,
                                          "direction": "gimbal-stabilized nadir; image top follows track",
                                          "stabilization": "pitch/roll held at nadir; yaw follows track",
                                          "rendering": "real 2020 orthophoto (OpenAerialMap, CC BY 4.0), rendered by planar homography: no 3D parallax, no relief, no lighting change, same capture date for all frames",
                                          "jpeg_quality": 90, "motion_blur": bool(args.blur)}}},
            "truth": {"file": "truth.csv", "rate_hz": 20, "evaluator_only": True,
                      "source": "simulated 200 Hz trajectory sampled at 20 Hz",
                      "orientation": "quaternion body FLU -> ENU; columns qw,qx,qy,qz"},
        },
        "counts": {"imu": len(imu_t), "baro": len(baro_t), "gnss": len(gnss_t),
                   "images": len(camera_t), "truth": len(truth_t)},
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(f"{out}: path={path_length:.1f} m duration={duration_s:.2f} s "
          f"frames={len(camera_t)} ({args.image_rate:g} Hz) seed={args.seed}")


def render_frame(raster, center_xy: np.ndarray, rotation: Rotation, agl_m: float,
                 fx: float, fy: float, decimation: int, blur: bool):
    width, height = 640, 480
    pixels = np.array([[0.0, 0.0], [width - 1.0, 0.0],
                       [width - 1.0, height - 1.0], [0.0, height - 1.0]])
    body_rays = np.column_stack((-(pixels[:, 1] - (height - 1) / 2.0) / fy,
                                 -(pixels[:, 0] - (width - 1) / 2.0) / fx,
                                 -np.ones(4)))
    rays = rotation.apply(body_rays)
    if np.any(rays[:, 2] >= -1e-8):
        raise RuntimeError("camera ray does not intersect the ground plane")
    ground_xy = center_xy + rays[:, :2] * (-agl_m / rays[:, 2])[:, None]
    map_to_pixel = ~raster.transform
    source_corners = np.array([map_to_pixel * tuple(point) for point in ground_xy], dtype=np.float32)
    pad = max(8, decimation * 4)
    col0 = max(0, int(math.floor(source_corners[:, 0].min())) - pad)
    row0 = max(0, int(math.floor(source_corners[:, 1].min())) - pad)
    col1 = min(raster.width, int(math.ceil(source_corners[:, 0].max())) + pad + 1)
    row1 = min(raster.height, int(math.ceil(source_corners[:, 1].max())) + pad + 1)
    if col1 <= col0 or row1 <= row0:
        raise RuntimeError("camera footprint falls outside the 2020 orthophoto")
    window = Window(col0, row0, col1 - col0, row1 - row0)
    out_w = max(1, int(math.ceil(window.width / decimation)))
    out_h = max(1, int(math.ceil(window.height / decimation)))
    rgb = raster.read(window=window, out_shape=(3, out_h, out_w), resampling=Resampling.bilinear)
    source_corners[:, 0] = (source_corners[:, 0] - col0) * out_w / window.width
    source_corners[:, 1] = (source_corners[:, 1] - row0) * out_h / window.height
    homography = cv2.getPerspectiveTransform(source_corners,
                                            pixels.astype(np.float32))
    bgr = np.ascontiguousarray(rgb[::-1].transpose(1, 2, 0))
    image = cv2.warpPerspective(bgr, homography, (width, height),
                                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                                borderValue=(0, 0, 0))
    if blur:
        kernel = np.zeros((5, 5), dtype=np.float32)
        kernel[:, 2] = 1.0 / 5.0
        image = cv2.filter2D(image, -1, kernel)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", default="base")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--speed", type=float, default=12.0, help="constant speed in m/s")
    parser.add_argument("--agl", type=float, default=120.0, help="altitude above ground in m")
    parser.add_argument("--duration-cap", type=float, default=600.0, help="maximum flight duration in s")
    parser.add_argument("--image-rate", type=float, default=5.0, help="camera frames per second")
    parser.add_argument("--noise", type=float, default=1.0, help="scale all declared sensor noise (0 for noise-free sensors)")
    parser.add_argument("--out", type=Path, help="override data/processed/t_replay/wufeng_sim_<variant>")
    parser.add_argument("--blur", action="store_true", help="apply a short directional camera motion-blur kernel")
    args = parser.parse_args()
    if not args.variant or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in args.variant):
        parser.error("--variant must contain only letters, digits, '_' or '-'")
    if min(args.speed, args.agl, args.duration_cap, args.image_rate) <= 0 or args.noise < 0:
        parser.error("speed, agl, duration-cap, image-rate must be positive; noise must be non-negative")
    for input_path in (CAMERA_RASTER, MAP_RASTER):
        if not input_path.exists():
            raise FileNotFoundError(input_path)
    mask, left, top, res_m = load_shared_valid_mask()
    path_xy, times, poses, path_length, full_duration, _, _, _ = choose_safe_route(
        mask, left, top, res_m, args.speed, args.agl, args.image_rate, args.duration_cap)
    write_sequence(args, path_xy, times, poses, path_length, full_duration)


if __name__ == "__main__":
    main()

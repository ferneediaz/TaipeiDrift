"""The drone's heading from the sun, simulated from the recorded attitude: no magnetometer needed.

A compass is hard to trust on a small drone: the motors and their currents make a magnetic field of
their own that changes with the throttle. The sun's direction is free of that. With the date, the time
and a rough position, where the sun stands in the sky is known (``heading.sun_position``). A sensor on
top of the drone measures where the sun stands relative to the airframe; the difference is the heading.

How a reading is simulated, step by step, for every camera frame:

1. **The sun in the world**: a unit vector (east, north, up) from its azimuth and elevation. At
   azimuth 180 (south) and elevation 60: (0, -0.5, 0.87).
2. **The sun in the drone's frame** (forward, left, up): turned by the drone's true attitude from the
   recording. The sensor sees only this.
3. **The sensor's error.** Two sensors:
   - *Digital sun sensor* (Fan, Peng and Gao 2016: a V-shaped slit over a line detector, 35 g, 200 mW). It
     measures two angles of the sun against its axis, which points straight up from the airframe, each
     to 0.1 degree, plus a mounting error that stays for the flight. It sees the sun only within 65
     degrees of its axis, in both directions: a level drone sees a sun higher than 25 degrees.
   - *Photodiode sun sensor*: five photodiodes on a small pyramid, one facing up and four tilted 45
     degrees forward, back, left and right. Each one's current follows the cosine of the angle between
     its face and the sun (no current when the sun is behind it), plus light from the sky and light
     reflected from the ground. The device solves the five currents for the sun's direction, assuming
     an even sky and no ground light; what it does not model becomes its error, typically degrees.
     Example: the sun 30 degrees off the top diode's axis gives that diode cos(30) = 0.87 of full current.
4. **Levelling.** The measured direction is turned level with the drone's tilt as its IMU estimates it,
   about 1 degree off, changing slowly (as in turns, where the accelerometers are fooled).
5. **The heading** is the sun's azimuth in the world minus its angle from the nose. With the sun high
   up, the level part of its direction is short, and small errors turn into large heading errors: the
   same limit as in ``heading.sun_heading_error_sd``. Example: azimuth 215 in the world, seen 35 degrees to
   the right of the nose: heading 180.
6. **Clouds.** The sun is hidden when a cloud lies between the drone and the sun: the same clouds that
   shade the camera's ground (``src.data.camera_model``). The drone's line to the sun is traced down to
   the ground, where that cloud's shadow lies. While the sun is hidden, the heading is carried on by the
   gyro, which drifts by a small rate that stays for the flight; a new sun reading resets it.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta

import numpy as np

from src.sensors.heading import sun_position


def sun_vector_enu(azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    """Unit vector towards the sun: (east, north, up)."""
    a, e = np.radians(azimuth_deg), np.radians(elevation_deg)
    return np.array([np.cos(e) * np.sin(a), np.cos(e) * np.cos(a), np.sin(e)])


def rotations(q: np.ndarray) -> np.ndarray:
    """(N, 3, 3) matrices that turn body vectors (forward, left, up) into ENU, from quaternions qw, qx, qy, qz."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], axis=1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], axis=1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], axis=1),
    ], axis=1)


def yaw_and_tilt(r: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split body-to-ENU rotations into the yaw (about up, counter-clockwise from east) and the tilt:
    r = Rz(yaw) @ tilt."""
    yaw = np.arctan2(r[:, 1, 0], r[:, 0, 0])
    c, s = np.cos(-yaw), np.sin(-yaw)
    rz_back = np.zeros_like(r)
    rz_back[:, 0, 0], rz_back[:, 0, 1], rz_back[:, 1, 0], rz_back[:, 1, 1], rz_back[:, 2, 2] = c, -s, s, c, 1.0
    return yaw, rz_back @ r


def small_rotation(axis: np.ndarray, angle_rad: np.ndarray) -> np.ndarray:
    """(N, 3, 3) rotations by ``angle_rad`` about unit ``axis`` (Rodrigues)."""
    k = axis / np.linalg.norm(axis, axis=1, keepdims=True)
    kx = np.zeros((len(k), 3, 3))
    kx[:, 0, 1], kx[:, 0, 2], kx[:, 1, 0], kx[:, 1, 2], kx[:, 2, 0], kx[:, 2, 1] = -k[:, 2], k[:, 1], k[:, 2], -k[:, 0], -k[:, 1], k[:, 0]
    a = np.asarray(angle_rad)[:, None, None]
    return np.eye(3)[None] + np.sin(a) * kx + (1 - np.cos(a)) * kx @ kx


@dataclass(frozen=True)
class SunSensorModel:
    kind: str = "digital"  # "digital" (Fan et al.) or "photodiode"
    noise_deg: float = 0.1  # digital: noise of each measured angle
    mount_sd_deg: float = 0.2  # digital: mounting error, drawn once per flight
    fov_deg: float = 65.0  # digital: the sun must lie within this angle of the axis, in both directions
    diode_tilt_deg: float = 45.0  # photodiode: tilt of the four side diodes
    sky_light: float = 0.15  # photodiode: light from the sky, as a share of the direct sun on a facing diode
    ground_light: float = 0.25  # photodiode: share of the light reflected by the ground (albedo)
    diode_noise: float = 0.01  # photodiode: noise of each current, as a share of full sun
    tilt_sd_deg: float = 1.0  # error of the IMU's tilt estimate used for levelling
    tilt_time_s: float = 5.0  # how long that error stays similar
    gyro_drift_sd_deg_s: float = 0.01  # gyro drift while the sun is hidden: a rate drawn once per flight


def _digital(s_body: np.ndarray, m: SunSensorModel, rng: np.random.Generator) -> np.ndarray:
    """Measured sun directions (N, 3), NaN where the sun lies outside the field of view."""
    mount = small_rotation(rng.normal(size=(1, 3)), np.radians([rng.normal(0.0, m.mount_sd_deg)]))[0]
    s = s_body @ mount.T
    ax = np.arctan2(s[:, 0], s[:, 2]) + np.radians(rng.normal(0.0, m.noise_deg, len(s)))
    ay = np.arctan2(s[:, 1], s[:, 2]) + np.radians(rng.normal(0.0, m.noise_deg, len(s)))
    seen = (s[:, 2] > 0) & (np.abs(ax) <= np.radians(m.fov_deg)) & (np.abs(ay) <= np.radians(m.fov_deg))
    out = np.column_stack([np.tan(ax), np.tan(ay), np.ones(len(s))])
    out /= np.linalg.norm(out, axis=1, keepdims=True)
    out[~seen] = np.nan
    return out


def _photodiode(s_body: np.ndarray, up_body: np.ndarray, m: SunSensorModel, rng: np.random.Generator) -> np.ndarray:
    """Sun directions (N, 3) solved from five simulated diode currents; NaN when the sun is too weak to tell."""
    t = np.radians(m.diode_tilt_deg)
    normals = np.array([[0, 0, 1], [np.sin(t), 0, np.cos(t)], [-np.sin(t), 0, np.cos(t)], [0, np.sin(t), np.cos(t)], [0, -np.sin(t), np.cos(t)]])
    facing = s_body @ normals.T  # (N, 5) cosine of the angle between each diode and the sun
    sky_view = (1 + up_body @ normals.T) / 2  # share of the sky each diode sees
    ground_view = 1 - sky_view
    sun_up = np.clip(s_body[:, 2:3], 0, None)  # how much the ground is lit by the sun (roughly)
    current = np.clip(facing, 0, None) + m.sky_light * sky_view + m.ground_light * (sun_up + m.sky_light) * ground_view
    current += rng.normal(0.0, m.diode_noise, current.shape)
    # the device's own model: current = normal . (D s) + S * sky_view, solved by least squares for D s and S
    out = np.full_like(s_body, np.nan)
    for i in range(len(s_body)):
        design = np.column_stack([normals, sky_view[i]])
        solution, *_ = np.linalg.lstsq(design, current[i], rcond=None)
        direct = solution[:3]
        if np.linalg.norm(direct) > 0.3:  # a clear sun: at least 30 percent of full direct light
            out[i] = direct / np.linalg.norm(direct)
    return out


def sun_heading_readings(
    model: SunSensorModel,
    attitude_q: np.ndarray,
    timestamp_s: np.ndarray,
    start: datetime,
    latitude_deg: float,
    longitude_deg: float,
    rng: np.random.Generator,
    sun_hidden: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Heading readings (bearing from north, degrees) for every frame, and whether each came from the sun.

    ``attitude_q`` is the true attitude per frame (qw, qx, qy, qz, body to ENU); ``sun_hidden`` marks frames
    where a cloud hides the sun. Frames without a sun reading carry the last heading on with the gyro.
    """
    r = rotations(attitude_q)
    n = len(r)
    when = [start + timedelta(seconds=float(s)) for s in timestamp_s]
    sun_enu = np.array([sun_vector_enu(*sun_position(latitude_deg, longitude_deg, w)) for w in when])
    s_body = np.einsum("nji,nj->ni", r, sun_enu)  # r transposed: ENU to body
    yaw_true, tilt = yaw_and_tilt(r)

    # the IMU's tilt estimate: the true tilt, turned by a slowly changing error about a level axis
    dt = np.diff(timestamp_s, prepend=timestamp_s[0])
    keep = np.exp(-dt / model.tilt_time_s)
    err = np.zeros((n, 2))
    for k in range(n):
        fresh = rng.normal(0.0, np.radians(model.tilt_sd_deg), 2)
        err[k] = fresh if k == 0 else keep[k] * err[k - 1] + np.sqrt(1 - keep[k] ** 2) * fresh
    angle = np.linalg.norm(err, axis=1)
    axis = np.column_stack([err, np.zeros(n)]) + np.array([1e-12, 0.0, 0.0])
    tilt_est = small_rotation(axis, angle) @ tilt
    up_body = np.einsum("nji,j->ni", r, np.array([0.0, 0.0, 1.0]))  # where up is, in the body frame

    measured = _digital(s_body, model, rng) if model.kind == "digital" else _photodiode(s_body, up_body, model, rng)
    if sun_hidden is not None:
        measured[np.asarray(sun_hidden, bool)] = np.nan
    s_level = np.einsum("nij,nj->ni", tilt_est, measured)  # in the yaw-only frame, NaN where no reading
    yaw_est = np.arctan2(sun_enu[:, 1], sun_enu[:, 0]) - np.arctan2(s_level[:, 1], s_level[:, 0])

    heading = np.empty(n)
    from_sun = np.isfinite(yaw_est)
    drift = np.radians(rng.normal(0.0, model.gyro_drift_sd_deg_s))  # rad/s, for the flight
    last_yaw, last_true, last_t = yaw_true[0], yaw_true[0], timestamp_s[0]  # before the first reading: the truth
    for k in range(n):
        if from_sun[k]:
            last_yaw, last_true, last_t = yaw_est[k], yaw_true[k], timestamp_s[k]
            y = yaw_est[k]
        else:  # the gyro measures the turn since the last reading, plus its drift
            turned = np.angle(np.exp(1j * (yaw_true[k] - last_true)))
            y = last_yaw + turned + drift * (timestamp_s[k] - last_t)
        heading[k] = np.mod(90.0 - np.degrees(y), 360.0)
    return heading, from_sun

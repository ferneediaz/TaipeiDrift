"""Heading sensors a small drone can carry, as error models: a compass and a sun sensor.

The navigator needs the drone's heading (the direction the nose points, as a bearing from north)
to turn camera frames north up and to carry the dead reckoning through turns. These models take
the recorded heading of a flight and add the error the chosen sensor would make. They never see
the position.

Compass (magnetometer): an offset that stays for the flight, from the magnetic field of the motors
and battery and from a local disturbance of the earth's field, plus noise. Default: offset with a
spread of 4 degrees, noise of 1 degree.

Sun sensor (Fan, Peng and Gao 2016, the V-shaped slit over a line detector; ``docs/reading-notes.md``):
it measures the direction of the sun to 0.1 degrees. With the time and a rough position, the sun's
direction in the sky is known, and the difference gives the heading. Two things limit it on a drone:

- The sun's direction has to be split into a part along the horizon. Its length there is
  cos(elevation), so a sensor error d becomes a heading error d / cos(elevation).
- The sensor tilts with the drone. A tilt error t of the drone's roll and pitch (from its IMU)
  moves the measured sun sideways by up to t * tan(elevation).

Together, heading error = sqrt((d / cos e)^2 + (t * tan e)^2). With d = 0.1 and t = 1 degree:
at an elevation of 30 degrees, sqrt(0.12^2 + 0.58^2) = 0.59 degrees; at 60 degrees, 1.74; at 80
degrees, 5.7. So on a drone the sun sensor is limited by the tilt it is told, and it fails when
the sun stands overhead, which in Taiwan happens around midday from May to July.
"""
from __future__ import annotations

from datetime import datetime, timezone

import numpy as np


def compass_heading(true_heading_deg: np.ndarray, rng: np.random.Generator, offset_sd_deg: float = 4.0, noise_sd_deg: float = 1.0) -> np.ndarray:
    """Recorded heading plus one offset for the whole flight plus noise per reading."""
    true_heading_deg = np.asarray(true_heading_deg, dtype=float)
    offset = rng.normal(0.0, offset_sd_deg)
    return true_heading_deg + offset + rng.normal(0.0, noise_sd_deg, true_heading_deg.shape)


def sun_heading_error_sd(elevation_deg: np.ndarray, sensor_sd_deg: float = 0.1, tilt_sd_deg: float = 1.0) -> np.ndarray:
    """Spread of the heading error from a sun sensor, in degrees (see the module docstring)."""
    e = np.radians(np.asarray(elevation_deg, dtype=float))
    return np.sqrt((sensor_sd_deg / np.cos(e)) ** 2 + (tilt_sd_deg * np.tan(e)) ** 2)


def sun_heading(
    true_heading_deg: np.ndarray,
    elevation_deg: np.ndarray,
    rng: np.random.Generator,
    sensor_sd_deg: float = 0.1,
    tilt_sd_deg: float = 1.0,
) -> np.ndarray:
    """Recorded heading plus the error of a sun sensor at the sun's elevation of each reading."""
    sd = sun_heading_error_sd(elevation_deg, sensor_sd_deg, tilt_sd_deg)
    return np.asarray(true_heading_deg, dtype=float) + rng.normal(0.0, 1.0, sd.shape) * sd


def sun_elevation(latitude_deg: float, longitude_deg: float, when: datetime) -> float:
    """Elevation of the sun above the horizon in degrees, by the NOAA solar calculator equations.

    ``when`` must carry a time zone. Accurate to well under a tenth of a degree, which is plenty
    for an error model.
    """
    t = when.astimezone(timezone.utc)
    day = t.toordinal() + 1721424.5  # Julian day at 0:00 UTC
    minutes = t.hour * 60 + t.minute + t.second / 60
    jc = (day + minutes / 1440 - 2451545.0) / 36525  # Julian centuries since 2000
    mean_long = (280.46646 + jc * (36000.76983 + jc * 0.0003032)) % 360
    mean_anom = 357.52911 + jc * (35999.05029 - 0.0001537 * jc)
    ecc = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc)
    m = np.radians(mean_anom)
    centre = np.sin(m) * (1.914602 - jc * (0.004817 + 0.000014 * jc)) + np.sin(2 * m) * (0.019993 - 0.000101 * jc) + np.sin(3 * m) * 0.000289
    omega = np.radians(125.04 - 1934.136 * jc)
    apparent_long = mean_long + centre - 0.00569 - 0.00478 * np.sin(omega)
    obliquity = 23 + (26 + (21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813))) / 60) / 60 + 0.00256 * np.cos(omega)
    declination = np.arcsin(np.sin(np.radians(obliquity)) * np.sin(np.radians(apparent_long)))
    y = np.tan(np.radians(obliquity) / 2) ** 2
    l0 = np.radians(mean_long)
    equation_of_time = 4 * np.degrees(
        y * np.sin(2 * l0) - 2 * ecc * np.sin(m) + 4 * ecc * y * np.sin(m) * np.cos(2 * l0)
        - 0.5 * y * y * np.sin(4 * l0) - 1.25 * ecc * ecc * np.sin(2 * m)
    )
    solar_minutes = (minutes + equation_of_time + 4 * longitude_deg) % 1440
    hour_angle = np.radians(solar_minutes / 4 - 180)
    lat = np.radians(latitude_deg)
    cos_zenith = np.sin(lat) * np.sin(declination) + np.cos(lat) * np.cos(declination) * np.cos(hour_angle)
    return float(90 - np.degrees(np.arccos(np.clip(cos_zenith, -1, 1))))

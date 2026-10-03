"""The simulated sun sensor: geometry checked with small hand examples, then the two sensors' errors."""
from datetime import datetime, timedelta, timezone

import numpy as np
import pytest

from src.sensors.sun_sensor import SunSensorModel, rotations, sun_heading_readings, sun_vector_enu, yaw_and_tilt

TAIWAN = timezone(timedelta(hours=8))
WUFENG = (24.064, 120.699)


def quaternion_for_heading(heading_deg: float, pitch_deg: float = 0.0) -> np.ndarray:
    """Body-to-ENU quaternion of a drone with this heading (bearing from north) and nose-down pitch."""
    yaw = np.radians(90.0 - heading_deg)  # ENU yaw, counter-clockwise from east
    pitch = np.radians(pitch_deg)  # about the body's left axis: positive tips the nose down in FLU
    qz = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    qy = np.array([np.cos(pitch / 2), 0, np.sin(pitch / 2), 0])
    w1, x1, y1, z1 = qz
    w2, x2, y2, z2 = qy
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def test_sun_vector_points_south_and_up():
    s = sun_vector_enu(180.0, 60.0)
    assert s == pytest.approx([0.0, -0.5, np.sqrt(3) / 2], abs=1e-9)


def test_rotation_of_a_drone_heading_east():
    # heading east = ENU yaw 0: the body's forward axis is east, its left axis north
    r = rotations(quaternion_for_heading(90.0)[None])[0]
    assert r @ [1, 0, 0] == pytest.approx([1, 0, 0], abs=1e-9)
    assert r @ [0, 1, 0] == pytest.approx([0, 1, 0], abs=1e-9)


def test_yaw_and_tilt_split():
    r = rotations(quaternion_for_heading(30.0, pitch_deg=10.0)[None])
    yaw, tilt = yaw_and_tilt(r)
    assert np.degrees(yaw[0]) == pytest.approx(60.0)  # heading 30 = ENU yaw 60
    assert np.degrees(np.arccos(tilt[0, 2, 2])) == pytest.approx(10.0)  # the tilt keeps only the 10 degrees


@pytest.mark.parametrize("kind, limit_deg", [("digital", 0.6), ("photodiode", 6.0)])
def test_heading_error_at_demo_time(kind, limit_deg):
    # 4 October, 13:00: the sun at about 56 degrees; a level drone turning through all headings
    n = 360
    headings = np.linspace(0, 359, n)
    q = np.array([quaternion_for_heading(h) for h in headings])
    model = SunSensorModel(kind=kind, tilt_sd_deg=0.3)
    got, from_sun = sun_heading_readings(model, q, np.arange(n) * 0.2, datetime(2026, 10, 4, 13, 0, tzinfo=TAIWAN), *WUFENG,
                                         np.random.default_rng(1))
    error = np.angle(np.exp(1j * np.radians(got - headings)), deg=True)
    assert from_sun.all()
    assert np.median(np.abs(error)) < limit_deg


def test_digital_sensor_cannot_see_a_low_sun():
    # 4 October, 07:30: the sun at about 22 degrees, outside the 65-degree view of a level sensor
    q = np.array([quaternion_for_heading(0.0)] * 5)
    _, from_sun = sun_heading_readings(SunSensorModel(), q, np.arange(5.0), datetime(2026, 10, 4, 7, 30, tzinfo=TAIWAN), *WUFENG,
                                       np.random.default_rng(2))
    assert not from_sun.any()


def test_hidden_sun_is_carried_by_the_gyro():
    n = 50
    q = np.array([quaternion_for_heading(100.0 + k) for k in range(n)])  # turning 1 degree per frame
    hidden = np.zeros(n, bool)
    hidden[10:40] = True
    got, from_sun = sun_heading_readings(SunSensorModel(gyro_drift_sd_deg_s=0.0), q, np.arange(n) * 0.2,
                                         datetime(2026, 10, 4, 13, 0, tzinfo=TAIWAN), *WUFENG, np.random.default_rng(3), hidden)
    assert not from_sun[10:40].any() and from_sun[:10].all()
    error = np.angle(np.exp(1j * np.radians(got - (100.0 + np.arange(n)))), deg=True)
    # a gyro without drift follows the 30-degree turn exactly: the error stays the last sun reading's error
    assert error[10:40] == pytest.approx(np.full(30, error[9]), abs=1e-6)

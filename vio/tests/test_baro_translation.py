"""Measured barometer model and monocular translation-direction diagnostics."""
import inspect

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vio.diagnostics.translation import causal_slope, direction_error_deg, inertial_scaled_velocity, sign_agnostic
from vio.sensors.simulated import BarometerConfig, BarometerStream, generate_barometer, relative_altitude

T = np.arange(0.0, 600.0, 0.1)
ONLY = dict(white_std_m=0.0, bias_walk_m_per_sqrt_s=0.0, drift_sigma_m_per_s=0.0, scale_error_min=0.0, scale_error_max=0.0)


def _baro(alt=None, seed=0, **kw):
    return generate_barometer(T, np.zeros_like(T) if alt is None else alt, BarometerConfig(**{**ONLY, **kw, "seed": seed}))


# ---------------------------------------------------------------- barometer
def test_measured_defaults():
    c = BarometerConfig()
    assert (c.white_std_m, c.bias_walk_m_per_sqrt_s, c.drift_sigma_m_per_s, c.scale_error_min, c.scale_error_max) == (0.30, 0.112, 0.0024, 0.03, 0.07)


def test_white_noise_scale():
    assert np.std(_baro(white_std_m=0.30).altitude_m) == pytest.approx(0.30, rel=0.03)


def test_random_walk_grows_with_sqrt_time():
    z = np.array([_baro(seed=s, bias_walk_m_per_sqrt_s=0.112).altitude_m for s in range(400)])
    for tau in (10.0, 100.0, 400.0):
        assert np.std(z[:, int(tau * 10)]) == pytest.approx(0.112 * np.sqrt(tau), rel=0.12)


def test_linear_drift_per_flight():
    b = _baro(seed=3, drift_sigma_m_per_s=0.0024)
    np.testing.assert_allclose(np.diff(b.altitude_m) / 0.1, b.drift_m_per_s, atol=1e-12)  # exactly linear
    slopes = [_baro(seed=s, drift_sigma_m_per_s=0.0024).drift_m_per_s for s in range(2000)]
    assert np.std(slopes) == pytest.approx(0.0024, rel=0.06)


def test_per_flight_scale_error():
    alt = 0.2 * T  # climbs 120 m
    scales = []
    for s in range(200):
        b = _baro(alt=alt, seed=s, scale_error_min=0.03, scale_error_max=0.07)
        np.testing.assert_allclose(b.altitude_m - b.altitude_m[0], (1 + b.scale_error) * (alt - alt[0]), atol=1e-9)
        scales.append(b.scale_error)
    a = np.abs(scales)
    assert a.min() >= 0.03 and a.max() <= 0.07 and 0.3 < np.mean(np.array(scales) > 0) < 0.7  # both directions


def test_deterministic_seed():
    a, b, c = generate_barometer(T, np.zeros_like(T), BarometerConfig(seed=7)), generate_barometer(T, np.zeros_like(T), BarometerConfig(seed=7)), \
        generate_barometer(T, np.zeros_like(T), BarometerConfig(seed=8))
    np.testing.assert_array_equal(a.altitude_m, b.altitude_m)
    assert not np.allclose(a.altitude_m, c.altitude_m)


def test_relative_altitude_reference_at_cut():
    b = generate_barometer(T, 5.0 + 0.1 * T, BarometerConfig(seed=1))
    rel = relative_altitude(b, 300)
    assert rel[0] == 0.0 and len(rel) == len(T) - 300
    np.testing.assert_allclose(rel, b.altitude_m[300:] - b.altitude_m[300])


# ---------------------------------------------------------------- translation
def test_translation_direction_convention():
    """Camera b displaced along +x of camera a: recoverPose direction (camera a axes) is +x, sign resolved."""
    from vio.vision.camera import pinhole_intrinsics
    from vio.vision.relative_pose import estimate_relative_pose
    rng = np.random.default_rng(0)
    K = pinhole_intrinsics(512, 512, 90.0)
    z = rng.uniform(8, 30, 300)
    P = np.column_stack([rng.uniform(-0.8, 0.8, (300, 2)) * z[:, None], z])
    proj = lambda Q: Q[:, :2] / Q[:, 2:3] * K[0, 0] + K[:2, 2]  # noqa: E731
    pose = estimate_relative_pose(proj(P), proj(P - [0.5, 0, 0]), K)
    np.testing.assert_allclose(pose.translation_dir, [1, 0, 0], atol=2e-3)


def test_direction_error_and_sign_handling():
    assert direction_error_deg([1, 0, 0], [0, 1, 0])[0] == pytest.approx(90.0)
    e = direction_error_deg([[1, 0, 0]], [[-1, 0.01, 0]])
    assert e[0] > 179 and sign_agnostic(e)[0] < 1
    assert np.isnan(direction_error_deg([0, 0, 0], [1, 0, 0])[0])  # zero motion: undefined, not 0


def test_zero_motion_gives_no_parallax():
    """Pure rotation: the translational parallax after derotation is ~0, so the interval is not accepted."""
    from vio.diagnostics.sources import intervals_from_tracks
    from vio.vision.camera import pinhole_intrinsics
    from vio.vision.feature_tracker import TrackResult
    from vio.vision.relative_pose import PoseConfig
    rng = np.random.default_rng(1)
    K = pinhole_intrinsics(512, 512, 90.0)
    rays = np.c_[rng.uniform(-0.6, 0.6, (300, 2)), np.ones(300)]
    Rr = Rotation.from_euler("xyz", [1, 2, 3], degrees=True).as_matrix()
    pa = rays[:, :2] * K[0, 0] + K[:2, 2]
    rb = rays @ Rr  # pure rotation
    pb = rb[:, :2] / rb[:, 2:3] * K[0, 0] + K[:2, 2]
    tr = TrackResult(5, 0, pa.astype(np.float32), pb.astype(np.float32), 300, False, reanchor="age")
    ivs = intervals_from_tracks([tr], np.arange(6) * 0.1, K, PoseConfig(), 5)
    assert not ivs or ivs[0].parallax_px < 0.5


def test_metric_scale_comes_from_the_estimator_not_ground_truth():
    v = inertial_scaled_velocity(np.array([[0.0, 2.0, 0.0]]), np.array([7.0]))
    np.testing.assert_allclose(v, [[0.0, 7.0, 0.0]])
    assert set(inspect.signature(inertial_scaled_velocity).parameters) == {"t_dir_world", "speed_est"}


def test_causal_slope_sanity():
    t = np.arange(0, 10, 0.5)
    v = np.column_stack([3.0 * t, -1.0 * t + 2, np.zeros_like(t)])  # constant acceleration (3, -1, 0)
    a = causal_slope(t, v, 5)
    assert np.all(np.isnan(a[:4]))
    np.testing.assert_allclose(a[4:], np.tile([3.0, -1.0, 0.0], (len(t) - 4, 1)), atol=1e-12)
    v2 = v.copy()
    v2[-1] += 100.0  # a future sample must not change earlier estimates (causal)
    np.testing.assert_allclose(causal_slope(t, v2, 5)[:-1], a[:-1])

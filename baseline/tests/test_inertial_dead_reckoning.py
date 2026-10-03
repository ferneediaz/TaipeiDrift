"""Validate frames, gravity handling, attitude propagation and integration of the estimator."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.data.synthetic import SCENARIOS, make_synthetic_trajectory
from src.data.trajectory import gravity_vector, quat_wxyz_to_rotation, rotation_to_quat_wxyz
from src.estimation.inertial_dead_reckoning import NavState, propagate, run_dead_reckoning

G = 9.81
IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])


def _max_error(scenario: str, rate_hz: float = 100.0, duration: float = 60.0) -> float:
    traj = make_synthetic_trajectory(scenario, duration=duration, rate_hz=rate_hz)
    res = run_dead_reckoning(traj, 5.0)
    return float(np.max(np.linalg.norm(res.position - traj.position_gt[res.start_index:], axis=1)))


@pytest.mark.parametrize("scenario,tol", [
    ("stationary", 1e-8),          # exact up to round-off
    ("constant_velocity", 1e-8),   # exact up to round-off
    ("spinning_hover", 1e-6),      # constant body rate: midpoint rotation is exact
    ("circle", 1e-2),              # time-varying acceleration: trapezoid error only
])
def test_ideal_imu_reproduces_truth(scenario, tol):
    assert _max_error(scenario) < tol


def test_circle_error_is_second_order():
    """Halving dt divides the error by about 4: integration error, not a frame bug."""
    e100 = _max_error("circle", rate_hz=100.0, duration=30.0)
    e200 = _max_error("circle", rate_hz=200.0, duration=30.0)
    assert 3.5 < e100 / e200 < 4.5


def test_attitude_propagation_body_rate():
    """A constant yaw rate of pi/2 rad/s for 1 s from level turns the nose from north to east."""
    t = np.linspace(0.0, 1.0, 101)
    gyro = np.tile([0.0, 0.0, np.pi / 2], (101, 1))
    accel = np.tile([0.0, 0.0, -G], (101, 1))
    _, _, att = propagate(t, accel, gyro, NavState(np.zeros(3), np.zeros(3), IDENTITY), gravity_vector("NED"))
    r = quat_wxyz_to_rotation(att[-1])
    np.testing.assert_allclose(r.apply([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)  # body x -> east


def test_attitude_increment_is_in_body_frame():
    """From a 90 deg roll, a body-z rate turns about world -y (NED), not world z."""
    roll90 = rotation_to_quat_wxyz(Rotation.from_euler("X", 90, degrees=True))
    t = np.linspace(0.0, 1.0, 11)
    gyro = np.tile([0.0, 0.0, 0.3], (11, 1))
    accel = np.zeros((11, 3))
    _, _, att = propagate(t, accel, gyro, NavState(np.zeros(3), np.zeros(3), roll90), np.zeros(3))
    expected = Rotation.from_euler("X", 90, degrees=True) * Rotation.from_euler("Z", 0.3)
    assert (quat_wxyz_to_rotation(att[-1]).inv() * expected).magnitude() < 1e-12


def test_body_force_is_rotated_into_world():
    """Nose east (yaw 90 deg), 1 m/s^2 forward thrust: the drone accelerates east, p_east = t^2 / 2."""
    yaw90 = rotation_to_quat_wxyz(Rotation.from_euler("Z", 90, degrees=True))
    t = np.linspace(0.0, 10.0, 1001)
    accel = np.tile([1.0, 0.0, -G], (1001, 1))  # forward specific force plus the hover reading
    p, v, _ = propagate(t, accel, np.zeros((1001, 3)), NavState(np.zeros(3), np.zeros(3), yaw90), gravity_vector("NED"))
    np.testing.assert_allclose(p[:, 1], 0.5 * t**2, atol=1e-9)
    np.testing.assert_allclose(p[:, [0, 2]], 0.0, atol=1e-9)
    np.testing.assert_allclose(v[-1], [0.0, 10.0, 0.0], atol=1e-9)


def test_wrong_gravity_sign_falls_at_twice_g():
    """Sanity check of the gravity handling: with the sign flipped a hovering drone 'falls' up at 2g."""
    t = np.linspace(0.0, 10.0, 1001)
    accel = np.tile([0.0, 0.0, -G], (1001, 1))
    p, _, _ = propagate(t, accel, np.zeros((1001, 3)), NavState(np.zeros(3), np.zeros(3), IDENTITY), -gravity_vector("NED"))
    np.testing.assert_allclose(p[-1, 2], -0.5 * 2 * G * 10.0**2, rtol=1e-9)


def test_free_fall():
    """Zero specific force means free fall: Down grows as g t^2 / 2."""
    t = np.linspace(0.0, 3.0, 301)
    p, v, _ = propagate(t, np.zeros((301, 3)), np.zeros((301, 3)), NavState(np.zeros(3), np.zeros(3), IDENTITY), gravity_vector("NED"))
    np.testing.assert_allclose(p[:, 2], 0.5 * G * t**2, atol=1e-9)
    np.testing.assert_allclose(v[-1, 2], 3.0 * G, atol=1e-9)


def test_starts_at_cutoff_from_true_state():
    traj = make_synthetic_trajectory("circle", duration=20.0)
    res = run_dead_reckoning(traj, 5.0)
    assert res.start_index == 500 and res.t0 == pytest.approx(5.0)
    np.testing.assert_array_equal(res.position[0], traj.position_gt[500])
    np.testing.assert_array_equal(res.velocity[0], traj.velocity_gt[500])
    assert len(res.timestamp) == len(traj) - 500


def test_cutoff_between_samples_uses_next_sample():
    traj = make_synthetic_trajectory("stationary", duration=10.0)
    assert run_dead_reckoning(traj, 5.004).start_index == 501


def test_cutoff_after_end_fails():
    traj = make_synthetic_trajectory("stationary", duration=10.0)
    with pytest.raises(ValueError, match="no samples"):
        run_dead_reckoning(traj, 20.0)


def test_ground_truth_after_cutoff_is_not_used():
    """Corrupting ground truth after t0 must leave the estimate unchanged."""
    traj = make_synthetic_trajectory("circle", duration=20.0)
    before = run_dead_reckoning(traj, 5.0)
    k0 = before.start_index
    traj.position_gt[k0 + 1:] += 1000.0
    traj.velocity_gt[k0 + 1:] *= -1.0
    traj.attitude_gt[k0 + 1:] = IDENTITY
    after = run_dead_reckoning(traj, 5.0)
    np.testing.assert_array_equal(before.position, after.position)
    np.testing.assert_array_equal(before.attitude, after.attitude)


@pytest.mark.parametrize("scenario", sorted(SCENARIOS))
def test_gyro_bias_causes_growing_drift(scenario):
    """A small gyro bias tilts the estimate, so gravity leaks in and the error grows."""
    from src.data.synthetic import ImuNoise
    traj = make_synthetic_trajectory(scenario, duration=60.0, noise=ImuNoise(gyro_bias=(0.001, 0.0, 0.0)))
    res = run_dead_reckoning(traj, 5.0)
    e = np.linalg.norm(res.position - traj.position_gt[res.start_index:], axis=1)
    assert e[-1] > 10.0 * e[len(e) // 4] > 0


@pytest.mark.parametrize("scenario", ["circle", "spinning_hover"])
def test_world_frame_gyro_reproduces_truth(scenario):
    """Mid-Air records world-frame rates; with gyroscope_frame='world' the estimate still matches."""
    traj = make_synthetic_trajectory(scenario, duration=60.0, gyroscope_frame="world")
    res = run_dead_reckoning(traj, 5.0)
    assert np.max(np.linalg.norm(res.position - traj.position_gt[res.start_index:], axis=1)) < 1e-2


def test_world_frame_gyro_read_as_body_drifts():
    """The Mid-Air failure mode: a world-frame gyro integrated as a body rate gives a large error."""
    traj = make_synthetic_trajectory("spinning_hover", duration=60.0, gyroscope_frame="world")
    traj.gyroscope_frame = "body"
    res = run_dead_reckoning(traj, 5.0)
    assert np.linalg.norm(res.position[-1] - traj.position_gt[-1]) > 100.0

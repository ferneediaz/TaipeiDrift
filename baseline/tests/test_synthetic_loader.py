"""The synthetic flights must be self-consistent before they can validate the estimator."""
import numpy as np
import pytest

from src.data.synthetic import SCENARIOS, ImuNoise, make_synthetic_trajectory
from src.data.trajectory import (
    Trajectory,
    gravity_vector,
    imu_consistency,
    quat_wxyz_to_rotation,
)

G = 9.81


@pytest.mark.parametrize("scenario", sorted(SCENARIOS))
def test_shapes_and_units(scenario):
    traj = make_synthetic_trajectory(scenario, duration=10.0, rate_hz=100.0)
    assert len(traj) == 1001
    np.testing.assert_allclose(np.diff(traj.timestamp), 0.01)
    np.testing.assert_allclose(np.linalg.norm(traj.attitude_gt, axis=1), 1.0)
    assert traj.world_frame == "NED"
    np.testing.assert_allclose(traj.gravity_world, [0, 0, G])


@pytest.mark.parametrize("scenario", sorted(SCENARIOS))
def test_velocity_is_derivative_of_position(scenario):
    traj = make_synthetic_trajectory(scenario, duration=10.0, rate_hz=1000.0)
    t, p = traj.timestamp, traj.position_gt
    v_fd = (p[2:] - p[:-2]) / (t[2:] - t[:-2])[:, None]
    np.testing.assert_allclose(v_fd, traj.velocity_gt[1:-1], atol=1e-4)


@pytest.mark.parametrize("scenario", sorted(SCENARIOS))
def test_ideal_imu_matches_ground_truth(scenario):
    check = imu_consistency(make_synthetic_trajectory(scenario, duration=10.0))
    assert check["accel_residual_median"] < 1e-3
    assert check["gyro_residual_median"] < 1e-6


def test_stationary_accelerometer_reads_minus_gravity_in_world():
    """At rest the specific force is -g: rotated to the world it points up, (0, 0, -g) in NED."""
    traj = make_synthetic_trajectory("stationary", duration=1.0)
    np.testing.assert_allclose(np.linalg.norm(traj.accelerometer, axis=1), G)
    f_world = quat_wxyz_to_rotation(traj.attitude_gt).apply(traj.accelerometer)
    np.testing.assert_allclose(f_world, np.tile([0, 0, -G], (len(traj), 1)), atol=1e-12)
    # tilted attitude, so the body-frame reading is not simply (0, 0, -g)
    assert abs(traj.accelerometer[0, 2] + G) > 0.1


def test_level_hover_reads_minus_g_on_body_z():
    """Same convention as Mid-Air (experiments/e_midair_imu_noise.py): level and still -> (0, 0, -g)."""
    traj = make_synthetic_trajectory("constant_velocity", duration=1.0)
    np.testing.assert_allclose(traj.accelerometer, np.tile([0, 0, -G], (len(traj), 1)), atol=1e-12)


def test_circle_gyro_has_bank_split():
    traj = make_synthetic_trajectory("circle", duration=1.0)
    bank, rate = np.deg2rad(15.0), 0.25
    np.testing.assert_allclose(traj.gyroscope[0], [0, rate * np.sin(bank), rate * np.cos(bank)], atol=1e-12)


def test_wrong_quaternion_order_is_detected():
    traj = make_synthetic_trajectory("constant_velocity", duration=5.0)
    swapped = Trajectory(
        timestamp=traj.timestamp, position_gt=traj.position_gt, velocity_gt=traj.velocity_gt,
        attitude_gt=traj.attitude_gt[:, [1, 2, 3, 0]],  # read as if it were (x, y, z, w)
        accelerometer=traj.accelerometer, gyroscope=traj.gyroscope,
    )
    assert imu_consistency(swapped)["accel_residual_median"] > 5.0


def test_wrong_gravity_sign_is_detected():
    traj = make_synthetic_trajectory("stationary", duration=5.0)
    traj.gravity_world = gravity_vector("ENU")
    assert imu_consistency(traj)["accel_residual_median"] > 15.0


def test_noise_is_deterministic():
    noise = ImuNoise(accel_white_std=0.1, gyro_white_std=0.01, seed=3)
    a = make_synthetic_trajectory("circle", duration=5.0, noise=noise)
    b = make_synthetic_trajectory("circle", duration=5.0, noise=noise)
    c = make_synthetic_trajectory("circle", duration=5.0)
    np.testing.assert_array_equal(a.accelerometer, b.accelerometer)
    assert not np.allclose(a.accelerometer, c.accelerometer)
    np.testing.assert_array_equal(a.position_gt, c.position_gt)  # noise only touches the IMU


def test_unknown_scenario():
    with pytest.raises(ValueError, match="unknown scenario"):
        make_synthetic_trajectory("loop_the_loop")

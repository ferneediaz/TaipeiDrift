"""Gravity roll/pitch update, yaw unobservability, acceleration rejection, quality-conditioned visual noise."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.data.synthetic import ImuNoise, make_synthetic_trajectory
from src.data.trajectory import quat_wxyz_to_rotation, rotation_to_quat_wxyz
from src.estimation.inertial_dead_reckoning import NavState
from vio.bias_experiment import DEG, run_on_trajectory
from vio.estimation.gyro_bias_kf import BiasInterval, BiasKFConfig, run_bias_kf
from vio.estimation.visual_quality import SIGMA_FLOOR_DEG, QualityModel
from vio.evaluation.attitude_metrics import tilt_heading_error_deg

GRAV = BiasKFConfig(gravity_update=True, gravity_sigma_deg=1.0, gyro_noise_density=0.0, bias_walk=1e-6)
OFF = BiasKFConfig(gravity_update=False, gyro_noise_density=0.0, bias_walk=1e-6)


def _level_flight(bias_world_deg_s, duration=60.0):
    """Level, constant-velocity flight (no linear acceleration) with a world-frame gyro bias."""
    return make_synthetic_trajectory("constant_velocity", duration=duration, gyroscope_frame="world",
                                     noise=ImuNoise(gyro_bias=tuple(np.asarray(bias_world_deg_s) * DEG)))


def _tilt_heading(out, traj):
    k0 = out.result.start_index
    return tilt_heading_error_deg(out.result.attitude, traj.attitude_gt[k0:], traj.gravity_world)


def test_gravity_sign_and_frame():
    """NED, level, at rest: accelerometer (0, 0, -g) -> measured gravity direction (0, 0, 1) = R^T g_unit."""
    traj = _level_flight([0, 0, 0], duration=10.0)
    np.testing.assert_allclose(-traj.accelerometer[0] / np.linalg.norm(traj.accelerometer[0]), [0, 0, 1], atol=1e-12)
    # start 3 deg tilted: gravity alone must bring the tilt back
    k0 = traj.index_at(5.0)
    q0 = rotation_to_quat_wxyz(Rotation.from_euler("x", 3, degrees=True) * quat_wxyz_to_rotation(traj.attitude_gt[k0]))
    cfg = BiasKFConfig(gravity_update=True, gravity_sigma_deg=1.0, init_att_std_deg=3.0, gyro_noise_density=0.0)
    out = run_bias_kf(traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], "world", traj.gravity_world,
                      NavState(traj.position_gt[k0], traj.velocity_gt[k0], q0), k0, [], cfg)
    tilt, _ = _tilt_heading(out, traj)
    assert tilt[0] == pytest.approx(3.0, abs=0.01) and tilt[-1] < 0.1


@pytest.mark.parametrize("axis", [0, 1])  # world x and y: roll/pitch-type drift for a level drone
def test_gravity_bounds_tilt_drift(axis):
    bias = np.zeros(3)
    bias[axis] = 0.1
    traj = _level_flight(bias)
    t_off, _ = _tilt_heading(run_on_trajectory(traj, 5.0, [], OFF), traj)
    t_on, _ = _tilt_heading(run_on_trajectory(traj, 5.0, [], GRAV), traj)
    assert t_off[-1] > 4.0
    assert t_on[-1] < 0.1 * t_off[-1]


def test_gravity_does_not_observe_yaw():
    traj = _level_flight([0, 0, 0.1])
    off, on = run_on_trajectory(traj, 5.0, [], OFF), run_on_trajectory(traj, 5.0, [], GRAV)
    t_off, h_off = _tilt_heading(off, traj)
    t_on, h_on = _tilt_heading(on, traj)
    assert abs(h_off[-1]) > 4.0
    np.testing.assert_allclose(h_on, h_off, atol=1e-6)  # heading drift untouched
    assert t_on[-1] < 0.05


def test_combined_drift_tilt_fixed_heading_not():
    traj = _level_flight([0.08, -0.06, 0.1])
    off, on = run_on_trajectory(traj, 5.0, [], OFF), run_on_trajectory(traj, 5.0, [], GRAV)
    t_off, h_off = _tilt_heading(off, traj)
    t_on, h_on = _tilt_heading(on, traj)
    assert t_on[-1] < 0.1 * t_off[-1]
    assert abs(h_on[-1]) > 0.8 * abs(h_off[-1])


def test_heading_correction_is_projected_out_even_with_correlations():
    """Correlated prior (tilt-heading) must not let gravity change heading."""
    traj = _level_flight([0, 0, 0], duration=20.0)
    k0 = traj.index_at(5.0)
    R0 = Rotation.from_euler("zx", [5, 2], degrees=True) * quat_wxyz_to_rotation(traj.attitude_gt[k0])
    cfg = BiasKFConfig(gravity_update=True, gravity_sigma_deg=1.0, init_att_std_deg=5.0, gyro_noise_density=0.0)
    out = run_bias_kf(traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], "world", traj.gravity_world,
                      NavState(traj.position_gt[k0], traj.velocity_gt[k0], rotation_to_quat_wxyz(R0)), k0, [], cfg)
    tilt, heading = _tilt_heading(out, traj)
    assert tilt[-1] < 0.1
    np.testing.assert_allclose(heading[-1], heading[0], atol=1e-6)


def test_strong_acceleration_is_rejected():
    traj = _level_flight([0.1, 0, 0], duration=20.0)
    traj.accelerometer = traj.accelerometer * (1.0 + 1.0 / 9.81)  # ||a|| = g + 1 m/s^2: not gravity-dominated
    out = run_on_trajectory(traj, 5.0, [], GRAV)
    assert out.gravity and not any(g.accepted for g in out.gravity)
    assert {g.reason for g in out.gravity} == {"norm"}
    noisy = _level_flight([0.1, 0, 0], duration=20.0)
    noisy.accelerometer = noisy.accelerometer + np.random.default_rng(0).normal(0, 1.0, noisy.accelerometer.shape)
    out = run_on_trajectory(noisy, 5.0, [], GRAV)
    assert not any(g.accepted for g in out.gravity) and any(g.reason == "variance" for g in out.gravity)


def test_gravity_on_body_frame_gyro_too():
    traj = make_synthetic_trajectory("constant_velocity", duration=60.0, gyroscope_frame="body",
                                     noise=ImuNoise(gyro_bias=(0.1 * DEG, 0.0, 0.0)))
    off = run_on_trajectory(traj, 5.0, [], OFF)
    on = run_on_trajectory(traj, 5.0, [], GRAV)
    assert _tilt_heading(on, traj)[0][-1] < 0.1 * _tilt_heading(off, traj)[0][-1]


def test_body_frame_gyro_bias_from_vision():
    """NTU-style body-frame gyro: relative visual rotations recover the body-frame bias."""
    traj = make_synthetic_trajectory("circle", duration=60.0, gyroscope_frame="body",
                                     noise=ImuNoise(gyro_bias=(0.3 * DEG, -0.2 * DEG, 0.1 * DEG)))
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    ivs = [BiasInterval(i, i + 50, (R[i].inv() * R[i + 50]).as_matrix()) for i in range(500, len(traj) - 50, 50)]
    out = run_on_trajectory(traj, 5.0, ivs, BiasKFConfig(sigma_vis_deg=0.01, gyro_noise_density=0.0, bias_walk=1e-6))
    np.testing.assert_allclose(out.bias[-1] / DEG, [0.3, -0.2, 0.1], atol=0.01)


# ---------------------------------------------------------------- quality-conditioned visual noise
QM = QualityModel(sigma0_deg=0.05, alpha_flow=3.0, beta_inlier=2.0, gamma_tracks=10.0)


def test_quality_sigma_is_monotone_and_floored():
    base = QM.sigma_deg(10.0, 256.0, 0.95, 300)
    assert QM.sigma_deg(30.0, 256.0, 0.95, 300) > base  # more image motion -> less trust
    assert QM.sigma_deg(10.0, 256.0, 0.70, 300) > base  # fewer inliers -> less trust
    assert QM.sigma_deg(10.0, 256.0, 0.95, 50) > base  # fewer tracks -> less trust
    assert QM.sigma_deg(0.0, 256.0, 1.0, 1000) >= SIGMA_FLOOR_DEG  # never below the floor
    assert QM.sigma_deg(10.0, 256.0, 0.99, 300) == pytest.approx(QM.sigma_deg(10.0, 256.0, 0.95, 300))  # good quality never raises trust above the reference


def test_same_rule_for_both_cameras():
    """Same angular image motion and quality -> same sigma, whatever the camera (Mid-Air f=256, NTU f=425)."""
    assert QM.sigma_deg(25.6, 256.0, 0.9, 150) == pytest.approx(QM.sigma_deg(42.5, 425.0, 0.9, 150))


def test_per_interval_sigma_is_used():
    traj = make_synthetic_trajectory("circle", duration=30.0, gyroscope_frame="world",
                                     noise=ImuNoise(gyro_bias=(0.2 * DEG, 0, 0)))
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    mk = lambda s: [BiasInterval(i, i + 50, (R[i].inv() * R[i + 50]).as_matrix(), sigma_deg=s) for i in range(500, len(traj) - 50, 50)]  # noqa: E731
    tight = run_on_trajectory(traj, 5.0, mk(0.01), BiasKFConfig(gyro_noise_density=0.0))
    loose = run_on_trajectory(traj, 5.0, mk(5.0), BiasKFConfig(gyro_noise_density=0.0))
    assert np.linalg.norm(tight.bias[-1] / DEG - [0.2, 0, 0]) < np.linalg.norm(loose.bias[-1] / DEG - [0.2, 0, 0])


def test_no_ground_truth_after_cutoff_with_gravity():
    traj = _level_flight([0.08, -0.06, 0.1], duration=30.0)
    a = run_on_trajectory(traj, 5.0, [], GRAV)
    k0 = traj.index_at(5.0)
    traj.attitude_gt[k0 + 1:] = [1.0, 0, 0, 0]
    traj.position_gt[k0 + 1:] += 50.0
    traj.velocity_gt[k0 + 1:] *= -1
    b = run_on_trajectory(traj, 5.0, [], GRAV)
    np.testing.assert_array_equal(a.result.attitude, b.result.attitude)
    assert [g.accepted for g in a.gravity] == [g.accepted for g in b.gravity]

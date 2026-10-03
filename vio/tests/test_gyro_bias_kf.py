"""Bias-only gyro filter: axis/sign recovery, zero bias, residual convention, leakage, fallback, no leakage of GT."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.data.trajectory import Trajectory, gravity_vector, rotation_to_quat_wxyz
from src.estimation.inertial_dead_reckoning import NavState, run_dead_reckoning
from vio.bias_experiment import DEG, intervals_from_measurements, run_on_trajectory, synthetic_case
from vio.estimation.gyro_bias_kf import BiasInterval, BiasKFConfig, run_bias_kf
from vio.evaluation.attitude_metrics import attitude_error_series
from vio.vision.measurements import VisualMeasurement

LOW_NOISE = BiasKFConfig(sigma_vis_deg=0.005, gyro_noise_density=0.0, bias_walk=1e-6)


def _recover(bias_deg_s, **kw):
    traj, ivs, _ = synthetic_case(bias_deg_s, vis_sigma_deg=0.005, **kw)
    out = run_on_trajectory(traj, 5.0, ivs, LOW_NOISE)
    return traj, out, out.bias[-1] / DEG


@pytest.mark.parametrize("bias", [[0.02, 0, 0], [0, 0.02, 0], [0, 0, 0.02], [0.02, -0.01, 0.015]])
def test_recovers_world_frame_bias_axis_and_sign(bias):
    traj, out, est = _recover(bias)
    assert np.linalg.norm(est - bias) < 0.006  # deg/s
    big = int(np.argmax(np.abs(bias)))
    assert np.sign(est[big]) == np.sign(bias[big])
    # drift after convergence: much slower than the biased IMU wherever the bias drives drift
    imu = attitude_error_series(run_dead_reckoning(traj, 5.0), traj).error_deg
    kf = attitude_error_series(out.result, traj).error_deg
    if imu[-1] > 1.0:
        assert kf[-1] < 0.2 * imu[-1]


def test_zero_bias_is_not_invented():
    _, out, est = _recover([0, 0, 0])
    assert np.linalg.norm(est) < 0.006
    assert np.linalg.norm(est) < 3 * np.linalg.norm(out.bias_std[-1] / DEG)


def test_residual_sign_convention():
    """r = Log(R_i C R_j^T) ~ -Delta_t * delta_b: with b_hat = 0 and a positive bias, r points along -b."""
    traj, ivs, b = synthetic_case([0.0, 0.0, 0.5], vis_sigma_deg=0.0, duration=20.0)
    out = run_on_trajectory(traj, 5.0, ivs[:1], BiasKFConfig(sigma_vis_deg=1e-4, gyro_noise_density=0.0, gate_prob=None))
    est = out.bias[-1]
    assert est[2] > 0.8 * b[2]  # one update already moves b_hat towards +b: the sign is right
    # explicit residual on a single interval
    R = Rotation.from_quat(traj.attitude_gt[:, [1, 2, 3, 0]])
    i, j = ivs[0].imu_i, ivs[0].imu_j
    Rh_j = R[i]
    for k in range(i, j):  # IMU propagation with b_hat = 0 (biased)
        Rh_j = Rotation.from_rotvec(0.5 * (traj.gyroscope[k] + traj.gyroscope[k + 1]) * 0.01) * Rh_j
    r = (R[i] * Rotation.from_matrix(ivs[0].C_body) * Rh_j.inv()).as_rotvec()
    np.testing.assert_allclose(r, -1.0 * b, rtol=0.05, atol=1e-6)


def _spin(rate_scale, bias_w=(0, 0, 0), dur=40.0):
    t = np.arange(int(dur * 100) + 1) / 100
    wb = np.array([0.3, -0.2, 0.5]) * rate_scale
    R = Rotation.from_euler("ZYX", [10, -5, 20], degrees=True) * Rotation.from_rotvec(t[:, None] * wb)
    g = gravity_vector("NED")
    z = np.zeros((len(t), 3))
    return Trajectory(timestamp=t, position_gt=z, velocity_gt=z, attitude_gt=rotation_to_quat_wxyz(R),
                      accelerometer=R.inv().apply(np.tile(-g, (len(t), 1))),
                      gyroscope=R.apply(np.tile(wb, (len(t), 1))) + np.asarray(bias_w), world_frame="NED",
                      gravity_world=g, gyroscope_frame="world")


def _run_with_attitude_error(traj, consider):
    R = Rotation.from_quat(traj.attitude_gt[:, [1, 2, 3, 0]])
    ivs = [BiasInterval(i, i + 100, (R[i].inv() * R[i + 100]).as_matrix()) for i in range(0, len(traj) - 100, 100)]
    q0 = rotation_to_quat_wxyz(Rotation.from_rotvec(np.deg2rad([2.0, -1.0, 1.5])) * R[0])
    cfg = BiasKFConfig(sigma_vis_deg=0.01, gyro_noise_density=0.0, bias_walk=1e-7, init_att_std_deg=2.7,
                       consider_attitude=consider)
    return run_bias_kf(traj.timestamp, traj.accelerometer, traj.gyroscope, "world", traj.gravity_world,
                       NavState(np.zeros(3), np.zeros(3), q0), 0, ivs, cfg).bias[-1] / DEG


def test_attitude_error_leaks_into_pure_bias_filter_with_rotation():
    """Confirms the diagnosis: with zero bias, initial attitude error leaks into b_hat in
    proportion to rotation; the consider-state filter stays near zero."""
    leak = [np.linalg.norm(_run_with_attitude_error(_spin(s), consider=False)) for s in (0.0, 0.5, 1.0)]
    assert leak[0] < 1e-6 and leak[1] > 0.1 and leak[2] > 1.5 * leak[1]
    assert np.linalg.norm(_run_with_attitude_error(_spin(1.0), consider=True)) < 0.03


def test_attitude_error_jacobian_has_rotation_term():
    """Finite difference: world gyro, left error R = Exp(dtheta) R_hat -> dtheta+ = Exp(psi) dtheta - dt delta_b."""
    rng = np.random.default_rng(0)
    Rh = Rotation.from_rotvec(rng.normal(size=3))
    w, dt = np.array([0.4, -0.7, 0.3]), 0.01
    dth, db = np.array([1e-4, -2e-4, 1.5e-4]), np.array([3e-5, -1e-5, 2e-5])
    Rt1 = Rotation.from_rotvec(w * dt) * Rotation.from_rotvec(dth) * Rh
    Rh1 = Rotation.from_rotvec((w + db) * dt) * Rh
    actual = (Rt1 * Rh1.inv()).as_rotvec()
    with_term = Rotation.from_rotvec(w * dt).as_matrix() @ dth - db * dt
    without = dth - db * dt
    e_with, e_without = np.abs(actual - with_term).max(), np.abs(actual - without).max()
    assert e_with < 1e-8 and e_without > 100 * e_with


def test_camera_failure_falls_back_to_imu():
    traj, ivs, _ = synthetic_case([0, 0, 0.05], vis_sigma_deg=0.005)
    failed = [BiasInterval(iv.imu_i, iv.imu_j, None, "simulated failure") for iv in ivs]
    out = run_on_trajectory(traj, 5.0, failed, LOW_NOISE)
    np.testing.assert_allclose(out.result.position, run_dead_reckoning(traj, 5.0).position, atol=1e-9)
    assert not any(u.accepted for u in out.updates) and np.all(out.bias == 0)
    traj, ivs, _ = synthetic_case([0, 0, 0.05], vis_sigma_deg=0.005, fail_every=3)  # partial outage
    est = run_on_trajectory(traj, 5.0, ivs, LOW_NOISE).bias[-1] / DEG
    assert abs(est[2] - 0.05) < 0.01


def test_no_ground_truth_after_cutoff_is_used():
    traj, ivs, _ = synthetic_case([0.02, -0.01, 0.015], vis_sigma_deg=0.005)
    a = run_on_trajectory(traj, 5.0, ivs, LOW_NOISE)
    k0 = traj.index_at(5.0)
    traj.position_gt[k0 + 1:] += 100.0
    traj.attitude_gt[k0 + 1:] = [1.0, 0, 0, 0]
    b = run_on_trajectory(traj, 5.0, ivs, LOW_NOISE)
    np.testing.assert_array_equal(a.result.position, b.result.position)
    np.testing.assert_array_equal(a.bias, b.bias)


def test_intervals_from_measurements_are_non_overlapping():
    ms = []
    for kf in (0, 25, 50):
        ms.append(VisualMeasurement(kf, kf, 4 * kf, 4 * kf, False, new_keyframe=kf == 0))
        for f in range(kf + 1, kf + 26):
            ms.append(VisualMeasurement(f, kf, 4 * f, 4 * kf, True, [np.eye(3)], end_of_span=f == kf + 25))
    one = intervals_from_measurements(ms, 25)
    half = intervals_from_measurements(ms, 12)
    assert [(i.imu_i, i.imu_j) for i in one] == [(0, 100), (100, 200), (200, 300)]
    assert [(i.imu_i, i.imu_j) for i in half] == [(0, 48), (100, 148), (200, 248)]


# ---------------------------------------------------------------- soft-threshold shrinkage
from vio.estimation.gyro_bias_kf import soft_threshold_bias  # noqa: E402


def test_soft_threshold_zero_stays_zero():
    np.testing.assert_array_equal(soft_threshold_bias(np.zeros(3), 0.01), np.zeros(3))


def test_soft_threshold_below_floor_gives_zero():
    np.testing.assert_array_equal(soft_threshold_bias([0.009, -0.005, 0.0], 0.01), [0.0, 0.0, 0.0])


def test_soft_threshold_shrinks_by_exactly_floor_per_axis_with_sign():
    out = soft_threshold_bias([0.03, -0.025, 0.004], 0.01)
    np.testing.assert_allclose(out, [0.02, -0.015, 0.0])
    assert np.sign(out[0]) == 1 and np.sign(out[1]) == -1


def test_raw_estimate_stays_unbiased_with_shrinkage():
    """The applied bias is shrunk, the raw b_hat still converges to the truth (offset compensated)."""
    traj, ivs, b = synthetic_case([0.2, -0.1, 0.15], vis_sigma_deg=0.005)
    cfg = BiasKFConfig(sigma_vis_deg=0.005, gyro_noise_density=0.0, bias_walk=1e-6, apply_floor_deg_s=0.05)
    out = run_on_trajectory(traj, 5.0, ivs, cfg)
    np.testing.assert_allclose(out.bias[-1] / DEG, [0.2, -0.1, 0.15], atol=0.02)
    np.testing.assert_allclose(out.bias_applied[-1] / DEG, soft_threshold_bias(out.bias[-1] / DEG, 0.05), atol=1e-9)

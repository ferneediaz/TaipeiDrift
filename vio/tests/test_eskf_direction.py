"""Visual velocity-direction ESKF update: residual, Jacobian, speed unobservability, gating, conventions."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.data.synthetic import ImuNoise, make_synthetic_trajectory
from src.data.trajectory import quat_wxyz_to_rotation
from src.estimation.inertial_dead_reckoning import NavState
from vio.estimation.eskf import ESKF, N_ERR, TH, V_, ImuNoiseModel
from vio.estimation.eskf_runner import (
    BaroUpdateConfig,
    DirectionUpdateConfig,
    EskfInputs,
    RotationUpdateConfig,
    direction_jacobian,
    run_eskf,
)
from vio.sensors.simulated import BarometerConfig
from vio.vision.measurements import VisualMeasurement

R_BC = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
CUT = 5.0


def _traj(frame="world", scenario="constant_velocity", duration=40.0):
    return make_synthetic_trajectory(scenario, duration=duration, gyroscope_frame=frame, noise=ImuNoise(seed=1))


def _dir_meas(traj, span=100, noise_deg=0.0, seed=0):
    """Simulated camera: displacement direction over each span, camera axes at the keyframe (test fixture)."""
    rng = np.random.default_rng(seed)
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    k0 = traj.index_at(CUT)
    out = []
    for i in range(k0, len(traj) - span, span):
        j = i + span
        d = traj.position_gt[j] - traj.position_gt[i]
        t_cam = R_BC.T @ R[i].inv().apply(d / max(np.linalg.norm(d), 1e-12))
        if noise_deg:
            ax = np.cross(t_cam, rng.normal(size=3))
            t_cam = Rotation.from_rotvec(ax / np.linalg.norm(ax) * np.radians(rng.normal(0, noise_deg))).apply(t_cam)
        out.append(VisualMeasurement(j // 4, i // 4, j, i, True, [], t_cam, end_of_span=True))
    return out


# the scenarios start with a deliberately wrong velocity, so the filter is told velocity is uncertain
NOISE = ImuNoiseModel(init_vel_std=2.0)


def _run(traj, v0_rot_deg=0.0, v0_scale=1.0, meas=None, sigma=1.5, enabled=True):
    k0 = traj.index_at(CUT)
    v0 = Rotation.from_euler("z", v0_rot_deg, degrees=True).apply(traj.velocity_gt[k0]) * v0_scale
    inp = EskfInputs(traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], traj.gyroscope_frame, traj.gravity_world,
                     NavState(traj.position_gt[k0].copy(), v0, traj.attitude_gt[k0].copy()), k0,
                     rotation_measurements=_dir_meas(traj) if meas is None else meas, R_bc_forward=R_BC)
    return run_eskf(inp, noise=NOISE, rot_cfg=RotationUpdateConfig(enabled=False), dir_cfg=DirectionUpdateConfig(enabled=enabled, sigma_deg=sigma))


def _dir_err(out, traj):
    k0 = out.result.start_index
    v, g = out.result.velocity, traj.velocity_gt[k0:]
    return np.degrees(np.arccos(np.clip(np.sum(v * g, 1) / np.linalg.norm(v, axis=1) / np.linalg.norm(g, axis=1), -1, 1)))


def _speed_err(out, traj):
    k0 = out.result.start_index
    return np.linalg.norm(out.result.velocity, axis=1) - np.linalg.norm(traj.velocity_gt[k0:], axis=1)


# ---------------------------------------------------------------- residual and Jacobian
def test_residual_zero_when_directions_agree_and_speed_unobservable():
    v = np.array([3.0, 4.0, -0.5])
    r, H, B = direction_jacobian(v, v / np.linalg.norm(v), N_ERR)
    np.testing.assert_allclose(r, 0.0, atol=1e-12)
    np.testing.assert_allclose(H[:, V_] @ v, 0.0, atol=1e-12)  # no sensitivity along the velocity: speed unobserved
    np.testing.assert_allclose(B.T @ v, 0.0, atol=1e-12)


def test_jacobian_matches_finite_differences():
    rng = np.random.default_rng(0)
    v_hat = np.array([5.0, -2.0, 1.0])
    R_hat = Rotation.from_euler("xyz", [5, -10, 30], degrees=True)
    d_body = R_hat.inv().apply(v_hat / np.linalg.norm(v_hat))  # camera sees the true direction in body axes
    dv = rng.normal(0, 1e-4, 3)
    dth = rng.normal(0, 1e-5, 3)
    R_true = Rotation.from_rotvec(dth) * R_hat
    v_true = v_hat + dv
    d_body_true = R_true.inv().apply(v_true / np.linalg.norm(v_true))
    d_vis_est = R_hat.apply(d_body_true)  # computed with the estimated attitude
    r, H, _ = direction_jacobian(v_hat, d_vis_est, N_ERR)
    dx = np.zeros(N_ERR)
    dx[V_], dx[TH] = dv, dth
    np.testing.assert_allclose(r, H @ dx, rtol=1e-3, atol=1e-9)


def test_jacobian_sign_moves_velocity_towards_measurement():
    f = ESKF(np.zeros(3), np.array([5.0, 0, 0]), [1, 0, 0, 0], [0, 0, 9.81], "world")
    d_meas = Rotation.from_euler("z", 5, degrees=True).apply([1.0, 0, 0])
    r, H, _ = direction_jacobian(f.v, d_meas, f.n)
    u = f.update(r, H, np.eye(2) * np.radians(1.0) ** 2, None)
    assert u.accepted and np.degrees(np.arctan2(f.v[1], f.v[0])) > 0.5  # turned towards +5 deg


# ---------------------------------------------------------------- synthetic scenarios
@pytest.mark.parametrize("frame", ["world", "body"])
def test_correct_speed_wrong_direction_is_fixed(frame):
    traj = _traj(frame)
    off = _run(traj, v0_rot_deg=10.0, enabled=False)
    on = _run(traj, v0_rot_deg=10.0)
    assert _dir_err(off, traj)[-1] > 8.0 and _dir_err(on, traj)[-1] < 1.0


def test_wrong_speed_correct_direction_is_not_fixed():
    traj = _traj()
    off = _run(traj, v0_scale=1.3, enabled=False)
    on = _run(traj, v0_scale=1.3)
    np.testing.assert_allclose(_speed_err(on, traj), _speed_err(off, traj), atol=1e-6)  # magnitude unobserved
    assert abs(_speed_err(on, traj)[-1]) > 1.0


def test_both_wrong_direction_fixed_magnitude_not():
    traj = _traj()
    off = _run(traj, v0_rot_deg=10.0, v0_scale=1.3, enabled=False)
    on = _run(traj, v0_rot_deg=10.0, v0_scale=1.3)
    assert _dir_err(on, traj)[-1] < 1.0 < _dir_err(off, traj)[-1]
    assert abs(_speed_err(on, traj)[-1] - _speed_err(off, traj)[-1]) < 0.1  # speed error stays


def test_near_zero_velocity_is_rejected():
    traj = _traj(scenario="stationary")
    out = _run(traj)
    d = [u for u in out.updates if u.kind == "direction"]
    assert d and not any(u.accepted for u in d) and {u.reason for u in d} == {"speed below threshold"}


def test_covariance_controls_influence():
    traj = _traj()
    noisy = _dir_meas(traj, noise_deg=5.0)
    tight = _run(traj, meas=noisy, sigma=0.1)
    loose = _run(traj, meas=noisy, sigma=5.0)
    assert np.mean(_dir_err(loose, traj)) < np.mean(_dir_err(tight, traj))  # trusting noise too much hurts


def test_no_arbitrary_position_jump():
    """A direction update may move position only through the position-velocity correlation: the jump
    must point towards the truth and stay within what the velocity error could have produced."""
    traj = _traj()
    on = _run(traj, v0_rot_deg=10.0)
    k0 = on.result.start_index
    first = next(u.imu_index for u in on.updates if u.kind == "direction" and u.accepted) - k0
    before = on.result.position[first - 1] + on.result.velocity[first - 1] * 0.01  # one prediction step, no update
    after = on.result.position[first]
    truth = traj.position_gt[k0 + first]
    v_err = np.linalg.norm(traj.velocity_gt[k0] - Rotation.from_euler("z", 10, degrees=True).apply(traj.velocity_gt[k0]))
    elapsed = traj.timestamp[k0 + first] - traj.timestamp[k0]
    assert np.linalg.norm(after - truth) < np.linalg.norm(before - truth)
    assert np.linalg.norm(after - before) <= v_err * elapsed * 1.2


def test_no_ground_truth_after_cutoff():
    traj = _traj()
    meas = _dir_meas(traj)
    a = _run(traj, v0_rot_deg=10.0, meas=meas)
    k0 = traj.index_at(CUT)
    traj.position_gt[k0 + 1:] += 100.0
    traj.velocity_gt[k0 + 1:] *= -1
    traj.attitude_gt[k0 + 1:] = [1.0, 0, 0, 0]
    b = _run(traj, v0_rot_deg=10.0, meas=meas)
    np.testing.assert_array_equal(a.result.velocity, b.result.velocity)


def test_realistic_barometer_is_default():
    b, u = BarometerConfig(), BaroUpdateConfig()
    assert (b.white_std_m, b.bias_walk_m_per_sqrt_s, b.drift_sigma_m_per_s, b.scale_error_min, b.scale_error_max) == (0.30, 0.112, 0.0024, 0.03, 0.07)
    assert (u.white_std_m, u.bias_walk_m_per_sqrt_s, u.drift_sigma_m_per_s) == (0.30, 0.112, 0.0024) and u.scale_error_rms > 0

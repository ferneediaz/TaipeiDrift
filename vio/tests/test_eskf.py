"""ESKF: prediction, covariance, visual updates, gating, fallback, bias recovery, no leakage, NIS/NEES."""
import inspect

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from scipy.stats import chi2

from src.data.synthetic import ImuNoise, make_synthetic_trajectory
from src.data.trajectory import quat_wxyz_to_rotation
from src.estimation.inertial_dead_reckoning import NavState, run_dead_reckoning
from vio.estimation.eskf import ESKF, N_ERR, ImuNoiseModel, skew
from vio.estimation.eskf_runner import BaroUpdateConfig, EskfInputs, FlowUpdateConfig, RotationUpdateConfig, run_eskf
from vio.evaluation.attitude_metrics import attitude_error_series
from vio.evaluation.consistency import chi2_interval, nees, nis, state_error, summarize_chi2
from vio.sensors.simulated import BarometerConfig, InjectedBias, inject_imu_bias, simulate_barometer
from vio.vision.camera import pinhole_intrinsics
from vio.vision.measurements import VisualMeasurement
from vio.vision.optical_flow import (
    FlowConfig,
    FlowPair,
    camera_velocity_from_flow,
    derotate,
    flow_pairs_from_tracks,
    height_from_known_velocity,
)
from vio.vision.feature_tracker import TrackResult

CUT = 5.0
R_BC_LEFT = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
R_BC_DOWN = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
BIAS = ImuNoise(gyro_bias=(0.003, -0.002, 0.0025), accel_bias=(0.05, -0.04, 0.03),
                gyro_white_std=0.005, accel_white_std=0.02, seed=3)


def _traj(scenario="circle", noise=BIAS, frame="world", duration=40.0):
    return make_synthetic_trajectory(scenario, duration=duration, noise=noise, gyroscope_frame=frame)


def _inputs(traj, **kw):
    k0 = traj.index_at(CUT)
    return EskfInputs(traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], traj.gyroscope_frame,
                      traj.gravity_world, NavState(traj.position_gt[k0].copy(), traj.velocity_gt[k0].copy(),
                                                   traj.attitude_gt[k0].copy()), k0, **kw)


def perfect_rotations(traj, age=25, step=4, error_deg=0.0, invalid=False):
    """Simulated forward camera: exact keyframe-relative body rotations (test fixture)."""
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    k0 = traj.index_at(CUT)
    out, kf = [], k0
    for idx in range(k0, len(traj), step):
        if idx == kf:
            out.append(VisualMeasurement(idx // step, kf // step, idx, kf, False, new_keyframe=True))
            continue
        C = R[kf].inv() * R[idx]
        if error_deg:
            C = C * Rotation.from_rotvec([np.deg2rad(error_deg), 0, 0])
        end = idx // step - kf // step >= age
        out.append(VisualMeasurement(idx // step, kf // step, idx, kf, not invalid, [] if invalid else [C.as_matrix()],
                                     end_of_span=end, reason="invalid" if invalid else ""))
        if end:
            kf = idx
    return out


def synthetic_flow_pairs(traj, step=4, n_pts=200, seed=0):
    """Simulated down camera over flat ground (z = 0 in NED): exact normalised correspondences."""
    rng = np.random.default_rng(seed)
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    k0 = traj.index_at(CUT)
    pairs = []
    for ib in range(k0 + step, len(traj), step):
        ia = ib - step
        Rwc_a = R[ia].as_matrix() @ R_BC_DOWN
        rays = np.c_[rng.uniform(-0.5, 0.5, (n_pts, 2)), np.ones(n_pts)]
        dw = rays @ Rwc_a.T
        pa_w = traj.position_gt[ia]
        s = -pa_w[2] / dw[:, 2]  # intersect with z = 0
        P = pa_w + s[:, None] * dw

        def project(i):
            Rwc = R[i].as_matrix() @ R_BC_DOWN
            pc = (P - traj.position_gt[i]) @ Rwc
            return pc[:, :2] / pc[:, 2:3]

        pairs.append(FlowPair(ib // step, ib, ia, project(ia), project(ib), n_pts, 1.0, True))
    hag = -traj.position_gt[k0:, 2]  # exact height above the z = 0 ground (fixture only)
    return pairs, hag


def _att(out, traj):
    return attitude_error_series(out.result, traj).error_deg


# ---------------------------------------------------------------- prediction
@pytest.mark.parametrize("frame", ["body", "world"])
def test_prediction_without_updates_equals_baseline(frame):
    traj = _traj(frame=frame)
    out = run_eskf(_inputs(traj))
    base = run_dead_reckoning(traj, CUT)
    np.testing.assert_allclose(out.result.position, base.position, atol=1e-9)
    np.testing.assert_allclose(_att(out, traj), attitude_error_series(base, traj).error_deg, atol=1e-9)


def test_quaternion_propagation_matches_constant_rate():
    f = ESKF(np.zeros(3), np.zeros(3), [1, 0, 0, 0], [0, 0, 9.81], "body")
    w = np.array([0.0, 0.0, np.pi / 2])
    for _ in range(100):
        f.predict([0, 0, -9.81], [0, 0, -9.81], w, w, 0.01)
    np.testing.assert_allclose(f.R.apply([1, 0, 0]), [0, 1, 0], atol=1e-12)
    assert np.isclose(np.linalg.norm(f.R.as_quat()), 1.0)
    np.testing.assert_allclose(f.p, 0.0, atol=1e-12)  # hover: specific force cancels gravity


def test_bias_is_subtracted_in_prediction():
    f = ESKF(np.zeros(3), np.zeros(3), [1, 0, 0, 0], [0, 0, 9.81], "world", bg0=[0, 0, 0.1], ba0=[0.2, 0, 0])
    for _ in range(100):
        f.predict([0.2, 0, -9.81], [0.2, 0, -9.81], [0, 0, 0.1], [0, 0, 0.1], 0.01)
    assert f.R.magnitude() < 1e-12
    np.testing.assert_allclose(f.v, 0.0, atol=1e-12)


def test_covariance_stays_symmetric_positive_and_grows():
    f = ESKF(np.zeros(3), np.zeros(3), [1, 0, 0, 0], [0, 0, 9.81], "world")
    P0 = f.P.copy()
    for _ in range(500):
        f.predict([0.1, 0, -9.81], [0.1, 0, -9.81], [0, 0.01, 0], [0, 0.01, 0], 0.01)
    assert np.allclose(f.P, f.P.T)
    assert np.all(np.linalg.eigvalsh(f.P) > 0)
    assert np.all(np.diag(f.P)[:9] > np.diag(P0)[:9])
    # attitude variance grows at least by the gyro-bias prior times t^2
    t = 5.0
    assert f.P[6, 6] > (ImuNoiseModel().init_gyro_bias_std * t) ** 2 * 0.9


def test_attitude_gyro_bias_correlation_sign():
    """With a world-frame gyro, dtheta' = -db_g, so the cross-covariance must become negative."""
    f = ESKF(np.zeros(3), np.zeros(3), [1, 0, 0, 0], [0, 0, 9.81], "world")
    for _ in range(100):
        f.predict([0, 0, -9.81], [0, 0, -9.81], np.zeros(3), np.zeros(3), 0.01)
    assert f.P[6, 12] < 0 and f.P[7, 13] < 0 and f.P[8, 14] < 0


def test_clone_is_static_but_correlated():
    f = ESKF(np.zeros(3), np.zeros(3), [1, 0, 0, 0], [0, 0, 9.81], "world")
    f.clone_attitude()
    assert f.n == N_ERR + 3
    np.testing.assert_allclose(f.P[N_ERR:, N_ERR:], f.P[6:9, 6:9])
    Pcc = f.P[N_ERR:, N_ERR:].copy()
    Rc = f.R_clone
    for _ in range(50):
        f.predict([0, 0, -9.81], [0, 0, -9.81], [0.1, 0, 0], [0.1, 0, 0], 0.01)
    np.testing.assert_allclose(f.P[N_ERR:, N_ERR:], Pcc)
    assert (f.R_clone.inv() * Rc).magnitude() == 0.0
    assert np.abs(f.P[6:9, N_ERR:]).max() > 0


# ---------------------------------------------------------------- relative rotation
@pytest.mark.parametrize("frame", ["body", "world"])
def test_relative_rotation_recovers_gyro_bias(frame):
    traj = _traj(frame=frame, duration=60.0)
    out = run_eskf(_inputs(traj, rotation_measurements=perfect_rotations(traj), R_bc_forward=R_BC_LEFT),
                   rot_cfg=RotationUpdateConfig(sigma_deg=0.05))
    true_bg = np.array(BIAS.gyro_bias)
    assert np.linalg.norm(out.gyro_bias[-1] - true_bg) < 0.25 * np.linalg.norm(true_bg)
    imu = run_eskf(_inputs(traj))
    assert _att(out, traj)[-1] < 0.3 * _att(imu, traj)[-1]


def test_relative_rotation_jacobian_numerically():
    f = ESKF(np.zeros(3), np.zeros(3), [1, 0, 0, 0], [0, 0, 9.81], "world")
    f.R = Rotation.from_euler("xyz", [10, -20, 30], degrees=True)
    f.clone_attitude()
    f.R = Rotation.from_euler("xyz", [12, -18, 35], degrees=True)
    C_true = (f.R_clone.inv() * f.R).as_matrix()
    d_now, d_clone = np.array([1e-4, -2e-4, 1.5e-4]), np.array([-1e-4, 0.5e-4, 2e-4])
    R_now_true, R_c_true = Rotation.from_rotvec(d_now) * f.R, Rotation.from_rotvec(d_clone) * f.R_clone
    C_meas = (R_c_true.inv() * R_now_true).as_matrix()
    r = (Rotation.from_matrix(C_true).inv() * Rotation.from_matrix(C_meas)).as_rotvec()
    RnT = f.R.as_matrix().T
    np.testing.assert_allclose(r, RnT @ (d_now - d_clone), atol=1e-7)


def test_velocity_jacobian_numerically():
    f = ESKF(np.zeros(3), np.array([5.0, -2.0, 1.0]), [1, 0, 0, 0], [0, 0, 9.81], "world")
    f.R = Rotation.from_euler("xyz", [5, -10, 40], degrees=True)
    M = (R_BC_DOWN.T @ f.R.as_matrix().T)[:2]
    H_th = M @ skew(f.v)
    d = np.array([1e-5, -2e-5, 3e-5])
    z_true = (R_BC_DOWN.T @ (Rotation.from_rotvec(d) * f.R).as_matrix().T @ f.v)[:2]
    np.testing.assert_allclose(z_true - M @ f.v, H_th @ d, rtol=1e-3)  # equal to first order


def test_rotation_update_needs_clone():
    f = ESKF(np.zeros(3), np.zeros(3), [1, 0, 0, 0], [0, 0, 9.81], "world")
    u = f.update_relative_rotation(np.eye(3), 0.01)
    assert not u.accepted and "clone" in u.reason


# ---------------------------------------------------------------- optical flow
def test_flow_velocity_from_exact_projection():
    traj = _traj("circle", noise=ImuNoise())
    pairs, hag = synthetic_flow_pairs(traj)
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    k0 = traj.index_at(CUT)
    p = pairs[10]
    Ra, Rb = R[p.prev_imu_index].as_matrix(), R[p.imu_index].as_matrix()
    R_ab = R_BC_DOWN.T @ Ra.T @ Rb @ R_BC_DOWN
    n_cam = (Ra @ R_BC_DOWN).T @ [0, 0, 1]
    fv = camera_velocity_from_flow(p.xa, p.xb, R_ab, n_cam, hag[p.prev_imu_index - k0], 0.04, 256.0)
    v_true = (R_BC_DOWN.T @ Rb.T @ (traj.position_gt[p.imu_index] - traj.position_gt[p.prev_imu_index]) / 0.04)
    np.testing.assert_allclose(fv.v_cam, v_true, atol=5e-3)  # camera b axes


def test_height_from_known_velocity():
    traj = _traj("constant_velocity", noise=ImuNoise())
    pairs, hag = synthetic_flow_pairs(traj)
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    p = pairs[5]
    Ra, Rb = R[p.prev_imu_index].as_matrix(), R[p.imu_index].as_matrix()
    t_cam = (Ra @ R_BC_DOWN).T @ (traj.position_gt[p.imu_index] - traj.position_gt[p.prev_imu_index])
    h = height_from_known_velocity(p.xa, p.xb, R_BC_DOWN.T @ Ra.T @ Rb @ R_BC_DOWN, (Ra @ R_BC_DOWN).T @ [0, 0, 1], t_cam)
    assert h == pytest.approx(-traj.position_gt[p.prev_imu_index, 2], rel=5e-3)


def test_derotation_removes_pure_rotation():
    rng = np.random.default_rng(1)
    xa = rng.uniform(-0.5, 0.5, (50, 2))
    R_ab = Rotation.from_euler("xyz", [1, -2, 3], degrees=True).as_matrix()
    rays = np.c_[xa, np.ones(50)] @ R_ab
    xb = rays[:, :2] / rays[:, 2:3]
    np.testing.assert_allclose(derotate(xa, xb, R_ab), 0.0, atol=1e-12)


def _baro_change(traj):
    b = simulate_barometer(traj, BarometerConfig(seed=2))
    k0 = traj.index_at(CUT)
    return b.altitude_m[k0:] - b.altitude_m[k0]


FLOW_OK = FlowUpdateConfig(rel_height_std=0.02, dir_sigma_deg=0.2, every_n_frames=5)


@pytest.mark.parametrize("scenario", ["circle", "constant_velocity"])
def test_flow_update_reduces_horizontal_drift(scenario):
    """Down-camera velocity observes horizontal motion; the barometer holds the vertical channel."""
    traj = _traj(scenario)
    pairs, hag = synthetic_flow_pairs(traj)
    dalt = _baro_change(traj)
    ref = run_eskf(_inputs(traj, baro_altitude_change=dalt), baro_cfg=BaroUpdateConfig())
    out = run_eskf(_inputs(traj, flow_pairs=pairs, R_bc_down=R_BC_DOWN, focal_px_down=256.0, height_above_ground=hag,
                           baro_altitude_change=dalt), flow_cfg=FLOW_OK, baro_cfg=BaroUpdateConfig())
    h = lambda o: np.linalg.norm(o.result.position[-1, :2] - traj.position_gt[-1, :2])  # noqa: E731
    assert h(out) < 0.5 * h(ref)
    assert abs(out.result.position[-1, 2] - traj.position_gt[-1, 2]) < 2.0
    flow = [u for u in out.updates if u.kind == "flow"]
    assert sum(u.accepted for u in flow) > 0.75 * len(flow)


def test_barometer_holds_altitude():
    traj = _traj("constant_velocity")
    imu = run_eskf(_inputs(traj))
    out = run_eskf(_inputs(traj, baro_altitude_change=_baro_change(traj)), baro_cfg=BaroUpdateConfig())
    v = lambda o: abs(o.result.position[-1, 2] - traj.position_gt[-1, 2])  # noqa: E731
    assert v(out) < 1.0 < v(imu)


def test_flow_pair_quality_checks():
    K = pinhole_intrinsics(512, 512, 90.0)
    rng = np.random.default_rng(0)
    few = TrackResult(1, 0, rng.uniform(0, 512, (10, 2)).astype(np.float32), rng.uniform(0, 512, (10, 2)).astype(np.float32), 10, False)
    pa = rng.uniform(0, 512, (200, 2)).astype(np.float32)
    noise = TrackResult(2, 1, pa, rng.uniform(0, 512, (200, 2)).astype(np.float32), 200, False)
    good = TrackResult(3, 2, pa, pa + np.float32([3.0, -1.0]), 200, False)
    pairs = flow_pairs_from_tracks([few, noise, good], K, lambda i: 4 * i, FlowConfig())
    assert [p.valid for p in pairs] == [False, False, True]
    assert pairs[0].reason == "too few tracks" and pairs[1].reason == "low inlier ratio"
    assert (pairs[2].imu_index, pairs[2].prev_imu_index) == (12, 8)  # frame i at IMU sample 4 i


def test_flow_fit_rejects_inconsistent_flow():
    rng = np.random.default_rng(2)
    xa = rng.uniform(-0.5, 0.5, (100, 2))
    xb = xa + rng.normal(0, 0.05, (100, 2))  # incoherent motion, ~13 px rms at f = 256
    assert camera_velocity_from_flow(xa, xb, np.eye(3), np.array([0, 0, 1.0]), 30.0, 0.04, 256.0) is None


# ---------------------------------------------------------------- gating and fallback
def test_mahalanobis_gate_rejects_outlier_and_keeps_state():
    f = ESKF(np.zeros(3), np.array([5.0, 0, 0]), [1, 0, 0, 0], [0, 0, 9.81], "world")
    p, v, P = f.p.copy(), f.v.copy(), f.P.copy()
    u = f.update_camera_velocity_xy(np.array([100.0, 100.0]), R_BC_DOWN, np.eye(2) * 0.01)
    assert not u.accepted and u.reason == "Mahalanobis gate" and u.nis > chi2.ppf(0.99, 2)
    np.testing.assert_array_equal(f.p, p)
    np.testing.assert_array_equal(f.v, v)
    np.testing.assert_array_equal(f.P, P)


def test_wrong_rotation_measurements_are_gated():
    traj = _traj()
    out = run_eskf(_inputs(traj, rotation_measurements=perfect_rotations(traj, error_deg=20.0), R_bc_forward=R_BC_LEFT),
                   rot_cfg=RotationUpdateConfig(sigma_deg=0.1))
    imu = run_eskf(_inputs(traj))
    assert not any(u.accepted for u in out.updates)
    np.testing.assert_allclose(out.result.position, imu.result.position, atol=1e-9)


def test_failed_vision_falls_back_to_imu():
    traj = _traj()
    pairs, hag = synthetic_flow_pairs(traj)
    for p in pairs:
        p.valid, p.reason = False, "low inlier ratio"
    out = run_eskf(_inputs(traj, rotation_measurements=perfect_rotations(traj, invalid=True), R_bc_forward=R_BC_LEFT,
                           flow_pairs=pairs, R_bc_down=R_BC_DOWN, focal_px_down=256.0, height_above_ground=hag),
                   rot_cfg=RotationUpdateConfig(), flow_cfg=FlowUpdateConfig())
    imu = run_eskf(_inputs(traj))
    np.testing.assert_allclose(out.result.position, imu.result.position, atol=1e-9)
    assert out.updates and not any(u.accepted for u in out.updates)


def test_update_timing_matches_imu_index():
    traj = _traj()
    meas = perfect_rotations(traj)
    out = run_eskf(_inputs(traj, rotation_measurements=meas, R_bc_forward=R_BC_LEFT), rot_cfg=RotationUpdateConfig())
    expected = [m.imu_index for m in meas if m.end_of_span]
    assert [u.imu_index for u in out.updates] == expected


# ---------------------------------------------------------------- bias recovery and injection
def test_injected_bias_recovery_with_both_cameras():
    base = _traj(noise=ImuNoise(gyro_white_std=0.005, accel_white_std=0.02, seed=4), duration=60.0)
    inj = InjectedBias(gyro=(0.004, -0.003, 0.002), accel=(0.05, -0.04, 0.03))
    traj = inject_imu_bias(base, inj)
    pairs, hag = synthetic_flow_pairs(traj)
    out = run_eskf(_inputs(traj, rotation_measurements=perfect_rotations(traj), R_bc_forward=R_BC_LEFT,
                           flow_pairs=pairs, R_bc_down=R_BC_DOWN, focal_px_down=256.0, height_above_ground=hag,
                           baro_altitude_change=_baro_change(traj)),
                   rot_cfg=RotationUpdateConfig(sigma_deg=0.05), flow_cfg=FLOW_OK, baro_cfg=BaroUpdateConfig())
    assert np.linalg.norm(out.gyro_bias[-1] - inj.gyro) < 0.25 * np.linalg.norm(inj.gyro)
    # accelerometer bias: report-only in the README; here only require it moved towards the truth
    assert np.linalg.norm(out.accel_bias[-1] - inj.accel) < np.linalg.norm(inj.accel)
    np.testing.assert_array_equal(traj.position_gt, base.position_gt)  # injection touches only the IMU


def test_inject_bias_adds_exact_offsets():
    base = _traj(noise=ImuNoise())
    traj = inject_imu_bias(base, InjectedBias((0.01, 0, 0), (0, 0.2, 0)))
    np.testing.assert_allclose(traj.gyroscope - base.gyroscope, np.tile([0.01, 0, 0], (len(base), 1)))
    np.testing.assert_allclose(traj.accelerometer - base.accelerometer, np.tile([0, 0.2, 0], (len(base), 1)))
    assert traj.metadata["injected_bias"]["gyro"] == [0.01, 0, 0]


def test_simulated_barometer_is_deterministic_and_noisy():
    traj = _traj("constant_velocity", noise=ImuNoise())
    a = simulate_barometer(traj, BarometerConfig(seed=1))
    b = simulate_barometer(traj, BarometerConfig(seed=1))
    np.testing.assert_array_equal(a.altitude_m, b.altitude_m)
    err = a.altitude_m - (-traj.position_gt[:, 2])
    assert 0.1 < np.std(np.diff(err)) < 1.0 and not np.allclose(err, 0)


# ---------------------------------------------------------------- leakage
def test_no_ground_truth_after_cutoff_reaches_the_filter():
    traj = _traj()
    meas = perfect_rotations(traj)
    pairs, hag = synthetic_flow_pairs(traj)
    kw = dict(rotation_measurements=meas, R_bc_forward=R_BC_LEFT, flow_pairs=pairs, R_bc_down=R_BC_DOWN,
              focal_px_down=256.0, height_above_ground=hag)
    a = run_eskf(_inputs(traj, **kw), rot_cfg=RotationUpdateConfig(), flow_cfg=FlowUpdateConfig())
    k0 = traj.index_at(CUT)
    traj.position_gt[k0 + 1:] += 1000.0
    traj.velocity_gt[k0 + 1:] *= -1
    traj.attitude_gt[k0 + 1:] = [1.0, 0, 0, 0]
    b = run_eskf(_inputs(traj, **kw), rot_cfg=RotationUpdateConfig(), flow_cfg=FlowUpdateConfig())
    np.testing.assert_array_equal(a.result.position, b.result.position)
    np.testing.assert_array_equal(a.gyro_bias, b.gyro_bias)


def test_runner_interface_takes_no_trajectory():
    fields = set(EskfInputs.__dataclass_fields__)
    assert not {"position_gt", "velocity_gt", "attitude_gt", "traj", "trajectory"} & fields
    assert "Trajectory" not in inspect.getsource(run_eskf)


# ---------------------------------------------------------------- NIS / NEES
def test_nis_and_nees_values():
    assert nis(np.array([1.0, 2.0]), np.diag([1.0, 4.0])) == pytest.approx(2.0)
    assert nees(np.array([3.0]), np.array([[9.0]])) == pytest.approx(1.0)
    lo, hi = chi2_interval(3)
    assert lo < 3 < hi
    s = summarize_chi2(np.random.default_rng(0).chisquare(3, 5000), 3)
    assert s.mean == pytest.approx(3.0, rel=0.05) and s.fraction_inside_95 == pytest.approx(0.95, abs=0.02)


def test_state_error_convention():
    q_est = np.array([[1.0, 0, 0, 0]])
    q_gt = Rotation.from_rotvec([0, 0, 0.1]).as_quat()[[3, 0, 1, 2]][None]
    e = state_error(np.zeros((1, 3)), np.zeros((1, 3)), q_est, np.ones((1, 3)), 2 * np.ones((1, 3)), q_gt)
    np.testing.assert_allclose(e[0], [1, 1, 1, 2, 2, 2, 0, 0, 0.1], atol=1e-12)


def test_filter_is_consistent_on_synthetic_imu_only():
    """NEES of a correctly modelled IMU-only filter should be near its dimension (not overconfident)."""
    noise = ImuNoiseModel(gyro_noise=0.005 * 0.1, accel_noise=0.02 * 0.1)
    traj = _traj(noise=ImuNoise(gyro_bias=(0.002, -0.001, 0.001), gyro_white_std=0.005, accel_white_std=0.02, seed=7))
    out = run_eskf(_inputs(traj), noise=noise)
    k0 = out.result.start_index
    idx = out.nav_cov_index[1:]
    e = state_error(out.result.position[idx], out.result.velocity[idx], out.result.attitude[idx],
                    traj.position_gt[k0 + idx], traj.velocity_gt[k0 + idx], traj.attitude_gt[k0 + idx])
    vals = [nees(e[i, 6:9], out.nav_cov[j + 1][6:9, 6:9]) for i, j in enumerate(range(len(idx)))]
    assert np.mean(vals) < 3 * 3

"""Visual attitude fusion: equivalence with the baseline, correction, fallback, gating, no leakage."""
import inspect

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import vio.estimation.visual_attitude_fusion as fusion_module
from src.data.synthetic import ImuNoise, make_synthetic_trajectory
from src.data.trajectory import quat_wxyz_to_rotation
from src.estimation.inertial_dead_reckoning import run_dead_reckoning
from vio.estimation.visual_attitude_fusion import FusionConfig, run_visual_attitude_fusion
from vio.evaluation.attitude_metrics import attitude_error_series
from vio.pipeline import run_fusion_on_trajectory
from vio.vision.measurements import VisualMeasurement

CUTOFF = 5.0
BIAS = ImuNoise(gyro_bias=(0.003, -0.002, 0.0025))


def _traj(frame="world", scenario="circle"):
    return make_synthetic_trajectory(scenario, duration=40.0, noise=BIAS, gyroscope_frame=frame)


def perfect_camera(traj, k0, step=4, age=25, error_deg=0.0, every_invalid=False):
    """Simulated camera: true relative body rotation between keyframe and frame (test fixture only)."""
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    out, kf = [], k0
    for idx in range(k0, len(traj), step):
        frame, kf_frame = idx // step, kf // step
        if idx == kf:
            out.append(VisualMeasurement(frame, kf_frame, idx, kf, False, new_keyframe=True, reason="new keyframe"))
            continue
        C = (R[kf].inv() * R[idx])
        if error_deg:
            C = C * Rotation.from_rotvec([np.deg2rad(error_deg), 0, 0])
        out.append(VisualMeasurement(frame, kf_frame, idx, kf, not every_invalid,
                                     [] if every_invalid else [C.as_matrix()], reason="invalid" if every_invalid else ""))
        if frame - kf_frame >= age:
            kf = idx
    return out


def _att_err(res, traj):
    return attitude_error_series(res, traj).error_deg


@pytest.mark.parametrize("frame", ["body", "world"])
def test_gain_zero_reproduces_baseline_exactly(frame):
    traj = _traj(frame)
    k0 = traj.index_at(CUTOFF)
    res, _ = run_fusion_on_trajectory(traj, CUTOFF, perfect_camera(traj, k0), FusionConfig(gain=0.0))
    base = run_dead_reckoning(traj, CUTOFF)
    np.testing.assert_allclose(res.position, base.position, atol=1e-9)
    np.testing.assert_allclose(_att_err(res, traj), _att_err(base, traj), atol=1e-9)


@pytest.mark.parametrize("frame", ["body", "world"])
def test_perfect_camera_removes_most_attitude_drift(frame):
    traj = _traj(frame)
    k0 = traj.index_at(CUTOFF)
    base = _att_err(run_dead_reckoning(traj, CUTOFF), traj)
    res, events = run_fusion_on_trajectory(traj, CUTOFF, perfect_camera(traj, k0), FusionConfig(gain=0.5, max_disagreement_deg=5.0))
    assert base[-1] > 3.0
    assert _att_err(res, traj)[-1] < 0.1 * base[-1]
    assert all(e.accepted for e in events if e.reason != "new keyframe")


def test_invalid_visual_frames_fall_back_to_imu():
    traj = _traj()
    k0 = traj.index_at(CUTOFF)
    res, events = run_fusion_on_trajectory(traj, CUTOFF, perfect_camera(traj, k0, every_invalid=True), FusionConfig(gain=0.5))
    base = run_dead_reckoning(traj, CUTOFF)
    np.testing.assert_allclose(res.position, base.position, atol=1e-9)
    assert not any(e.accepted for e in events)


def test_partial_outage_still_runs_and_helps():
    traj = _traj()
    k0 = traj.index_at(CUTOFF)
    meas = perfect_camera(traj, k0)
    for m in meas[len(meas) // 3 : 2 * len(meas) // 3]:  # camera blind for the middle third
        m.valid, m.body_rotation_candidates, m.reason = False, [], "too few correspondences"
    res, _ = run_fusion_on_trajectory(traj, CUTOFF, meas, FusionConfig(gain=0.5, max_disagreement_deg=5.0))
    base = _att_err(run_dead_reckoning(traj, CUTOFF), traj)
    assert np.all(np.isfinite(res.position))
    assert _att_err(res, traj)[-1] < base[-1]


def test_gate_rejects_rotation_far_from_gyro():
    traj = _traj()
    k0 = traj.index_at(CUTOFF)
    res, events = run_fusion_on_trajectory(traj, CUTOFF, perfect_camera(traj, k0, error_deg=10.0),
                                           FusionConfig(gain=0.5, max_disagreement_deg=2.0))
    np.testing.assert_allclose(res.position, run_dead_reckoning(traj, CUTOFF).position, atol=1e-9)
    assert {e.reason for e in events if e.reason != "new keyframe"} == {"disagrees with gyro"}


def test_closest_candidate_is_chosen():
    traj = _traj()
    k0 = traj.index_at(CUTOFF)
    meas = perfect_camera(traj, k0)
    wrong = Rotation.from_euler("x", 30, degrees=True).as_matrix()
    for m in meas:
        if m.body_rotation_candidates:
            m.body_rotation_candidates = [wrong @ m.body_rotation_candidates[0], m.body_rotation_candidates[0]]
    res, _ = run_fusion_on_trajectory(traj, CUTOFF, meas, FusionConfig(gain=0.5, max_disagreement_deg=5.0))
    assert _att_err(res, traj)[-1] < 0.5


def test_quaternion_sign_of_initial_state_does_not_matter():
    traj = _traj()
    k0 = traj.index_at(CUTOFF)
    meas = perfect_camera(traj, k0)
    a, _ = run_fusion_on_trajectory(traj, CUTOFF, meas, FusionConfig(gain=0.3))
    traj.attitude_gt[k0] *= -1.0  # same rotation, opposite sign
    b, _ = run_fusion_on_trajectory(traj, CUTOFF, meas, FusionConfig(gain=0.3))
    np.testing.assert_allclose(a.position, b.position, atol=1e-9)
    np.testing.assert_allclose(_att_err(a, traj), _att_err(b, traj), atol=1e-9)


def test_ground_truth_after_cutoff_is_not_used():
    """Measurements are fixed beforehand; corrupting ground truth after t0 must not change the estimate."""
    traj = _traj()
    k0 = traj.index_at(CUTOFF)
    meas = perfect_camera(traj, k0)
    a, _ = run_fusion_on_trajectory(traj, CUTOFF, meas, FusionConfig(gain=0.3))
    traj.position_gt[k0 + 1:] += 500.0
    traj.velocity_gt[k0 + 1:] *= -1.0
    traj.attitude_gt[k0 + 1:] = [1.0, 0.0, 0.0, 0.0]
    b, _ = run_fusion_on_trajectory(traj, CUTOFF, meas, FusionConfig(gain=0.3))
    np.testing.assert_array_equal(a.position, b.position)
    np.testing.assert_array_equal(a.attitude, b.attitude)


def test_estimator_interface_has_no_ground_truth_access():
    params = set(inspect.signature(run_visual_attitude_fusion).parameters)
    assert params == {"timestamp", "accelerometer", "gyroscope", "initial", "gravity_world",
                      "gyroscope_frame", "measurements", "start_index", "cfg"}
    assert "oracle" not in inspect.getsource(fusion_module).lower().replace("oracle.py", "")


def test_gain_out_of_range():
    traj = _traj()
    with pytest.raises(ValueError, match="gain"):
        run_fusion_on_trajectory(traj, CUTOFF, [], FusionConfig(gain=1.5))

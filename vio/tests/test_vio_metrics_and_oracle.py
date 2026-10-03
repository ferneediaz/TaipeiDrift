"""Attitude error metric and isolation of the ground-truth-attitude oracle."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.data.synthetic import ImuNoise, make_synthetic_trajectory
from src.data.trajectory import rotation_to_quat_wxyz
from src.estimation.inertial_dead_reckoning import run_dead_reckoning
from vio.estimation.oracle import ORACLE_LABEL, oracle_ground_truth_attitude
from vio.evaluation.attitude_metrics import (
    attitude_at_horizons,
    attitude_error_series,
    geodesic_angle_deg,
    summarize_attitude,
)


def test_geodesic_angle_known_rotation():
    q0 = np.array([1.0, 0, 0, 0])
    q30 = rotation_to_quat_wxyz(Rotation.from_euler("z", 30, degrees=True))
    assert geodesic_angle_deg(q0, q30) == pytest.approx(30.0)
    assert geodesic_angle_deg(q30, q0) == pytest.approx(30.0)


def test_geodesic_angle_ignores_quaternion_sign():
    q = rotation_to_quat_wxyz(Rotation.from_euler("xyz", [10, -20, 35], degrees=True))
    assert geodesic_angle_deg(q, -q) == pytest.approx(0.0, abs=1e-9)
    r = rotation_to_quat_wxyz(Rotation.from_euler("xyz", [12, -20, 35], degrees=True))
    assert geodesic_angle_deg(q, r) == pytest.approx(geodesic_angle_deg(-q, r))


def test_geodesic_angle_near_180_and_tiny():
    q0 = np.array([1.0, 0, 0, 0])
    assert geodesic_angle_deg(q0, np.array([0.0, 1, 0, 0])) == pytest.approx(180.0)
    tiny = rotation_to_quat_wxyz(Rotation.from_rotvec([1e-7, 0, 0]))
    assert geodesic_angle_deg(q0, tiny) == pytest.approx(np.degrees(1e-7), rel=1e-6)


def test_geodesic_angle_vectorised():
    q = rotation_to_quat_wxyz(Rotation.from_euler("z", [0, 10, 20], degrees=True))
    np.testing.assert_allclose(geodesic_angle_deg(q, np.tile([1.0, 0, 0, 0], (3, 1))), [0, 10, 20], atol=1e-9)


def test_attitude_summary_and_horizons():
    traj = make_synthetic_trajectory("circle", duration=30.0, noise=ImuNoise(gyro_bias=(0.002, 0, 0)), gyroscope_frame="world")
    s = attitude_error_series(run_dead_reckoning(traj, 5.0), traj)
    summ = summarize_attitude(s)
    assert s.error_deg[0] == pytest.approx(0.0, abs=1e-9)
    assert summ.final_deg > 0 and summ.max_deg >= summ.final_deg - 1e-12
    h = attitude_at_horizons(s, [10.0, 100.0])
    assert h[100.0] is None and h[10.0] > 0


# ---------- oracle ----------

def test_oracle_is_exact_with_ideal_accelerometer_and_any_gyro():
    """Oracle ignores the gyro entirely: a heavily biased gyro does not affect it."""
    traj = make_synthetic_trajectory("circle", duration=30.0, noise=ImuNoise(gyro_bias=(0.05, -0.05, 0.05)))
    res = oracle_ground_truth_attitude(traj, 5.0)
    assert np.max(np.linalg.norm(res.position - traj.position_gt[res.start_index:], axis=1)) < 1e-2


def test_oracle_uses_ground_truth_and_is_labelled():
    """The oracle DOES read ground truth after t0 (that is the point) and says so."""
    traj = make_synthetic_trajectory("circle", duration=20.0)
    a = oracle_ground_truth_attitude(traj, 5.0)
    traj.attitude_gt[a.start_index + 1:] = [1.0, 0, 0, 0]
    b = oracle_ground_truth_attitude(traj, 5.0)
    assert not np.allclose(a.position, b.position)
    assert "ORACLE" in ORACLE_LABEL and "not an estimator" in ORACLE_LABEL


def test_oracle_does_not_modify_the_trajectory():
    traj = make_synthetic_trajectory("circle", duration=20.0)
    before = traj.attitude_gt.copy()
    res = oracle_ground_truth_attitude(traj, 5.0)
    res.attitude[:] = 0.0
    np.testing.assert_array_equal(traj.attitude_gt, before)

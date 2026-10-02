import numpy as np
import pytest

from src.data.synthetic import make_synthetic_trajectory
from src.estimation.inertial_dead_reckoning import DeadReckoningResult
from src.evaluation.trajectory_metrics import (
    ErrorSeries,
    error_at_horizons,
    error_series,
    position_errors,
    summarize,
    time_to_exceed,
)


def _series(errors, dt=1.0):
    e = np.asarray(errors, dtype=float)
    return ErrorSeries(np.arange(len(e)) * dt, e, e, np.zeros_like(e))


def test_position_errors_euclidean():
    p_est = np.array([[3.0, 4.0, 0.0], [0.0, 0.0, 2.0]])
    np.testing.assert_allclose(position_errors(p_est, np.zeros((2, 3))), [5.0, 2.0])


def test_summary_values():
    s = summarize(_series([0.0, 3.0, 4.0]))
    assert s.final == 4.0
    assert s.max == 4.0
    assert s.mean == pytest.approx(7.0 / 3.0)
    assert s.rmse == pytest.approx(np.sqrt(25.0 / 3.0))
    assert s.duration_s == 2.0


def test_error_at_horizons_interpolates():
    out = error_at_horizons(_series([0.0, 2.0, 4.0]), [0.5, 2.0, 3.0])
    assert out[0.5] == pytest.approx(1.0)
    assert out[2.0] == pytest.approx(4.0)
    assert out[3.0] is None


def test_time_to_exceed():
    out = time_to_exceed(_series([0.0, 0.5, 2.0, 11.0]), [1.0, 10.0, 100.0])
    assert out == {1.0: 2.0, 10.0: 3.0, 100.0: None}


def test_error_series_aligns_with_start_index():
    traj = make_synthetic_trajectory("constant_velocity", duration=10.0)
    k0 = 300
    offset = np.array([3.0, 4.0, 12.0])
    res = DeadReckoningResult(
        timestamp=traj.timestamp[k0:],
        position=traj.position_gt[k0:] + offset,
        velocity=traj.velocity_gt[k0:],
        attitude=traj.attitude_gt[k0:],
        start_index=k0,
        t0=float(traj.timestamp[k0]),
    )
    s = error_series(res, traj)
    assert s.time_since_loss[0] == 0.0
    np.testing.assert_allclose(s.error, 13.0)
    np.testing.assert_allclose(s.horizontal, 5.0)
    np.testing.assert_allclose(s.vertical, 12.0)

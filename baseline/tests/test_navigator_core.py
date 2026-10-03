"""The small pieces of the navigator, checked with numbers that can be followed by hand."""
import numpy as np
import pytest

from src.estimation.navigator_core import (
    DEGRADED,
    DISAGREES_WITH_ESTIMATE,
    LOST,
    LOW_SCORE,
    OK,
    TRACKING,
    allowed_distance,
    blend,
    fit_motion_matrix,
    fix_decision,
    predicted_variance,
    status,
)


def test_motion_matrix_is_recovered_from_noisy_steps():
    rng = np.random.default_rng(0)
    true_matrix = np.array([[-0.06, -0.48], [0.002, -0.10]])  # the size of the one learned on ALTO
    shifts = rng.normal(0.0, 5.0, (200, 2))
    steps = shifts @ true_matrix + rng.normal(0.0, 0.01, (200, 2))
    np.testing.assert_allclose(fit_motion_matrix(shifts, steps), true_matrix, atol=1e-3)


def test_motion_matrix_example_from_the_docstring():
    # every frame shifts by (10, 0) or (0, 10) pixels; 10 pixels to the right mean 2.5 m east
    shifts = np.array([[10.0, 0.0], [0.0, 10.0], [10.0, 0.0], [0.0, 10.0]])
    steps = np.array([[0.0, 2.5], [-2.5, 0.0], [0.0, 2.5], [-2.5, 0.0]])  # (north, east)
    matrix = fit_motion_matrix(shifts, steps)
    np.testing.assert_allclose(np.array([20.0, 0.0]) @ matrix, [0.0, 5.0], atol=1e-9)


def test_predicted_variance_adds_drift_as_variance():
    assert predicted_variance(3.0**2, 300.0, 0.10) == pytest.approx(9.0 + 900.0)
    assert predicted_variance(25.0, 0.0, 0.10) == 25.0


def test_blend_example_from_the_docstring():
    estimate, variance, gain = blend(np.array([0.0, 0.0]), 900.0, np.array([100.0, 50.0]), 225.0)
    assert gain == pytest.approx(0.8)
    np.testing.assert_allclose(estimate, [80.0, 40.0])
    assert variance == pytest.approx(180.0)


def test_blend_with_equal_certainty_meets_in_the_middle():
    estimate, variance, gain = blend(np.array([10.0, 0.0]), 100.0, np.array([20.0, 0.0]), 100.0)
    assert gain == pytest.approx(0.5)
    np.testing.assert_allclose(estimate, [15.0, 0.0])
    assert variance == pytest.approx(50.0)


def test_blend_never_increases_the_uncertainty():
    for variance in (1.0, 50.0, 4000.0):
        _, new_variance, _ = blend(np.zeros(2), variance, np.ones(2), 225.0)
        assert new_variance < variance
        assert new_variance < 225.0


def test_allowed_distance_example_from_the_docstring():
    assert allowed_distance(900.0, 225.0, 3.0) == pytest.approx(100.62, abs=0.01)


def test_fix_decision_reasons():
    assert fix_decision(score=0.5, distance=20.0, allowed=100.0, min_score=0.33) == (True, OK)
    assert fix_decision(score=0.2, distance=20.0, allowed=100.0, min_score=0.33) == (False, LOW_SCORE)
    assert fix_decision(score=0.5, distance=150.0, allowed=100.0, min_score=0.33) == (False, DISAGREES_WITH_ESTIMATE)
    # a low score is reported first, wherever the match lies
    assert fix_decision(score=0.2, distance=150.0, allowed=100.0, min_score=0.33) == (False, LOW_SCORE)
    # the limits themselves pass
    assert fix_decision(score=0.33, distance=100.0, allowed=100.0, min_score=0.33) == (True, OK)


def test_status_follows_the_uncertainty():
    assert status(5.0) == TRACKING
    assert status(30.0) == TRACKING
    assert status(31.0) == DEGRADED
    assert status(101.0) == LOST
    assert status(40.0, degraded_above=50.0, lost_above=60.0) == TRACKING

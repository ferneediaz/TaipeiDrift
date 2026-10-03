import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim" / "nodes"))
sys.path.insert(0, str(ROOT / "baseline"))
sys.path.insert(0, str(ROOT))

from gnss_velocity_fit import fit_position_velocity
from vio.estimation.eskf import ESKF


def fixes(times, velocity=(1.2, -0.4, 0.1), covariance=None, noise=None):
    covariance = np.diag([2.25, 2.25, 9.0]) if covariance is None else covariance
    origin = np.array([4.0, -2.0, 8.0])
    rng = np.random.default_rng(21)
    result = []
    for t in times:
        p = origin + np.asarray(velocity) * t
        if noise is not None:
            p = p + rng.multivariate_normal(np.zeros(3), covariance)
        result.append((t, p, covariance))
    return result


def test_constant_velocity_gnss_fit_recovers_three_axis_velocity():
    result = fit_position_velocity(fixes(50.0 + np.arange(9, dtype=float)))
    assert result["reason"] == "ok"
    np.testing.assert_allclose(result["velocity"], [1.2, -0.4, 0.1], atol=1e-10)
    assert result["span_s"] == 8.0


def test_stationary_noisy_fixes_do_not_imply_large_horizontal_speed():
    cov = np.diag([2.25, 2.25, 9.0])
    samples = []
    rng = np.random.default_rng(22)
    for t in range(9):
        samples.append((float(t), rng.multivariate_normal([0, 0, 0], cov), cov))
    result = fit_position_velocity(samples)
    assert result["reason"] == "ok"
    assert np.linalg.norm(result["velocity"][:2]) < 0.8


def test_velocity_covariance_decreases_with_longer_span_and_more_samples():
    cov = np.diag([2.25, 2.25, 9.0])
    short = fit_position_velocity(fixes([0, 1, 2, 3, 4, 5], covariance=cov),
                                  window_s=8, min_samples=6, min_span_s=5)
    sparse_same_span = fit_position_velocity(fixes(np.linspace(0, 8, 6), covariance=cov))
    long_span = fit_position_velocity(fixes(np.arange(9, dtype=float), covariance=cov))
    assert np.trace(long_span["covariance"]) < np.trace(short["covariance"])
    assert np.trace(long_span["covariance"]) < np.trace(sparse_same_span["covariance"])
    noisier = fit_position_velocity(fixes(np.arange(9, dtype=float), covariance=2 * cov))
    np.testing.assert_allclose(np.trace(noisier["covariance"]), 2 * np.trace(long_span["covariance"]))


def test_insufficient_or_nonmonotonic_history_is_not_used():
    assert fit_position_velocity(fixes([0, 1, 2, 3, 4]))["reason"] == "insufficient_samples"
    assert fit_position_velocity(fixes([0, 1, 2, 3, 4, 3]))["reason"] == "timestamps_not_strictly_increasing"
    too_short = fit_position_velocity(fixes([0, 1, 2, 3, 4, 5]), min_span_s=6)
    assert too_short["reason"] == "insufficient_time_span"


def test_single_position_outlier_is_rejected_before_refit():
    samples = fixes(np.arange(9, dtype=float))
    t, p, cov = samples[4]
    samples[4] = (t, p + np.array([18.0, -12.0, 8.0]), cov)
    result = fit_position_velocity(samples)
    assert result["reason"] == "ok"
    assert result["rejected_stamps"] == [4.0]
    np.testing.assert_allclose(result["velocity"], [1.2, -0.4, 0.1], atol=1e-10)


def test_fitted_gnss_velocity_uses_existing_eskf_velocity_update():
    fit = fit_position_velocity(fixes(np.arange(9, dtype=float)))
    filt = ESKF([0, 0, 0], [0, 0, 0], [1, 0, 0, 0], [0, 0, -9.80665], "body")
    result = filt.update_velocity(fit["velocity"], fit["covariance"], None)
    assert result.accepted
    assert np.linalg.norm(filt.v - fit["velocity"]) < np.linalg.norm(fit["velocity"])


def test_body_frame_truth_velocity_rotates_into_world_for_velocity_scoring():
    from scipy.spatial.transform import Rotation
    from frame_conversions import body_velocity_to_world

    q = Rotation.from_euler("z", 90, degrees=True).as_quat()
    np.testing.assert_allclose(body_velocity_to_world([5.0, 0.0, 0.0], q), [0.0, 5.0, 0.0], atol=1e-12)

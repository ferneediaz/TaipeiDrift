"""Heading sensors and simulated dead reckoning, checked with numbers that can be followed by hand."""
from datetime import datetime, timedelta, timezone

import numpy as np
import pytest

from src.sensors.dead_reckoning import simulated_steps
from src.sensors.heading import compass_heading, sun_elevation, sun_heading, sun_heading_error_sd

TAIWAN = timezone(timedelta(hours=8))


@pytest.mark.parametrize(
    "lat, lon, when, expected",
    [
        (25.03, 121.56, datetime(2026, 10, 3, 12, 0, tzinfo=TAIWAN), 60.72),  # Taipei, the hackathon
        (25.03, 121.56, datetime(2026, 6, 21, 12, 0, tzinfo=TAIWAN), 88.11),  # Taipei, midsummer: nearly overhead
        (32.30, 119.90, datetime(2018, 10, 23, 8, 32, tzinfo=TAIWAN), 26.59),  # UAV-VisLoc flight 03
    ],
)
def test_sun_elevation_matches_pvlib(lat, lon, when, expected):
    # expected values: pvlib.solarposition.get_solarposition, geometric elevation
    assert sun_elevation(lat, lon, when) == pytest.approx(expected, abs=0.05)


def test_sun_heading_error_examples_from_the_docstring():
    np.testing.assert_allclose(sun_heading_error_sd([30.0, 60.0, 80.0]), [0.59, 1.74, 5.7], atol=0.01)


def test_compass_has_one_offset_for_the_whole_flight():
    rng = np.random.default_rng(0)
    true = np.linspace(0.0, 350.0, 5000)
    error = compass_heading(true, rng, offset_sd_deg=4.0, noise_sd_deg=1.0) - true
    assert error.std() == pytest.approx(1.0, rel=0.05)  # around the offset, only the noise varies


def test_sun_heading_spread_follows_the_elevation():
    rng = np.random.default_rng(0)
    true = np.zeros(20000)
    low = sun_heading(true, np.full(20000, 30.0), rng) - true
    high = sun_heading(true, np.full(20000, 80.0), rng) - true
    assert low.std() == pytest.approx(0.59, rel=0.05)
    assert high.std() == pytest.approx(5.7, rel=0.05)


def test_simulated_step_example_from_the_docstring():
    rng = np.random.default_rng(0)
    path = np.array([[0.0, 0.0], [100.0, 0.0]])
    steps = simulated_steps(path, np.array([0.0, 3.0]), rng, scale_offset_sd=0.0, scale_walk_per_sqrt_km=0.0, noise_m_per_sqrt_100m=0.0)
    np.testing.assert_allclose(steps, [[0.0, 0.0], [99.86, 5.23]], atol=0.01)


def test_simulated_steps_drift_without_a_heading_error():
    rng = np.random.default_rng(1)
    east = np.arange(0.0, 10_000.0, 95.0)
    path = np.column_stack([np.zeros_like(east), east])
    steps = simulated_steps(path, np.zeros(len(path)), rng)
    end_error = np.linalg.norm(steps.sum(axis=0) - (path[-1] - path[0]))
    assert 10.0 < end_error < 1000.0  # it drifts, by less than 10 percent over 10 km

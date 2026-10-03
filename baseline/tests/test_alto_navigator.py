"""Regression on the real ALTO validation section. Skipped when the data is not on this machine.

The numbers are those of ``experiments/h_alto_end_to_end.py`` and of ``docs/findings.md``,
section 3.4: median, worst and end error in metres after GNSS is lost at 300 m.
"""
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from src.data.alto import AltoConfig, load_alto_flight
from src.estimation.camera_navigator import NavigatorConfig, calibrate, navigate
from src.estimation.image_motion import image_shift, shifts_for_flight
from src.evaluation.navigation_metrics import summarize_navigation

REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_ROOT = REPO_ROOT / "data" / "raw" / "alto"

pytestmark = pytest.mark.skipif(not (DATA_ROOT / "Val.zip").is_file(), reason="ALTO Val.zip is not downloaded")

EXPECTED = [
    # settings, (median, worst, end), fixes used, fixes rejected
    (dict(fix_every_m=None), (472.4, 657.0, 608.2), 0, 0),
    (dict(fix_every_m=100.0), (25.5, 50.3, 26.3), 39, 0),
    (dict(fix_every_m=300.0), (30.6, 83.1, 15.7), 13, 0),
    (dict(fix_every_m=400.0), (285.1, 897.9, 897.9), 7, 0),  # the cliff: wrong fixes are used
    (dict(fix_every_m=1000.0, min_score=0.33), (472.4, 657.0, 608.2), 0, 3),  # the check refuses them
    (dict(fix_every_m=1000.0, min_score=0.33, search="sized"), (56.3, 278.1, 10.0), 4, 0),  # the larger search recovers
    (dict(fix_every_m=2000.0, min_score=0.33, search="sized"), (115.5, 577.6, 196.7), 1, 0),
]


@pytest.fixture(scope="module")
def flight():
    return load_alto_flight(AltoConfig(data_root=str(DATA_ROOT)))


@pytest.fixture(scope="module")
def shifts(flight):
    return shifts_for_flight(flight, REPO_ROOT / "data" / "processed" / f"{flight.name}_flow.npy")


@pytest.fixture(scope="module")
def calibration(flight, shifts):
    return calibrate(flight, shifts, NavigatorConfig())


def test_section_is_the_one_measured(flight):
    assert len(flight) == 1684
    assert len(flight.reference) == 459
    assert flight.travelled[-1] == pytest.approx(4592.0, abs=1.0)


def test_image_shift_matches_the_stored_values(flight, shifts):
    for k in (1, 400, 1200):
        np.testing.assert_allclose(image_shift(flight.frame(k - 1), flight.frame(k)), shifts[k], atol=1e-6)


def test_calibration(calibration):
    assert calibration.jam_index == 118
    assert calibration.zoom == pytest.approx(0.85)
    assert calibration.angle == pytest.approx(10.0)
    np.testing.assert_allclose(calibration.fix_offset, [-6.6, 3.6], atol=0.05)  # north, east


@pytest.mark.parametrize("settings, expected, used, rejected", EXPECTED)
def test_errors_match_the_experiment(flight, shifts, calibration, settings, expected, used, rejected):
    result = navigate(flight, shifts, calibration, replace(NavigatorConfig(), **settings))
    summary = summarize_navigation(result, flight)
    assert (summary.median, summary.worst, summary.end) == pytest.approx(expected, abs=0.06)
    assert (summary.fixes_used, summary.fixes_rejected) == (used, rejected)

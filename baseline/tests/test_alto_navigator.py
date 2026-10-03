"""Regression on the real ALTO validation section. Skipped when the data is not on this machine.

The numbers are those of ``experiments/h_alto_end_to_end.py`` and of ``docs/findings.md``,
section 3.4: median, worst and end error in metres after GNSS is lost at 300 m.
"""
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from src.data.alto import AltoConfig, load_alto_flight
from src.data.camera_flight import prepare
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


# The search that knows nothing about the true path: one map from all five reference folders,
# searched in a circle around the estimate (findings, section 3.8).
AREA = NavigatorConfig(search="area")
EXPECTED_AREA = [
    (dict(fix_every_m=100.0), (25.22, 49.87, 27.68), 39, 0),
    (dict(fix_every_m=300.0, min_score=0.33), (31.14, 72.87, 37.76), 12, 1),
    (dict(fix_every_m=400.0), (35.96, 279.88, 279.88), 9, 0),  # no check: one wrong fix is used
    (dict(fix_every_m=1000.0, min_score=0.33), (56.12, 278.07, 10.32), 4, 0),
]


@pytest.fixture(scope="module")
def area_flight():
    return load_alto_flight(AltoConfig(data_root=str(DATA_ROOT), ground_map=True, map_cache_dir=str(REPO_ROOT / "data" / "processed")))


@pytest.fixture(scope="module")
def area_calibration(area_flight, shifts):
    return calibrate(area_flight, shifts, AREA)


def test_map_shows_each_reference_image_at_its_coordinates(area_flight):
    ground = area_flight.ground_map
    for i in (0, 200, 458):
        reference = prepare(area_flight.reference.load(i))
        x, y = np.round(ground.to_pixel(area_flight.reference.position[i])).astype(int)
        crop = prepare(ground.image[y - 250 : y + 250, x - 250 : x + 250])
        assert np.corrcoef(reference.ravel(), crop.ravel())[0, 1] > 0.9


def test_area_calibration(area_calibration):
    assert area_calibration.jam_index == 118
    assert area_calibration.zoom == pytest.approx(0.85)
    assert area_calibration.angle == pytest.approx(10.0)
    np.testing.assert_allclose(area_calibration.fix_offset, [-6.36, 3.32], atol=0.05)


@pytest.mark.parametrize("settings, expected, used, rejected", EXPECTED_AREA)
def test_area_search_errors(area_flight, shifts, area_calibration, settings, expected, used, rejected):
    summary = summarize_navigation(navigate(area_flight, shifts, area_calibration, replace(AREA, **settings)), area_flight)
    assert (summary.median, summary.worst, summary.end) == pytest.approx(expected, abs=0.06)
    assert (summary.fixes_used, summary.fixes_rejected) == (used, rejected)

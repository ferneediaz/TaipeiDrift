"""The camera navigator end to end on the generated flight."""
from dataclasses import replace

import numpy as np
import pytest

from src.data.camera_flight import CameraFlight, ReferenceMap
from src.data.synthetic_camera import make_synthetic_camera_flight
from src.estimation.camera_navigator import NavigatorConfig, calibrate, jam_index, navigate, run_camera_navigator
from src.estimation.image_motion import shifts_for_flight
from src.estimation.navigator_core import DISAGREES_WITH_ESTIMATE, FRAMES_DISAGREE, LOW_SCORE, OFF_MAP, OK
from src.evaluation.navigation_metrics import error_at_distances, fix_errors, navigation_errors, summarize_navigation

CFG = NavigatorConfig(jam_after_m=300.0, fix_every_m=100.0)


@pytest.fixture(scope="module")
def flight():
    return make_synthetic_camera_flight()


@pytest.fixture(scope="module")
def shifts(flight):
    return shifts_for_flight(flight)


@pytest.fixture(scope="module")
def calibration(flight, shifts):
    return calibrate(flight, shifts, CFG)


def test_calibration_learns_zoom_rotation_and_scale(flight, calibration):
    assert calibration.jam_index == jam_index(flight, 300.0) == 100
    assert calibration.zoom == pytest.approx(flight.metadata["zoom"], abs=0.01)
    assert calibration.angle == pytest.approx(flight.metadata["rotation_deg"], abs=0.01)
    assert np.linalg.norm(calibration.fix_offset) < 2.0
    # one pixel of image shift is about 0.6 m * 0.85 = 0.51 m of ground
    metres_per_pixel = np.sqrt(abs(np.linalg.det(calibration.motion_matrix)))
    assert metres_per_pixel == pytest.approx(0.51, rel=0.1)


def test_camera_alone_drifts_and_fixes_hold_it(flight, shifts, calibration):
    alone = summarize_navigation(navigate(flight, shifts, calibration, replace(CFG, fix_every_m=None)), flight)
    fixed = summarize_navigation(navigate(flight, shifts, calibration, CFG), flight)
    assert alone.fixes_used == alone.fixes_rejected == 0
    assert 5.0 < alone.end < 0.10 * alone.distance_m  # it drifts, by less than 10 percent of the distance
    assert fixed.fixes_used >= 5 and fixed.fixes_rejected == 0
    assert fixed.end < 0.5 * alone.end
    assert fixed.worst < 12.0
    assert fixed.used_but_wrong == 0


def test_result_has_one_row_per_frame_after_the_jam(flight, shifts, calibration):
    result = navigate(flight, shifts, calibration, CFG)
    rows = len(flight) - calibration.jam_index
    assert result.start_index == calibration.jam_index
    assert result.position.shape == (rows, 2)
    assert result.sigma.shape == (rows,)
    assert len(result.status) == rows
    np.testing.assert_array_equal(result.position[0], flight.position_gt[calibration.jam_index])
    errors = navigation_errors(result, flight)
    assert errors.distance_since_jam[0] == 0.0 and errors.error[0] == 0.0
    at = error_at_distances(errors, [100.0, 10_000.0])
    assert at[100.0] is not None and at[10_000.0] is None
    assert len(fix_errors(result, flight)) == len(result.fixes)


def test_truth_after_the_jam_is_never_read(flight, shifts, calibration):
    blind = np.array(flight.position_gt)
    blind[calibration.jam_index + 1 :] = np.nan
    hidden = CameraFlight(flight.name, flight.timestamp, blind, flight.load_frame, flight.reference, flight.metadata)
    a = run_camera_navigator(flight, CFG, shifts=shifts)
    b = run_camera_navigator(hidden, CFG, shifts=shifts)
    np.testing.assert_array_equal(a.position, b.position)
    assert [f.used for f in a.fixes] == [f.used for f in b.fixes]


def test_uncertainty_grows_without_fixes_and_drops_at_a_fix(flight, shifts, calibration):
    alone = navigate(flight, shifts, calibration, replace(CFG, fix_every_m=None))
    assert np.all(np.diff(alone.sigma) >= 0)
    assert alone.sigma[0] == pytest.approx(CFG.start_sigma_m)
    assert alone.sigma[-1] == pytest.approx(np.hypot(3.0, 0.10 * np.linalg.norm(np.diff(alone.position, axis=0), axis=1).sum()), rel=1e-6)

    fixed = navigate(flight, shifts, calibration, CFG)
    first = fixed.fixes[0]
    row = first.frame - fixed.start_index
    assert first.used and first.reason == OK
    assert fixed.sigma[row] < fixed.sigma[row - 1]
    assert fixed.sigma.max() < alone.sigma.max()
    assert set(fixed.status) <= {"TRACKING", "DEGRADED", "LOST"}


def test_a_low_score_is_rejected_and_changes_nothing(flight, shifts, calibration):
    alone = navigate(flight, shifts, calibration, replace(CFG, fix_every_m=None))
    strict = navigate(flight, shifts, calibration, replace(CFG, min_score=2.0))  # no match can score 2
    assert len(strict.fixes) >= 5
    assert all(not f.used and f.reason == LOW_SCORE for f in strict.fixes)
    np.testing.assert_array_equal(strict.position, alone.position)
    np.testing.assert_array_equal(strict.sigma, alone.sigma)


def _misled(flight, offset_north_m):
    """The same flight with a map whose coordinates are wrong: confident matches at the wrong place."""
    reference = flight.reference
    wrong_map = ReferenceMap(reference.position + np.array([offset_north_m, 0.0]), reference.metres_per_pixel, reference.load)
    return CameraFlight(flight.name, flight.timestamp, flight.position_gt, flight.load_frame, wrong_map, flight.metadata)


def test_a_fix_far_from_the_estimate_is_rejected(flight, shifts, calibration):
    misled = _misled(flight, 500.0)
    alone = navigate(flight, shifts, calibration, replace(CFG, fix_every_m=None))
    result = navigate(misled, shifts, calibration, replace(CFG, search="sized"))
    assert len(result.fixes) >= 5
    assert all(not f.used and f.reason == DISAGREES_WITH_ESTIMATE for f in result.fixes)
    assert sum(f.score > 0.9 for f in result.fixes) >= 5  # the matches themselves look fine
    np.testing.assert_array_equal(result.position, alone.position)
    summary = summarize_navigation(result, misled)
    assert summary.fixes_rejected == len(result.fixes) and summary.rejected_but_right == 0


def test_known_limit_the_distance_check_widens_with_the_uncertainty(flight, shifts, calibration):
    """A wrong place 200 m away is rejected at first, and believed once 3 sigma has grown past 200 m.

    This documents a limit of the check: the distance test alone cannot catch a confident match at a
    wrong place that lies inside the stated uncertainty. It found 600 m without a fix on this flight.
    """
    result = navigate(_misled(flight, 200.0), shifts, calibration, replace(CFG, search="sized"))
    reasons = [f.reason for f in result.fixes]
    assert reasons[0] == DISAGREES_WITH_ESTIMATE
    assert OK in reasons
    first_believed = result.fixes[reasons.index(OK)]
    assert first_believed.allowed > 180.0 and first_believed.score > 0.9


def test_agreement_of_three_frames(flight, shifts, calibration):
    """With the option on, each fix is matched in three frames; on clean ground they agree."""
    result = navigate(flight, shifts, calibration, replace(CFG, agreement_frames=3))
    assert len(result.fixes) >= 5
    assert all(f.used and f.frames_agreeing >= 2 for f in result.fixes)
    assert summarize_navigation(result, flight).worst < 12.0
    # a consistently wrong map fools the frames alike, so agreement alone does not catch it;
    # every fix is still refused, mostly by the distance check
    misled = navigate(_misled(flight, 500.0), shifts, calibration, replace(CFG, search="sized", agreement_frames=3))
    assert misled.fixes and not any(f.used for f in misled.fixes)
    assert all(f.reason in (DISAGREES_WITH_ESTIMATE, FRAMES_DISAGREE) for f in misled.fixes)
    assert sum(f.reason == DISAGREES_WITH_ESTIMATE and f.frames_agreeing >= 2 for f in misled.fixes) >= 4


def test_sized_search_looks_at_more_images_when_less_certain(flight, shifts, calibration):
    near = navigate(flight, shifts, calibration, replace(CFG, search="sized", fix_every_m=100.0))
    far = navigate(flight, shifts, calibration, replace(CFG, search="sized", fix_every_m=500.0))
    assert far.fixes and near.fixes
    assert far.fixes[0].candidates > near.fixes[0].candidates
    assert far.fixes[0].allowed > near.fixes[0].allowed


def test_bad_settings_are_refused(flight, shifts, calibration):
    with pytest.raises(ValueError):
        navigate(flight, shifts, calibration, replace(CFG, search="everywhere"))
    with pytest.raises(ValueError):
        calibrate(flight, shifts, replace(CFG, jam_after_m=1e6))
    without_map = replace(flight, ground_map=None)
    with pytest.raises(ValueError):
        navigate(without_map, shifts, calibration, replace(CFG, search="area"))


AREA = replace(CFG, search="area", min_score=0.33)


@pytest.fixture(scope="module")
def area_calibration(flight, shifts):
    return calibrate(flight, shifts, AREA)


def test_area_search_calibrates_and_holds_the_position(flight, shifts, area_calibration):
    assert area_calibration.zoom == pytest.approx(flight.metadata["zoom"], abs=0.01)
    assert area_calibration.angle == pytest.approx(flight.metadata["rotation_deg"], abs=0.01)
    result = navigate(flight, shifts, area_calibration, AREA)
    summary = summarize_navigation(result, flight)
    assert summary.fixes_used >= 5 and summary.fixes_rejected == 0 and summary.used_but_wrong == 0
    assert summary.worst < 12.0
    assert all(f.search_radius_m >= AREA.min_search_radius_m for f in result.fixes)


def test_area_search_reads_no_truth_after_the_jam(flight, shifts):
    blind = np.array(flight.position_gt)
    blind[jam_index(flight, AREA.jam_after_m) + 1 :] = np.nan
    a = run_camera_navigator(flight, AREA, shifts=shifts)
    b = run_camera_navigator(replace(flight, position_gt=blind), AREA, shifts=shifts)
    np.testing.assert_array_equal(a.position, b.position)


def test_area_search_refuses_a_map_with_wrong_coordinates(flight, shifts, area_calibration):
    ground = flight.ground_map
    wrong = replace(flight, ground_map=replace(ground, origin=ground.origin + np.array([500.0, 0.0]), _prepared=None))
    result = navigate(wrong, shifts, area_calibration, AREA)
    assert result.fixes and not any(f.used for f in result.fixes)
    assert {f.reason for f in result.fixes} <= {DISAGREES_WITH_ESTIMATE, LOW_SCORE, OFF_MAP}


def test_area_search_off_the_map_is_reported(flight, shifts, area_calibration):
    ground = flight.ground_map
    empty = replace(flight, ground_map=replace(ground, covered=np.zeros(ground.shape, bool), _prepared=None, _covered_float=None))
    result = navigate(empty, shifts, area_calibration, AREA)
    assert result.fixes and all(f.reason == OFF_MAP and not f.used for f in result.fixes)
    alone = navigate(flight, shifts, area_calibration, replace(AREA, fix_every_m=None))
    np.testing.assert_array_equal(result.position, alone.position)

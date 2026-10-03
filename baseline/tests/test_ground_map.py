"""The map of the whole area: coordinates, building it from pieces, and searching it."""
import numpy as np
import pytest

from src.data.camera_flight import prepare
from src.data.ground_map import GroundMap, mosaic
from src.data.synthetic_camera import make_synthetic_camera_flight
from src.estimation.map_matching import make_template, search_area


@pytest.fixture(scope="module")
def flight():
    return make_synthetic_camera_flight(length_m=450.0)


def test_coordinates_example_from_the_docstring():
    ground = GroundMap(np.zeros((100, 100), np.uint8), 0.5, np.array([100.0, 0.0]))
    np.testing.assert_allclose(ground.to_position(20, 40), [80.0, 10.0])
    np.testing.assert_allclose(ground.to_pixel([80.0, 10.0]), [20.0, 40.0])


def test_mosaic_puts_each_piece_at_its_coordinates():
    a = np.full((4, 4), 100, np.uint8)
    b = np.full((4, 4), 200, np.uint8)
    # b lies 2 m (2 pixels) east of a, so their pieces overlap by two columns
    ground = mosaic(np.array([[0.0, 0.0], [0.0, 2.0]]), lambda i: (a, b)[i], metres_per_pixel=1.0, zoom_unit_px=4)
    assert ground.covered[:4, :6].all() and not ground.covered[4:].any() and not ground.covered[:, 6:].any()
    np.testing.assert_array_equal(ground.image[0, :6], [100, 100, 150, 150, 200, 200])  # the overlap is averaged
    np.testing.assert_allclose(ground.to_pixel([0.0, 0.0]), [2.0, 2.0])  # centre of a: 2 pixels from the corner


def test_prepared_map_fills_empty_parts_first():
    image = np.zeros((64, 64), np.uint8)
    image[:, :32] = np.arange(32, dtype=np.uint8)[None, :] * 8
    covered = np.zeros((64, 64), bool)
    covered[:, :32] = True
    prepared = GroundMap(image, 1.0, np.zeros(2), covered, zoom_unit_px=64).prepared()
    assert prepared.dtype == np.float32 and prepared.shape == (64, 64)
    assert prepared[:, 40:].std() == pytest.approx(0.0)  # the empty half is flat


def test_search_area_finds_the_true_position(flight):
    k = 60
    estimate = flight.position_gt[k] + np.array([30.0, -20.0])  # the estimate is 36 m off
    found = search_area(prepare(flight.frame(k)), flight.ground_map, estimate, 60.0, [0.85], [15.0])
    assert np.linalg.norm(found.position - flight.position_gt[k]) < 1.5
    assert found.score > 0.9


def test_search_area_stays_inside_its_circle(flight):
    k = 60
    estimate = flight.position_gt[k] + np.array([0.0, 100.0])  # the true place is 100 m away
    found = search_area(prepare(flight.frame(k)), flight.ground_map, estimate, 30.0, [0.85], [15.0])
    assert np.linalg.norm(found.position - estimate) <= 30.0 + 0.6
    assert found.score < 0.5


def test_search_area_off_the_map_finds_nothing(flight):
    blank = GroundMap(
        flight.ground_map.image, flight.ground_map.metres_per_pixel, flight.ground_map.origin,
        np.zeros(flight.ground_map.shape, bool), flight.ground_map.zoom_unit_px,
    )
    assert search_area(prepare(flight.frame(10)), blank, flight.position_gt[10], 60.0, [0.85], [15.0]) is None


def test_inscribed_template_has_no_black_corners():
    bright = np.full((160, 160), 200.0, np.float32)
    turned = make_template(bright, zoom=1.0, angle=45.0, reference_side=160, keep=0.8, inscribed=True)
    assert turned.min() > 150.0  # no corner of the turned frame is cut in
    assert make_template(bright, 1.0, 45.0, 160, keep=0.8).min() < 50.0  # without it, the corners are black
    np.testing.assert_array_equal(  # at small angles nothing changes
        make_template(bright, 0.85, 10.0, 160, keep=0.8, inscribed=True), make_template(bright, 0.85, 10.0, 160, keep=0.8)
    )


def test_quarters_agree_at_the_true_place_and_not_at_a_look_alike(flight):
    from src.estimation.map_matching import quad_agreement

    k = 60
    frame = prepare(flight.frame(k))
    right = search_area(frame, flight.ground_map, flight.position_gt[k], 60.0, [0.85], [15.0])
    assert quad_agreement(frame, flight.ground_map, right, flight.position_gt[k], 60.0) == 4
    elsewhere = prepare(flight.frame(k + 80))  # a frame 240 m further on, searched here
    wrong = search_area(elsewhere, flight.ground_map, flight.position_gt[k], 60.0, [0.85], [15.0])
    assert quad_agreement(elsewhere, flight.ground_map, wrong, flight.position_gt[k], 60.0) <= 2

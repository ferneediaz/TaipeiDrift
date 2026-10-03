"""Image shift and position fixes on the generated flight, where the truth is known exactly."""
import cv2
import numpy as np
import pytest

from src.data.camera_flight import prepare
from src.data.synthetic_camera import IMAGE_PX, make_synthetic_camera_flight
from src.estimation.image_motion import image_shift, shifts_for_flight
from src.estimation.map_matching import make_template, match


@pytest.fixture(scope="module")
def flight():
    return make_synthetic_camera_flight(length_m=450.0)


def _textured(side: int, seed: int = 1) -> np.ndarray:
    rng = np.random.default_rng(seed)
    coarse = rng.random((side // 8, side // 8), dtype=np.float32)
    return (cv2.resize(coarse, (side, side), interpolation=cv2.INTER_CUBIC) * 255).clip(0, 255).astype(np.uint8)


@pytest.mark.parametrize("side", [160, 500])
def test_image_shift_measures_a_known_shift(side):
    image = _textured(side + 40)
    previous = image[20 : 20 + side, 20 : 20 + side]
    current = image[14 : 14 + side, 30 : 30 + side]  # the content moved 10 px left and 6 px down
    np.testing.assert_allclose(image_shift(previous, current), [-10.0, 6.0], atol=0.5)


def test_shifts_for_flight_are_cached(flight, tmp_path):
    cache = tmp_path / "shifts.npy"
    first = shifts_for_flight(flight, cache)
    assert first.shape == (len(flight), 2)
    np.testing.assert_array_equal(first[0], [0.0, 0.0])
    assert cache.is_file()
    np.save(cache, first + 1.0)  # a second call must read the file and not recompute
    np.testing.assert_array_equal(shifts_for_flight(flight, cache), first + 1.0)


def test_shifts_point_one_way_on_a_straight_flight(flight):
    shifts = shifts_for_flight(flight)[1:]
    assert np.linalg.norm(np.median(shifts, axis=0)) > 2.0  # 3 m per frame at about 0.5 m per pixel
    assert np.all(np.std(shifts, axis=0) < 1.5)


def test_template_has_the_expected_size():
    frame = prepare(_textured(IMAGE_PX))
    template = make_template(frame, zoom=0.85, angle=15.0, reference_side=IMAGE_PX, keep=0.8)
    assert template.shape == (108, 108)  # round(160 * 0.85) = 136, of which 80 percent
    assert template.dtype == np.float32


def test_match_finds_the_true_position(flight):
    k = 60
    candidates = flight.reference.nearest(flight.position_gt[k], 7)
    found = match(prepare(flight.frame(k)), flight.reference, candidates, zooms=[0.85], angles=[15.0])
    assert np.linalg.norm(found.position - flight.position_gt[k]) < 2.0
    assert found.score > 0.6
    assert found.reference_index in candidates


def test_match_recovers_zoom_and_angle(flight):
    k = 90
    candidates = flight.reference.nearest(flight.position_gt[k], 5)
    found = match(prepare(flight.frame(k)), flight.reference, candidates, zooms=np.arange(0.7, 1.01, 0.05), angles=np.arange(0.0, 31.0, 5.0))
    assert found.zoom == pytest.approx(flight.metadata["zoom"], abs=0.01)
    assert found.angle == pytest.approx(flight.metadata["rotation_deg"], abs=0.01)
    assert np.linalg.norm(found.position - flight.position_gt[k]) < 2.0


def test_match_at_a_wrong_place_scores_low(flight):
    k = 20
    right = match(prepare(flight.frame(k)), flight.reference, flight.reference.nearest(flight.position_gt[k], 3), [0.85], [15.0])
    far = flight.position_gt[k] + np.array([0.0, 300.0])
    wrong = match(prepare(flight.frame(k)), flight.reference, flight.reference.nearest(far, 3), [0.85], [15.0])
    assert wrong.score < 0.5 * right.score


def test_match_needs_candidates(flight):
    with pytest.raises(ValueError):
        match(prepare(flight.frame(0)), flight.reference, [], [0.85], [15.0])

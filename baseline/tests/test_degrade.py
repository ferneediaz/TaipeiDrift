"""Degraded camera frames: the noise of less light, blur and haze behave as described."""
import numpy as np

from src.data.degrade import degrade, haze, less_light


def test_less_light_keeps_the_brightness_and_adds_the_expected_noise():
    rng = np.random.default_rng(0)
    grey = np.full((200, 200), 128, np.uint8)  # about 1,000 particles at the recorded light
    for light, relative in ((1.0, 0.032), (1 / 64, 0.25)):
        out = less_light(grey, light, rng).astype(float)
        assert abs(out.mean() - 128) < 3
        # the docstring's numbers: 1,000 +- 32 (3 %), and 16 +- 4 (25 %) with 64 times less light
        assert abs(out.std() / 128 - relative) < 0.2 * relative + 0.01


def test_haze_leaves_the_stated_share_of_the_contrast():
    rng = np.random.default_rng(0)
    stripes = np.tile(np.array([50, 200], np.uint8), (100, 50))
    out = haze(stripes, 0.1, rng).astype(float)
    assert abs((out[:, 1::2].mean() - out[:, ::2].mean()) - 0.1 * 150) < 3


def test_the_same_frame_degrades_the_same_way_twice():
    frame = np.random.default_rng(1).integers(0, 255, (64, 64)).astype(np.uint8)
    a = degrade(frame, "light", 1 / 16, index=7, metres_per_pixel=0.5)
    b = degrade(frame, "light", 1 / 16, index=7, metres_per_pixel=0.5)
    c = degrade(frame, "light", 1 / 16, index=8, metres_per_pixel=0.5)
    assert np.array_equal(a, b) and not np.array_equal(a, c)

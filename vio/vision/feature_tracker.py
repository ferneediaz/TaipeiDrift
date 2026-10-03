"""Keyframe feature tracking: Shi-Tomasi corners followed by pyramidal Lucas-Kanade.

Corners are detected on a keyframe and tracked frame to frame. Each call
returns the surviving tracks as matched pixel pairs (keyframe, current frame),
so relative motion is always measured against the keyframe rather than
chained over consecutive frames. A new keyframe is taken when too few tracks
survive or the keyframe gets too old. ``max_keyframe_age = 1`` gives plain
consecutive-frame tracking.

Outliers are removed with a forward-backward check: a point tracked forward
and then back must land within ``fb_max_error`` pixels of where it started.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class TrackerConfig:
    """Parameters of the tracker. Pixel values refer to the processed (possibly downscaled) image."""

    max_corners: int = 400
    quality_level: float = 0.01
    min_distance: int = 10  # px between detected corners
    block_size: int = 7
    lk_window: int = 21  # px, Lucas-Kanade search window
    lk_levels: int = 3  # pyramid levels
    fb_max_error: float = 1.0  # px, forward-backward consistency threshold
    min_tracks: int = 80  # take a new keyframe below this many tracks
    max_keyframe_age: int = 10  # frames


@dataclass
class TrackResult:
    """Correspondences between the current keyframe and the current frame."""

    frame_index: int
    keyframe_index: int  # frame index of the keyframe the points refer to
    keyframe_points: np.ndarray  # (M, 2) px in the keyframe
    current_points: np.ndarray  # (M, 2) px in the current frame
    n_detected: int  # corners detected on the keyframe
    new_keyframe: bool  # this frame became the keyframe (no motion measured)
    reanchor: str = ""  # after this measurement the tracker took a new keyframe: "age", "few tracks" or ""


class KeyframeTracker:
    """Track corners from a keyframe through the following frames."""

    def __init__(self, cfg: TrackerConfig | None = None) -> None:
        self.cfg = cfg or TrackerConfig()
        self._lk = dict(
            winSize=(self.cfg.lk_window, self.cfg.lk_window),
            maxLevel=self.cfg.lk_levels,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        self._prev_gray: np.ndarray | None = None
        self._kf_index = -1
        self._kf_pts = np.empty((0, 2), np.float32)
        self._cur_pts = np.empty((0, 2), np.float32)
        self._n_detected = 0

    def _detect(self, gray: np.ndarray) -> np.ndarray:
        c = self.cfg
        pts = cv2.goodFeaturesToTrack(gray, c.max_corners, c.quality_level, c.min_distance, blockSize=c.block_size)
        return np.empty((0, 2), np.float32) if pts is None else pts.reshape(-1, 2).astype(np.float32)

    def _start_keyframe(self, gray: np.ndarray, index: int) -> TrackResult:
        pts = self._detect(gray)
        self._kf_index, self._kf_pts, self._cur_pts = index, pts, pts.copy()
        self._n_detected = len(pts)
        self._prev_gray = gray
        return TrackResult(index, index, pts, pts.copy(), len(pts), new_keyframe=True)

    def process(self, gray: np.ndarray, index: int) -> TrackResult:
        """Track into a new grayscale frame with the given frame index."""
        if self._prev_gray is None or len(self._cur_pts) == 0:
            return self._start_keyframe(gray, index)

        p0 = self._cur_pts.reshape(-1, 1, 2)
        p1, st1, _ = cv2.calcOpticalFlowPyrLK(self._prev_gray, gray, p0, None, **self._lk)
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self._prev_gray, p1, None, **self._lk)
        fb = np.linalg.norm(p0.reshape(-1, 2) - p0r.reshape(-1, 2), axis=1)
        h, w = gray.shape
        p1f = p1.reshape(-1, 2)
        inside = (p1f[:, 0] >= 0) & (p1f[:, 0] < w) & (p1f[:, 1] >= 0) & (p1f[:, 1] < h)
        keep = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb < self.cfg.fb_max_error) & inside

        self._kf_pts, self._cur_pts = self._kf_pts[keep], p1f[keep]
        self._prev_gray = gray
        result = TrackResult(index, self._kf_index, self._kf_pts.copy(), self._cur_pts.copy(),
                             self._n_detected, new_keyframe=False)
        # Re-anchor for the next frame once this measurement has been returned.
        if len(self._cur_pts) < self.cfg.min_tracks:
            result.reanchor = "few tracks"
        elif index - self._kf_index >= self.cfg.max_keyframe_age:
            result.reanchor = "age"
        if result.reanchor:
            self._start_keyframe(gray, index)
        return result

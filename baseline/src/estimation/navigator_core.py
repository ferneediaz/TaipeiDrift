"""The small pieces that carry the logic of the camera navigator.

1. Learn from GNSS: turn image shift in pixels into ground steps in metres.
2. Blend: combine a position fix with the current estimate, each weighted by how certain it is.
3. Decide: may this fix be used, and if not, why.

Uncertainty is kept as one number, the variance in square metres. Its square root, sigma, is
the typical size of the position error in metres.
"""
from __future__ import annotations

import numpy as np

# reasons returned by ``fix_decision``
OK = "OK"
LOW_SCORE = "LOW_SCORE"
DISAGREES_WITH_ESTIMATE = "DISAGREES_WITH_ESTIMATE"

# what the navigator reports about itself, see ``status``
TRACKING = "TRACKING"
DEGRADED = "DEGRADED"
LOST = "LOST"


def fit_motion_matrix(shifts: np.ndarray, steps: np.ndarray) -> np.ndarray:
    """Learn how image shift maps to ground motion, from a stretch where GNSS still works.

    Finds the 2 x 2 matrix ``A`` with ``steps ≈ shifts @ A`` by least squares. The matrix holds
    the scale (metres per pixel of image shift) and the direction (how the camera is turned
    against north) at once, so no camera calibration and no height are needed.

    Example: if every frame shifts by (10, 0) pixels while the aircraft moves 2.5 m east, then
    a shift of (20, 0) pixels will be read as 5 m east.

    Args:
        shifts: (K, 2) image shift per frame, pixels.
        steps: (K, 2) true ground step over the same frames, (north, east) in metres.

    Returns:
        (2, 2) matrix; ``shift @ matrix`` is the ground step in metres.
    """
    matrix, *_ = np.linalg.lstsq(np.asarray(shifts, dtype=float), np.asarray(steps, dtype=float), rcond=None)
    return matrix


def predicted_variance(variance: float, distance_since_fix: float, drift_rate: float) -> float:
    """Uncertainty of the estimate after flying on without a fix.

    Dead reckoning by camera loses about ``drift_rate`` metres per metre flown, so the error
    it adds is ``drift_rate * distance``. Independent errors add as variances.

    Example: sigma 3 m at the last fix, 300 m flown since, drift rate 0.10. The added error is
    30 m, and the new sigma is sqrt(3² + 30²) = 30.1 m.
    """
    return variance + (drift_rate * distance_since_fix) ** 2


def blend(estimate: np.ndarray, variance: float, fix: np.ndarray, fix_variance: float) -> tuple[np.ndarray, float, float]:
    """Combine the current estimate with a position fix.

    The estimate moves towards the fix by the share ``gain = variance / (variance + fix_variance)``.
    The less certain the estimate is compared with the fix, the further it moves.

    Example: the estimate has sigma 30 m (variance 900) and the fix has sigma 15 m (variance 225).
    The gain is 900 / 1125 = 0.8, so the estimate moves 80 percent of the way to the fix, and
    its new variance is 0.2 * 900 = 180, a sigma of 13.4 m.

    Returns:
        the new estimate, its variance, and the gain.
    """
    gain = variance / (variance + fix_variance)
    return estimate + gain * (np.asarray(fix) - estimate), (1.0 - gain) * variance, gain


def allowed_distance(variance: float, fix_variance: float, gate_sigmas: float = 3.0) -> float:
    """How far a fix may lie from the estimate and still be believed.

    If estimate and fix are both right, their distance is rarely larger than a few times
    ``sqrt(variance + fix_variance)``. A fix further away than that is more likely a wrong match.

    Example: estimate sigma 30 m, fix sigma 15 m, 3 sigmas allowed: 3 * sqrt(900 + 225) = 100.6 m.
    """
    return gate_sigmas * float(np.sqrt(variance + fix_variance))


def fix_decision(score: float, distance: float, allowed: float, min_score: float) -> tuple[bool, str]:
    """Decide whether a position fix is used.

    Args:
        score: matching score of the fix.
        distance: metres between the fix and the current estimate.
        allowed: largest distance at which the fix is still believed, see ``allowed_distance``.
        min_score: lowest score at which a match counts as recognised ground.

    Returns:
        ``(use, reason)``. A low score is reported first: such a match carries no information,
        wherever it lies. A good score far from the estimate is the case to look at, since either
        the match is at a wrong place that looks alike, or the estimate has drifted further than
        its stated uncertainty.
    """
    if score < min_score:
        return False, LOW_SCORE
    if distance > allowed:
        return False, DISAGREES_WITH_ESTIMATE
    return True, OK


def status(sigma: float, degraded_above: float = 30.0, lost_above: float = 100.0) -> str:
    """What the navigator reports about its own estimate, from its uncertainty in metres."""
    if sigma > lost_above:
        return LOST
    if sigma > degraded_above:
        return DEGRADED
    return TRACKING

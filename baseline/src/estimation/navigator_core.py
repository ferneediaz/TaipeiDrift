"""The small pieces that carry the logic of the camera navigator.

1. Learn from GNSS: turn image shift in pixels into ground steps in metres.
2. Blend: combine a position fix with the current estimate, each weighted by how certain it is.
3. Decide: may this fix be used, and if not, why.

Uncertainty is kept as one number, the variance in square metres. Its square root, sigma, is
the typical size of the position error in metres.
"""
from __future__ import annotations

import numpy as np

# reasons returned by ``fix_decision``, and by the navigator when frames disagree
OK = "OK"
LOW_SCORE = "LOW_SCORE"
DISAGREES_WITH_ESTIMATE = "DISAGREES_WITH_ESTIMATE"
FRAMES_DISAGREE = "FRAMES_DISAGREE"
OFF_MAP = "OFF_MAP"  # the search circle holds no place where the frame lies fully on the map
UNCONFIRMED = "UNCONFIRMED"  # a large jump, held until the next fix agrees with it
QUARTERS_DISAGREE = "QUARTERS_DISAGREE"  # fewer than three quarters of the frame land with the whole

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


def fit_rotation_scale(shifts: np.ndarray, steps: np.ndarray, moving_share: float = 0.5) -> np.ndarray:
    """Learn image shift to ground motion as one turn and one scale, from medians.

    A camera looking straight down sees the ground slide by the drone's own motion, turned by how
    the camera sits against north and scaled by the height: two numbers. Two reasons to prefer this
    to ``fit_motion_matrix`` on a turning drone:

    - Before the jam the drone may fly in one direction only. A general 2 x 2 matrix is then not
      determined across that direction; a turn and a scale are.
    - A quadcopter tilts into every turn and every change of speed, and a camera fixed to it sees
      the ground jump without the drone moving. Least squares lets these jumps shrink the scale;
      the median of the frame-by-frame ratios ignores them.

    Each frame gives a ratio of ground step to image shift, written as complex numbers
    (north + i east over x + i y). Its size is metres per pixel, its angle the turn.

    Example: the image slides 5 px down, (0, 5), while the drone flies 2 m north, (2, 0). The ratio is
    2 / 5i = -0.4i: 0.4 m per pixel, turned by -90 degrees, and the matrix is [[0, -0.4], [0.4, 0]],
    so a shift of (0, 10) px is read as 4 m north.

    Args:
        shifts: (K, 2) image shift per frame, pixels.
        steps: (K, 2) true ground step over the same frames, (north, east) in metres.
        moving_share: only frames whose step is at least this share of the median step are used,
            so that hovering frames do not count.

    Returns:
        (2, 2) matrix; ``shift @ matrix`` is the ground step in metres.
    """
    z = np.asarray(shifts, dtype=float) @ np.array([1.0, 1j])
    w = np.asarray(steps, dtype=float) @ np.array([1.0, 1j])
    moving = (np.abs(w) >= moving_share * np.median(np.abs(w))) & (np.abs(z) > 1e-6)
    if moving.sum() < 3:
        raise ValueError("fewer than three moving frames before the jam to learn the camera's scale from")
    ratio = w[moving] / z[moving]
    unit = ratio / np.abs(ratio)
    c = np.median(np.abs(ratio)) * np.exp(1j * np.arctan2(np.median(unit.imag), np.median(unit.real)))
    return np.array([[c.real, c.imag], [-c.imag, c.real]])


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


def agreeing_fixes(positions: np.ndarray, radius: float, needed: int) -> np.ndarray | None:
    """Find the largest group of fixes that land on the same place.

    Several frames a few metres apart are matched, and each fix is moved to the same moment by
    the dead-reckoned motion in between. A right place shows up in all of them; a wrong place
    that only looks alike rarely repeats. This is the rule the Tomahawk missile used (2 of 3
    frames, Irani and Christ 1994).

    Example: fixes at (0, 0), (3, 4) and (200, 0), radius 10 m, 2 needed. The first two lie 5 m
    apart and agree; the third is far from both. Returns [0, 1].

    Args:
        positions: (K, 2) fixes, already moved to the same moment, m.
        radius: two fixes agree if they are at most this far apart, m.
        needed: smallest group that counts.

    Returns:
        indices of the largest group in which every fix lies within ``radius`` of every other,
        or ``None`` if no group reaches ``needed``.
    """
    positions = np.asarray(positions, dtype=float)
    close = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2) <= radius
    count = len(positions)
    best: list[int] = []
    for mask in range(1, 2**count):  # K is small (3 to 5), so every group can be tried
        group = [i for i in range(count) if mask >> i & 1]
        if len(group) > len(best) and all(close[i, j] for i in group for j in group):
            best = group
    return np.array(best) if len(best) >= needed else None


def fixes_agree(
    earlier: np.ndarray,
    later: np.ndarray,
    moved: np.ndarray,
    distance_between: float,
    fix_variance: float,
    drift_rate: float,
    gate_sigmas: float = 3.0,
) -> bool:
    """Do two fixes agree, once the earlier one is carried forward by the motion in between?

    Each fix has its own error, and the dead-reckoned motion between them drifts. So the gap
    allowed between the later fix and the carried-forward earlier one is

        gate_sigmas * sqrt(2 * fix_variance + (drift_rate * distance_between)^2)

    Example: fixes accurate to 15 m (variance 225), 300 m apart, 10 percent drift: the gap may be
    3 * sqrt(2 * 225 + 30^2) = 3 * 36.7 = 110 m. Two fixes at different ground 300 m apart that
    both point to the same wrong place are rare; one look-alike place is not.
    """
    limit = gate_sigmas * np.sqrt(2.0 * fix_variance + (drift_rate * distance_between) ** 2)
    gap = np.asarray(later, dtype=float) - (np.asarray(earlier, dtype=float) + np.asarray(moved, dtype=float))
    return bool(np.linalg.norm(gap) <= limit)


def status(sigma: float, degraded_above: float = 30.0, lost_above: float = 100.0) -> str:
    """What the navigator reports about its own estimate, from its uncertainty in metres."""
    if sigma > lost_above:
        return LOST
    if sigma > degraded_above:
        return DEGRADED
    return TRACKING

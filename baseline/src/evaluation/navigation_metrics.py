"""How far the camera navigator is from the truth, and how well its check works.

The true position after the jam is read only here.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from src.data.camera_flight import CameraFlight
from src.estimation.camera_navigator import NavigatorResult


@dataclass
class NavigationErrors:
    """Per-frame errors from the jam onwards."""

    distance_since_jam: np.ndarray  # (M,) true distance flown since the jam, m
    error: np.ndarray  # (M,) m between estimate and truth
    sigma: np.ndarray  # (M,) uncertainty the navigator stated, m


@dataclass
class NavigationSummary:
    """Scalar summary of one run. Distances in metres."""

    distance_m: float  # true distance flown after the jam
    median: float
    p90: float  # 90 percent of the frames have an error below this
    worst: float
    end: float
    fixes_used: int
    fixes_rejected: int
    used_but_wrong: int  # fixes that were used although far from the truth
    rejected_but_right: int  # fixes that were rejected although close to the truth
    within_3_sigma: float  # share of frames whose error is at most 3 times the stated uncertainty

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


def navigation_errors(result: NavigatorResult, flight: CameraFlight) -> NavigationErrors:
    """Compare the estimated path with the truth at the same frames."""
    k0 = result.start_index
    truth = flight.position_gt[k0 : k0 + len(result.position)]
    travelled = flight.travelled[k0 : k0 + len(result.position)]
    return NavigationErrors(
        distance_since_jam=travelled - travelled[0],
        error=np.linalg.norm(result.position - truth, axis=1),
        sigma=result.sigma,
    )


def fix_errors(result: NavigatorResult, flight: CameraFlight) -> np.ndarray:
    """Error of every attempted fix against the truth at its frame, in metres."""
    return np.array([np.linalg.norm(f.position - flight.position_gt[f.frame]) for f in result.fixes])


def summarize_navigation(
    result: NavigatorResult,
    flight: CameraFlight,
    wrong_above_m: float = 50.0,
    right_within_m: float = 30.0,
) -> NavigationSummary:
    """Median, 90 percent, worst and end error, and the counts that describe the check.

    Args:
        wrong_above_m: a used fix counts as wrong if it is further than this from the truth.
        right_within_m: a rejected fix counts as wrongly rejected if it is closer than this.
    """
    errors = navigation_errors(result, flight)
    e = errors.error
    fix_error = fix_errors(result, flight)
    used = np.array([f.used for f in result.fixes], dtype=bool)
    return NavigationSummary(
        distance_m=float(errors.distance_since_jam[-1]),
        median=float(np.median(e)),
        p90=float(np.percentile(e, 90)),
        worst=float(e.max()),
        end=float(e[-1]),
        fixes_used=int(used.sum()),
        fixes_rejected=int((~used).sum()),
        used_but_wrong=int((fix_error[used] > wrong_above_m).sum()),
        rejected_but_right=int((fix_error[~used] <= right_within_m).sum()),
        within_3_sigma=float(np.mean(e <= 3.0 * errors.sigma)),
    )


def error_at_distances(errors: NavigationErrors, distances: list[float]) -> dict[float, float | None]:
    """Position error at fixed distances after the jam. Beyond the end of the flight: ``None``."""
    out: dict[float, float | None] = {}
    for d in distances:
        if d > errors.distance_since_jam[-1]:
            out[d] = None
        else:
            out[d] = float(errors.error[np.searchsorted(errors.distance_since_jam, d)])
    return out

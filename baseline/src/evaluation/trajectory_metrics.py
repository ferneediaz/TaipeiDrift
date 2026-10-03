"""Position-error metrics for an estimate against ground truth after GNSS loss."""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from src.data.trajectory import Trajectory
from src.estimation.inertial_dead_reckoning import DeadReckoningResult


@dataclass
class ErrorSeries:
    """Per-sample errors from the GNSS cutoff onwards."""

    time_since_loss: np.ndarray  # (M,) s
    error: np.ndarray  # (M,) m, ||p_est - p_gt||
    horizontal: np.ndarray  # (M,) m, first two world axes
    vertical: np.ndarray  # (M,) m, absolute error on the third world axis


@dataclass
class ErrorSummary:
    """Scalar summary of an error series, all in metres except duration."""

    duration_s: float
    final: float
    rmse: float
    mean: float
    max: float
    final_horizontal: float
    final_vertical: float

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


def position_errors(p_est: np.ndarray, p_gt: np.ndarray) -> np.ndarray:
    """Euclidean position error e(t) = ||p_est(t) - p_gt(t)|| per sample."""
    return np.linalg.norm(np.asarray(p_est) - np.asarray(p_gt), axis=1)


def error_series(result: DeadReckoningResult, traj: Trajectory) -> ErrorSeries:
    """Compare an estimate with the ground truth at the same samples."""
    p_gt = traj.position_gt[result.start_index : result.start_index + len(result.timestamp)]
    diff = result.position - p_gt
    return ErrorSeries(
        time_since_loss=result.timestamp - result.t0,
        error=np.linalg.norm(diff, axis=1),
        horizontal=np.linalg.norm(diff[:, :2], axis=1),
        vertical=np.abs(diff[:, 2]),
    )


def summarize(series: ErrorSeries) -> ErrorSummary:
    """Final, RMSE, mean and maximum position error."""
    e = series.error
    return ErrorSummary(
        duration_s=float(series.time_since_loss[-1]),
        final=float(e[-1]),
        rmse=float(np.sqrt(np.mean(e**2))),
        mean=float(np.mean(e)),
        max=float(np.max(e)),
        final_horizontal=float(series.horizontal[-1]),
        final_vertical=float(series.vertical[-1]),
    )


def error_at_horizons(series: ErrorSeries, horizons: list[float]) -> dict[float, float | None]:
    """Position error at fixed times after GNSS loss (linear interpolation).

    Horizons beyond the end of the flight map to ``None``.
    """
    out: dict[float, float | None] = {}
    for h in horizons:
        if h > series.time_since_loss[-1]:
            out[h] = None
        else:
            out[h] = float(np.interp(h, series.time_since_loss, series.error))
    return out


def time_to_exceed(series: ErrorSeries, thresholds: list[float]) -> dict[float, float | None]:
    """First time after GNSS loss at which the error exceeds each threshold.

    ``None`` means the threshold was never exceeded.
    """
    out: dict[float, float | None] = {}
    for th in thresholds:
        above = np.nonzero(series.error > th)[0]
        out[th] = float(series.time_since_loss[above[0]]) if above.size else None
    return out

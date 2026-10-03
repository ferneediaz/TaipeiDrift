"""Plots of an estimated trajectory against ground truth, and of the error."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from src.data.trajectory import Trajectory  # noqa: E402
from src.estimation.inertial_dead_reckoning import DeadReckoningResult  # noqa: E402
from src.evaluation.trajectory_metrics import ErrorSeries  # noqa: E402

TRUTH = "#2a78d6"
ESTIMATE = "#eb6834"
THIRD = "#1baf7a"
MUTED = "#52514e"


def _style(ax: plt.Axes) -> None:
    ax.grid(True, color="#e4e3df", linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def _east_north_up(p: np.ndarray, frame: str) -> np.ndarray:
    """Reorder world positions to (east, north, up) for plotting."""
    if frame == "NED":
        return np.column_stack([p[:, 1], p[:, 0], -p[:, 2]])
    if frame == "ENU":
        return p
    raise ValueError(f"unsupported world frame {frame!r}")


def plot_trajectory_2d(result: DeadReckoningResult, traj: Trajectory, path: Path) -> None:
    """Top view (east vs north) and altitude vs time, truth against estimate."""
    gt = _east_north_up(traj.position_gt, traj.world_frame)
    est = _east_north_up(result.position, traj.world_frame)
    k0 = result.start_index

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={"width_ratios": [1, 1.3]})
    ax1.plot(gt[:, 0], gt[:, 1], color=TRUTH, lw=2, label="Ground truth")
    ax1.plot(est[:, 0], est[:, 1], color=ESTIMATE, lw=2, label="IMU dead reckoning")
    ax1.plot(*gt[k0, :2], "o", color=MUTED, ms=8, label="GNSS lost")
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.set_title("Top view")
    ax1.set_aspect("equal", adjustable="datalim")
    _style(ax1)

    ax2.plot(traj.timestamp, gt[:, 2], color=TRUTH, lw=2, label="Ground truth")
    ax2.plot(result.timestamp, est[:, 2], color=ESTIMATE, lw=2, label="IMU dead reckoning")
    ax2.axvline(result.t0, color=MUTED, lw=1, ls="--")
    ax2.annotate("GNSS lost", (result.t0, 1.0), xycoords=("data", "axes fraction"),
                 xytext=(4, -12), textcoords="offset points", color=MUTED)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Altitude (m)")
    ax2.set_title("Altitude")
    _style(ax2)

    fig.suptitle(traj.name)
    fig.legend(*ax1.get_legend_handles_labels(), loc="lower center", ncol=3, frameon=False)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_error(series: ErrorSeries, path: Path, title: str = "") -> None:
    """Position error against time since GNSS loss."""
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(series.time_since_loss, series.error, color=TRUTH, lw=2, label="Total")
    ax.plot(series.time_since_loss, series.horizontal, color=ESTIMATE, lw=2, label="Horizontal")
    ax.plot(series.time_since_loss, series.vertical, color=THIRD, lw=2, label="Vertical")
    ax.set_xlabel("Time since GNSS loss (s)")
    ax.set_ylabel("Position error (m)")
    ax.set_title(title or "Position error")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False)
    _style(ax)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_trajectory_3d(result: DeadReckoningResult, traj: Trajectory, path: Path) -> None:
    """3D view of truth and estimate (east, north, up)."""
    gt = _east_north_up(traj.position_gt, traj.world_frame)
    est = _east_north_up(result.position, traj.world_frame)
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(projection="3d")
    ax.plot(*gt.T, color=TRUTH, lw=2, label="Ground truth")
    ax.plot(*est.T, color=ESTIMATE, lw=2, label="IMU dead reckoning")
    ax.scatter(*gt[result.start_index], color=MUTED, s=40, label="GNSS lost")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Up (m)")
    ax.set_title(traj.name)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)

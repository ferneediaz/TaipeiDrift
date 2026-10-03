"""Comparison plots: IMU-only vs visual-assisted vs the ground-truth-attitude oracle."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import cv2  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from src.data.trajectory import Trajectory  # noqa: E402
from src.estimation.inertial_dead_reckoning import DeadReckoningResult  # noqa: E402

TRUTH = "#0b0b0b"
IMU = "#eb6834"
VISUAL = "#2a78d6"
ORACLE = "#1baf7a"
MUTED = "#52514e"
GRID = "#e4e3df"


def _style(ax: plt.Axes) -> None:
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def _en(p: np.ndarray, frame: str) -> np.ndarray:
    """(east, north) for plotting."""
    return np.column_stack([p[:, 1], p[:, 0]]) if frame == "NED" else p[:, :2]


def plot_trajectory(traj: Trajectory, runs: dict[str, DeadReckoningResult], path: Path) -> None:
    """Top view of the truth and every run. Keys: 'imu', 'visual', 'oracle'."""
    styles = {"imu": (IMU, "-", "IMU only"), "visual": (VISUAL, "-", "IMU + visual attitude"),
              "oracle": (ORACLE, "--", "Oracle: true attitude (not an estimator)")}
    fig, ax = plt.subplots(figsize=(7.5, 7))
    gt = _en(traj.position_gt, traj.world_frame)
    ax.plot(gt[:, 0], gt[:, 1], color=TRUTH, lw=2, label="Ground truth")
    for key, res in runs.items():
        c, ls, label = styles[key]
        e = _en(res.position, traj.world_frame)
        ax.plot(e[:, 0], e[:, 1], color=c, lw=2, ls=ls, label=label)
    k0 = next(iter(runs.values())).start_index
    ax.plot(*gt[k0], "o", color=MUTED, ms=8, label="GNSS lost")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"Top view, {traj.name}")
    ax.set_aspect("equal", adjustable="datalim")
    _style(ax)
    fig.legend(*ax.get_legend_handles_labels(), loc="lower center", ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_series(t: dict[str, np.ndarray], y: dict[str, np.ndarray], ylabel: str, title: str, path: Path) -> None:
    """Error against time since GNSS loss for each run present in ``y``."""
    styles = {"imu": (IMU, "-", "IMU only"), "visual": (VISUAL, "-", "IMU + visual attitude"),
              "oracle": (ORACLE, "--", "Oracle: true attitude (not an estimator)")}
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for key, values in y.items():
        c, ls, label = styles[key]
        ax.plot(t[key], values, color=c, lw=2, ls=ls, label=label)
    ax.set_xlabel("Time since GNSS loss (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False)
    _style(ax)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_visual_tracking(time_s: np.ndarray, n_tracked: np.ndarray, n_inliers: np.ndarray, valid: np.ndarray,
                         accepted: np.ndarray, path: Path, title: str) -> None:
    """Tracked features and RANSAC inliers per frame, with valid / accepted measurements marked."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    ax1.plot(time_s, n_tracked, color=MUTED, lw=1.2, label="Tracked features")
    ax1.plot(time_s, n_inliers, color=VISUAL, lw=1.2, label="RANSAC inliers")
    ax1.set_ylabel("Features per frame")
    ax1.set_ylim(bottom=0)
    ax1.set_title(title)
    ax1.legend(frameon=False, loc="upper right")
    _style(ax1)
    ax2.plot(time_s, valid.astype(float) + 0.04, "|", color=VISUAL, ms=6, label="valid")
    ax2.plot(time_s, accepted.astype(float) * 0.5, "|", color=ORACLE, ms=6, label="accepted by fusion")
    ax2.set_yticks([0.04, 0.5, 1.04], ["invalid / rejected", "accepted", "valid"])
    ax2.set_ylim(-0.2, 1.3)
    ax2.set_xlabel("Time since GNSS loss (s)")
    _style(ax2)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def draw_tracks(gray: np.ndarray, pts_kf: np.ndarray, pts_cur: np.ndarray, inlier: np.ndarray, path: Path) -> None:
    """Debug image: flow vectors from keyframe to current frame, inliers blue, outliers orange."""
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for (a, b), ok in zip(zip(pts_kf, pts_cur), inlier):
        color = (214, 120, 42) if ok else (52, 104, 235)  # BGR
        cv2.line(img, tuple(np.int32(a)), tuple(np.int32(b)), color, 1, cv2.LINE_AA)
        cv2.circle(img, tuple(np.int32(b)), 2, color, -1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)

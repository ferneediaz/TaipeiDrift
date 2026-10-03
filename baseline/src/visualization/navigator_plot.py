"""Plots of the camera navigator: the path from above, and the error over the distance flown."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from src.data.camera_flight import CameraFlight  # noqa: E402
from src.estimation.camera_navigator import NavigatorResult  # noqa: E402
from src.evaluation.navigation_metrics import navigation_errors  # noqa: E402
from src.visualization.trajectory_plot import MUTED, TRUTH, _style  # noqa: E402

RUN_COLORS = ["#eb6834", "#1baf7a", "#8a5cd6", "#d6a12a", "#d64a8a", "#2aa7d6"]


def plot_navigation(
    flight: CameraFlight,
    runs: dict[str, NavigatorResult],
    path: Path,
    title: str = "",
    show_fixes_of: str | None = None,
) -> None:
    """Top view of truth and estimates, and the position error against the distance since the jam.

    Args:
        runs: label -> result, drawn in this order.
        show_fixes_of: label of the run whose fixes and stated uncertainty are drawn. Defaults to the last run.
    """
    if not runs:
        raise ValueError("nothing to plot")
    show_fixes_of = show_fixes_of or list(runs)[-1]
    truth = flight.position_gt
    jam = next(iter(runs.values())).start_index

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8.5))
    ax1.plot(truth[:, 1], truth[:, 0], color=TRUTH, lw=2.5, label="True path")
    for (label, result), color in zip(runs.items(), RUN_COLORS):
        ax1.plot(result.position[:, 1], result.position[:, 0], color=color, lw=1.3, label=label)
    main = runs[show_fixes_of]
    used = np.array([f.position for f in main.fixes if f.used]).reshape(-1, 2)
    rejected = np.array([f.position for f in main.fixes if not f.used]).reshape(-1, 2)
    if len(used):
        ax1.plot(used[:, 1], used[:, 0], "o", color=MUTED, ms=4, label=f"Fix used ({show_fixes_of})")
    if len(rejected):
        ax1.plot(rejected[:, 1], rejected[:, 0], "x", color="#c0392b", ms=7, mew=2, label="Fix rejected")
    ax1.plot(truth[jam, 1], truth[jam, 0], "o", color=MUTED, ms=9, mfc="none", mew=2, label="GNSS lost")
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.set_title(title or f"Camera navigator, {flight.name}")
    ax1.legend(frameon=False, fontsize=8, ncol=2)
    _style(ax1)

    for (label, result), color in zip(runs.items(), RUN_COLORS):
        errors = navigation_errors(result, flight)
        ax2.plot(errors.distance_since_jam, np.maximum(errors.error, 0.1), color=color, lw=1.6, label=label)
        if label == show_fixes_of:
            ax2.plot(errors.distance_since_jam, 3 * errors.sigma, color=color, lw=1, ls=":", label=f"3 x stated uncertainty ({label})")
    ax2.set_xlabel("Distance flown since GNSS was lost (m)")
    ax2.set_ylabel("Position error (m)")
    ax2.set_yscale("log")
    ax2.set_ylim(1, 2000)
    ax2.set_xlim(left=0)
    ax2.legend(frameon=False, fontsize=8, ncol=2)
    _style(ax2)

    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_stanford(errors: dict[str, "NavigationErrors"], alert_limit_m: float, path: Path, title: str = "", sigmas: float = 3.0) -> None:
    """Stanford-ESA integrity diagram: each frame's error against the bound the navigator stated.

    Below the diagonal the stated bound held. Right of the alert limit the navigator warns. The
    corner above the alert limit and left of it is hazardous: a large error with an all-clear.
    """
    fig, axes = plt.subplots(1, len(errors), figsize=(5.2 * len(errors), 4.8), squeeze=False)
    for ax, (label, err) in zip(axes[0], errors.items()):
        protection = np.maximum(sigmas * err.sigma, 0.1)
        e = np.maximum(err.error, 0.1)
        top = max(10 * alert_limit_m, float(e.max()) * 1.2, float(protection.max()) * 1.2)
        bins = np.logspace(-1, np.log10(top), 60)
        h, xe, ye = np.histogram2d(protection, e, bins=[bins, bins])
        ax.pcolormesh(xe, ye, np.ma.masked_equal(h.T, 0), cmap="viridis", norm=matplotlib.colors.LogNorm(), shading="auto")
        ax.plot([0.1, top], [0.1, top], color=MUTED, lw=1)
        ax.axvline(alert_limit_m, color="#d64a4a", lw=1)
        ax.axhline(alert_limit_m, color="#d64a4a", lw=1)
        ax.fill_between([0.1, alert_limit_m], alert_limit_m, top, color="#d64a4a", alpha=0.08, lw=0)
        ax.text(0.15, top / 1.6, "hazardous:\nlarge error,\nall-clear", fontsize=8, color="#b03030", va="top")
        ax.text(alert_limit_m * 1.15, 0.15, "navigator warns", fontsize=8, color=MUTED)
        ax.set_xscale("log"); ax.set_yscale("log")
        ax.set_xlim(0.1, top); ax.set_ylim(0.1, top)
        ax.set_xlabel(f"stated bound, {sigmas:g} sigma (m)")
        ax.set_ylabel("true error (m)")
        ax.set_title(label, fontsize=10)
        _style(ax)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)

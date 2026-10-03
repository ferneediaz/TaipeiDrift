"""Plots for the ESKF ablation: trajectories, errors, biases, NIS and NEES."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.stats import chi2  # noqa: E402

TRUTH = "#0b0b0b"
STYLE = {
    "imu_only": ("#eb6834", "IMU only"),
    "imu_baro": ("#eda100", "IMU + baro"),
    "forward_rotation": ("#2a78d6", "+ forward rotation"),
    "down_flow": ("#1baf7a", "+ down flow"),
    "both": ("#4a3aa7", "+ both cameras"),
}
GRID = "#e4e3df"
MUTED = "#52514e"


def _style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def _save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_trajectories(gt_p: np.ndarray, k0: int, runs: dict[str, np.ndarray], title: str, path: Path) -> None:
    """Top view (east, north); NED positions."""
    fig, ax = plt.subplots(figsize=(7.5, 7))
    ax.plot(gt_p[:, 1], gt_p[:, 0], color=TRUTH, lw=2.2, label="Ground truth")
    for name, p in runs.items():
        c, lab = STYLE[name]
        ax.plot(p[:, 1], p[:, 0], color=c, lw=1.8, label=lab)
    ax.plot(gt_p[k0, 1], gt_p[k0, 0], "o", color=MUTED, ms=8, label="GNSS lost")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="datalim")
    _style(ax)
    fig.legend(*ax.get_legend_handles_labels(), loc="lower center", ncol=3, frameon=False)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_series(t: np.ndarray, series: dict[str, np.ndarray], ylabel: str, title: str, path: Path, log: bool = False) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for name, y in series.items():
        c, lab = STYLE[name]
        ax.plot(t, y, color=c, lw=2, label=lab)
    ax.set_xlabel("Time since GNSS loss (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_xlim(left=0)
    if log:
        ax.set_yscale("log")
    else:
        ax.set_ylim(bottom=0)
    ax.legend(frameon=False)
    _style(ax)
    _save(fig, path)


def plot_biases(t: np.ndarray, est: dict[str, tuple[np.ndarray, np.ndarray]], true_g: np.ndarray, true_a: np.ndarray,
                title: str, path: Path) -> None:
    """Gyro (mrad/s) and accelerometer (m/s^2) bias per axis: estimates against the truth."""
    fig, axes = plt.subplots(2, 3, figsize=(13, 6.5), sharex=True)
    for row, (label, scale, truth) in enumerate([("Gyro bias (mrad/s)", 1e3, true_g), ("Accel bias (m/s²)", 1.0, true_a)]):
        for i, axis in enumerate("xyz"):
            ax = axes[row, i]
            ax.plot(t, truth[:, i] * scale, color=TRUTH, lw=2, label="Truth (injected + native)")
            for name, (bg, ba) in est.items():
                c, lab = STYLE[name]
                ax.plot(t, (bg if row == 0 else ba)[:, i] * scale, color=c, lw=1.6, label=lab)
            ax.set_title(f"{label}, {axis}")
            _style(ax)
            if row == 1:
                ax.set_xlabel("Time since GNSS loss (s)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.suptitle(title)
    fig.legend(handles, labels, loc="lower center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_nis(updates: dict[str, tuple[np.ndarray, np.ndarray, int]], title: str, path: Path) -> None:
    """NIS of accepted updates over time, with the 95 % chi-square band for one sample."""
    kinds = [k for k in updates if len(updates[k][0])]
    fig, axes = plt.subplots(len(kinds) or 1, 1, figsize=(9, 3.2 * max(1, len(kinds))), squeeze=False)
    for ax, kind in zip(axes[:, 0], kinds):
        t, v, dof = updates[kind]
        lo, hi = chi2.ppf(0.025, dof), chi2.ppf(0.975, dof)
        ax.axhspan(lo, hi, color=GRID, label="95 % band")
        ax.axhline(dof, color=MUTED, lw=1, ls="--", label=f"expected {dof}")
        ax.plot(t, v, ".", ms=3, color=STYLE["both"][0], label=f"{kind} NIS")
        ax.set_yscale("log")
        ax.set_ylabel("NIS")
        ax.set_title(f"{title}: {kind} updates (dof {dof})")
        ax.legend(frameon=False, loc="upper right")
        _style(ax)
    axes[-1, 0].set_xlabel("Time since GNSS loss (s)")
    _save(fig, path)

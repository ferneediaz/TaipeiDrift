"""Benchmark plots. Every trajectory is drawn as a point so outliers stay visible."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from vio.benchmark.aggregate import value  # noqa: E402

METHOD_STYLE = {"imu_only": ("#eb6834", "IMU only"), "visual": ("#2a78d6", "Visual (complementary)"),
                "ground_truth_attitude_oracle": ("#1baf7a", "Oracle: true attitude")}
COND_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7"]
GRID, INK = "#e4e3df", "#52514e"


def _style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def _save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _strip(ax, groups: list[tuple[str, np.ndarray, str]], log: bool):
    """Box (median, quartiles, 5-95 % whiskers) plus every value as a jittered point."""
    rng = np.random.default_rng(0)
    for i, (label, vals, color) in enumerate(groups):
        v = vals[np.isfinite(vals)]
        if v.size == 0:
            continue
        ax.boxplot([v], positions=[i], widths=0.5, whis=(5, 95), showfliers=False, patch_artist=True,
                   boxprops=dict(facecolor="none", edgecolor=INK), medianprops=dict(color=INK, lw=2),
                   whiskerprops=dict(color=INK), capprops=dict(color=INK))
        ax.scatter(i + rng.uniform(-0.18, 0.18, v.size), v, s=14, color=color, alpha=0.75, zorder=3, linewidths=0)
        ax.annotate(f"n={v.size}\nmed {np.median(v):.3g}", (i, 1.0), xycoords=("data", "axes fraction"),
                    ha="center", va="bottom", fontsize=8, color=INK)
    ax.set_xticks(range(len(groups)), [g[0] for g in groups])
    if log:
        ax.set_yscale("log")
    _style(ax)


def method_distribution(results, quantity, field, methods, ylabel, title, path, log=True):
    fig, ax = plt.subplots(figsize=(7.5, 5))
    groups = [(METHOD_STYLE[m][1], np.array([value(r, m, quantity, field) for r in results]), METHOD_STYLE[m][0]) for m in methods]
    _strip(ax, groups, log)
    ax.set_ylabel(ylabel)
    ax.set_title(title, pad=28)
    _save(fig, path)


def horizon_distribution(results, quantity, horizons, methods, ylabel, title, path, log=True):
    fig, axes = plt.subplots(1, len(horizons), figsize=(4 * len(horizons), 4.8), sharey=True)
    for ax, h in zip(np.atleast_1d(axes), horizons):
        groups = [(METHOD_STYLE[m][1].split(" (")[0].replace("Oracle: true attitude", "Oracle"),
                   np.array([value(r, m, quantity, h) for r in results]), METHOD_STYLE[m][0]) for m in methods]
        _strip(ax, groups, log)
        ax.set_title(f"{h} after GNSS loss" if h != "final" else "end of flight", pad=26)
        ax.tick_params(axis="x", labelrotation=20)
    np.atleast_1d(axes)[0].set_ylabel(ylabel)
    fig.suptitle(title)
    _save(fig, path)


def paired_scatter(results, quantity, field, label, title, path, log=True):
    pairs = [r for r in results if r["methods"].get("visual")]
    conds = sorted({r["condition"] for r in pairs})
    fig, ax = plt.subplots(figsize=(6.5, 6.2))
    allv = []
    for i, c in enumerate(conds):
        sel = [r for r in pairs if r["condition"] == c]
        x = np.array([value(r, "imu_only", quantity, field) for r in sel])
        y = np.array([value(r, "visual", quantity, field) for r in sel])
        allv += list(x) + list(y)
        ax.scatter(x, y, s=28, color=COND_COLORS[i % len(COND_COLORS)], label=c, edgecolors="white", linewidths=0.6, zorder=3)
    if allv:
        lo, hi = np.nanmin(allv), np.nanmax(allv)
        lo = lo * 0.8 if log else lo - 0.05 * (hi - lo)
        ax.plot([lo, hi * 1.2], [lo, hi * 1.2], color=INK, lw=1, ls="--", label="no change")
        ax.set_xlim(lo, hi * 1.2)
        ax.set_ylim(lo, hi * 1.2)
    if log:
        ax.set_xscale("log")
        ax.set_yscale("log")
    ax.set_xlabel(f"IMU only, {label}")
    ax.set_ylabel(f"Visual, {label}")
    ax.set_title(title + "\nbelow the line = vision helps")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(frameon=False, loc="upper left")
    _style(ax)
    _save(fig, path)


def oracle_gap_plot(results, path):
    rows = sorted(results, key=lambda r: value(r, "imu_only", "position_m", "final"))
    imu = np.array([value(r, "imu_only", "position_m", "final") for r in rows])
    vis = np.array([value(r, "visual", "position_m", "final") for r in rows])
    orc = np.array([value(r, "ground_truth_attitude_oracle", "position_m", "final") for r in rows])
    x = np.arange(len(rows))
    fig, ax = plt.subplots(figsize=(max(9, 0.12 * len(rows)), 5))
    ax.vlines(x, orc, imu, color=GRID, lw=2, zorder=1)
    ax.scatter(x, imu, s=16, color=METHOD_STYLE["imu_only"][0], label="IMU only", zorder=3)
    ax.scatter(x, vis, s=16, color=METHOD_STYLE["visual"][0], label="Visual", zorder=4)
    ax.scatter(x, orc, s=16, color=METHOD_STYLE["ground_truth_attitude_oracle"][0], label="Oracle: true attitude (not an estimator)", zorder=3)
    ax.set_yscale("log")
    ax.set_xlabel("Trajectories, sorted by IMU-only final error")
    ax.set_ylabel("Final position error (m)")
    ax.set_title("Gap to the perfect-attitude oracle (grey line = IMU - oracle)")
    ax.legend(frameon=False, loc="upper left")
    _style(ax)
    _save(fig, path)


def validity_vs_improvement(results, path):
    rows = [r for r in results if r.get("visual_statistics")]
    conds = sorted({r["condition"] for r in rows})
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    for ax, (q, lab) in zip(axes, [("attitude_deg", "attitude change (deg)"), ("position_m", "position change (m)")]):
        for i, c in enumerate(conds):
            sel = [r for r in rows if r["condition"] == c]
            x = [r["visual_statistics"]["valid_fraction"] * 100 for r in sel]
            y = [value(r, "visual", q, "final") - value(r, "imu_only", q, "final") for r in sel]
            ax.scatter(x, y, s=28, color=COND_COLORS[i % len(COND_COLORS)], label=c, edgecolors="white", linewidths=0.6)
        ax.axhline(0, color=INK, lw=1, ls="--")
        ax.set_xlabel("Valid visual measurements (%)")
        ax.set_ylabel(f"Final {lab}, visual - IMU")
        ax.set_title(f"Below 0 = vision helps ({q.split('_')[0]})")
        _style(ax)
    axes[0].legend(frameon=False)
    _save(fig, path)


def drift_vs_change(results, path):
    rows = [r for r in results if r["methods"].get("visual")]
    conds = sorted({r["condition"] for r in rows})
    fig, ax = plt.subplots(figsize=(7, 5))
    for i, c in enumerate(conds):
        sel = [r for r in rows if r["condition"] == c]
        x = [value(r, "imu_only", "attitude_deg", "final") for r in sel]
        y = [value(r, "visual", "attitude_deg", "final") - value(r, "imu_only", "attitude_deg", "final") for r in sel]
        ax.scatter(x, y, s=28, color=COND_COLORS[i % len(COND_COLORS)], label=c, edgecolors="white", linewidths=0.6)
    ax.axhline(0, color=INK, lw=1, ls="--")
    ax.set_xscale("log")
    ax.set_xlabel("IMU-only final attitude error (deg)")
    ax.set_ylabel("Final attitude error, visual - IMU (deg)")
    ax.set_title("Does vision help more when the gyro drifts more?\nbelow 0 = vision helps")
    ax.legend(frameon=False)
    _style(ax)
    _save(fig, path)


def make_all(results: list[dict], horizons: list[str], out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    allm = ["imu_only", "visual", "ground_truth_attitude_oracle"]
    method_distribution(results, "position_m", "final", allm, "Final position error (m)",
                        "Final position error after GNSS loss, all trajectories", out / "position_error_distribution.png")
    method_distribution(results, "attitude_deg", "final", ["imu_only", "visual"], "Final attitude error (deg)",
                        "Final attitude error after GNSS loss", out / "attitude_error_distribution.png")
    paired_scatter(results, "position_m", "final", "final position error (m)", "Final position error per trajectory",
                   out / "final_position_scatter.png")
    paired_scatter(results, "attitude_deg", "final", "final attitude error (deg)", "Final attitude error per trajectory",
                   out / "final_attitude_scatter.png")
    oracle_gap_plot(results, out / "oracle_gap.png")
    validity_vs_improvement(results, out / "visual_validity_vs_improvement.png")
    drift_vs_change(results, out / "imu_drift_vs_visual_change.png")
    horizon_distribution(results, "position_m", horizons, allm, "Position error (m)", "Position error by horizon",
                         out / "position_error_horizons.png")
    horizon_distribution(results, "attitude_deg", horizons, ["imu_only", "visual"], "Attitude error (deg)",
                         "Attitude error by horizon", out / "attitude_error_horizons.png")

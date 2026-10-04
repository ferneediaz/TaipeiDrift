"""Figures for Monte Carlo experiments (reads results/<experiment>/)."""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from trn.analysis.style import COLORS, LABELS, ROUTE_LABELS, plt, setup
from trn.analysis.summary import load_rows


def _summary(folder: Path) -> list[dict]:
    with open(folder / "summary.csv") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            if k not in ("route", "point", "filter"):
                r[k] = float(v)
    return rows


def plot_error_vs_time(folder: Path, point: str = "default", fname: str = "error_vs_time.png",
                       title: str = "") -> Path:
    """Median (line) and 25-75 % band (shaded) of horizontal error over runs, per route and filter."""
    setup()
    files = sorted(folder.glob(f"series_*_{point}.npz"))
    fig, axes = plt.subplots(1, len(files), figsize=(5.2 * len(files), 3.8), constrained_layout=True, squeeze=False)
    for ax, f in zip(axes[0], files):
        route = f.name[len("series_"):-len(f"_{point}.npz")]
        d = np.load(f)
        t = d["t"] / 60
        ins = np.hypot(d["ins_err"][..., 0], d["ins_err"][..., 1])
        ax.plot(t, np.median(ins, 0), color=COLORS["ins"], lw=1, ls=":", label=LABELS["ins"])
        for k in [k[:-4] for k in d.files if k.endswith("_err") and k != "ins_err"]:
            e = np.hypot(d[k + "_err"][..., 0], d[k + "_err"][..., 1])
            ax.plot(t, np.median(e, 0), color=COLORS[k], lw=1.6, label=LABELS[k])
            ax.fill_between(t, np.percentile(e, 25, 0), np.percentile(e, 75, 0), color=COLORS[k], alpha=0.15, lw=0)
        ax.set_yscale("log"); ax.set_ylim(1, 3e5)
        ax.set_title(f"{ROUTE_LABELS.get(route, route)} (n={d['runs'].size})")
        ax.set_xlabel("time since GNSS loss [min]")
    axes[0, 0].set_ylabel("horizontal error [m] (median, IQR band)")
    axes[0, 0].legend(fontsize=7, loc="upper left")
    if title:
        fig.suptitle(title)
    out = folder / fname
    fig.savefig(out)
    plt.close(fig)
    return out


def plot_sweep(folder: Path, key: str, xlabel: str, fname: str, xscale: float = 1.0, title: str = "",
               filters: list[str] | None = None, xlog: bool = False, fixed: dict | None = None) -> Path:
    """Median post-burn-in RMSE (bootstrap 95 % CI) and divergence rate vs. a swept parameter.

    ``fixed`` restricts the plot to sweep points with the given values of other swept keys.
    """
    setup()
    rows = load_rows(folder)
    if fixed:
        rows = [r for r in rows if all(str(r[k]) == str(v) for k, v in fixed.items())]
    routes = list(dict.fromkeys(r["route"] for r in rows))
    filters = filters or list(dict.fromkeys(r["filter"] for r in rows))
    keep = {r["point"] for r in rows}
    summ = [s for s in _summary(folder) if s["point"] in keep]
    xs_by_point = {r["point"]: float(r[key]) if not isinstance(r[key], str) else r[key] for r in rows}
    fig, axes = plt.subplots(2, len(routes), figsize=(4.8 * len(routes), 6.2), constrained_layout=True, squeeze=False)
    for j, route in enumerate(routes):
        for filt in filters:
            ss = sorted([s for s in summ if s["route"] == route and s["filter"] == filt],
                        key=lambda s: xs_by_point[s["point"]])
            if not ss:
                continue
            x = np.array([xs_by_point[s["point"]] for s in ss], float) * xscale
            y = np.array([s["rmse_post_median"] for s in ss])
            lo = np.array([s["rmse_post_ci_lo"] for s in ss]); hi = np.array([s["rmse_post_ci_hi"] for s in ss])
            axes[0, j].plot(x, y, "-o", ms=3, color=COLORS[filt], label=LABELS[filt])
            axes[0, j].fill_between(x, lo, hi, color=COLORS[filt], alpha=0.15, lw=0)
            dv = np.array([s["divergence_rate"] for s in ss])
            dlo = np.array([s["divergence_ci_lo"] for s in ss]); dhi = np.array([s["divergence_ci_hi"] for s in ss])
            axes[1, j].errorbar(x, 100 * dv, yerr=[100 * (dv - dlo), 100 * (dhi - dv)], fmt="-o", ms=3, capsize=2,
                                color=COLORS[filt], label=LABELS[filt])
        axes[0, j].set_yscale("log"); axes[0, j].set_title(ROUTE_LABELS.get(route, route))
        axes[1, j].set_ylim(-3, 103); axes[1, j].set_xlabel(xlabel)
        if xlog:
            axes[0, j].set_xscale("log"); axes[1, j].set_xscale("log")
    axes[0, 0].set_ylabel("RMSE after 5 min [m]\n(median, 95 % bootstrap CI)")
    axes[1, 0].set_ylabel("divergence rate [%]\n(final error > 1 km, Wilson 95 % CI)")
    axes[0, 0].legend(fontsize=7)
    if title:
        fig.suptitle(title)
    out = folder / fname
    fig.savefig(out)
    plt.close(fig)
    return out


def summary_table_md(folder: Path, filters: list[str] | None = None) -> str:
    """Markdown table of key metrics."""
    summ = _summary(folder)
    filters = filters or list(dict.fromkeys(s["filter"] for s in summ))
    lines = ["| route | point | filter | RMSE med [m] | CEP50 [m] | CEP95 [m] | final err med [m] | converged | "
             "diverged | false fix | NEES med | ground echo % | ms/step |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for s in summ:
        if s["filter"] not in filters:
            continue
        lines.append(f"| {s['route'].split('_')[0]} | {s['point']} | {LABELS.get(s['filter'], s['filter'])} | "
                     f"{s['rmse_post_median']:.0f} | {s['cep50_median']:.0f} | {s['cep95_median']:.0f} | "
                     f"{s['final_err_median']:.0f} | {100*s['converged_frac']:.0f}% | {100*s['divergence_rate']:.0f}% | "
                     f"{100*s['false_fix_rate']:.0f}% | {s['nees_median']:.1f} | {100*s['usable_ground_frac']:.0f} | "
                     f"{s['runtime_ms']:.2f} |")
    return "\n".join(lines)

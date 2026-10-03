"""M7: build all report figures and tables from finished experiments (+ two example runs).

    python -m trn.experiments.m7_report
Figures go to docs/figures/, tables to docs/results_tables.md.
"""
from __future__ import annotations

import shutil

import numpy as np

from trn.analysis.plots import plot_error_vs_time, plot_sweep, summary_table_md
from trn.analysis.style import COLORS, LABELS, ROUTE_LABELS, hillshade, plt, setup
from trn.common.config import apply_overrides, load_base_config, project_path
from trn.experiments.runner import run_once
from trn.terrain.metrics import roughness_along_track

DOCS = project_path("docs/figures")
RES = project_path("results")


def example_run(route: str, clouds: float, run: int, fname: str) -> None:
    """Map with tracks, error with +-3 sigma, terrain roughness overlay for one run."""
    setup()
    over = {"atmosphere.enabled": clouds > 0, "atmosphere.cloud_fraction": clouds}
    cfg = apply_overrides(load_base_config(), over)
    names = ["tercom", "mpf_baseline", "mpf_gated", "mpf_proposed"]
    r = run_once(cfg, route, run, filters=names, salt="example", keep=True)
    rd, ob = r["rundata"], r["onboard"]
    t = rd.t / 60
    fig = plt.figure(figsize=(13, 7.5))
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.5], hspace=0.45, wspace=0.12)
    axm = fig.add_subplot(gs[:, 0])
    p = rd.truth_pos
    pad = 4000
    g = ob.grid
    x0, x1 = p[:, 0].min() - pad, p[:, 0].max() + pad
    y0, y1 = p[:, 1].min() - pad, p[:, 1].max() + pad
    c0, c1 = int((x0 - g.x0) / g.dx), int((x1 - g.x0) / g.dx)
    r0, r1 = int((g.y0 - y1) / g.dx), int((g.y0 - y0) / g.dx)
    z = np.asarray(g.z[r0:r1, c0:c1])
    ext = (g.x0 + c0 * g.dx, g.x0 + c1 * g.dx, g.y0 - r1 * g.dx, g.y0 - r0 * g.dx)
    axm.imshow(hillshade(z, g.dx), extent=ext, cmap="gray")
    axm.imshow(z, extent=ext, cmap="terrain", alpha=0.35, vmin=0, vmax=3950)
    axm.plot(p[:, 0], p[:, 1], "k-", lw=2, label="truth")
    axm.plot(rd.ins_pos[:, 0], rd.ins_pos[:, 1], ":", color=COLORS["ins"], lw=1.5, label="INS only")
    for n in names:
        e = r["filters"][n]["est_full"]
        axm.plot(e[:, 0], e[:, 1], "-", color=COLORS[n], lw=1, label=LABELS[n])
    axm.set_xlim(x0, x1); axm.set_ylim(y0, y1); axm.set_xticks([]); axm.set_yticks([])
    axm.set_title(f"{ROUTE_LABELS[route]} — cloud fraction {clouds:.0%}\ntrue vs estimated tracks")
    axm.legend(fontsize=7, loc="lower left", frameon=True)
    for k, n in enumerate(["mpf_baseline", "mpf_proposed"]):
        ax = fig.add_subplot(gs[k, 1])
        e = r["filters"][n]["est_full"][:, :2] - p[:, :2]
        cov = r["filters"][n]["cov_full"]
        for j, (lab, col) in enumerate([("East", "#1f77b4"), ("North", "#ff7f0e")]):
            ax.plot(t, e[:, j], color=col, lw=0.8, label=f"{lab} error")
            s3 = 3 * np.sqrt(cov[:, j, j])
            ax.fill_between(t, -s3, s3, color=col, alpha=0.12, lw=0)
        lim = max(100.0, float(np.percentile(np.abs(e[t > 5]), 95)) * 1.5)
        ax.set_ylim(-lim, lim)
        ax.set_title(f"{LABELS[n]}: error and ±3σ bounds")
        ax.set_ylabel("error [m]"); ax.legend(fontsize=7, loc="upper right", ncol=2)
    ax = fig.add_subplot(gs[2, 1])
    ds = float(np.median(np.hypot(*np.diff(p[:, :2], axis=0).T)))
    rough = roughness_along_track(ob.grid.interp(p[:, 0], p[:, 1]), ds)
    for n in ["mpf_baseline", "mpf_proposed"]:
        e = r["filters"][n]["est_full"][:, :2] - p[:, :2]
        ax.plot(t, np.hypot(e[:, 0], e[:, 1]), color=COLORS[n], lw=1, label=LABELS[n])
    ax.set_yscale("log"); ax.set_ylabel("horizontal error [m]"); ax.set_xlabel("time [min]")
    ax2 = ax.twinx()
    ax2.fill_between(t, 0, rough, color="#9467bd", alpha=0.2, lw=0)
    ax2.set_ylabel("terrain roughness σ_1km [m]", color="#9467bd"); ax2.grid(False)
    lab = rd.laser.labels[:, 0, :]
    vis = ((lab == 1) | (lab == 6)).any(axis=1).astype(float)
    vis_s = np.convolve(vis, np.ones(300) / 300, mode="same")
    ax.plot(t, 10 ** (vis_s * 2 - 0.0), color="#2ca02c", lw=0.8, ls="--", label="ground-echo rate (30 s), 1..100 scale")
    ax.set_title("horizontal error vs terrain roughness (shaded) and ground-echo availability")
    ax.legend(fontsize=7, loc="upper left")
    fig.savefig(DOCS / fname, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    tables = []
    if (RES / "m4_baselines" / "summary.csv").exists():
        f = plot_error_vs_time(RES / "m4_baselines", "default", "m4_error_vs_time.png",
                               "M4: clear air, tactical IMU, nadir beam")
        shutil.copy(f, DOCS / f.name)
        tables.append("## M4 — clear air\n\n" + summary_table_md(RES / "m4_baselines"))
    if (RES / "m5_clouds" / "summary.csv").exists():
        f = plot_sweep(RES / "m5_clouds", "atmosphere.cloud_fraction", "cloud fraction [%]", "m5_cloud_sweep.png",
                       xscale=100, title="M5: navigation error vs. cloud fraction (paired runs)")
        shutil.copy(f, DOCS / f.name)
        for cf in ("0.5", "0.9"):
            f = plot_error_vs_time(RES / "m5_clouds", f"cloud_fraction={cf}", f"m5_error_vs_time_cf{cf}.png",
                                   f"M5: cloud fraction {float(cf):.0%}")
            shutil.copy(f, DOCS / f.name)
        tables.append("## M5 — cloud sweep\n\n" + summary_table_md(RES / "m5_clouds"))
    sweeps = [("m6_altitude", "trajectory.agl_m", "altitude AGL [m]", False),
              ("m6_imu", "imu.grade", "IMU grade", False),
              ("m6_baro", "imu.baro.enabled", "barometer", False)]
    for name, key, xl, xlog in sweeps:
        if (RES / name / "summary.csv").exists():
            if key in ("imu.grade", "imu.baro.enabled"):
                _categorical(RES / name, key, xl, f"{name}.png")
            else:
                f = plot_sweep(RES / name, key, xl, f"{name}.png", title=f"M6: {xl}")
                shutil.copy(f, DOCS / f.name)
            tables.append(f"## {name}\n\n" + summary_table_md(RES / name))
    for name in ("m6_beams", "m6_map"):
        if (RES / name / "summary.csv").exists():
            _categorical(RES / name, None, "configuration", f"{name}.png")
            tables.append(f"## {name}\n\n" + summary_table_md(RES / name))
    project_path("docs/results_tables.md").write_text("# Result tables (generated by m7_report)\n\n" +
                                                      "\n\n".join(tables) + "\n")
    example_run("A_mountain_crossing", 0.5, 1, "example_A_clouds50.png")
    example_run("C_foothills", 0.0, 1, "example_C_clear.png")
    print("report figures written to docs/figures, tables to docs/results_tables.md")


def _categorical(folder, key, xlabel, fname) -> None:
    """Grouped bars: median RMSE (with CI) and divergence rate per configuration, route and filter."""
    from trn.analysis.plots import _summary
    setup()
    summ = _summary(folder)
    routes = list(dict.fromkeys(s["route"] for s in summ))
    points = list(dict.fromkeys(s["point"] for s in summ))
    filters = list(dict.fromkeys(s["filter"] for s in summ))
    fig, axes = plt.subplots(2, len(routes), figsize=(4.8 * len(routes), 6), constrained_layout=True, squeeze=False)
    w = 0.8 / len(filters)
    for j, route in enumerate(routes):
        for i, f in enumerate(filters):
            ss = [next(s for s in summ if s["route"] == route and s["filter"] == f and s["point"] == p) for p in points]
            x = np.arange(len(points)) + (i - (len(filters) - 1) / 2) * w
            y = np.array([s["rmse_post_median"] for s in ss])
            err = np.array([[s["rmse_post_median"] - s["rmse_post_ci_lo"], s["rmse_post_ci_hi"] - s["rmse_post_median"]]
                            for s in ss]).T
            axes[0, j].bar(x, y, w, yerr=np.nan_to_num(err), color=COLORS[f], label=LABELS[f], capsize=2)
            axes[1, j].bar(x, [100 * s["divergence_rate"] for s in ss], w, color=COLORS[f])
        for a in axes[:, j]:
            a.set_xticks(np.arange(len(points)))
            a.set_xticklabels([p.replace("__", "\n").split("=")[-1] if "__" not in p else p.replace("__", "\n")
                               for p in points], fontsize=7)
        axes[0, j].set_yscale("log"); axes[0, j].set_title(ROUTE_LABELS.get(route, route))
        axes[1, j].set_ylim(0, 100); axes[1, j].set_xlabel(xlabel)
    axes[0, 0].set_ylabel("RMSE after 5 min [m] (median, 95 % CI)")
    axes[1, 0].set_ylabel("divergence rate [%]")
    axes[0, 0].legend(fontsize=7)
    fig.savefig(folder / fname); fig.savefig(DOCS / fname)
    plt.close(fig)


if __name__ == "__main__":
    main()

"""Figures of the simulated flights for docs/simulation-results.md.

    python scripts/sim_figures.py      # after scripts/sim_progress.py

docs/figures/sim_flights.png   the six flights over the Wufeng photo, and their roles
docs/figures/sim_progress.png  the improvement loop: error and wrong fixes per development flight, stage by stage
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
FIGURES = REPO / "docs" / "figures"
ORDER = ["wufeng_corridor_100m", "wufeng_north_120m", "wufeng_south_65m", "wufeng_south_80m", "wufeng_north_90m", "wufeng_south_110m"]


def flights_figure(cfg: dict) -> None:
    import rasterio
    from rasterio.enums import Resampling

    roles = {**{n: "development" for n in cfg["flights"]["development"]}, **{n: "held out" for n in cfg["flights"]["held_out"]},
             **{n: "sealed" for n in cfg["flights"]["sealed"]}}
    routes = {**cfg["flights"]["development"], **cfg["flights"]["held_out"], **cfg["flights"]["sealed"]}
    north = json.loads((REPO / "sim/scenarios/wufeng_corridor.json").read_text())
    south = json.loads((REPO / "sim/scenarios/wufeng_south_80m.json").read_text())
    oe, on = north["origin_easting_m"], north["origin_northing_m"]
    with rasterio.open(REPO / cfg["maps"]["2020"]) as src:
        f = 8
        img = src.read([1, 2, 3], out_shape=(3, src.height // f, src.width // f), resampling=Resampling.average)
        b = src.bounds
    img = np.transpose(img, (1, 2, 0)).copy()
    img[img.sum(axis=2) == 0] = 255  # outside the photo: white

    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_axes([0.06, 0.06, 0.42, 0.82])
    ax.imshow(img, extent=[b.left - oe, b.right - oe, b.bottom - on, b.top - on])
    for plan, colour, style, label in ((south, "#00c8ff", "-", "south first: 80, 65, 110 m"),
                                       (north, "#ffcc00", (0, (6, 4)), "north first: 100, 120, 90 m")):
        w = np.array(plan["waypoints_xy_m"])
        ax.plot(w[:, 0], w[:, 1], color=colour, lw=2.6, ls=style, label=label)
        s = np.r_[0, np.cumsum(np.hypot(*np.diff(w, axis=0).T))]
        ax.plot(*w[np.searchsorted(s, 450)], "o", ms=13, mfc="none", mec=colour, mew=3)
        for frac in (0.2, 0.45, 0.7, 0.92):
            i = np.searchsorted(s, frac * s[-1])
            d = (w[i + 1] - w[i]) / np.linalg.norm(w[i + 1] - w[i])
            ax.annotate("", xy=w[i] + d * 60, xytext=w[i], arrowprops=dict(arrowstyle="-|>", color=colour, lw=2.4, mutation_scale=18))
    ax.plot(0, 0, "*", color="white", ms=18, mec="k", mew=1.2, label="take-off and climb")
    ax.plot([], [], "o", mfc="none", mec="k", mew=2.5, ms=11, label="GNSS lost, after 450 m")
    ax.set_xlim(-1250, 1250)
    ax.set_ylim(-1650, 1450)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.legend(loc="lower left", fontsize=9.5, framealpha=0.95)
    fig.text(0.27, 0.955, "Simulated flights over Wufeng, Taichung", ha="center", fontsize=14, weight="bold")
    fig.text(0.27, 0.925, "ground: the real 2020 aerial photo; the navigator's map: the 2018 photo of the same place", ha="center", fontsize=10)

    tx = fig.add_axes([0.52, 0.06, 0.46, 0.86])
    tx.axis("off")
    rows = []
    for name in ORDER:
        plan = json.loads((REPO / routes[name]).read_text())
        frames = sum(1 for _ in open(REPO / "recordings" / name / "images.csv")) - 1
        rows.append([name.replace("wufeng_", ""), f"{plan['altitude_m']:.0f} m", f"{plan['speed_mps']:g} m/s",
                     f"{plan['length_m'] / 1000:.1f} km", f"{frames:,}", roles[name]])
    t = tx.table(cellText=rows, colLabels=["flight", "height", "speed", "route", "frames", "role"], bbox=[0.0, 0.62, 1.0, 0.36],
                 cellLoc="left", colLoc="left", colWidths=[0.27, 0.12, 0.13, 0.12, 0.12, 0.2])
    t.auto_set_font_size(False)
    t.set_fontsize(10.5)
    shade = {"development": "#eaf6ea", "held out": "#fff4d6", "sealed": "#ececec"}
    for (r, c), cell in t.get_celld().items():
        cell.set_edgecolor("#bbbbbb")
        if r == 0:
            cell.set_text_props(weight="bold")
            cell.set_facecolor("#dddddd")
        else:
            cell.set_facecolor(shade[rows[r - 1][5]])
    tx.text(0.0, 0.55, "How the flights are used", weight="bold", fontsize=12, transform=tx.transAxes, va="top")
    tx.text(0.0, 0.50, "Roles fixed at 15:20, before the new flights were recorded.\n\n"
            "Development: the navigator may be changed after looking at them.\n"
            "Held out: run once with frozen settings; that first result is reported.\n"
            "Sealed: not run and not looked at until the solution is frozen;\n    then run once, and those are the numbers we quote.\n\n"
            "All fly the same corridor, the only ground both photos cover;\nthey differ in height, speed and direction.\n\n"
            "Simulator: Gazebo Harmonic in Docker, a Mid-Air-like quadcopter;\ndown camera 512 px, 90 degrees, 5 frames per second;\n"
            "IMU, barometer and GNSS with noise.", fontsize=10.5, transform=tx.transAxes, va="top", linespacing=1.35)
    fig.savefig(FIGURES / "sim_flights.png", dpi=110)
    plt.close(fig)


def progress_figure(data: dict) -> None:
    stages, flights, res = data["stages"], data["flights"], data["results"]
    labels = [s.split(" ", 1)[1] for s in stages]
    fig, axes = plt.subplots(1, len(flights), figsize=(15, 6.2), sharey=True)
    x = np.arange(len(stages))
    for ax, name in zip(np.atleast_1d(axes), flights):
        alone = np.median([r["median"] for r in res[stages[-1]][name]["camera_alone"]])
        ax.axhline(alone, color="#e8743b", ls="--", lw=1.6, label=f"camera alone, sun heading: median {alone:.0f} m")
        for i, stage in enumerate(stages):
            m = res[stage][name]["map_2018"]
            med, p90, worst = (float(np.median([r[k] for r in m])) for k in ("median", "p90", "worst"))
            wrong = sum(int(r["used_but_wrong"]) for r in m)
            works = wrong == 0 and min(r["within_3_sigma"] for r in m) >= 0.99 and max(r["integrity_hazardous"] for r in m) == 0
            colour = "#2a9d4b" if works else "#c0392b"
            ax.vlines(i, med, worst, color=colour, lw=1.2, alpha=0.6)
            ax.vlines(i, med, p90, color=colour, lw=5, alpha=0.8)
            ax.plot(i, med, "o", color=colour, ms=10, mec="white", mew=1.5)
            ax.text(i, worst + 6, f"{worst:.0f}", ha="center", fontsize=8, color="#555555")
            ax.text(i, -16, "works" if works else f"fails\n{wrong} wrong" if wrong else "fails", ha="center", va="top", fontsize=8.5,
                    color=colour, weight="bold")
            ax.text(i + 0.12, med, f"{med:.0f}", va="center", fontsize=9)
        ax.set_xticks(x)
        ax.set_xticklabels([f"{i + 1}" for i in x])
        ax.set_xlim(-0.6, len(stages) - 0.4)
        ax.set_ylim(-45, 260)
        ax.set_title(name.replace("wufeng_", "flight "), fontsize=11)
        ax.set_xlabel("stage")
        ax.grid(axis="y", alpha=0.3)
        ax.legend(loc="upper right", fontsize=8.5)
    np.atleast_1d(axes)[0].set_ylabel("position error with the 2018 map (m)")
    fig.suptitle("Step by step on the development flights: one change per stage, each kept only if every flight works",
                 fontsize=12.5, weight="bold", y=0.99)
    fig.text(0.5, 0.905, "dot: median over 3 heading-sensor draws; thick bar: up to the 90th percentile; thin line and number: worst. "
             "Green: works (no wrong fix used, stated bound held 99 percent, never \"within 50 m\" while further off).",
             ha="center", fontsize=9)
    fig.text(0.5, 0.015, "Stages:  " + "     ".join(f"{i + 1}  {lab}" for i, lab in enumerate(labels)), ha="center", fontsize=9.5)
    fig.subplots_adjust(left=0.06, right=0.99, top=0.84, bottom=0.17, wspace=0.08)
    fig.savefig(FIGURES / "sim_progress.png", dpi=110)
    plt.close(fig)


def main() -> None:
    cfg = yaml.safe_load((REPO / "baseline/configs/sim_navigator.yaml").read_text())
    FIGURES.mkdir(parents=True, exist_ok=True)
    flights_figure(cfg)
    progress = REPO / "outputs" / "sim_progress.json"
    if progress.is_file():
        progress_figure(json.loads(progress.read_text()))
    print("figures written to docs/figures/")


if __name__ == "__main__":
    main()

"""M1: onboard-map generation, map-difference report (truth DSM vs onboard map) and route terrain plots.

    python -m trn.experiments.m1_maps
Outputs: results/m1_maps/ (report.json, figures) - also copied to docs/figures/.
"""
from __future__ import annotations

import json
import shutil

import numpy as np

from trn.analysis.style import ROUTE_LABELS, hillshade, plt, setup
from trn.common.config import load_base_config, project_path
from trn.terrain.metrics import roughness_along_track, slope_grid
from trn.terrain.onboard_builder import build_onboard_map
from trn.trajectory.fixed_wing import generate
from trn.truth.truth_map import TruthMap

SUB = 3  # subsample DSM posts for statistics (every 3rd row/col -> ~2.5 M samples per tile)


def _stats(x: np.ndarray) -> dict:
    x = x[np.isfinite(x)]
    q = np.percentile(x, [5, 50, 95])
    return dict(n=int(x.size), bias=float(x.mean()), std=float(x.std()), median=float(q[1]), p05=float(q[0]),
                p95=float(q[2]), rms=float(np.sqrt(np.mean(x ** 2))),
                robust_std=float(1.4826 * np.median(np.abs(x - q[1]))))


def main() -> None:
    setup()
    cfg = load_base_config()
    out = project_path(cfg["data"]["results_dir"]) / "m1_maps"
    out.mkdir(parents=True, exist_ok=True)
    report = {}
    fig_h, axh = plt.subplots(1, 3, figsize=(13, 3.4), constrained_layout=True)
    fig_m, axm = plt.subplots(1, 3, figsize=(14, 6), constrained_layout=True)
    fig_p, axp = plt.subplots(3, 1, figsize=(11, 8.5), constrained_layout=True)
    for k, route in enumerate(cfg["routes"]):
        tm = TruthMap.load(cfg, route)
        ob = build_onboard_map(cfg, route)
        top = np.asarray(tm.top.z)[::SUB, ::SUB].astype(np.float64)
        gnd = np.asarray(tm.ground.z)[::SUB, ::SUB].astype(np.float64)
        sea = np.asarray(tm.sea)[::SUB, ::SUB].astype(bool)
        h, w = top.shape
        X = tm.top.x0 + np.arange(w) * tm.top.dx * SUB
        Y = tm.top.y0 - np.arange(h) * tm.top.dx * SUB
        XX, YY = np.meshgrid(X, Y)
        onb = ob.grid.interp(XX.ravel(), YY.ravel()).reshape(h, w)
        land = ~sea & np.isfinite(top) & np.isfinite(onb)
        d_top = np.where(land, top - onb, np.nan)          # what a top-surface laser sees vs onboard map
        d_bare = np.where(land, gnd - onb, np.nan)         # bare-earth map error (resampling, 2025 vs 2025)
        canopy = top - gnd
        slope = slope_grid(gnd, tm.top.dx * SUB)
        cls = {"plain (slope<2°, h<100 m)": (np.degrees(np.arctan(slope)) < 2) & (gnd < 100),
               "hills (100-1000 m)": (gnd >= 100) & (gnd < 1000),
               "mountain (h>=1000 m)": gnd >= 1000}
        veg = {"bare (DSM-DEM<1 m)": canopy < 1.0, "vegetated/built (DSM-DEM>5 m)": canopy > 5.0}
        r = dict(onboard=ob.description, top_minus_onboard=_stats(d_top), bare_minus_onboard=_stats(d_bare),
                 canopy_frac_gt2m=float(np.nanmean((canopy > 2)[land])),
                 by_terrain={n: dict(frac=float(np.mean(m[land])), top=_stats(d_top[m]), bare=_stats(d_bare[m]))
                             for n, m in cls.items() if np.any(m & land)},
                 by_vegetation={n: dict(frac=float(np.mean(m[land])), top=_stats(d_top[m]))
                                for n, m in veg.items() if np.any(m & land)})
        report[route] = r
        # histogram
        ax = axh[k]
        bins = np.linspace(-20, 50, 141)
        ax.hist(d_top[land], bins, density=True, alpha=0.6, color="#2ca02c", label="DSM (truth top) − onboard")
        ax.hist(d_bare[land], bins, density=True, alpha=0.6, color="#1f77b4", label="DEM 20 m − onboard 30 m")
        ax.set_title(f"{ROUTE_LABELS[route]}\nDSM−onboard: bias {r['top_minus_onboard']['bias']:.1f} m, "
                     f"σ {r['top_minus_onboard']['std']:.1f} m | bare σ {r['bare_minus_onboard']['std']:.1f} m")
        ax.set_xlabel("height difference [m]"); ax.set_yscale("log")
        if k == 0:
            ax.legend(loc="upper right")
        # map image
        ax = axm[k]
        ext = (X[0], X[-1], Y[-1], Y[0])
        ax.imshow(hillshade(gnd, tm.top.dx * SUB), extent=ext, cmap="gray", alpha=1.0)
        im = ax.imshow(d_top, extent=ext, cmap="RdBu_r", vmin=-30, vmax=30, alpha=0.75)
        traj = generate(cfg, route, tm.top)
        ax.plot(traj.pos[::100, 0], traj.pos[::100, 1], "k-", lw=1.5)
        ax.set_title(f"{ROUTE_LABELS[route]}: DSM − onboard map")
        ax.set_xticks([]); ax.set_yticks([])
        # route profile
        ax = axp[k]
        p = traj.pos[::20]
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(p[:, :2], axis=0).T))]) / 1000
        z_top = tm.top.interp(p[:, 0], p[:, 1]); z_gnd = tm.ground.interp(p[:, 0], p[:, 1])
        z_onb = ob.grid.interp(p[:, 0], p[:, 1])
        ax.fill_between(s, 0, z_gnd, color="#c7b299", alpha=0.6, label="ground (DEM 20 m)")
        ax.plot(s, z_top, color="#2ca02c", lw=0.6, label="top surface (DSM, truth)")
        ax.plot(s, z_onb, color="#1f77b4", lw=0.6, ls="--", label="onboard map (30 m)")
        ax.plot(s, p[:, 2], color="k", lw=1.2, label="aircraft altitude")
        rough = roughness_along_track(z_gnd, (s[1] - s[0]) * 1000)
        ax2 = ax.twinx(); ax2.plot(s, rough, color="#9467bd", lw=0.8); ax2.set_ylabel("roughness σ_1km [m]", color="#9467bd")
        ax2.grid(False)
        ax.set_title(f"{ROUTE_LABELS[route]} — {s[-1]:.0f} km, mean roughness {np.nanmean(rough):.0f} m")
        ax.set_ylabel("height MSL [m]")
        report[route]["route_length_km"] = float(s[-1])
        report[route]["mean_roughness_m"] = float(np.nanmean(rough))
        if k == 0:
            ax.legend(loc="upper right", ncol=4, fontsize=7)
    axp[-1].set_xlabel("distance along track [km]")
    fig_m.colorbar(im, ax=axm, shrink=0.6, label="DSM − onboard [m] (red: canopy/buildings above map)")
    fig_h.savefig(out / "m1_map_difference_hist.png"); fig_m.savefig(out / "m1_map_difference_maps.png")
    fig_p.savefig(out / "m1_route_profiles.png")
    (out / "report.json").write_text(json.dumps(report, indent=1))
    docs = project_path("docs/figures")
    for f in out.glob("*.png"):
        shutil.copy(f, docs / f.name)
    for route, r in report.items():
        print(route, f"DSM-onboard bias {r['top_minus_onboard']['bias']:.2f} std {r['top_minus_onboard']['std']:.2f} | "
                     f"bare std {r['bare_minus_onboard']['std']:.2f} robust {r['bare_minus_onboard']['robust_std']:.2f} | "
                     f"canopy>2m {r['canopy_frac_gt2m']:.2f}")
        for n, v in r["by_terrain"].items():
            print(f"   {n:28s} frac {v['frac']:.2f} top bias {v['top']['bias']:6.2f} std {v['top']['std']:6.2f} | bare std {v['bare']['std']:.2f}")
        for n, v in r["by_vegetation"].items():
            print(f"   {n:28s} frac {v['frac']:.2f} top bias {v['top']['bias']:6.2f} std {v['top']['std']:6.2f}")


if __name__ == "__main__":
    main()

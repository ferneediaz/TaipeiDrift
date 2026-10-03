"""M2: laser altimeter in clear air (and one cloudy illustration): simulated vs true terrain profile.

    python -m trn.experiments.m2_laser
"""
from __future__ import annotations

import json
import shutil

import numpy as np

from trn.analysis.style import ROUTE_LABELS, plt, setup
from trn.atmosphere.clouds import build_atmosphere
from trn.common.config import apply_overrides, load_base_config, project_path
from trn.laser.altimeter import LABEL_NAMES, simulate_laser
from trn.trajectory.fixed_wing import generate
from trn.truth.truth_map import TruthMap

LCOL = {1: "#1f77b4", 2: "#2ca02c", 3: "#7f7f7f", 4: "#bcbd22", 5: "#d62728", 6: "#17becf"}


def main() -> None:
    setup()
    base = load_base_config()
    out = project_path(base["data"]["results_dir"]) / "m2_laser"
    out.mkdir(parents=True, exist_ok=True)
    rep = {}
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), constrained_layout=True)
    scen = [("A_mountain_crossing", {}, 52.0), ("C_foothills", {}, 30.0), ("B_coastal_plain", {}, 40.0),
            ("A_mountain_crossing", {"atmosphere.enabled": True, "atmosphere.cloud_fraction": 0.5}, 10.0)]
    for ax, (route, over, s0) in zip(axes, scen):
        cfg = apply_overrides(base, over)
        tm = TruthMap.load(cfg, route)
        tr = generate(cfg, route, tm.top)
        rng = np.random.default_rng(42)
        atm = build_atmosphere(cfg, route, tm.ground, rng)
        idx = np.arange(0, tr.n, 10)
        L = simulate_laser(cfg["laser"], tm, atm, tr.t[idx], tr.pos[idx], tr.C[idx], rng)
        p = tr.pos[idx]
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(p[:, :2], axis=0).T))]) / 1000
        r, lab = L.ranges[:, 0, :], L.labels[:, 0, :]
        d = np.einsum("nij,j->ni", tr.C[idx], L.beams_body[0])        # body-fixed beam (tilts in turns)
        last = np.array([x[np.isfinite(x)][-1] if np.isfinite(x).any() else np.nan for x in r])
        res_top = last - L.r_true[:, 0]
        key = f"{route}{'_clouds50' if over else ''}"
        counts = {LABEL_NAMES[k]: float((lab == k).any(axis=1).mean()) for k in range(1, 7)}
        counts["no echo"] = float((~np.isfinite(r)).all(axis=1).mean())
        rep[key] = dict(echo_presence=counts, max_true_range=float(np.nanmax(L.r_true)),
                        last_echo_minus_top_median=float(np.nanmedian(res_top)))
        win = 6.0 if over else 2.0
        m = (s >= s0) & (s <= s0 + win)
        ax.plot(s[m], tm.top.interp(p[m, 0], p[m, 1]), color="#2ca02c", lw=1, label="DSM top surface (truth)")
        ax.plot(s[m], tm.ground.interp(p[m, 0], p[m, 1]), color="#8c564b", lw=1, label="ground (DEM 20 m)")
        for k, c in LCOL.items():
            sel = (lab[m] == k)
            if sel.any():
                ss = np.repeat(s[m][:, None], r.shape[1], 1)[sel]
                hz = p[m, 2][:, None] + r[m] * d[m, 2][:, None]
                ax.scatter(ss, hz[sel], s=3, color=c,
                           label=f"echo: {LABEL_NAMES[k]}")
        zt = tm.top.interp(p[m, 0], p[m, 1]); zg = tm.ground.interp(p[m, 0], p[m, 1])
        if not over:
            ax.set_ylim(np.nanmin(zg) - 40, np.nanmax(zt) + 40)   # clutter echoes fall outside this window
        if over:
            ax.axhspan(atm.base, atm.base + float(atm.thickness.max()), color="#7f7f7f", alpha=0.1, label="cloud layer")
        t = (f"{ROUTE_LABELS[route]}{' — 50 % cloud' if over else ' — clear air (zoom 2 km)'}: "
             f"height of each echo hit (z + r·u_z)")
        ax.set_title(t); ax.set_ylabel("height MSL [m]"); ax.legend(loc="upper right", fontsize=7, ncol=3, markerscale=3)
    axes[-1].set_xlabel("distance along track [km]")
    fig.savefig(out / "m2_laser_profiles.png")
    (out / "report.json").write_text(json.dumps(rep, indent=1))
    shutil.copy(out / "m2_laser_profiles.png", project_path("docs/figures") / "m2_laser_profiles.png")
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()

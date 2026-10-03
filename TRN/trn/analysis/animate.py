"""Animated GIF of one run: particle clouds of the baseline and proposed MPF over the onboard map,
laser echoes along the track, and the error history.

    python -m trn.analysis.animate --route A_mountain_crossing --clouds 0.5 [--minutes 12] [--out docs/figures/x.gif]
"""
from __future__ import annotations

import argparse

import numpy as np
from matplotlib.animation import PillowWriter

from trn.analysis.style import COLORS, LABELS, ROUTE_LABELS, hillshade, plt, setup
from trn.common.config import apply_overrides, load_base_config, project_path
from trn.experiments.runner import make_filter, simulate_sensors
from trn.filters.base import Measurement
from trn.ins.error_model import build_error_model
from trn.laser.altimeter import LABEL_NAMES

LCOL = {1: "#1f77b4", 2: "#2ca02c", 3: "#7f7f7f", 4: "#bcbd22", 5: "#d62728", 6: "#17becf"}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", default="A_mountain_crossing")
    ap.add_argument("--clouds", type=float, default=0.5)
    ap.add_argument("--run", type=int, default=3)
    ap.add_argument("--minutes", type=float, default=12.0)
    ap.add_argument("--frame_s", type=float, default=6.0)
    ap.add_argument("--window_m", type=float, default=2500.0)
    ap.add_argument("--out", default="docs/figures/animation.gif")
    a = ap.parse_args()
    setup()
    over = {"atmosphere.enabled": a.clouds > 0, "atmosphere.cloud_fraction": a.clouds}
    cfg = apply_overrides(load_base_config(), over)
    rd, aux, ob = simulate_sensors(cfg, a.route, a.run, salt="animation")
    ins = aux["ins"]
    em = build_error_model(cfg, rd.traj.lat_ref, rd.traj.n_ref)
    names = ["mpf_baseline", "mpf_proposed"]
    flt = {n: make_filter(n, ob, cfg, em, aux["seed"]) for n in names}
    n_steps = min(rd.t.size, int(a.minutes * 60 * cfg["laser"]["pulse_rate_hz"]))
    every = int(a.frame_s * cfg["laser"]["pulse_rate_hz"])
    frames, err = [], {n: np.full(n_steps, np.nan) for n in names}
    for k in range(n_steps):
        m = Measurement(float(rd.t[k]), ins.pos[k], ins.vel[k], ins.C[k], ins.fn[k], rd.laser.ranges[k], float(rd.baro[k]))
        snap = {}
        for n, f in flt.items():
            e = f.initialize(m) if k == 0 else f.step(m)
            err[n][k] = np.hypot(*(e.pos[:2] - rd.truth_pos[k, :2]))
            if k % every == 0:
                w = f._w()
                sel = np.random.default_rng(k).choice(f.N, 400, p=w)
                snap[n] = (m.ins_pos[:2] - f.xi[sel, :2], e.pos[:2])
        if k % every == 0:
            frames.append((k, snap))
    g = ob.grid
    z = np.asarray(g.z)
    hs = hillshade(z, g.dx)
    lab, rng_ = rd.laser.labels[:, 0, :], rd.laser.ranges[:, 0, :]
    d0 = np.einsum("nij,j->ni", rd.traj.C[ins.idx], rd.laser.beams_body[0])
    fig = plt.figure(figsize=(11, 5.6))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.35], hspace=0.35, wspace=0.18)
    axm, axp, axe = fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])
    writer = PillowWriter(fps=6)
    out = project_path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with writer.saving(fig, str(out), dpi=80):
        for k, snap in frames:
            for ax in (axm, axp, axe):
                ax.cla()
            c = rd.truth_pos[k, :2]
            W = a.window_m
            r0, r1 = int((g.y0 - (c[1] + W)) / g.dx), int((g.y0 - (c[1] - W)) / g.dx)
            c0, c1 = int((c[0] - W - g.x0) / g.dx), int((c[0] + W - g.x0) / g.dx)
            axm.imshow(hs[max(r0, 0):r1, max(c0, 0):c1], cmap="gray",
                       extent=(g.x0 + c0 * g.dx, g.x0 + c1 * g.dx, g.y0 - r1 * g.dx, g.y0 - r0 * g.dx))
            for n in names:
                p, est = snap[n]
                axm.scatter(p[:, 0], p[:, 1], s=2, color=COLORS[n], alpha=0.35)
                axm.plot(*est, "o", ms=7, mfc="none", mec=COLORS[n], mew=2, label=LABELS[n])
            tr = rd.truth_pos[max(0, k - 600):k + 1]
            axm.plot(tr[:, 0], tr[:, 1], "k-", lw=1)
            axm.plot(*c, "k*", ms=12, label="truth")
            ip = ins.pos[k, :2]
            if abs(ip[0] - c[0]) < W and abs(ip[1] - c[1]) < W:
                axm.plot(*ip, "x", color=COLORS["ins"], ms=9, mew=2, label="INS only")
            axm.set_xlim(c[0] - W, c[0] + W); axm.set_ylim(c[1] - W, c[1] + W)
            axm.set_xticks([]); axm.set_yticks([])
            axm.set_title(f"{ROUTE_LABELS[a.route]}  t = {rd.t[k]/60:4.1f} min\nparticles on onboard map (±{W/1000:.1f} km)")
            axm.legend(loc="lower left", fontsize=6.5, framealpha=0.8, frameon=True)
            # echo profile (last 60 s)
            j0 = max(0, k - 600)
            ss = rd.t[j0:k + 1] - rd.t[k]
            pz = rd.truth_pos[j0:k + 1, 2]
            for L, col in LCOL.items():
                sel = lab[j0:k + 1] == L
                if sel.any():
                    hz = pz[:, None] + rng_[j0:k + 1] * d0[j0:k + 1, 2][:, None]
                    axp.scatter(np.repeat(ss[:, None], 3, 1)[sel], hz[sel], s=2, color=col, label=LABEL_NAMES[L])
            zt = ob.grid.interp(rd.truth_pos[j0:k + 1, 0], rd.truth_pos[j0:k + 1, 1])
            axp.plot(ss, zt, color="k", lw=0.8, label="onboard map under truth")
            axp.set_ylim(np.nanmin(zt) - 150, max(np.nanmax(zt) + 300, cfg["atmosphere"]["layer"]["base_msl_m"][a.route] + 500
                                                    if a.clouds > 0 else np.nanmax(zt) + 300))
            axp.set_xlim(-60, 0); axp.set_ylabel("echo height [m]"); axp.set_xlabel("seconds before now")
            axp.set_title(f"laser echoes (nadir beam), cloud fraction {a.clouds:.0%}")
            axp.legend(loc="upper left", fontsize=6, ncol=4, markerscale=3)
            for n in names:
                axe.plot(rd.t[:k + 1] / 60, err[n][:k + 1], color=COLORS[n], lw=1.3, label=LABELS[n])
            ie = np.hypot(*(ins.pos[:k + 1, :2] - rd.truth_pos[:k + 1, :2]).T)
            axe.plot(rd.t[:k + 1] / 60, ie, color=COLORS["ins"], lw=1, ls=":", label="INS only")
            axe.set_yscale("log"); axe.set_ylim(1, 3e4); axe.set_xlim(0, a.minutes)
            axe.set_xlabel("time [min]"); axe.set_ylabel("horizontal error [m]")
            axe.legend(loc="upper right", fontsize=6.5)
            writer.grab_frame()
    print(f"wrote {out} ({len(frames)} frames)")


if __name__ == "__main__":
    main()

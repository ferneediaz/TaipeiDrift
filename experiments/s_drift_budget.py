"""Height, scale and heading budget: which added sensor shrinks camera dead-reckoning drift most?

SIMULATED unless a number is quoted from a measured file. Four parts, one script, every input declared:

S. Stereo and other height sources vs height above ground (AGL): relative height error, which sets the
   camera-speed scale error (speed = image motion x AGL / focal) and the zoom search range of a map fix.
V. Video-derived altitude vs a barometer in an evaluation: how much an altitude taken from the video itself
   understates the drift compared with a barometer whose error model comes from the REAL Zurich log.
H. Sun-sensor heading after de-tilt with an imperfect AHRS, Taiwan sun geometry by month and hour, and
   (if data/processed/taiwan_sunshine/monthly_sunshine.csv exists) CWA sunshine fractions.
C. Combination Monte Carlo: along-track (scale) and cross-track (heading) drift of camera dead reckoning
   over a 4.6 km flight (ALTO length) with each added sensor; distance until the p95 error leaves a +-60 m
   map-fix search window; cost and power per option (prices are list prices, see the report).

Measured anchors used as inputs (not re-measured here):
  ALTO camera dead reckoning: 575 m along-track (13 % scale), 198 m cross-track (3 deg) after 4.6 km
  (docs/findings.md 3.4). Zurich barometer: white 0.30 m, random walk 0.112 m/sqrt(s), ramp sigma
  0.0024 m/s (data/processed/zurich_vertical/summary.json, experiments/s_zurich_vertical.py).

Run from the repository root:
    python experiments/s_drift_budget.py [--seeds 2000]
Outputs: data/processed/drift_budget/*.csv, *.png, summary.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from n_sensor_fusion import TAIWAN_SITES, UTC_OFFSET_TAIWAN, solar_az_el  # noqa: E402

D2R = np.pi / 180

# Zurich-calibrated barometer (MEASURED fit, one flight) and the first-draft generator for comparison.
BARO_ZURICH = dict(white=0.30, rw=0.112, ramp=0.0024)
BARO_OLD_DRAFT = dict(white=0.40, rw=0.01, ramp=0.002)

# Stereo rigs: baseline m, focal px (geometric, from vendor HFOV and per-eye width), price USD, mass g, power W.
# Vendor pages (Oct 2026): realsenseai.com D435/D455, docs.luxonis.com OAK-D Lite/Pro/LR, docs.stereolabs.com
# ZED 2i, orbbec.com Gemini 2. URLs in docs/research/sensor-fusion.md. OAK-D LR focal assumes the 82 deg lens.
RIGS = {
    "RealSense D435 (50 mm)": dict(B=0.050, f=686.0, price=375, mass=75, power=3.4),
    "Orbbec Gemini 2 (50 mm)": dict(B=0.050, f=629.0, price=234, mass=98, power=2.5),
    "OAK-D Lite (75 mm)": dict(B=0.075, f=433.0, price=269, mass=61, power=3.0),
    "OAK-D Pro (75 mm)": dict(B=0.075, f=763.0, price=429, mass=91, power=3.0),
    "RealSense D455 (95 mm)": dict(B=0.095, f=686.0, price=499, mass=116, power=3.46),
    "ZED 2i (120 mm)": dict(B=0.120, f=774.0, price=499, mass=229, power=1.9),
    "OAK-D LR (150 mm)": dict(B=0.150, f=736.0, price=779, mass=415, power=5.5),
    "custom 0.30 m (George 2023)": dict(B=0.30, f=1000.0, price=np.nan, mass=np.nan, power=np.nan),
    "custom 0.41 m (Song 2017)": dict(B=0.41, f=1000.0, price=np.nan, mass=np.nan, power=np.nan),
    "custom 1.0 m wing": dict(B=1.0, f=1000.0, price=np.nan, mass=np.nan, power=np.nan),
}
SIGMA_D_PX = 0.10  # per-frame disparity noise: RealSense tuning guide <0.1 px RMS on textured flat targets
OFFSET_D_PX = 0.05  # residual rectification/disparity offset per flight (assumption, thermal/vibration)
N_AVG = 10  # frames averaged per height estimate (10 Hz over 1 s, noise assumed independent: optimistic)
THRESH = (0.02, 0.05, 0.13)  # relative AGL error: VO-grade scale, one-zoom map fix, ALTO's measured 13 %


def baro_error(rng, n_seeds, t, model):
    """Relative barometer error since t=0, (seeds, len(t)); random walk + per-flight ramp + white."""
    dt = np.diff(t, prepend=t[0])
    rw = np.cumsum(model["rw"] * np.sqrt(dt) * rng.standard_normal((n_seeds, t.size)), axis=1)
    ramp = model["ramp"] * rng.standard_normal((n_seeds, 1)) * t
    white = model["white"] * rng.standard_normal((n_seeds, t.size))
    return rw + ramp + white - white[:, :1]


# ---------------------------------------------------------------------------------------------------
# Part S
# ---------------------------------------------------------------------------------------------------


def stereo_rel(h, B, f, sigma_d=SIGMA_D_PX, offset=OFFSET_D_PX, n=N_AVG):
    fb = f * B
    return np.sqrt((h * sigma_d / fb) ** 2 / n + (h * offset / fb) ** 2)


def part_s(out: Path, figures: bool):
    h = np.geomspace(5, 600, 400)
    rows = []
    for name, r in RIGS.items():
        rel = stereo_rel(h, r["B"], r["f"])
        row = dict(rig=name, baseline_m=r["B"], focal_px=round(r["f"], 1), price_usd=r["price"], mass_g=r["mass"],
                   power_w=r["power"], disparity_at_100m_px=round(r["f"] * r["B"] / 100, 3),
                   max_agl_disparity_ge_1px_m=round(r["f"] * r["B"], 1))
        for th in THRESH:
            ok = h[rel <= th]
            row[f"max_agl_rel_le_{int(th * 100)}pct_m"] = round(float(ok.max()), 1) if ok.size else 0.0
        rows.append(row)
    stereo = pd.DataFrame(rows)
    stereo.to_csv(out / "stereo_bands.csv", index=False)

    # Baro + DEM: AGL = baro altitude - DEM at the estimated position.
    def baro_dem_rel(h, t, slope, sigma_x, dem_sigma=2.0, init_sigma=2.0):
        b = BARO_ZURICH
        baro = np.sqrt(2 * b["white"] ** 2 / 10 + b["rw"] ** 2 * t + (b["ramp"] * t) ** 2)
        return np.sqrt(baro ** 2 + dem_sigma ** 2 + init_sigma ** 2 + (slope * sigma_x) ** 2) / h

    # Temporal (multi-view) stereo: baseline = V * dt known only as well as the speed is known.
    def temporal_rel(h, f, V, dt, sigma_flow, sigma_v_rel):
        return np.sqrt((h * sigma_flow / (f * V * dt)) ** 2 + sigma_v_rel ** 2)

    others = []
    for hh in (30, 60, 100, 150, 200, 300, 500):
        others.append(dict(
            agl_m=hh,
            baro_dem_flat_60s=baro_dem_rel(hh, 60, 0.0, 0.0), baro_dem_flat_600s=baro_dem_rel(hh, 600, 0.0, 0.0),
            baro_dem_hilly_600s_x100m=baro_dem_rel(hh, 600, 0.15, 100.0),
            temporal_stereo_imu_v_1pct=temporal_rel(hh, 1000, 15, 1.0, 0.2, 0.01),
            temporal_stereo_imu_v_8pct=temporal_rel(hh, 1000, 15, 1.0, 0.2, 0.08),
            **{f"stereo_{k.split(' (')[0]}": stereo_rel(hh, r["B"], r["f"]) for k, r in RIGS.items()},
        ))
    others = pd.DataFrame(others)
    others.to_csv(out / "height_source_rel_error.csv", index=False)

    if figures:
        plt = _plt()
        fig, ax = plt.subplots(figsize=(9, 5.5))
        for name, r in RIGS.items():
            ax.plot(h, 100 * stereo_rel(h, r["B"], r["f"]), lw=1, label=f"stereo {name}")
        for t_, ls in ((60, "-"), (600, "--")):
            ax.plot(h, 100 * baro_dem_rel(h, t_, 0, 0), "k", ls=ls, lw=2, label=f"baro (Zurich) + DEM, flat, {t_} s")
        ax.plot(h, 100 * baro_dem_rel(h, 600, 0.15, 100), "k:", lw=2, label="baro + DEM, 15 % slope, 100 m pos. error")
        for th in THRESH:
            ax.axhline(100 * th, color="grey", lw=0.6)
        ax.set_xscale("log"); ax.set_yscale("log"); ax.set_ylim(0.1, 300)
        ax.set_xlabel("height above ground [m]"); ax.set_ylabel("relative height error = scale error [%]")
        ax.set_title(f"SIMULATED: height sources (stereo {SIGMA_D_PX} px noise, {OFFSET_D_PX} px offset, {N_AVG} frames)")
        ax.grid(alpha=.3, which="both"); ax.legend(fontsize=6.5, ncol=2)
        fig.tight_layout(); fig.savefig(out / "height_sources_vs_agl.png", dpi=130); plt.close(fig)
    return stereo, others


# ---------------------------------------------------------------------------------------------------
# Part V
# ---------------------------------------------------------------------------------------------------


def part_v(out: Path, seeds: int, rng):
    """Along-track drift from the height used for camera speed. Flat terrain, AGL 120 m, 15 m/s, 600 s."""
    V, agl, T, dt = 15.0, 120.0, 600.0, 0.5
    t = np.arange(0, T + dt, dt)
    init = rng.standard_normal((seeds, 1))  # height error at the cut (GNSS vertical + DEM), shared draw
    sources = {
        "truth height (upper bound)": np.zeros((seeds, t.size)),
        "barometer, Zurich-calibrated model": baro_error(rng, seeds, t, BARO_ZURICH) + 3.0 * init,
        "barometer, Zurich model, perfect height at the cut": baro_error(rng, seeds, t, BARO_ZURICH),
        "barometer, first-draft model": baro_error(rng, seeds, t, BARO_OLD_DRAFT) + 3.0 * init,
        # Video-derived altitude: e.g. SfM/visual odometry over the same frames, bundle-adjusted with the
        # GNSS part; no drift, small scale error, smooth. It is NOT a barometer.
        "video-derived altitude (SfM-like, 1 % scale, no drift)": 0.01 * agl * rng.standard_normal((seeds, 1))
        + 0.2 * rng.standard_normal((seeds, t.size)),
    }
    rows = []
    for name, e in sources.items():
        along = np.cumsum(V * e / agl * dt, axis=1)
        for tt in (60, 300, 600):
            k = int(tt / dt)
            a = np.abs(along[:, k])
            rows.append(dict(height_source=name, t_s=tt, distance_m=V * tt, along_p50_m=float(np.median(a)),
                             along_p95_m=float(np.quantile(a, .95))))
    tab = pd.DataFrame(rows)
    tab.to_csv(out / "video_vs_baro_eval_bias.csv", index=False)
    return tab


# ---------------------------------------------------------------------------------------------------
# Part H
# ---------------------------------------------------------------------------------------------------


def sun_heading_sigma(el_deg, sensor_deg, tilt_deg):
    el = np.asarray(el_deg) * D2R
    return np.sqrt((sensor_deg / np.cos(el)) ** 2 + (tilt_deg * np.tan(el)) ** 2)


def part_h(out: Path, figures: bool):
    els = np.array([10, 20, 30, 40, 50, 60, 70, 75, 80, 85])
    grid = []
    for s in (0.1, 0.5, 1.0):
        for tl in (0.25, 0.5, 1.0, 2.0):
            grid.append(dict(sensor_sigma_deg=s, ahrs_tilt_sigma_deg=tl,
                             **{f"el_{e}": round(float(sun_heading_sigma(e, s, tl)), 2) for e in els}))
    grid = pd.DataFrame(grid)
    grid.to_csv(out / "sun_heading_sigma_table.csv", index=False)

    # Taiwan: share of daytime (07:00-17:00 local) when the sun is usable by an up-looking sensor.
    sun_path = Path("data/processed/taiwan_sunshine/monthly_sunshine.csv")
    sunshine = pd.read_csv(sun_path) if sun_path.exists() else None
    rows = []
    year = 2026
    for site, (lat, lon) in TAIWAN_SITES.items():
        for month in range(1, 13):
            days = pd.date_range(f"{year}-{month:02d}-01", periods=pd.Timestamp(f"{year}-{month:02d}-01").days_in_month)
            doy = days.dayofyear.to_numpy()
            hours_local = np.arange(7, 17, 1 / 12)
            _, el = solar_az_el(lat, lon, year, doy[:, None], hours_local[None, :] - UTC_OFFSET_TAIWAN)
            el = np.degrees(el)
            row = dict(site=site, month=month, noon_elev_max_deg=float(el.max()))
            for fov_half, label in ((60, "fov60"), (85, "fov85")):
                for sensor, tilt, slabel in ((0.5, 0.5, "good"), (1.0, 2.0, "field")):
                    sig = sun_heading_sigma(el, sensor, tilt)
                    usable = (el >= 90 - fov_half) & (el > 5) & (sig <= 2.0)
                    row[f"usable_{label}_{slabel}_sigma_le_2deg"] = float(usable.mean())
            if sunshine is not None:
                key = {"Taipei": "Taipei", "Wufeng_Taichung": "Taichung", "Kaohsiung": "Kaohsiung"}[site]
                m = sunshine[(sunshine.station.str.contains(key, case=False)) & (sunshine.month == month)]
                row["cwa_sunshine_fraction"] = float(m.sunshine_fraction.iloc[0]) if len(m) else np.nan
            rows.append(row)
    geo = pd.DataFrame(rows)
    if "cwa_sunshine_fraction" in geo:
        for c in [c for c in geo.columns if c.startswith("usable_")]:
            geo[c.replace("usable_", "expected_")] = geo[c] * geo.cwa_sunshine_fraction
    geo.to_csv(out / "sun_usability_taiwan.csv", index=False)

    if figures:
        plt = _plt()
        fig, ax = plt.subplots(figsize=(8, 4.5))
        e = np.linspace(5, 88, 200)
        for s, tl in ((0.1, 0.25), (0.5, 0.5), (1.0, 1.0), (1.0, 2.0)):
            ax.plot(e, sun_heading_sigma(e, s, tl), label=f"sensor {s} deg, AHRS tilt {tl} deg")
        ax.axhline(3, color="r", ls="--", lw=.8, label="ALTO camera heading error, 3 deg (MEASURED)")
        ax.set_yscale("log"); ax.set_xlabel("sun elevation [deg]"); ax.set_ylabel("heading sigma [deg]")
        ax.grid(alpha=.3, which="both"); ax.legend(fontsize=7)
        ax.set_title("Sun-sensor heading after de-tilt: sqrt((s/cos el)^2 + (tilt tan el)^2)")
        fig.tight_layout(); fig.savefig(out / "sun_heading_sigma.png", dpi=130); plt.close(fig)
    return grid, geo


# ---------------------------------------------------------------------------------------------------
# Part C
# ---------------------------------------------------------------------------------------------------

TERRAIN = dict(amp=np.array([20.0, 8.0, 3.0]), lam=np.array([2000.0, 600.0, 200.0]))


def terrain_grid(s, phase):
    """Terrain elevation at shared positions s (n,) for every seed: (seeds, n)."""
    return np.sin(s[None, :, None] * (2 * np.pi / TERRAIN["lam"]) + phase[:, None, :]) @ TERRAIN["amp"]


def terrain_at(x, phase):
    """Terrain elevation at one position per seed, x (seeds,): (seeds,)."""
    return np.sin(x[:, None] * (2 * np.pi / TERRAIN["lam"]) + phase) @ TERRAIN["amp"]


SCALE_OPTIONS = ("stale_scale", "baro_only", "baro_dem", "stereo_oakd_lr", "stereo_custom_1m")
HEADING_OPTIONS = ("camera_3deg", "gyro_calibrated", "sun_field", "sun_good_ahrs", "sun_good_ahrs_40pct_gyro")
COSTS = {  # added hardware USD, added power W; list prices / INFERENCE, see report
    "stale_scale": (0, 0.0), "baro_only": (0, 0.0), "baro_dem": (0, 0.0),
    "stereo_oakd_lr": (779, 5.5), "stereo_custom_1m": (400, 4.0),
    "camera_3deg": (0, 0.0), "gyro_calibrated": (0, 0.0), "sun_field": (150, 0.01), "sun_good_ahrs": (1650, 1.0),
    "sun_good_ahrs_40pct_gyro": (1650, 1.0),  # open Foresail-type sensor ~150 USD + good AHRS ~1500 USD (INFERENCE)
}


def part_c(out: Path, seeds: int, rng, figures: bool, V=15.0, agl0=120.0, D=4600.0, sun_el=50.0, dem_sigma=2.0):
    dt = 0.5
    t = np.arange(0, D / V + dt, dt)
    s = V * t
    n = t.size
    phase = rng.uniform(0, 2 * np.pi, (seeds, TERRAIN["amp"].size))
    h_ter = terrain_grid(s, phase)  # (seeds, n)
    alt = h_ter[:, :1] + agl0  # constant-altitude cruise
    agl = alt - h_ter
    baro = baro_error(rng, seeds, t, BARO_ZURICH)
    init = 3.0 * rng.standard_normal((seeds, 1))  # AGL error at the cut (GNSS vertical + DEM)
    dem_bias = dem_sigma * rng.standard_normal((seeds, 1))
    dem_phase = rng.uniform(0, 2 * np.pi, (seeds, 2))

    def dem_err(x):
        return dem_bias + dem_sigma / 2.0 * (1.5 * np.sin(2 * np.pi * x / 300 + dem_phase[:, :1])
                                             + 1.0 * np.sin(2 * np.pi * x / 800 + dem_phase[:, 1:]))

    oak, cus = RIGS["OAK-D LR (150 mm)"], RIGS["custom 1.0 m wing"]
    st_off = OFFSET_D_PX * rng.standard_normal((seeds, 1))
    st_noise = SIGMA_D_PX / np.sqrt(N_AVG) * rng.standard_normal((seeds, n))

    def stereo_agl(r):
        fb = r["f"] * r["B"]
        d = fb / agl + st_off + st_noise
        return np.where(d > 0.05, fb / np.maximum(d, 0.05), 1e4)

    # heading errors (rad)
    psi0 = 1.0 * D2R * rng.standard_normal((seeds, 1))
    head = {
        "camera_3deg": 3.0 * D2R * rng.standard_normal((seeds, 1)) * np.ones((1, n)),
        "gyro_calibrated": psi0 + 0.005 * D2R * rng.standard_normal((seeds, 1)) * t,
        "sun_field": float(sun_heading_sigma(sun_el, 1.0, 2.0)) * D2R * (
            0.7 * rng.standard_normal((seeds, 1)) + 0.3 * rng.standard_normal((seeds, n))),
        "sun_good_ahrs": float(sun_heading_sigma(sun_el, 0.5, 0.5)) * D2R * (
            0.7 * rng.standard_normal((seeds, 1)) + 0.3 * rng.standard_normal((seeds, n))),
    }
    # Sun visible 40 % of the time (CWA Taichung October sunshine 0.58 x usable geometry 0.69), clear and
    # blocked spells of mean 60 s / 90 s; the gyro carries the heading from the last sun reading in between.
    vis = np.empty((seeds, n), bool)
    state = rng.random(seeds) < 0.4
    u = rng.random((seeds, n))
    for k in range(n):
        flip = u[:, k] < np.where(state, dt / 60.0, dt / 90.0)
        state = np.where(flip, ~state, state)
        vis[:, k] = state
    bg = 0.005 * D2R * rng.standard_normal((seeds, 1))
    sun_e = head["sun_good_ahrs"]
    gap = np.zeros((seeds, n))
    last = psi0[:, 0].copy()
    for k in range(n):
        last = np.where(vis[:, k], sun_e[:, k], last + (bg[:, 0] * dt if k else 0.0))
        gap[:, k] = last
    head["sun_good_ahrs_40pct_gyro"] = gap
    # scale: along-track error rate = (AGL_est - AGL) / AGL; integrate with DEM lookups at the estimate
    along = {}
    for opt in SCALE_OPTIONS:
        ds = np.zeros((seeds, n))
        for k in range(1, n):
            if opt == "stale_scale":
                agl_est = agl[:, 0] + init[:, 0]
            elif opt == "baro_only":
                agl_est = agl[:, 0] + init[:, 0] + baro[:, k]  # altitude change only; terrain change unseen
            elif opt == "baro_dem":
                x_hat = s[k] + ds[:, k - 1]
                h_dem = terrain_at(x_hat, phase)
                agl_est = alt[:, 0] + baro[:, k] + init[:, 0] - (h_dem + dem_err(x_hat[:, None])[:, 0])
            elif opt == "stereo_oakd_lr":
                agl_est = stereo_agl(oak)[:, k]
            else:
                agl_est = stereo_agl(cus)[:, k]
            eps = np.clip((agl_est - agl[:, k]) / agl[:, k], -1, 1)
            ds[:, k] = ds[:, k - 1] + V * dt * eps
        along[opt] = ds
    cross = {opt: np.cumsum(V * dt * np.sin(e), axis=1) for opt, e in head.items()}

    rows = []
    for so in SCALE_OPTIONS:
        for ho in HEADING_OPTIONS:
            err = np.hypot(along[so], cross[ho])
            p95 = np.quantile(err, .95, axis=0)
            leave = s[np.argmax(p95 > 60)] if (p95 > 60).any() else np.inf
            c = COSTS[so][0] + COSTS[ho][0]
            w = COSTS[so][1] + COSTS[ho][1]
            rows.append(dict(scale=so, heading=ho, final_p50_m=float(np.median(err[:, -1])),
                             final_p95_m=float(np.quantile(err[:, -1], .95)),
                             along_p50_m=float(np.median(np.abs(along[so][:, -1]))),
                             cross_p50_m=float(np.median(np.abs(cross[ho][:, -1]))),
                             distance_until_p95_gt_60m=float(leave), added_cost_usd=c, added_power_w=w))
    tab = pd.DataFrame(rows).sort_values("final_p50_m")
    tab.to_csv(out / "combination_drift.csv", index=False)
    if figures:
        plt = _plt()
        fig, ax = plt.subplots(figsize=(9, 5))
        for so in SCALE_OPTIONS:
            ax.plot(s / 1000, np.median(np.abs(along[so]), axis=0), label=f"along: {so}")
        for ho in HEADING_OPTIONS:
            ax.plot(s / 1000, np.median(np.abs(cross[ho]), axis=0), "--", label=f"cross: {ho}")
        ax.set_yscale("log"); ax.set_xlabel("distance since cut [km]"); ax.set_ylabel("median |error| [m]")
        ax.axhline(60, color="k", lw=.6); ax.grid(alpha=.3, which="both"); ax.legend(fontsize=7, ncol=2)
        ax.set_title(f"SIMULATED: camera dead-reckoning drift, AGL {agl0:.0f} m over rolling terrain, {V:.0f} m/s")
        fig.tight_layout(); fig.savefig(out / "combination_drift.png", dpi=130); plt.close(fig)
    terrain_stats = dict(agl_min=float(agl.min()), agl_max=float(agl.max()),
                         stale_scale_rel_err_p50=float(np.median(np.abs((agl[:, :1] - agl) / agl)[:, -1])))
    return tab, terrain_stats


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seeds", type=int, default=2000)
    p.add_argument("--seed0", type=int, default=0)
    p.add_argument("--out", type=Path, default=Path("data/processed/drift_budget"))
    p.add_argument("--no-figures", action="store_true")
    p.add_argument("--terrain-scale", type=float, default=1.0, help="multiplies the terrain relief in part C")
    p.add_argument("--dem-sigma", type=float, default=2.0, help="DEM bias sigma in part C, m (GLO-30 rel. < 2 m LE90)")
    args = p.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed0)
    fig = not args.no_figures
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", 40)

    stereo, others = part_s(args.out, fig)
    print("S. stereo bands (max AGL in m for a relative height error threshold)\n", stereo.to_string(index=False))
    print(others.round(3).to_string(index=False))
    vtab = part_v(args.out, args.seeds, rng)
    print("\nV. along-track drift from the height source\n", vtab.round(1).to_string(index=False))
    grid, geo = part_h(args.out, fig)
    print("\nH. sun heading sigma (deg)\n", grid.to_string(index=False))
    print(geo.round(2).to_string(index=False))
    TERRAIN["amp"] = TERRAIN["amp"] * args.terrain_scale
    ctab, tstats = part_c(args.out, min(args.seeds, 1000), rng, fig, dem_sigma=args.dem_sigma)
    print("\nC. combinations\n", ctab.round(1).to_string(index=False), "\n", tstats)
    summary = dict(kind="SIMULATED with measured anchors (see docstring)", args={k: str(v) for k, v in vars(args).items()},
                   assumptions=dict(sigma_d_px=SIGMA_D_PX, offset_d_px=OFFSET_D_PX, n_avg=N_AVG, baro=BARO_ZURICH,
                                    rigs={k: {kk: (None if isinstance(vv, float) and np.isnan(vv) else vv)
                                              for kk, vv in r.items()} for k, r in RIGS.items()}, costs=COSTS),
                   stereo_bands=stereo.to_dict("records"), video_vs_baro=vtab.to_dict("records"),
                   combination=ctab.to_dict("records"), terrain=tstats)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2, default=float) + "\n")


if __name__ == "__main__":
    main()

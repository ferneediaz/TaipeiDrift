"""Real barometer error model and real-data vertical replay (Zurich Urban MAV sample, one flight).

MEASURED, real sensors, one flight: Fotokite (tethered, walked through Zurich, ground speed median 0.8 m/s),
Pixhawk barometer 10 Hz, raw accelerometer 10 Hz (instantaneous samples, not pre-integrated), PX4 attitude
quaternion 50 Hz, GNSS ~5 Hz. Reference: Pix4D photogrammetric camera positions (1 Hz), used ONLY to score.

Part A, barometer error model: e(t) = baro altitude - photogrammetric altitude. Its structure function
D(tau) = |e(t + tau) - e(t)| is the relative-altitude error after a cut of length tau. A white + random walk
+ per-flight ramp model is fitted to the RMS structure function, and the synthetic generator of
experiments/n_sensor_fusion.py (nominal and strong regimes) is compared at the same horizons.

Part B, vertical replay: GNSS altitude until a cut, then baro (+ raw IMU) only. Cuts every --start-step s.
Variants: baro_only (offset frozen at the cut), imu_baro (4-state EKF), imu_only (EKF without baro after the
cut, shows why the IMU cannot carry height alone), baro_lowpass (causal 1 s smoothing, the no-IMU control
for noise reduction). Scored on altitude change since the cut and on detrended (high-frequency) error.

Leakage notes: the quaternion comes from the PX4 attitude filter, which may use GNSS for acceleration
compensation; only tilt enters the vertical channel, so the effect is second order (stated, not removed).
The accelerometer noise level is estimated from the first 60 s of the log (before every cut).

Run from the repository root:
    python experiments/s_zurich_vertical.py
Outputs: data/processed/zurich_vertical/{summary.json, baro_error_model.csv, replay_by_horizon.csv, *.png}
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p_zurich_baro import load  # noqa: E402  (same loader and truth timing as the baseline script)

G = 9.80665
HORIZONS_S = (1, 3, 10, 30, 60, 120, 300, 600, 1200)
REPLAY_HORIZONS_S = (10, 30, 60, 120, 300, 600, 1200)
YAW180 = np.diag([-1.0, -1.0, 1.0])  # raw accel axes -> body: smallest horizontal mean of the rotated gravity


def quat_to_R(q):
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def load_imu(log_dir: Path, t_grid_us):
    acc = pd.read_csv(log_dir / "RawAccel.csv", skipinitialspace=True).iloc[:, :5]
    acc.columns = ["t_us", "err", "x", "y", "z"]
    pose = pd.read_csv(log_dir / "OnboardPose.csv", skipinitialspace=True).iloc[:, [0, 14, 15, 16, 17]]
    pose.columns = ["t_us", "qw", "qx", "qy", "qz"]
    q = np.stack([np.interp(acc.t_us, pose.t_us, pose[c]) for c in ("qw", "qx", "qy", "qz")], 1)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    f_ned = np.einsum("nij,nj->ni", quat_to_R(q), acc[["x", "y", "z"]].to_numpy() @ YAW180.T)
    up_specific = -f_ned[:, 2]  # specific force along up; minus g is the vertical acceleration (+ bias)
    return np.interp(t_grid_us, acc.t_us, up_specific - G)


def gnss_fixes(log_dir: Path):
    g = pd.read_csv(log_dir / "OnboardGPS.csv", skipinitialspace=True).iloc[:, :5]
    g.columns = ["t_us", "imgid", "lat", "lon", "alt_m"]
    new = g[["lat", "lon", "alt_m"]].diff().abs().sum(axis=1) > 0
    new.iloc[0] = True
    return g[new].t_us.to_numpy() / 1e6, g[new].alt_m.to_numpy()


# ---------------------------------------------------------------------------------------------------
# Part A
# ---------------------------------------------------------------------------------------------------


def structure_function(t, e, horizons, step=1.0):
    """All pairs (t_i, t_i + tau) on the 1 Hz truth grid; returns per-horizon stats of e(t+tau) - e(t)."""
    rows = []
    for tau in horizons:
        j = np.searchsorted(t, t + tau)
        ok = (j < len(t))
        j = np.minimum(j, len(t) - 1)
        ok &= np.abs(t[j] - t - tau) <= 0.6 * step + 0.5
        de = e[j[ok]] - e[ok]
        a = np.abs(de)
        rows.append(dict(horizon_s=tau, n_pairs=int(ok.sum()), rms_m=float(np.sqrt(np.mean(de ** 2))),
                         median_abs_m=float(np.median(a)), p95_abs_m=float(np.quantile(a, .95))))
    return pd.DataFrame(rows)


def fit_model(sf: pd.DataFrame):
    """RMS(tau)^2 = 2 s_w^2 + q tau + (d tau)^2, non-negative least squares on the RMS^2 values."""
    from scipy.optimize import nnls

    tau = sf.horizon_s.to_numpy(float)
    A = np.stack([2 * np.ones_like(tau), tau, tau * tau], 1)
    y = sf.rms_m.to_numpy() ** 2
    w = 1 / y  # relative error weighting: every horizon counts
    coef, _ = nnls(A * w[:, None], y * w)
    sw2, q, d2 = coef
    return dict(white_sigma_m=float(np.sqrt(sw2)), rw_m_per_sqrt_s=float(np.sqrt(q)), ramp_m_per_s=float(np.sqrt(d2)),
                fitted_rms_m=dict(zip(map(int, tau), np.sqrt(A @ coef).round(3).tolist())))


def generator_structure(horizons, noise, rw, drift_draw, seeds=4000, rng=None):
    """Relative error |e(t0 + tau) - e(t0)| for the n_sensor_fusion generator baro model (no truth noise)."""
    rng = rng or np.random.default_rng(0)
    out = []
    for tau in horizons:
        d = drift_draw(rng, seeds)
        de = d * tau + rw * np.sqrt(tau) * rng.standard_normal(seeds) + noise * np.sqrt(2) * rng.standard_normal(seeds)
        a = np.abs(de)
        out.append(dict(horizon_s=tau, median_abs_m=float(np.median(a)), p95_abs_m=float(np.quantile(a, .95)),
                        rms_m=float(np.sqrt(np.mean(de ** 2)))))
    return pd.DataFrame(out)


# ---------------------------------------------------------------------------------------------------
# Part B
# ---------------------------------------------------------------------------------------------------


def vertical_ekf(t, baro, accel, gnss_t, gnss_z, k_cut, use_imu, baro_after_cut, cfg):
    """Batched over cuts: element i uses GNSS only before k_cut[i]; baro after the cut only if baro_after_cut.

    State [z, vz, b_baro, b_acc]; z in the GNSS datum. Returns z estimates (B, N).
    """
    B, N = len(k_cut), len(t)
    x = np.zeros((B, 4))
    x[:, 0] = baro[0]
    P = np.zeros((B, 4, 4))
    P[:, 0, 0] = P[:, 2, 2] = 1e4
    P[:, 1, 1] = 4.0
    P[:, 3, 3] = cfg["acc_bias_sigma0"] ** 2
    gi = np.searchsorted(gnss_t, t, side="right") - 1  # latest fix at or before t_k
    last_g = -1
    out = np.empty((B, N))
    Hb = np.array([1.0, 0, 1.0, 0])
    Hg = np.array([1.0, 0, 0, 0])
    for k in range(N):
        if k > 0:
            dt = t[k] - t[k - 1]
            F = np.eye(4)
            F[0, 1] = dt
            g2 = np.array([0.5 * dt * dt, dt])
            sa = cfg["acc_sigma"] if use_imu else cfg["manoeuvre_sigma"]
            Q = np.zeros((4, 4))
            Q[:2, :2] = sa ** 2 * np.outer(g2, g2)
            Q[2, 2] = cfg["baro_bias_rw"] ** 2 * dt
            if use_imu:
                F[0, 3], F[1, 3] = -0.5 * dt * dt, -dt
                Q[3, 3] = cfg["acc_bias_rw"] ** 2 * dt
            x = x @ F.T
            if use_imu:
                x[:, 0] += g2[0] * accel[k - 1]
                x[:, 1] += g2[1] * accel[k - 1]
            P = F @ P @ F.T + Q
        pre = k < k_cut
        if gi[k] > last_g and gi[k] >= 0:
            last_g = gi[k]
            _upd(x, P, np.where(pre, gnss_z[gi[k]], np.nan), Hg, cfg["gnss_sigma"] ** 2)
        y = baro[k] if baro_after_cut else np.where(pre, baro[k], np.nan)
        _upd(x, P, np.broadcast_to(y, (B,)).astype(float), Hb, cfg["baro_sigma"] ** 2)
        out[:, k] = x[:, 0]
    return out


def _upd(x, P, y, H, R):
    ok = np.isfinite(y)
    if not ok.any():
        return
    nu = np.where(ok, y - x @ H, 0.0)
    PH = P @ H
    S = PH @ H + R
    K = PH / S[:, None]
    w = ok.astype(float)
    x += K * (nu * w)[:, None]
    P -= w[:, None, None] * (K[:, :, None] * PH[:, None, :])


def causal_lowpass(y, dt, tau):
    a = dt / (tau + dt)
    out = np.empty_like(y)
    out[0] = y[0]
    for k in range(1, len(y)):
        out[k] = out[k - 1] + a * (y[k] - out[k - 1])
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--logs", type=Path, default=Path("data/raw/zurich_mav/AGZ_subset/Log Files"))
    p.add_argument("--output", type=Path, default=Path("data/processed/zurich_vertical"))
    p.add_argument("--start-step", type=float, default=30.0)
    p.add_argument("--no-figures", action="store_true")
    args = p.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    baro, gps, truth = load(args.logs)
    t_tr = truth.t_s.to_numpy()
    z_tr = truth.z_gt.to_numpy()
    z_baro_tr = np.interp(t_tr, baro.t_s, baro.altitude_m)
    # Scale check: regress (sensor - reference) on altitude and time. A non-zero altitude slope is a scale error.
    A = np.stack([z_tr - z_tr.mean(), np.ones_like(z_tr), t_tr - t_tr[0]], 1)
    z_gnss_tr = np.interp(t_tr, gps.t_s, gps.alt_m)
    scale_fit = {}
    for name, zs in (("baro", z_baro_tr), ("gnss", z_gnss_tr)):
        c, *_ = np.linalg.lstsq(A, zs - z_tr, rcond=None)
        scale_fit[name] = dict(scale_error=float(c[0]), ramp_m_per_s=float(c[2]),
                               residual_rms_m=float(np.std(zs - z_tr - A @ c)))


    # ---- Part A
    e = z_baro_tr - z_tr
    e = e - e[0]
    sf = structure_function(t_tr, e, HORIZONS_S)
    model = fit_model(sf)
    # Same structure function after removing the fitted scale error (what a pre-cut scale calibration could give).
    e_sc = e - scale_fit["baro"]["scale_error"] * (z_tr - z_tr[0])
    sf_scale_removed = structure_function(t_tr, e_sc, HORIZONS_S)
    # Reference noise proxy: second difference of the photogrammetric altitude at 1 Hz (smooth motion assumed).
    d2 = np.diff(z_tr, 2)
    truth_noise_proxy = float(1.4826 * np.median(np.abs(d2 - np.median(d2))) / np.sqrt(6))
    d2b = np.diff(np.interp(t_tr, baro.t_s, baro.altitude_m), 2)
    baro_noise_proxy_1hz = float(1.4826 * np.median(np.abs(d2b - np.median(d2b))) / np.sqrt(6))
    bz = baro.altitude_m.to_numpy()
    d2r = np.diff(bz, 2)
    baro_noise_10hz = float(1.4826 * np.median(np.abs(d2r - np.median(d2r))) / np.sqrt(6))
    # Speed dependence: baro error vs GNSS ground speed (dynamic pressure proxy) at 1 Hz.
    gv = pd.read_csv(args.logs / "OnboardGPS.csv", skipinitialspace=True).iloc[:, [0, 10, 11]]
    gv.columns = ["t_us", "vn", "ve"]
    speed = np.interp(t_tr, gv.t_us / 1e6, np.hypot(gv.vn, gv.ve))
    e_hp = e - pd.Series(e).rolling(121, center=True, min_periods=30).median().to_numpy()
    okc = np.isfinite(e_hp)
    speed_corr = float(np.corrcoef(e_hp[okc], speed[okc] ** 2)[0, 1])
    temp = np.interp(t_tr, baro.t_s, baro.temperature_c)

    gens = {
        "n_sensor_fusion nominal": generator_structure(HORIZONS_S, 0.4, 0.01, lambda r, n: 0.002 * r.standard_normal(n)),
        "n_sensor_fusion strong": generator_structure(
            HORIZONS_S, 0.4, 0.03, lambda r, n: np.where(r.random(n) < .5, 1, -1) * r.uniform(.015, .025, n)),
        "calibrated on Zurich": generator_structure(
            HORIZONS_S, model["white_sigma_m"], model["rw_m_per_sqrt_s"],
            lambda r, n: model["ramp_m_per_s"] * r.standard_normal(n)),
    }
    rows = [dict(source="Zurich measured (vs photogrammetry)", **r) for r in sf.to_dict("records")]
    for name, gdf in gens.items():
        rows += [dict(source=f"SIMULATED {name}", **r) for r in gdf.to_dict("records")]
    pd.DataFrame(rows).to_csv(args.output / "baro_error_model.csv", index=False)

    # ---- Part B
    t = baro.t_s.to_numpy()
    tg = (t * 1e6).astype(np.int64)
    accel = load_imu(args.logs, tg)
    gnss_t, gnss_z = gnss_fixes(args.logs)
    b0 = t < t[0] + 60
    acc_hp = accel[b0] - pd.Series(accel[b0]).rolling(21, center=True, min_periods=5).mean().to_numpy()
    acc_sigma = float(np.nanstd(acc_hp))
    cfg = dict(baro_sigma=0.5, baro_bias_rw=0.03, gnss_sigma=2.5, acc_sigma=acc_sigma, acc_bias_rw=0.002,
               acc_bias_sigma0=1.0, manoeuvre_sigma=0.5)
    cuts = np.arange(t_tr[0] + 120, t_tr[-1], args.start_step)
    k_cut = np.searchsorted(t, cuts)
    z_ib = vertical_ekf(t, bz, accel, gnss_t, gnss_z, k_cut, True, True, cfg)
    z_io = vertical_ekf(t, bz, accel, gnss_t, gnss_z, k_cut, True, False, cfg)
    z_b_lp = causal_lowpass(bz, 0.1, 1.0)
    variants = {
        "baro_only": np.broadcast_to(bz, z_ib.shape),
        "baro_lowpass_1s": np.broadcast_to(z_b_lp, z_ib.shape),
        "imu_baro_ekf": z_ib,
        "imu_only_after_cut": z_io,
    }
    rep = []
    hf = {name: [] for name in variants}
    for i, tc in enumerate(cuts):
        i0 = int(np.searchsorted(t_tr, tc))
        if i0 >= len(t_tr):
            continue
        for name, Z in variants.items():
            ze = np.interp(t_tr, t, Z[i])
            for h in REPLAY_HORIZONS_S:
                i1 = int(np.searchsorted(t_tr, t_tr[i0] + h))
                if i1 >= len(t_tr) or abs(t_tr[i1] - t_tr[i0] - h) > 2:
                    continue
                err = (ze[i1] - ze[i0]) - (z_tr[i1] - z_tr[i0])
                rep.append(dict(cut_s=t_tr[i0], horizon_s=h, variant=name, error_m=err))
            # high-frequency error inside the first 60 s after the cut, mean removed
            j = (t_tr >= t_tr[i0]) & (t_tr < t_tr[i0] + 60)
            if j.sum() > 30:
                r = (ze[j] - z_tr[j])
                hf[name].append(float(np.std(r - np.mean(r))))
    rep = pd.DataFrame(rep)
    by = rep.groupby(["variant", "horizon_s"]).error_m.agg(
        n="size", median_abs=lambda v: np.median(np.abs(v)), p95_abs=lambda v: np.quantile(np.abs(v), .95)).reset_index()
    by.to_csv(args.output / "replay_by_horizon.csv", index=False)
    rep.to_csv(args.output / "replay_errors.csv", index=False)
    hf_tab = {k: dict(median_m=float(np.median(v)), p95_m=float(np.quantile(v, .95)), n=len(v)) for k, v in hf.items()}

    summary = dict(
        kind="MEASURED: real sensors, one tethered flight, photogrammetric reference; overlapping windows",
        duration_s=float(t_tr[-1] - t_tr[0]), truth_alt_range_m=[float(z_tr.min()), float(z_tr.max())],
        ground_speed_median_mps=float(np.median(speed)), temperature_range_c=[float(temp.min()), float(temp.max())],
        part_a=dict(structure_function=sf.round(3).to_dict("records"), fitted_model=model,
                    truth_noise_proxy_m=truth_noise_proxy, baro_noise_proxy_1hz_m=baro_noise_proxy_1hz,
                    baro_white_noise_10hz_m=baro_noise_10hz,
                    corr_highpass_baro_error_vs_speed_squared=speed_corr,
                    corr_baro_error_vs_temperature=float(np.corrcoef(e, temp)[0, 1]),
                    scale_fit_vs_photogrammetry=scale_fit,
                    structure_function_scale_removed=sf_scale_removed.round(3).to_dict("records"),
                    generators={k: v.round(3).to_dict("records") for k, v in gens.items()}),
        part_b=dict(config=cfg, n_cuts=int(len(cuts)), by_horizon=by.round(3).to_dict("records"),
                    highfreq_error_first_60s=hf_tab),
    )
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    if not args.no_figures:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5))
        a1.plot(sf.horizon_s, sf.median_abs_m, "ko-", label="Zurich MEASURED median")
        a1.plot(sf.horizon_s, sf.p95_abs_m, "ko--", label="Zurich MEASURED p95")
        for (name, gdf), c in zip(gens.items(), ("C0", "C3", "C2")):
            a1.plot(gdf.horizon_s, gdf.median_abs_m, "-", color=c, label=f"SIM {name} median")
            a1.plot(gdf.horizon_s, gdf.p95_abs_m, "--", color=c, label=f"SIM {name} p95")
        a1.set_xscale("log"); a1.set_yscale("log"); a1.grid(alpha=.3)
        a1.set_xlabel("seconds since cut"); a1.set_ylabel("|baro relative altitude error| [m]")
        a1.legend(fontsize=7); a1.set_title("Barometer: real vs synthetic models")
        for name in variants:
            g = by[by.variant == name]
            a2.plot(g.horizon_s, g.median_abs, "o-", label=f"{name} median")
        a2.set_xscale("log"); a2.set_yscale("log"); a2.grid(alpha=.3); a2.legend(fontsize=7)
        a2.set_xlabel("seconds since cut"); a2.set_ylabel("|altitude change error| [m]")
        a2.set_title("Zurich replay (MEASURED, one flight)")
        fig.tight_layout(); fig.savefig(args.output / "zurich_vertical.png", dpi=130); plt.close(fig)

    print(json.dumps(dict(model=model, truth_noise_proxy_m=round(truth_noise_proxy, 3),
                          baro_noise_10hz=round(baro_noise_10hz, 3), baro_noise_1hz=round(baro_noise_proxy_1hz, 3),
                          speed_corr=round(speed_corr, 3), acc_sigma=round(acc_sigma, 3)), indent=1))
    print(sf.round(2).to_string(index=False))
    print(pd.DataFrame(rows)[lambda d: d.horizon_s.isin([10, 60, 300, 600, 1200])].pivot_table(
        index="horizon_s", columns="source", values="p95_abs_m").round(2).to_string())
    print(by.pivot_table(index="horizon_s", columns="variant", values="median_abs").round(2).to_string())
    print(by.pivot_table(index="horizon_s", columns="variant", values="p95_abs").round(2).to_string())
    print("high-frequency (first 60 s, mean removed):", json.dumps(hf_tab))


if __name__ == "__main__":
    main()

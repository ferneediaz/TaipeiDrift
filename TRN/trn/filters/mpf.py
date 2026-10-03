"""Marginalized (Rao-Blackwellized) particle filter for laser TRN.

Nonlinear (particle) states: xi = INS position error [dE, dN, dU]; estimated position = INS - xi.
Linear states per particle (Kalman, shared covariance P because the model matrices do not depend on
the particle): velocity error, misalignment, accel/gyro biases, baro bias (Schoen, Gustafsson &
Nordlund 2005, with the particle increment used as pseudo-measurement of the velocity error).

Variants (same code, different laser likelihood):
  baseline  - Gaussian on the last echo, missing echoes skipped (Carroll & Canciani 2021 style)
  gated     - baseline + innovation gate (robust reference, not in the literature baseline)
  proposed  - obscuration-aware multi-echo mixture likelihood (trn.filters.likelihoods)
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
from numba import njit, prange

from trn.filters.base import Estimate, Measurement, NavFilter
from trn.filters.likelihoods import loglik_baseline, loglik_proposed
from trn.filters.resampling import roughen, systematic_resample
from trn.ins.error_model import ErrorModel
from trn.laser.sensor_model import beam_body_vectors, p_detect
from trn.terrain.grid import interp1, raycast1
from trn.terrain.onboard_map import OnboardMap


@njit(cache=True, parallel=True)
def predict_ranges(z, x0, y0, dx, smax, slope, ins_pos, xi, dirs, rmax, mstep, tol, sig_l, sig_map, c_slope,
                   rho, sig):
    """Predicted ground range and its sigma for every particle and beam on the ONBOARD map."""
    N = xi.shape[0]
    B = dirs.shape[0]
    for i in prange(N):
        px = ins_pos[0] - xi[i, 0]
        py = ins_pos[1] - xi[i, 1]
        pz = ins_pos[2] - xi[i, 2]
        for b in range(B):
            ux, uy, uz = dirs[b, 0], dirs[b, 1], dirs[b, 2]
            r = raycast1(z, x0, y0, dx, smax, px, py, pz, ux, uy, uz, rmax * 1.2, mstep, tol)
            rho[i, b] = r
            if np.isnan(r):
                sig[i, b] = 1.0
                continue
            sl = interp1(slope, x0, y0, dx, px + ux * r, py + uy * r)
            if np.isnan(sl):
                sl = 0.0
            sh2 = sig_map * sig_map + (c_slope * sl * dx) ** 2
            uzz = max(abs(uz), 0.2)
            sig[i, b] = math.sqrt(sig_l * sig_l + sh2 / (uzz * uzz))


class MarginalizedPF(NavFilter):
    """MPF with selectable laser likelihood variant."""

    def __init__(self, onboard: OnboardMap, cfg: dict, variant: str, em: ErrorModel, rng: np.random.Generator,
                 baro_enabled: bool):
        super().__init__(onboard)
        if variant not in ("baseline", "gated", "proposed"):
            raise ValueError(variant)
        self.variant = variant
        self.name = {"baseline": "MPF-baseline", "gated": "MPF-gated", "proposed": "MPF-proposed"}[variant]
        self.fc = cfg["filters"]["common"]
        self.vc = cfg["filters"][f"mpf_{variant}"]
        self.lc = cfg["laser"]
        self.em = em
        self.rng = rng
        self.baro_enabled = baro_enabled
        self.N = int(self.fc["n_particles"])
        self.beams = beam_body_vectors(self.lc["beam_sets"][self.lc["beam_set"]])
        self.vel_sigma = float(cfg["imu"]["init_errors"]["vel_mps"])
        g = onboard.grid
        self._g = (np.asarray(g.z), g.x0, g.y0, g.dx, g.slope_bound())
        self._slope = np.asarray(onboard.slope.z)
        self.prev: Measurement | None = None
        self.prev_t: float | None = None
        self.nis = deque(maxlen=int(self.fc["divergence"]["window"]))
        self.last_reinit = -1e9
        self.last_baro = -1e9
        self.pvis = float(self.vc.get("p_visible", 0.8))
        self.lam_s = float(self.vc.get("short_rate", 0.5))

    # ------------------------------------------------------------------ init / output
    def initialize(self, m: Measurement) -> Estimate:
        sh, sv = self.fc["init_sigma_horiz_m"], self.fc["init_sigma_vert_m"]
        k = float(self.fc["init_proposal_inflation"])
        u = self.rng.standard_normal((self.N, 3))
        self.xi = u * k * np.array([sh, sh, sv])
        self.xl = np.zeros((self.N, 13))
        self.P = self.em.P0_linear(self.vel_sigma)
        self.logw = -0.5 * np.sum(u ** 2, axis=1) * (k * k - 1.0)   # prior / proposal
        self.prev = m
        return self._measurement_update(m)

    def _estimate(self, m: Measurement, neff: float, reinit: bool) -> Estimate:
        w = self._w()
        mu = w @ self.xi
        d = self.xi[:, :2] - mu[:2]
        cov = (d * w[:, None]).T @ d
        return Estimate(pos=m.ins_pos - mu, cov=cov, neff=neff, reinit=reinit,
                        extra={"pvis": self.pvis, "lam_s": self.lam_s})

    def _w(self) -> np.ndarray:
        w = np.exp(self.logw - self.logw.max())
        return w / w.sum()

    # ------------------------------------------------------------------ time update
    def _time_update(self, prev: Measurement, dt: float) -> None:
        Axx, An, Alx, Al, Qn, Ql = self.em.discrete(prev.ins_fn, prev.ins_vel, prev.ins_C, prev.ins_pos, dt)
        Qn = Qn + np.eye(3) * (self.fc["pos_process_noise_mps"] ** 2) * dt
        P = self.P
        Nm = An @ P @ An.T + Qn
        Lc = np.linalg.cholesky(Nm)
        pred = self.xi @ Axx.T + self.xl @ An.T
        xi_new = pred + self.rng.standard_normal((self.N, 3)) @ Lc.T
        z = xi_new - self.xi @ Axx.T
        L = Al @ P @ An.T @ np.linalg.inv(Nm)
        self.xl = self.xl @ Al.T + self.xi @ Alx.T + (z - self.xl @ An.T) @ L.T
        P = Al @ P @ Al.T + Ql - L @ Nm @ L.T
        self.P = 0.5 * (P + P.T)
        self.xi = xi_new

    # ------------------------------------------------------------------ measurement update
    def _measurement_update(self, m: Measurement) -> Estimate:
        z, x0, y0, dx, smax = self._g
        dirs = np.ascontiguousarray((m.ins_C @ self.beams.T).T)
        B = dirs.shape[0]
        rho = np.empty((self.N, B)); sig = np.empty((self.N, B))
        predict_ranges(z, x0, y0, dx, smax, self._slope, m.ins_pos, self.xi, dirs, self.lc["max_range_m"],
                       self.lc["raycast_min_step_m"], 0.05, self.lc["range_sigma_m"], self.vc.get("map_sigma_m", self.fc["map_sigma_m"]),
                       self.fc["map_slope_coeff"], rho, sig)
        w = self._w()
        ok = ~np.isnan(rho)
        rho0 = np.where(ok, rho, 0.0)
        wsum = np.maximum(w @ ok, 1e-12)
        rbar = (w @ rho0) / wsum
        rvar = (w @ ((rho0 - rbar) ** 2 * ok)) / wsum
        sbar2 = w @ (sig ** 2)
        ranges = m.ranges
        ll = np.empty(self.N)
        nis_val = np.nan
        if self.variant in ("baseline", "gated"):
            last = np.array([r[np.isfinite(r)][-1] if np.isfinite(r).any() else np.nan for r in ranges])
            use = np.isfinite(last)
            if self.variant == "gated":
                use &= np.abs(last - rbar) <= self.vc["gate"] * np.sqrt(sbar2 + rvar)
            loglik_baseline(ranges, rho, sig, use, ll)
            if np.isfinite(last).any():
                b = int(np.argmax(np.isfinite(last)))
                nis_val = (last[b] - rbar[b]) ** 2 / (sbar2[b] + rvar[b])
                if self.variant == "gated" and not use[b]:
                    nis_val = np.nan
        else:
            pg = np.empty((self.N, B))
            vc = self.vc
            loglik_proposed(ranges, rho, sig, self.pvis, self.lam_s, vc["canopy_weight"], vc["canopy_length_m"],
                            vc["clutter_rate"], self.lc["max_range_m"], vc["min_range_m"], self.lc["detect_r50_m"],
                            self.lc["detect_width_m"], self.lc["detect_pmax"], ll, pg)
        dt = m.t - self.prev_t if self.prev_t is not None else 1.0 / self.lc["pulse_rate_hz"]
        self.prev_t = m.t
        alpha = min(1.0, float(np.linalg.norm(m.ins_vel[:2])) * max(dt, 1e-3) / self.fc["decorrelation_length_m"])
        ll = alpha * ll
        beta = self._temper(ll)
        self.logw = self.logw + beta * ll
        self.logw -= self.logw.max()
        if self.variant == "proposed":
            wp = self._w()
            beta = wp @ pg                                   # posterior P(ground echo present) per beam
            kcount = np.isfinite(ranges).sum(axis=1)
            for b in range(B):
                if beta[b] > 0.5 and kcount[b] > 0:
                    rr = ranges[b][np.isfinite(ranges[b])]
                    rsel = rr[np.argmin(np.abs(rr - rbar[b]))]
                    nis_val = (rsel - rbar[b]) ** 2 / (sbar2[b] + rvar[b])
                    break
            spread = float(np.sqrt(np.sum(wp @ (self.xi[:, :2] - wp @ self.xi[:, :2]) ** 2)))
            if vc["adapt"] and spread < vc["adapt_max_spread_m"]:
                a = vc["adapt_alpha"]
                pdc = np.array([p_detect(r, self.lc["detect_r50_m"], self.lc["detect_width_m"], self.lc["detect_pmax"])
                                for r in rbar])
                pv_obs = float(np.mean(beta / np.maximum(pdc, 1e-3)))
                ls_obs = float(np.mean(np.maximum(kcount - beta - vc["clutter_rate"], 0.0)))
                lo, hi = vc["p_visible_bounds"]
                self.pvis = float(np.clip((1 - a) * self.pvis + a * pv_obs, lo, hi))
                lo, hi = vc["short_rate_bounds"]
                self.lam_s = float(np.clip((1 - a) * self.lam_s + a * ls_obs, lo, hi))
        # barometer (linear measurement in xi_U and baro bias)
        bc = self.fc["baro_rate_hz"]
        if self.baro_enabled and np.isfinite(m.baro) and m.t - self.last_baro >= 1.0 / bc - 1e-9:
            self.last_baro = m.t
            Rb = self.em.baro_noise ** 2
            S = self.P[12, 12] + Rb
            nu = m.baro - (m.ins_pos[2] - self.xi[:, 2]) - self.xl[:, 12]
            self.logw += -0.5 * nu ** 2 / S
            K = self.P[:, 12] / S
            self.xl += nu[:, None] * K[None, :]
            self.P = self.P - np.outer(K, K) * S
        # divergence monitor
        reinit = False
        dc = self.fc["divergence"]
        if np.isfinite(nis_val):
            self.nis.append(nis_val)
        if (dc["enabled"] and len(self.nis) == self.nis.maxlen and np.mean(self.nis) > dc["nis_threshold"]
                and m.t - self.last_reinit > dc["cooldown_s"]):
            w = self._w()
            mu = w @ self.xi
            self.xi[:, :2] = mu[:2] + dc["reinit_sigma_m"] * self.rng.standard_normal((self.N, 2))
            self.logw[:] = 0.0
            self.nis.clear()
            self.last_reinit = m.t
            reinit = True
        # resampling + roughening
        w = self._w()
        neff = 1.0 / np.sum(w ** 2)
        if neff < self.fc["resample_neff_frac"] * self.N:
            idx = systematic_resample(w, self.rng)
            self.xi = self.xi[idx]
            self.xl = self.xl[idx]
            self.logw = np.zeros(self.N)
            roughen(self.xi, self.fc["roughening_K"], self.rng)
            h = float(self.fc["linear_roughening_h"])
            if h > 0:
                Lp = np.linalg.cholesky(self.P + 1e-12 * np.eye(self.P.shape[0]))
                self.xl += h * self.rng.standard_normal(self.xl.shape) @ Lp.T
        return self._estimate(m, neff, reinit)

    def _temper(self, ll: np.ndarray) -> float:
        """Largest exponent beta in (0, 1] keeping N_eff >= min_neff_frac * N after the update."""
        tc = self.fc["tempering"]
        if not tc["enabled"]:
            return 1.0
        target = tc["min_neff_frac"] * self.N

        def neff(b: float) -> float:
            lw = self.logw + b * ll
            w = np.exp(lw - lw.max())
            w /= w.sum()
            return 1.0 / np.sum(w * w)

        if neff(1.0) >= target:
            return 1.0
        lo, hi = 0.0, 1.0
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            if neff(mid) >= target:
                lo = mid
            else:
                hi = mid
        self.last_beta = lo
        return max(lo, 1e-4)

    # ------------------------------------------------------------------ public
    def step(self, m: Measurement) -> Estimate:
        dt = m.t - self.prev.t
        if dt > 0:
            self._time_update(self.prev, dt)
        self.prev = m
        return self._measurement_update(m)

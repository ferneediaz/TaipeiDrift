"""Laser measurement likelihoods (numba). rho = predicted ground range from the onboard map.

Baseline (Carroll & Canciani style): Gaussian on the LAST echo, missing echoes skipped.
Proposed (obscuration-aware): association mixture over all echoes {r_1 < ... < r_k}:
    L = (1 - P_D) prod_i kappa(r_i) + P_D sum_j N(r_j; rho, s^2) prod_{i<j} kappa(r_i) prod_{i>j} kappa_c
    kappa(r)  = lambda_s c_short(r | rho) + kappa_c        (cloud / fog / canopy, and clutter)
    c_short   = w_can * canopy(rho - r) + (1 - w_can) * U(r_min, rho)   (soft cut-off at rho)
    kappa_c   = lambda_c / R_max ;   P_D = p_visible * P_detect(rho);   k = 0 -> L = 1 - P_D
Echoes longer than the predicted ground range cannot be cloud/canopy, which makes the hypotheses
position dependent and informative.
"""
from __future__ import annotations

import math

import numpy as np
from numba import njit, prange

from trn.laser.sensor_model import p_detect

SQRT2PI = math.sqrt(2.0 * math.pi)


@njit(cache=True, inline="always")
def _phi(x):
    return 0.5 * math.erfc(-x / math.sqrt(2.0))


@njit(cache=True, parallel=True)
def loglik_baseline(ranges, rho, sig, use_beam, out):
    """Sum over beams of log N(last echo; rho, sig^2). ranges (B,M); rho, sig (N,B); use_beam (B,) bool."""
    N, B = rho.shape
    M = ranges.shape[1]
    for i in prange(N):
        acc = 0.0
        for b in range(B):
            if not use_beam[b]:
                continue
            r = np.nan
            for e in range(M):
                if not np.isnan(ranges[b, e]):
                    r = ranges[b, e]
            if np.isnan(r):
                continue
            if np.isnan(rho[i, b]):
                acc += -12.5  # unknown map / no predicted hit: ~5-sigma penalty
                continue
            z = (r - rho[i, b]) / sig[i, b]
            acc += -0.5 * z * z - math.log(sig[i, b])
        out[i] = acc


@njit(cache=True, parallel=True)
def loglik_proposed(ranges, rho, sig, pvis, lam_s, w_can, l_can, lam_c, rmax, rmin, r50, wd, pmax,
                    out, post_g):
    """Obscuration-aware mixture likelihood; also returns the per-particle posterior P(ground echo present)
    summed over beams in post_g (N, B)."""
    N, B = rho.shape
    M = ranges.shape[1]
    kc = lam_c / rmax
    for i in prange(N):
        acc = 0.0
        for b in range(B):
            k = 0
            for e in range(M):
                if not np.isnan(ranges[b, e]):
                    k += 1
            rh = rho[i, b]
            if np.isnan(rh):
                # unknown map: only the "no ground" explanation with a generic density
                lk = 1.0
                for e in range(k):
                    lk *= (lam_s + lam_c) / rmax
                acc += math.log(lk + 1e-300) - 1.0
                post_g[i, b] = 0.0
                continue
            s = sig[i, b]
            pd = pvis * p_detect(rh, r50, wd, pmax)
            kap = np.empty(M)
            for e in range(k):
                r = ranges[b, e]
                d = rh - r
                can = math.exp(-max(d, 0.0) / l_can) / l_can * _phi(d / s)
                uni = _phi(d / s) / max(rh - rmin, 1.0)
                kap[e] = lam_s * (w_can * can + (1.0 - w_can) * uni) + kc
            t0 = 1.0 - pd
            for e in range(k):
                t0 *= kap[e]
            t1 = 0.0
            for j in range(k):
                z = (ranges[b, j] - rh) / s
                term = math.exp(-0.5 * z * z) / (SQRT2PI * s)
                for e in range(j):
                    term *= kap[e]
                for e in range(j + 1, k):
                    term *= kc
                t1 += term
            L = t0 + pd * t1
            acc += math.log(L + 1e-300)
            post_g[i, b] = pd * t1 / (L + 1e-300)
        out[i] = acc

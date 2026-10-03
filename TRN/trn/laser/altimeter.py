"""Multi-echo pulsed laser altimeter simulated on the TRUTH surfaces (simulator side only).

Per pulse and beam: footprint-averaged ray cast on the DSM top surface; canopy/building returns with
ground echoes through gaps (ground = native 20 m DEM); cloud and valley-fog echoes with two-way
attenuation exp(-2 tau); range-dependent detection; spurious echoes; noise and quantisation;
at most ``max_echoes`` (first / intermediate / last). Echo source labels are kept for diagnostics only
and are never passed to the filters.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numba import njit, prange

from trn.laser.sensor_model import beam_body_vectors, p_detect
from trn.terrain.grid import interp1, raycast1

LBL_NONE, LBL_GROUND, LBL_CANOPY, LBL_CLOUD, LBL_FOG, LBL_CLUTTER, LBL_SEA = 0, 1, 2, 3, 4, 5, 6
LABEL_NAMES = {0: "none", 1: "ground", 2: "canopy/building", 3: "cloud", 4: "fog", 5: "clutter", 6: "sea"}
N_UNIFORM = 12





@njit(cache=True)
def _field(arr, x0, y0, res, x, y):
    """Nearest-cell lookup in a periodic field."""
    h, w = arr.shape
    i = int(math.floor((y0 - y) / res + 0.5)) % h
    j = int(math.floor((x - x0) / res + 0.5)) % w
    return arr[i, j]


@njit(cache=True)
def _clear_tau(z_hi, z_lo, uzabs, ext0, H):
    z_lo = max(z_lo, 0.0)
    z_hi = max(z_hi, z_lo)
    return ext0 * H * (math.exp(-z_lo / H) - math.exp(-z_hi / H)) / max(uzabs, 1e-3)


@njit(cache=True)
def _trunc_exp(u, sigma, L):
    """Sample penetration depth (two-way extinction 2*sigma) truncated to [0, L]."""
    a = 2.0 * sigma
    return -math.log(1.0 - u * (1.0 - math.exp(-a * L))) / a


@njit(cache=True, parallel=True)
def simulate_pulses(top, tx0, ty0, tdx, tsmax, gnd, gx0, gy0, gdx, gsmax, sea, origins, dirs, foot, times,
                    U, Z, lp, atm_x0, atm_y0, atm_res, thick, base, ext_c, fog, fog_top, ext_f, ext0, scale_h,
                    wind_e, wind_n, atm_on, ranges, labels, r_true):
    """Simulate all pulses. lp = laser parameter vector (see :func:`laser_params`)."""
    sig, quant, rmax, r50, wdt, pmax, sea_f, can_thr, p_gap, sep, max_e, p_fa, mstep, tol = (
        lp[0], lp[1], lp[2], lp[3], lp[4], lp[5], lp[6], lp[7], lp[8], lp[9], int(lp[10]), lp[11], lp[12], lp[13])
    n, B = dirs.shape[0], dirs.shape[1]
    nf = foot.shape[2]
    for k in prange(n):
        px, py, pz = origins[k, 0], origins[k, 1], origins[k, 2]
        sx = wind_e * times[k]
        sy = wind_n * times[k]
        for b in range(B):
            ux, uy, uz = dirs[k, b, 0], dirs[k, b, 1], dirs[k, b, 2]
            # footprint-averaged top-surface range
            r_c = raycast1(top, tx0, ty0, tdx, tsmax, px, py, pz, ux, uy, uz, rmax, mstep, tol)
            acc = 0.0
            cnt = 0
            if not np.isnan(r_c):
                acc = r_c
                cnt = 1
                for q in range(nf):
                    rq = raycast1(top, tx0, ty0, tdx, tsmax, px, py, pz, foot[k, b, q, 0], foot[k, b, q, 1],
                                  foot[k, b, q, 2], rmax, mstep, tol)
                    if not np.isnan(rq):
                        acc += rq
                        cnt += 1
            r_top = acc / cnt if cnt > 0 else np.nan
            r_true[k, b] = r_top
            ev_r = np.full(8, np.nan)
            ev_l = np.zeros(8, np.int8)
            ne = 0
            u = U[k, b]
            if not np.isnan(r_top):
                hx = px + ux * r_c
                hy = py + uy * r_c
                hz_top = interp1(top, tx0, ty0, tdx, hx, hy)
                hz_g = interp1(gnd, gx0, gy0, gdx, hx, hy)
                ci = int(math.floor((gy0 - hy) / gdx + 0.5))
                cj = int(math.floor((hx - gx0) / gdx + 0.5))
                is_sea = False
                if ci >= 0 and cj >= 0 and ci < sea.shape[0] and cj < sea.shape[1]:
                    is_sea = sea[ci, cj] == 1
                canopy = (not np.isnan(hz_g)) and (hz_top - hz_g > can_thr)
                tau_acc = 0.0
                uzabs = -uz
                if atm_on == 1 and uzabs > 1e-3:
                    # --- cloud layer
                    rb = (pz - base) / uzabs
                    if rb > 0.0:
                        T = _field(thick, atm_x0, atm_y0, atm_res, px + ux * rb - sx, py + uy * rb - sy)
                        if T > 0.0:
                            r_in = max(0.0, (pz - base - T) / uzabs)
                            r_out = min(rb, r_top)
                            if r_out > r_in:
                                L = r_out - r_in
                                tl = ext_c * L
                                pe = p_detect(r_in, r50, wdt, pmax) * (1.0 - math.exp(-2.0 * tl))
                                if u[0] < pe and ne < 8:
                                    ev_r[ne] = r_in + _trunc_exp(u[1], ext_c, L)
                                    ev_l[ne] = 3
                                    ne += 1
                                tau_acc += tl
                    # --- valley fog (below fog top, in fog-covered valley cells at the hit point)
                    if fog_top > hz_top and _field(fog, atm_x0, atm_y0, atm_res, hx, hy) == 1:
                        r_in = max(0.0, (pz - fog_top) / uzabs)
                        if r_top > r_in:
                            L = r_top - r_in
                            tl = ext_f * L
                            pe = p_detect(r_in, r50, wdt, pmax) * math.exp(-2.0 * tau_acc) * (1.0 - math.exp(-2.0 * tl))
                            if u[2] < pe and ne < 8:
                                ev_r[ne] = r_in + _trunc_exp(u[3], ext_f, L)
                                ev_l[ne] = 4
                                ne += 1
                            tau_acc += tl
                    tau_acc += _clear_tau(pz, hz_top, uzabs, ext0, scale_h)
                trans = math.exp(-2.0 * tau_acc)
                if canopy:
                    if u[4] < p_detect(r_top, r50, wdt, pmax) * trans and ne < 8:
                        ev_r[ne] = r_top
                        ev_l[ne] = 2
                        ne += 1
                    if u[5] < p_gap:
                        r_g = raycast1(gnd, gx0, gy0, gdx, gsmax, px, py, pz, ux, uy, uz, rmax, mstep, tol)
                        if not np.isnan(r_g) and u[6] < p_detect(r_g, r50, wdt, pmax) * trans and ne < 8:
                            ev_r[ne] = r_g
                            ev_l[ne] = 1
                            ne += 1
                else:
                    pd = p_detect(r_top, r50, wdt, pmax) * trans * (sea_f if is_sea else 1.0)
                    if u[6] < pd and ne < 8:
                        ev_r[ne] = r_top
                        ev_l[ne] = 6 if is_sea else 1
                        ne += 1
            if u[7] < p_fa and ne < 8:
                ev_r[ne] = u[8] * rmax
                ev_l[ne] = 5
                ne += 1
            # noise, quantisation, range gate
            m = 0
            for e in range(ne):
                r = ev_r[e] + sig * Z[k, b, e % Z.shape[2]]
                r = math.floor(r / quant + 0.5) * quant
                if r > 0.0 and r <= rmax:
                    ev_r[m] = r
                    ev_l[m] = ev_l[e]
                    m += 1
            # sort by range (insertion sort, m <= 8)
            for i in range(1, m):
                j = i
                while j > 0 and ev_r[j - 1] > ev_r[j]:
                    tr = ev_r[j - 1]; ev_r[j - 1] = ev_r[j]; ev_r[j] = tr
                    tl2 = ev_l[j - 1]; ev_l[j - 1] = ev_l[j]; ev_l[j] = tl2
                    j -= 1
            # merge echoes closer than the pulse separation (first one wins)
            q = 0
            for i in range(m):
                if q == 0 or ev_r[i] - ev_r[q - 1] >= sep:
                    ev_r[q] = ev_r[i]
                    ev_l[q] = ev_l[i]
                    q += 1
            # keep first / intermediate / last
            if q > max_e:
                ev_r[1] = ev_r[q - 2]; ev_l[1] = ev_l[q - 2]
                ev_r[2] = ev_r[q - 1]; ev_l[2] = ev_l[q - 1]
                q = max_e
            for i in range(max_e):
                if i < q:
                    ranges[k, b, i] = ev_r[i]
                    labels[k, b, i] = ev_l[i]
                else:
                    ranges[k, b, i] = np.nan
                    labels[k, b, i] = 0


def laser_params(lc: dict) -> np.ndarray:
    return np.array([lc["range_sigma_m"], lc["range_quant_m"], lc["max_range_m"], lc["detect_r50_m"],
                     lc["detect_width_m"], lc["detect_pmax"], lc["sea_detect_factor"], lc["canopy_threshold_m"],
                     lc["canopy_gap_prob"], lc["min_echo_separation_m"], lc["max_echoes"], lc["false_alarm_prob"],
                     lc["raycast_min_step_m"], lc["raycast_tol_m"]], dtype=np.float64)


@dataclass
class LaserData:
    """Simulated echoes (n_pulses, n_beams, max_echoes) + diagnostics."""

    t: np.ndarray
    ranges: np.ndarray
    labels: np.ndarray       # DIAGNOSTIC ONLY - never given to filters
    r_true: np.ndarray       # true top-surface range per beam (diagnostic)
    beams_body: np.ndarray


def footprint_dirs(d: np.ndarray, half_div: float, n_rays: int) -> np.ndarray:
    """(n,B,n_rays-1,3) directions on the divergence cone around beam directions d (n,B,3)."""
    m = max(n_rays - 1, 0)
    if m == 0:
        return np.zeros(d.shape[:2] + (0, 3))
    ref = np.where(np.abs(d[..., 2:3]) < 0.9, np.array([0, 0, 1.0]), np.array([1.0, 0, 0]))
    e1 = np.cross(d, ref); e1 /= np.linalg.norm(e1, axis=-1, keepdims=True)
    e2 = np.cross(d, e1)
    ang = 2 * np.pi * np.arange(m) / m
    out = (d[..., None, :] + np.tan(half_div) * (np.cos(ang)[None, None, :, None] * e1[..., None, :]
                                                 + np.sin(ang)[None, None, :, None] * e2[..., None, :]))
    return out / np.linalg.norm(out, axis=-1, keepdims=True)


def simulate_laser(lc: dict, truth, atm, t: np.ndarray, pos: np.ndarray, C: np.ndarray,
                   rng: np.random.Generator) -> LaserData:
    """Simulate the altimeter at truth poses (pos (n,3), C (n,3,3)) and times t."""
    bb = beam_body_vectors(lc["beam_sets"][lc["beam_set"]])
    dirs = np.einsum("nij,bj->nbi", C, bb)
    foot = footprint_dirs(dirs, 0.5e-3 * lc["divergence_mrad"], int(lc["footprint_rays"]))
    n, B, M = dirs.shape[0], dirs.shape[1], int(lc["max_echoes"])
    U = rng.random((n, B, 12))
    Z = rng.standard_normal((n, B, 8))
    ranges = np.empty((n, B, M)); labels = np.empty((n, B, M), np.int8); r_true = np.empty((n, B))
    tg, gg = truth.top, truth.ground
    simulate_pulses(tg.z, tg.x0, tg.y0, tg.dx, tg.slope_bound(), gg.z, gg.x0, gg.y0, gg.dx, gg.slope_bound(),
                    np.asarray(truth.sea), np.ascontiguousarray(pos), np.ascontiguousarray(dirs),
                    np.ascontiguousarray(foot), t, U, Z, laser_params(lc), *atm.as_tuple(), ranges, labels, r_true)
    return LaserData(t, ranges, labels, r_true, bb)

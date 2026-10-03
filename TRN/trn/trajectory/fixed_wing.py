"""Fixed-wing truth trajectory: waypoint line following with bank-limited coordinated turns.

Outputs a discrete truth at ``rate_hz`` that is exactly consistent with the strapdown equations
in :mod:`trn.ins.strapdown` (positions are Euler-integrated velocities), so an error-free IMU
reproduces the truth to numerical precision.
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numba import njit
from scipy.ndimage import maximum_filter, maximum_filter1d, uniform_filter1d

from trn.common.config import project_path
from trn.common.frames import latitude_model, lonlat_to_tm2
from trn.ins.nav_math import euler_to_C


@dataclass
class Trajectory:
    """Truth trajectory sampled at rate 1/dt."""

    t: np.ndarray        # (n,)
    pos: np.ndarray      # (n,3) E, N, U(MSL) [m]
    vel: np.ndarray      # (n,3) ENU [m/s]
    C: np.ndarray        # (n,3,3) body(FLU)->ENU
    euler: np.ndarray    # (n,3) roll, pitch, yaw [rad]
    dt: float
    lat_ref: float
    n_ref: float
    route: str

    @property
    def n(self) -> int:
        return self.t.shape[0]


@njit(cache=True)
def _guidance(wps, V, dt, max_bank, tau, gain, lookahead, g, max_steps):
    E = np.empty(max_steps); N = np.empty(max_steps); chi = np.empty(max_steps); phi = np.empty(max_steps)
    d0 = wps[1] - wps[0]
    E[0] = wps[0, 0]; N[0] = wps[0, 1]; chi[0] = math.atan2(d0[1], d0[0]); phi[0] = 0.0
    seg = 0
    nseg = wps.shape[0] - 1
    k = 0
    while k < max_steps - 1:
        a = wps[seg]; b = wps[seg + 1]
        sv = b - a
        L = math.sqrt(sv[0] ** 2 + sv[1] ** 2)
        u = sv / L
        s = (E[k] - a[0]) * u[0] + (N[k] - a[1]) * u[1]
        # switch segment with turn anticipation
        if seg < nseg - 1:
            c = wps[seg + 2] - b
            turn = abs(math.atan2(u[0] * c[1] - u[1] * c[0], u[0] * c[0] + u[1] * c[1]))
            rmin = V * V / (g * math.tan(max_bank))
            if s >= L - rmin * math.tan(0.5 * turn) - V * tau:
                seg += 1
                continue
        elif s >= L:
            break
        tx = a[0] + u[0] * (s + lookahead); ty = a[1] + u[1] * (s + lookahead)
        chid = math.atan2(ty - N[k], tx - E[k])
        err = (chid - chi[k] + math.pi) % (2 * math.pi) - math.pi
        cmd = min(max_bank, max(-max_bank, -gain * err))
        phi[k + 1] = phi[k] + dt * (cmd - phi[k]) / tau
        chi[k + 1] = chi[k] - dt * g * math.tan(phi[k]) / V
        E[k + 1] = E[k] + dt * V * math.cos(chi[k])
        N[k + 1] = N[k] + dt * V * math.sin(chi[k])
        k += 1
    return E[:k + 1], N[:k + 1], chi[:k + 1], phi[:k + 1]


def _altitude_profile(cfg: dict, route: str, E: np.ndarray, N: np.ndarray, top_grid) -> np.ndarray:
    tc = cfg["trajectory"]
    rc = cfg["routes"][route]
    if tc["altitude_mode"] == "msl":
        return np.full(E.shape, float(rc["altitude_msl_m"]))
    # constant AGL: terrain envelope (corridor max), look-ahead, climb-rate limit
    V, dt = tc["speed_mps"], 1.0 / tc["rate_hz"]
    corr = int(round(tc["agl_corridor_m"] / top_grid.dx))
    rmin, cmin = top_grid.xy_to_rc(E, N)
    r0, r1 = int(rmin.min()) - corr - 2, int(rmin.max()) + corr + 3
    c0, c1 = int(cmin.min()) - corr - 2, int(cmin.max()) + corr + 3
    sub = np.nan_to_num(np.asarray(top_grid.z[r0:r1, c0:c1], np.float64), nan=0.0)
    env = maximum_filter(sub, size=2 * corr + 1)
    ri = np.clip(np.rint(rmin).astype(int) - r0, 0, sub.shape[0] - 1)
    ci = np.clip(np.rint(cmin).astype(int) - c0, 0, sub.shape[1] - 1)
    agl = tc.get("agl_m") or rc["agl_m"]
    h = env[ri, ci] + float(agl)
    la = max(1, int(tc["agl_lookahead_m"] / (V * dt)))
    h = maximum_filter1d(h, 2 * la + 1, mode="nearest")
    dmax = tc["max_climb_mps"] * dt
    for k in range(1, h.size):
        h[k] = max(h[k], h[k - 1] - dmax)
    for k in range(h.size - 2, -1, -1):
        h[k] = max(h[k], h[k + 1] - dmax)
    hs = uniform_filter1d(h, int(10.0 / dt), mode="nearest")
    return np.maximum(hs, h - 5.0)


def generate(cfg: dict, route: str, top_grid) -> Trajectory:
    """Generate (or load cached) truth trajectory for a route."""
    tc = cfg["trajectory"]
    rc = cfg["routes"][route]
    key = hashlib.sha1(json.dumps([tc, rc], sort_keys=True).encode()).hexdigest()[:10]
    cache = project_path(cfg["data"]["processed_dir"]) / route / f"traj_{key}.npz"
    if cache.exists():
        z = np.load(cache)
        return Trajectory(z["t"], z["pos"], z["vel"], z["C"], z["euler"], float(z["dt"]), float(z["lat_ref"]),
                          float(z["n_ref"]), route)
    wps = lonlat_to_tm2(np.array(rc["waypoints_lonlat"]))
    V, dt = float(tc["speed_mps"]), 1.0 / float(tc["rate_hz"])
    g = float(cfg["imu"]["gravity_mps2"])
    length = np.hypot(*np.diff(wps, axis=0).T).sum()
    E, N, chi, phi = _guidance(wps, V, dt, math.radians(tc["max_bank_deg"]), tc["bank_time_const_s"],
                               tc["heading_gain"], tc["lookahead_m"], g, int(3 * length / V / dt))
    U = _altitude_profile(cfg, route, E, N, top_grid)
    pos = np.column_stack([E, N, U])
    vel = np.empty_like(pos)
    vel[:-1] = np.diff(pos, axis=0) / dt
    vel[-1] = vel[-2]
    pos = np.vstack([pos[:1], pos[0] + np.cumsum(vel[:-1] * dt, axis=0)])   # exact Euler consistency
    pitch = np.arctan2(vel[:, 2], np.hypot(vel[:, 0], vel[:, 1]))
    euler = np.column_stack([phi, pitch, chi])
    C = euler_to_C(phi, pitch, chi)
    lat_ref, n_ref = latitude_model(*wps[0])
    t = np.arange(pos.shape[0]) * dt
    np.savez(cache, t=t, pos=pos, vel=vel, C=C, euler=euler, dt=dt, lat_ref=lat_ref, n_ref=n_ref)
    return Trajectory(t, pos, vel, C, euler, dt, lat_ref, n_ref, route)


def check_clearance(traj: Trajectory, top_grid, min_clearance: float, corridor_m: float = 300.0) -> float:
    """Minimum clearance above the DSM within a +-corridor (asserted >= min_clearance)."""
    k = int(round(corridor_m / top_grid.dx))
    r, c = top_grid.xy_to_rc(traj.pos[::10, 0], traj.pos[::10, 1])
    clear = np.inf
    for rr, cc, h in zip(np.rint(r).astype(int), np.rint(c).astype(int), traj.pos[::10, 2]):
        win = np.asarray(top_grid.z[max(rr - k, 0):rr + k + 1, max(cc - k, 0):cc + k + 1])
        clear = min(clear, h - np.nanmax(win))
    assert clear >= min_clearance, f"terrain clearance {clear:.0f} m < {min_clearance} m"
    return float(clear)

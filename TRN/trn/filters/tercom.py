"""TERCOM-style batch profile correlation (classic baseline).

Accumulate a profile of measured terrain heights (altitude - nadir range * |u_z|, last echo) along the
INS track; every ``profile_length_m`` search a grid of horizontal offsets around the current correction
and pick the minimum of the mean-removed MSD (or MAD). The fix resets the horizontal position offset.
"""
from __future__ import annotations

import numpy as np
from numba import njit, prange

from trn.filters.base import Estimate, Measurement, NavFilter
from trn.laser.sensor_model import beam_body_vectors
from trn.terrain.grid import interp1
from trn.terrain.onboard_map import OnboardMap


@njit(cache=True, parallel=True)
def _search(z, x0, y0, dx, e, n, hm, cE, cN, offs, metric, out):
    K = offs.shape[0]
    m = e.shape[0]
    for k in prange(K):
        d = np.empty(m)
        cnt = 0
        for i in range(m):
            h = interp1(z, x0, y0, dx, e[i] - cE - offs[k, 0], n[i] - cN - offs[k, 1])
            if not np.isnan(h):
                d[cnt] = hm[i] - h
                cnt += 1
        if cnt < 0.9 * m:
            out[k] = np.inf
            continue
        mu = d[:cnt].mean()
        if metric == 0:
            out[k] = np.mean((d[:cnt] - mu) ** 2)
        else:
            out[k] = np.mean(np.abs(d[:cnt] - mu))


class Tercom(NavFilter):
    """Batch profile-correlation filter producing position resets."""

    name = "TERCOM"

    def __init__(self, onboard: OnboardMap, cfg: dict):
        super().__init__(onboard)
        self.tc = cfg["filters"]["tercom"]
        self.fc = cfg["filters"]["common"]
        beams = beam_body_vectors(cfg["laser"]["beam_sets"][cfg["laser"]["beam_set"]])
        self.beam = int(np.argmin(beams[:, 2]))          # most nadir-looking beam
        self.bvec = beams[self.beam]
        g = onboard.grid
        self._g = (np.asarray(g.z), g.x0, g.y0, g.dx)

    def initialize(self, m: Measurement) -> Estimate:
        self.c = np.zeros(2)
        self.sigma = float(self.fc["init_sigma_horiz_m"])
        self.buf: list = []
        self.dist = 0.0
        self.prev = m
        self.fixes = 0
        return self._out(m)

    def _out(self, m: Measurement) -> Estimate:
        pos = m.ins_pos.copy()
        pos[:2] -= self.c
        return Estimate(pos=pos, cov=np.eye(2) * self.sigma ** 2, extra={"fixes": self.fixes})

    def step(self, m: Measurement) -> Estimate:
        dt = m.t - self.prev.t
        self.sigma = float(np.hypot(self.sigma, self.tc["sigma_growth_mps"] * dt))
        self.dist += float(np.hypot(*(m.ins_pos[:2] - self.prev.ins_pos[:2])))
        self.prev = m
        r = m.ranges[self.beam]
        r = r[np.isfinite(r)]
        uz = abs((m.ins_C @ self.bvec)[2])
        alt = m.baro if np.isfinite(m.baro) else m.ins_pos[2]
        self.buf.append((m.ins_pos[0], m.ins_pos[1], alt - r[-1] * uz if r.size else np.nan))
        if self.dist >= self.tc["profile_length_m"]:
            self._fix()
            self.buf = []
            self.dist = 0.0
        return self._out(m)

    def _fix(self) -> None:
        a = np.array(self.buf)
        valid = np.isfinite(a[:, 2])
        if valid.mean() < self.tc["min_valid_frac"]:
            return
        z, x0, y0, dx = self._g
        a = a[valid]
        step = max(1, int(round(len(a) * dx / max(self.tc["profile_length_m"], 1.0))))
        a = a[::step]
        R = float(np.clip(self.tc["search_sigma_mult"] * self.sigma, self.tc["min_search_radius_m"],
                          self.tc["max_search_radius_m"]))
        g = np.arange(-R, R + dx / 2, dx)
        offs = np.array(np.meshgrid(g, g)).reshape(2, -1).T.copy()
        out = np.empty(offs.shape[0])
        _search(z, x0, y0, dx, np.ascontiguousarray(a[:, 0]), np.ascontiguousarray(a[:, 1]),
                np.ascontiguousarray(a[:, 2]), self.c[0], self.c[1], offs, 0 if self.tc["metric"] == "msd" else 1, out)
        if not np.isfinite(out).any():
            return
        k = int(np.argmin(out))
        mmin = max(out[k], 1e-6)
        msd = out if self.tc["metric"] == "msd" else out ** 2
        msd_min = max(msd[k], 1e-6)
        neff = self.tc["profile_length_m"] / self.tc["corr_length_m"]
        ww = np.exp(-0.5 * neff * (np.where(np.isfinite(msd), msd, np.inf) - msd_min) / msd_min)
        ww /= ww.sum()
        mu = ww @ offs
        var = float(np.mean(ww @ (offs - mu) ** 2)) + dx ** 2 / 12
        self.c = self.c + offs[k]
        self.sigma = float(max(np.sqrt(var), self.tc["min_sigma_m"]))
        self.fixes += 1

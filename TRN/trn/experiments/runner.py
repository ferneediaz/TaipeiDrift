"""One reproducible simulation run: truth -> INS -> atmosphere -> laser -> filters -> metrics.

Only this module (and the simulator packages) touch the truth map. Filters receive an OnboardMap and
:class:`~trn.filters.base.Measurement` objects that contain no truth information.
"""
from __future__ import annotations

import json
import time
import zlib
from dataclasses import dataclass

import numpy as np

from trn.analysis.metrics import run_metrics
from trn.atmosphere.clouds import build_atmosphere
from trn.common.rng import run_rngs
from trn.filters.base import Measurement
from trn.filters.mpf import MarginalizedPF
from trn.filters.tercom import Tercom
from trn.ins.error_model import build_error_model
from trn.ins.strapdown import baro_measurements, run_ins
from trn.laser.altimeter import LBL_GROUND, LBL_SEA, simulate_laser
from trn.terrain.onboard_builder import build_onboard_map
from trn.terrain.onboard_map import OnboardMap
from trn.trajectory.fixed_wing import check_clearance, generate
from trn.truth.truth_map import TruthMap

FILTERS = ("tercom", "mpf_baseline", "mpf_gated", "mpf_proposed")
_CACHE: dict = {}


def _truth(cfg: dict, route: str) -> TruthMap:
    k = ("truth", route)
    if k not in _CACHE:
        _CACHE[k] = TruthMap.load(cfg, route)
    return _CACHE[k]


def _trajectory(cfg: dict, route: str, truth: TruthMap):
    key = ("traj", route, json.dumps([cfg["trajectory"], cfg["routes"][route]], sort_keys=True))
    if key not in _CACHE:
        tr = generate(cfg, route, truth.top)
        check_clearance(tr, truth.top, cfg["trajectory"]["min_clearance_m"], cfg["trajectory"]["agl_corridor_m"])
        _CACHE[key] = tr
    return _CACHE[key]


def make_filter(name: str, onboard: OnboardMap, cfg: dict, em, seed: int):
    rng = np.random.default_rng(seed)
    baro = bool(cfg["imu"]["baro"]["enabled"])
    if name == "tercom":
        return Tercom(onboard, cfg)
    return MarginalizedPF(onboard, cfg, name.split("_", 1)[1], em, rng, baro)


@dataclass
class RunData:
    """Simulated sensor data of one run (kept for plots/animations)."""

    t: np.ndarray
    truth_pos: np.ndarray
    ins_pos: np.ndarray
    laser: object
    atm: object
    baro: np.ndarray
    traj: object


def simulate_sensors(cfg: dict, route: str, run_idx: int, salt: str = "") -> tuple[RunData, dict, OnboardMap]:
    """Truth, INS, atmosphere, laser and baro for one run."""
    rngs = run_rngs(int(cfg["montecarlo"]["base_seed"]), run_idx, route + salt)
    truth = _truth(cfg, route)
    traj = _trajectory(cfg, route, truth)
    rate = float(cfg["laser"]["pulse_rate_hz"])
    ins = run_ins(traj, cfg["imu"], rate, rngs["ins"])
    tpos, tC, t = traj.pos[ins.idx], traj.C[ins.idx], traj.t[ins.idx]
    atm = build_atmosphere(cfg, route, truth.ground, rngs["atmosphere"])
    laser = simulate_laser(cfg["laser"], truth, atm, t, tpos, tC, rngs["laser"])
    rt = laser.r_true[np.isfinite(laser.r_true)]
    assert rt.size == 0 or rt.max() <= cfg["trajectory"]["max_slant_range_m"] + 1, "slant range above limit"
    baro = baro_measurements(tpos[:, 2], 1.0 / rate, cfg["imu"]["baro"], rngs["baro"])
    onboard = build_onboard_map(cfg, route, rngs["map"])
    seed = int(rngs["filter"].integers(2 ** 31))
    return RunData(t, tpos, ins.pos, laser, atm, baro, traj), dict(ins=ins, seed=seed), onboard


def run_once(cfg: dict, route: str, run_idx: int, filters=FILTERS, salt: str = "", keep: bool = False) -> dict:
    """Run all filters on identical simulated data; return metrics and decimated time series."""
    rd, aux, onboard = simulate_sensors(cfg, route, run_idx, salt)
    ins = aux["ins"]
    em = build_error_model(cfg, rd.traj.lat_ref, rd.traj.n_ref)
    n = rd.t.shape[0]
    meas = [Measurement(t=float(rd.t[k]), ins_pos=ins.pos[k], ins_vel=ins.vel[k], ins_C=ins.C[k], ins_fn=ins.fn[k],
                        ranges=rd.laser.ranges[k], baro=float(rd.baro[k])) for k in range(n)]
    dec = max(1, int(round(cfg["laser"]["pulse_rate_hz"] / cfg["montecarlo"]["output_rate_hz"])))
    lab = rd.laser.labels
    usable = float(((lab == LBL_GROUND) | (lab == LBL_SEA)).any(axis=2).mean())
    out = dict(route=route, run=run_idx, usable_ground_frac=usable, t=rd.t[::dec], filters={})
    ins_err = ins.pos[:, :2] - rd.truth_pos[:, :2]
    out["ins_err"] = ins_err[::dec].astype(np.float32)
    for name in filters:
        f = make_filter(name, onboard, cfg, em, aux["seed"])
        est = np.empty((n, 3)); cov = np.empty((n, 2, 2)); neff = np.empty(n); reinit = np.zeros(n, bool)
        pvis = np.full(n, np.nan)
        t0 = time.perf_counter()
        e = f.initialize(meas[0])
        for k in range(n):
            if k > 0:
                e = f.step(meas[k])
            est[k] = e.pos; cov[k] = e.cov; neff[k] = e.neff; reinit[k] = e.reinit
            pvis[k] = e.extra.get("pvis", np.nan)
        rt = (time.perf_counter() - t0) / n
        err = est[:, :2] - rd.truth_pos[:, :2]
        m = run_metrics(rd.t, err, cov, cfg["montecarlo"]["metrics"])
        m.update(runtime_ms=rt * 1e3, n_reinit=int(reinit.sum()), realtime_factor=(1.0 / cfg["laser"]["pulse_rate_hz"]) / rt)
        rec = dict(metrics=m, err=err[::dec].astype(np.float32), sig=np.sqrt(np.trace(cov, axis1=1, axis2=2))[::dec]
                   .astype(np.float32), pvis=pvis[::dec].astype(np.float32))
        if keep:
            rec.update(est_full=est, cov_full=cov, neff_full=neff, reinit_full=reinit)
        out["filters"][name] = rec
    if keep:
        out["rundata"] = rd
        out["onboard"] = onboard
    return out

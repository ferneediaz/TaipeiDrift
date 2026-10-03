"""M3: free-inertial INS drift per IMU grade vs. analytic Schuler-loop prediction.

    python -m trn.experiments.m3_ins [--runs 20]
"""
from __future__ import annotations

import argparse
import json
import math
import shutil

import numpy as np

from trn.analysis.style import plt, setup
from trn.common.config import load_base_config, project_path
from trn.ins.strapdown import ImuErrorParams, run_ins
from trn.trajectory.fixed_wing import generate
from trn.truth.truth_map import TruthMap

GRADE_COL = {"navigation": "#2ca02c", "tactical": "#1f77b4", "mems": "#d62728"}


def analytic_rms(t: np.ndarray, p: ImuErrorParams, ie: dict, V: float, g: float, R: float) -> np.ndarray:
    """Horizontal RMS error for constant biases + initial errors (single-axis Schuler model, both axes)."""
    ws = math.sqrt(g / R)
    s, c = np.sin(ws * t), np.cos(ws * t)
    var = (ie["horiz_pos_m"] ** 2 + (ie["vel_mps"] * s / ws) ** 2 + (R * p.init_att[0] * (1 - c)) ** 2
           + (p.accel_bias * (1 - c) / ws ** 2) ** 2 + (R * p.gyro_bias * (t - s / ws)) ** 2)
    var_heading = 0.5 * ((V * p.init_att[2] * t) ** 2 + (0.5 * V * p.gyro_bias * t ** 2) ** 2)
    return np.sqrt(2 * var + var_heading)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=20)
    a = ap.parse_args()
    setup()
    cfg = load_base_config()
    out = project_path(cfg["data"]["results_dir"]) / "m3_ins"
    out.mkdir(parents=True, exist_ok=True)
    route = "A_mountain_crossing"
    tm = TruthMap.load(cfg, route)
    tr = generate(cfg, route, tm.top)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2), constrained_layout=True)
    rep = {}
    for grade in ("navigation", "tactical", "mems"):
        errs, verr = [], []
        for i in range(a.runs):
            ins = run_ins(tr, cfg["imu"], 1.0, np.random.default_rng(1000 + i), grade=grade)
            e = ins.pos - tr.pos[ins.idx]
            errs.append(np.hypot(e[:, 0], e[:, 1])); verr.append(e[:, 2])
        E = np.array(errs); t = tr.t[ins.idx]
        rms = np.sqrt(np.mean(E ** 2, axis=0))
        p = ImuErrorParams.from_config(cfg["imu"], grade)
        an = analytic_rms(t, p, cfg["imu"]["init_errors"], cfg["trajectory"]["speed_mps"], 9.79, cfg["imu"]["earth_radius_m"])
        c = GRADE_COL[grade]
        ax[0].plot(t / 60, rms / 1000, color=c, lw=2, label=f"{grade}: simulated RMS ({a.runs} runs)")
        ax[0].plot(t / 60, an / 1000, color=c, lw=1, ls="--", label=f"{grade}: analytic Schuler model")
        ax[1].plot(t / 60, np.sqrt(np.mean(np.array(verr) ** 2, axis=0)) / 1000, color=c, lw=2, label=grade)
        rep[grade] = dict(rms_horiz_km_at_10min=float(rms[t >= 600][0] / 1000), rms_horiz_km_at_end=float(rms[-1] / 1000),
                          analytic_km_at_end=float(an[-1] / 1000),
                          drift_rate_nmi_per_h=float(rms[-1] / 1852 / (t[-1] / 3600)),
                          rms_vert_km_at_end=float(np.sqrt(np.mean(np.array(verr)[:, -1] ** 2)) / 1000))
    ax[0].set_yscale("log"); ax[0].set_xlabel("time since GNSS loss [min]"); ax[0].set_ylabel("horizontal error [km]")
    ax[0].set_title("Unaided INS horizontal drift (route A trajectory)"); ax[0].legend(fontsize=7)
    ax[1].set_yscale("log"); ax[1].set_xlabel("time since GNSS loss [min]"); ax[1].set_ylabel("vertical RMS error [km]")
    ax[1].set_title("Unaided vertical channel (unstable; barometer needed)"); ax[1].legend(fontsize=7)
    fig.savefig(out / "m3_ins_drift.png")
    (out / "report.json").write_text(json.dumps(rep, indent=1))
    shutil.copy(out / "m3_ins_drift.png", project_path("docs/figures") / "m3_ins_drift.png")
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()

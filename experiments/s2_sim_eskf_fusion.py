#!/usr/bin/env python3
"""S2: Alessandro's ESKF on the simulated Wufeng flight, alone and fused with our camera-to-map fixes.

Same recording, same GNSS cut and same scoring as experiments/s1_sim_map_fix.py (SimDemoMinimal):
  recordings/ilhan_wufeng_south_80m, GNSS until the true distance flown reaches 450 m, scoring after the cut.

Estimator: vio/estimation/eskf.py (identical to TaipeiDrift-ro-ale-simulation/vio/estimation/eskf.py), used as is.
The offline driver below reproduces his live sim adapter (sim/nodes/eskf_ros_adapter.py on ale-simulation):
  - start: settled IMU window (0.5 s, gravity -> roll/pitch, yaw = 0 ENU, mean accel/gyro -> initial biases),
    position from the latest GNSS fix, height 0 (take-off pad); ImuNoiseModel from vio/configs/midair_eskf.yaml
  - IMU 100 Hz prediction (gyro in BODY axes), barometer altitude update every 20 IMU samples with his noise model
  - GNSS 1 Hz until the cut: horizontal position update (gate 0.999) + 5 s finite-difference velocity (his adapter);
    horizontal only, because the recorded gnss.csv stops at t = 6.8 s and the pre-cut GNSS is the SAME
    simulated 1 Hz horizontal series as S1 (truth + N(0, 1.5 m) per axis, load_recording()).
  - after the cut: down-camera flow velocity (his Mid-Air design, eskf_runner._flow_update with the parameters of
    vio/configs/midair_eskf.yaml down_camera), height above ground learned BEFORE the cut from flow + the
    GNSS-aided ESKF velocity/attitude, then carried by the barometer.
Configurations:
  A   ESKF alone after the cut (IMU + baro + down-camera flow)
  B   A + our camera-to-map fixes at 1 Hz (every 5th recorded image), fix covariance = S1's pre-cut calibration
      (outputs/s1_sim/calibration.json fix_sd_m), his 99 % Mahalanobis gate. Fixes are computed LIVE with the
      ESKF's own prior (window centre), sigma (window size), heading and altitude via s1_sim_map_fix.fix_at.
  (No exploratory variant: the brief allowed one only if B rejected most correct fixes; B accepted 506 of 507.)
  C   B, but the matcher gets the ESKF's FULL attitude (roll, pitch, heading) and height: nothing from truth is used
      after the cut (in B roll/pitch come from the truth quaternion, S1's AHRS stand-in).
Randomness: the ESKF and the fixes are deterministic given the inputs; the only random input is the SIMULATED pre-cut
GNSS noise. Seed 0 = S1's series exactly (load_recording); seeds 1-4 redraw N(0, 1.5 m) per axis at the same times.

Frames (explicit):
  world  = ENU metres from the recording / route origin (truth.csv e_m, n_m, u_m); his ESKF is frame-agnostic
           given gravity, so gravity_world = (0, 0, -9.80665) (his sim adapter does the same; his Mid-Air runner
           uses NED with gravity (0, 0, +g)). Flow plane normal "world down" = (0, 0, -1) here, (0, 0, +1) in NED.
  body   = sensor_link FLU (imu.csv), ESKF gyroscope_frame = "body".
  camera = OpenCV optical (x right, y down, z forward) of the down camera; R_bc = gazebo_down_optical_to_flu()
           = [[0,-1,0],[-1,0,0],[0,0,-1]] (meta.json T_body_cam; x_cv = body right, y_cv = body back, z_cv = down).
  heading passed to fix_at = compass bearing of body +x, clockwise from north = atan2(R[0,0], R[1,0]).
Truth is used only for scoring (and, inside fix_at, for roll/pitch as an AHRS stand-in, as in S1).

  PY=/Users/ilhan.neuville/dev/hackathon/TaipeiDrift/.venv/bin/python
  $PY experiments/s2_sim_eskf_fusion.py run --config A B --seeds 0 1 2 3 4 --workers 3 [--camera realistic]
  # another flight: S1 must have calibrated it first (s1_sim_map_fix.py calibrate --recording R --route J --out S)
  $PY experiments/s2_sim_eskf_fusion.py run --recording R --route J --s1-out S --out O --config A B --seeds 0
  $PY experiments/s2_sim_report.py
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pickle
import sys
import time
from dataclasses import replace
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "experiments"), str(ROOT / "baseline"), str(ROOT)]

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import yaml  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from scipy.stats import chi2  # noqa: E402

from vio.eskf_pipeline import noise_model  # noqa: E402
from vio.estimation.eskf import ESKF, N_ERR, P_, V_  # noqa: E402
from vio.pipeline import tracker_config  # noqa: E402
from vio.vision.feature_tracker import KeyframeTracker  # noqa: E402
from vio.vision.optical_flow import FlowConfig, camera_velocity_from_flow, flow_pairs_from_tracks, \
    height_from_known_velocity  # noqa: E402

OUT = ROOT / "outputs/s2_sim"                         # --out
RUNS = OUT / "runs"
REC = ROOT / "recordings/ilhan_wufeng_south_80m"      # --recording
ROUTE = ROOT / "sim/scenarios/wufeng_south_80m.json"  # --route
S1_OUT = ROOT / "outputs/s1_sim"                      # --s1-out: S1's calibration[_<camera>].json for this flight
CFG = yaml.safe_load((ROOT / "vio/configs/midair_eskf.yaml").read_text())
VIO_CFG = yaml.safe_load((ROOT / CFG["vio_config"]).read_text())
G = 9.80665
GRAVITY_ENU = np.array([0.0, 0.0, -G])
UP = np.array([0.0, 0.0, 1.0])
DOWN_ENU = np.array([0.0, 0.0, -1.0])
R_BC_DOWN = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # gazebo_down_optical_to_flu()
GNSS_SD = 1.5            # m per horizontal axis, S1's simulated GNSS (sim_gnss.fallback_sigma_m[0])
VEL_BASELINE_S = 5.0     # GNSS finite-difference velocity baseline: ale-simulation midair_eskf.yaml sim_gnss
FIX_GATE = 0.99          # his gate for camera updates
FIX_EVERY = 5            # recorded images (5 Hz) -> 1 Hz, as S1
HEIGHT_LEARN_S = 10.0    # his 2 s at 25 Hz (50 pairs) -> 10 s at 5 Hz (50 pairs)
WRONG_M = 10.0           # a fix farther than this from truth is "wrong" (S1 convention)
CONFIGS = {"A": dict(fixes=False), "B": dict(fixes=True, eskf_tilt=False), "C": dict(fixes=True, eskf_tilt=True)}
LABELS = {"A": "A: Alessandro ESKF alone", "B": "B: ESKF + our map fixes (his 99 % gate)",
          "C": "C: as B, matcher attitude + height from the ESKF (no truth after the cut)"}


def s1():
    import s1_sim_map_fix as S1
    return S1


def _resolve(p) -> Path:
    """Absolute path; relative paths are taken from the current folder, else from the repository root (as S1)."""
    q = Path(p).expanduser()
    return q.resolve() if q.is_absolute() or q.exists() else ROOT / q


def configure(out, recording, route, s1_out) -> None:
    """Set the flight and folders, here and in S1 (fix_at reads S1.OUT for the calibration). Called in every
    process: spawned workers do not inherit globals."""
    global OUT, RUNS, REC, ROUTE, S1_OUT
    OUT, REC, ROUTE, S1_OUT = _resolve(out), _resolve(recording), _resolve(route), _resolve(s1_out)
    RUNS = OUT / "runs"
    s1().OUT = S1_OUT


# ----------------------------------------------------------------------------- data

def load_inputs():
    S1 = s1()
    rec = S1.load_recording(str(REC), str(ROUTE))
    d = rec.path
    imu = pd.read_csv(d / "imu.csv")
    baro = pd.read_csv(d / "baro.csv")
    truth = pd.read_csv(d / "truth.csv")
    return rec, imu, baro, truth


def flow_pairs(rec, camera: str):
    """Consecutive down-camera frame pairs (his tracker, max_keyframe_age = 1), cached. Index = later image.
    Tracked from the flight start (rec.first): the realistic camera is defined from there; only pairs from 10 s
    before the cut onwards are used, and each pair is detected afresh on its first frame (age 1)."""
    S1 = s1()
    path = OUT / f"cache/flow_pairs_{rec.path.name}_{camera}.pkl"
    if path.exists():
        return pickle.loads(path.read_bytes())
    tr_cfg = replace(tracker_config(VIO_CFG), max_keyframe_age=1)
    tracker = KeyframeTracker(tr_cfg)
    tracks = []
    for j in range(rec.first, rec.last + 1):
        tracks.append(tracker.process(S1.load_image(rec, j, camera), j))
    K = np.array([[rec.cam["fx_px"], 0, rec.cam["cx_px"]], [0, rec.cam["fy_px"], rec.cam["cy_px"]], [0, 0, 1.0]])
    dc = CFG["down_camera"]
    fcfg = FlowConfig(min_tracks=dc["min_tracks"], min_inlier_ratio=dc["min_inlier_ratio"],
                      max_residual_px=dc["max_residual_px"])
    pairs = {p.frame_index: p for p in flow_pairs_from_tracks(tracks, K, lambda i: i, fcfg)}  # imu idx = image idx
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(pickle.dumps(pairs))
    return pairs


def baro_altitude(baro: pd.DataFrame):
    """Pressure -> altitude change (m) with his constant: 8434.5 ln(p0 / p), p0 = first sample."""
    p = baro.pressure_pa.to_numpy(float)
    return baro.t_s.to_numpy(float), 8434.5 * np.log(p[0] / np.maximum(p, 1.0))


def compass(R: Rotation) -> float:
    m = R.as_matrix()
    return float(math.degrees(math.atan2(m[0, 0], m[1, 0])) % 360.0)


# ----------------------------------------------------------------------------- filter run

def simulated_gnss(rec, truth: pd.DataFrame, seed: int):
    """Pre-cut 1 Hz horizontal GNSS. Seed 0: S1's series; other seeds: truth + N(0, GNSS_SD) at the same times."""
    if seed == 0:
        return rec.gnss_t, rec.gnss_e, rec.gnss_n
    rng = np.random.default_rng(1000 + seed)
    tt = truth.t_s.to_numpy(float)
    e = np.interp(rec.gnss_t, tt, truth.e_m) + rng.normal(0, GNSS_SD, len(rec.gnss_t))
    n = np.interp(rec.gnss_t, tt, truth.n_m) + rng.normal(0, GNSS_SD, len(rec.gnss_t))
    return rec.gnss_t, e, n


def run(config: str, seed: int = 0, camera: str = "ideal") -> dict:
    out_dir = RUNS / camera
    S1 = s1()
    rec, imu, baro, truth = load_inputs()
    spec = CONFIGS[config]
    pairs = flow_pairs(rec, camera)
    noise = noise_model(CFG)
    dc, bc, gc = CFG["down_camera"], CFG["barometer"], CFG["sim_gnss"]
    fcfg = FlowConfig(min_tracks=dc["min_tracks"], min_inlier_ratio=dc["min_inlier_ratio"],
                      max_residual_px=dc["max_residual_px"])
    focal = float(rec.cam["fx_px"])

    t = imu.t_s.to_numpy(float)
    acc = imu[["ax", "ay", "az"]].to_numpy(float)
    gyr = imu[["gx", "gy", "gz"]].to_numpy(float)
    bt, balt = baro_altitude(baro)
    t_cut = float(rec.t[rec.cut])
    gt, ge, gn = simulated_gnss(rec, truth, seed)          # pre-cut only (<= t_cut)
    tt = truth.t_s.to_numpy(float)
    tru = truth[["e_m", "n_m", "u_m"]].to_numpy(float)

    # --- initialisation (his try_initialize_from_measurements)
    n_init = int(gc["attitude_init_samples"])
    k = None
    for kk in range(n_init, len(t)):
        if t[kk] < 1.0 or gt[0] > t[kk]:
            continue
        a, w = acc[kk - n_init:kk], gyr[kk - n_init:kk]
        ma = a.mean(0)
        if (np.std(np.linalg.norm(a, axis=1)) <= 0.25 and 9.0 <= np.linalg.norm(ma) <= 10.6
                and np.linalg.norm(w.mean(0)) <= 0.05):
            k = kk
            break
    up_b = ma / np.linalg.norm(ma)
    cross = np.cross(up_b, UP)
    ang = math.acos(float(np.clip(up_b @ UP, -1, 1)))
    align = Rotation.from_rotvec(cross / max(np.linalg.norm(cross), 1e-12) * ang)
    q = align.as_quat()
    ba0 = ma - align.inv().apply([0, 0, G])
    bg0 = w.mean(0)
    ig = int(np.searchsorted(gt, t[k], side="right") - 1)
    f = ESKF([ge[ig], gn[ig], 0.0], [0, 0, 0], [q[3], q[0], q[1], q[2]], GRAVITY_ENU, "body", noise, ba0=ba0, bg0=bg0)
    t0 = float(t[k])
    gnss_next = ig + 1
    anchor = (gt[ig], np.array([ge[ig], gn[ig]]))
    baro0_alt = baro_ref_z = None
    img_t = rec.t
    j = int(np.searchsorted(img_t, t[k], side="right"))
    img_state = {}                      # image index -> (R, v) at the image time (estimator, for flow/height)
    h0 = baro_ref = None
    height_info = {}
    rows, fixes, gnss_nis, flow_log = [], [], [], []
    fix_calls_s = 0.0

    def cov_xy():
        return f.P[:2, :2]

    def truth_at(tq):
        return np.array([np.interp(tq, tt, tru[:, i]) for i in range(3)])

    def baro_at(tq):
        return float(np.interp(tq, bt, balt))

    for kk in range(k + 1, len(t)):
        dt = t[kk] - t[kk - 1]
        if dt <= 0:
            continue
        f.predict(acc[kk - 1], acc[kk], gyr[kk - 1], gyr[kk], dt)
        tk = t[kk]
        # barometer, every 20 IMU samples (his update_baro)
        if (kk - k) % int(bc["every_n_samples"]) == 0:
            alt = baro_at(tk)
            if baro0_alt is None:
                baro0_alt, baro_ref_z = alt, float(f.p[2])
            dz = alt - baro0_alt
            el = max(0.0, tk - t0)
            sig = math.sqrt(bc["white_std_m"] ** 2 + bc["bias_walk_m_per_sqrt_s"] ** 2 * el
                            + (bc["drift_sigma_m_per_s"] * el) ** 2 + (bc["scale_error_rms"] * dz) ** 2)
            f.update_altitude(baro_ref_z + dz, UP, sig, CFG["forward_camera"]["gate_prob"])
        # GNSS until the cut: horizontal position + 5 s finite-difference velocity (his on_gnss, horizontal)
        while gnss_next < len(gt) and gt[gnss_next] <= tk:
            z = np.array([ge[gnss_next], gn[gnss_next]])
            C = np.eye(2) * GNSS_SD ** 2
            H = np.zeros((2, f.n))
            H[:, 0:2] = np.eye(2)
            u = f.update(z - f.p[:2], H, C, gc["gate_prob"])
            gnss_nis.append(dict(t_s=float(gt[gnss_next]), nis=u.nis, accepted=u.accepted))
            dtg = gt[gnss_next] - anchor[0]
            if dtg >= VEL_BASELINE_S:
                Hv = np.zeros((2, f.n))
                Hv[:, 3:5] = np.eye(2)
                vz = (z - anchor[1]) / dtg
                f.update(vz - f.v[:2], Hv, 2 * C / dtg ** 2, gc["gate_prob"])
                anchor = (gt[gnss_next], z)
            gnss_next += 1
        # images
        while j < len(img_t) and img_t[j] <= tk:
            tj = img_t[j]
            after = j >= rec.cut
            if after and h0 is None:
                h0, baro_ref, height_info = learn_height(rec, pairs, img_state, j, fcfg, bt, balt)
            # flow update after the cut (his _flow_update; plane normal = ENU down)
            if after and j - 1 in img_state and j in pairs and h0 is not None:
                flow_log.append(flow_update(f, pairs[j], img_state[j - 1][0], img_t[j] - img_t[j - 1],
                                            h0 + baro_at(tj) - baro_ref, focal, fcfg, dc))
            # camera-to-map fix (B, C)
            if spec["fixes"] and after and (j - rec.cut) % FIX_EVERY == 0 and j <= rec.last:
                Pxy = cov_xy()
                sig = float(math.sqrt(max(np.linalg.eigvalsh(Pxy).max(), 0.0)))
                tr = truth_at(tj)
                prior = f.p[:2].copy()
                c0 = time.perf_counter()
                fx = S1.fix_at(j, prior, sig, compass(f.R), float(f.p[2]), recording=str(REC), route=str(ROUTE),
                               cam_kind=camera, R_body_to_enu=f.R.as_matrix() if spec["eskf_tilt"] else None)
                fix_calls_s += time.perf_counter() - c0
                row = dict(image=j, t_s=tj, dist_since_cut_m=float(rec.travelled[j] - rec.travelled[rec.cut]),
                           prior_err_m=float(np.hypot(*(prior - tr[:2]))), prior_sigma_m=sig,
                           tilt_err_deg=float(np.degrees(np.arccos(np.clip(
                               f.R.as_matrix()[:, 2] @ S1.r_enu_body(rec.q[j])[:, 2], -1.0, 1.0)))),
                           heading_err_deg=float(S1.wrap180(compass(f.R) - S1.heading_of(S1.r_enu_body(rec.q[j])))))
                if fx is None:
                    row.update(status="nofix")
                else:
                    e, n_, Cf, diag = fx
                    zf = np.array([e, n_])
                    H = np.zeros((2, f.n))
                    H[:, 0:2] = np.eye(2)
                    u = f.update(zf - f.p[:2], H, np.asarray(Cf), FIX_GATE)
                    ferr = float(np.hypot(*(zf - tr[:2])))
                    row.update(status="accepted" if u.accepted else "rejected", nis=u.nis, z_e=e, z_n=n_,
                               fix_err_m=ferr, wrong=ferr > WRONG_M, fix_sd_m=float(math.sqrt(Cf[0][0])),
                               half_m=diag.get("half_m"))
                fixes.append(row)
            Rj = f.R
            img_state[j] = (Rj, f.v.copy())
            if j >= rec.first:
                tr = truth_at(tj)
                err = f.p[:2] - tr[:2]
                Pxy = cov_xy()
                rows.append(dict(image=j, t_s=tj, after_cut=after, travelled_m=float(rec.travelled[j]),
                                 dist_since_cut_m=float(rec.travelled[j] - rec.travelled[rec.cut]),
                                 est_e=f.p[0], est_n=f.p[1], est_u=f.p[2], true_e=tr[0], true_n=tr[1], true_u=tr[2],
                                 err_h_m=float(np.hypot(*err)), err_u_m=float(f.p[2] - tr[2]),
                                 sigma_h_m=float(math.sqrt(np.trace(Pxy))),
                                 nees_h=float(err @ np.linalg.solve(Pxy, err)),
                                 heading_err_deg=float(S1.wrap180(compass(Rj) - S1.heading_of(S1.r_enu_body(rec.q[j]))))))
            j += 1
            if j > rec.last:
                break
        if j > rec.last:
            break

    df = pd.DataFrame(rows)
    fx = pd.DataFrame(fixes)
    fl = pd.DataFrame(flow_log)
    out_dir.mkdir(parents=True, exist_ok=True)
    name = f"{config}_s{seed}"
    df.to_csv(out_dir / f"{name}_track.csv", index=False)
    fx.to_csv(out_dir / f"{name}_fixes.csv", index=False)
    fl.to_csv(out_dir / f"{name}_flow.csv", index=False)
    pd.DataFrame(gnss_nis).to_csv(out_dir / f"{name}_gnss_nis.csv", index=False)
    summ = summarize(df, fx, fl, pd.DataFrame(gnss_nis), t_cut)
    summ.update(config=config, seed=seed, camera=camera, label=LABELS[config], height=height_info,
                fix_calls_s=fix_calls_s, init_t_s=t0, cut_t_s=t_cut, cut_image=int(rec.cut),
                gnss_source=rec.gnss_source if seed == 0 else f"SIMULATED 1 Hz, truth + N(0, {GNSS_SD} m), seed {seed}")
    (out_dir / f"{name}_summary.json").write_text(json.dumps(summ, indent=1, default=float))
    return summ


def learn_height(rec, pairs, img_state, j_cut, fcfg, bt, balt):
    """Height above ground at the cut from pre-cut flow and the GNSS-aided ESKF velocity/attitude (no truth)."""
    t_cut = rec.t[j_cut]
    hs, used = [], []
    for jj in range(j_cut - 1, 0, -1):
        if rec.t[jj] < t_cut - HEIGHT_LEARN_S:
            break
        p = pairs.get(jj)
        if p is None or not p.valid or jj - 1 not in img_state or jj not in img_state:
            continue
        (Ra, va), (Rb, vb) = img_state[jj - 1], img_state[jj]
        Ra_m, Rb_m = Ra.as_matrix(), Rb.as_matrix()
        R_ab = R_BC_DOWN.T @ (Ra_m.T @ Rb_m) @ R_BC_DOWN
        n_cam = (Ra_m @ R_BC_DOWN).T @ DOWN_ENU
        dtj = rec.t[jj] - rec.t[jj - 1]
        t_cam = (Ra_m @ R_BC_DOWN).T @ (0.5 * (va + vb)) * dtj
        h = height_from_known_velocity(p.xa, p.xb, R_ab, n_cam, t_cam, fcfg)
        if h is not None and 1.0 < h < 500.0:
            hs.append(h)
            used.append(jj)
    if not hs:
        raise RuntimeError("could not learn the height above ground before the cut")
    h0 = float(np.median(hs))
    ref = float(np.mean([np.interp(rec.t[jj], bt, balt) for jj in used]))
    return h0, ref, dict(height_learned_m=h0, pairs=len(hs), spread_iqr_m=float(np.subtract(*np.percentile(hs, [75, 25]))),
                         true_height_at_cut_m=float(rec.u[j_cut]))


def flow_update(f: ESKF, pair, Ra: Rotation, dt: float, h: float, focal: float, fcfg: FlowConfig, dc: dict) -> dict:
    """eskf_runner._flow_update with the ENU plane normal."""
    if not pair.valid:
        return dict(accepted=False, nis=np.nan, reason=pair.reason)
    Ra_m, Rb_m = Ra.as_matrix(), f.R.as_matrix()
    R_ab = R_BC_DOWN.T @ (Ra_m.T @ Rb_m) @ R_BC_DOWN
    n_cam = (Ra_m @ R_BC_DOWN).T @ DOWN_ENU
    fv = camera_velocity_from_flow(pair.xa, pair.xb, R_ab, n_cam, h, dt, focal, fcfg)
    if fv is None:
        return dict(accepted=False, nis=np.nan, reason="flow fit rejected")
    z = fv.v_cam[:2]
    speed = max(float(np.linalg.norm(z)), float(np.linalg.norm((R_BC_DOWN.T @ Rb_m.T @ f.v)[:2])))
    u = z / max(np.linalg.norm(z), 1e-9)
    uu = np.outer(u, u)
    Rm = (fv.cov_xy + np.eye(2) * dc["min_sigma_mps"] ** 2 + (speed * dc["rel_height_std"]) ** 2 * uu
          + (speed * np.deg2rad(dc["dir_sigma_deg"])) ** 2 * (np.eye(2) - uu))
    pred = (R_BC_DOWN.T @ Rb_m.T @ f.v)[:2]
    res = f.update_camera_velocity_xy(z, R_BC_DOWN, Rm, dc["gate_prob"])
    return dict(accepted=res.accepted, nis=res.nis, reason=res.reason, z_x=z[0], z_y=z[1], pred_x=pred[0],
                pred_y=pred[1], h=h)


# ----------------------------------------------------------------------------- scoring

def summarize(df, fx, fl, gn, t_cut) -> dict:
    a = df[df.after_cut]
    e = a.err_h_m.to_numpy()
    out = dict(after_cut=dict(images=len(a), dist_flown_m=float(a.dist_since_cut_m.iloc[-1]),
                              median_m=float(np.median(e)), p90_m=float(np.percentile(e, 90)),
                              max_m=float(e.max()), final_m=float(e[-1]),
                              final_vertical_m=float(a.err_u_m.iloc[-1]),
                              nees_h_median=float(a.nees_h.median()), nees_h_mean=float(a.nees_h.mean()),
                              nees_h_frac_above_99=float((a.nees_h > chi2.ppf(0.99, 2)).mean()),
                              heading_err_abs_median_deg=float(a.heading_err_deg.abs().median()),
                              heading_err_abs_max_deg=float(a.heading_err_deg.abs().max())),
               at_cut_err_m=float(a.err_h_m.iloc[0]))
    if len(gn):
        nis = gn.nis.to_numpy()
        out["gnss_precut"] = dict(n=len(gn), accepted=int(gn.accepted.sum()), nis_mean=float(np.nanmean(nis)),
                                  nis_median=float(np.nanmedian(nis)))
    if len(fl):
        out["flow"] = dict(attempted=len(fl), accepted=int(fl.accepted.sum()),
                           nis_median_accepted=float(fl[fl.accepted].nis.median()) if fl.accepted.any() else None,
                           reasons=fl[~fl.accepted].reason.value_counts().to_dict())
    if len(fx):
        got = fx[fx.status != "nofix"]
        wrong = got.wrong.astype(bool) if len(got) else pd.Series(dtype=bool)
        acc, rej = got.status == "accepted", got.status == "rejected"
        out["fixes"] = dict(attempts=len(fx), nofix=int((fx.status == "nofix").sum()), with_fix=len(got),
                            accepted=int(acc.sum()), rejected=int(rej.sum()),
                            wrong_total=int(wrong.sum()), wrong_accepted=int((wrong & acc).sum()),
                            correct_rejected=int((~wrong & rej).sum()),
                            nis_accepted_median=float(got[acc].nis.median()) if acc.any() else None,
                            nis_accepted_mean=float(got[acc].nis.mean()) if acc.any() else None,
                            nis_rejected=got[rej].nis.round(2).tolist(),
                            fix_err_median_m=float(got.fix_err_m.median()) if len(got) else None,
                            fix_err_p90_m=float(got.fix_err_m.quantile(0.9)) if len(got) else None,
                            fix_err_max_m=float(got.fix_err_m.max()) if len(got) else None)
    return out


def _one(job):
    c, seed, camera, paths = job
    configure(*paths)
    t0 = time.time()
    s = run(c, seed, camera)
    a = s["after_cut"]
    return (f"{camera} {c} s{seed}: median {a['median_m']:.1f} p90 {a['p90_m']:.1f} max {a['max_m']:.1f} final {a['final_m']:.1f} m;"
            f" fixes {s.get('fixes')}; flow {s['flow']['accepted']}/{s['flow']['attempted']};"
            f" height {s['height']}; {time.time() - t0:.0f} s")


def cmd_run(args):
    paths = (str(OUT), str(REC), str(ROUTE), str(S1_OUT))
    jobs = [(c, s, args.camera, paths) for s in args.seeds for c in args.config]
    if args.workers <= 1:
        for j in jobs:
            print(_one(j), flush=True)
        return
    from multiprocessing import get_context
    with get_context("spawn").Pool(args.workers) as pool:
        for line in pool.imap_unordered(_one, jobs):
            print(line, flush=True)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--config", nargs="+", default=["A", "B"])
    r.add_argument("--seeds", nargs="+", type=int, default=[0])
    r.add_argument("--workers", type=int, default=1)
    r.add_argument("--camera", default="ideal", choices=["ideal", "realistic"])
    r.add_argument("--recording", default=str(REC), help="recordings/<name> (taipeidrift-replay/1)")
    r.add_argument("--route", default=str(ROUTE), help="sim/scenarios/<route>.json of that recording")
    r.add_argument("--s1-out", default=str(S1_OUT),
                   help="folder where `s1_sim_map_fix.py calibrate/run --out` wrote calibration[_<camera>].json")
    r.add_argument("--out", default=str(OUT), help="output folder (runs/<camera>/, cache/)")
    r.set_defaults(fn=cmd_run)
    a = ap.parse_args()
    configure(a.out, a.recording, a.route, a.s1_out)
    cal = s1().calib_path(a.camera)
    if not cal.exists():
        raise SystemExit(f"{cal} missing: run s1_sim_map_fix.py calibrate --camera {a.camera} --out {S1_OUT} first")
    a.fn(a)


if __name__ == "__main__":
    main()

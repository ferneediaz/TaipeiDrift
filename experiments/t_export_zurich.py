"""Export a time window of the Zurich Urban MAV dataset (AGZ) to `taipeidrift-replay/1`.

Evidence: MEASURED (real Pixhawk logs, real GoPro images, Pix4D photogrammetric truth).
Source: https://rpg.ifi.uzh.ch/zurichmavdataset.html , archive https://download.ifi.uzh.ch/rpg/AGZ_data/AGZ.zip
Images of a window are pulled with experiments/t_remote_zip.py (HTTP range), not the 28 GB archive.

What each output column comes from (all timestamps: PX4 boot clock, microseconds, one clock):
  imu.csv        OnboardPose.csv Omega_*/Accel_* (50 Hz). Frame: empirically P = A @ raw with
                 A ~ [[-.71,-.71,0],[-.71,.71,0],[0,0,-1]] against RawGyro (fit printed by --check-imu),
                 z up (accel_z ~ +9.4 at rest). Attenuated vs raw (filtered upstream) -> declared.
  imu_raw_10hz.csv  RawGyro/RawAccel x,y,z (10 Hz, raw sensor axes, aliased vibration). Extra file.
  baro.csv       BarometricPressure.csv pressure (hPa -> Pa), temperature; alt_isa_m recomputed.
  gnss.csv       OnboardGPS.csv rows deduplicated (the file repeats the last fix at each image, 30 Hz;
                 distinct fixes ~5 Hz). alt = MSL per readme. eph/epv -> hacc/vacc (epv often garbage).
  images.csv     image time = OnboardGPS timestamp of the row carrying that imgid.
  truth.csv      GroundTruthAGL.csv (Pix4D, UTM 32N, 1 Hz on every 30th image) -> lat/lon, local ENU.
                 Camera orientation omega/phi/kappa kept as extra columns; q left empty (camera->body
                 rotation is not published).

Usage:
  .venv/bin/python experiments/t_export_zurich.py --t0 1800 --t1 2400 \
      --images data/raw/zurich_mav/AGZ_window/AGZ/"MAV Images" --out data/processed/t_replay/zurich_agz_1800_2400
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np
import pandas as pd
from pyproj import Transformer

from t_replay import SCHEMA, isa_altitude_m

LOGS = Path("data/raw/zurich_mav/AGZ_subset/Log Files")
CALIB = Path("data/raw/zurich_mav/AGZ_subset/calibration_data.npz")


def rd(name: str) -> pd.DataFrame:
    d = pd.read_csv(LOGS / name, skipinitialspace=True)
    d = d.loc[:, ~d.columns.str.startswith("Unnamed")]
    d["t_abs"] = d.iloc[:, 0] / 1e6
    return d


def ecef_to_enu(xyz: np.ndarray, lat0: float, lon0: float, xyz0: np.ndarray) -> np.ndarray:
    la, lo = np.radians(lat0), np.radians(lon0)
    R = np.array([[-np.sin(lo), np.cos(lo), 0],
                  [-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)],
                  [np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)]])
    return (xyz - xyz0) @ R.T


def fit_imu_axes(pose: pd.DataFrame, raw_g: pd.DataFrame) -> list[list[float]]:
    t = raw_g.t_abs.to_numpy()
    P = np.column_stack([np.interp(t, pose.t_abs, pose[c]) for c in ("Omega_x", "Omega_y", "Omega_z")])
    X = np.column_stack([raw_g[["x", "y", "z"]].to_numpy(), np.ones(len(t))])
    M = np.linalg.lstsq(X, P, rcond=None)[0][:3].T  # P ~ M @ raw
    return np.round(M, 3).tolist()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--t0", type=float, required=True, help="window start, PX4 clock seconds")
    ap.add_argument("--t1", type=float, required=True)
    ap.add_argument("--images", type=Path, required=True, help="directory with NNNNN.jpg")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cut", type=float, default=60.0, help="suggested GNSS cut, s after window start")
    a = ap.parse_args()
    out = a.out
    out.mkdir(parents=True, exist_ok=True)
    w = lambda d: d[(d.t_abs >= a.t0) & (d.t_abs < a.t1)].copy()  # noqa: E731

    pose, rg, ra = rd("OnboardPose.csv"), rd("RawGyro.csv"), rd("RawAccel.csv")
    imu_axes = fit_imu_axes(pose, rg)
    p = w(pose)
    imu = pd.DataFrame({"t_s": p.t_abs - a.t0,
                        "gx": p.Omega_x, "gy": p.Omega_y, "gz": p.Omega_z,
                        "ax": p.Accel_x, "ay": p.Accel_y, "az": p.Accel_z})
    imu.to_csv(out / "imu.csv", index=False, float_format="%.6f")

    raw = pd.merge_asof(w(rg).sort_values("t_abs"), w(ra).sort_values("t_abs")[["t_abs", "x", "y", "z"]],
                        on="t_abs", suffixes=("_g", "_a"), direction="nearest", tolerance=0.02)
    pd.DataFrame({"t_s": raw.t_abs - a.t0, "gx": raw.x_g, "gy": raw.y_g, "gz": raw.z_g,
                  "ax": raw.x_a, "ay": raw.y_a, "az": raw.z_a}).to_csv(
        out / "imu_raw_10hz.csv", index=False, float_format="%.6f")

    b = w(rd("BarometricPressure.csv"))
    pa = b.Pressure.to_numpy() * 100.0
    pd.DataFrame({"t_s": b.t_abs - a.t0, "pressure_pa": pa, "temperature_c": b.Temperature,
                  "alt_isa_m": isa_altitude_m(pa)}).to_csv(out / "baro.csv", index=False, float_format="%.4f")

    gps_all = rd("OnboardGPS.csv")
    key = ["lat", "lon", "alt", "vel_n_m_s", "vel_e_m_s", "vel_d_m_s"]
    g = w(gps_all)
    g = g[(g[key].diff().abs().sum(axis=1) > 0) | (np.arange(len(g)) == 0)]
    epv = g.epv_m.where(g.epv_m > 1e-3)
    gnss = pd.DataFrame({"t_s": g.t_abs - a.t0, "lat_deg": g.lat, "lon_deg": g.lon, "alt_m": g.alt,
                         "fix_type": g.fix_type, "hacc_m": g.eph_m, "vacc_m": epv,
                         "ve_mps": g.vel_e_m_s, "vn_mps": g.vel_n_m_s, "vu_mps": -g.vel_d_m_s,
                         "nsat": g.num_sat})
    gnss.to_csv(out / "gnss.csv", index=False, float_format="%.8f")

    img_t = gps_all.groupby("imgid").t_abs.first()
    gi = img_t[(img_t >= a.t0) & (img_t < a.t1)]
    rel = os.path.relpath(a.images, out)
    names = [f"{i:05d}.jpg" for i in gi.index]
    present = np.array([(a.images / n).exists() for n in names])
    images = pd.DataFrame({"t_s": gi.to_numpy() - a.t0, "cam": "cam0",
                           "path": [f"{rel}/{n}" for n in names]})[present]
    images.to_csv(out / "images.csv", index=False, float_format="%.6f")

    gt = pd.read_csv(LOGS / "GroundTruthAGL.csv", skipinitialspace=True)
    gt = gt.loc[:, ~gt.columns.str.startswith("Unnamed")]
    gt["t_abs"] = gt.imgid.map(img_t)
    gt = w(gt.dropna(subset=["t_abs"])).sort_values("t_abs")
    lon, lat = Transformer.from_crs(32632, 4326, always_xy=True).transform(gt.x_gt.to_numpy(), gt.y_gt.to_numpy())
    to_ecef = Transformer.from_crs(4979, 4978, always_xy=True)
    xyz = np.column_stack(to_ecef.transform(lon, lat, gt.z_gt.to_numpy()))
    lat0, lon0, h0 = float(lat[0]), float(lon[0]), float(gt.z_gt.iloc[0])
    enu = ecef_to_enu(xyz, lat0, lon0, xyz[0])
    truth = pd.DataFrame({"t_s": gt.t_abs - a.t0, "e_m": enu[:, 0], "n_m": enu[:, 1], "u_m": enu[:, 2],
                          "lat_deg": lat, "lon_deg": lon, "alt_m": gt.z_gt,
                          "qw": np.nan, "qx": np.nan, "qy": np.nan, "qz": np.nan,
                          "cam_omega_deg": gt.omega_gt, "cam_phi_deg": gt.phi_gt, "cam_kappa_deg": gt.kappa_gt,
                          "imgid": gt.imgid})
    truth.to_csv(out / "truth.csv", index=False, float_format="%.6f")

    cal = np.load(CALIB)
    span = lambda d: float(d.t_s.max() - d.t_s.min())  # noqa: E731
    rate = lambda d: round((len(d) - 1) / span(d), 2) if len(d) > 1 else None  # noqa: E731
    meta = {
        "schema": SCHEMA,
        "sequence": out.name,
        "evidence_label": "MEASURED",
        "source": "Zurich Urban MAV dataset (AGZ), Majdik, Till, Scaramuzza, IJRR 2017",
        "url": "https://rpg.ifi.uzh.ch/zurichmavdataset.html",
        "licence": ("Log files: no restriction stated in readme. MAV images: property of the authors, "
                    "'can be used for academic research without any limitations' (AGZ readme.txt)."),
        "window_px4_clock_s": [a.t0, a.t1],
        "gnss_cut_s": a.cut,
        "origin": {"lat_deg": lat0, "lon_deg": lon0, "alt_m": h0,
                   "note": "first truth sample of the window; ENU axes at that point"},
        "platform": "Fotokite-type tethered quadrotor, Pixhawk autopilot, GoPro Hero 4, Zurich streets",
        "sensors": {
            "imu": {"rate_hz": rate(imu), "file": "imu.csv",
                    "provenance": "OnboardPose.csv Omega/Accel as logged by PX4 (50 Hz)",
                    "frame": "body frame of OnboardPose, z up at rest; raw->this frame fit (rows=out axes)",
                    "raw_to_frame_fit": imu_axes,
                    "caveat": "amplitude ~0.65x raw on x/y gyro: low-pass filtered upstream (INFERENCE)",
                    "extra_file": "imu_raw_10hz.csv (raw sensor axes, 10 Hz, aliased)"},
            "baro": {"rate_hz": rate(b.assign(t_s=b.t_abs)), "file": "baro.csv",
                     "altitude_semantics": "raw pressure from Pixhawk barometer (MS5611); alt_isa_m recomputed "
                                           "from pressure with ISA p0=101325 Pa (dataset's own altitude column "
                                           "is the same function within 3 mm, see p_zurich_baro)",
                     "provenance": "BarometricPressure.csv"},
            "gnss": {"rate_hz": rate(gnss), "alt_ref": "msl (per readme)", "file": "gnss.csv",
                     "provenance": "OnboardGPS.csv, consecutive duplicate rows removed",
                     "caveat": "hacc_m/vacc_m are PX4 eph/epv; epv frequently invalid -> empty"},
            "camera": {"cams": {"cam0": {
                "rate_hz": rate(images), "width": 1920, "height": 1080,
                "K": cal["intrinsic_matrix"].round(4).tolist(), "dist_opencv": cal["distCoeff"].ravel().round(6).tolist(),
                "direction": "forward/side-looking at street level (NOT nadir)",
                "T_body_cam": None,
                "provenance": "GoPro Hero 4 frames; time = OnboardGPS row of that imgid"}}},
            "truth": {"source": "Pix4D photogrammetric camera poses (GroundTruthAGL.csv), UTM 32N",
                      "rate_hz": rate(truth), "evaluator_only": True,
                      "accuracy": "not published; median 3.1 m horizontal offset to onboard GPS over the flight",
                      "vertical_datum": "not documented (values close to GPS MSL)"},
        },
        "counts": {"imu": len(imu), "baro": len(b), "gnss": len(gnss), "images": int(present.sum()),
                   "images_missing": int((~present).sum()), "truth": len(truth)},
        "generator": "experiments/t_export_zurich.py",
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta["counts"]), "imu axes fit", imu_axes)


if __name__ == "__main__":
    main()

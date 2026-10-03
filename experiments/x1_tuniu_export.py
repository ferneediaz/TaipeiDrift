#!/usr/bin/env python3
"""Stage 0: export the Tuniu River Phantom 4 RTK survey (2019-04-11) to taipeidrift-replay/1.

Reads the photos in place (symlink folder), the MRK RTK log and the DJI XMP of every photo.

Writes data/processed/t_replay/tuniu_tw_1/:
  images.csv     photo times (MRK GPS time, 0 = first photo) and paths via photos/ -> Downloads
  truth.csv      EVALUATOR ONLY: RTK antenna position (MRK), ENU at the first photo
  gnss.csv       RTK positions of the PRE-CUT phase only (first two survey legs)
  attitude.csv   DJI fused attitude (gimbal + flight angles from XMP); no DJI speed / altitude
  meta.json      provenance, camera intrinsics + distortion (DJI DewarpData), cut time
and data/processed/x_tuniu/stage0_protocol.json (legs, cut, test photos).

Legs are segmented on the RTK track only to define the protocol cut (first two straight legs);
nothing derived from RTK after the cut is written to an estimator-side file.

Run: .venv/bin/python experiments/x1_tuniu_export.py
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image
from pyproj import Transformer

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path("/Users/ilhan.neuville/Downloads/20190411_Miaoli_Toufeng_Tuniu-River_5.75K/100_0005")
OUT = ROOT / "data/processed/t_replay/tuniu_tw_1"
XOUT = ROOT / "data/processed/x_tuniu"
LEG_HEADING_TOL_DEG = 20.0   # a straight leg: track heading within this of the leg's median heading
LEG_MIN_PHOTOS = 8


def xmp_description(path: Path) -> dict[str, str]:
    with Image.open(path) as image:
        try:
            xmp = image.getxmp()["xmpmeta"]["RDF"]["Description"]
        except (AttributeError, KeyError, TypeError) as exc:
            raise ValueError(f"{path}: missing DJI XMP RDF description") from exc
    return {str(k): str(v).strip() for k, v in xmp.items()}


def parse_mrk(path: Path) -> pd.DataFrame:
    """MRK: idx, GPS s of week, [week], N/E/V antenna->CMOS offsets (mm), lat, lon, ellh, std, flag."""
    rows = []
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        f = line.split("\t")
        if len(f) < 10:
            continue
        num = [x.split(",")[0].strip() for x in f]
        try:
            rows.append(dict(idx=int(num[0]), t_gps_s=float(num[1]), week=int(num[2].strip("[]")),
                             off_n_mm=float(num[3]), off_e_mm=float(num[4]), off_v_mm=float(num[5]),
                             lat_deg=float(num[6]), lon_deg=float(num[7]), alt_m=float(num[8]),
                             fix_type=int(num[-1])))
        except ValueError as exc:
            raise ValueError(f"{path}:{line_no}: unparseable MRK row") from exc
    if not rows:
        raise ValueError(f"{path}: no MRK rows")
    df = pd.DataFrame(rows)
    if df.week.nunique() != 1:
        raise ValueError("MRK spans a GPS week rollover; not handled")
    return df


def to_enu(lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
    to_ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
    xyz = np.column_stack(to_ecef.transform(lon, lat, alt))
    lat0, lon0 = math.radians(float(lat[0])), math.radians(float(lon[0]))
    slat, clat, slon, clon = math.sin(lat0), math.cos(lat0), math.sin(lon0), math.cos(lon0)
    d = xyz - xyz[0]
    return np.column_stack((
        -slon * d[:, 0] + clon * d[:, 1],
        -slat * clon * d[:, 0] - slat * slon * d[:, 1] + clat * d[:, 2],
        clat * clon * d[:, 0] + clat * slon * d[:, 1] + slat * d[:, 2],
    ))


def segment_legs(enu: np.ndarray) -> np.ndarray:
    """Leg id per photo (-1 = turn/transit). Heading of the step arriving at each photo, then runs."""
    step = np.diff(enu[:, :2], axis=0)
    hd = np.degrees(np.arctan2(step[:, 0], step[:, 1])) % 360
    hd = np.r_[hd[0], hd]                       # photo 0 takes the heading of the first step
    leg = -np.ones(len(enu), int)
    start, k = 0, 0
    for i in range(1, len(enu) + 1):
        if i == len(enu) or abs((hd[i] - hd[start] + 180) % 360 - 180) > LEG_HEADING_TOL_DEG:
            if i - start >= LEG_MIN_PHOTOS:
                leg[start:i] = k
                k += 1
            start = i
    return leg


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", type=Path, default=SOURCE)
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    src, out = args.source.resolve(), args.out.resolve()
    mrk = parse_mrk(src / "100_0005_Timestamp.MRK")
    photos = sorted(src.glob("100_0005_*.JPG"), key=lambda p: int(p.stem.rsplit("_", 1)[1]))
    if len(photos) != len(mrk) or [int(p.stem.rsplit("_", 1)[1]) for p in photos] != mrk.idx.tolist():
        raise ValueError(f"photo/MRK mismatch: {len(photos)} photos, {len(mrk)} MRK rows")

    t0 = mrk.t_gps_s.iloc[0]
    attitude, image_rows, camera, max_dpos = [], [], None, 0.0
    for photo, row in zip(photos, mrk.itertuples()):
        xmp = xmp_description(photo)
        need = ("GpsLatitude", "GpsLongtitude", "DewarpData", "CalibratedFocalLength",
                "GimbalPitchDegree", "GimbalYawDegree", "GimbalRollDegree",
                "FlightPitchDegree", "FlightYawDegree", "FlightRollDegree")
        missing = [k for k in need if k not in xmp]
        if missing:
            raise ValueError(f"{photo}: missing XMP fields {missing}")
        dlat = (float(xmp["GpsLatitude"]) - row.lat_deg) * 110570
        dlon = (float(xmp["GpsLongtitude"]) - row.lon_deg) * 111320 * math.cos(math.radians(row.lat_deg))
        max_dpos = max(max_dpos, math.hypot(dlat, dlon))
        t = row.t_gps_s - t0
        image_rows.append({"t_s": t, "cam": xmp.get("Model", "FC6310R"), "path": f"photos/{photo.name}"})
        attitude.append({"t_s": t, "frame": row.idx,
                         "gimbal_pitch_deg": float(xmp["GimbalPitchDegree"]),
                         "gimbal_yaw_deg": float(xmp["GimbalYawDegree"]),
                         "gimbal_roll_deg": float(xmp["GimbalRollDegree"]),
                         "flight_pitch_deg": float(xmp["FlightPitchDegree"]),
                         "flight_yaw_deg": float(xmp["FlightYawDegree"]),
                         "flight_roll_deg": float(xmp["FlightRollDegree"])})
        date, coeffs = xmp["DewarpData"].split(";", 1)
        vals = [float(v) for v in coeffs.split(",")]
        if len(vals) != 9:
            raise ValueError(f"unexpected DewarpData {xmp['DewarpData']!r}")
        with Image.open(photo) as im:
            w, h = im.size
        cam = {"model": xmp.get("Model", "FC6310R"), "image_width_px": w, "image_height_px": h,
               "fx_px": vals[0], "fy_px": vals[1],
               "cx_px": w / 2 + vals[2], "cy_px": h / 2 + vals[3],
               "distortion_k1_k2_p1_p2_k3": vals[4:9],
               "distortion_model": "OpenCV radial-tangential (k1,k2,p1,p2,k3) on normalised coordinates; "
                                   "DJI DewarpData = fx,fy,cx_offset,cy_offset,k1,k2,p1,p2,k3, offsets from image centre",
               "dewarp_calibration_date": date.strip(), "dewarp_flag": xmp.get("DewarpFlag"),
               "xmp_calibrated_focal_length_px": float(xmp["CalibratedFocalLength"]),
               "xmp_calibrated_optical_centre_px": [float(xmp["CalibratedOpticalCenterX"]),
                                                    float(xmp["CalibratedOpticalCenterY"])]}
        if camera is None:
            camera = cam
        elif cam != camera:
            raise ValueError(f"{photo}: camera calibration differs from first photo")
    if max_dpos > 0.5:
        raise ValueError(f"XMP vs MRK position differs by {max_dpos:.2f} m")

    lat, lon, alt = (mrk[c].to_numpy(float) for c in ("lat_deg", "lon_deg", "alt_m"))
    enu = to_enu(lat, lon, alt)
    rel_t = mrk.t_gps_s.to_numpy(float) - t0
    leg = segment_legs(enu)
    pre = np.flatnonzero(leg <= 1)
    pre = pre[pre <= np.flatnonzero(leg == 1).max()]
    k_last = int(np.flatnonzero(leg == 1).max())
    cut_s = float((rel_t[k_last] + rel_t[k_last + 1]) / 2)
    pre_mask = rel_t < cut_s

    truth = pd.DataFrame({"t_s": rel_t, "e_m": enu[:, 0], "n_m": enu[:, 1], "u_m": enu[:, 2],
                          "lat_deg": lat, "lon_deg": lon, "alt_m": alt,
                          "qw": np.nan, "qx": np.nan, "qy": np.nan, "qz": np.nan})
    tp, ep = rel_t[pre_mask], enu[pre_mask]
    vel = np.gradient(ep, tp, axis=0, edge_order=2)
    gnss = pd.DataFrame({"t_s": tp, "lat_deg": lat[pre_mask], "lon_deg": lon[pre_mask], "alt_m": alt[pre_mask],
                         "fix_type": mrk.fix_type.to_numpy()[pre_mask], "hacc_m": 0.01, "vacc_m": 0.025,
                         "ve_mps": vel[:, 0], "vn_mps": vel[:, 1], "vu_mps": vel[:, 2], "nsat": np.nan})

    out.mkdir(parents=True, exist_ok=True)
    link = out / "photos"
    if link.is_symlink():
        if link.resolve() != src:
            raise FileExistsError(f"{link} -> {link.resolve()}, expected {src}")
    elif link.exists():
        raise FileExistsError(f"{link} exists and is not a symlink")
    else:
        link.symlink_to(src, target_is_directory=True)
    pd.DataFrame(image_rows).to_csv(out / "images.csv", index=False)
    truth.to_csv(out / "truth.csv", index=False)
    gnss.to_csv(out / "gnss.csv", index=False)
    pd.DataFrame(attitude).to_csv(out / "attitude.csv", index=False)
    meta = {
        "schema": "taipeidrift-replay/1", "sequence": "tuniu_tw_1", "evidence_label": "MEASURED",
        "source": "DJI Phantom 4 RTK survey 100_0005, Tuniu River 5.75K, Toufen, Miaoli, 2019-04-11 "
                  "(photos read in place via photos/ symlink; never copied)",
        "origin": {"lat_deg": float(lat[0]), "lon_deg": float(lon[0]), "alt_m": float(alt[0]),
                   "alt_ref": "WGS84 ellipsoid", "frame": "local ENU tangent plane at the first photo"},
        "gnss_cut_s": cut_s,
        "sensors": {
            "camera": {"file": "images.csv", "label": "MEASURED", "rate_hz": float(1 / np.median(np.diff(rel_t))),
                       "timestamp": "MRK GPS seconds of week minus first photo"},
            "gnss": {"file": "gnss.csv", "label": "MEASURED", "alt_ref": "ellipsoid",
                     "source": "MRK RTK (flag 50 = fixed), PRE-CUT rows only (first two survey legs); "
                               "velocities = finite differences of RTK positions"},
            "truth": {"file": "truth.csv", "label": "MEASURED", "evaluator_only": True,
                      "source": "MRK RTK antenna phase centre (antenna->camera offset <= 0.2 m not applied); "
                                "alt_m ellipsoidal; quaternion not provided"},
            "dji_fused_attitude": {"file": "attitude.csv", "label": "MEASURED (DJI fused attitude)",
                                   "source": "XMP Gimbal{Yaw,Pitch,Roll}Degree (absolute, NED) and Flight*Degree. "
                                             "DJI FlightX/Y/ZSpeed, AbsoluteAltitude, RelativeAltitude deliberately not exported"},
        },
        "camera": camera,
        "licence": "User-provided DJI imagery and metadata; research use only, not redistributed",
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")

    legs = [{"leg": int(k), "first_frame": int(np.flatnonzero(leg == k)[0] + 1),
             "last_frame": int(np.flatnonzero(leg == k)[-1] + 1),
             "heading_deg": float(np.degrees(np.arctan2(*(enu[leg == k][-1, :2] - enu[leg == k][0, :2]))) % 360)}
            for k in range(leg.max() + 1)]
    protocol = {"cut_s": cut_s, "precut_frames": [int(i + 1) for i in np.flatnonzero(pre_mask)],
                "test_frames": [int(i + 1) for i in np.flatnonzero(~pre_mask)],
                "leg_per_frame": leg.tolist(), "legs": legs,
                "note": "legs segmented on the RTK track (protocol definition only)"}
    XOUT.mkdir(parents=True, exist_ok=True)
    (XOUT / "stage0_protocol.json").write_text(json.dumps(protocol, indent=1) + "\n")

    print(f"exported {len(photos)} photos, {len(mrk)} MRK RTK rows, duration {rel_t[-1]:.1f} s; "
          f"max XMP-MRK position diff {max_dpos:.3f} m")
    print(f"straight legs found: {leg.max() + 1}; turn/transit photos: {(leg < 0).sum()}")
    for L in legs[:3]:
        print(f"  leg {L['leg']}: frames {L['first_frame']}-{L['last_frame']} heading {L['heading_deg']:.1f} deg")
    print(f"cut at t={cut_s:.2f} s: pre-cut photos {pre_mask.sum()} (frames 1-{k_last + 1}), "
          f"test photos {(~pre_mask).sum()}")
    print(f"camera {camera['image_width_px']}x{camera['image_height_px']} fx={camera['fx_px']:.2f} "
          f"fy={camera['fy_px']:.2f} c=({camera['cx_px']:.2f},{camera['cy_px']:.2f}) "
          f"dist={camera['distortion_k1_k2_p1_p2_k3']}")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

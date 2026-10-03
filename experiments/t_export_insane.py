"""Export INSANE sequences to `taipeidrift-replay/1`.

The bundled sensor CSV timestamps and nav-camera timestamp values are seconds
on the dataset's ROS clock. RTK GPS1 timestamps are shifted by the dataset's
`t_mag_gps` offset before joining that clock. The exporter keeps only fixed RTK
samples as evaluator-only truth and references the extracted PNGs in place.
"""
from __future__ import annotations

import argparse
import ast
import json
import os
import re
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from t_replay import SCHEMA, isa_altitude_m

ROOT = Path(__file__).resolve().parents[1]
PLOT_DIR = ROOT / "data/processed/t_insane"
DATA_URL = "https://cns-data.aau.at/insane-dataset/"
LICENSE_URL = DATA_URL + "LICENSE.txt"
# Generic replay camera ID; provenance below retains the source nav_cam name.
CAMERA_NAME = "cam0"

SEQUENCES = {
    "mars_1": {
        "sensors_zip": ROOT / "data/raw/insane/mars_1_sensors.zip",
        "nav_root": ROOT / "data/raw/insane/mars_1_images/mars_1_nav_cam",
        "calibration_file": (
            ROOT / "data/raw/insane/mars_1_images/insane_sensor_calib_preprocessed"
            / "camera_calibration_mars.yaml"
        ),
        "calibration_name": "camera_calibration_mars.yaml",
        "out": ROOT / "data/processed/t_replay/insane_mars_1",
        "plot_path": PLOT_DIR / "mars_1_path.png",
        "plot_title": "INSANE Mars 1 fixed RTK path (local ENU)",
        "platform": "UAV flight in the Negev Mars-analog desert; Mars 1 circular trajectory",
        "enu_reference": "Negev reference",
        "reference": "Dataset README reference GPS coordinate for Negev: 30.599929, 34.867308, 526.594",
    },
    "outdoor_1": {
        "sensors_zip": ROOT / "data/raw/insane/outdoor_1_sensors.zip",
        "nav_root": ROOT / "data/raw/insane/outdoor_1_images/outdoor_1_nav_cam",
        "calibration_file": (
            ROOT / "data/raw/insane/mars_1_images/insane_sensor_calib_preprocessed"
            / "camera_calibration_klu1.yaml"
        ),
        "calibration_name": "camera_calibration_klu1.yaml",
        "out": ROOT / "data/processed/t_replay/insane_outdoor_1",
        "plot_path": PLOT_DIR / "outdoor_1_path.png",
        "plot_title": "INSANE outdoor 1 fixed RTK path (local ENU)",
        "platform": "UAV flight at the Klagenfurt model airfield; outdoor_1 square trajectory",
        "enu_reference": "Klagenfurt model-airfield reference",
        "reference": "Dataset README reference GPS coordinate for Klagenfurt: 46.606867, 14.279121, 484.017",
    },
}


def _member(archive: zipfile.ZipFile, basename: str) -> str:
    matches = [name for name in archive.namelist() if name.endswith("/" + basename)]
    if len(matches) != 1:
        raise ValueError(f"Expected one {basename} in {archive.filename}, found {len(matches)}")
    return matches[0]


def _read_csv(archive: zipfile.ZipFile, basename: str) -> pd.DataFrame:
    with archive.open(_member(archive, basename)) as source:
        return pd.read_csv(source, skipinitialspace=True)


def _time_offset_s(archive: zipfile.ZipFile) -> float:
    with archive.open(_member(archive, "time_info.yaml")) as source:
        text = source.read().decode("utf-8")
    match = re.search(r"(?m)^\s*t_mag_gps:\s*([-+\d.eE]+)", text)
    if match is None:
        raise ValueError("time_info.yaml does not contain t_mag_gps")
    return float(match.group(1))


def _clean(frame: pd.DataFrame, time_column: str = "t") -> pd.DataFrame:
    return (
        frame.replace([np.inf, -np.inf], np.nan)
        .dropna(subset=[time_column])
        .sort_values(time_column, kind="stable")
        .drop_duplicates(time_column, keep="first")
        .reset_index(drop=True)
    )


def _rate_hz(times: pd.Series | np.ndarray) -> float | None:
    values = np.asarray(times, dtype=float)
    if values.size < 2 or values[-1] <= values[0]:
        return None
    return round(float((values.size - 1) / (values[-1] - values[0])), 2)


def _yaml_section(text: str, name: str) -> str:
    match = re.search(rf"(?m)^{re.escape(name)}:[ \t]*$", text)
    if match is None:
        raise ValueError(f"Camera calibration is missing section {name}")
    start = match.end()
    next_section = re.search(r"(?m)^[A-Za-z0-9_]+:[ \t]*$", text[start:])
    end = start + next_section.start() if next_section else len(text)
    return text[start:end]


def _yaml_value(section: str, key: str):
    match = re.search(rf"(?m)^[ \t]*{re.escape(key)}:[ \t]*([^\n]*)", section)
    if match is None:
        raise ValueError(f"Camera calibration is missing {key}")
    value = match.group(1).strip()
    if value.startswith("[") and "]" not in value:
        closing = section.find("]", match.end(1))
        if closing == -1:
            raise ValueError(f"Camera calibration list {key} is not closed")
        value = section[match.start(1):closing + 1].strip()
    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError):
        return value


def _yaml_matrix(section: str, key: str) -> list[list[float]]:
    match = re.search(rf"(?m)^[ \t]*{re.escape(key)}:[ \t]*$", section)
    if match is None:
        raise ValueError(f"Camera calibration is missing {key}")
    rows = re.findall(r"(?m)^[ \t]*-[ \t]*(\[[^\n]*\])[ \t]*$", section[match.end():])
    if len(rows) < 4:
        raise ValueError(f"Camera calibration matrix {key} has {len(rows)} rows")
    return [[float(value) for value in row] for row in ast.literal_eval("[" + ",".join(rows[:4]) + "]")]


def _camera_calibration(calibration_file: Path, calibration_name: str) -> dict:
    text = calibration_file.read_text()
    radtan = _yaml_section(text, "nav_cam_radtan")
    fov = _yaml_section(text, "nav_cam_fov")
    extrinsic = _yaml_section(text, "nav_cam_extr")
    intrinsics = [float(value) for value in _yaml_value(radtan, "intrinsics")]
    fx, fy, cx, cy = intrinsics
    return {
        "width": int(_yaml_value(radtan, "resolution")[0]),
        "height": int(_yaml_value(radtan, "resolution")[1]),
        "K": [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        "distortion_model": _yaml_value(radtan, "distortion_model"),
        "dist_opencv": [float(value) for value in _yaml_value(radtan, "distortion_coeffs")],
        "T_cam_px4imu": _yaml_matrix(extrinsic, "T_cam_px4imu"),
        "alternative_fov_model": {
            "camera_model": _yaml_value(fov, "camera_model"),
            "distortion_model": _yaml_value(fov, "distortion_model"),
            "intrinsics": [float(value) for value in _yaml_value(fov, "intrinsics")],
            "distortion_coeffs": [float(value) for value in _yaml_value(fov, "distortion_coeffs")],
        },
        "calibration_file": calibration_name,
    }


def _plot_truth_path(truth: pd.DataFrame, plot_path: Path, title: str) -> None:
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(truth.e_m, truth.n_m, color="#125a9c", linewidth=1.5)
    ax.scatter(truth.e_m.iloc[0], truth.n_m.iloc[0], color="#269c58", label="first", zorder=3)
    ax.scatter(truth.e_m.iloc[-1], truth.n_m.iloc[-1], color="#d34a3a", label="last", zorder=3)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Export an INSANE sequence to taipeidrift-replay/1.")
    parser.add_argument("--sequence", choices=tuple(SEQUENCES), default="mars_1")
    sequence = parser.parse_args().sequence
    config = SEQUENCES[sequence]
    SENSORS_ZIP = config["sensors_zip"]
    NAV_ROOT = config["nav_root"]
    NAV_IMAGE_DIR = NAV_ROOT / "img"
    CALIBRATION_FILE = config["calibration_file"]
    OUT = config["out"]
    PLOT_PATH = config["plot_path"]
    if not SENSORS_ZIP.is_file():
        raise FileNotFoundError(SENSORS_ZIP)
    for path in (NAV_ROOT / "nav_cam_timestamps.csv", NAV_IMAGE_DIR, CALIBRATION_FILE):
        if not path.exists():
            raise FileNotFoundError(path)
    OUT.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(SENSORS_ZIP) as archive:
        imu_source = _clean(_read_csv(archive, "px4_imu.csv"))
        baro_source = _clean(_read_csv(archive, "px4_baro.csv"))
        gnss_source = _clean(_read_csv(archive, "px4_gps.csv"))
        velocity_source = _clean(_read_csv(archive, "px4_gps_vel_data.csv"))
        rtk_source = _clean(_read_csv(archive, "rtk_gps1.csv"))
        rtk_time_offset_s = _time_offset_s(archive)

    required_imu = {"t", "a_x", "a_y", "a_z", "w_x", "w_y", "w_z"}
    required_baro = {"t", "p"}
    required_gnss = {"t", "lat", "long", "alt", "cov_p_x", "cov_p_y", "cov_p_z"}
    required_velocity = {"t", "v_x", "v_y", "v_z"}
    required_rtk = {"t", "lat", "long", "alt", "p_x", "p_y", "p_z", "rtk_status"}
    for frame, required, source_name in (
        (imu_source, required_imu, "px4_imu.csv"),
        (baro_source, required_baro, "px4_baro.csv"),
        (gnss_source, required_gnss, "px4_gps.csv"),
        (velocity_source, required_velocity, "px4_gps_vel_data.csv"),
        (rtk_source, required_rtk, "rtk_gps1.csv"),
    ):
        missing = required.difference(frame.columns)
        if missing:
            raise ValueError(f"{source_name} is missing columns {sorted(missing)}")

    image_times = pd.read_csv(
        NAV_ROOT / "nav_cam_timestamps.csv",
        comment="#",
        header=None,
        names=["img_no", "t_abs_s", "filename"],
        skipinitialspace=True,
    )
    image_times = image_times.dropna(subset=["t_abs_s", "filename"]).sort_values("t_abs_s")
    image_paths = [NAV_IMAGE_DIR / f"{int(value)}.png" for value in image_times.filename]
    missing_images = [path for path in image_paths if not path.is_file()]
    if missing_images:
        raise FileNotFoundError(f"{len(missing_images)} timestamped images missing; first: {missing_images[0]}")

    fixed_rtk = rtk_source.loc[rtk_source.rtk_status == 1].copy()
    fixed_rtk["t_common"] = fixed_rtk.t - rtk_time_offset_s
    fixed_rtk = _clean(fixed_rtk, "t_common")
    if fixed_rtk.empty:
        raise ValueError("rtk_gps1.csv has no fixed RTK samples (rtk_status == 1)")

    t0_abs_s = min(
        float(imu_source.t.iloc[0]),
        float(baro_source.t.iloc[0]),
        float(gnss_source.t.iloc[0]),
        float(image_times.t_abs_s.min()),
        float(fixed_rtk.t_common.iloc[0]),
    )

    imu = pd.DataFrame({
        "t_s": imu_source.t - t0_abs_s,
        "gx": imu_source.w_x,
        "gy": imu_source.w_y,
        "gz": imu_source.w_z,
        "ax": imu_source.a_x,
        "ay": imu_source.a_y,
        "az": imu_source.a_z,
    })
    imu.to_csv(OUT / "imu.csv", index=False, float_format="%.9f", na_rep="")

    pressure_pa = baro_source.p.to_numpy(dtype=float)
    if np.any(~np.isfinite(pressure_pa)) or np.any(pressure_pa <= 0):
        raise ValueError("px4_baro.csv has non-finite or non-positive pressure")
    baro = pd.DataFrame({
        "t_s": baro_source.t - t0_abs_s,
        "pressure_pa": pressure_pa,
        "temperature_c": np.nan,
        "alt_isa_m": isa_altitude_m(pressure_pa),
    })
    baro.to_csv(OUT / "baro.csv", index=False, float_format="%.9f", na_rep="")

    velocity = velocity_source.rename(columns={"t": "t_velocity"})
    gnss_source = pd.merge_asof(
        gnss_source.sort_values("t"),
        velocity.sort_values("t_velocity"),
        left_on="t",
        right_on="t_velocity",
        direction="nearest",
        tolerance=0.025,
    )
    cov_x = gnss_source.cov_p_x.to_numpy(dtype=float)
    cov_y = gnss_source.cov_p_y.to_numpy(dtype=float)
    cov_z = gnss_source.cov_p_z.to_numpy(dtype=float)
    hacc = np.sqrt(np.maximum(cov_x, cov_y))
    vacc = np.sqrt(cov_z)
    hacc[(cov_x < 0) | (cov_y < 0)] = np.nan
    vacc[cov_z < 0] = np.nan
    gnss = pd.DataFrame({
        "t_s": gnss_source.t - t0_abs_s,
        "lat_deg": gnss_source.lat,
        "lon_deg": gnss_source["long"],
        "alt_m": gnss_source.alt,
        "fix_type": np.nan,
        "hacc_m": hacc,
        "vacc_m": vacc,
        "ve_mps": gnss_source.v_x,
        "vn_mps": gnss_source.v_y,
        "vu_mps": gnss_source.v_z,
        "nsat": np.nan,
    })
    gnss.to_csv(OUT / "gnss.csv", index=False, float_format="%.9f", na_rep="")

    xyz = fixed_rtk[["p_x", "p_y", "p_z"]].to_numpy(dtype=float)
    xyz_origin = xyz[0]
    truth = pd.DataFrame({
        "t_s": fixed_rtk.t_common - t0_abs_s,
        "e_m": xyz[:, 0] - xyz_origin[0],
        "n_m": xyz[:, 1] - xyz_origin[1],
        "u_m": xyz[:, 2] - xyz_origin[2],
        "lat_deg": fixed_rtk.lat,
        "lon_deg": fixed_rtk["long"],
        "alt_m": fixed_rtk.alt,
        "qw": np.nan,
        "qx": np.nan,
        "qy": np.nan,
        "qz": np.nan,
    })
    truth.to_csv(OUT / "truth.csv", index=False, float_format="%.9f", na_rep="")

    relative_images = os.path.relpath(NAV_IMAGE_DIR, OUT)
    images = pd.DataFrame({
        "t_s": image_times.t_abs_s - t0_abs_s,
        "cam": CAMERA_NAME,
        "path": [f"{relative_images}/{path.name}" for path in image_paths],
    })
    images.to_csv(OUT / "images.csv", index=False, float_format="%.9f")

    cal = _camera_calibration(CALIBRATION_FILE, config["calibration_name"])
    rate = lambda frame: _rate_hz(frame.t_s)
    meta = {
        "schema": SCHEMA,
        "sequence": OUT.name,
        "evidence_label": "MEASURED",
        "source": "INSANE: Cross-Domain UAV Data Sets with Increased Number of Sensors for developing Advanced and Novel Estimators",
        "url": "https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/datasets/insane-dataset/",
        "data_urls": {
            "sensors": DATA_URL + f"{sequence}_sensors.zip",
            "nav_camera": DATA_URL + f"{sequence}_nav_cam.zip",
            "calibration": DATA_URL + "insane_sensor_calib_preprocessed.zip",
        },
        "licence": (
            "INSANE dataset BSD-2 with additional conditions; official LICENSE.txt: "
            "\"Without limiting other conditions in the License, the grant of rights under the "
            "License will not include, and the License does not grant to you, the right to Sell the Software.\" "
            "Academic use requires citation of the authors."
        ),
        "licence_url": LICENSE_URL,
        "platform": config["platform"],
        "clock": {
            "source": "Dataset ROS clock in seconds (Unix-like timestamps)",
            "sequence_zero_abs_s": float(t0_abs_s),
            "t_s_rule": "absolute dataset timestamp minus sequence_zero_abs_s; zero is the earliest sample across exported streams",
            "rtk_alignment": f"rtk_gps1.t_common = rtk_gps1.t - time_info.t_mag_gps ({rtk_time_offset_s:.6f} s)",
            "image_timestamp_note": "nav_cam_timestamps.csv values are seconds from ROS Time.to_sec(); its header says t[ns] but values are seconds",
        },
        "origin": {
            "lat_deg": float(fixed_rtk.lat.iloc[0]),
            "lon_deg": float(fixed_rtk["long"].iloc[0]),
            "alt_m": float(fixed_rtk.alt.iloc[0]),
            "note": f"first fixed RTK GPS1 sample; truth e/n/u recentered at this sample, ENU axes follow the dataset's {config['enu_reference']}",
        },
        "sensors": {
            "imu": {
                "file": "imu.csv",
                "rate_hz": rate(imu),
                "provenance": "PX4 flight-controller stream px4_imu.csv (/mavros/imu/data_raw); README reports ENU [t,a_x,a_y,a_z,w_x,w_y,w_z]",
                "frame": "ENU as labeled in the INSANE README; values kept in dataset-exported axes without rotation",
                "gyro_units": "rad/s",
                "accel_units": "m/s^2",
            },
            "baro": {
                "file": "baro.csv",
                "rate_hz": rate(baro),
                "provenance": "PX4 /mavros/imu/static_pressure stream px4_baro.csv",
                "altitude_semantics": "README.txt: \"PX4 Baro [pa] format: [t, p]\"; p is retained as pressure_pa in Pa; alt_isa_m is recomputed with ISA p0=101325 Pa. Temperature is not provided.",
            },
            "gnss": {
                "file": "gnss.csv",
                "rate_hz": rate(gnss),
                "provenance": "Non-RTK PX4 receiver px4_gps.csv; px4_gps_vel_data.csv nearest-time matched within 0.025 s",
                "alt_ref": "not stated in the bundled README; alt is kept as logged",
                "accuracy_note": "hacc_m and vacc_m are square roots of the maximum horizontal and vertical diagonal covariance values; fix_type and nsat are not present in px4_gps.csv and are empty",
                "velocity_frame": "v_x/v_y/v_z passed through as east/north/up per MAVROS global_position/raw/gps_vel ENU convention",
            },
            "camera": {
                "cams": {
                    CAMERA_NAME: {
                        "file": "images.csv",
                        "rate_hz": rate(images),
                        **cal,
                        "direction": "down-looking navigation camera",
                        "provenance": f"{sequence}_nav_cam.zip PNGs; times from nav_cam_timestamps.csv; calibration from {config['calibration_name']}",
                    }
                }
            },
            "truth": {
                "source": "rtk_gps1.csv fixed RTK solution only (rtk_status == 1); p_x/p_y/p_z in dataset-local ENU",
                "rate_hz": rate(truth),
                "evaluator_only": True,
                "orientation": "No quaternion is provided by the RTK CSV; qw/qx/qy/qz are empty.",
                "reference": f"{config['reference']}; p_x/p_y/p_z are recentered at the first fixed RTK sample.",
            },
        },
        "counts": {
            "imu": len(imu),
            "baro": len(baro),
            "gnss": len(gnss),
            "images": len(images),
            "truth": len(truth),
            "fixed_rtk_fraction": float((rtk_source.rtk_status == 1).mean()),
        },
        "generator": "experiments/t_export_insane.py",
    }

    (OUT / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    _plot_truth_path(truth, PLOT_PATH, config["plot_title"])
    print(json.dumps(meta["counts"], sort_keys=True))
    print(f"path_plot={PLOT_PATH.relative_to(ROOT)} ({PLOT_PATH.stat().st_size} bytes)")


if __name__ == "__main__":
    main()

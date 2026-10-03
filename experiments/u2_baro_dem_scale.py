"""ALTO camera dead reckoning with frozen, baro-minus-DEM, and oracle AGL scale.

The pre-cut calibration uses the first 300 m exactly as in h_alto_end_to_end.py:
fit image flow to GNSS displacement, then freeze that camera mapping.  After the
cut there are no map/image fixes.  The barometer is SIMULATED from the recorded
ALTO altitude plus a replayed measured Zurich baro-error profile; it is not an
ALTO barometer.  Truth positions after the cut are used only for scoring and
for the explicitly oracle (true-AGL) upper bound.

Run from the repository root:
    .venv/bin/python experiments/u2_baro_dem_scale.py --kill-test
    .venv/bin/python experiments/u2_baro_dem_scale.py
"""
from __future__ import annotations

import argparse
import json
import os
import time
import zipfile
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rasterio
from pyproj import Transformer
from rasterio.windows import from_bounds

ROOT = Path.cwd()
ZIP_PATH = ROOT / "data/raw/alto/Val.zip"
VERTICAL_RESIDUALS = ROOT / "data/processed/zurich_vertical/replay_errors.csv"
FALLBACK_RESIDUALS = ROOT / "data/processed/zurich_baro/relative_altitude_errors.csv"
OUTPUT_DIR = ROOT / "data/processed/u2_baro_dem_scale"
FLOW_CACHE = OUTPUT_DIR / "alto_flow.npy"
DEM_CACHE = OUTPUT_DIR / "alto_dem.npz"
JAM_AT_M = 300.0
KEEP_FLOW = 1200  # approximately 20 min at one ALTO query row per second
CRS_TRANSFORMER = Transformer.from_crs("EPSG:32617", "EPSG:4326", always_xy=True)


def load_alto() -> tuple[zipfile.ZipFile, pd.DataFrame, np.ndarray, np.ndarray, int]:
    archive = zipfile.ZipFile(ZIP_PATH)
    query = pd.read_csv(archive.open("Val/query.csv"))
    truth = query[["easting", "northing"]].to_numpy(dtype=np.float64)
    travelled = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(truth, axis=0), axis=1))]
    jam = int(np.searchsorted(travelled, JAM_AT_M))
    return archive, query, truth, travelled, jam


def image_motion(archive: zipfile.ZipFile, query: pd.DataFrame, n_frames: int, save: bool) -> np.ndarray:
    """Copied from h_alto_end_to_end.py, with a cache confined to this experiment."""
    if FLOW_CACHE.exists():
        cached = np.load(FLOW_CACHE)
        if len(cached) >= n_frames:
            return cached[:n_frames]
    flow = np.zeros((n_frames, 2), dtype=np.float64)
    previous = None
    for k, name in enumerate(query.name.iloc[:n_frames]):
        raw = np.frombuffer(archive.read(f"Val/query_images/{name}"), np.uint8)
        frame = cv2.resize(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE), (250, 250), interpolation=cv2.INTER_AREA)
        if previous is not None:
            field = cv2.calcOpticalFlowFarneback(previous, frame, None, 0.5, 4, 21, 3, 7, 1.5, 0)
            flow[k] = np.median(field[60:190, 60:190].reshape(-1, 2), axis=0) * 2
        previous = frame
    if save:
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        np.save(FLOW_CACHE, flow)
    return flow


def load_dem(truth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Use the Copernicus GLO-30 S3 tile and pixel mapping from i_alto_zoom_check.py."""
    if not DEM_CACHE.exists():
        lon, lat = CRS_TRANSFORMER.transform(truth[:, 0], truth[:, 1])
        west, east = float(lon.min() - 0.02), float(lon.max() + 0.02)
        south, north = float(lat.min() - 0.02), float(lat.max() + 0.02)
        tile = f"Copernicus_DSM_COG_10_N{int(np.floor(south)):02d}_00_W{int(-np.floor(west)):03d}_00_DEM"
        url = f"/vsicurl/https://copernicus-dem-30m.s3.amazonaws.com/{tile}/{tile}.tif"
        with rasterio.open(url) as dataset:
            window = from_bounds(west, south, east, north, dataset.transform)
            data = dataset.read(1, window=window).astype(np.float32)
            transform = dataset.window_transform(window)
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        np.savez(DEM_CACHE, data=data, t=np.array([transform.a, transform.b, transform.c, transform.d, transform.e, transform.f]))
        print(f"Copernicus GLO-30 tile: {tile}")
    dem = np.load(DEM_CACHE)
    return dem["data"], dem["t"]


def ground_height(xy: np.ndarray, dem: np.ndarray, affine: np.ndarray) -> np.ndarray:
    lon, lat = CRS_TRANSFORMER.transform(xy[:, 0], xy[:, 1])
    a, _, c, _, e, f = affine
    col = (np.asarray(lon) - c) / a - 0.5
    row = (np.asarray(lat) - f) / e - 0.5
    return cv2.remap(
        dem,
        col.astype(np.float32).reshape(-1, 1),
        row.astype(np.float32).reshape(-1, 1),
        cv2.INTER_LINEAR,
    ).ravel()


def residual_profile(n_frames: int) -> tuple[np.ndarray, dict]:
    """Replay one actual Zurich baro-only residual-vs-horizon curve, piecewise linearly.

    Zurich's files contain sparse GNSS-cut residuals rather than a 1 Hz sensor
    series.  The longest measured baro_only replay is used as a deterministic
    empirical profile (0 m at the cut, then its measured horizon residuals).
    ALTO has no timestamps in query.csv; query row index is treated as seconds.
    """
    if VERTICAL_RESIDUALS.exists():
        data = pd.read_csv(VERTICAL_RESIDUALS)
        rows = data[data.variant == "baro_only"].copy()
        source = str(VERTICAL_RESIDUALS.relative_to(ROOT))
        err_col = "error_m"
    else:
        data = pd.read_csv(FALLBACK_RESIDUALS)
        rows = data.copy()
        source = str(FALLBACK_RESIDUALS.relative_to(ROOT))
        err_col = "baro_error_m"
    rows = rows.dropna(subset=["cut_s", "horizon_s", err_col])
    # Prefer the earliest complete, longest residual profile for reproducibility.
    profiles = []
    for cut_s, group in rows.groupby("cut_s", sort=True):
        group = group.sort_values("horizon_s")
        profiles.append((float(group.horizon_s.max()), len(group), float(cut_s), group))
    if not profiles:
        raise ValueError(f"No Zurich barometer residuals found in {source}")
    best_horizon = max(item[0] for item in profiles)
    profile = min((item for item in profiles if item[0] == best_horizon), key=lambda item: item[2])[3]
    horizons = profile.horizon_s.to_numpy(dtype=np.float64)
    errors = profile[err_col].to_numpy(dtype=np.float64)
    seconds = np.r_[0.0, horizons]
    residual_m = np.r_[0.0, errors]
    # Distinct horizon samples are guaranteed by the source, but coalesce defensively.
    unique_seconds, inverse = np.unique(seconds, return_inverse=True)
    if len(unique_seconds) != len(seconds):
        values = np.zeros_like(unique_seconds)
        counts = np.zeros_like(unique_seconds)
        np.add.at(values, inverse, residual_m)
        np.add.at(counts, inverse, 1)
        seconds, residual_m = unique_seconds, values / counts
    residual = np.interp(np.arange(n_frames, dtype=np.float64), seconds, residual_m)
    detail = {
        "source": source,
        "source_variant": "baro_only" if err_col == "error_m" else "baro_error_m",
        "source_cut_s": float(profile.cut_s.iloc[0]),
        "horizon_s": horizons.tolist(),
        "residual_m": errors.tolist(),
        "replay_time_assumption": "ALTO query row index is treated as seconds; piecewise linear between measured Zurich cut horizons; endpoint held after final horizon",
    }
    return residual, detail


def calibrate_and_run(
    query: pd.DataFrame,
    truth: np.ndarray,
    travelled: np.ndarray,
    jam: int,
    flow: np.ndarray,
    terrain_true: np.ndarray,
    sim_baro: np.ndarray,
    dem: np.ndarray,
    dem_transform: np.ndarray,
    stop: int,
) -> tuple[dict, dict]:
    """Learn flow/focal and vertical datum using only the first 300 m; DR has no fixes."""
    steps = np.diff(truth, axis=0)
    flow_to_ground, *_ = np.linalg.lstsq(flow[1 : jam + 1], steps[:jam], rcond=None)
    calibration_agl = query.altitude.to_numpy(dtype=np.float64)[: jam + 1] - terrain_true[: jam + 1]
    h0 = float(np.median(calibration_agl))
    true_agl = query.altitude.to_numpy(dtype=np.float64) - terrain_true
    datum_offset = float(np.median(true_agl[: jam + 1] - (sim_baro[: jam + 1] - terrain_true[: jam + 1])))
    singular_values = np.linalg.svd(flow_to_ground, compute_uv=False)
    m_per_pixel = float(np.mean(singular_values))
    focal_px = h0 / m_per_pixel

    initial = truth[jam].copy()
    end = min(stop, len(query))
    initial_baro_agl = float(sim_baro[jam] - terrain_true[jam] + datum_offset)
    initial_true_agl = float(true_agl[jam])
    paths = {name: [initial.copy()] for name in ("frozen_zoom", "baro_dem", "true_agl")}
    estimates = {name: initial.copy() for name in paths}
    scales = {"frozen_zoom": [1.0], "baro_dem": [initial_baro_agl / h0], "true_agl": [initial_true_agl / h0]}
    heights = {"frozen_zoom": [h0], "baro_dem": [initial_baro_agl], "true_agl": [initial_true_agl]}

    for k in range(jam + 1, end):
        estimated_terrain = float(ground_height(estimates["baro_dem"].reshape(1, 2), dem, dem_transform)[0])
        baro_agl = float(sim_baro[k] - estimated_terrain + datum_offset)
        true_height = float(true_agl[k])
        scale_values = {
            "frozen_zoom": 1.0,
            "baro_dem": baro_agl / h0,
            "true_agl": true_height / h0,
        }
        height_values = {"frozen_zoom": h0, "baro_dem": baro_agl, "true_agl": true_height}
        for name in paths:
            estimates[name] = estimates[name] + (flow[k] @ flow_to_ground) * scale_values[name]
            paths[name].append(estimates[name].copy())
            scales[name].append(scale_values[name])
            heights[name].append(height_values[name])

    result = {}
    for name, path in paths.items():
        path_array = np.asarray(path)
        truth_segment = truth[jam:end]
        errors = np.linalg.norm(path_array - truth_segment, axis=1)
        scale_array = np.asarray(scales[name], dtype=np.float64)
        reference_scale = true_agl[jam:end] / h0
        relative_scale_error = np.abs(scale_array / reference_scale - 1.0)
        result[name] = {
            "path": path_array,
            "error": errors,
            "scale": scale_array,
            "height": np.asarray(heights[name], dtype=np.float64),
            "scale_error": relative_scale_error,
        }
    calibration = {
        "flow_to_ground": flow_to_ground.tolist(),
        "calibration_height_m": h0,
        "datum_offset_m": datum_offset,
        "effective_focal_px": focal_px,
        "mean_m_per_pixel_at_calibration": m_per_pixel,
        "calibration_frame": jam,
        "calibration_distance_m": float(travelled[jam]),
    }
    return result, calibration


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kill-test", action="store_true", help="run the first 1200 post-cut frames (~20 min at one row/s)")
    args = parser.parse_args()
    started = time.time()
    archive, query, truth, travelled, jam = load_alto()
    stop = min(jam + KEEP_FLOW + 1, len(query)) if args.kill_test else len(query)
    flow = image_motion(archive, query, stop, save=not args.kill_test and stop == len(query))
    dem, dem_transform = load_dem(truth)
    terrain_true = ground_height(truth, dem, dem_transform)
    residual, residual_info = residual_profile(len(query))
    sim_baro = query.altitude.to_numpy(dtype=np.float64) + residual
    results, calibration = calibrate_and_run(query, truth, travelled, jam, flow, terrain_true, sim_baro, dem, dem_transform, stop)

    print(f"ALTO cut: {travelled[jam]:.1f} m (frame {jam}); replay window {stop - jam - 1} post-cut frames")
    print(f"Calibration only before cut: datum offset {calibration['datum_offset_m']:.3f} m; effective focal {calibration['effective_focal_px']:.1f} px; AGL reference {calibration['calibration_height_m']:.2f} m")
    for name, result in results.items():
        print(
            f"{name:12s} scale error median {np.median(result['scale_error']):.2%}; "
            f"position error median {np.median(result['error']):.1f} m, end {result['error'][-1]:.1f} m"
        )

    if args.kill_test:
        frozen_end = results["frozen_zoom"]["error"][-1]
        baro_end = results["baro_dem"]["error"][-1]
        gain = 1.0 - baro_end / frozen_end if frozen_end > 0 else 0.0
        print(f"20-minute kill-test criterion: baro-DEM endpoint gain {gain:.1%} vs frozen scale")
        print("KILL: relative gain < 20%; stop experiment." if gain < 0.20 else "CONTINUE: kill criterion did not fire.")
        test_result = {
            "evidence_label": "MEASURED partial ALTO path; SIMULATED barometer; oracle true-AGL comparator",
            "command": ".venv/bin/python experiments/u2_baro_dem_scale.py --kill-test",
            "cut_distance_m": float(travelled[jam]),
            "cut_frame": jam,
            "post_cut_frames": int(stop - jam - 1),
            "assumed_seconds_per_query_row": 1,
            "calibration": calibration,
            "variants": {
                name: {
                    "median_scale_error_percent": float(np.median(result["scale_error"]) * 100.0),
                    "median_position_error_m": float(np.median(result["error"])),
                    "endpoint_position_error_m": float(result["error"][-1]),
                }
                for name, result in results.items()
            },
            "baro_dem_endpoint_gain_vs_frozen_percent": float(gain * 100.0),
            "kill_threshold_percent": 20.0,
            "decision": "kill" if gain < 0.20 else "continue",
            "runtime_s": time.time() - started,
        }
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / "kill_test.json").write_text(json.dumps(test_result, indent=2) + "\n")
        print(f"Saved: {OUTPUT_DIR / 'kill_test.json'}")
        print(f"test runtime: {time.time() - started:.1f} s")
        archive.close()
        return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    frame_index = np.arange(jam, stop)
    distance_after_cut = travelled[frame_index] - travelled[jam]
    truth_segment = truth[jam:stop]
    altitude = query.altitude.to_numpy(dtype=np.float64)[jam:stop]
    true_agl_segment = altitude - terrain_true[jam:stop]
    scale_rows = {
        "frame": frame_index,
        "distance_after_cut_m": distance_after_cut,
        "true_easting": truth_segment[:, 0],
        "true_northing": truth_segment[:, 1],
        "recorded_altitude_m": altitude,
        "true_agl_m": true_agl_segment,
    }
    trajectory_rows = {"frame": frame_index, "distance_after_cut_m": distance_after_cut, "true_easting": truth_segment[:, 0], "true_northing": truth_segment[:, 1]}
    for name, result in results.items():
        scale_rows[f"{name}_height_m"] = result["height"]
        scale_rows[f"{name}_scale"] = result["scale"]
        scale_rows[f"{name}_scale_error_fraction"] = result["scale_error"]
        scale_rows[f"{name}_position_error_m"] = result["error"]
        trajectory_rows[f"{name}_easting"] = result["path"][:, 0]
        trajectory_rows[f"{name}_northing"] = result["path"][:, 1]
        trajectory_rows[f"{name}_position_error_m"] = result["error"]
    pd.DataFrame(scale_rows).to_csv(OUTPUT_DIR / "scale_series.csv", index=False)
    pd.DataFrame(trajectory_rows).to_csv(OUTPUT_DIR / "trajectory.csv", index=False)

    baseline_end = float(results["frozen_zoom"]["error"][-1])
    summary = {
        "experiment": "X2 camera dead-reckoning scale from barometric altitude minus DEM on ALTO Val",
        "evidence_label": "MEASURED ALTO camera/terrain trajectory; SIMULATED barometer (ALTO query altitude + replayed Zurich residuals); true_AGL is an ORACLE upper bound",
        "source_zip": str(ZIP_PATH.relative_to(ROOT)),
        "cut_distance_m": float(travelled[jam]),
        "cut_frame": jam,
        "total_distance_m": float(travelled[-1]),
        "post_cut_frames": int(len(query) - jam - 1),
        "team_reported_end_error_m": 608.0,
        "calibration": calibration,
        "barometer_simulation": residual_info,
        "variants": {},
        "runtime_s": time.time() - started,
        "limits": [
            "ALTO query.csv has no timestamps; the Zurich residual profile is replayed at one second per query row.",
            "Zurich residual input is a sparse measured baro-only residual-vs-horizon profile, linearly interpolated; its endpoint is held beyond the last observed horizon.",
            "The baro-DEM estimator queries terrain at its own estimated position. True AGL intentionally uses true position and altitude as an oracle upper bound.",
            "No map/image fixes are used after the first 300 m.",
        ],
    }
    for name, result in results.items():
        summary["variants"][name] = {
            "scale_error_median_fraction": float(np.median(result["scale_error"])),
            "scale_error_median_percent": float(np.median(result["scale_error"]) * 100.0),
            "position_error_median_m": float(np.median(result["error"])),
            "position_error_final_m": float(result["error"][-1]),
            "end_error_reduction_vs_frozen_percent": float((1.0 - result["error"][-1] / baseline_end) * 100.0) if baseline_end > 0 else 0.0,
        }
    (OUTPUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    colors = {"frozen_zoom": "tab:gray", "baro_dem": "tab:blue", "true_agl": "tab:green"}
    labels = {"frozen_zoom": "Frozen zoom (team baseline)", "baro_dem": "Simulated baro − DEM at estimated position", "true_agl": "True AGL (oracle upper bound)"}
    for name, result in results.items():
        axes[0].plot(distance_after_cut, result["error"], label=labels[name], color=colors[name])
        axes[1].plot(distance_after_cut, result["scale_error"] * 100.0, label=labels[name], color=colors[name])
    axes[0].axhline(608.0, color="black", linestyle=":", label="Team reported end error: 608 m")
    axes[0].set_ylabel("Position error (m)")
    axes[0].set_title("ALTO Val camera dead reckoning; GNSS cut after first 300 m")
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[1].set_xlabel("Distance flown after cut (m)")
    axes[1].set_ylabel("Absolute scale error (%)")
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "scale_and_position_error.png", dpi=140)
    plt.close(fig)

    print(f"Team reported end error: 608 m; measured frozen-scale end error: {baseline_end:.1f} m")
    print(f"Saved: {OUTPUT_DIR / 'summary.json'}")
    print(f"Saved: {OUTPUT_DIR / 'scale_series.csv'}")
    print(f"Saved: {OUTPUT_DIR / 'trajectory.csv'}")
    print(f"Saved: {OUTPUT_DIR / 'scale_and_position_error.png'}")
    print(f"runtime: {time.time() - started:.1f} s")
    archive.close()


if __name__ == "__main__":
    main()

"""Measure heading-reference value on real ALTO camera dead reckoning.

The camera flow and frozen image-to-world map are measured from ALTO. The
heading oracle uses ALTO orientation; sun/gyro heading errors are simulated.
Run from the repository root with:
    .venv/bin/python experiments/s_alto_heading.py
"""
from __future__ import annotations

import json

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

import u2_baro_dem_scale as u2

OUTPUT_DIR = u2.ROOT / "data/processed/alto_heading"
SEEDS = tuple(range(20))
NO_FIX_TARGETS_M = {"frozen_zoom": 608.2, "baro_dem": 639.2, "true_agl": 426.0}
HEADING_ROTATION_SIGN = -1.0
HEADING_VARIANTS = ("heading_oracle", "sun_good", "sun_field", "gyro")
SCALES_WITH_HEADING = ("frozen_zoom", "true_agl")
SUN_GOOD_SIGMA_DEG = 0.98
SUN_FIELD_SIGMA_DEG = 2.85
GYRO_INITIAL_SIGMA_DEG = 1.0
GYRO_DRIFT_SIGMA_DEG_PER_S = 0.005

EVIDENCE_LABELS = {
    "heading_oracle": "MEASURED camera motion; ORACLE ALTO heading",
    "sun_good": "SIMULATED heading sensors on MEASURED camera motion",
    "sun_field": "SIMULATED heading sensors on MEASURED camera motion",
    "gyro": "SIMULATED heading sensors on MEASURED camera motion",
}
NO_HEADING_EVIDENCE = {
    "frozen_zoom": "MEASURED camera motion; frozen scale; no heading correction",
    "baro_dem": "MEASURED camera motion; SIMULATED baro-DEM scale; no heading correction",
    "true_agl": "MEASURED camera motion; ORACLE true-AGL scale; no heading correction",
}


def heading_truth(query: pd.DataFrame, truth: np.ndarray) -> np.ndarray:
    """Return unwrapped compass yaw from the ALTO ECEF orientation quaternions."""
    lon, lat = u2.CRS_TRANSFORMER.transform(truth[:, 0], truth[:, 1])
    lat_rad, lon_rad = np.radians(lat), np.radians(lon)
    east = np.stack((-np.sin(lon_rad), np.cos(lon_rad), np.zeros_like(lon_rad)), axis=1)
    north = np.stack(
        (
            -np.sin(lat_rad) * np.cos(lon_rad),
            -np.sin(lat_rad) * np.sin(lon_rad),
            np.cos(lat_rad),
        ),
        axis=1,
    )
    quaternion = query[["orient_x", "orient_y", "orient_z", "orient_w"]].to_numpy(dtype=np.float64)
    forward = Rotation.from_quat(quaternion).apply([1.0, 0.0, 0.0])
    compass_yaw = np.arctan2(np.sum(forward * east, axis=1), np.sum(forward * north, axis=1))
    return np.unwrap(compass_yaw)


def rotate_xy(vectors: np.ndarray, angles_rad: np.ndarray) -> np.ndarray:
    """Rotate EN vectors by mathematical angles (positive is counter-clockwise)."""
    cosine, sine = np.cos(angles_rad), np.sin(angles_rad)
    return np.stack(
        (cosine * vectors[..., 0] - sine * vectors[..., 1],
         sine * vectors[..., 0] + cosine * vectors[..., 1]),
        axis=-1,
    )


def verify_heading_sign(
    truth: np.ndarray,
    travelled: np.ndarray,
    jam: int,
    flow: np.ndarray,
    yaw: np.ndarray,
) -> dict:
    """Choose the EN rotation sign using a held-out segment of the pre-cut 300 m."""
    fit_distance_m = min(100.0, float(travelled[jam]) / 2.0)
    fit_end = int(np.searchsorted(travelled, fit_distance_m, side="left"))
    fit_steps = truth[1 : fit_end + 1] - truth[:fit_end]
    fit_map, *_ = np.linalg.lstsq(flow[1 : fit_end + 1], fit_steps, rcond=None)

    frames = np.arange(fit_end + 1, jam + 1, dtype=np.int64)
    measured_steps = flow[frames] @ fit_map
    true_steps = truth[frames] - truth[frames - 1]
    yaw_delta = yaw[frames] - yaw[fit_end]

    def rms(predicted: np.ndarray) -> float:
        return float(np.sqrt(np.mean(np.sum((predicted - true_steps) ** 2, axis=1))))

    baseline_rms = rms(measured_steps)
    candidate_rms = {}
    for sign in (-1.0, 1.0):
        candidate_rms[str(int(sign))] = rms(rotate_xy(measured_steps, sign * yaw_delta))
    chosen_sign = min((-1.0, 1.0), key=lambda sign: candidate_rms[str(int(sign))])
    corrected_rms = candidate_rms[str(int(chosen_sign))]
    if chosen_sign != HEADING_ROTATION_SIGN or corrected_rms >= baseline_rms:
        raise AssertionError(
            "Pre-cut heading sign check did not reduce held-out flow-to-ground residual: "
            f"baseline={baseline_rms:.6f}, candidates={candidate_rms}"
        )
    return {
        "method": "fit flow-to-ground on 0-100 m, validate per-step residual on held-out 100-300 m",
        "fit_end_frame": fit_end,
        "fit_end_distance_m": float(travelled[fit_end]),
        "validation_start_distance_m": float(travelled[fit_end]),
        "validation_end_distance_m": float(travelled[jam]),
        "validation_frames": int(len(frames)),
        "baseline_rms_m": baseline_rms,
        "candidate_rms_by_sign_m": candidate_rms,
        "chosen_sign": int(chosen_sign),
        "corrected_rms_m": corrected_rms,
        "rms_reduction_m": baseline_rms - corrected_rms,
        "rotation_convention": "compass yaw is clockwise from north; EN vector rotation angle is -yaw delta",
    }


def sensor_yaw_errors(variant: str, seed: int, n_steps: int) -> np.ndarray:
    """Draw one sensor's yaw-error trace in degrees for post-cut steps."""
    if variant == "heading_oracle":
        return np.zeros(n_steps, dtype=np.float64)

    rng = np.random.default_rng(seed)
    if variant == "sun_good":
        return 0.7 * rng.normal(0.0, SUN_GOOD_SIGMA_DEG) + 0.3 * rng.normal(
            0.0, SUN_GOOD_SIGMA_DEG, size=n_steps
        )
    if variant == "sun_field":
        return 0.7 * rng.normal(0.0, SUN_FIELD_SIGMA_DEG) + 0.3 * rng.normal(
            0.0, SUN_FIELD_SIGMA_DEG, size=n_steps
        )
    if variant == "gyro":
        initial_error = rng.normal(0.0, GYRO_INITIAL_SIGMA_DEG)
        drift_rate = rng.normal(0.0, GYRO_DRIFT_SIGMA_DEG_PER_S)
        elapsed_s = np.arange(1, n_steps + 1, dtype=np.float64)
        return initial_error + drift_rate * elapsed_s
    raise ValueError(f"Unknown heading variant: {variant}")


def simulate_heading(
    scale_name: str,
    heading_name: str,
    truth: np.ndarray,
    jam: int,
    flow: np.ndarray,
    flow_to_ground: np.ndarray,
    calibration_height_m: float,
    true_agl: np.ndarray,
    yaw: np.ndarray,
) -> np.ndarray:
    """Replay scale-specific measured camera motion with one heading model."""
    seeds = (0,) if heading_name == "heading_oracle" else SEEDS
    n_steps = len(truth) - jam - 1
    paths = np.empty((len(seeds), len(truth) - jam, 2), dtype=np.float64)
    paths[:, 0] = truth[jam]
    yaw_delta_deg = np.degrees(yaw[jam + 1 :] - yaw[jam])

    for seed_index, seed in enumerate(seeds):
        yaw_error_deg = sensor_yaw_errors(heading_name, seed, n_steps)
        for step_index, frame in enumerate(range(jam + 1, len(truth))):
            camera_step = flow[frame] @ flow_to_ground
            angle_rad = HEADING_ROTATION_SIGN * np.radians(yaw_delta_deg[step_index] + yaw_error_deg[step_index])
            corrected_step = rotate_xy(camera_step, angle_rad)
            if scale_name == "true_agl":
                scale = true_agl[frame] / calibration_height_m
            else:
                scale = 1.0
            paths[seed_index, step_index + 1] = paths[seed_index, step_index] + corrected_step * scale
    return paths


def final_error_components(
    paths: np.ndarray,
    truth_segment: np.ndarray,
    truth: np.ndarray,
    travelled: np.ndarray,
) -> dict:
    """Summarize seed-median position errors and signed final along/cross errors."""
    if paths.ndim == 2:
        paths = paths[None, :, :]
    errors = np.linalg.norm(paths - truth_segment[None, :, :], axis=2)
    seed_final_error = errors[:, -1]
    seed_median_error = np.median(errors, axis=1)
    final_vector = paths[:, -1, :] - truth[-1]

    direction_start = int(np.searchsorted(travelled, travelled[-1] - 200.0, side="left"))
    direction = truth[-1] - truth[direction_start]
    direction /= np.linalg.norm(direction)
    cross_direction = np.array([-direction[1], direction[0]])
    along = final_vector @ direction
    cross = final_vector @ cross_direction
    return {
        "final_error_m": float(np.median(seed_final_error)),
        "median_error_m": float(np.median(seed_median_error)),
        "along_track_final_error_m": float(np.median(along)),
        "cross_track_final_error_m": float(np.median(cross)),
        "seed_count": int(len(paths)),
    }


def main() -> None:
    archive, query, truth, travelled, jam = u2.load_alto()
    # Read existing caches only: save=False avoids modifying the u2 flow cache.
    flow = u2.image_motion(archive, query, len(query), save=False)
    dem, dem_transform = u2.load_dem(truth)
    terrain_true = u2.ground_height(truth, dem, dem_transform)
    residual, residual_info = u2.residual_profile(len(query))
    altitude = query.altitude.to_numpy(dtype=np.float64)
    simulated_baro = altitude + residual
    true_agl = altitude - terrain_true

    no_fix_results, calibration = u2.calibrate_and_run(
        query, truth, travelled, jam, flow, terrain_true, simulated_baro, dem, dem_transform, len(query)
    )
    no_fix_reproduction = {}
    for name, target_m in NO_FIX_TARGETS_M.items():
        measured_m = float(no_fix_results[name]["error"][-1])
        difference_m = abs(measured_m - target_m)
        print(f"No-fix {name}: {measured_m:.3f} m (target {target_m:.1f} m; Δ {difference_m:.3f} m)")
        assert difference_m <= 1.0, f"{name} final error differs from requested reproduction target by {difference_m:.3f} m"
        no_fix_reproduction[name] = {
            "target_final_error_m": target_m,
            "rerun_final_error_m": measured_m,
            "difference_m": difference_m,
        }

    flow_to_ground = np.asarray(calibration["flow_to_ground"], dtype=np.float64)
    calibration_height_m = float(calibration["calibration_height_m"])
    yaw = heading_truth(query, truth)
    sign_check = verify_heading_sign(truth, travelled, jam, flow, yaw)
    print(
        "Pre-cut held-out heading sign check: "
        f"RMS {sign_check['baseline_rms_m']:.6f} m -> {sign_check['corrected_rms_m']:.6f} m "
        f"(sign {sign_check['chosen_sign']:+d}; validation {sign_check['validation_start_distance_m']:.1f}-"
        f"{sign_check['validation_end_distance_m']:.1f} m)"
    )

    rows = []
    for name in ("frozen_zoom", "baro_dem", "true_agl"):
        path = np.asarray(no_fix_results[name]["path"], dtype=np.float64)
        metrics = final_error_components(path, truth[jam:], truth, travelled)
        rows.append({
            "scale": name,
            "heading": "none",
            "evidence_label": NO_HEADING_EVIDENCE[name],
            **metrics,
        })

    for scale_name in SCALES_WITH_HEADING:
        for heading_name in HEADING_VARIANTS:
            paths = simulate_heading(
                scale_name,
                heading_name,
                truth,
                jam,
                flow,
                flow_to_ground,
                calibration_height_m,
                true_agl,
                yaw,
            )
            metrics = final_error_components(paths, truth[jam:], truth, travelled)
            rows.append({
                "scale": scale_name,
                "heading": heading_name,
                "evidence_label": EVIDENCE_LABELS[heading_name],
                **metrics,
            })

    table = pd.DataFrame(rows)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    table.to_csv(OUTPUT_DIR / "table.csv", index=False)

    by_combination = {
        f"{row['scale']}+{row['heading']}": {
            key: (int(row[key]) if key == "seed_count" else float(row[key]))
            for key in (
                "final_error_m", "median_error_m", "along_track_final_error_m",
                "cross_track_final_error_m", "seed_count",
            )
        }
        for row in rows
    }
    baseline_error_m = by_combination["frozen_zoom+none"]["final_error_m"]
    improvement_shares = {}
    for label, combination in (
        ("heading_oracle_alone", "frozen_zoom+heading_oracle"),
        ("true_agl_alone", "true_agl+none"),
        ("both", "true_agl+heading_oracle"),
    ):
        error_m = by_combination[combination]["final_error_m"]
        improvement_shares[label] = {
            "comparison": combination,
            "final_error_m": error_m,
            "error_removed_m": baseline_error_m - error_m,
            "share_of_frozen_zoom_no_fix_error_removed": (baseline_error_m - error_m) / baseline_error_m,
        }

    summary = {
        "experiment": "ALTO heading reference value on measured camera dead reckoning",
        "command": ".venv/bin/python experiments/s_alto_heading.py",
        "source_zip": str(u2.ZIP_PATH.relative_to(u2.ROOT)),
        "evidence_label": "MEASURED ALTO camera motion and orientation; simulated sun/gyro heading errors; true_agl is an ORACLE scale",
        "cut_frame": int(jam),
        "cut_distance_m": float(travelled[jam]),
        "post_cut_distance_m": float(travelled[-1] - travelled[jam]),
        "post_cut_frames": int(len(query) - jam - 1),
        "calibration": calibration,
        "barometer_simulation": residual_info,
        "no_fix_reproduction": no_fix_reproduction,
        "heading_truth": {
            "source_columns": ["orient_x", "orient_y", "orient_z", "orient_w"],
            "heading_definition": "Compass yaw of quaternion-rotated aircraft forward axis, clockwise from local north",
            "median_heading_deg": float(np.degrees(np.median(yaw)) % 360.0),
        },
        "heading_sign_check": sign_check,
        "heading_models": {
            "heading_oracle": "Rotate each measured flow_to_ground displacement by negative true compass-yaw change since the cut; zero sensor error.",
            "sun_good": {
                "evidence_label": EVIDENCE_LABELS["sun_good"],
                "yaw_error_deg": "0.7*N(0,0.98 deg) per seed + 0.3*N(0,0.98 deg) independently per post-cut frame",
            },
            "sun_field": {
                "evidence_label": EVIDENCE_LABELS["sun_field"],
                "yaw_error_deg": "0.7*N(0,2.85 deg) per seed + 0.3*N(0,2.85 deg) independently per post-cut frame",
            },
            "gyro": {
                "evidence_label": EVIDENCE_LABELS["gyro"],
                "yaw_error_deg": "N(0,1 deg) per seed at the cut + N(0,0.005 deg/s) per seed times elapsed seconds",
                "elapsed_time_assumption": "one query row per second; first post-cut step uses t=1 s",
            },
            "seed_count": len(SEEDS),
            "seeds": list(SEEDS),
            "non_oracle_label": "SIMULATED heading sensors on MEASURED camera motion",
        },
        "metric_definitions": {
            "final_error_m": "Median across seeds of endpoint Euclidean position error.",
            "median_error_m": "Median across seeds of each seed's median post-cut Euclidean position error, including the zero-error cut sample.",
            "along_track_final_error_m": "Signed endpoint error projected along true travel direction over the final 200 m.",
            "cross_track_final_error_m": "Signed endpoint error projected onto the left-of-travel unit vector over the final 200 m.",
        },
        "results": rows,
        "shares_of_608m_error_removed": {
            "baseline_combination": "frozen_zoom+none",
            "baseline_error_m": baseline_error_m,
            **improvement_shares,
        },
        "limits": [
            "ALTO query rows are treated as one second apart for gyro drift, matching u2's Zurich residual replay assumption.",
            "Heading sensors are simulated additive yaw-error models applied to measured camera motion; they are not ALTO sensor recordings.",
            "The sign check uses a held-out 100-300 m segment within the pre-cut 300 m, with its map fitted only on the first 100 m.",
        ],
    }
    (OUTPUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    display = table.copy()
    for column in (
        "final_error_m", "median_error_m", "along_track_final_error_m", "cross_track_final_error_m"
    ):
        display[column] = display[column].map(lambda value: f"{value:+.1f}" if "track" in column else f"{value:.1f}")
    print("\nALTO heading variants (m; random variants summarized across 20 seeds):")
    print(display.to_string(index=False))
    print("\nShare of the frozen_zoom no-fix endpoint error removed:")
    for name, result in improvement_shares.items():
        print(f"  {name}: {100.0 * result['share_of_frozen_zoom_no_fix_error_removed']:.1f}%")
    print(f"Saved: {OUTPUT_DIR / 'summary.json'}")
    print(f"Saved: {OUTPUT_DIR / 'table.csv'}")
    archive.close()


if __name__ == "__main__":
    main()

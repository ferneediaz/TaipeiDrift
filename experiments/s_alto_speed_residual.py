"""Diagnose the ALTO along-track speed residual with oracle scale and heading.

Run from the repository root with:
    .venv/bin/python experiments/s_alto_speed_residual.py

Only the all-frame calibration uses post-cut truth; that variant is diagnostic,
not an online-usable estimator.
"""
from __future__ import annotations

import json

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

import s_alto_heading as heading
import u2_baro_dem_scale as u2

OUTPUT_DIR = u2.ROOT / "data/processed/alto_speed_residual"
BIN_WIDTH_M = 200.0
BASELINE_TARGET_M = 368.5072241535003


def fit_isotropic_map(flow: np.ndarray, targets: np.ndarray) -> np.ndarray:
    """Fit a two-parameter scale-times-rotation map to row-vector flows."""
    fx, fy = flow.T
    design = np.empty((2 * len(flow), 2), dtype=np.float64)
    design[0::2, 0] = fx
    design[0::2, 1] = -fy
    design[1::2, 0] = fy
    design[1::2, 1] = fx
    a, b = np.linalg.lstsq(design, targets.reshape(-1), rcond=None)[0]
    return np.array([[a, b], [-b, a]], dtype=np.float64)


def angle_degrees(values: np.ndarray) -> np.ndarray:
    """Interpret small roll/pitch values as radians, otherwise as degrees."""
    values = np.asarray(values, dtype=np.float64)
    finite = np.abs(values[np.isfinite(values)])
    if finite.size and np.percentile(finite, 90) <= 2.0 * np.pi:
        return np.degrees(values)
    return values


def tilt_series(query: pd.DataFrame, truth: np.ndarray) -> tuple[np.ndarray, str]:
    """Return roll/pitch tilt; derive local-NED Euler angles if columns are absent."""
    columns = {column.lower(): column for column in query.columns}
    if "roll" in columns and "pitch" in columns:
        roll = angle_degrees(query[columns["roll"]].to_numpy(dtype=np.float64))
        pitch = angle_degrees(query[columns["pitch"]].to_numpy(dtype=np.float64))
        source = "query.csv roll/pitch columns (units inferred from values)"
    else:
        lon, lat = u2.CRS_TRANSFORMER.transform(truth[:, 0], truth[:, 1])
        lon_rad, lat_rad = np.radians(lon), np.radians(lat)
        east = np.stack((-np.sin(lon_rad), np.cos(lon_rad), np.zeros_like(lon_rad)), axis=1)
        north = np.stack(
            (
                -np.sin(lat_rad) * np.cos(lon_rad),
                -np.sin(lat_rad) * np.sin(lon_rad),
                np.cos(lat_rad),
            ),
            axis=1,
        )
        up = np.stack(
            (
                np.cos(lat_rad) * np.cos(lon_rad),
                np.cos(lat_rad) * np.sin(lon_rad),
                np.sin(lat_rad),
            ),
            axis=1,
        )
        ned_basis_ecef = np.stack((north, east, -up), axis=2)
        quaternion = query[["orient_x", "orient_y", "orient_z", "orient_w"]].to_numpy(dtype=np.float64)
        body_to_ecef = Rotation.from_quat(quaternion).as_matrix()
        body_to_ned = np.einsum("nji,njk->nik", ned_basis_ecef, body_to_ecef)
        _, pitch, roll = Rotation.from_matrix(body_to_ned).as_euler("ZYX", degrees=True).T
        source = "query.csv orientation quaternion converted to local-NED aerospace Euler roll/pitch"
    return np.hypot(roll, pitch), source


def pearson(left: np.ndarray, right: np.ndarray) -> float | None:
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    valid = np.isfinite(left) & np.isfinite(right)
    if valid.sum() < 2 or np.ptp(left[valid]) == 0.0 or np.ptp(right[valid]) == 0.0:
        return None
    return float(np.corrcoef(left[valid], right[valid])[0, 1])


def slope_percent(distance_m: np.ndarray, elevation_m: np.ndarray) -> float | None:
    valid = np.isfinite(distance_m) & np.isfinite(elevation_m)
    if valid.sum() < 2 or np.ptp(distance_m[valid]) == 0.0:
        return None
    x = distance_m[valid] - np.mean(distance_m[valid])
    y = elevation_m[valid] - np.mean(elevation_m[valid])
    return float(np.dot(x, y) / np.dot(x, x) * 100.0)


def make_bins(
    travelled: np.ndarray,
    terrain: np.ndarray,
    jam: int,
    true_agl: np.ndarray,
    tilt_deg: np.ndarray,
    flow: np.ndarray,
    step_ratio: np.ndarray,
) -> pd.DataFrame:
    post_frames = np.arange(jam + 1, len(travelled), dtype=np.int64)
    distance = travelled[post_frames] - travelled[jam]
    bin_ids = np.floor(distance / BIN_WIDTH_M).astype(np.int64)
    rows = []
    for bin_id in np.unique(bin_ids):
        mask = bin_ids == bin_id
        frames = post_frames[mask]
        local_distance = distance[mask]
        slope = slope_percent(local_distance, terrain[frames])
        rows.append(
            {
                "bin_start_distance_m": float(bin_id * BIN_WIDTH_M),
                "bin_end_distance_m": float(min((bin_id + 1) * BIN_WIDTH_M, travelled[-1] - travelled[jam])),
                "frame_start": int(frames[0]),
                "frame_end": int(frames[-1]),
                "n_frames": int(mask.sum()),
                "step_ratio_median": float(np.median(step_ratio[mask])),
                "step_ratio_mean": float(np.mean(step_ratio[mask])),
                "true_agl_median": float(np.median(true_agl[frames])),
                "tilt_magnitude_deg_median": float(np.median(tilt_deg[frames])),
                "flow_magnitude_px_median": float(np.median(np.linalg.norm(flow[frames], axis=1))),
                "terrain_slope_along_pct": slope,
                "terrain_slope_abs_pct": abs(slope) if slope is not None else None,
            }
        )
    return pd.DataFrame(rows)


def main() -> None:
    archive, query, truth, travelled, jam = u2.load_alto()
    try:
        # The flow and DEM caches are read-only inputs; save=False avoids changing u2 outputs.
        flow = u2.image_motion(archive, query, len(query), save=False)
        dem, dem_transform = u2.load_dem(truth)
        terrain = u2.ground_height(truth, dem, dem_transform)
        altitude = query.altitude.to_numpy(dtype=np.float64)
        true_agl = altitude - terrain
        calibration_height = float(np.median(true_agl[: jam + 1]))
        yaw = heading.heading_truth(query, truth)
        tilt_deg, tilt_source = tilt_series(query, truth)

        true_steps = np.diff(truth, axis=0)
        baseline_map, *_ = np.linalg.lstsq(flow[1 : jam + 1], true_steps[:jam], rcond=None)
        isotropic_map = fit_isotropic_map(flow[1 : jam + 1], true_steps[:jam])

        # De-rotate truth into the cut-frame camera map coordinates, then remove
        # the same true-AGL scale used by the replay before fitting all frames.
        all_frames = np.arange(1, len(truth), dtype=np.int64)
        heading_correction = heading.HEADING_ROTATION_SIGN * (yaw[all_frames] - yaw[jam])
        canonical_steps = heading.rotate_xy(true_steps, -heading_correction)
        normalized_targets = canonical_steps / (true_agl[all_frames] / calibration_height)[:, None]
        oracle_map, *_ = np.linalg.lstsq(flow[1:], normalized_targets, rcond=None)

        variant_specs = (
            ("baseline", baseline_map, "full 2x2 least squares on first 300 m"),
            ("oracle_calibration", oracle_map, "full 2x2; all frames; true-AGL normalized; post-cut truth diagnostic only"),
            ("isotropic_300m", isotropic_map, "two-parameter scale × rotation on first 300 m"),
        )
        rows = []
        paths = {}
        step_ratios = {}
        for name, flow_to_ground, model in variant_specs:
            path = heading.simulate_heading(
                "true_agl", "heading_oracle", truth, jam, flow, flow_to_ground,
                calibration_height, true_agl, yaw,
            )[0]
            paths[name] = path
            estimated_steps = np.diff(path, axis=0)
            ratio = np.linalg.norm(estimated_steps, axis=1) / np.linalg.norm(true_steps[jam:], axis=1)
            step_ratios[name] = ratio
            metrics = heading.final_error_components(path, truth[jam:], truth, travelled)
            rows.append(
                {
                    "variant": name,
                    "evidence_label": "MEASURED ALTO flow + ORACLE true_AGL scale + ORACLE heading",
                    "calibration_model": model,
                    **metrics,
                    "median_step_ratio": float(np.median(ratio)),
                }
            )

        baseline_error = rows[0]["final_error_m"]
        if abs(baseline_error - BASELINE_TARGET_M) > 1.0:
            raise AssertionError(
                f"baseline reproduction failed: {baseline_error:.3f} m vs {BASELINE_TARGET_M:.3f} m target"
            )

        post_frames = np.arange(jam + 1, len(truth), dtype=np.int64)
        bins = make_bins(
            travelled, terrain, jam, true_agl, tilt_deg, flow, step_ratios["baseline"]
        )
        correlation_columns = {
            "true_agl_m": "true_agl_median",
            "tilt_magnitude_deg": "tilt_magnitude_deg_median",
            "flow_magnitude_px": "flow_magnitude_px_median",
            "terrain_slope_abs_pct": "terrain_slope_abs_pct",
            "terrain_slope_along_pct": "terrain_slope_along_pct",
        }
        correlations = {
            name: {
                "pearson_r": pearson(bins["step_ratio_median"].to_numpy(), bins[column].to_numpy()),
                "n_bins": int(len(bins)),
                "method": "Pearson correlation across 200 m bin medians; ratio is baseline estimated/true step norm",
            }
            for name, column in correlation_columns.items()
        }
        scored_candidates = {
            name: result["pearson_r"]
            for name, result in correlations.items()
            if result["pearson_r"] is not None
        }
        strongest_name = max(scored_candidates, key=lambda name: abs(scored_candidates[name])) if scored_candidates else None
        strongest = (
            {"candidate": strongest_name, "pearson_r": scored_candidates[strongest_name]}
            if strongest_name is not None
            else None
        )

        summary = {
            "experiment": "ALTO speed residual with oracle AGL scale and heading",
            "command": ".venv/bin/python experiments/s_alto_speed_residual.py",
            "source_zip": str(u2.ZIP_PATH.relative_to(u2.ROOT)),
            "evidence_label": "MEASURED ALTO camera motion; ORACLE true-AGL scale and heading in every replay",
            "cut_frame": int(jam),
            "cut_distance_m": float(travelled[jam]),
            "post_cut_distance_m": float(travelled[-1] - travelled[jam]),
            "post_cut_frames": int(len(query) - jam - 1),
            "calibration_height_m": calibration_height,
            "baseline_reproduction_target_m": BASELINE_TARGET_M,
            "baseline_reproduction_difference_m": float(abs(baseline_error - BASELINE_TARGET_M)),
            "variants": rows,
            "step_ratio_diagnostic": {
                "variant": "baseline",
                "definition": "norm(estimated post-cut step) / norm(diff of truth positions); one sample per post-cut frame",
                "bin_width_m": BIN_WIDTH_M,
                "tilt_source": tilt_source,
                "terrain_slope_method": "OLS elevation-vs-travelled-distance slope within each 200 m bin; absolute value used for slope-magnitude correlation",
                "correlations_across_bin_medians": correlations,
                "strongest_absolute_correlation": strongest,
                "bin_count": int(len(bins)),
            },
            "oracle_calibration_note": "Uses all route frames, including post-cut truth, after undoing the oracle heading rotation and dividing truth steps by true_agl/calibration_height. Diagnostic only; not a deployable calibration.",
            "metric_definitions": {
                "final_error_m": "Endpoint Euclidean error against truth.",
                "along_track_final_error_m": "Signed endpoint error projected along true travel direction over the final 200 m, matching s_alto_heading.py.",
                "cross_track_final_error_m": "Signed endpoint error projected onto the left-of-travel unit vector over the final 200 m, matching s_alto_heading.py.",
                "median_step_ratio": "Median post-cut norm(estimated step) / norm(true step). Values below 1 indicate camera-speed underestimation.",
            },
        }

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        bins.to_csv(OUTPUT_DIR / "bins.csv", index=False)

        variant_table = pd.DataFrame(rows)
        print("ALTO speed residual variants (true_AGL + heading_oracle for all):")
        print(
            variant_table[
                ["variant", "final_error_m", "along_track_final_error_m", "cross_track_final_error_m", "median_step_ratio"]
            ].to_string(index=False, float_format=lambda value: f"{value:.3f}")
        )
        correlation_table = pd.DataFrame(
            [
                {"candidate": name, "pearson_r": result["pearson_r"], "n_bins": result["n_bins"]}
                for name, result in correlations.items()
            ]
        )
        print("\nStep-ratio correlations across 200 m bin medians:")
        print(correlation_table.to_string(index=False, float_format=lambda value: f"{value:.3f}"))
        print("\nBaseline step-ratio bin table:")
        print(bins.to_string(index=False, float_format=lambda value: f"{value:.3f}"))
        print(f"\nStrongest absolute correlate: {strongest}")
        print(f"Saved: {OUTPUT_DIR / 'summary.json'}")
        print(f"Saved: {OUTPUT_DIR / 'bins.csv'}")
    finally:
        archive.close()


if __name__ == "__main__":
    main()

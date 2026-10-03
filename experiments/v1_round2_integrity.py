"""Exploratory integrity and baro-DEM zoom priors on ALTO Round 2 Train.

NOT HELD-OUT: the earlier diagnosis already inspected all eight Round 2 sections.
The frozen state machine and its pre-cut calibration are imported from
experiments/t_alto_heldout.py. No post-cut truth enters a matching decision.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import rasterio
from pyproj import Transformer
from rasterio.merge import merge

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "experiments"))
import r_alto_heldout_quad as Q  # noqa: E402
import r_map_benchmark as B  # noqa: E402
import t_alto_heldout as H  # noqa: E402

OUT_DIR = ROOT / "data/processed/v1_round2_integrity"
DEM_CACHE = OUT_DIR / "alto_train_glo30.npz"
VERTICAL_RESIDUALS = ROOT / "data/processed/zurich_vertical/replay_errors.csv"
SCOPE = (
    "EXPLORATORY / NOT HELD-OUT: all eight ALTO Round 2 Train sections were already "
    "inspected by the prior diagnosis."
)
EVIDENCE_LABEL = (
    "MEASURED ALTO; SIMULATED Zurich baro residual + Copernicus GLO-30 AGL zoom prior; "
    "INFERENCE proportional-AGL zoom model."
)
DEM_MARGIN_DEG = 0.05
ZOOM_MIN, ZOOM_MAX, ZOOM_STEP = 0.35, 1.20, 0.05
ZOOM_HALF_WIDTH = 0.40
QUAD_AGREE_M = 4.0


class DemMap:
    def __init__(self, data: np.ndarray, affine: np.ndarray, transformer: Transformer):
        self.data = np.asarray(data, dtype=np.float32)
        self.affine = np.asarray(affine, dtype=np.float64)
        self.transformer = transformer
        self.a, _, self.c, _, self.e, self.f = self.affine

    def sample(self, xy: np.ndarray) -> np.ndarray:
        points = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        lon, lat = self.transformer.transform(points[:, 0], points[:, 1])
        col = (np.asarray(lon) - self.c) / self.a - 0.5
        row = (np.asarray(lat) - self.f) / self.e - 0.5
        inside = (
            np.isfinite(col)
            & np.isfinite(row)
            & (col >= 0)
            & (col <= self.data.shape[1] - 1)
            & (row >= 0)
            & (row <= self.data.shape[0] - 1)
        )
        if not bool(np.all(inside)):
            raise ValueError(
                "Copernicus DEM crop does not cover an estimated position; "
                "increase DEM_MARGIN_DEG and rerun."
            )
        values = cv2.remap(
            self.data,
            col.astype(np.float32).reshape(-1, 1),
            row.astype(np.float32).reshape(-1, 1),
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        ).ravel()
        if not np.isfinite(values).all() or np.any(values < -1000):
            raise ValueError("Copernicus DEM returned non-finite or nodata elevations")
        return values


def _dem_tile_path(lat_degree: int, lon_degree: int) -> str:
    tile = (
        f"Copernicus_DSM_COG_10_N{lat_degree:02d}_00_"
        f"W{abs(lon_degree):03d}_00_DEM"
    )
    return f"/vsicurl/https://copernicus-dem-30m.s3.amazonaws.com/{tile}/{tile}.tif"


def load_train_dem(truth_xy: np.ndarray) -> tuple[DemMap, dict]:
    """Load a GLO-30 crop using the same Copernicus COG grid as i_alto_zoom_check."""
    transformer = Transformer.from_crs("EPSG:32617", "EPSG:4326", always_xy=True)
    if DEM_CACHE.is_file():
        cached = np.load(DEM_CACHE)
        data = cached["data"]
        affine = cached["t"]
        bounds = cached["bounds"].tolist()
        tile_names = cached["tiles"].astype(str).tolist()
        print(f"Using cached Copernicus GLO-30 crop: {DEM_CACHE}", flush=True)
        return DemMap(data, affine, transformer), {
            "cache": str(DEM_CACHE.relative_to(ROOT)),
            "bounds_wsen_lonlat": bounds,
            "tiles": tile_names,
            "source": "Copernicus GLO-30 COG via copernicus-dem-30m.s3.amazonaws.com",
            "crop_margin_deg": DEM_MARGIN_DEG,
        }

    lon, lat = transformer.transform(truth_xy[:, 0], truth_xy[:, 1])
    west = float(np.min(lon) - DEM_MARGIN_DEG)
    east = float(np.max(lon) + DEM_MARGIN_DEG)
    south = float(np.min(lat) - DEM_MARGIN_DEG)
    north = float(np.max(lat) + DEM_MARGIN_DEG)
    lon_degrees = range(math.floor(west), math.floor(east) + 1)
    lat_degrees = range(math.floor(south), math.floor(north) + 1)
    tile_urls = [
        (lat_degree, lon_degree, _dem_tile_path(lat_degree, lon_degree))
        for lat_degree in lat_degrees
        for lon_degree in lon_degrees
    ]

    sources = []
    try:
        for _, _, url in tile_urls:
            print(f"Opening Copernicus GLO-30: {url}", flush=True)
            sources.append(rasterio.open(url))
        mosaic, transform = merge(
            sources,
            bounds=(west, south, east, north),
            nodata=np.nan,
            dtype="float32",
        )
    finally:
        for source in sources:
            source.close()

    data = np.asarray(mosaic[0], dtype=np.float32)
    if not data.size or not np.isfinite(data).any():
        raise ValueError("Copernicus GLO-30 returned an empty DEM crop")
    affine = np.array(
        [transform.a, transform.b, transform.c, transform.d, transform.e, transform.f],
        dtype=np.float64,
    )
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        DEM_CACHE,
        data=data,
        t=affine,
        bounds=np.array([west, south, east, north], dtype=np.float64),
        tiles=np.array([f"N{lat:02d}_W{abs(lon):03d}" for lat, lon, _ in tile_urls]),
        evidence_label=np.array(EVIDENCE_LABEL),
        evidence_scope=np.array(SCOPE),
    )
    details = {
        "cache": str(DEM_CACHE.relative_to(ROOT)),
        "bounds_wsen_lonlat": [west, south, east, north],
        "tiles": [url.rsplit("/", 1)[-1].removesuffix(".tif") for _, _, url in tile_urls],
        "source": "Copernicus GLO-30 COG via copernicus-dem-30m.s3.amazonaws.com",
        "crop_margin_deg": DEM_MARGIN_DEG,
    }
    return DemMap(data, affine, transformer), details


def load_residual_profile() -> tuple[np.ndarray, np.ndarray, dict]:
    """Select the earliest Zurich baro_only profile with the longest measured horizon."""
    if not VERTICAL_RESIDUALS.is_file():
        raise FileNotFoundError(f"Required Zurich residual data missing: {VERTICAL_RESIDUALS}")
    data = pd.read_csv(VERTICAL_RESIDUALS)
    rows = data[data.variant == "baro_only"].dropna(
        subset=["cut_s", "horizon_s", "error_m"]
    )
    profiles = []
    for cut_s, group in rows.groupby("cut_s", sort=True):
        group = group.sort_values("horizon_s")
        profiles.append((float(group.horizon_s.max()), float(cut_s), group))
    if not profiles:
        raise ValueError("No Zurich baro_only replay residuals found")
    max_horizon = max(item[0] for item in profiles)
    horizon_s, cut_s, profile = min(
        (item for item in profiles if item[0] == max_horizon),
        key=lambda item: item[1],
    )
    horizons = profile.horizon_s.to_numpy(dtype=np.float64)
    residuals = profile.error_m.to_numpy(dtype=np.float64)
    source_times = np.r_[0.0, horizons]
    source_errors = np.r_[0.0, residuals]
    detail = {
        "evidence_label": "SIMULATED (empirical residual profile replayed from measured Zurich baro_only errors)",
        "source": str(VERTICAL_RESIDUALS.relative_to(ROOT)),
        "source_variant": "baro_only",
        "source_cut_s": cut_s,
        "horizon_s": horizons.tolist(),
        "residual_m": residuals.tolist(),
        "timebase": "ALTO query timestamp nanoseconds; reset residual to 0 at each section's 300 m GNSS cut; piecewise linear; hold final Zurich residual past its last horizon.",
    }
    return source_times, source_errors, detail


def simulate_baro(
    dataset: H.AltoDataset,
    section: H.PreparedRange,
    residual_times: np.ndarray,
    residual_errors: np.ndarray,
) -> np.ndarray:
    """Build query altitude plus Zurich residual, replayed at cut-relative seconds."""
    i0 = section.i0
    i1 = section.i1
    query = dataset.query.iloc[i0:i1]
    altitude = query.altitude.to_numpy(dtype=np.float64).copy()
    timestamps = query.timestamp.to_numpy(dtype=np.int64)
    elapsed = (timestamps - timestamps[section.jam]).astype(np.float64) * 1e-9
    post_cut = np.arange(len(query)) > section.jam
    altitude[post_cut] += np.interp(
        elapsed[post_cut],
        residual_times,
        residual_errors,
        left=0.0,
        right=float(residual_errors[-1]),
    )
    return altitude


def _zoom_grid(predicted_zoom: float) -> np.ndarray:
    offsets = np.arange(-ZOOM_HALF_WIDTH, ZOOM_HALF_WIDTH + 1e-9, ZOOM_STEP)
    grid = np.clip(predicted_zoom + offsets, ZOOM_MIN, ZOOM_MAX)
    return np.unique(np.round(grid, 8))


def _legacy_zoom_grid(zoom: float, since_fix_m: float) -> np.ndarray:
    zooms = np.clip(zoom + np.arange(-0.10, 0.11, 0.05), 0.5, 1.1)
    if since_fix_m > 400.0:
        zooms = np.arange(0.60, 1.101, 0.05)
    return zooms


def _reference_candidates(
    dataset: H.AltoDataset,
    estimate: np.ndarray,
    predicted_var: float,
) -> np.ndarray:
    candidates = dataset.nearest(estimate, H.CALIB_NEAREST)
    radius = max(60.0, 3.0 * math.sqrt(predicted_var))
    inside = np.where(np.linalg.norm(dataset.ref_xy - estimate, axis=1) <= radius)[0]
    return inside if len(inside) >= H.CALIB_NEAREST else candidates


def _agl_zoom_prior(
    estimate: np.ndarray,
    baro_altitude_m: float,
    calibration_agl_m: float,
    calibration_zoom: float,
    dem: DemMap,
) -> tuple[float, float]:
    ground_m = float(dem.sample(estimate.reshape(1, 2))[0])
    agl_m = float(baro_altitude_m - ground_m)
    if not np.isfinite(agl_m) or agl_m <= 0.0:
        raise ValueError(f"Invalid baro-DEM AGL {agl_m} m at estimated position")
    predicted_zoom = float(calibration_zoom * agl_m / calibration_agl_m)
    return predicted_zoom, agl_m


def _quad_check_at_full_match(
    dataset: H.AltoDataset,
    absolute_k: int,
    full_match_position: np.ndarray,
    matched_zoom: float,
    matched_angle: float,
    valid_cache: dict[str, np.ndarray],
) -> tuple[int, float]:
    """Check four independent query patches against the selected H full-ZNCC fix."""
    ref_index = int(dataset.nearest(full_match_position, 1)[0])
    reference, valid = Q._reference_window(
        dataset, ref_index, full_match_position, valid_cache
    )
    member = f"{dataset.prefix}/query_images/{dataset.query.name.iloc[absolute_k]}"
    frame = dataset.load_image(member)
    query = Q._query160(frame, matched_zoom, matched_angle)
    full_point = np.array([B.SEARCH / 2.0, B.SEARCH / 2.0], dtype=np.float64)
    stats = B.quad_consensus(
        query,
        reference,
        valid,
        full_point,
        tol=QUAD_AGREE_M / H.REF_MPP,
    )
    third_best_m = float(stats["quad_d3"] * H.REF_MPP)
    return int(stats["quad_n"]), third_best_m

def _run_variant(
    dataset: H.AltoDataset,
    section: H.PreparedRange,
    config: H.RunConfig,
    *,
    variant: str,
    use_quad: bool,
    use_zoom_prior: bool,
    dem: DemMap,
    residual_times: np.ndarray,
    residual_errors: np.ndarray,
    valid_cache: dict[str, np.ndarray],
) -> tuple[H.RunResult, list[dict]]:
    """Run the H state machine with only requested fix-acceptance/search changes."""
    estimate = section.truth[section.jam].copy()
    path = [estimate.copy()]
    variance = 3.0**2
    since_fix_m = 0.0
    since_try_m = 0.0
    zoom = section.zoom0
    scale = 1.0
    fix_log: list[tuple[float, float, float, float]] = []
    detail_rows: list[dict] = []
    baro_altitude = simulate_baro(dataset, section, residual_times, residual_errors)
    pre_cut_xy = section.truth[: section.jam + 1]
    pre_cut_ground = dem.sample(pre_cut_xy)
    raw_query_altitude = dataset.query.altitude.to_numpy(dtype=np.float64)[
        section.i0 : section.i0 + section.jam + 1
    ]
    calibration_agl = float(np.median(raw_query_altitude - pre_cut_ground))
    if not np.isfinite(calibration_agl) or calibration_agl <= 0.0:
        raise ValueError(f"Invalid pre-cut median baro-DEM AGL {calibration_agl} m")

    for local_k in range(section.jam + 1, len(section.truth)):
        step = dataset.flow[section.i0 + local_k] @ section.A0 * scale
        estimate = estimate + step
        step_m = float(np.linalg.norm(step))
        since_fix_m += step_m
        since_try_m += step_m
        if config.fix_every_m and since_try_m >= config.fix_every_m:
            since_try_m = 0.0
            predicted_var = variance + (H.DRIFT_RATE * since_fix_m) ** 2
            candidates = _reference_candidates(dataset, estimate, predicted_var)
            predicted_zoom = math.nan
            agl_m = math.nan
            if use_zoom_prior:
                predicted_zoom, agl_m = _agl_zoom_prior(
                    estimate,
                    float(baro_altitude[local_k]),
                    calibration_agl,
                    section.zoom0,
                    dem,
                )
                zooms = _zoom_grid(predicted_zoom)
            else:
                zooms = _legacy_zoom_grid(zoom, since_fix_m)

            quad_n = 0
            quad_3rd_m = math.nan
            score, position, new_zoom, new_angle = H.match_fix(
                dataset,
                section.i0 + local_k,
                zooms,
                np.array([section.angle0 - 5, section.angle0, section.angle0 + 5]),
                candidates,
            )
            finite = position is not None and np.isfinite(position).all()
            fix_error = math.nan
            agrees = False
            use = False
            if finite:
                full_match_position = np.asarray(position, dtype=np.float64)
                if use_quad:
                    quad_n, quad_3rd_m = _quad_check_at_full_match(
                        dataset,
                        section.i0 + local_k,
                        full_match_position,
                        new_zoom,
                        new_angle,
                        valid_cache,
                    )
                position = full_match_position - section.offset0
                fix_error = float(np.linalg.norm(position - section.truth[local_k]))
                agrees = bool(
                    np.linalg.norm(position - estimate)
                    <= 3.0 * math.sqrt(predicted_var + H.FIX_SIGMA**2)
                )
                if use_quad:
                    use = bool(agrees and quad_n >= 3)
                else:
                    use = bool(agrees and score >= config.min_score)
            fix_log.append(
                (
                    float(section.i0 + local_k),
                    float(score),
                    fix_error,
                    float(use),
                )
            )
            detail_rows.append(
                {
                    "evidence_scope": SCOPE,
                    "evidence_label": EVIDENCE_LABEL,
                    "section": "",
                    "variant": variant,
                    "spacing_m": float(config.fix_every_m),
                    "frame": int(section.i0 + local_k),
                    "score": float(score),
                    "fix_error_m_scoring_only": fix_error,
                    "innovation_gate_pass": agrees,
                    "quad_agree_count_4m": quad_n if use_quad else math.nan,
                    "quad_third_best_distance_m": quad_3rd_m if use_quad else math.nan,
                    "baro_dem_agl_m_simulated": agl_m,
                    "zoom_prior": predicted_zoom,
                    "zoom_search_min": float(np.min(zooms)),
                    "zoom_search_max": float(np.max(zooms)),
                    "accepted": use,
                }
            )
            if use:
                gain = predicted_var / (predicted_var + H.FIX_SIGMA**2)
                estimate = estimate + gain * (position - estimate)
                variance = (1.0 - gain) * predicted_var
                scale, zoom, since_fix_m = new_zoom / section.zoom0, new_zoom, 0.0
        path.append(estimate.copy())

    path_array = np.asarray(path)
    errors = np.linalg.norm(path_array - section.truth[section.jam :], axis=1)
    result = H.RunResult(path_array, errors, np.asarray(fix_log, dtype=float).reshape(-1, 4))
    return result, detail_rows


def _metrics(
    section_name: str,
    section: H.PreparedRange,
    km: float,
    variant: str,
    spacing_m: float,
    result: H.RunResult,
) -> dict:
    used, rejected, wrong_gt50, _ = H.error_counts(result)
    return {
        "evidence_scope": SCOPE,
        "evidence_label": EVIDENCE_LABEL,
        "section": section_name,
        "i0": section.i0,
        "i1": section.i1,
        "section_km": km,
        "variant": variant,
        "spacing_m": spacing_m,
        "median_error_m": float(np.median(result.error)),
        "end_error_m": float(result.error[-1]),
        "fixes_attempted": int(used + rejected),
        "fixes_used": used,
        "fixes_rejected": rejected,
        "accepted_wrong_gt50_m": wrong_gt50,
    }


def _reproduce_baseline(
    dataset: H.AltoDataset,
    sections: list[tuple[str, H.PreparedRange, float]],
) -> list[dict]:
    saved_path = ROOT / H.OUT_DIR / "results.csv"
    if not saved_path.is_file():
        raise FileNotFoundError(f"Frozen t_alto_heldout output required: {saved_path}")
    saved = pd.read_csv(saved_path)
    baseline_rows: list[dict] = []
    configs = {300.0: "sized_gate_300", 1000.0: "sized_gate_1000"}
    h_configs = {config.name: config for config in H.train_section_configurations()}
    for section_name, section, km in sections:
        for spacing_m, config_name in configs.items():
            config = h_configs[config_name]
            result = H.run_config(dataset, section, config)
            row = _metrics(section_name, section, km, "frozen_team_chain", spacing_m, result)
            expected = saved[(saved.section == section_name) & (saved.config == config_name)]
            if len(expected) != 1:
                raise RuntimeError(f"Expected exactly one t_alto_heldout row for {section_name}/{config_name}")
            target = expected.iloc[0]
            checks = (
                (row["median_error_m"], float(target["median"])),
                (row["end_error_m"], float(target["end"])),
                (row["fixes_used"], int(target["used"])),
                (row["fixes_rejected"], int(target["rejected"])),
                (row["accepted_wrong_gt50_m"], int(target["wrong_gt50"])),
            )
            if not all(np.isclose(actual, wanted, rtol=0.0, atol=1e-6) for actual, wanted in checks):
                raise RuntimeError(
                    f"Frozen baseline mismatch {section_name}/{config_name}: {checks}"
                )
            baseline_rows.append(row)
            print(
                f"Baseline reproduction PASS {section_name} {int(spacing_m)} m: "
                f"median={row['median_error_m']:.2f} m, end={row['end_error_m']:.2f} m, "
                f"used/rejected/wrong>50={row['fixes_used']}/{row['fixes_rejected']}/"
                f"{row['accepted_wrong_gt50_m']}",
                flush=True,
            )
    print("Frozen t_alto_heldout per-section reproduction PASS (absolute tolerance 1e-6).", flush=True)
    return baseline_rows


def _summaries(rows: list[dict]) -> list[dict]:
    frame = pd.DataFrame(rows)
    output = []
    for (variant, spacing), group in frame.groupby(["variant", "spacing_m"], sort=False):
        median_values = group.median_error_m.to_numpy(dtype=np.float64)
        end_values = group.end_error_m.to_numpy(dtype=np.float64)
        output.append(
            {
                "evidence_scope": SCOPE,
                "evidence_label": EVIDENCE_LABEL,
                "variant": str(variant),
                "spacing_m": int(spacing),
                "sections": int(len(group)),
                "section_median_error_m_median": float(np.median(median_values)),
                "section_median_error_m_min": float(np.min(median_values)),
                "section_median_error_m_max": float(np.max(median_values)),
                "section_end_error_m_median": float(np.median(end_values)),
                "section_end_error_m_min": float(np.min(end_values)),
                "section_end_error_m_max": float(np.max(end_values)),
                "fixes_attempted_total": int(group.fixes_attempted.sum()),
                "fixes_used_total": int(group.fixes_used.sum()),
                "fixes_rejected_total": int(group.fixes_rejected.sum()),
                "accepted_wrong_gt50_total": int(group.accepted_wrong_gt50_m.sum()),
            }
        )
    return output


def main() -> None:
    if Path.cwd().resolve() != ROOT:
        raise SystemExit(f"Run from the repository root: cd {ROOT} && .venv/bin/python experiments/v1_round2_integrity.py")
    if not (ROOT / H.TRAIN_ZIP).is_file():
        raise FileNotFoundError(f"ALTO Round 2 Train archive missing: {ROOT / H.TRAIN_ZIP}")
    if not VERTICAL_RESIDUALS.is_file():
        raise FileNotFoundError(f"Zurich residual data missing: {VERTICAL_RESIDUALS}")

    cv2.setNumThreads(1)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print(SCOPE, flush=True)
    print("Evidence: ALTO replay/matches MEASURED; baro residual + zoom prior SIMULATED.", flush=True)

    dataset = H.AltoDataset(ROOT / H.TRAIN_ZIP, "Train")
    valid_cache: dict[str, np.ndarray] = {}
    try:
        dataset.image_motion()
        full_travelled = np.r_[
            0.0,
            np.cumsum(np.linalg.norm(np.diff(dataset.truth, axis=0), axis=1)),
        ]
        ranges = H.section_ranges(full_travelled)
        if len(ranges) != 8:
            raise RuntimeError(f"Expected 8 previously inspected Round 2 sections, found {len(ranges)}")
        sections = []
        for number, (i0, i1) in enumerate(ranges, 1):
            section = H.prepare_range(dataset, (i0, i1))
            km = float((full_travelled[i1 - 1] - full_travelled[i0]) / 1000.0)
            sections.append((f"section_{number}", section, km))
        # First reproduce the earlier held-out script's exact target policies before any variants.
        baseline_rows = _reproduce_baseline(dataset, sections)

        dem, dem_detail = load_train_dem(dataset.truth)
        residual_times, residual_errors, residual_detail = load_residual_profile()
        variant_specs = (
            ("quad_ge3", True, False),
            ("baro_dem_zoom_prior", False, True),
            ("quad_ge3_baro_dem_zoom_prior", True, True),
        )
        variant_rows: list[dict] = []
        fix_rows: list[dict] = []
        config_by_spacing = {
            int(config.fix_every_m): config
            for config in H.train_section_configurations()
            if config.name in ("sized_gate_300", "sized_gate_1000")
        }
        for section_name, section, km in sections:
            for spacing_m in (300, 1000):
                config = config_by_spacing[spacing_m]
                for variant, use_quad, use_zoom_prior in variant_specs:
                    started = time.perf_counter()
                    result, details = _run_variant(
                        dataset,
                        section,
                        config,
                        variant=variant,
                        use_quad=use_quad,
                        use_zoom_prior=use_zoom_prior,
                        dem=dem,
                        residual_times=residual_times,
                        residual_errors=residual_errors,
                        valid_cache=valid_cache,
                    )
                    runtime_s = time.perf_counter() - started
                    row = _metrics(section_name, section, km, variant, spacing_m, result)
                    row["runtime_s"] = runtime_s
                    variant_rows.append(row)
                    for detail in details:
                        detail["section"] = section_name
                        fix_rows.append(detail)
                    print(
                        f"{variant} {section_name} {spacing_m} m: median={row['median_error_m']:.2f} m, "
                        f"end={row['end_error_m']:.2f} m, used/rejected/wrong>50="
                        f"{row['fixes_used']}/{row['fixes_rejected']}/{row['accepted_wrong_gt50_m']}; "
                        f"{runtime_s:.1f} s",
                        flush=True,
                    )
    finally:
        dataset.close()

    all_rows = baseline_rows + variant_rows
    summary_rows = _summaries(all_rows)
    sections_path = OUT_DIR / "sections.csv"
    summary_path = OUT_DIR / "summary.csv"
    fixes_path = OUT_DIR / "fixes.csv"
    pd.DataFrame(all_rows).to_csv(sections_path, index=False)
    pd.DataFrame(summary_rows).to_csv(summary_path, index=False)
    pd.DataFrame(fix_rows).to_csv(fixes_path, index=False)
    summary_json = {
        "experiment": "ALTO Round 2 Train exploratory quad integrity and simulated baro-DEM zoom prior",
        "evidence_scope": SCOPE,
        "held_out": False,
        "explicit_limit": "This is not held-out replication: the previous Round 2 diagnosis already inspected all eight sections.",
        "evidence_labels": {
            "measured": "ALTO Round 2 Train imagery, reference tiles, query altitude/position metadata and post-cut ground truth used for evaluation only.",
            "simulated": "Barometer = ALTO query altitude + Zurich measured baro_only replay error; Copernicus GLO-30 terrain and AGL-proportional zoom prior.",
            "inference": "Zoom is assumed proportional to AGL relative to the median pre-cut AGL and the frozen section zoom calibration.",
        },
        "command": ".venv/bin/python experiments/v1_round2_integrity.py",
        "baseline_check": {
            "source": "experiments/t_alto_heldout.py run_config; data/processed/t_alto_heldout/results.csv",
            "target_configs": ["sized_gate_300", "sized_gate_1000"],
            "matching_fields": ["median_error_m", "end_error_m", "fixes_used", "fixes_rejected", "accepted_wrong_gt50_m"],
            "absolute_tolerance": 1e-6,
            "passed": True,
        },
        "sections": 8,
        "fix_spacings_m": [300, 1000],
        "policy_definitions": {
            "frozen_team_chain": "Exact imported H.run_config sized_gate_{300,1000}; includes the frozen 0.33 score gate.",
            "quad_ge3": "Keep the frozen full-tile H.match_fix and run r_alto_heldout_quad's Q._reference_window plus r_map_benchmark's B.quad_consensus at its selected fix; accept when >=3 disjoint subtemplates agree with that full ZNCC position within 4 m and the unchanged innovation gate passes; no 0.33 score gate.",
            "baro_dem_zoom_prior": "Frozen full-tile H.match_fix and 0.33 score gate; search zoom centered at section.zoom0*(simulated baro minus DEM at propagated estimate)/(median pre-cut baro minus DEM at pre-cut GNSS truth), +/-0.40 in 0.05 steps clipped to [0.35,1.20].",
            "quad_ge3_baro_dem_zoom_prior": "Same full H match, four-subtemplate 4 m integrity gate and simulated AGL-centered zoom search; no 0.33 score gate.",
        },
        "simulated_baro": residual_detail,
        "dem": dem_detail,
        "filter_contract": "Only H's pre-cut GNSS initialization/calibration and optical-flow propagation; post-cut fix decisions use propagated position, images/reference tiles, configured gates and (where selected) simulated baro + DEM. Post-cut truth is read only for error scoring.",
        "aggregates": summary_rows,
        "outputs": [str(sections_path.relative_to(ROOT)), str(summary_path.relative_to(ROOT)), str(fixes_path.relative_to(ROOT))],
    }
    json_path = OUT_DIR / "summary.json"
    json_path.write_text(json.dumps(summary_json, indent=2) + "\n", encoding="utf-8")

    print(f"\n{SCOPE}", flush=True)
    print("Section medians (median/end errors) and pooled fix counts:", flush=True)
    for row in summary_rows:
        print(
            f"  {row['variant']:34s} {row['spacing_m']:4d} m: "
            f"median {row['section_median_error_m_median']:.1f} m "
            f"[{row['section_median_error_m_min']:.1f}, {row['section_median_error_m_max']:.1f}], "
            f"end {row['section_end_error_m_median']:.1f} m "
            f"[{row['section_end_error_m_min']:.1f}, {row['section_end_error_m_max']:.1f}], "
            f"used/rejected/wrong>50={row['fixes_used_total']}/{row['fixes_rejected_total']}/"
            f"{row['accepted_wrong_gt50_total']}",
            flush=True,
        )
    print(f"Wrote {sections_path}, {summary_path}, {fixes_path}, {json_path}", flush=True)


if __name__ == "__main__":
    main()

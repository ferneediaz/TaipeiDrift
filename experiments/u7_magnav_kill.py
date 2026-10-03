"""Kill-test coarse EMAG2v3 anomaly-map matching for Taiwan.

Run from the repository root with:
    .venv/bin/python experiments/u7_magnav_kill.py

The grid-derived gradients are evaluated on published NOAA data, not flight
measurements. The correlation experiment is explicitly SIMULATED: it samples
the grid along an offset/heading-perturbed path and adds white Gaussian noise.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from urllib.request import Request, urlopen

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rasterio
from pyproj import Geod
from rasterio.windows import Window
from scipy.ndimage import map_coordinates

ROOT = Path(__file__).resolve().parents[1]
RAW_DIR = ROOT / "data/raw/emag2v3"
OUT_DIR = ROOT / "data/processed/u7_magnav_kill"
GRID_FILENAMES = {
    "sea_level": "EMAG2_V3_20170530_Sealevel.tif",
    "upward_continued": "EMAG2_V3_UpCont_DataTiff.tif",
}
GRID_URLS = {
    "sea_level": "https://www.ngdc.noaa.gov/geomag/data/EMAG2/EMAG2_V3_20170530/EMAG2_V3_20170530_Sealevel.tif",
    "upward_continued": "https://www.ngdc.noaa.gov/geomag/data/EMAG2/EMAG2_V3_UpCont_DataTiff.tif",
}
NOAA_PRODUCT_URL = "https://www.ncei.noaa.gov/products/earth-magnetic-model-anomaly-grid-2"
NOAA_DOWNLOAD_URL = "https://www.ncei.noaa.gov/products/earth-magnetic-model-anomaly-grid-2/download-data"
NOAA_METADATA_URL = "https://doi.org/10.7289/V5H70CVX"
GRID_README_URL = "https://www.ngdc.noaa.gov/geomag/data/EMAG2/EMAG2_readme.txt"
UAV_INTERFERENCE_URL = "https://doi.org/10.5194/gi-10-101-2021"
GEOD = Geod(ellps="WGS84")

BUDai = (120.13, 23.38)  # longitude, latitude; approx. Budai
MAGONG = (119.58, 23.57)  # longitude, latitude; approx. Magong
TRACK_LENGTH_M = 50_000.0
PROFILE_SPACING_M = 250.0
SEARCH_LIMIT_M = 5_000.0
HEADING_ERRORS_DEG = (-5.0, -3.0, -1.0, 0.0, 1.0, 3.0, 5.0)
DEFAULT_SIGMA_B_NT = 20.0
DEFAULT_TRIALS = 100
DEFAULT_SEED = 20261003


class GeoTiffSampler:
    """Bilinearly sample a small local window from a global north-up GeoTIFF."""

    def __init__(self, path: Path):
        self.path = path
        self.dataset = rasterio.open(path)
        self.bounds = self.dataset.bounds
        if self.dataset.crs is not None and self.dataset.crs.to_epsg() != 4326:
            raise ValueError(f"Expected an EPSG:4326 grid, got {self.dataset.crs}")
        if self.dataset.crs is None and not (
            -180.0 <= self.bounds.left <= 0.0
            and 180.0 <= self.bounds.right <= 360.0
            and -90.0 <= self.bounds.bottom < self.bounds.top <= 90.0
        ):
            raise ValueError(f"Missing CRS tag and non-global lon/lat bounds: {self.bounds}")
        if self.dataset.count != 1:
            raise ValueError(f"Expected a single-band anomaly grid, got {self.dataset.count} bands")
        self.inverse = ~self.dataset.transform
        self.width = self.dataset.width
        self.height = self.dataset.height
        self.resolution_deg = (abs(self.dataset.res[0]), abs(self.dataset.res[1]))
        self.nodata = self.dataset.nodata
        self.crs_description = (
            self.dataset.crs.to_string()
            if self.dataset.crs is not None
            else "EPSG:4326 inferred from NOAA metadata and global lon/lat bounds (GeoTIFF CRS tag absent)"
        )
        if self.dataset.transform.b != 0 or self.dataset.transform.d != 0:
            raise ValueError("Rotated GeoTIFF grids are not supported by this sampler")

    def close(self) -> None:
        self.dataset.close()

    def metadata(self) -> dict:
        return {
            "file": str(self.path.resolve().relative_to(ROOT)),
            "width_px": self.width,
            "height_px": self.height,
            "crs": self.crs_description,
            "crs_tag_present": self.dataset.crs is not None,
            "resolution_deg": list(self.resolution_deg),
            "resolution_arcmin": [value * 60.0 for value in self.resolution_deg],
            "bounds": [self.bounds.left, self.bounds.bottom, self.bounds.right, self.bounds.top],
            "nodata": self.nodata,
            "dtype": self.dataset.dtypes[0],
        }

    def sample(self, longitude, latitude) -> np.ndarray:
        lon, lat = np.broadcast_arrays(
            np.asarray(longitude, dtype=np.float64),
            np.asarray(latitude, dtype=np.float64),
        )
        original_shape = lon.shape
        lon_flat = lon.reshape(-1).copy()
        lat_flat = lat.reshape(-1)
        if self.bounds.left >= 0.0 and self.bounds.right > 180.0:
            lon_flat %= 360.0

        transform = self.inverse
        col = transform.a * lon_flat + transform.b * lat_flat + transform.c - 0.5
        row = transform.d * lon_flat + transform.e * lat_flat + transform.f - 0.5
        finite_coords = np.isfinite(col) & np.isfinite(row)
        if not finite_coords.any():
            return np.full(original_shape, np.nan, dtype=np.float64)

        c0 = max(int(np.floor(np.min(col[finite_coords]))) - 1, 0)
        r0 = max(int(np.floor(np.min(row[finite_coords]))) - 1, 0)
        c1 = min(int(np.ceil(np.max(col[finite_coords]))) + 2, self.width)
        r1 = min(int(np.ceil(np.max(row[finite_coords]))) + 2, self.height)
        if c1 <= c0 or r1 <= r0:
            return np.full(original_shape, np.nan, dtype=np.float64)

        band = self.dataset.read(
            1,
            window=Window(c0, r0, c1 - c0, r1 - r0),
            masked=True,
        )
        values = np.asarray(band.filled(np.nan), dtype=np.float64)
        # EMAG2v3's ASCII format uses 99999 for no data; reject sentinel values
        # as well if they are not represented in a GeoTIFF mask.
        values[(~np.isfinite(values)) | (np.abs(values) >= 10_000.0)] = np.nan
        coordinates = np.vstack((row - r0, col - c0))
        sampled = map_coordinates(
            values,
            coordinates,
            order=1,
            mode="constant",
            cval=np.nan,
            prefilter=False,
        )
        sampled[~finite_coords] = np.nan
        return sampled.reshape(original_shape)


def fetch_grid(path: Path, url: str) -> None:
    if path.is_file() and path.stat().st_size > 0:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".part")
    request = Request(url, headers={"User-Agent": "TaipeiDrift MagNav research experiment"})
    print(f"Downloading official NOAA grid: {url}")
    with urlopen(request, timeout=120) as response, temporary.open("wb") as target:
        while chunk := response.read(1024 * 1024):
            target.write(chunk)
    temporary.replace(path)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def make_tracks() -> list[dict]:
    bearing, _, endpoint_distance = GEOD.inv(*BUDai, *MAGONG)
    strait_end = GEOD.fwd(*BUDai, bearing, TRACK_LENGTH_M)[:2]
    inland_start = (120.60, 24.00)  # inland Taichung, eastbound toward foothills
    inland_bearing = 90.0
    inland_end = GEOD.fwd(*inland_start, inland_bearing, TRACK_LENGTH_M)[:2]
    return [
        {
            "name": "taiwan_strait_budai_toward_magong",
            "map": "sea_level",
            "start_lon": BUDai[0],
            "start_lat": BUDai[1],
            "bearing_deg": bearing,
            "end_lon": strait_end[0],
            "end_lat": strait_end[1],
            "length_m": TRACK_LENGTH_M,
            "description": "50 km geodesic from approximate Budai toward Magong; direct Budai-Magong separation is longer.",
            "budai_magong_distance_m": endpoint_distance,
        },
        {
            "name": "taichung_inland_eastbound",
            "map": "upward_continued",
            "start_lon": inland_start[0],
            "start_lat": inland_start[1],
            "bearing_deg": inland_bearing,
            "end_lon": inland_end[0],
            "end_lat": inland_end[1],
            "length_m": TRACK_LENGTH_M,
            "description": "50 km geodesic eastbound from inland Taichung toward the foothills.",
            "budai_magong_distance_m": None,
        },
    ]


def positions(track: dict, along_track_m: np.ndarray, heading_error_deg: float = 0.0):
    station = np.asarray(along_track_m, dtype=np.float64)
    bearing = track["bearing_deg"] + heading_error_deg
    return GEOD.fwd(
        np.full(station.shape, track["start_lon"], dtype=np.float64),
        np.full(station.shape, track["start_lat"], dtype=np.float64),
        np.full(station.shape, bearing, dtype=np.float64),
        station,
    )[:2]


def native_cell_spacing_m(sampler: GeoTiffSampler, latitude_deg: float) -> float:
    lon_m = sampler.resolution_deg[0] * 111_320.0 * math.cos(math.radians(latitude_deg))
    lat_m = sampler.resolution_deg[1] * 111_132.0
    return math.sqrt(lon_m * lat_m)


def compute_track_profile(track: dict, sampler: GeoTiffSampler, sigma_b_nt: float) -> dict:
    stations = np.arange(0.0, TRACK_LENGTH_M + PROFILE_SPACING_M / 2.0, PROFILE_SPACING_M)
    longitude, latitude = positions(track, stations)
    anomaly = sampler.sample(longitude, latitude)
    baseline_m = native_cell_spacing_m(sampler, track["start_lat"])
    half = baseline_m / 2.0
    before_lon, before_lat = positions(track, stations - half)
    after_lon, after_lat = positions(track, stations + half)
    before = sampler.sample(before_lon, before_lat)
    after = sampler.sample(after_lon, after_lat)
    gradient_nT_per_m = (after - before) / baseline_m
    sigma_pos_m = np.full(stations.shape, np.inf, dtype=np.float64)
    observable = np.isfinite(gradient_nT_per_m) & (np.abs(gradient_nT_per_m) > 0.0)
    sigma_pos_m[observable] = sigma_b_nt / np.abs(gradient_nT_per_m[observable])

    return {
        "stations_m": stations,
        "longitude": longitude,
        "latitude": latitude,
        "anomaly_nT": anomaly,
        "gradient_nT_per_km": gradient_nT_per_m * 1000.0,
        "sigma_pos_m": sigma_pos_m,
        "gradient_baseline_m": baseline_m,
    }


def write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def correlation_trials(
    track: dict,
    sampler: GeoTiffSampler,
    sigma_b_nt: float,
    trial_count: int,
    rng: np.random.Generator,
) -> list[dict]:
    step = PROFILE_SPACING_M
    sample_stations = np.arange(0.0, TRACK_LENGTH_M + step / 2.0, step)
    max_shift_steps = int(round(SEARCH_LIMIT_M / step))
    candidate_offsets = np.arange(-max_shift_steps, max_shift_steps + 1, dtype=np.float64) * step
    reference_stations = np.arange(
        -SEARCH_LIMIT_M,
        TRACK_LENGTH_M + SEARCH_LIMIT_M + step / 2.0,
        step,
    )
    reference_lon, reference_lat = positions(track, reference_stations)
    reference = sampler.sample(reference_lon, reference_lat)
    # One template per trial shift; this is a deliberately simple normalized
    # 1-D cross-correlation, without a particle filter or auxiliary sensors.
    template_rows = []
    for offset in candidate_offsets:
        start = int(round((offset + SEARCH_LIMIT_M) / step))
        template_rows.append(reference[start : start + sample_stations.size])
    templates = np.asarray(template_rows, dtype=np.float64)

    true_offsets = rng.uniform(-SEARCH_LIMIT_M, SEARCH_LIMIT_M, size=trial_count)
    start_lons, start_lats, _ = GEOD.fwd(
        np.full(trial_count, track["start_lon"], dtype=np.float64),
        np.full(trial_count, track["start_lat"], dtype=np.float64),
        np.full(trial_count, track["bearing_deg"], dtype=np.float64),
        true_offsets,
    )
    result_rows = []

    for heading_error in HEADING_ERRORS_DEG:
        bearing_values = np.full((trial_count, sample_stations.size), track["bearing_deg"] + heading_error)
        start_lon_grid = np.broadcast_to(start_lons[:, None], bearing_values.shape)
        start_lat_grid = np.broadcast_to(start_lats[:, None], bearing_values.shape)
        station_grid = np.broadcast_to(sample_stations[None, :], bearing_values.shape)
        observed_lon, observed_lat, _ = GEOD.fwd(
            start_lon_grid.reshape(-1),
            start_lat_grid.reshape(-1),
            bearing_values.reshape(-1),
            station_grid.reshape(-1),
        )
        observed = sampler.sample(observed_lon, observed_lat).reshape(bearing_values.shape)
        observed += rng.normal(0.0, sigma_b_nt, size=observed.shape)

        # Pairwise Pearson correlation while safely ignoring grid nodata.
        obs_3d = observed[:, None, :]
        template_3d = templates[None, :, :]
        valid = np.isfinite(obs_3d) & np.isfinite(template_3d)
        count = valid.sum(axis=2)
        safe_obs = np.where(valid, obs_3d, 0.0)
        safe_template = np.where(valid, template_3d, 0.0)
        mean_obs = np.divide(safe_obs.sum(axis=2), count, out=np.zeros_like(count, dtype=float), where=count > 0)
        mean_template = np.divide(
            safe_template.sum(axis=2), count, out=np.zeros_like(count, dtype=float), where=count > 0
        )
        centered_obs = np.where(valid, obs_3d - mean_obs[:, :, None], 0.0)
        centered_template = np.where(valid, template_3d - mean_template[:, :, None], 0.0)
        covariance = (centered_obs * centered_template).sum(axis=2)
        denom = np.sqrt(
            (centered_obs * centered_obs).sum(axis=2)
            * (centered_template * centered_template).sum(axis=2)
        )
        correlations = np.divide(covariance, denom, out=np.full_like(covariance, np.nan), where=denom > 0.0)
        correlations[count < max(20, int(0.8 * sample_stations.size))] = np.nan
        correlations[~np.isfinite(correlations)] = np.nan
        safe_correlations = np.where(np.isfinite(correlations), correlations, -np.inf)
        best_indices = np.argmax(safe_correlations, axis=1)
        best_offsets = candidate_offsets[best_indices]
        best_correlations = correlations[np.arange(trial_count), best_indices]
        valid_estimates = np.isfinite(best_correlations)
        errors = np.abs(best_offsets - true_offsets)

        for index in range(trial_count):
            result_rows.append(
                {
                    "track": track["name"],
                    "heading_error_deg": heading_error,
                    "trial": index,
                    "true_offset_m": true_offsets[index],
                    "estimated_offset_m": best_offsets[index] if valid_estimates[index] else "",
                    "abs_error_m": errors[index] if valid_estimates[index] else "",
                    "peak_correlation": best_correlations[index] if valid_estimates[index] else "",
                    "valid": bool(valid_estimates[index]),
                }
            )

    return result_rows


def summarize_correlations(rows: list[dict], trial_count: int) -> list[dict]:
    summaries = []
    for track_name in sorted({row["track"] for row in rows}):
        for heading_error in HEADING_ERRORS_DEG:
            selected = [
                row for row in rows
                if row["track"] == track_name and row["heading_error_deg"] == heading_error and row["valid"]
            ]
            errors = np.asarray([float(row["abs_error_m"]) for row in selected], dtype=np.float64)
            peaks = np.asarray([float(row["peak_correlation"]) for row in selected], dtype=np.float64)
            summaries.append(
                {
                    "track": track_name,
                    "heading_error_deg": heading_error,
                    "trials_requested": trial_count,
                    "trials_valid": len(selected),
                    "median_abs_error_m": float(np.median(errors)) if errors.size else None,
                    "p95_abs_error_m": float(np.percentile(errors, 95)) if errors.size else None,
                    "within_500m_pct": float(100.0 * np.mean(errors <= 500.0)) if errors.size else None,
                    "median_peak_correlation": float(np.median(peaks)) if peaks.size else None,
                }
            )
    return summaries


def profile_summary(name: str, profile: dict) -> dict:
    anomaly = profile["anomaly_nT"]
    gradient = profile["gradient_nT_per_km"]
    sigma = profile["sigma_pos_m"]
    valid_gradient = np.isfinite(gradient)
    finite_sigma = sigma[np.isfinite(sigma)]
    bad_bound = ~np.isfinite(sigma) | (sigma >= 500.0)
    return {
        "track": name,
        "profile_samples": int(anomaly.size),
        "valid_anomaly_samples": int(np.isfinite(anomaly).sum()),
        "anomaly_coverage_pct": float(100.0 * np.isfinite(anomaly).mean()),
        "valid_gradient_samples": int(valid_gradient.sum()),
        "gradient_coverage_pct": float(100.0 * valid_gradient.mean()),
        "gradient_baseline_m": float(profile["gradient_baseline_m"]),
        "gradient_abs_median_nT_per_km": float(np.nanmedian(np.abs(gradient))) if valid_gradient.any() else None,
        "gradient_abs_p90_nT_per_km": float(np.nanpercentile(np.abs(gradient[valid_gradient]), 90)) if valid_gradient.any() else None,
        "sigma_pos_finite_median_m": float(np.median(finite_sigma)) if finite_sigma.size else None,
        "sigma_pos_finite_p90_m": float(np.percentile(finite_sigma, 90)) if finite_sigma.size else None,
        "sigma_pos_unbounded_samples": int((~np.isfinite(sigma)).sum()),
        "sigma_pos_ge_500m_pct": float(100.0 * bad_bound.mean()),
        "kill": bool(bad_bound.mean() > 0.5),
    }


def save_plot(track_profiles: dict[str, dict], sigma_b_nt: float, path: Path) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    titles = {
        "taiwan_strait_budai_toward_magong": "Strait: Budai → Magong",
        "taichung_inland_eastbound": "Inland: Taichung eastbound",
    }
    for row, (name, profile) in enumerate(track_profiles.items()):
        distance_km = profile["stations_m"] / 1000.0
        axes[row, 0].plot(distance_km, profile["anomaly_nT"], color="#214f86", linewidth=1.4)
        axes[row, 0].set(title=f"{titles[name]} — anomaly", xlabel="Distance along track (km)", ylabel="nT")
        axes[row, 1].plot(distance_km, profile["gradient_nT_per_km"], color="#a14a27", linewidth=1.2)
        axes[row, 1].axhline(0.0, color="black", linewidth=0.6)
        axes[row, 1].set(title="Along-track gradient", xlabel="Distance along track (km)", ylabel="nT/km")
        axes[row, 2].semilogy(distance_km, profile["sigma_pos_m"], color="#26734d", linewidth=1.2)
        axes[row, 2].axhline(500.0, color="#a00000", linestyle="--", linewidth=1.0, label="500 m kill threshold")
        axes[row, 2].set(title="CRLB-style σ position", xlabel="Distance along track (km)", ylabel="m")
        axes[row, 2].legend(fontsize=8)
    fig.suptitle(f"EMAG2v3: coarse-grid MagNav kill test ({sigma_b_nt:g} nT assumed noise)")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def run(args: argparse.Namespace) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for key, url in GRID_URLS.items():
        fetch_grid(RAW_DIR / GRID_FILENAMES[key], url)
    tracks = make_tracks()
    samplers = {
        "sea_level": GeoTiffSampler(RAW_DIR / GRID_FILENAMES["sea_level"]),
        "upward_continued": GeoTiffSampler(RAW_DIR / GRID_FILENAMES["upward_continued"]),
    }
    rng = np.random.default_rng(args.seed)
    profiles: dict[str, dict] = {}
    profile_rows = []
    track_summaries = []
    correlation_rows = []
    for track in tracks:
        sampler = samplers[track["map"]]
        profile = compute_track_profile(track, sampler, args.sigma_b_nt)
        profiles[track["name"]] = profile
        track_summary = profile_summary(track["name"], profile)
        track_summary.update(
            {
                "map": track["map"],
                "map_altitude": "sea level over oceanic/coastal regions" if track["map"] == "sea_level" else "4 km above WGS84 ellipsoid",
                "description": track["description"],
                "start_lon_lat": [track["start_lon"], track["start_lat"]],
                "end_lon_lat": [track["end_lon"], track["end_lat"]],
                "bearing_deg_initial": float(track["bearing_deg"]),
                "length_m": track["length_m"],
                "budai_magong_geodesic_distance_m": track["budai_magong_distance_m"],
            }
        )
        track_summaries.append(track_summary)
        for i, station in enumerate(profile["stations_m"]):
            profile_rows.append(
                {
                    "track": track["name"],
                    "map": track["map"],
                    "along_track_m": station,
                    "longitude": profile["longitude"][i],
                    "latitude": profile["latitude"][i],
                    "anomaly_nT": profile["anomaly_nT"][i],
                    "gradient_nT_per_km": profile["gradient_nT_per_km"][i],
                    "sigma_pos_m": profile["sigma_pos_m"][i],
                }
            )
        correlation_rows.extend(
            correlation_trials(track, sampler, args.sigma_b_nt, args.trials, rng)
        )

    correlation_summaries = summarize_correlations(correlation_rows, args.trials)
    write_csv(
        OUT_DIR / "track_profiles.csv",
        ["track", "map", "along_track_m", "longitude", "latitude", "anomaly_nT", "gradient_nT_per_km", "sigma_pos_m"],
        profile_rows,
    )
    write_csv(
        OUT_DIR / "correlation_trials.csv",
        ["track", "heading_error_deg", "trial", "true_offset_m", "estimated_offset_m", "abs_error_m", "peak_correlation", "valid"],
        correlation_rows,
    )
    write_csv(
        OUT_DIR / "correlation_summary.csv",
        ["track", "heading_error_deg", "trials_requested", "trials_valid", "median_abs_error_m", "p95_abs_error_m", "within_500m_pct", "median_peak_correlation"],
        correlation_summaries,
    )
    plot_path = OUT_DIR / "u7_magnav_profiles.png"
    save_plot(profiles, args.sigma_b_nt, plot_path)

    grid_metadata = {}
    for key, sampler in samplers.items():
        grid_metadata[key] = sampler.metadata()
    for sampler in samplers.values():
        sampler.close()

    manifest = {
        "experiment": "X7 MagNav kill test",
        "run_label": "MEASURED on published NOAA EMAG2v3 grid; correlation matching is SIMULATED",
        "run_parameters": {
            "sigma_B_nT": args.sigma_b_nt,
            "platform_noise_argument": (
                f"{args.sigma_b_nt:g} nT is an optimistic effective residual after calibration/compensation, not a demonstrated onboard value. "
                "Tuck et al. measured 21.4-574.2 nT peak UAV interference (with motors engaged); currents as low as 1 A changed the field. "
                "Thus platform noise can exceed the assumed value, worsening the bound."
            ),
            "heading_error_scenarios_deg": list(HEADING_ERRORS_DEG),
            "simulated_trials_per_heading_and_track": args.trials,
            "simulated_random_seed": args.seed,
            "profile_and_observation_spacing_m": PROFILE_SPACING_M,
            "correlation_search_range_m": [-SEARCH_LIMIT_M, SEARCH_LIMIT_M],
            "correlation_search_step_m": PROFILE_SPACING_M,
            "gradient_method": "centered finite difference along geodesic with baseline one approximate native grid cell",
            "position_bound": "sigma_pos = sigma_B / abs(dB/ds); infinite when gradient is missing or zero",
            "kill_rule": "kill when sigma_pos >= 500 m on more than 50% of track sample points; missing/unobservable samples count as failures",
        },
        "data_source": {
            "product": "EMAG2v3 Earth Magnetic Anomaly Grid, version 3",
            "publisher": "NOAA National Centers for Environmental Information (NCEI)",
            "doi": NOAA_METADATA_URL,
            "official_product_url": NOAA_PRODUCT_URL,
            "official_downloads_url": NOAA_DOWNLOAD_URL,
            "format_readme_url": GRID_README_URL,
            "license_and_limitations": (
                "NOAA metadata says the dataset is not subject to copyright protection within the United States, "
                "but explicitly states it is not to be used for navigation and is not suitable for navigation; cite the dataset. "
                "This research kill test is not an endorsement for operational use."
            ),
            "spatial_resolution": "2 arc-minutes (approximately 3.7 km north-south and 3.4 km east-west at 24 degrees N)",
            "sea_level_grid_altitude": "sea-level anomalies over oceanic regions, including coastal areas where available",
            "upward_continued_grid_altitude": "continuous 4 km above WGS84 ellipsoid",
            "anomaly_measurements_period": "1946-01-01 through 2014-02-20 per NCEI metadata",
            "grid_files": grid_metadata,
            "sha256": {
                key: sha256_file(RAW_DIR / GRID_FILENAMES[key])
                for key in GRID_URLS
            },
        },
        "limitations": [
            "No onboard magnetic measurements or flight data were collected: anomaly profiles are computed from NOAA grids, and correlation scores are SIMULATED.",
            "The correlation generator treats the map as the true field and adds independent Gaussian noise; it omits map-cell uncertainty, temporal variation, and motor/attitude-correlated interference.",
            "The strait uses the sea-level grid (potentially optimistic for a drone above sea level); the inland grid is upward continued to 4 km because that is the published land product.",
            "NOAA explicitly says EMAG2v3 is not suitable for navigation; this result is a research kill test, not operational validation.",
            "Budai-Magong is about 60 km geodesically, so the analyzed 50 km line starts at Budai and follows the direct bearing toward Magong.",
        ],
        "platform_noise_reference": {
            "citation": "Tuck et al. (2021), Magnetic interference mapping of four types of unmanned aircraft systems intended for aeromagnetic surveying",
            "url": UAV_INTERFERENCE_URL,
            "reported_peak_interference_nT": [21.4, 574.2],
            "reported_background_scanner_rms_nT": [3.1, 7.4],
            "reported_current_effect": "field change detected at currents as low as 1 A",
            "interpretation": f"the chosen {args.sigma_b_nt:g} nT sigma is optimistic and not a flight-tested noise measurement for this vehicle",
        },
        "tracks": track_summaries,
        "correlation_summaries": correlation_summaries,
        "outputs": [
            "track_profiles.csv",
            "correlation_trials.csv",
            "correlation_summary.csv",
            plot_path.name,
            "summary.json",
        ],
    }
    (OUT_DIR / "summary.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    print("X7 — EMAG2v3 MagNav kill test")
    print("Evidence: MEASURED on NOAA grid; correlation matching: SIMULATED")
    print("Grid: EMAG2v3, 2 arc-min (~3.7 km N-S, ~3.4 km E-W at 24°N)")
    print("Altitude: sea-level over ocean/coasts for Strait; 4 km above WGS84 ellipsoid inland")
    print("NOAA metadata: no U.S. copyright protection stated; cite the grid; explicitly not suitable for navigation")
    print(f"Assumed sigma_B: {args.sigma_b_nt:.1f} nT (optimistic effective platform residual; UAV interference reported at 21.4–574.2 nT peak)")
    for summary in track_summaries:
        verdict = "KILL" if summary["kill"] else "does not meet kill threshold"
        median = summary["sigma_pos_finite_median_m"]
        p90 = summary["sigma_pos_finite_p90_m"]
        median_text = f"{median:.0f} m" if median is not None else "n/a"
        p90_text = f"{p90:.0f} m" if p90 is not None else "n/a"
        print(
            f"{summary['track']}: |grad| median={summary['gradient_abs_median_nT_per_km']:.3f} nT/km; "
            f"finite sigma_pos median={median_text}, p90={p90_text}; "
            f"unbounded={summary['sigma_pos_unbounded_samples']}; "
            f">=500m/infinite on {summary['sigma_pos_ge_500m_pct']:.1f}% ({verdict}); "
            f"grid coverage={summary['valid_gradient_samples']}/{summary['profile_samples']}"
        )
    print(f"SIMULATED 1-D correlation ({args.trials} trials per setting):")
    for summary in correlation_summaries:
        if summary["heading_error_deg"] in (0.0, 3.0, 5.0):
            print(
                f"  {summary['track']}, heading={summary['heading_error_deg']:+.0f}°: "
                f"median |error|={summary['median_abs_error_m']}, p95={summary['p95_abs_error_m']}, "
                f"<=500m={summary['within_500m_pct']}%, valid={summary['trials_valid']}/{summary['trials_requested']}"
            )
    print(f"Files: {OUT_DIR.relative_to(ROOT)} (track_profiles.csv, correlation_summary.csv, correlation_trials.csv, summary.json, {plot_path.name})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sigma-b-nt", type=float, default=DEFAULT_SIGMA_B_NT)
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    args = parser.parse_args()
    if args.sigma_b_nt <= 0.0:
        parser.error("--sigma-b-nt must be positive")
    if args.trials < 1:
        parser.error("--trials must be at least 1")
    return args


if __name__ == "__main__":
    run(parse_args())

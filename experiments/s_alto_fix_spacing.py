"""Test how baro-minus-DEM scale affects ALTO camera dead reckoning between simulated fixes.

Run from the repository root:
    .venv/bin/python experiments/s_alto_fix_spacing.py

Reads the ALTO flow/DEM caches and helpers from u2_baro_dem_scale.py. Outputs are
written under data/processed/alto_fix_spacing/.
"""
from __future__ import annotations

import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import u2_baro_dem_scale as u2

OUTPUT_DIR = u2.ROOT / "data/processed/alto_fix_spacing"
FIX_SPACINGS_M = (300, 500, 1000, 2000)
SEEDS = tuple(range(20))
FIX_POSITION_SIGMA_M = 15.0
FROZEN_ZOOM_RELATIVE_SIGMA = 0.057
PRE_FIX_SEARCH_RADIUS_M = 60.0
VARIANTS = ("frozen_zoom", "baro_dem", "true_agl")
VARIANT_LABELS = {
    "frozen_zoom": "Frozen zoom (team baseline)",
    "baro_dem": "Baro − DEM at estimated position",
    "true_agl": "True AGL (oracle)",
}
VARIANT_COLORS = {"frozen_zoom": "tab:gray", "baro_dem": "tab:blue", "true_agl": "tab:green"}


def reproduce_no_fix(query, truth, travelled, jam, flow, terrain_true, sim_baro, dem, dem_transform):
    """Run u2's unmodified, no-fix implementation and check its published endpoints."""
    results, calibration = u2.calibrate_and_run(
        query, truth, travelled, jam, flow, terrain_true, sim_baro, dem, dem_transform, len(query)
    )
    expected_summary = json.loads((u2.OUTPUT_DIR / "summary.json").read_text())
    expected = expected_summary["variants"]
    for name in VARIANTS:
        measured = float(results[name]["error"][-1])
        published = float(expected[name]["position_error_final_m"])
        difference = abs(measured - published)
        print(f"No-fix reproduction {name}: u2 summary={published:.3f} m; rerun={measured:.3f} m; |Δ|={difference:.3f} m")
        assert difference <= 1.0, f"{name} endpoint differs from u2 summary by {difference:.3f} m"
    return results, calibration, expected_summary


def simulate_spacing(
    spacing_m, truth, travelled, jam, flow, flow_to_ground, h0, datum_offset, true_agl, sim_baro, dem, dem_transform
):
    """Run all seeded trajectories together; fixes occur at the first row beyond each L-m target."""
    post_cut_distance = travelled[jam:] - travelled[jam]
    n_fixes = int(np.floor((post_cut_distance[-1] + 1e-9) / spacing_m))
    fix_targets = spacing_m * np.arange(1, n_fixes + 1, dtype=np.float64)
    fix_indices = jam + np.searchsorted(post_cut_distance, fix_targets, side="left")
    n_fixes = len(fix_indices)
    fix_at_frame = {int(frame): i for i, frame in enumerate(fix_indices)}

    position_noise = np.empty((len(SEEDS), n_fixes, 2), dtype=np.float64)
    zoom_noise = np.empty((len(SEEDS), n_fixes), dtype=np.float64)
    for seed_index, seed in enumerate(SEEDS):
        rng = np.random.default_rng(seed)
        position_noise[seed_index] = rng.normal(0.0, FIX_POSITION_SIGMA_M, size=(n_fixes, 2))
        zoom_noise[seed_index] = rng.normal(0.0, FROZEN_ZOOM_RELATIVE_SIGMA, size=n_fixes)

    estimates = {
        name: np.repeat(truth[jam][None, :], len(SEEDS), axis=0).astype(np.float64)
        for name in VARIANTS
    }
    frozen_scale = np.ones(len(SEEDS), dtype=np.float64)
    errors = {name: np.empty((len(SEEDS), len(truth) - jam), dtype=np.float64) for name in VARIANTS}
    for name in VARIANTS:
        errors[name][:, 0] = 0.0
    pre_fix_errors = {name: np.empty((len(SEEDS), n_fixes), dtype=np.float64) for name in VARIANTS}

    for frame in range(jam + 1, len(truth)):
        estimated_terrain = u2.ground_height(estimates["baro_dem"], dem, dem_transform)
        baro_agl = sim_baro[frame] - estimated_terrain + datum_offset
        scales = {
            "frozen_zoom": frozen_scale,
            "baro_dem": baro_agl / h0,
            "true_agl": np.full(len(SEEDS), true_agl[frame] / h0, dtype=np.float64),
        }
        motion_m = flow[frame] @ flow_to_ground
        for name in VARIANTS:
            estimates[name] += motion_m[None, :] * scales[name][:, None]
        before_fix = {
            name: np.linalg.norm(estimates[name] - truth[frame], axis=1)
            for name in VARIANTS
        }

        fix_index = fix_at_frame.get(frame)
        if fix_index is not None:
            for name in VARIANTS:
                pre_fix_errors[name][:, fix_index] = before_fix[name]
                estimates[name] = truth[frame][None, :] + position_noise[:, fix_index, :]
            frozen_scale = (true_agl[frame] / h0) * (1.0 + zoom_noise[:, fix_index])
            for name in VARIANTS:
                errors[name][:, frame - jam] = np.linalg.norm(estimates[name] - truth[frame], axis=1)
        else:
            for name in VARIANTS:
                errors[name][:, frame - jam] = before_fix[name]

    metrics = {}
    for name in VARIANTS:
        sample_errors = errors[name].ravel()
        per_seed_max = np.max(pre_fix_errors[name], axis=1)
        metrics[name] = {
            "median_position_error_m": float(np.median(sample_errors)),
            "p95_position_error_m": float(np.percentile(sample_errors, 95)),
            "median_seed_max_pre_fix_error_m": float(np.median(per_seed_max)),
            "share_fixes_pre_fix_error_over_60m": float(np.mean(pre_fix_errors[name] > PRE_FIX_SEARCH_RADIUS_M)),
        }
    return metrics, n_fixes




def main() -> None:
    archive, query, truth, travelled, jam = u2.load_alto()
    # Read existing u2 caches only; save=False prevents modifying its flow cache.
    flow = u2.image_motion(archive, query, len(query), save=False)
    dem, dem_transform = u2.load_dem(truth)
    terrain_true = u2.ground_height(truth, dem, dem_transform)
    residual, residual_info = u2.residual_profile(len(query))
    altitude = query.altitude.to_numpy(dtype=np.float64)
    sim_baro = altitude + residual
    true_agl = altitude - terrain_true

    no_fix_results, calibration, expected_summary = reproduce_no_fix(
        query, truth, travelled, jam, flow, terrain_true, sim_baro, dem, dem_transform
    )
    datum_offset = float(calibration["datum_offset_m"])
    flow_to_ground = np.asarray(calibration["flow_to_ground"], dtype=np.float64)
    h0 = float(calibration["calibration_height_m"])

    rows = []
    result_by_spacing = {}
    fix_counts = {}
    for spacing_m in FIX_SPACINGS_M:
        metrics, fix_count = simulate_spacing(
            spacing_m, truth, travelled, jam, flow, flow_to_ground, h0, datum_offset, true_agl, sim_baro, dem, dem_transform
        )
        result_by_spacing[spacing_m] = metrics
        fix_counts[spacing_m] = fix_count
        for name in VARIANTS:
            rows.append({
                "fix_spacing_m": spacing_m,
                "variant": name,
                "fix_count": fix_count,
                **metrics[name],
            })

    table = pd.DataFrame(rows)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    table.to_csv(OUTPUT_DIR / "table.csv", index=False)

    summary = {
        "experiment": "ALTO dead-reckoning scale with simulated position fixes",
        "command": ".venv/bin/python experiments/s_alto_fix_spacing.py",
        "evidence_label": "MEASURED ALTO camera/terrain trajectory; SIMULATED barometer and fixes; true_agl is an ORACLE upper bound",
        "source_zip": str(u2.ZIP_PATH.relative_to(u2.ROOT)),
        "cut_frame": int(jam),
        "cut_distance_m": float(travelled[jam]),
        "post_cut_distance_m": float(travelled[-1] - travelled[jam]),
        "post_cut_frames": int(len(query) - jam - 1),
        "calibration": calibration,
        "barometer_simulation": residual_info,
        "no_fix_reproduction": {
            name: {
                "u2_summary_final_error_m": float(expected_summary["variants"][name]["position_error_final_m"]),
                "rerun_final_error_m": float(no_fix_results[name]["error"][-1]),
            }
            for name in VARIANTS
        },
        "fix_model": {
            "spacing_m": list(FIX_SPACINGS_M),
            "seed_count": len(SEEDS),
            "seeds": list(SEEDS),
            "position_error_label": "SIMULATED fix, error from docs/findings.md 4-15 m",
            "position_noise_standard_deviation_per_axis_m": FIX_POSITION_SIGMA_M,
            "position_noise_shared_across_variants_per_seed_and_fix": True,
            "frozen_zoom_scale_renewal": "(true AGL at fix / calibration AGL) * (1 + N(0, 0.057))",
            "frozen_zoom_relative_renewal_sigma": FROZEN_ZOOM_RELATIVE_SIGMA,
            "baro_dem_scale": "simulated barometer minus DEM sampled at the current estimated position",
            "true_agl_scale": "altitude minus DEM at the true position; oracle comparator",
            "pre_fix_search_radius_m": PRE_FIX_SEARCH_RADIUS_M,
            "sampled_fix_location": "first ALTO query row at or beyond each L-m increment of true travelled distance after the cut",
        },
        "metric_definitions": {
            "position_error": "Median and 95th percentile across all seeds and post-cut query samples; fix samples use the post-reset position error.",
            "median_seed_max_pre_fix_error_m": "For each seed, maximum pre-reset error among its fixes; reported value is the median of those maxima.",
            "share_fixes_pre_fix_error_over_60m": "Fraction of all seeded fixes whose pre-reset error exceeds the 60 m benchmark search radius.",
        },
        "results": [
            {"fix_spacing_m": spacing_m, "fix_count_per_seed": int(fix_counts[spacing_m]), "variants": result_by_spacing[spacing_m]}
            for spacing_m in FIX_SPACINGS_M
        ],
        "limits": [
            "Fix locations are snapped to the first query row at or beyond each requested true-distance interval.",
            "Fixes are simulated position resets, not actual map-match results; the requested 15 m per-axis Gaussian sigma is used.",
            "ALTO query rows are treated as one second apart by the reused Zurich residual replay.",
        ],
    }
    (OUTPUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    for name in VARIANTS:
        y = [result_by_spacing[L][name]["median_seed_max_pre_fix_error_m"] for L in FIX_SPACINGS_M]
        ax.plot(FIX_SPACINGS_M, y, marker="o", linewidth=2, label=VARIANT_LABELS[name], color=VARIANT_COLORS[name])
    ax.axhline(PRE_FIX_SEARCH_RADIUS_M, color="black", linestyle="--", linewidth=1, label="60 m benchmark search radius")
    ax.set_xlabel("Fix spacing L (m true distance)")
    ax.set_ylabel("Median over seeds of maximum pre-fix error (m)")
    ax.set_title("ALTO simulated fixes: drift between position resets")
    ax.set_xticks(FIX_SPACINGS_M)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "fix_spacing.png", dpi=150)
    plt.close(fig)

    display = table.copy()
    display["median_position_error_m"] = display["median_position_error_m"].map(lambda v: f"{v:.1f}")
    display["p95_position_error_m"] = display["p95_position_error_m"].map(lambda v: f"{v:.1f}")
    display["median_seed_max_pre_fix_error_m"] = display["median_seed_max_pre_fix_error_m"].map(lambda v: f"{v:.1f}")
    display["share_fixes_pre_fix_error_over_60m"] = display["share_fixes_pre_fix_error_over_60m"].map(lambda v: f"{100.0 * v:.1f}%")
    print("\nFix-spacing results (position-error stats over seeds and samples; pre-fix maxima per seed):")
    print(display.to_string(index=False))
    print(f"Saved: {OUTPUT_DIR / 'summary.json'}")
    print(f"Saved: {OUTPUT_DIR / 'table.csv'}")
    print(f"Saved: {OUTPUT_DIR / 'fix_spacing.png'}")
    archive.close()


if __name__ == "__main__":
    main()

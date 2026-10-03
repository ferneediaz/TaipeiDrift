"""Evaluator-side ALTO held-out diagnosis; all post-jam truth use is diagnostic.

Run from the repository root with ``.venv/bin/python experiments/t_alto_diag.py``.
"""
from __future__ import annotations

import argparse

from dataclasses import replace

import cv2
import numpy as np
import pandas as pd

import t_alto_heldout as alto

DIAG_DIR = alto.OUT_DIR / "diag"
FIX_SPACING_M = 300.0
ORACLE_ZOOMS = np.round(np.arange(0.40, 1.4001, 0.05), 2)
ORACLE_ANGLES = np.arange(-10.0, 36.0, 5.0)


def frame_indices(section: alto.PreparedRange) -> list[tuple[int, float]]:
    """Frames nearest each 300 m truth-distance target after the GNSS jam."""
    after_jam = section.travelled - section.travelled[section.jam]
    end_distance = float(after_jam[-1])
    targets = np.arange(FIX_SPACING_M, end_distance + 1e-9, FIX_SPACING_M)
    result = []
    for target in targets:
        local_k = int(np.searchsorted(after_jam, target, side="left"))
        if local_k > section.jam and local_k < len(section.truth):
            result.append((local_k, float(after_jam[local_k])))
    return result


def calibrated_fix_rows(
    dataset: alto.AltoDataset, name: str, section: alto.PreparedRange
) -> tuple[dict, list[dict]]:
    frames = frame_indices(section)
    zooms = np.clip(
        section.zoom0 + np.arange(-0.10, 0.11, 0.05), 0.5, 1.1
    )
    angles = np.array([section.angle0 - 5.0, section.angle0, section.angle0 + 5.0])
    rows = []
    for local_k, distance_m in frames:
        truth = section.truth[local_k]
        score, position, selected_zoom, selected_angle = alto.match_fix(
            dataset,
            section.i0 + local_k,
            zooms,
            angles,
            dataset.nearest(truth, alto.CALIB_NEAREST),
        )
        rows.append(
            {
                "dataset": name,
                "section": name if name == "Val" else None,
                "frame": section.i0 + local_k,
                "distance_after_jam_m": distance_m,
                "truth_e_m": float(truth[0]),
                "truth_n_m": float(truth[1]),
                "calibrated_zoom0": section.zoom0,
                "calibrated_angle0_deg": section.angle0,
                "selected_zoom": selected_zoom,
                "selected_angle_deg": selected_angle,
                "score": float(score),
                "fix_error_m": float(np.linalg.norm(position - truth)),
            }
        )
    for row in rows:
        if name != "Val":
            row["section"] = name
    errors = np.array([row["fix_error_m"] for row in rows], dtype=float)
    scores = np.array([row["score"] for row in rows], dtype=float)
    summary = {
        "section": name,
        "num_fixes": len(rows),
        "within_30m_fraction": float(np.mean(errors <= 30.0)) if len(errors) else np.nan,
        "median_error_m": float(np.median(errors)) if len(errors) else np.nan,
        "median_score": float(np.median(scores)) if len(scores) else np.nan,
        "truth_centered": True,
        "fix_spacing_m": FIX_SPACING_M,
    }
    return summary, rows


def oracle_score_rows(
    dataset: alto.AltoDataset,
    section_name: str,
    section: alto.PreparedRange,
) -> list[dict]:
    """Score every fixed zoom/angle pair on each sampled frame, using true-centred 7 refs."""
    rows = []
    for local_k, distance_m in frame_indices(section):
        truth = section.truth[local_k]
        candidates = dataset.nearest(truth, alto.CALIB_NEAREST)
        query_name = f"{dataset.prefix}/query_images/{dataset.query.name.iloc[section.i0 + local_k]}"
        frame = dataset.load_image(query_name)
        reference_images = [
            dataset.load_image(
                f"{dataset.prefix}/reference_images/{dataset.reference.name.iloc[ri]}"
            )
            for ri in candidates
        ]
        for zoom in ORACLE_ZOOMS:
            transformed = []
            for angle in ORACLE_ANGLES:
                template = alto.template(frame, float(zoom), float(angle))
                best_score = -1.0
                for reference in reference_images:
                    _, score, _, _ = cv2.minMaxLoc(
                        cv2.matchTemplate(reference, template, cv2.TM_CCOEFF_NORMED)
                    )
                    if score > best_score:
                        best_score = float(score)
                transformed.append(best_score)
            for angle, score in zip(ORACLE_ANGLES, transformed):
                rows.append(
                    {
                        "section": section_name,
                        "frame": section.i0 + local_k,
                        "distance_after_jam_m": distance_m,
                        "zoom": float(zoom),
                        "angle_deg": float(angle),
                        "score": float(score),
                    }
                )
    return rows


def choose_oracles(
    score_rows: list[dict], calibration: pd.DataFrame
) -> tuple[pd.DataFrame, dict[str, tuple[float, float]]]:
    scores = pd.DataFrame(score_rows)
    grouped = (
        scores.groupby(["section", "zoom", "angle_deg"], sort=False)
        .score.median()
        .rename("median_score")
        .reset_index()
    )
    best_indices = grouped.groupby("section", sort=False).median_score.idxmax()
    best = grouped.loc[best_indices].copy().reset_index(drop=True)
    cal = calibration.set_index("section")
    best["calibrated_zoom0"] = best.section.map(cal.zoom0)
    best["calibrated_angle0_deg"] = best.section.map(cal.angle0_deg)
    best["calibrated_zoom_at_edge"] = np.isclose(
        best.calibrated_zoom0, 0.60, atol=1e-6
    ) | np.isclose(best.calibrated_zoom0, 1.00, atol=1e-6)
    best["oracle_zoom_at_wide_grid_edge"] = np.isclose(
        best.zoom, 0.40, atol=1e-9
    ) | np.isclose(best.zoom, 1.40, atol=1e-9)
    best["oracle_zoom_outside_calibration_grid"] = (
        (best.zoom < 0.60 - 1e-9) | (best.zoom > 1.00 + 1e-9)
    )
    best["edge_with_oracle_outside"] = (
        best.calibrated_zoom_at_edge & best.oracle_zoom_outside_calibration_grid
    )
    best["label"] = "DIAGNOSTIC oracle zoom - uses truth"
    oracle_values = {
        row.section: (float(row.zoom), float(row.angle_deg))
        for row in best.itertuples(index=False)
    }
    return best, oracle_values


def dead_reckoning_row(
    dataset: alto.AltoDataset, section_name: str, section: alto.PreparedRange
) -> dict:
    camera = alto.run_config(dataset, section, alto.RunConfig("camera only", None))
    error_vector = camera.path[-1] - section.truth[-1]
    direction_start = int(
        np.searchsorted(section.travelled, section.travelled[-1] - 200.0, side="left")
    )
    route_vector = section.truth[-1] - section.truth[direction_start]
    route_length = float(section.travelled[-1] - section.travelled[section.jam])
    direction = route_vector / np.linalg.norm(route_vector)
    left = np.array([-direction[1], direction[0]])
    along = float(error_vector @ direction)
    cross = float(error_vector @ left)
    return {
        "section": section_name,
        "post_jam_distance_m": route_length,
        "end_error_m": float(camera.error[-1]),
        "along_track_error_m": along,
        "cross_track_error_m": cross,
        "implied_scale_error_pct": 100.0 * along / route_length,
        "implied_heading_error_deg": float(np.degrees(np.arctan2(cross, route_length))),
        "axis": "true final 200 m direction; cross positive left; denominator is post-jam distance",
    }


def run_oracle_fixes_only() -> None:
    """Score the selected oracle pair at each saved truth-centred frame."""
    calibration = pd.read_csv(alto.OUT_DIR / "calibration.csv")
    grid_rows = pd.read_csv(DIAG_DIR / "oracle_grid_scores.csv")
    oracle_summary, _ = choose_oracles(grid_rows.to_dict(orient="records"), calibration)
    oracle_summary.to_csv(DIAG_DIR / "oracle_calibration.csv", index=False)
    oracle = oracle_summary.set_index("section")
    samples = pd.read_csv(DIAG_DIR / "single_fix_samples.csv")
    grid = grid_rows.set_index(["section", "frame", "zoom", "angle_deg"])
    dataset = alto.AltoDataset(alto.TRAIN_ZIP, "Train")
    output = []
    for sample in samples[samples.section.str.startswith("section_")].itertuples(index=False):
        selected = oracle.loc[sample.section]
        zoom, angle = float(selected.zoom), float(selected.angle_deg)
        truth = np.array([sample.truth_e_m, sample.truth_n_m])
        score, position, _, _ = alto.match_fix(
            dataset,
            int(sample.frame),
            np.array([zoom]),
            np.array([angle]),
            dataset.nearest(truth, alto.CALIB_NEAREST),
        )
        grid_score = float(grid.loc[(sample.section, sample.frame, zoom, angle)].score)
        if not np.isclose(score, grid_score, atol=1e-6):
            dataset.close()
            raise RuntimeError(
                f"oracle score mismatch for {sample.section}, frame {sample.frame}"
            )
        output.append(
            {
                "section": sample.section,
                "frame": int(sample.frame),
                "distance_after_jam_m": float(sample.distance_after_jam_m),
                "oracle_zoom0": zoom,
                "oracle_angle0_deg": angle,
                "score": float(score),
                "fix_error_m": float(np.linalg.norm(position - truth)),
                "label": "DIAGNOSTIC oracle zoom - uses truth",
            }
        )
    result = pd.DataFrame(output)
    result.to_csv(DIAG_DIR / "oracle_single_fix_samples.csv", index=False)
    summaries = (
        result.groupby("section", sort=False)
        .agg(
            num_fixes=("fix_error_m", "size"),
            within_30m_fraction=("fix_error_m", lambda errors: float((errors <= 30).mean())),
            median_error_m=("fix_error_m", "median"),
            median_score=("score", "median"),
            oracle_zoom0=("oracle_zoom0", "first"),
            oracle_angle0_deg=("oracle_angle0_deg", "first"),
        )
        .reset_index()
    )
    summaries.to_csv(DIAG_DIR / "oracle_single_fix_summary.csv", index=False)
    dataset.close()
    print("Q1 oracle-pair single-fix check:")
    print(
        summaries[
            ["section", "within_30m_fraction", "median_error_m", "median_score"]
        ].to_string(index=False, float_format=lambda value: f"{value:.3f}")
    )


def run_q3_only() -> None:
    """Rerun only oracle-calibrated policies using saved grid choices."""
    DIAG_DIR.mkdir(parents=True, exist_ok=True)
    calibration = pd.read_csv(alto.OUT_DIR / "calibration.csv").set_index("section")
    frozen_results = pd.read_csv(alto.OUT_DIR / "results.csv")
    oracle = pd.read_csv(DIAG_DIR / "oracle_calibration.csv").set_index("section")
    dataset = alto.AltoDataset(alto.TRAIN_ZIP, "Train")
    dataset.image_motion()
    full_travelled = np.r_[
        0.0,
        np.cumsum(np.linalg.norm(np.diff(dataset.truth, axis=0), axis=1)),
    ]
    ranges = alto.section_ranges(full_travelled)
    if len(ranges) != 8:
        dataset.close()
        raise RuntimeError(f"expected eight Train sections; found {len(ranges)}")

    configs = {config.name: config for config in alto.train_section_configurations()}
    rows = []
    for index, frame_range in enumerate(ranges, 1):
        name = f"section_{index}"
        section = alto.prepare_range(dataset, frame_range)
        saved = calibration.loc[name]
        if not (
            np.isclose(section.zoom0, saved.zoom0, atol=1e-6)
            and np.isclose(section.angle0, saved.angle0_deg, atol=1e-6)
        ):
            dataset.close()
            raise RuntimeError(f"recomputed calibration differs for {name}")
        section = replace(
            section, zoom0=float(saved.zoom0), angle0=float(saved.angle0_deg)
        )
        oracle_row = oracle.loc[name]
        oracle_zoom, oracle_angle = float(oracle_row.zoom), float(oracle_row.angle_deg)
        oracle_section = replace(section, zoom0=oracle_zoom, angle0=oracle_angle)
        for config_name in ("nearest_300", "sized_gate_1000"):
            result = alto.run_config(dataset, oracle_section, configs[config_name])
            frozen = frozen_results[
                (frozen_results.section == name)
                & (frozen_results.config == config_name)
            ].iloc[0]
            frozen_median = float(frozen["median"])
            oracle_median = float(np.median(result.error))
            rows.append(
                {
                    "section": name,
                    "label": "DIAGNOSTIC oracle zoom - uses truth",
                    "config": config_name,
                    "calibrated_zoom0": section.zoom0,
                    "calibrated_angle0_deg": section.angle0,
                    "oracle_zoom0": oracle_zoom,
                    "oracle_angle0_deg": oracle_angle,
                    "frozen_median_m": frozen_median,
                    "oracle_median_m": oracle_median,
                    "median_delta_m": oracle_median - frozen_median,
                    "frozen_end_m": float(frozen["end"]),
                    "oracle_end_m": float(result.error[-1]),
                    "oracle_used_fixes": int((result.log[:, 3] == 1).sum()),
                    "oracle_rejected_fixes": int((result.log[:, 3] == 0).sum()),
                }
            )
        print(
            f"{name}: nearest_300 {rows[-2]['oracle_median_m']:.1f} m; "
            f"sized_gate_1000 {rows[-1]['oracle_median_m']:.1f} m",
            flush=True,
        )
    results = pd.DataFrame(rows)
    results.to_csv(DIAG_DIR / "oracle_rerun.csv", index=False)
    print(
        results[
            ["section", "config", "frozen_median_m", "oracle_median_m", "median_delta_m"]
        ].to_string(index=False, float_format=lambda value: f"{value:.1f}")
    )
    dataset.close()


def run_q4_only() -> None:
    """Recompute the camera-only decomposition with the final-200-m track axis."""
    DIAG_DIR.mkdir(parents=True, exist_ok=True)
    train = alto.AltoDataset(alto.TRAIN_ZIP, "Train")
    train.image_motion()
    full_travelled = np.r_[
        0.0,
        np.cumsum(np.linalg.norm(np.diff(train.truth, axis=0), axis=1)),
    ]
    ranges = alto.section_ranges(full_travelled)
    if len(ranges) != 8:
        train.close()
        raise RuntimeError(f"expected eight Train sections; found {len(ranges)}")
    rows = [
        dead_reckoning_row(
            train, f"section_{index}", alto.prepare_range(train, frame_range)
        )
        for index, frame_range in enumerate(ranges, 1)
    ]
    val = alto.AltoDataset(alto.VAL_ZIP, "Val")
    val.image_motion()
    val_section = alto.prepare_range(val, (0, len(val.query)))
    rows.append(dead_reckoning_row(val, "Val", val_section))
    table = pd.DataFrame(rows)
    table.to_csv(DIAG_DIR / "dead_reckoning.csv", index=False)
    print(
        table[
            ["section", "along_track_error_m", "cross_track_error_m",
             "implied_scale_error_pct", "implied_heading_error_deg"]
        ].to_string(index=False, float_format=lambda value: f"{value:+.1f}")
    )
    val.close()
    train.close()


def run() -> None:
    DIAG_DIR.mkdir(parents=True, exist_ok=True)
    calibration = pd.read_csv(alto.OUT_DIR / "calibration.csv")
    frozen_results = pd.read_csv(alto.OUT_DIR / "results.csv")
    section_calibration = calibration.set_index("section")

    train = alto.AltoDataset(alto.TRAIN_ZIP, "Train")
    train.image_motion()
    full_travelled = np.r_[
        0.0,
        np.cumsum(np.linalg.norm(np.diff(train.truth, axis=0), axis=1)),
    ]
    ranges = alto.section_ranges(full_travelled)
    if len(ranges) != 8:
        train.close()
        raise RuntimeError(f"expected eight Train sections; found {len(ranges)}")

    sections: list[tuple[str, alto.PreparedRange]] = []
    for index, frame_range in enumerate(ranges, 1):
        name = f"section_{index}"
        section = alto.prepare_range(train, frame_range)
        row = section_calibration.loc[name]
        if not (
            np.isclose(section.zoom0, row.zoom0, atol=1e-6)
            and np.isclose(section.angle0, row.angle0_deg, atol=1e-6)
        ):
            train.close()
            raise RuntimeError(f"recomputed calibration differs from calibration.csv for {name}")
        section = replace(
            section, zoom0=float(row.zoom0), angle0=float(row.angle0_deg)
        )
        sections.append((name, section))

    val = alto.AltoDataset(alto.VAL_ZIP, "Val")
    val.image_motion()
    val_section = alto.prepare_range(val, (0, len(val.query)))

    # Q1: use exactly the pipeline's per-fix zoom/angle grids, but centre nearest-7 on truth.
    q1_summaries = []
    q1_samples = []
    summary, rows = calibrated_fix_rows(val, "Val", val_section)
    q1_summaries.append(summary)
    q1_samples.extend(rows)
    for name, section in sections:
        summary, rows = calibrated_fix_rows(train, name, section)
        q1_summaries.append(summary)
        q1_samples.extend(rows)
    pd.DataFrame(q1_summaries).to_csv(DIAG_DIR / "single_fix_summary.csv", index=False)
    pd.DataFrame(q1_samples).to_csv(DIAG_DIR / "single_fix_samples.csv", index=False)

    # Q2: maximize median per-frame score at one fixed zoom/angle pair per section.
    print("Scoring wide oracle grid on truth-centred Train fixes (210 pairs/frame)...", flush=True)
    oracle_rows = []
    oracle_rows.extend(
        row
        for name, section in sections
        for row in oracle_score_rows(train, name, section)
    )
    oracle_summary, oracle_values = choose_oracles(oracle_rows, calibration)
    pd.DataFrame(oracle_rows).to_csv(DIAG_DIR / "oracle_grid_scores.csv", index=False)
    oracle_summary.to_csv(DIAG_DIR / "oracle_calibration.csv", index=False)
    run_oracle_fixes_only()

    # Q3: freeze every pipeline value except the selected oracle zoom0/angle0 pair.
    configs = {c.name: c for c in alto.train_section_configurations()}
    q3_rows = []
    print("Rerunning frozen nearest_300 and sized_gate_1000 with diagnostic oracle parameters...", flush=True)
    for name, section in sections:
        oracle_zoom, oracle_angle = oracle_values[name]
        oracle_section = replace(section, zoom0=oracle_zoom, angle0=oracle_angle)
        for config_name in ("nearest_300", "sized_gate_1000"):
            result = alto.run_config(train, oracle_section, configs[config_name])
            frozen = frozen_results[
                (frozen_results.section == name)
                & (frozen_results.config == config_name)
            ].iloc[0]
            frozen_median = float(frozen["median"])
            frozen_end = float(frozen["end"])
            oracle_median = float(np.median(result.error))
            q3_rows.append(
                {
                    "section": name,
                    "label": "DIAGNOSTIC oracle zoom - uses truth",
                    "config": config_name,
                    "calibrated_zoom0": section.zoom0,
                    "calibrated_angle0_deg": section.angle0,
                    "oracle_zoom0": oracle_zoom,
                    "oracle_angle0_deg": oracle_angle,
                    "frozen_median_m": frozen_median,
                    "oracle_median_m": oracle_median,
                    "median_delta_m": oracle_median - frozen_median,
                    "frozen_end_m": frozen_end,
                    "oracle_end_m": float(result.error[-1]),
                    "oracle_used_fixes": int((result.log[:, 3] == 1).sum()),
                    "oracle_rejected_fixes": int((result.log[:, 3] == 0).sum()),
                }
            )
    pd.DataFrame(q3_rows).to_csv(DIAG_DIR / "oracle_rerun.csv", index=False)

    # Q4: camera-only error projected onto the final 200 m true-track axis.
    q4_rows = [dead_reckoning_row(train, name, section) for name, section in sections]
    q4_rows.append(dead_reckoning_row(val, "Val", val_section))
    pd.DataFrame(q4_rows).to_csv(DIAG_DIR / "dead_reckoning.csv", index=False)

    print("\nQ1: truth-centred calibrated single fixes (Val comparator + eight sections)")
    print(
        pd.DataFrame(q1_summaries)[
            ["section", "num_fixes", "within_30m_fraction", "median_error_m", "median_score"]
        ].to_string(index=False, float_format=lambda value: f"{value:.3f}")
    )
    print("\nQ2: oracle fixed zoom/angle by maximum median score")
    print(
        oracle_summary[
            [
                "section", "calibrated_zoom0", "zoom", "angle_deg", "median_score",
                "calibrated_zoom_at_edge", "oracle_zoom_outside_calibration_grid",
                "oracle_zoom_at_wide_grid_edge", "edge_with_oracle_outside",
            ]
        ].to_string(index=False, float_format=lambda value: f"{value:.3f}")
    )
    print("\nQ3: frozen vs oracle pipeline section medians (oracle minus frozen, m)")
    print(
        pd.DataFrame(q3_rows)[
            ["section", "config", "frozen_median_m", "oracle_median_m", "median_delta_m"]
        ].to_string(index=False, float_format=lambda value: f"{value:.1f}")
    )
    print("\nQ4: camera-only scale/heading decomposition")
    print(
        pd.DataFrame(q4_rows)[
            ["section", "along_track_error_m", "cross_track_error_m", "implied_scale_error_pct", "implied_heading_error_deg"]
        ].to_string(index=False, float_format=lambda value: f"{value:+.1f}")
    )
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--oracle-fixes-only", action="store_true")
    modes.add_argument("--q3-only", action="store_true")
    modes.add_argument("--q4-only", action="store_true")
    args = parser.parse_args()
    if args.oracle_fixes_only:
        DIAG_DIR.mkdir(parents=True, exist_ok=True)
        run_oracle_fixes_only()
    elif args.q3_only:
        run_q3_only()
    elif args.q4_only:
        run_q4_only()
    else:
        run()

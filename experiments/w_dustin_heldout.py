#!/usr/bin/env python3
"""Run Dustin's frozen one-map ALTO navigator on Round 2 Train sections."""
from __future__ import annotations

import json
import sys
import time
import zipfile
from dataclasses import replace
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import pandas as pd
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
DUSTIN_ROOT = ROOT.parent / "TaipeiDrift-alto-nav"
BASELINE = DUSTIN_ROOT / "baseline"
sys.path.insert(0, str(BASELINE))

from src.data.alto import (  # noqa: E402
    AltoConfig,
    FRAME_RATE_HZ,
    MAIN_REFERENCE_FOLDER,
    REFERENCE_METRES_PER_PIXEL,
)
from src.data.camera_flight import CameraFlight, ReferenceMap  # noqa: E402
from src.data.ground_map import mosaic  # noqa: E402
from src.estimation.camera_navigator import (  # noqa: E402
    NavigatorConfig,
    calibrate,
    navigate,
)
from src.estimation.image_motion import shifts_for_flight  # noqa: E402
from src.evaluation.navigation_metrics import (  # noqa: E402
    fix_errors,
    navigation_errors,
    summarize_navigation,
)

OUT_DIR = ROOT / "data/processed/w_dustin_heldout"
ARCHIVE = ROOT / "data/raw/alto/UAV_Round2_Train.zip"
VAL_RESULTS = ROOT / "data/processed/t_alto_heldout/results.csv"
CONFIG_PATH = BASELINE / "configs/alto_navigator.yaml"
EXPECTED_FOLDERS = {
    "offset_0_None",
    "offset_40_North",
    "offset_40_South",
}
RUN_ORDER = (
    "camera_only",
    "map_every_100",
    "map_every_300",
    "map_every_400_no_check",
    "map_every_1000",
)
LEAKY_COMPARATORS = {
    "camera_only": "none",
    "map_every_100": "nearest_100",
    "map_every_300": "sized_gate_300",
    "map_every_400_no_check": None,
    "map_every_1000": "sized_gate_1000",
}


def section_ranges(travelled: np.ndarray) -> list[tuple[int, int]]:
    """Same 4.6-km section boundaries and short-tail merge as t_alto_heldout.py."""
    n = len(travelled)
    total = float(travelled[-1])
    boundaries = [0]
    target = 4600.0
    while target < total:
        index = int(np.searchsorted(travelled, target))
        if index <= boundaries[-1] or index >= n:
            break
        boundaries.append(index)
        target += 4600.0
    if boundaries[-1] != n:
        boundaries.append(n)
    if len(boundaries) > 2 and total - travelled[boundaries[-2]] <= 2000.0:
        boundaries.pop(-2)
    return list(zip(boundaries[:-1], boundaries[1:]))


def load_round2_flight(
    cfg: AltoConfig, archive_path: Path
) -> tuple[CameraFlight, zipfile.ZipFile, dict]:
    """Adapt the complete Round 2 archive while using Dustin's map and flight classes."""
    if not archive_path.is_file():
        raise FileNotFoundError(archive_path)
    archive = zipfile.ZipFile(archive_path)
    section = cfg.section
    query = pd.read_csv(archive.open(f"{section}/query.csv"))
    references = pd.read_csv(archive.open(f"{section}/reference.csv"))
    folder_counts = references.name.str.split("/").str[0].value_counts().to_dict()
    if set(folder_counts) != EXPECTED_FOLDERS:
        raise RuntimeError(f"unexpected Round 2 reference folders: {folder_counts}")
    if not cfg.ground_map:
        raise ValueError("the leak-free Round 2 run requires one map of all reference folders")

    origin = query[["northing", "easting"]].to_numpy()[0]

    def decode(member: str) -> np.ndarray:
        image = cv2.imdecode(np.frombuffer(archive.read(member), np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"could not decode {member}")
        return image

    reference_names = references.name.tolist()
    centres = references[["northing", "easting"]].to_numpy() - origin
    ground_map = mosaic(
        centres,
        lambda i: decode(f"{section}/reference_images/{reference_names[i]}"),
        cfg.metres_per_pixel,
        zoom_unit_px=500,
    )

    on_route = references[
        references.name.str.startswith(f"{MAIN_REFERENCE_FOLDER}/")
    ].reset_index(drop=True)
    route_names = on_route.name.tolist()
    reference_map = ReferenceMap(
        position=on_route[["northing", "easting"]].to_numpy() - origin,
        metres_per_pixel=cfg.metres_per_pixel,
        load=lambda i: decode(f"{section}/reference_images/{route_names[i]}"),
    )
    flight = CameraFlight(
        name="alto_round2_train",
        timestamp=np.arange(len(query)) / FRAME_RATE_HZ,
        position_gt=query[["northing", "easting"]].to_numpy() - origin,
        load_frame=lambda i: decode(f"{section}/query_images/{query.name.iloc[i]}"),
        reference=reference_map,
        metadata={
            "source": "alto_round2",
            "file": str(archive_path),
            "section": section,
            "origin_northing_easting": origin.tolist(),
            "reference_folders": folder_counts,
        },
        ground_map=ground_map,
    )
    map_info = {
        "reference_rows": len(references),
        "reference_folders": folder_counts,
        "shape_px": list(ground_map.image.shape),
        "covered_fraction": float(ground_map.covered.mean()),
        "metres_per_pixel": ground_map.metres_per_pixel,
    }
    return flight, archive, map_info


def section_flight(full: CameraFlight, start: int, stop: int, number: int) -> CameraFlight:
    """Slice a flight without changing the shared map or reference coordinate frame."""
    return CameraFlight(
        name=f"alto_round2_train_section_{number}",
        timestamp=full.timestamp[start:stop],
        position_gt=full.position_gt[start:stop],
        load_frame=lambda i, i0=start: full.frame(i0 + int(i)),
        reference=full.reference,
        metadata={**full.metadata, "frame_start": start, "frame_stop": stop},
        ground_map=full.ground_map,
    )


def run_configurations() -> dict[str, NavigatorConfig]:
    raw = yaml.safe_load(CONFIG_PATH.read_text())
    shared = NavigatorConfig(**raw["navigator"])
    return {
        name: replace(shared, **raw["runs"][name])
        for name in RUN_ORDER
    }


def make_summary(results: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for config in RUN_ORDER:
        group = results[results.config == config]
        medians = group.median_error_m
        worst = group.worst_error_m
        ends = group.end_error_m
        within = group.within_3sigma_pct
        rows.append(
            {
                "config": config,
                "median_of_section_medians_m": float(medians.median()),
                "min_section_median_m": float(medians.min()),
                "max_section_median_m": float(medians.max()),
                "median_section_worst_m": float(worst.median()),
                "min_section_worst_m": float(worst.min()),
                "max_section_worst_m": float(worst.max()),
                "median_section_end_m": float(ends.median()),
                "min_section_end_m": float(ends.min()),
                "max_section_end_m": float(ends.max()),
                "median_within_3sigma_pct": float(within.median()),
                "min_within_3sigma_pct": float(within.min()),
                "max_within_3sigma_pct": float(within.max()),
                "total_fixes_used": int(group.fixes_used.sum()),
                "total_fixes_rejected": int(group.fixes_rejected.sum()),
                "total_wrong_gt50": int(group.wrong_gt50.sum()),
                "median_navigation_runtime_s": float(group.navigation_runtime_s.median()),
            }
        )
    return pd.DataFrame(rows)


def make_comparison(results: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    legacy = pd.read_csv(VAL_RESULTS)
    legacy = legacy[legacy.section != "full"].copy()
    records = []
    for row in results.to_dict(orient="records"):
        old_name = LEAKY_COMPARATORS[row["config"]]
        old = None
        if old_name is not None:
            match = legacy[
                (legacy.section == row["section"]) & (legacy.config == old_name)
            ]
            if len(match) != 1:
                raise RuntimeError(
                    f"expected one leaky comparator for {row['section']} {old_name}; found {len(match)}"
                )
            old = match.iloc[0]
            if int(old.i0) != int(row["i0"]) or int(old.i1) != int(row["i1"]):
                raise RuntimeError(f"section boundaries differ for {row['section']}")
        records.append(
            {
                "section": row["section"],
                "config": row["config"],
                "leaky_config": old_name or "unavailable_same_spacing",
                "round2_median_m": row["median_error_m"],
                "round2_worst_m": row["worst_error_m"],
                "round2_end_m": row["end_error_m"],
                "round2_used": row["fixes_used"],
                "round2_rejected": row["fixes_rejected"],
                "round2_wrong_gt50": row["wrong_gt50"],
                "leaky_median_m": float(old["median"]) if old is not None else np.nan,
                "leaky_worst_m": float(old["worst"]) if old is not None else np.nan,
                "leaky_end_m": float(old["end"]) if old is not None else np.nan,
                "leaky_used": int(old["used"]) if old is not None else np.nan,
                "leaky_rejected": int(old["rejected"]) if old is not None else np.nan,
                "leaky_wrong_gt50": int(old["wrong_gt50"]) if old is not None else np.nan,
            }
        )
    comparison = pd.DataFrame(records)
    summary_rows = []
    for config in RUN_ORDER:
        group = comparison[comparison.config == config]
        paired = group.dropna(subset=["leaky_median_m"])
        summary_rows.append(
            {
                "config": config,
                "leaky_config": LEAKY_COMPARATORS[config] or "unavailable_same_spacing",
                "round2_median_of_section_medians_m": float(group.round2_median_m.median()),
                "round2_median_range_m": f"{group.round2_median_m.min():.1f}–{group.round2_median_m.max():.1f}",
                "round2_median_section_worst_m": float(group.round2_worst_m.median()),
                "round2_median_section_end_m": float(group.round2_end_m.median()),
                "leaky_median_of_section_medians_m": (
                    float(paired.leaky_median_m.median()) if len(paired) else np.nan
                ),
                "leaky_median_range_m": (
                    f"{paired.leaky_median_m.min():.1f}–{paired.leaky_median_m.max():.1f}"
                    if len(paired)
                    else "N/A"
                ),
                "leaky_median_section_worst_m": (
                    float(paired.leaky_worst_m.median()) if len(paired) else np.nan
                ),
                "leaky_median_section_end_m": (
                    float(paired.leaky_end_m.median()) if len(paired) else np.nan
                ),
            }
        )
    return comparison, pd.DataFrame(summary_rows)


def plot_300m_error(series: dict[str, tuple[np.ndarray, np.ndarray]], output: Path) -> None:
    fig, axes = plt.subplots(4, 2, figsize=(13, 12), sharex=False)
    for index, (section, (distance, error)) in enumerate(series.items()):
        ax = axes.flat[index]
        ax.plot(distance, error, linewidth=0.8)
        ax.set_title(section.replace("section_", "Section "))
        ax.set_xlabel("Distance after GNSS jam (m)")
        ax.set_ylabel("Horizontal error (m)")
        ax.grid(alpha=0.25)
    fig.suptitle("ALTO Round 2 Train — one map, fix every 300 m, score ≥0.33")
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main() -> None:
    started = time.perf_counter()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if not ARCHIVE.is_file():
        raise FileNotFoundError(ARCHIVE)

    config_path = ROOT / "data/raw/alto/round2/Train.zip"
    if config_path.exists() and not zipfile.is_zipfile(config_path):
        print(
            f"Track C loader path is not a ZIP ({config_path.stat().st_size} bytes); "
            f"reading the complete archive at {ARCHIVE}",
            flush=True,
        )
    alto_cfg = AltoConfig(
        data_root=str(ARCHIVE.parent),
        section="Train",
        metres_per_pixel=REFERENCE_METRES_PER_PIXEL,
        ground_map=True,
        map_cache_dir=None,
    )
    map_started = time.perf_counter()
    flight, archive, map_info = load_round2_flight(alto_cfg, ARCHIVE)
    map_runtime = time.perf_counter() - map_started
    print(
        f"Round 2 map: {map_info['reference_rows']} reference images from "
        f"{map_info['reference_folders']}; shape={map_info['shape_px']} px, "
        f"covered={map_info['covered_fraction']:.3f}, {map_runtime:.1f} s",
        flush=True,
    )

    travelled = flight.travelled
    ranges = section_ranges(travelled)
    if len(ranges) != 8:
        archive.close()
        raise RuntimeError(f"protocol expected 8 sections; generated {len(ranges)}")
    print(
        f"Train frames={len(flight)}, travelled={travelled[-1] / 1000:.3f} km, "
        f"sections={len(ranges)}",
        flush=True,
    )

    flow_path = OUT_DIR / "flow_round2_train.npy"
    flow_started = time.perf_counter()
    shifts = shifts_for_flight(flight, flow_path)
    flow_runtime = time.perf_counter() - flow_started
    print(f"Image motion ready: {flow_path} ({flow_runtime:.1f} s)", flush=True)

    run_cfgs = run_configurations()
    results_path = OUT_DIR / "results.csv"
    calibration_path = OUT_DIR / "calibration.csv"
    result_rows: list[dict] = []
    calibration_rows: list[dict] = []
    error_series: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    navigation_runtime_total = 0.0

    for number, (i0, i1) in enumerate(ranges, 1):
        name = f"section_{number}"
        subflight = section_flight(flight, i0, i1, number)
        local_travelled = subflight.travelled
        km = float((travelled[i1 - 1] - travelled[i0]) / 1000.0)
        calibrations = {}
        for run_name, run_cfg in run_cfgs.items():
            family = "map" if run_cfg.search == "area" else "reference_images"
            if family not in calibrations:
                calibration_started = time.perf_counter()
                c = calibrate(subflight, shifts[i0:i1], run_cfg)
                calibration_runtime = time.perf_counter() - calibration_started
                calibrations[family] = c
                calibration_rows.append(
                    {
                        "section": name,
                        "i0": i0,
                        "i1": i1,
                        "km": km,
                        "search_family": family,
                        "jam_frame_local": c.jam_index,
                        "jam_frame_global": i0 + c.jam_index,
                        "jam_distance_m": float(local_travelled[c.jam_index]),
                        "zoom": c.zoom,
                        "angle_deg": c.angle,
                        "fix_offset_north_m": float(c.fix_offset[0]),
                        "fix_offset_east_m": float(c.fix_offset[1]),
                        "calibration_runtime_s": calibration_runtime,
                    }
                )
                print(
                    f"{name} [{i0}:{i1}) {km:.2f} km; {family} calibration "
                    f"zoom={c.zoom:.2f}, angle={c.angle:.0f}°, "
                    f"jam={local_travelled[c.jam_index]:.1f} m",
                    flush=True,
                )
                pd.DataFrame(calibration_rows).to_csv(calibration_path, index=False)

        for run_name, run_cfg in run_cfgs.items():
            family = "map" if run_cfg.search == "area" else "reference_images"
            t0 = time.perf_counter()
            result = navigate(
                subflight,
                shifts[i0:i1],
                calibrations[family],
                run_cfg,
            )
            runtime_s = time.perf_counter() - t0
            navigation_runtime_total += runtime_s
            errors = navigation_errors(result, subflight)
            summary = summarize_navigation(result, subflight)
            within_count = int(np.count_nonzero(errors.error <= 3.0 * errors.sigma))
            within_frames = len(errors.error)
            row = {
                "section": name,
                "i0": i0,
                "i1": i1,
                "km": km,
                "config": run_name,
                "median_error_m": summary.median,
                "worst_error_m": summary.worst,
                "end_error_m": summary.end,
                "fixes_used": summary.fixes_used,
                "fixes_rejected": summary.fixes_rejected,
                "wrong_gt50": summary.used_but_wrong,
                "within_3sigma_frames": within_count,
                "postjam_frames": within_frames,
                "within_3sigma_pct": 100.0 * within_count / within_frames,
                "navigation_runtime_s": runtime_s,
            }
            result_rows.append(row)
            pd.DataFrame(result_rows).to_csv(results_path, index=False)
            if run_name == "map_every_300":
                error_series[name] = (errors.distance_since_jam.copy(), errors.error.copy())
            print(
                f"{name} {run_name}: median={summary.median:.1f} m, "
                f"worst={summary.worst:.1f} m, end={summary.end:.1f} m; "
                f"fixes {summary.fixes_used} used/{summary.fixes_rejected} rejected/"
                f"{summary.used_but_wrong} wrong>50; within 3σ {within_count}/{within_frames} "
                f"({row['within_3sigma_pct']:.1f}%); {runtime_s:.1f} s",
                flush=True,
            )

    results = pd.DataFrame(result_rows)
    summary = make_summary(results)
    summary.to_csv(OUT_DIR / "summary.csv", index=False)
    comparison, comparison_summary = make_comparison(results)
    comparison.to_csv(OUT_DIR / "comparison.csv", index=False)
    comparison_summary.to_csv(OUT_DIR / "comparison_summary.csv", index=False)
    pd.DataFrame(calibration_rows).to_csv(calibration_path, index=False)
    plot_300m_error(error_series, OUT_DIR / "map_every_300_error_by_section.png")

    total_runtime = time.perf_counter() - started
    run_info = {
        "status": "MEASURED",
        "archive": str(ARCHIVE.relative_to(ROOT)),
        "implementation_worktree": str(DUSTIN_ROOT),
        "section_count": len(ranges),
        "section_ranges": [
            {"section": f"section_{i}", "i0": start, "i1": stop}
            for i, (start, stop) in enumerate(ranges, 1)
        ],
        "configs": RUN_ORDER,
        "truth_after_jam_used_only_for_scoring": True,
        "map": map_info,
        "map_build_runtime_s": map_runtime,
        "image_motion_runtime_s": flow_runtime,
        "navigation_runtime_sum_s": navigation_runtime_total,
        "total_runtime_s": total_runtime,
    }
    (OUT_DIR / "run.json").write_text(json.dumps(run_info, indent=2))

    print("Round 2 section summary (median and min–max are across section values):", flush=True)
    print(
        summary[
            [
                "config",
                "median_of_section_medians_m",
                "min_section_median_m",
                "max_section_median_m",
                "median_section_worst_m",
                "median_section_end_m",
                "total_fixes_used",
                "total_fixes_rejected",
                "total_wrong_gt50",
            ]
        ].to_string(index=False, float_format=lambda value: f"{value:.1f}"),
        flush=True,
    )
    print("Round 2 vs. leaky t_alto_heldout (same configuration where available):", flush=True)
    print(
        comparison_summary.to_string(index=False, float_format=lambda value: f"{value:.1f}"),
        flush=True,
    )
    print(
        f"Runtimes: map build {map_runtime:.1f} s; image motion {flow_runtime:.1f} s; "
        f"navigation sum {navigation_runtime_total:.1f} s; total {total_runtime:.1f} s",
        flush=True,
    )
    print(f"Per-section results: {results_path}", flush=True)
    print(f"Summary: {OUT_DIR / 'summary.csv'}", flush=True)
    print(f"Comparison: {OUT_DIR / 'comparison.csv'}", flush=True)
    print(f"Figure: {OUT_DIR / 'map_every_300_error_by_section.png'}", flush=True)
    print(
        "Limit: Round 2 references cover only the route and ±40 m, not Val's five-folder ±190 m strip.",
        flush=True,
    )
    archive.close()


if __name__ == "__main__":
    main()

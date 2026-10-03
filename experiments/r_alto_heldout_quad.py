"""Compare the frozen ALTO fix policy with a quadrant-gated ZNCC fix on Round 2 Train.

Run from the repository root with:
  .venv/bin/python experiments/r_alto_heldout_quad.py
Baseline sections are rerun and checked against t_alto_heldout/results.csv before
any quad-gated runs begin. Truth after the GNSS cut is used only for scoring.
"""
from __future__ import annotations

import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "experiments"))
import r_map_benchmark as B  # noqa: E402
import t_alto_heldout as H  # noqa: E402

OUT_DIR = ROOT / "data/processed/r_alto_heldout_quad"
ZOOM_SCALES = (0.65, 0.75, 0.85, 0.95, 1.05)
QUAD_ANGLES = (-10.0, -5.0, 0.0, 5.0, 10.0)
CONFIGS = (
    H.RunConfig("nearest_100", 100.0),
    H.RunConfig("nearest_300", 300.0),
    H.RunConfig("sized_gate_1000", 1000.0, sized_search=True, min_score=H.MIN_SCORE),
)


def _query160(frame: np.ndarray, zoom: float, heading_deg: float) -> np.ndarray:
    """Apply the harness zoom and heading correction, then extract B.QUERY pixels."""
    gray = np.clip(frame, 0, 255).astype(np.uint8, copy=False)
    height = max(B.QUERY, int(round(gray.shape[0] * zoom)))
    width = max(B.QUERY, int(round(gray.shape[1] * zoom)))
    scaled = cv2.resize(gray, (width, height), interpolation=cv2.INTER_AREA)
    matrix = cv2.getRotationMatrix2D((width / 2.0, height / 2.0), heading_deg, 1.0)
    north_up = cv2.warpAffine(
        scaled,
        matrix,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    top = (height - B.QUERY) // 2
    left = (width - B.QUERY) // 2
    query = north_up[top : top + B.QUERY, left : left + B.QUERY]
    if query.shape != (B.QUERY, B.QUERY) or query.dtype != np.uint8:
        raise ValueError(f"Prepared ZNCC query has invalid format: {query.shape}, {query.dtype}")
    return np.ascontiguousarray(query)


def _reference_window(
    dataset: H.AltoDataset,
    ref_index: int,
    prior: np.ndarray,
    valid_cache: dict[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Recenter one measured ALTO tile on the estimate, never on query truth."""
    name = str(dataset.reference.name.iloc[ref_index])
    member = f"{dataset.prefix}/reference_images/{name}"
    tile = np.clip(dataset.load_image(member), 0, 255).astype(np.uint8, copy=False)
    valid = valid_cache.get(name)
    if valid is None:
        raw = np.frombuffer(dataset.archive.read(member), np.uint8)
        decoded = cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)
        if decoded is None:
            raise FileNotFoundError(f"Could not decode ALTO reference tile: {member}")
        valid = (decoded != 0).astype(np.uint8)
        valid_cache[name] = valid

    tile_centre = dataset.ref_xy[ref_index]
    px = tile.shape[1] / 2.0 + (prior[0] - tile_centre[0]) / H.REF_MPP
    py = tile.shape[0] / 2.0 - (prior[1] - tile_centre[1]) / H.REF_MPP
    matrix = np.array(
        [[1.0, 0.0, B.SEARCH / 2.0 - px], [0.0, 1.0, B.SEARCH / 2.0 - py]],
        dtype=np.float32,
    )
    reference = cv2.warpAffine(
        tile,
        matrix,
        (B.SEARCH, B.SEARCH),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    window_valid = cv2.warpAffine(
        valid,
        matrix,
        (B.SEARCH, B.SEARCH),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return np.ascontiguousarray(reference), np.ascontiguousarray(window_valid)


def quad_fix(
    query_frame: np.ndarray,
    reference: np.ndarray,
    valid: np.ndarray,
    prior: np.ndarray,
    zoom: float,
    heading_deg: float,
) -> tuple[float, np.ndarray | None, float, float, int]:
    """Return a locally estimated fix only when at least three quadrants agree.

    Position is in UTM metres. A rejected match returns ``None`` for position,
    matching the held-out harness's no-fix convention. No truth is an argument.
    """
    query = _query160(query_frame, zoom, heading_deg)
    point, stats = B.zncc(query, reference, valid, angles=QUAD_ANGLES, scales=ZOOM_SCALES)
    score = float(stats.get("score", 0.0))
    quad_n = int(stats.get("quad_n", 0))
    if not np.isfinite(point).all() or quad_n < 3:
        return score, None, zoom, float(stats.get("hyp_angle", 0.0)), quad_n

    position = prior + np.array(
        [(point[0] - B.SEARCH / 2.0) * H.REF_MPP,
         -(point[1] - B.SEARCH / 2.0) * H.REF_MPP],
        dtype=float,
    )
    new_zoom = float(np.clip(zoom * float(stats.get("hyp_scale", 1.0)), 0.5, 1.1))
    return score, position, new_zoom, float(stats.get("hyp_angle", 0.0)), quad_n


def _quad_search(
    dataset: H.AltoDataset,
    absolute_k: int,
    prior: np.ndarray,
    zooms: np.ndarray,
    heading_deg: float,
    candidates: np.ndarray,
    valid_cache: dict[str, np.ndarray],
) -> tuple[float, np.ndarray | None, float, float, int]:
    """Search the frozen zoom/candidate grids; the top-scoring match must pass quad."""
    query_name = f"{dataset.prefix}/query_images/{dataset.query.name.iloc[absolute_k]}"
    frame = dataset.load_image(query_name)
    best: tuple[float, np.ndarray | None, float, float, int] = (-1.0, None, 0.0, 0.0, 0)
    best_score = -1.0
    for zoom in zooms:
        for ref_index in candidates:
            reference, valid = _reference_window(dataset, int(ref_index), prior, valid_cache)
            score, position, new_zoom, angle, quad_n = quad_fix(
                frame, reference, valid, prior, float(zoom), heading_deg
            )
            if score > best_score:
                best_score = score
                best = (score, position, new_zoom, angle, quad_n)
    if best[1] is None:
        return best
    return best


def _run_quad_config(
    dataset: H.AltoDataset,
    section: H.PreparedRange,
    config: H.RunConfig,
    valid_cache: dict[str, np.ndarray],
) -> tuple[H.RunResult, int, int]:
    """Run the held-out filter with a pure quadrant gate and score-only truth access."""
    estimate = section.truth[section.jam].copy()  # last position available from pre-jam GNSS
    path = [estimate.copy()]
    variance = 3.0**2
    since_fix = 0.0
    since_try = 0.0
    zoom = section.zoom0
    scale = 1.0
    log: list[tuple[int, float, float, float]] = []
    attempted = accepted = 0

    for k in range(section.jam + 1, len(section.truth)):
        step = dataset.flow[section.i0 + k] @ section.A0 * scale
        estimate = estimate + step
        since_fix += np.linalg.norm(step)
        since_try += np.linalg.norm(step)
        if config.fix_every_m and since_try >= config.fix_every_m:
            attempted += 1
            since_try = 0.0
            predicted_var = variance + (H.DRIFT_RATE * since_fix) ** 2
            zooms = np.clip(zoom + np.arange(-0.10, 0.11, 0.05), 0.5, 1.1)
            candidates = dataset.nearest(estimate, H.CALIB_NEAREST)
            if config.sized_search:
                radius = max(60.0, 3 * np.sqrt(predicted_var))
                inside = np.where(np.linalg.norm(dataset.ref_xy - estimate, axis=1) <= radius)[0]
                candidates = inside if len(inside) >= H.CALIB_NEAREST else candidates
                if since_fix > 400:
                    zooms = np.arange(0.60, 1.101, 0.05)

            score, position, new_zoom, _, quad_n = _quad_search(
                dataset,
                section.i0 + k,
                estimate,
                zooms,
                section.angle0,
                candidates,
                valid_cache,
            )
            if position is None:
                agrees = False
                use = False
                fix_error = float("nan")
            else:
                position = position - section.offset0
                agrees = np.linalg.norm(position - estimate) <= 3 * np.sqrt(
                    predicted_var + H.FIX_SIGMA**2
                )
                use = bool(agrees and score >= config.min_score and quad_n >= 3)
                # Scoring only: this value is never used in the fix decision or state update.
                fix_error = float(np.linalg.norm(position - section.truth[k]))
            log.append((section.i0 + k, score, fix_error, float(use)))
            if use:
                accepted += 1
                gain = predicted_var / (predicted_var + H.FIX_SIGMA**2)
                estimate = estimate + gain * (position - estimate)
                variance = (1 - gain) * predicted_var
                scale, zoom, since_fix = new_zoom / section.zoom0, new_zoom, 0.0
        path.append(estimate.copy())

    path_array = np.asarray(path)
    errors = np.linalg.norm(path_array - section.truth[section.jam :], axis=1)
    return H.RunResult(path_array, errors, np.asarray(log, dtype=float).reshape(-1, 4)), attempted, accepted


def _baseline_rows(
    dataset: H.AltoDataset,
    sections: list[tuple[str, H.PreparedRange, float]],
) -> list[dict]:
    saved = pd.read_csv(H.OUT_DIR / "results.csv")
    saved_summary = pd.read_csv(H.OUT_DIR / "summary.csv").set_index("config")
    out: list[dict] = []
    compared: dict[str, list[dict]] = {config.name: [] for config in CONFIGS}
    for section_name, section, km in sections:
        for config in CONFIGS:
            result = H.run_config(dataset, section, config)
            used, rejected, wrong, _ = H.error_counts(result)
            row = {
                "section": section_name,
                "i0": section.i0,
                "i1": section.i1,
                "section_km": km,
                "policy": "baseline",
                "spacing": config.name,
                "median_error_m": float(np.median(result.error)),
                "final_error_m": float(result.error[-1]),
                "fixes_attempted": int(used + rejected),
                "fixes_accepted": used,
                "accepted_wrong_gt50_m": wrong,
                "runtime_s": 0.0,
            }
            saved_row = saved[(saved.section == section_name) & (saved.config == config.name)]
            if len(saved_row) != 1:
                raise RuntimeError(f"Expected one saved baseline row for {section_name}/{config.name}")
            expected = saved_row.iloc[0]
            pairs = (
                ("median_error_m", "median"),
                ("final_error_m", "end"),
                ("fixes_accepted", "used"),
                ("accepted_wrong_gt50_m", "wrong_gt50"),
                ("fixes_attempted", None),
            )
            for actual_name, expected_name in pairs:
                actual = row[actual_name]
                target = (int(expected.used) + int(expected.rejected)) if expected_name is None else expected[expected_name]
                if not np.isclose(actual, target, rtol=0.0, atol=1e-6):
                    raise RuntimeError(
                        f"Baseline mismatch {section_name}/{config.name} {actual_name}: "
                        f"rerun={actual}, saved={target}"
                    )
            out.append(row)
            compared[config.name].append(row)
            print(
                f"baseline {section_name} {config.name}: median={row['median_error_m']:.1f} m, "
                f"final={row['final_error_m']:.1f} m, fixes={used}/{used + rejected} accepted/attempted, "
                f"wrong>50m={wrong}",
                flush=True,
            )

    for config_name, rows in compared.items():
        group = saved_summary.loc[config_name]
        medians = np.asarray([row["median_error_m"] for row in rows])
        totals = {
            "total_used": sum(row["fixes_accepted"] for row in rows),
            "total_wrong_gt50": sum(row["accepted_wrong_gt50_m"] for row in rows),
        }
        checks = (
            (float(np.median(medians)), float(group["median"])),
            (float(np.min(medians)), float(group["min"])),
            (float(np.max(medians)), float(group["max"])),
            (float(totals["total_used"]), float(group["total_used"])),
            (float(totals["total_wrong_gt50"]), float(group["total_wrong_gt50"])),
        )
        if not all(np.isclose(actual, expected, rtol=0.0, atol=1e-6) for actual, expected in checks):
            raise RuntimeError(f"Baseline summary mismatch for {config_name}: {checks}")
        print(
            f"baseline summary {config_name}: section-median median/range="
            f"{np.median(medians):.1f} [{np.min(medians):.1f}, {np.max(medians):.1f}] m; "
            f"saved outputs match",
            flush=True,
        )
    return out


def _row_from_quad(
    section_name: str,
    section: H.PreparedRange,
    km: float,
    config: H.RunConfig,
    result: H.RunResult,
    attempted: int,
    accepted: int,
    runtime_s: float,
) -> dict:
    accepted_log = result.log[result.log[:, 3] == 1] if len(result.log) else np.empty((0, 4))
    wrong = int(np.sum(accepted_log[:, 2] > 50.0)) if len(accepted_log) else 0
    return {
        "section": section_name,
        "i0": section.i0,
        "i1": section.i1,
        "section_km": km,
        "policy": "quad_gated",
        "spacing": config.name,
        "median_error_m": float(np.median(result.error)),
        "final_error_m": float(result.error[-1]),
        "fixes_attempted": attempted,
        "fixes_accepted": accepted,
        "accepted_wrong_gt50_m": wrong,
        "runtime_s": runtime_s,
    }


def _aggregate(rows: list[dict]) -> list[dict]:
    frame = pd.DataFrame(rows)
    summary: list[dict] = []
    for (policy, spacing), group in frame.groupby(["policy", "spacing"], sort=False):
        medians = group["median_error_m"].to_numpy(dtype=float)
        summary.append(
            {
                "policy": str(policy),
                "spacing": str(spacing),
                "sections": int(len(group)),
                "section_median_error_m_median": float(np.median(medians)),
                "section_median_error_m_min": float(np.min(medians)),
                "section_median_error_m_max": float(np.max(medians)),
                "final_error_m_median": float(group["final_error_m"].median()),
                "fixes_attempted": int(group["fixes_attempted"].sum()),
                "fixes_accepted": int(group["fixes_accepted"].sum()),
                "accepted_wrong_gt50_m": int(group["accepted_wrong_gt50_m"].sum()),
            }
        )
    return summary


def main() -> None:
    cv2.setNumThreads(1)
    if not (H.OUT_DIR / "results.csv").is_file() or not (H.OUT_DIR / "summary.csv").is_file():
        raise FileNotFoundError("Saved t_alto_heldout results.csv and summary.csv are required")

    dataset = H.AltoDataset(H.TRAIN_ZIP, "Train")
    valid_cache: dict[str, np.ndarray] = {}
    try:
        dataset.image_motion()
        full_travelled = np.r_[
            0.0,
            np.cumsum(np.linalg.norm(np.diff(dataset.truth, axis=0), axis=1)),
        ]
        ranges = H.section_ranges(full_travelled)
        if len(ranges) != 8:
            raise RuntimeError(f"Expected 8 held-out sections, found {len(ranges)}")
        sections: list[tuple[str, H.PreparedRange, float]] = []
        for number, (i0, i1) in enumerate(ranges, 1):
            section = H.prepare_range(dataset, (i0, i1))
            km = float((full_travelled[i1 - 1] - full_travelled[i0]) / 1000.0)
            sections.append((f"section_{number}", section, km))

        started = time.time()
        rows = _baseline_rows(dataset, sections)
        baseline_s = time.time() - started
        print(f"baseline reproduction PASS: selected frozen configs match saved outputs ({baseline_s:.1f} s)", flush=True)

        for section_name, section, km in sections:
            for config in CONFIGS:
                started = time.time()
                result, attempted, accepted = _run_quad_config(dataset, section, config, valid_cache)
                runtime_s = time.time() - started
                row = _row_from_quad(section_name, section, km, config, result, attempted, accepted, runtime_s)
                rows.append(row)
                print(
                    f"quad {section_name} {config.name}: median={row['median_error_m']:.1f} m, "
                    f"final={row['final_error_m']:.1f} m, fixes={accepted}/{attempted} accepted/attempted, "
                    f"wrong>50m={row['accepted_wrong_gt50_m']}; {runtime_s:.1f} s",
                    flush=True,
                )
    finally:
        dataset.close()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    sections_path = OUT_DIR / "sections.csv"
    summary_path = OUT_DIR / "summary.json"
    result_frame = pd.DataFrame(rows)
    result_frame.to_csv(sections_path, index=False)
    aggregate = _aggregate(rows)
    summary = {
        "experiment": "ALTO Round 2 Train held-out: legacy baseline vs quadrant-gated ZNCC",
        "evidence_label": "MEASURED (real ALTO Round 2 Train imagery, telemetry, and reference tiles)",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": ".venv/bin/python experiments/r_alto_heldout_quad.py",
        "dataset": {
            "archive": str(H.TRAIN_ZIP),
            "sections": 8,
            "section_frames_and_ranges_from": "experiments/t_alto_heldout.py section_ranges",
            "baseline_configs_reproduced": [config.name for config in CONFIGS],
            "baseline_comparison": "Fresh per-section medians, final errors, accepted counts, attempted counts, and wrong-accepted counts matched saved t_alto_heldout/results.csv; aggregate medians/ranges and counts matched summary.csv at absolute tolerance 1e-6.",
            "baseline_runtime_s": float(baseline_s),
        },
        "frozen_harness_parameters": {
            "reference_mpp": H.REF_MPP,
            "jam_at_m": H.JAM_AT_M,
            "fix_sigma_m": H.FIX_SIGMA,
            "drift_rate": H.DRIFT_RATE,
            "keep": H.KEEP,
            "min_score": H.MIN_SCORE,
            "calibration_zooms": [float(value) for value in H.CALIB_ZOOMS],
            "calibration_angles_deg": [float(value) for value in H.CALIB_ANGLES],
            "nearest_reference_count": H.CALIB_NEAREST,
            "fix_spacings": [config.name for config in CONFIGS],
        },
        "quad_matcher": {
            "library": "experiments/r_map_benchmark.py B.zncc",
            "query_shape_gray_uint8": [B.QUERY, B.QUERY],
            "preprocessing": "Resize by current held-out harness zoom, rotate by the section's pre-jam calibrated angle, then center-crop 160x160; search windows are centered on the propagated prior at 0.60 m/px.",
            "angles_deg_after_derotation": list(QUAD_ANGLES),
            "scales": list(ZOOM_SCALES),
            "acceptance": "Finite fix and quad_n >= 3, then retain the harness innovation gate and configured minimum-score gate.",
            "rejected_fix_policy": "Pure gating: no legacy matcher/fallback fix replaces a rejected ZNCC match.",
        },
        "truth_isolation": {
            "checked": True,
            "method": "The quad_fix function receives only a query frame, one reference window and validity mask, propagated prior, zoom, and pre-jam heading; it has no truth argument. Candidate selection and window placement use the propagated prior. Post-jam truth is accessed only to score attempted accepted fixes and compute the path-error series/section summaries.",
            "initialization_and_calibration": "The unchanged ALTO harness initializes each jammed section at its last GNSS position and learns zoom, heading, offset, and the flow map using the pre-jam GNSS phase; these are not post-jam truth inputs.",
            "filter_decisions": "After the GNSS cut, state updates depend only on optical-flow propagation, prior-centered reference search, fix score/quad_n, the innovation gate, and the existing filter variance/gain rules.",
        },
        "section_results_and_aggregates": aggregate,
        "outputs": [str(sections_path), str(summary_path)],
    }
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print("Baseline vs quad-gated summary (section-median error median [min, max]):", flush=True)
    for row in aggregate:
        print(
            f"  {row['policy']:10s} {row['spacing']:18s} "
            f"{row['section_median_error_m_median']:.1f} "
            f"[{row['section_median_error_m_min']:.1f}, {row['section_median_error_m_max']:.1f}] m; "
            f"accepted={row['fixes_accepted']}/{row['fixes_attempted']}; "
            f"wrong>50m={row['accepted_wrong_gt50_m']}",
            flush=True,
        )
    print(f"Wrote {sections_path} and {summary_path}", flush=True)


if __name__ == "__main__":
    main()

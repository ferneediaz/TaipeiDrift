"""Benchmark the current estimators on every Mid-Air trajectory currently on disk.

    python vio/scripts/run_midair_benchmark.py --data-root data/MidAir --gnss-cutoff 5.0 --resume

Methods: imu_only (baseline), visual (forward-camera complementary attitude correction, frozen
settings), ground_truth_attitude_oracle (NOT an estimator). Downloads may still be running: the
dataset is scanned once at the start, the valid list is frozen for this run, incomplete files are
skipped with a reason, and nothing in the dataset folder is written. Rerun later to include newly
completed trajectories; with --resume, finished trajectories whose settings and data files are
unchanged are not recomputed.

Outputs (outputs/midair_benchmark/): discovery_report.json, trajectory_metrics.csv,
visual_statistics.csv, benchmark_summary.json, plots/, trajectories/<name>/metrics.json.
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import sys
import time
from pathlib import Path

import yaml

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))

from vio.benchmark import aggregate as agg  # noqa: E402
from vio.benchmark.discovery import discover  # noqa: E402
from vio.benchmark.runner import fingerprint, run_trajectory  # noqa: E402

CSV_COLUMNS = ["environment", "condition", "trajectory", "duration_s", "estimator", "gnss_cutoff_s",
               "position_error_10s", "position_error_30s", "position_error_60s", "position_error_final",
               "position_error_mean", "position_error_median", "position_rmse", "position_error_max",
               "attitude_error_10s_deg", "attitude_error_30s_deg", "attitude_error_60s_deg", "attitude_error_final_deg",
               "attitude_error_mean_deg", "attitude_error_median_deg", "attitude_error_max_deg",
               "visual_valid_fraction", "visual_accepted_fraction", "mean_tracks", "mean_inliers", "mean_inlier_ratio",
               "status"]
VISUAL_COLUMNS = ["environment", "condition", "trajectory", "frames_processed", "attempted_updates", "valid_measurements",
                  "accepted_updates", "valid_fraction", "accepted_fraction", "rejected_updates", "visual_failures",
                  "tracks_mean", "tracks_median", "inliers_mean", "inliers_median", "inlier_ratio_mean",
                  "inlier_ratio_median", "keyframe_resets_track_loss", "keyframes_by_age", "invalid_reasons",
                  "rejection_reasons"]


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=VIO_DIR / "configs" / "midair_benchmark.yaml")
    p.add_argument("--data-root", default="data/MidAir")
    p.add_argument("--gnss-cutoff", type=float)
    p.add_argument("--environments", nargs="+")
    p.add_argument("--conditions", nargs="+")
    p.add_argument("--trajectory", nargs="+", dest="trajectories", help="e.g. 3 trajectory_0003 3002")
    p.add_argument("--camera", help="override the camera (default from config: left)")
    p.add_argument("--max-trajectories", type=int, help="run only the first N valid trajectories")
    p.add_argument("--no-visual", action="store_true", help="IMU-only and oracle only (fast)")
    p.add_argument("--visual-subset", nargs="+", metavar="COND:ID",
                   help="run the visual estimator only on these, e.g. sunny:8 cloudy:3002 (IMU and oracle still run on all)")
    p.add_argument("--output-dir", type=Path)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--resume", action="store_true", help="reuse finished trajectories with matching settings and data")
    g.add_argument("--overwrite", action="store_true", help="recompute every trajectory")
    return p.parse_args(argv)


def fmt(x) -> str:
    """Deterministic CSV formatting: 6 significant decimals, empty for missing."""
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return ""
    if isinstance(x, float):
        return f"{x:.6f}"
    return str(x)


def csv_rows(results: list[dict]) -> tuple[list[list[str]], list[list[str]]]:
    rows, vrows = [], []
    for r in results:
        vs = r.get("visual_statistics") or {}
        for est in ("imu_only", "visual", "ground_truth_attitude_oracle"):
            m = r["methods"].get(est)
            base = [r["environment"], r["condition"], r["trajectory"], r["duration_s"], est, r["gnss_cutoff_s"]]
            if not m:
                rows.append([fmt(x) for x in base] + [""] * (len(CSV_COLUMNS) - len(base) - 1)
                            + [f"unavailable: {r.get('visual_unavailable_reason', '')}"])
                continue
            p, a = m["position_m"], m["attitude_deg"]
            vis = [vs.get("valid_fraction"), vs.get("accepted_fraction"), vs.get("tracks_mean"), vs.get("inliers_mean"),
                   vs.get("inlier_ratio_mean")] if est == "visual" else [None] * 5
            rows.append([fmt(x) for x in base + [p["10s"], p["30s"], p["60s"], p["final"], p["mean"], p["median"], p["rmse"], p["max"],
                                                 a["10s"], a["30s"], a["60s"], a["final"], a["mean"], a["median"], a["max"]] + vis] + ["ok"])
        if vs:
            vrows.append([fmt(r[k]) for k in ("environment", "condition", "trajectory")]
                         + [fmt(vs[k]) if not isinstance(vs[k], dict) else json.dumps(vs[k], sort_keys=True) for k in VISUAL_COLUMNS[3:]])
    return rows, vrows


def write_csv(path: Path, header: list[str], rows: list[list[str]]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(header)
        w.writerows(rows)


def summarize(results: list[dict], cfg: dict, horizons: list[str]) -> dict:
    return {
        "n_trajectories": len(results),
        "n_with_visual": sum(1 for r in results if r["methods"].get("visual")),
        "conditions": sorted({f"{r['environment']}/{r['condition']}" for r in results}),
        "distributions": agg.distributions(results, horizons),
        "paired_visual_minus_imu": agg.paired(results, horizons, cfg["paired_tolerance"], cfg["percent_min_denominator"]),
        "drift_strata": agg.drift_strata(results, cfg["drift_strata"], cfg["paired_tolerance"]),
        "oracle_gap": agg.oracle_gap(results, cfg["percent_min_denominator"]["position_m"]),
        "visual_correlations": agg.visual_validity_correlation(results),
        "visual_statistics": {k: agg.describe([r["visual_statistics"][k] for r in results if r.get("visual_statistics")])
                              for k in ("valid_fraction", "accepted_fraction", "tracks_mean", "inliers_mean", "inlier_ratio_mean",
                                        "keyframe_resets_track_loss")},
    }


def _clean(x):
    """JSON-safe: NaN/inf -> None, for strict JSON readers."""
    if isinstance(x, float) and not math.isfinite(x):
        return None
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, list):
        return [_clean(v) for v in x]
    return x


def run_benchmark(args: argparse.Namespace, make_plots: bool = True) -> dict:
    cfg = yaml.safe_load(args.config.read_text())
    vio_cfg = yaml.safe_load((REPO_ROOT / cfg["vio_config"]).read_text())
    base_cfg = yaml.safe_load((REPO_ROOT / cfg["baseline_config"]).read_text())
    cutoff = args.gnss_cutoff if args.gnss_cutoff is not None else cfg["gnss_cutoff_s"]
    camera = args.camera or cfg["camera"]
    horizons = [f"{h:g}s" for h in cfg["horizons_s"]] + ["final"]
    out = Path(args.output_dir or REPO_ROOT / cfg["output_dir"])
    out.mkdir(parents=True, exist_ok=True)
    stream = vio_cfg["cameras"][camera]["stream"]

    # 1. discover once and freeze the list for this run
    data_root = Path(args.data_root)
    if not data_root.is_absolute():
        data_root = REPO_ROOT / data_root
    report = discover(data_root, out / "cache" / "sensor_records", stream, args.environments, args.conditions,
                      args.trajectories, base_cfg["midair"]["imu_rate_hz"], cfg["discovery"]["min_duration_s"],
                      cfg["discovery"]["min_frames"])
    if args.no_visual:
        for e in report.entries:
            if e.visual_valid:
                e.visual_valid, e.visual_reason = False, "visual disabled (--no-visual)"
    subset, excluded = None, set()
    if args.visual_subset:
        subset = {(c, f"trajectory_{int(i):04d}") for c, i in (x.split(":") for x in args.visual_subset)}
        for e in report.entries:
            if e.visual_valid and (e.condition, e.trajectory) not in subset:
                e.visual_valid, e.visual_reason = False, "not in the visual subset of this run (--visual-subset)"
                excluded.add(e.key)
    (out / "discovery_report.json").write_text(json.dumps(report.to_json(), indent=2, sort_keys=True))
    c = report.counts()
    print(f"discovered {c['discovered']} | IMU-valid {c['imu_valid']} | visual-valid {c['visual_valid']} | "
          f"IMU-invalid {c['imu_invalid']} | visual unavailable {c['visual_unavailable']} | unusable conditions {c['conditions_unusable']}")
    for e in report.entries:
        if not e.imu_valid:
            print(f"  skip {e.key}: {e.imu_reason}")
    for ci in report.condition_issues:
        print(f"  skip condition {ci['environment']}/{ci['condition']}: {ci['reason']}")

    todo = [e for e in report.entries if e.imu_valid]
    if args.max_trajectories:
        todo = todo[: args.max_trajectories]

    # 2. run the frozen list
    results, reused = [], 0
    bench_fp_cfg = {k: cfg[k] for k in ("horizons_s", "discovery")}
    for i, e in enumerate(todo, 1):
        tdir = out / "trajectories" / e.name
        mpath = tdir / "metrics.json"
        fp = fingerprint(e, bench_fp_cfg, vio_cfg, camera, cutoff)
        accept = {fp}
        if e.key in excluded:  # a stored result that includes the visual run is better: keep it
            full = copy.copy(e)
            full.visual_valid = True
            accept.add(fingerprint(full, bench_fp_cfg, vio_cfg, camera, cutoff))
        if args.resume and mpath.is_file():
            try:
                stored = json.loads(mpath.read_text())
                if stored.get("fingerprint") in accept:
                    results.append(stored)
                    reused += 1
                    continue
            except (OSError, json.JSONDecodeError):
                pass
        t = time.time()
        try:
            r = run_trajectory(e, base_cfg["midair"], vio_cfg, camera, cutoff, cfg["horizons_s"])
        except Exception as ex:  # a broken file must not stop the benchmark
            print(f"[{i}/{len(todo)}] {e.key}: FAILED {type(ex).__name__}: {ex}")
            continue
        r["fingerprint"] = fp
        tdir.mkdir(parents=True, exist_ok=True)
        mpath.write_text(json.dumps(_clean(r), indent=2, sort_keys=True))
        results.append(json.loads(mpath.read_text()))
        m = r["methods"]
        vtxt = (f"visual {m['visual']['position_m']['final']:7.1f} m {m['visual']['attitude_deg']['final']:5.2f} deg"
                if m.get("visual") else "visual n/a")
        print(f"[{i}/{len(todo)}] {e.key}: imu {m['imu_only']['position_m']['final']:7.1f} m "
              f"{m['imu_only']['attitude_deg']['final']:5.2f} deg | {vtxt} | oracle "
              f"{m['ground_truth_attitude_oracle']['position_m']['final']:6.1f} m ({time.time() - t:.0f} s)", flush=True)
    if reused:
        print(f"reused {reused} finished trajectories (--resume)")

    # 3. aggregate and write
    results.sort(key=lambda r: r["key"])
    rows, vrows = csv_rows(results)
    write_csv(out / "trajectory_metrics.csv", CSV_COLUMNS, rows)
    write_csv(out / "visual_statistics.csv", VISUAL_COLUMNS, vrows)
    summary = summarize(results, cfg, horizons)
    summary.update({"gnss_cutoff_s": cutoff, "camera_stream": stream, "discovery_counts": c,
                    "visual_subset": sorted(f"{a}:{b}" for a, b in subset) if subset else None,
                    "frozen_settings": {k: vio_cfg[k] for k in ("tracker", "pose", "fusion")}})
    (out / "benchmark_summary.json").write_text(json.dumps(_clean(summary), indent=2, sort_keys=True))
    if make_plots and results:
        from vio.benchmark.plots import make_all
        make_all(results, horizons, out / "plots")
    print(f"results in {out}")
    return summary


def main(argv=None) -> int:
    run_benchmark(parse_args(argv))
    return 0


if __name__ == "__main__":
    sys.exit(main())

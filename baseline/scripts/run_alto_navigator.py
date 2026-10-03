"""Run the camera navigator: GNSS for the first stretch, then the camera alone.

From the repository root:

    python baseline/scripts/run_alto_navigator.py                    # ALTO validation section
    python baseline/scripts/run_alto_navigator.py --section Train    # the section the settings were not tuned on
    python baseline/scripts/run_alto_navigator.py --synthetic        # generated flight, no download needed
    python baseline/scripts/run_alto_navigator.py --only camera_only every_300

Results go to outputs/alto_navigator/<run_name>/: metrics.json, errors.csv, fixes.csv and navigator.png.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

BASELINE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = BASELINE_DIR.parent
sys.path.insert(0, str(BASELINE_DIR))

from src.data.alto import AltoConfig, AltoDataNotFound, load_alto_flight  # noqa: E402
from src.data.camera_flight import CameraFlight  # noqa: E402
from src.data.synthetic_camera import make_synthetic_camera_flight  # noqa: E402
from src.estimation.camera_navigator import NavigatorConfig, calibrate, navigate  # noqa: E402
from src.estimation.image_motion import shifts_for_flight  # noqa: E402
from src.evaluation.navigation_metrics import (  # noqa: E402
    error_at_distances,
    fix_errors,
    navigation_errors,
    summarize_navigation,
)
from src.visualization.navigator_plot import plot_navigation  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=BASELINE_DIR / "configs" / "alto_navigator.yaml")
    p.add_argument("--synthetic", action="store_true", help="use a generated flight instead of ALTO")
    p.add_argument("--section", help="ALTO section: Val or Train")
    p.add_argument("--data-root", help="folder with the ALTO zip files")
    p.add_argument("--jam-after", type=float, help="distance in metres after which GNSS is lost")
    p.add_argument("--only", nargs="+", help="names of the runs to do (default: all in the config)")
    p.add_argument("--run-name", help="output subfolder name")
    p.add_argument("--output-dir", type=Path, help="parent folder for run outputs")
    return p.parse_args()


def load_flight(args: argparse.Namespace, cfg: dict) -> CameraFlight:
    if args.synthetic:
        return make_synthetic_camera_flight(**cfg["synthetic"])
    a = dict(cfg["alto"])
    if args.section:
        a["section"] = args.section
    if args.data_root:
        a["data_root"] = args.data_root
    if not Path(a["data_root"]).is_absolute():
        a["data_root"] = str(REPO_ROOT / a["data_root"])
    return load_alto_flight(AltoConfig(**a))


def main() -> int:
    args = parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    try:
        flight = load_flight(args, cfg)
    except AltoDataNotFound as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    shared = dict(cfg["navigator"])
    if args.jam_after is not None:
        shared["jam_after_m"] = args.jam_after
    base = NavigatorConfig(**shared)
    runs = {name: dict(settings) for name, settings in cfg["runs"].items() if not args.only or name in args.only}
    if not runs:
        print(f"error: no run matches {args.only}; the config has {sorted(cfg['runs'])}", file=sys.stderr)
        return 2

    run_name = args.run_name or flight.name
    out = Path(args.output_dir or REPO_ROOT / cfg["output_dir"]) / run_name
    out.mkdir(parents=True, exist_ok=True)

    print(f"flight {flight.name}: {len(flight)} frames, {flight.travelled[-1]:.0f} m, {len(flight.reference)} reference images")
    t0 = time.time()
    shifts = shifts_for_flight(flight, REPO_ROOT / cfg["shift_cache_dir"] / f"{flight.name}_flow.npy")
    calibration = calibrate(flight, shifts, base)
    print(f"GNSS lost after {flight.travelled[calibration.jam_index]:.0f} m (frame {calibration.jam_index}). "
          f"Learned before that: zoom {calibration.zoom:.2f}, rotation {calibration.angle:.0f} deg, "
          f"fix offset north {calibration.fix_offset[0]:+.1f} m, east {calibration.fix_offset[1]:+.1f} m")

    results, metrics_runs, fix_rows = {}, {}, []
    for name, settings in runs.items():
        t_run = time.time()
        result = navigate(flight, shifts, calibration, replace(base, **settings))
        results[name] = result
        errors = navigation_errors(result, flight)
        summary = summarize_navigation(result, flight)
        at = error_at_distances(errors, cfg["error_distances_m"])
        metrics_runs[name] = {
            "settings": settings,
            "position_error_m": summary.as_dict(),
            "error_at_distance_since_jam_m": {f"{d:g}m": v for d, v in at.items()},
            "status_share": {s: float(np.mean(np.array(result.status) == s)) for s in ("TRACKING", "DEGRADED", "LOST")},
            "run_time_s": round(time.time() - t_run, 2),
        }
        for fix, error in zip(result.fixes, fix_errors(result, flight)):
            fix_rows.append([name, fix.frame, f"{fix.score:.4f}", f"{fix.position[0]:.2f}", f"{fix.position[1]:.2f}", f"{fix.zoom:.2f}",
                             f"{fix.angle:.1f}", fix.reference_index, fix.candidates, f"{fix.distance:.2f}", f"{fix.allowed:.2f}",
                             int(fix.used), fix.reason, f"{error:.2f}"])
        line = (f"{name:20s} median {summary.median:6.1f} m, 90% below {summary.p90:6.1f}, worst {summary.worst:6.1f}, "
                f"end {summary.end:6.1f}")
        if result.fixes:
            line += (f" | fixes used {summary.fixes_used}, rejected {summary.fixes_rejected}, "
                     f"used but wrong {summary.used_but_wrong}, rejected but right {summary.rejected_but_right}")
        print(line)

    metrics = {
        "run_name": run_name,
        "flight": flight.name,
        "frames": len(flight),
        "distance_m": float(flight.travelled[-1]),
        "metadata": {k: v for k, v in flight.metadata.items() if not isinstance(v, np.ndarray)},
        "shared_settings": shared,
        "calibration": {
            "jam_frame": calibration.jam_index,
            "jam_after_m": float(flight.travelled[calibration.jam_index]),
            "zoom": calibration.zoom,
            "angle_deg": calibration.angle,
            "fix_offset_north_east_m": calibration.fix_offset.tolist(),
            "motion_matrix": calibration.motion_matrix.tolist(),
        },
        "runs": metrics_runs,
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2, default=str))

    first = next(iter(results.values()))
    distance = navigation_errors(first, flight).distance_since_jam
    columns = [distance] + [navigation_errors(r, flight).error for r in results.values()] + [r.sigma for r in results.values()]
    header = ["distance_since_jam_m"] + [f"error_m_{n}" for n in results] + [f"sigma_m_{n}" for n in results]
    np.savetxt(out / "errors.csv", np.column_stack(columns), delimiter=",", header=",".join(header), comments="", fmt="%.3f")
    with open(out / "fixes.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["run", "frame", "score", "north_m", "east_m", "zoom", "angle_deg", "reference_index", "candidates",
                         "distance_to_estimate_m", "allowed_distance_m", "used", "reason", "error_m"])
        writer.writerows(fix_rows)

    drawn = {name: results[name] for name in cfg.get("plot", []) if name in results} or results
    plot_navigation(flight, drawn, out / "navigator.png",
                    title=f"{flight.name}: GNSS lost after {flight.travelled[calibration.jam_index]:.0f} m, then the camera alone")
    print(f"results in {out} ({time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

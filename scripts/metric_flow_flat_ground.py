"""Score a simulator run with Alessandro's metric flow speed (sim/nodes/metric_flow.py), and the same filter
without it on the same flight.

    python scripts/metric_flow_flat_ground.py [RUN_DIR]

RUN_DIR (default outputs/sim_runs/metric_velocity/OF_terrain_flat_1) is the folder the launch wrote
(flow_velocity.csv from sim/nodes/run_logger.py) plus compare_flow_vs_control.csv from
sim/scripts/log_two_estimators.py, whose docstring gives the commands of the run. Run folders are not in git.

It prints: how good the speed readings are against the truth; how many the filter took; the position error of
both filters after the GNSS cut; and the heading drift. The truth is used for scoring only.
"""
import argparse
import csv
import math
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
COLUMNS = ["t", "gnss_available", "est_x", "est_y", "est_z", "est_vx", "est_vy", "est_yaw", "pos_var_x", "pos_var_y",
           "gt_x", "gt_y", "gt_z", "gt_vx", "gt_vy", "gt_yaw"]


def number(row: dict, key: str) -> float:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def spread(values) -> str:
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    return f"median {np.median(a):.2f}, 90 percent below {np.percentile(a, 90):.2f} (n={len(a)})" if len(a) else "none"


def readings(run_dir: Path) -> None:
    flow = list(csv.DictReader((run_dir / "flow_velocity.csv").open()))
    scored = [r for r in flow if math.isfinite(number(r, "flow_vector_error"))]
    taken = [r for r in scored if r["update_accepted"] == "True"]
    gated = [r for r in scored if r["update_accepted"] == "False"]
    span = number(flow[-1], "timestamp") - number(flow[0], "timestamp")
    print(f"The speed readings ({len(flow)} picture pairs in {span:.0f} s: {len(flow) / span:.1f} per second of 25)")
    print("  why a pair gave no update:", dict(Counter(r["rejection_reason"] or "taken" for r in flow)))
    print("  error against the truth, m/s:", spread([number(r, "flow_vector_error") for r in scored]))
    print("  speed error, m/s:           ", spread([number(r, "flow_speed_error") for r in scored]))
    print("  direction error, degrees:   ", spread([number(r, "flow_direction_error_deg") for r in scored]))
    print(f"  offered to the filter {len(taken) + len(gated)}: taken {len(taken)}, turned down by the gate {len(gated)}")
    print("    taken, error m/s:      ", spread([number(r, "flow_vector_error") for r in taken]))
    print("    turned down, error m/s:", spread([number(r, "flow_vector_error") for r in gated]))
    cruise = [r for r in scored if number(r, "timestamp") > 30]
    dt = np.array([number(r, "image_dt_s") for r in cruise])
    error = np.array([number(r, "flow_vector_error") for r in cruise])
    print("  by the time between the two pictures (after 30 s):")
    for low, high in ((0, 0.045), (0.045, 0.085), (0.085, 0.125), (0.125, 0.165), (0.165, 0.201)):
        inside = (dt > low) & (dt <= high)
        if inside.any():
            print(f"    {low:.3f} to {high:.3f} s: median {np.median(error[inside]):.2f} m/s (n={inside.sum()})")


def estimators(run_dir: Path) -> None:
    rows = list(csv.DictReader((run_dir / "compare_flow_vs_control.csv").open()))
    index = {c: i for i, c in enumerate(COLUMNS)}
    data = {name: np.array([[float(r[c]) for c in COLUMNS] for r in rows if r["name"] == name])
            for name in ("flow", "control")}
    cut = min(d[d[:, index["gnss_available"]] == 0][0, 0] for d in data.values())
    series = {}
    for name, d in data.items():
        col = lambda key, d=d: d[:, index[key]]  # noqa: E731
        yaw = col("est_yaw") - col("gt_yaw")
        series[name] = {
            "t": col("t"),
            "position": np.hypot(col("est_x") - col("gt_x"), col("est_y") - col("gt_y")),
            "velocity": np.hypot(col("est_vx") - col("gt_vx"), col("est_vy") - col("gt_vy")),
            "yaw": np.degrees(np.arctan2(np.sin(yaw), np.cos(yaw))),
            "sigma": np.sqrt(np.maximum(col("pos_var_x") + col("pos_var_y"), 0.0)),
        }
    end = series["flow"]["t"][-1]
    truth = data["flow"][data["flow"][:, 0] > cut + 10]
    centre = truth[:, [index["gt_x"], index["gt_y"]]].mean(axis=0)
    radius = np.median(np.hypot(truth[:, index["gt_x"]] - centre[0], truth[:, index["gt_y"]] - centre[1]))
    speed = np.median(np.hypot(truth[:, index["gt_vx"]], truth[:, index["gt_vy"]]))
    print(f"\nBoth filters on the same flight: GNSS lost at {cut:.1f} s, {end - cut:.0f} s without it; the drone "
          f"circles with a radius of {radius:.0f} m at {np.median(truth[:, index['gt_z']]):.0f} m and {speed:.1f} m/s "
          f"({speed * (end - cut):.0f} m flown)")
    print("  horizontal position error, m      with the flow   without   | stated sigma with the flow")
    for after in (0, 5, 10, 20, 25, 30, 45, 60, 90, 120, 150):
        at = {name: int(np.argmin(np.abs(s["t"] - (cut + after)))) for name, s in series.items()}
        print(f"    {after:4d} s after the loss          {series['flow']['position'][at['flow']]:8.1f}  "
              f"{series['control']['position'][at['control']]:8.1f}   | {series['flow']['sigma'][at['flow']]:6.1f}")
    for name, label in (("flow", "with the flow"), ("control", "without")):
        s = series[name]
        lost = s["t"] >= cut
        drift = np.polyfit(s["t"][lost], np.unwrap(np.radians(s["yaw"][lost])), 1)[0]
        print(f"  {label}: median {np.median(s['position'][lost]):.1f} m, worst {s['position'][lost].max():.1f} m, "
              f"at the end {s['position'][-1]:.1f} m; velocity error median {np.median(s['velocity'][lost]):.2f} m/s; "
              f"heading error {s['yaw'][lost][0]:.1f} degrees at the loss, {s['yaw'][-1]:.1f} at the end "
              f"({math.degrees(drift) * 60:.1f} per minute)")

    flow = list(csv.DictReader((run_dir / "flow_velocity.csv").open()))
    offered = [(number(r, "timestamp"), r["update_accepted"] == "True") for r in flow if r["update_attempted"] == "True"]
    after_cut = [(t, ok) for t, ok in offered if t >= cut]
    first_taken = next((t for t, ok in after_cut if ok), None)
    if first_taken is not None:
        refused = sum(1 for t, ok in after_cut if t < first_taken)
        s = series["flow"]
        early = (s["t"] >= cut) & (s["t"] <= first_taken + 5)
        peak = int(np.argmax(s["position"][early]))
        later = s["position"][np.argmin(np.abs(s["t"] - (first_taken + 1)))]
        print(f"  after the loss the filter turned down the first {refused} readings; its error peaked at "
              f"{s['position'][early][peak]:.0f} m, {s['t'][early][peak] - cut:.0f} s after the loss; the first reading "
              f"it took came {first_taken - cut:.0f} s after the loss, and 1 s later the error was {later:.0f} m")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", nargs="?", type=Path, default=ROOT / "outputs/sim_runs/metric_velocity/OF_terrain_flat_1")
    args = ap.parse_args()
    readings(args.run_dir)
    estimators(args.run_dir)


if __name__ == "__main__":
    main()

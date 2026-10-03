"""Summarize a metric-flow simulator run; all GT reads are evaluation-only."""
import argparse
import bisect
import csv
import json
import math
from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[2]
R_BC_DOWN = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def rows(path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def f(row, key):
    try:
        v = float(row[key])
        return v if math.isfinite(v) else math.nan
    except (KeyError, TypeError, ValueError):
        return math.nan


def stats(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    return {"n": int(len(a)), "median": float(np.median(a)), "p90": float(np.percentile(a, 90)),
            "mean": float(np.mean(a))} if len(a) else {"n": 0, "median": None, "p90": None, "mean": None}


def city_boxes():
    path = ROOT / "sim" / "models" / "city" / "model.sdf"
    boxes = []
    for link in ET.parse(path).findall(".//link"):
        pose = np.fromstring(link.findtext("pose", "0 0 0 0 0 0"), sep=" ")
        if len(pose) < 6:
            continue
        for box in link.findall("./collision/geometry/box"):
            size = np.fromstring(box.findtext("size", ""), sep=" ")
            if len(size) == 3:
                boxes.append((pose[:3] - size / 2, pose[:3] + size / 2))
    return boxes


def ray_box(origin, direction, boxes):
    nearest = math.inf
    for lo, hi in boxes:
        t0, t1 = -math.inf, math.inf
        possible = True
        for axis in range(3):
            if abs(direction[axis]) < 1e-10:
                if origin[axis] < lo[axis] or origin[axis] > hi[axis]:
                    possible = False
                    break
                continue
            a, b = (lo[axis] - origin[axis]) / direction[axis], (hi[axis] - origin[axis]) / direction[axis]
            t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
            if t1 < t0:
                possible = False
                break
        hit = t0 if t0 > 0 else t1
        if possible and hit > 0:
            nearest = min(nearest, hit)
    return nearest if math.isfinite(nearest) else math.nan


def add_range_truth(run_dir, range_rows, traj_rows):
    t = np.asarray([f(r, "timestamp_sim_s") for r in traj_rows])
    p = np.asarray([[f(r, f"gt_{axis}") for axis in "xyz"] for r in traj_rows])
    rpy = np.asarray([[f(r, f"gt_{axis}") for axis in ("roll", "pitch", "yaw")] for r in traj_rows])
    valid = np.isfinite(t) & np.all(np.isfinite(p), axis=1) & np.all(np.isfinite(rpy), axis=1)
    t, p, rpy = t[valid], p[valid], rpy[valid]
    rotations = Rotation.from_euler("xyz", rpy)
    slerp = Slerp(t, rotations)
    boxes = city_boxes()
    out = run_dir / "range_validation.csv"
    with out.open("w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(["timestamp", "measured_range_m", "gt_ray_expected_range_m_evaluation_only",
                    "range_error_m_evaluation_only", "gt_x", "gt_y", "gt_z", "valid"])
        for row in range_rows:
            stamp, measured = f(row, "timestamp"), f(row, "range_m")
            if not np.isfinite(stamp) or not t[0] <= stamp <= t[-1]:
                continue
            j = int(np.clip(np.searchsorted(t, stamp), 1, len(t) - 1))
            u = (stamp - t[j - 1]) / max(t[j] - t[j - 1], 1e-12)
            pos = (1 - u) * p[j - 1] + u * p[j]
            R = slerp([stamp])[0]
            origin = pos + R.apply([0.5, 0, 0])
            direction = R.apply(R_BC_DOWN[:, 2])
            expected = ray_box(origin, direction, boxes)
            err = measured - expected if np.isfinite(measured) and np.isfinite(expected) else math.nan
            w.writerow([stamp, measured, expected, err, *pos.tolist(), int(np.isfinite(measured))])
    return rows(out)


def summarize(run_dir, cutoff_s=20.0):
    run_dir = Path(run_dir)
    trajectory = rows(run_dir / "trajectory.csv")
    ranges = rows(run_dir / "range_debug.csv") if (run_dir / "range_debug.csv").exists() else []
    if ranges and trajectory and not (run_dir / "range_validation.csv").exists():
        range_eval = add_range_truth(run_dir, ranges, trajectory)
    else:
        range_eval = rows(run_dir / "range_validation.csv") if (run_dir / "range_validation.csv").exists() else []
    flow = rows(run_dir / "flow_velocity.csv") if (run_dir / "flow_velocity.csv").exists() else []
    velocity_rows = rows(run_dir / "velocity_debug.csv") if (run_dir / "velocity_debug.csv").exists() else []
    cov_rows = rows(run_dir / "state_covariance.csv") if (run_dir / "state_covariance.csv").exists() else []
    table = []
    for row in trajectory:
        pos_gt = np.array([f(row, f"gt_{axis}") for axis in "xyz"])
        pos_est = np.array([f(row, f"est_{axis}") for axis in "xyz"])
        table.append((f(row, "timestamp_sim_s"), np.linalg.norm(pos_est - pos_gt),
                      np.linalg.norm((pos_est - pos_gt)[:2])))
    table = [r for r in table if np.isfinite(r[0]) and np.isfinite(r[1])]
    vel_table = []
    for row in velocity_rows:
        gt = np.array([f(row, f"gt_v{axis}_enu") for axis in "xyz"])
        est = np.array([f(row, f"est_v{axis}_enu") for axis in "xyz"])
        if np.all(np.isfinite(gt)) and np.all(np.isfinite(est)):
            vel_table.append((f(row, "timestamp_sim_s"), np.linalg.norm(est - gt),
                              np.linalg.norm((est - gt)[:2]),
                              abs(np.linalg.norm(est[:2]) - np.linalg.norm(gt[:2]))))
    metadata_path = run_dir / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.exists() else {}
    configured_cutoff = f(metadata, "gnss_cutoff_s_since_first_fix")
    if np.isfinite(configured_cutoff) and configured_cutoff >= 0:
        t_first = f(trajectory[0], "timestamp_sim_s") if trajectory else 0.0
        denial = t_first + configured_cutoff
    else:
        denial = None
    horizons = {}
    for name, delta in [("cutoff", 0), ("+5s", 5), ("+10s", 10), ("+20s", 20), ("+30s", 30), ("final", None)]:
        target = table[-1][0] if delta is None else (denial + delta if denial is not None else math.nan)
        if not np.isfinite(target):
            horizons[name] = None
            continue
        nearest = min(table, key=lambda r: abs(r[0] - target)) if table else None
        nearest_v = min(vel_table, key=lambda r: abs(r[0] - target)) if vel_table else None
        horizons[name] = ({"t": nearest[0], "position_error_m": nearest[1],
                           "horizontal_position_error_m": nearest[2],
                           "velocity_vector_error_mps": nearest_v[1] if nearest_v else None,
                           "horizontal_velocity_vector_error_mps": nearest_v[2] if nearest_v else None,
                           "horizontal_speed_error_mps": nearest_v[3] if nearest_v else None} if nearest else None)
    finite_flow = [r for r in flow if np.isfinite(f(r, "flow_vector_error"))]
    accepted_flow = [r for r in finite_flow if r.get("update_accepted") == "True"]
    rejected_flow = [r for r in finite_flow if r.get("update_accepted") == "False"]
    range_error = [f(r, "range_error_m_evaluation_only") for r in range_eval]
    result = {
        "run_dir": str(run_dir), "duration_s": table[-1][0] if table else None,
        "gnss_denial_detected_s": denial,
        "position_error_m": {"final": table[-1][1], "max": max(r[1] for r in table)} if table else None,
        "velocity_error": {"vector_mps": stats([r[1] for r in vel_table]),
                           "horizontal_vector_mps": stats([r[2] for r in vel_table]),
                           "horizontal_speed_mps": stats([r[3] for r in vel_table])},
        "range": {"valid_scans": sum(f(r, "valid") == 1 for r in ranges),
                  "evaluation_signed_error_m": stats(range_error),
                  "evaluation_absolute_error_m": stats(np.abs(range_error)),
                  "local_surface_dynamic_range_m": (max(f(r, "range_m") for r in ranges if np.isfinite(f(r, "range_m"))) -
                                                     min(f(r, "range_m") for r in ranges if np.isfinite(f(r, "range_m")))) if ranges else None},
        "flow": {"estimates": len(flow), "valid_gt_scored": len(finite_flow),
                 "update_attempted": sum(r.get("update_attempted") == "True" for r in flow),
                 "accepted": sum(r.get("update_accepted") == "True" for r in flow),
                 "rejected": sum(r.get("update_accepted") == "False" for r in flow),
                 "correlated_frame_skips": sum("decimated" in r.get("rejection_reason", "") for r in flow),
                 "accepted_error_mps": stats([f(r, "flow_vector_error") for r in accepted_flow]),
                 "rejected_error_mps": stats([f(r, "flow_vector_error") for r in rejected_flow]),
                 "rejection_reasons": dict(Counter(r.get("rejection_reason", "") for r in flow)),
                 "vector_error_mps": stats([f(r, "flow_vector_error") for r in finite_flow]),
                 "speed_error_mps": stats([f(r, "flow_speed_error") for r in finite_flow]),
                 "direction_error_deg": stats([f(r, "flow_direction_error_deg") for r in finite_flow])},
        "at_horizons": horizons,
        "state_covariance_diagonal": {key: stats([f(r, key) for r in cov_rows]) for key in
            ("position_var_x", "position_var_y", "position_var_z", "velocity_var_x", "velocity_var_y", "velocity_var_z")},
    }
    (run_dir / "metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--cutoff-s", type=float, default=20.0)
    args = ap.parse_args()
    print(json.dumps(summarize(args.run_dir, args.cutoff_s), indent=2))


if __name__ == "__main__":
    main()

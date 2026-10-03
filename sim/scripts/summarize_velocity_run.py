"""Summarize saved velocity-phase CSVs and write metrics.json beside a run."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def number(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else math.nan
    except (KeyError, TypeError, ValueError):
        return math.nan


def vector_error(est, truth):
    return float(np.linalg.norm(np.asarray(est) - np.asarray(truth)))


def direction_error(est, truth):
    est, truth = np.asarray(est), np.asarray(truth)
    ne, ng = float(np.linalg.norm(est)), float(np.linalg.norm(truth))
    if ne < 0.1 or ng < 0.1:
        return math.nan
    return math.degrees(math.acos(float(np.clip(est @ truth / (ne * ng), -1.0, 1.0))))


def percentile(values, p):
    values = [float(x) for x in values if math.isfinite(float(x))]
    return float(np.percentile(values, p)) if values else None


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def summarize(run_dir, legacy_body_velocity=False, cutoff_override=None):
    run_dir = Path(run_dir)
    metadata = json.loads((run_dir / "metadata.json").read_text(encoding="utf-8"))
    trajectory = list(csv.DictReader((run_dir / "trajectory.csv").open(encoding="utf-8", newline="")))
    positions = []
    for row in trajectory:
        gt = np.array([number(row, f"gt_{a}") for a in "xyz"])
        est = np.array([number(row, f"est_{a}") for a in "xyz"])
        if np.all(np.isfinite(gt)) and np.all(np.isfinite(est)):
            delta = est - gt
            positions.append({"t": number(row, "timestamp_sim_s"), "position_3d": float(np.linalg.norm(delta)),
                              "position_horizontal": float(np.linalg.norm(delta[:2])),
                              "position_vertical": abs(float(delta[2])), "gnss": number(row, "gnss_available")})

    velocity_path = run_dir / "velocity_debug.csv"
    velocity_rows = (list(csv.DictReader(velocity_path.open(encoding="utf-8", newline="")))
                     if velocity_path.exists() else [])
    if not velocity_rows:
        for row in trajectory:
            q = [number(row, f"gt_{x}") for x in ("qx", "qy", "qz", "qw")]
            if not all(math.isfinite(x) for x in q):
                q = Rotation.from_euler("xyz", [number(row, f"gt_{x}") for x in ("roll", "pitch", "yaw")]).as_quat()
            gt_body = np.array([number(row, f"gt_v{x}") for x in "xyz"])
            gt = Rotation.from_quat(q).apply(gt_body) if legacy_body_velocity else gt_body
            est = np.array([number(row, f"est_v{x}") for x in "xyz"])
            delta = est - gt
            speed_gt, speed_est = float(np.linalg.norm(gt)), float(np.linalg.norm(est))
            velocity_rows.append({"timestamp_sim_s": number(row, "timestamp_sim_s"),
                "gnss_available": number(row, "gnss_available"),
                "speed_error_est": abs(speed_est - speed_gt),
                "direction_error_est_deg": direction_error(est, gt),
                "vector_error_est": float(np.linalg.norm(delta)),
                "horizontal_velocity_error_est": float(np.linalg.norm(delta[:2])),
                "gt_vx_enu": gt[0], "gt_vy_enu": gt[1], "gt_vz_enu": gt[2],
                "est_vx_enu": est[0], "est_vy_enu": est[1], "est_vz_enu": est[2]})

    previous_endpoint = None
    gnss_debug_path = run_dir / "gnss_debug.csv"
    if legacy_body_velocity and gnss_debug_path.exists():
        endpoint_rows = []
        for row in csv.DictReader(gnss_debug_path.open(encoding="utf-8", newline="")):
            try:
                obs = json.loads(row["velocity_observation_enu_mps"])
            except (KeyError, TypeError, json.JSONDecodeError):
                continue
            if obs is None:
                continue
            t = number(row, "stamp_s")
            gt_row = min(trajectory, key=lambda x: abs(number(x, "timestamp_sim_s") - t))
            gt_body = np.array([number(gt_row, f"gt_v{a}") for a in "xyz"])
            gt_rot = Rotation.from_euler("xyz", [number(gt_row, f"gt_{a}") for a in ("roll", "pitch", "yaw")])
            gt_v = gt_rot.apply(gt_body)
            fit_v = np.asarray(obs, dtype=float)
            endpoint_rows.append({"stamp_s": t, "vector_error_mps": vector_error(fit_v, gt_v),
                "speed_error_mps": abs(float(np.linalg.norm(fit_v)) - float(np.linalg.norm(gt_v))),
                "direction_error_deg": finite_or_none(direction_error(fit_v, gt_v)),
                "fit_speed_mps": float(np.linalg.norm(fit_v)),
                "gt_speed_mps": float(np.linalg.norm(gt_v)),
                "accepted": str(row.get("velocity_accepted", "")).lower() == "true"})
        previous_endpoint = {
            "measurement_count": len(endpoint_rows),
            "accepted": sum(x["accepted"] for x in endpoint_rows),
            "rejected": sum(not x["accepted"] for x in endpoint_rows),
            "vector_error_median_mps": percentile([x["vector_error_mps"] for x in endpoint_rows], 50),
            "vector_error_p90_mps": percentile([x["vector_error_mps"] for x in endpoint_rows], 90),
            "speed_error_median_mps": percentile([x["speed_error_mps"] for x in endpoint_rows], 50),
            "speed_error_p90_mps": percentile([x["speed_error_mps"] for x in endpoint_rows], 90),
            "measurements": endpoint_rows,
        }

    if cutoff_override is not None:
        cutoff = cutoff_override
    else:
        false_rows = [r for r in positions if r["gnss"] == 0]
        cutoff = false_rows[0]["t"] if false_rows else None
    position_summary = None
    velocity_milestones = {}
    if positions:
        position_summary = {"final_3d_m": positions[-1]["position_3d"],
                            "max_3d_m": max(x["position_3d"] for x in positions),
                            "final_horizontal_m": positions[-1]["position_horizontal"],
                            "final_vertical_m": positions[-1]["position_vertical"]}
    if cutoff is not None:
        for label, target in [("cutoff", cutoff), ("plus_5_s", cutoff + 5), ("plus_10_s", cutoff + 10),
                              ("plus_20_s", cutoff + 20), ("plus_30_s", cutoff + 30), ("final", positions[-1]["t"] if positions else cutoff)]:
            p = min(positions, key=lambda x: abs(x["t"] - target)) if positions else None
            v = min(velocity_rows, key=lambda x: abs(number(x, "timestamp_sim_s") - target)) if velocity_rows else None
            if p and (label == "final" or abs(p["t"] - target) <= 1.0):
                direction_deg = number(v, "direction_error_est_deg") if v else math.nan
                gt_speed = float(np.linalg.norm([number(v, f"gt_v{a}_enu") for a in "xyz"])) if v else math.nan
                direction_component = (2.0 * gt_speed * math.sin(math.radians(direction_deg) / 2.0)
                                       if math.isfinite(direction_deg) and math.isfinite(gt_speed) else math.nan)
                velocity_milestones[label] = {
                    "timestamp_sim_s": p["t"], "position_3d_m": p["position_3d"],
                    "position_horizontal_m": p["position_horizontal"], "position_vertical_m": p["position_vertical"],
                    "velocity_vector_error_mps": finite_or_none(number(v, "vector_error_est")) if v else None,
                    "horizontal_velocity_error_mps": finite_or_none(number(v, "horizontal_velocity_error_est")) if v else None,
                    "speed_error_mps": finite_or_none(number(v, "speed_error_est")) if v else None,
                    "direction_error_deg": finite_or_none(direction_deg),
                    "direction_only_equivalent_error_mps": finite_or_none(direction_component),
                    "gt_speed_mps": finite_or_none(gt_speed),
                    "est_speed_mps": float(np.linalg.norm([number(v, f"est_v{a}_enu") for a in "xyz"])) if v else None,
                }

    fit_rows = [r for r in velocity_rows if number(r, "fit_event") == 1]
    accepted = sum(str(r.get("fit_accepted", "")).lower() == "true" for r in fit_rows)
    rejected = sum(str(r.get("fit_accepted", "")).lower() == "false" for r in fit_rows)
    fit_metrics = {
        "measurement_count": len(fit_rows), "accepted": accepted, "rejected": rejected,
        "vector_error_median_mps": percentile([number(r, "fit_vector_error") for r in fit_rows], 50),
        "vector_error_p90_mps": percentile([number(r, "fit_vector_error") for r in fit_rows], 90),
        "speed_error_median_mps": percentile([number(r, "fit_speed_error") for r in fit_rows], 50),
        "speed_error_p90_mps": percentile([number(r, "fit_speed_error") for r in fit_rows], 90),
        "direction_error_median_deg": percentile([number(r, "fit_direction_error_deg") for r in fit_rows], 50),
        "direction_error_p90_deg": percentile([number(r, "fit_direction_error_deg") for r in fit_rows], 90),
        "fit_sample_count_median": percentile([number(r, "fit_sample_count") for r in fit_rows], 50),
        "fit_residual_rms_m_median": percentile([number(r, "fit_residual_rms_m") for r in fit_rows], 50),
    }
    low_speed = [r for r in fit_rows if math.hypot(number(r, "gt_vx_enu"), number(r, "gt_vy_enu")) < 0.5]
    fit_metrics["low_speed_fit_count"] = len(low_speed)
    fit_metrics["low_speed_gt_horizontal_median_mps"] = percentile(
        [math.hypot(number(r, "gt_vx_enu"), number(r, "gt_vy_enu")) for r in low_speed], 50)
    fit_metrics["low_speed_fitted_horizontal_median_mps"] = percentile(
        [math.hypot(number(r, "gnss_fit_vx_enu"), number(r, "gnss_fit_vy_enu")) for r in low_speed], 50)
    low_speed_fit_horizontal = [math.hypot(number(r, "gnss_fit_vx_enu"), number(r, "gnss_fit_vy_enu"))
                                for r in low_speed]
    fit_metrics["low_speed_fitted_horizontal_p90_mps"] = percentile(low_speed_fit_horizontal, 90)
    fit_metrics["low_speed_fitted_horizontal_max_mps"] = max(
        (x for x in low_speed_fit_horizontal if math.isfinite(x)), default=None)
    post_cutoff_rows = ([r for r in velocity_rows if cutoff is not None and
                         number(r, "timestamp_sim_s") >= cutoff and number(r, "gnss_available") == 0]
                        if cutoff is not None else [])
    speed_errors = [number(r, "speed_error_est") for r in post_cutoff_rows]
    direction_components = []
    for row in post_cutoff_rows:
        speed = float(np.linalg.norm([number(row, f"gt_v{a}_enu") for a in "xyz"]))
        angle = number(row, "direction_error_est_deg")
        direction_components.append(2.0 * speed * math.sin(math.radians(angle) / 2.0)
                                    if math.isfinite(angle) and math.isfinite(speed) else math.nan)
    comparable = [(s, d) for s, d in zip(speed_errors, direction_components)
                  if math.isfinite(s) and math.isfinite(d)]
    fit_metrics["post_cutoff_error_decomposition"] = {
        "samples": len(comparable),
        "speed_error_median_mps": percentile([x[0] for x in comparable], 50),
        "speed_error_p90_mps": percentile([x[0] for x in comparable], 90),
        "direction_only_equivalent_median_mps": percentile([x[1] for x in comparable], 50),
        "direction_only_equivalent_p90_mps": percentile([x[1] for x in comparable], 90),
        "speed_error_larger_fraction": (sum(s > d for s, d in comparable) / len(comparable)
                                         if comparable else None),
    }
    return {"run_id": metadata.get("run_id"), "cutoff_sim_time_s": cutoff,
            "position_error": position_summary, "velocity_milestones": velocity_milestones,
            "gnss_velocity_fit": fit_metrics, "previous_endpoint_velocity": previous_endpoint}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--legacy-body-velocity", action="store_true",
                        help="rotate legacy trajectory GT twist from body into world before scoring")
    parser.add_argument("--cutoff-s", type=float, help="override cutoff time for a legacy run")
    parser.add_argument("--output", type=Path, help="write metrics here instead of modifying the run folder")
    args = parser.parse_args()
    report = summarize(args.run_dir, args.legacy_body_velocity, args.cutoff_s)
    text = json.dumps(report, indent=2, allow_nan=False)
    output = args.output or args.run_dir / "metrics.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()

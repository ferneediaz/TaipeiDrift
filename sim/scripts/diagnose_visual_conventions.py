"""Score candidate camera translation-direction conventions against GT (evaluation only)."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
from frame_conversions import gazebo_optical_to_flu


def angle_deg(a, b):
    c = np.clip(np.dot(a, b) / max(np.linalg.norm(a) * np.linalg.norm(b), 1e-12), -1.0, 1.0)
    return math.degrees(math.acos(float(c)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    args = ap.parse_args()
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    Odometry = get_message("nav_msgs/msg/Odometry")
    String = get_message("std_msgs/msg/String")
    truth, events = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == "/ground_truth/odom":
            m = deserialize_message(raw, Odometry)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p, q = m.pose.pose.position, m.pose.pose.orientation
            truth.append((t, np.array([p.x, p.y, p.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])))
        elif topic == "/nav/estimator_status":
            try:
                x = json.loads(deserialize_message(raw, String).data)
                if x.get("event") == "visual_span" and x.get("visual_translation_direction_camera") is not None:
                    events.append(x)
            except (ValueError, TypeError):
                pass
    times = np.array([x[0] for x in truth])
    positions = np.stack([x[1] for x in truth])
    rotations = Rotation.concatenate([x[2] for x in truth])

    def at(t):
        j = int(np.searchsorted(times, t))
        if j == 0 or j == len(times):
            return None
        i = j - 1
        if times[j] == times[i]:
            return positions[i], rotations[i]
        pos = positions[i] + (positions[j] - positions[i]) * ((t - times[i]) / (times[j] - times[i]))
        rot = Slerp(times[[i, j]], Rotation.concatenate([rotations[i], rotations[j]]))([t])[0]
        return pos, rot

    r_sim = gazebo_optical_to_flu()
    r_midair = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    mappings = {"sim_Rbc": r_sim, "sim_Rbc_T": r_sim.T, "midair_Rbc": r_midair,
                "midair_Rbc_T": r_midair.T, "identity": np.eye(3)}
    errors = {f"{name}{sign:+d}": [] for name in mappings for sign in (1, -1)}
    horizontal_errors = {k: [] for k in errors}
    for e in events:
        a, b = at(e["start_stamp"]), at(e["end_stamp"])
        if a is None or b is None:
            continue
        disp = b[0] - a[0]
        if np.linalg.norm(disp) < 1e-6:
            continue
        d_cam = np.asarray(e["visual_translation_direction_camera"], float)
        for name, Rbc in mappings.items():
            for sign in (1, -1):
                key = f"{name}{sign:+d}"
                d_world = a[1].apply(Rbc @ (sign * d_cam))
                err = angle_deg(d_world, disp)
                errors[key].append(err)
                if np.linalg.norm(disp[:2]) > 1.0:
                    horizontal_errors[key].append(err)
    def stats(d):
        return {k: {"n": len(v), "median_deg": float(np.median(v)),
                    "p90_deg": float(np.percentile(v, 90))} for k, v in d.items() if v}
    print(json.dumps({"valid_direction_intervals": len(events), "candidate_all_motion": stats(errors),
                      "candidate_horizontal_motion_gt_displacement_gt_1m": stats(horizontal_errors)}, indent=2))


if __name__ == "__main__":
    main()

"""Five-second IMU-only convention check; GT initializes once and scores only."""
import argparse
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "baseline"))
sys.path.insert(0, str(ROOT))
from vio.estimation.eskf import ESKF
from vio.eskf_pipeline import noise_model


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--initialize-at", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=5.0)
    args = ap.parse_args()
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    Imu = get_message("sensor_msgs/msg/Imu")
    Odom = get_message("nav_msgs/msg/Odometry")
    imus, truths = [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == "/imu/data":
            m = deserialize_message(raw, Imu)
            t = stamp(m)
            if t <= args.initialize_at + args.duration + 0.1:
                imus.append((t, np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]),
                             np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])))
        elif topic == "/ground_truth/odom":
            m = deserialize_message(raw, Odom)
            t = stamp(m)
            if t <= args.initialize_at + args.duration + 0.1:
                p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
                truths.append((t, np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]),
                               np.array([v.x, v.y, v.z])))
        if imus and truths and max(imus[-1][0], truths[-1][0]) > args.initialize_at + args.duration + 0.05:
            break
    t0 = min((t for t, _, _ in imus if t >= args.initialize_at), default=None)
    t1 = t0 + args.duration if t0 is not None else None
    if t0 is None:
        raise RuntimeError("bag does not contain requested IMU interval")
    truth_times = np.array([x[0] for x in truths])
    def truth_at(t):
        j = int(np.searchsorted(truth_times, t))
        if j >= len(truths):
            j = len(truths) - 1
        return truths[j]
    truth0, truth1 = truth_at(t0), truth_at(t1)
    window = [x for x in imus if t0 - 0.5 <= x[0] < t0]
    R0 = Rotation.from_quat(truth0[2])
    mean_a = np.mean([x[1] for x in window], axis=0)
    mean_w = np.mean([x[2] for x in window], axis=0)
    ba0 = mean_a - R0.inv().apply([0, 0, 9.80665])
    cfg = yaml.safe_load((ROOT / "vio/configs/midair_eskf.yaml").read_text(encoding="utf-8"))
    f = ESKF(truth0[1], truth0[3], [R0.as_quat()[3], *R0.as_quat()[:3]],
             [0, 0, -9.80665], "body", noise_model(cfg), ba0=ba0, bg0=mean_w)
    interval = [x for x in imus if x[0] >= t0 and x[0] <= t1]
    for (ta, aa, wa), (tb, ab, wb) in zip(interval, interval[1:]):
        f.predict(aa, ab, wa, wb, tb - ta)
    dp = f.p - truth1[1]
    dv = f.v - truth1[3]
    dr = math_degrees((Rotation.from_quat(truth1[2]).inv() * f.R).magnitude())
    print({"initialize_at_s": t0, "end_s": interval[-1][0], "duration_s": interval[-1][0] - t0,
           "initial_ba_from_imu": ba0.tolist(), "initial_bg_from_imu": mean_w.tolist(),
           "position_error_m": float(np.linalg.norm(dp)), "position_error_enu_m": dp.tolist(),
           "velocity_error_mps": float(np.linalg.norm(dv)), "attitude_error_deg": dr,
           "ground_truth_used_after_initialization": False})


def math_degrees(x):
    return float(np.rad2deg(x))


if __name__ == "__main__":
    main()

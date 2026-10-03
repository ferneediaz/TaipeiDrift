"""Score the drone's position from the ships' bearings (nodes/rf_nav.py) against ground truth, and plot it.

Run inside the container while the strait world runs, best without GNSS:
    ros2 launch sim/launch/sim.launch.py world:=strait demo:=true gps:=false
    python3 sim/scripts/check_rf_nav.py [seconds] [--out data/sim/rf_nav]

Collects for `seconds` of wall time (default 300), then prints the position and heading errors, how often the
truth lies inside the filter's own 2-sigma circle, and saves a map and an error plot as PNG.
"""
import argparse
import json
import math
import time
from pathlib import Path

import matplotlib
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class Collect(Node):
    def __init__(self):
        super().__init__("check_rf_nav")
        self.truth, self.est, self.ships, self.det = [], [], {}, []
        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, sd)
        self.create_subscription(Odometry, "/rf_nav/odom", self.on_est, 10)
        self.create_subscription(String, "/rf/detections", lambda m: self.det.append(json.loads(m.data)), 50)
        self.create_timer(2.0, self.find_ships)

    def find_ships(self):
        for name, _ in self.get_topic_names_and_types():
            if name.startswith("/ships/") and name.endswith("/odom") and name not in self.ships:
                self.ships[name] = []
                self.create_subscription(Odometry, name, lambda m, n=name: self.ships[n].append(
                    (m.pose.pose.position.x, m.pose.pose.position.y)), qos_profile_sensor_data)

    def on_truth(self, m):
        if not self.truth or stamp(m) - self.truth[-1][0] >= 0.1:  # 10 Hz is plenty
            p = m.pose.pose.position
            self.truth.append((stamp(m), p.x, p.y, yaw(m.pose.pose.orientation)))

    def on_est(self, m):
        if not self.est or stamp(m) - self.est[-1][0] >= 0.1:
            p, c = m.pose.pose.position, m.pose.covariance
            self.est.append((stamp(m), p.x, p.y, yaw(m.pose.pose.orientation), c[0], c[7], c[1], c[35]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seconds", nargs="?", type=float, default=300.0)
    ap.add_argument("--out", default=str(REPO / "data/sim/rf_nav"))
    args = ap.parse_args()
    rclpy.init()
    node = Collect()
    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    truth, est = np.array(node.truth), np.array(node.est)
    if len(est) < 10:
        raise SystemExit(f"Only {len(est)} estimates on /rf_nav/odom. Is the strait world running?")

    # truth at the estimate times
    tx = np.interp(est[:, 0], truth[:, 0], truth[:, 1])
    ty = np.interp(est[:, 0], truth[:, 0], truth[:, 2])
    tpsi = np.interp(est[:, 0], truth[:, 0], np.unwrap(truth[:, 3]))
    err = np.hypot(est[:, 1] - tx, est[:, 2] - ty)
    herr = np.degrees(np.angle(np.exp(1j * (est[:, 3] - tpsi))))
    sigma = np.sqrt(est[:, 4] + est[:, 5])
    # inside the 2-sigma ellipse: squared Mahalanobis distance < 6.18 (2 degrees of freedom, 95.4 %)
    d = np.column_stack([est[:, 1] - tx, est[:, 2] - ty])
    inside = []
    for (cxx, cyy, cxy), v in zip(est[:, [4, 5, 6]], d):
        inside.append(v @ np.linalg.solve(np.array([[cxx, cxy], [cxy, cyy]]), v) < 6.18)
    settled = est[:, 0] >= est[0, 0] + 30  # leave out the first 30 s after the first fix

    t0 = est[0, 0]
    print(f"{len(node.det)} bearings decoded, {len(est)} estimates over {est[-1, 0] - t0:.0f} s of simulation")
    print(f"first fix at sim time {t0:.1f} s, error {err[0]:.1f} m")
    for label, sel in (("all", slice(None)), ("after the first 30 s", settled)):
        e = err[sel]
        if len(e):
            print(f"  {label:>20}: position error median {np.median(e):6.1f} m, RMS {np.sqrt(np.mean(e ** 2)):6.1f} m,"
                  f" 95 % {np.percentile(e, 95):6.1f} m, max {e.max():6.1f} m;"
                  f" heading RMS {np.sqrt(np.mean(herr[sel] ** 2)):4.1f} deg;"
                  f" truth inside the 2-sigma ellipse {100 * np.mean(np.array(inside)[sel]):3.0f} % (want about 95)")

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    png = out / f"rf_nav_{time.strftime('%Y%m%d_%H%M%S')}.png"
    fig, (a, b) = plt.subplots(1, 2, figsize=(15, 6.5), gridspec_kw={"width_ratios": [1.3, 1]})
    for name, track in node.ships.items():
        if track:
            tr = np.array(track)
            a.plot(tr[:, 0], tr[:, 1], color="0.6", lw=6, alpha=0.5)
            a.annotate(name.split("/")[2], tr[-1], fontsize=8, color="0.3")
    a.plot(truth[:, 1], truth[:, 2], color="k", lw=1.5, label="drone, ground truth")
    a.plot(est[:, 1], est[:, 2], color="tab:red", lw=1, label="drone, from the ships' bearings")
    a.plot(est[0, 1], est[0, 2], "o", color="tab:red", label="first fix")
    a.set_aspect("equal")
    a.set_xlabel("east, m")
    a.set_ylabel("north, m")
    a.set_title("Position without GNSS: triangulation on three AIS ships")
    a.legend(loc="lower left", fontsize=8)
    a.grid(alpha=0.3)
    tt = est[:, 0] - t0
    b.plot(tt, err, color="tab:red", lw=1, label="position error")
    b.plot(tt, 2 * sigma, color="tab:red", lw=1, ls="--", alpha=0.6, label="2 sigma, the filter's own")
    b.set_xlabel("s since the first fix")
    b.set_ylabel("m")
    b.set_ylim(0, max(10.0, np.percentile(np.r_[err, 2 * sigma], 98) * 1.2))
    b.grid(alpha=0.3)
    b.legend(fontsize=8)
    b.set_title("Error against ground truth")
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    print(f"plot: {png.relative_to(REPO) if png.is_relative_to(REPO) else png}")


if __name__ == "__main__":
    main()

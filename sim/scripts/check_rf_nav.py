"""Score the drone's navigation in the strait world against ground truth, and plot it.

Three estimates run side by side on the same flight (sim.launch.py with the ships):
    /rf_nav/odom   the ships' bearings only (nodes/rf_nav.py): horizontal position and heading
    /nav/odom      the ESKF (nodes/eskf_ros_adapter.py): GNSS + IMU + barometer + forward camera
    /nav_rf/odom   the same ESKF, also fusing the ships' position fix (rf_fix:=true)

Run inside the container while the strait world runs:
    sim/run.sh                                  (on the host: GNSS first, then cut; see gnss_cutoff_s)
    python3 sim/scripts/check_rf_nav.py [seconds] [--out data/sim/rf_nav]

Collects for `seconds` of wall time (default 300), then prints, for each estimate, the horizontal position error,
the height error (ESKF only), the heading error, and how often the truth lies inside the estimate's own 2-sigma
ellipse, over the whole run and after the GNSS cutoff. Saves a map and an error plot as PNG.
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
from scipy.stats import chi2
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
TWO_SIGMA_2D = chi2.ppf(0.9545, 2)  # squared Mahalanobis distance of the 2-sigma ellipse in 2D (6.18)
ESTIMATES = {  # topic: (label, colour, has height)
    "/rf_nav/odom": ("ships' bearings only (rf_nav)", "tab:red", False),
    "/nav/odom": ("ESKF: GNSS, then IMU + baro + camera", "tab:blue", True),
    "/nav_rf/odom": ("ESKF + ships' fix", "tab:green", True),
}


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class Collect(Node):
    def __init__(self):
        super().__init__("check_rf_nav")
        self.truth, self.ships, self.det = [], {}, []
        self.est = {topic: [] for topic in ESTIMATES}
        self.cutoff = None  # sim time when the GNSS gate closed
        self.clock = None
        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, sd)
        for topic in ESTIMATES:
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.on_est(t, m), sd)
        self.create_subscription(String, "/rf/detections", lambda m: self.det.append(json.loads(m.data)), 50)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, "/nav/gnss_available", self.on_gnss, latched)
        self.create_timer(2.0, self.find_ships)

    def find_ships(self):
        for name, _ in self.get_topic_names_and_types():
            if name.startswith("/ships/") and name.endswith("/odom") and name not in self.ships:
                self.ships[name] = []
                self.create_subscription(Odometry, name, lambda m, n=name: self.ships[n].append(
                    (m.pose.pose.position.x, m.pose.pose.position.y)), qos_profile_sensor_data)

    def on_gnss(self, m):
        if not m.data and self.cutoff is None and self.truth:
            self.cutoff = self.truth[-1][0]

    def on_truth(self, m):
        if not self.truth or stamp(m) - self.truth[-1][0] >= 0.1:  # 10 Hz is plenty
            p = m.pose.pose.position
            self.truth.append((stamp(m), p.x, p.y, p.z, yaw(m.pose.pose.orientation)))

    def on_est(self, topic, m):
        est = self.est[topic]
        if not est or stamp(m) - est[-1][0] >= 0.1:
            p, c = m.pose.pose.position, m.pose.covariance
            est.append((stamp(m), p.x, p.y, p.z, yaw(m.pose.pose.orientation), c[0], c[7], c[1]))


def score(est, truth, cutoff):
    """Errors of one estimate at its own times, against interpolated truth."""
    t = est[:, 0]
    tx, ty, tz = (np.interp(t, truth[:, 0], truth[:, i]) for i in (1, 2, 3))
    tpsi = np.interp(t, truth[:, 0], np.unwrap(truth[:, 4]))
    err = np.hypot(est[:, 1] - tx, est[:, 2] - ty)
    zerr = est[:, 3] - tz
    herr = np.degrees(np.angle(np.exp(1j * (est[:, 4] - tpsi))))
    d = np.column_stack([est[:, 1] - tx, est[:, 2] - ty])
    inside = np.array([v @ np.linalg.solve(np.array([[cxx, cxy], [cxy, cyy]]), v) < TWO_SIGMA_2D
                       for (cxx, cyy, cxy), v in zip(est[:, [5, 6, 7]], d)])
    after = t >= cutoff if cutoff is not None else np.zeros(len(t), bool)
    return dict(t=t, err=err, zerr=zerr, herr=herr, inside=inside, after=after, sigma=np.sqrt(est[:, 5] + est[:, 6]))


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
    truth = np.array(node.truth)
    if len(truth) < 10:
        raise SystemExit("No ground truth. Is the strait world running?")
    cutoff = node.cutoff
    print(f"{len(node.det)} bearings decoded over {truth[-1, 0] - truth[0, 0]:.0f} s of simulation; "
          + (f"GNSS cut at sim time {cutoff:.1f} s" if cutoff is not None else "GNSS never cut"))

    scores = {}
    for topic, (label, _, has_z) in ESTIMATES.items():
        est = np.array(node.est[topic])
        if len(est) < 10:
            print(f"\n{label} ({topic}): only {len(est)} estimates, not scored")
            continue
        s = scores[topic] = score(est, truth, cutoff)
        print(f"\n{label} ({topic}): first estimate at sim time {s['t'][0]:.1f} s")
        for name, sel in (("whole run", slice(None)), ("after the GNSS cutoff", s["after"])):
            e = s["err"][sel]
            if not len(e):
                continue
            z = f" height RMS {np.sqrt(np.mean(s['zerr'][sel] ** 2)):5.1f} m;" if has_z else ""
            print(f"  {name:>21}: horizontal error median {np.median(e):6.1f} m, RMS {np.sqrt(np.mean(e ** 2)):6.1f} m,"
                  f" 95 % {np.percentile(e, 95):6.1f} m, max {e.max():6.1f} m, at the end {e[-1]:6.1f} m;{z}"
                  f" heading RMS {np.sqrt(np.mean(s['herr'][sel] ** 2)):4.1f} deg;"
                  f" truth inside 2 sigma {100 * np.mean(s['inside'][sel]):3.0f} % (want about 95)")
    if not scores:
        raise SystemExit("No estimates to score.")

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
    t0 = truth[0, 0]
    for topic, s in scores.items():
        label, colour, _ = ESTIMATES[topic]
        est = np.array(node.est[topic])
        a.plot(est[:, 1], est[:, 2], color=colour, lw=1, label=label)
        b.plot(s["t"] - t0, s["err"], color=colour, lw=1, label=label)
        b.plot(s["t"] - t0, 2 * s["sigma"], color=colour, lw=0.8, ls="--", alpha=0.5)
    if cutoff is not None:
        b.axvline(cutoff - t0, color="0.4", ls=":", label="GNSS cut")
    # frame the flight and the ships; an estimate that drifts far away runs off the edge instead of shrinking the map
    area = np.vstack([truth[:, 1:3], *[np.array(tr) for tr in node.ships.values() if tr]])
    lo, hi = area.min(0), area.max(0)
    pad = 0.15 * (hi - lo).max()
    a.set_xlim(lo[0] - pad, hi[0] + pad)
    a.set_ylim(lo[1] - pad, hi[1] + pad)
    a.set_aspect("equal")
    a.set_xlabel("east, m")
    a.set_ylabel("north, m")
    a.set_title("Strait world: position estimates")
    a.legend(loc="lower left", fontsize=8)
    a.grid(alpha=0.3)
    b.set_xlabel("simulation s")
    b.set_ylabel("horizontal error, m (dashed: the estimate's own 2 sigma)")
    b.set_yscale("log")  # drift grows to kilometres while the aided estimates stay at tens of metres
    allerr = np.concatenate([s["err"] for s in scores.values()])
    b.set_ylim(1.0, max(10.0, allerr.max() * 1.5))
    b.grid(alpha=0.3)
    b.legend(fontsize=8)
    b.set_title("Error against ground truth")
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    print(f"\nplot: {png.relative_to(REPO) if png.is_relative_to(REPO) else png}")


if __name__ == "__main__":
    main()

"""Log two estimators against the ground truth on the same flight (runs inside the simulator container).

    /nav/odom       the launch's estimator
    /nav_ctrl/odom  a second instance of the same estimator with one setting changed

Both read the same sensor messages, so the flight, the noise and the GNSS cut are identical: a matched comparison.
One CSV row per estimator message, with the truth nearest in time. The truth is used for scoring only; neither
estimator reads it. scripts/metric_flow_flat_ground.py scores the file.

Start the second estimator and this logger BEFORE the launch, so both estimators see the flight from its first
message (here: the launch runs with metric_flow:=true, the second instance without it; terrain world):

    cd sim
    docker compose exec -d sim bash -ic "python3 sim/nodes/eskf_ros_adapter.py --ros-args \\
        -p metric_flow:=false -p publish_tf:=false -p gps_origin_latitude:=24.064 -p gps_origin_longitude:=120.699 \\
        -p gps_origin_elevation:=60.0 -r __node:=eskf_control -r /nav/odom:=/nav_ctrl/odom \\
        -r /nav/estimator_status:=/nav_ctrl/estimator_status"
    docker compose exec -d sim bash -ic "python3 sim/scripts/log_two_estimators.py --out outputs/sim_runs/compare.csv"
    ./run.sh terrain metric_flow:=true run_label:=OF_terrain_flat_1

The GPS origin must be the world's (spherical_coordinates in sim/worlds/<world>.sdf), as the launch passes it.
"""
import argparse
import bisect
import csv

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Logger(Node):
    def __init__(self, out, topics):
        super().__init__("ctrl_logger", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.file = open(out, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["t", "name", "gnss_available", "est_x", "est_y", "est_z", "est_vx", "est_vy", "est_vz",
                              "est_yaw", "pos_var_x", "pos_var_y", "gt_t", "gt_x", "gt_y", "gt_z", "gt_vx", "gt_vy",
                              "gt_vz", "gt_yaw"])
        self.truth_t, self.truth = [], []
        self.pending = []
        self.gnss = True
        self.rows = 0
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Bool, "/nav/gnss_available", lambda m: setattr(self, "gnss", m.data), 10)
        for name, topic in topics:
            self.create_subscription(Odometry, topic, lambda m, n=name: self.on_estimate(n, m), qos_profile_sensor_data)
        self.create_timer(1.0, self.flush)

    def on_truth(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        v_world = rot.apply([v.x, v.y, v.z])  # the odometry twist is in the body frame
        self.truth_t.append(stamp(m))
        self.truth.append((p.x, p.y, p.z, *v_world, rot.as_euler("zyx")[0]))
        if len(self.truth_t) > 4000:  # 40 s at 100 Hz
            del self.truth_t[:2000], self.truth[:2000]

    def on_estimate(self, name, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("zyx")[0]
        cov = m.pose.covariance
        self.pending.append((stamp(m), name, int(self.gnss), p.x, p.y, p.z, v.x, v.y, v.z, yaw, cov[0], cov[7]))

    def flush(self):
        """Write the estimates whose time the truth has passed, each with the truth nearest in time."""
        if not self.truth_t:
            return
        keep = []
        for row in self.pending:
            t = row[0]
            if t > self.truth_t[-1]:
                keep.append(row)
                continue
            i = bisect.bisect_left(self.truth_t, t)
            j = min((k for k in (i - 1, i) if 0 <= k < len(self.truth_t)), key=lambda k: abs(self.truth_t[k] - t))
            self.writer.writerow([*row, self.truth_t[j], *self.truth[j]])
            self.rows += 1
        self.pending = keep
        self.file.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--topics", nargs="+", default=["flow=/nav/odom", "control=/nav_ctrl/odom"],
                    help="NAME=TOPIC for each estimator to log (nav_msgs/Odometry); in the strait world for example eskf=/nav/odom eskf_rf=/nav_rf/odom rf=/rf_nav/odom")
    args = ap.parse_args()
    rclpy.init()
    node = Logger(args.out, [tuple(s.split("=", 1)) for s in args.topics])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush()
        node.file.close()


if __name__ == "__main__":
    main()

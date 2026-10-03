"""Write a compact, latest-sample trajectory CSV for an experiment run."""
import argparse
import csv
import json
import math
import json
from pathlib import Path
import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, FluidPressure, Imu, NavSatFix
from std_msgs.msg import Bool, String
from scipy.spatial.transform import Rotation, Slerp
from run_policy import gps_csv_values


def stamp(msg):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


def pose_values(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
    v = msg.twist.twist.linear
    return [p.x, p.y, p.z, v.x, v.y, v.z, *rpy]


class RunLogger(Node):
    def __init__(self, out):
        super().__init__("sim_run_logger", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.file = (Path(out) / "trajectory.csv").open("w", newline="", encoding="utf-8")
        self.run_dir = Path(out)
        self.log_file = (self.run_dir / "sim.log").open("a", encoding="utf-8")
        self.debug_file = (self.run_dir / "estimator_debug.csv").open("w", newline="", encoding="utf-8")
        self.debug_writer = csv.writer(self.debug_file)
        self.debug_writer.writerow(["event", "start_stamp_s", "end_stamp_s", "frame_dt_s", "imu_state_stamp_s",
            "tracks", "inliers", "inlier_ratio", "rotation_accepted", "rotation_nis", "rotation_reason",
            "rotation_innovation_deg", "visual_relative_rotation_body", "predicted_relative_rotation_body",
            "gt_rotation_error_deg", "gt_rotation_error_if_inverse_deg", "direction_accepted", "direction_nis",
            "direction_reason", "direction_innovation_deg", "visual_direction_world", "gt_direction_error_deg"])
        self.gnss_debug_file = (self.run_dir / "gnss_debug.csv").open("w", newline="", encoding="utf-8")
        self.gnss_debug_writer = csv.writer(self.gnss_debug_file)
        self.gnss_debug_writer.writerow(["stamp_s", "estimator_state_stamp_s", "position_accepted", "position_nis",
            "position_rejection_reason", "velocity_observation_enu_mps", "velocity_covariance_enu",
            "velocity_accepted", "velocity_nis", "velocity_rejection_reason"])
        self.gt_history = []
        self.started = False
        self.rows = 0
        self.writer = csv.writer(self.file)
        self.writer.writerow(["timestamp_sim_s", "gt_x", "gt_y", "gt_z", "gt_vx", "gt_vy", "gt_vz",
                              "gt_roll", "gt_pitch", "gt_yaw", "est_x", "est_y", "est_z", "est_vx",
                              "est_vy", "est_vz", "est_roll", "est_pitch", "est_yaw", "gps_x_lat_deg",
                              "gps_y_lon_deg", "gps_z_alt_m", "gnss_available", "barometer_pa",
                              "imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz"])
        self.gt = self.est = self.gps = self.imu = self.baro = None
        self.gnss_available = False
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/nav/odom", lambda m: setattr(self, "est", m), qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/sim/gps_raw", lambda m: setattr(self, "gps", m), qos_profile_sensor_data)
        self.create_subscription(Bool, "/nav/gnss_available", lambda m: setattr(self, "gnss_available", m.data), 10)
        self.create_subscription(Imu, "/imu/data", lambda m: setattr(self, "imu", m), qos_profile_sensor_data)
        self.create_subscription(FluidPressure, "/air_pressure", lambda m: setattr(self, "baro", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/forward/camera_info", self.camera_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/down/camera_info", self.camera_info, qos_profile_sensor_data)
        self.create_subscription(String, "/nav/estimator_status", self.estimator_status, qos_profile_sensor_data)
        self.last_camera = {}
        self.create_timer(0.1, self.write_row)

    def camera_info(self, m):
        self.last_camera[m.header.frame_id] = {"width": m.width, "height": m.height, "k": list(m.k)}

    def on_truth(self, m):
        self.gt = m
        self.gt_history.append((stamp(m), m))
        newest = self.gt_history[-1][0]
        while self.gt_history and self.gt_history[0][0] < newest - 20.0:
            self.gt_history.pop(0)

    def truth_at(self, t):
        before = [item for item in self.gt_history if item[0] <= t]
        after = [item for item in self.gt_history if item[0] >= t]
        if not before or not after:
            return None
        t0, m0 = before[-1]
        t1, m1 = after[0]
        p0, p1 = m0.pose.pose.position, m1.pose.pose.position
        pos0, pos1 = np.array([p0.x, p0.y, p0.z]), np.array([p1.x, p1.y, p1.z])
        q0, q1 = m0.pose.pose.orientation, m1.pose.pose.orientation
        r0 = Rotation.from_quat([q0.x, q0.y, q0.z, q0.w])
        r1 = Rotation.from_quat([q1.x, q1.y, q1.z, q1.w])
        if t1 > t0:
            u = (t - t0) / (t1 - t0)
            pos = (1 - u) * pos0 + u * pos1
            rot = Slerp([t0, t1], Rotation.concatenate([r0, r1]))([t])[0]
        else:
            pos, rot = pos0, r0
        return pos, rot

    def estimator_status(self, msg):
        try:
            d = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if d.get("event") == "gnss_update":
            self.gnss_debug_writer.writerow([d.get("stamp"), d.get("estimator_state_stamp"), d.get("accepted"),
                d.get("nis"), d.get("reason"), json.dumps(d.get("velocity_observation")),
                json.dumps(d.get("velocity_covariance")), d.get("velocity_accepted"), d.get("velocity_nis"),
                d.get("velocity_reason")])
            self.gnss_debug_file.flush()
            return
        if d.get("event") != "visual_span":
            return
        ratio = d["inliers"] / d["tracks"] if d.get("tracks") else math.nan
        gt_rot_error = gt_inv_error = gt_dir_error = math.nan
        start = self.truth_at(d.get("start_stamp")) if d.get("start_stamp") is not None else None
        end = self.truth_at(d.get("end_stamp")) if d.get("end_stamp") is not None else None
        if start is not None and end is not None:
            gt_rel = start[1].inv() * end[1]
            if d.get("visual_relative_rotation_body") is not None:
                vis = Rotation.from_matrix(np.asarray(d["visual_relative_rotation_body"]))
                gt_rot_error = math.degrees((gt_rel.inv() * vis).magnitude())
                gt_inv_error = math.degrees((gt_rel.inv() * vis.inv()).magnitude())
            if d.get("visual_translation_direction_world") is not None:
                disp = end[0] - start[0]
                vis_dir = np.asarray(d["visual_translation_direction_world"], dtype=float)
                if np.linalg.norm(disp) > 1e-6 and np.linalg.norm(vis_dir) > 1e-6:
                    c = np.clip(np.dot(disp, vis_dir) / (np.linalg.norm(disp) * np.linalg.norm(vis_dir)), -1, 1)
                    gt_dir_error = math.degrees(math.acos(float(c)))
        self.debug_writer.writerow([d.get("event"), d.get("start_stamp"), d.get("end_stamp"), d.get("frame_dt"),
            d.get("imu_state_stamp"), d.get("tracks"), d.get("inliers"), ratio, d.get("rotation_accepted"),
            d.get("rotation_nis"), d.get("rotation_reason"), d.get("rotation_innovation_deg"),
            json.dumps(d.get("visual_relative_rotation_body")), json.dumps(d.get("predicted_relative_rotation_body")),
            gt_rot_error, gt_inv_error, d.get("direction_accepted"), d.get("direction_nis"),
            d.get("direction_reason"), d.get("direction_innovation_deg"),
            json.dumps(d.get("visual_translation_direction_world")), gt_dir_error])
        self.debug_file.flush()

    def write_row(self):
        if self.gt is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.started:
            self.started = True
            meta_path = self.run_dir / "metadata.json"
            metadata = json.loads(meta_path.read_text(encoding="utf-8"))
            metadata["simulation_start_time_s"] = now
            meta_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        # CSV's GPS fields become blank immediately on denial. The rosbag retains raw GPS separately.
        gps_rows = ([stamp(self.gps), self.gps.latitude, self.gps.longitude, self.gps.altitude]
                    if self.gps is not None else None)
        gps = gps_csv_values(self.gnss_available, gps_rows, now)
        gps_ok = all(math.isfinite(v) for v in gps)
        imu = self.imu
        imu_v = ([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
                  imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z] if imu else [math.nan] * 6)
        gt = pose_values(self.gt)
        est = pose_values(self.est) if self.est else [math.nan] * 9
        self.writer.writerow([now, *gt, *est, *gps, int(gps_ok),
                              self.baro.fluid_pressure if self.baro else math.nan, *imu_v])
        self.file.flush()
        self.rows += 1
        if self.rows == 1 or self.rows % 100 == 0:
            self.log_file.write(f"sim_t={now:.2f}s gnss_available={self.gnss_available} "
                                f"gt_enu=({gt[0]:.1f},{gt[1]:.1f},{gt[2]:.1f}) csv_rows={self.rows}\n")
            self.log_file.flush()

    def close(self):
        self.file.close()
        self.log_file.close()
        self.debug_file.close()
        self.gnss_debug_file.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = RunLogger(args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

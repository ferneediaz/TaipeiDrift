"""ROS adapter: simulated GNSS + IMU + barometer, with independently switchable vision updates.

With rf_fix:=true it also fuses the position from the AIS ships' bearings (/rf_nav/odom, nodes/rf_nav.py) like a
GNSS position fix (horizontal position, plus heading; see on_rf), with the settings of eskf_rf_fix in
rf_config (sim/config/rf.yaml). That keeps the estimate anchored after the GNSS cutoff, and lets it start without
GNSS (gps:=false). Only an /rf_nav/odom message that carries a new bearing is used (its covariance shrank), so the
filtered RF track is not counted again between bearings. publish_tf:=false lets a second instance run beside the
first one.
"""
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, FluidPressure, Image, Imu, NavSatFix
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "baseline"))
sys.path.insert(0, str(REPO))
from vio.estimation.eskf import ESKF, ImuNoiseModel, P_, V_, TH, skew, UpdateResult
from vio.eskf_pipeline import noise_model
from vio.pipeline import CameraSetup, pose_config, tracker_config
from vio.vision.feature_tracker import KeyframeTracker
from vio.vision.measurements import measurements_from_tracks
from vio.vision.relative_pose import PoseConfig
from vio.vision.camera import pinhole_intrinsics
from gnss_projection import geodetic_to_enu, geodetic_covariance_to_enu
from frame_conversions import gazebo_optical_to_flu, gazebo_down_optical_to_flu
from visual_update_math import direction_update_terms


def msg_time(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class FrozenEskfAdapter(Node):
    def __init__(self):
        super().__init__("frozen_eskf_adapter", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        with (REPO / "vio/configs/midair_eskf.yaml").open(encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)
        with (REPO / self.cfg["vio_config"]).open(encoding="utf-8") as f:
            self.vio_cfg = yaml.safe_load(f)
        self.noise: ImuNoiseModel = noise_model(self.cfg)
        self.fc = self.cfg["forward_camera"]
        self.direction = self.cfg["direction"]
        self.gnss_cfg = self.cfg["sim_gnss"]
        self.world_origin = (float(self.declare_parameter("gps_origin_latitude", 25.0330).value),
                             float(self.declare_parameter("gps_origin_longitude", 121.5654).value),
                             float(self.declare_parameter("gps_origin_elevation", 10.0).value))
        self.vision_rotation = bool(self.declare_parameter("vision_rotation", True).value)
        self.vision_direction = bool(self.declare_parameter("vision_direction", True).value)
        self.rf_fix = bool(self.declare_parameter("rf_fix", False).value)
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        rf_config = Path(self.declare_parameter("rf_config", str(REPO / "sim/config/rf.yaml")).value)
        if self.rf_fix:
            with rf_config.open(encoding="utf-8") as f:
                self.rf_cfg = yaml.safe_load(f)["eskf_rf_fix"]
        camera = self.vio_cfg["cameras"][self.fc["camera"]]
        self.R_bc = gazebo_optical_to_flu()
        self.camera = CameraSetup.from_config(self.vio_cfg, self.fc["camera"])
        self.pose_cfg: PoseConfig = pose_config(self.vio_cfg)
        self.tracker_cfg = tracker_config(self.vio_cfg)
        # In the 512px Gazebo camera, fewer than the Mid-Air offline 80-track
        # reanchor target are normal. Keep keyframes until the unchanged pose
        # estimator's own 30-correspondence validity floor is reached.
        self.tracker_cfg.min_tracks = self.pose_cfg.min_correspondences
        self.tracker = KeyframeTracker(self.tracker_cfg)
        self.K = None
        self.frame_index = 0
        self.filter = None
        self.gnss_origin = None
        self.last_gnss = None
        self.gnss_velocity_anchor = None
        self.last_gnss_update_stamp = None
        self.gnss_updates = 0
        self.gnss_rejections = 0
        self.startup_accel = []
        self.startup_gyro = []
        self.baro0 = None
        self.baro_reference_z = None
        self.baro_latest = None
        self.t0 = None
        self.last_imu = None
        self.sample_index = -1
        self.accel_history = []
        self.gyro_history = []
        self.time_history = []
        self.vel_history = []
        self.clone_index = None
        self.clone_stamp = None
        self.cut_seen = False
        self.last_rf = None             # (t, xy, 2x2 covariance) of the latest RF fix with a new bearing
        self.rf_trace = math.inf        # covariance trace of the previous /rf_nav/odom message
        self.rf_rejected_in_row = 0
        self.rf_updates = 0
        self.rf_rejections = 0
        self.camera_frames = 0
        self.camera_spans = 0
        self.last_direction_innovation_deg = None
        self.pub = self.create_publisher(Odometry, "/nav/odom", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, "/nav/estimator_status", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_sensor_transforms()
        self.create_subscription(NavSatFix, "/gps/fix", self.on_gnss, qos_profile_sensor_data)
        if self.rf_fix:
            self.create_subscription(Odometry, "/rf_nav/odom", self.on_rf, 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(FluidPressure, "/air_pressure", self.on_baro, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/forward/camera_info", self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/forward/image_raw", self.on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"ESKF adapter ready; GNSS{' and RF' if self.rf_fix else ''} initialize/update position; vision rotation={self.vision_rotation}, "
            f"direction={self.vision_direction}; tracker reanchor floor={self.tracker_cfg.min_tracks} "
            f"(pose minimum={self.pose_cfg.min_correspondences}); no ground-truth subscription")

    def on_gnss(self, msg):
        t = msg_time(msg)
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude) and math.isfinite(msg.altitude)):
            return
        position = geodetic_to_enu(msg.latitude, msg.longitude, msg.altitude, self.world_origin)
        covariance = geodetic_covariance_to_enu(msg.position_covariance, msg.latitude,
                                                 self.gnss_cfg["fallback_sigma_m"])
        if self.filter is None:
            self.last_gnss = (t, position, covariance)
            self.try_initialize_from_measurements()
            return
        if self.last_gnss_update_stamp is not None and t <= self.last_gnss_update_stamp:
            return
        position_result = self.filter.update_position(position, covariance, self.gnss_cfg["gate_prob"])
        velocity_result = None
        velocity_observation = velocity_covariance = None
        if self.gnss_velocity_anchor is None:  # the filter started from an RF fix
            self.gnss_velocity_anchor = (t, position.copy(), covariance.copy())
        anchor_t, anchor_p, anchor_cov = self.gnss_velocity_anchor
        dt = t - anchor_t
        if dt >= self.gnss_cfg["velocity_baseline_s"]:
            # Difference separated fixes; uncertainty is propagated from both endpoint covariances.
            velocity_observation = (position - anchor_p) / dt
            velocity_covariance = (covariance + anchor_cov) / (dt * dt)
            velocity_result = self.filter.update_velocity(velocity_observation, velocity_covariance,
                                                           self.gnss_cfg["gate_prob"])
            # Use disjoint baselines to avoid pretending overlapping differences are independent.
            self.gnss_velocity_anchor = (t, position.copy(), covariance.copy())
        self.last_gnss_update_stamp = t
        self.gnss_updates += int(position_result.accepted)
        self.gnss_rejections += int(not position_result.accepted)
        self.status_pub.publish(String(data=json.dumps({"event": "gnss_update", "stamp": t,
            "estimator_state_stamp": self.time_history[-1] if self.time_history else None,
            "accepted": position_result.accepted, "nis": position_result.nis, "reason": position_result.reason,
            "velocity_observation": velocity_observation.tolist() if velocity_observation is not None else None,
            "velocity_covariance": velocity_covariance.tolist() if velocity_covariance is not None else None,
            "velocity_accepted": velocity_result.accepted if velocity_result else None,
            "velocity_nis": velocity_result.nis if velocity_result else None,
            "velocity_reason": velocity_result.reason if velocity_result else None,
            "updates": self.gnss_updates, "rejections": self.gnss_rejections})))

    def on_rf(self, msg):
        """Horizontal position and heading from the ships' bearings (no height: rf_nav has none).

        Fused like a GNSS position fix, with two differences, both measured in the strait world:
        - no finite-difference velocity: RF errors wander slowly (tens of metres over tens of seconds, from the
          direction finder's fixed bias and the ships' geometry), so differencing fixes 5 s apart gave velocity
          errors of 10 m/s. The position updates correct the velocity through the filter's correlations instead.
        - the heading is fused too: without GNSS the ESKF's yaw starts as an assumption (0) and is unobservable,
          and rf_nav estimates it from the same bearings.
        If the gate rejects reanchor_after fixes in a row, the ESKF has drifted (the ships are the absolute
        reference): its horizontal position is reset to the fix, with the fix's covariance.
        """
        t = msg_time(msg)
        c6 = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
        c = c6[:2, :2]
        trace, previous = float(np.trace(c)), self.rf_trace
        self.rf_trace = trace
        xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        # rf_nav publishes its filtered state at the IMU rate; between bearings that is a prediction, no new
        # information. Its covariance only shrinks when a bearing was fused: use those messages only.
        if not (np.all(np.isfinite(xy)) and np.all(np.isfinite(c))) or trace >= previous:
            return
        self.last_rf = (t, xy, c)
        if self.filter is None:
            self.try_initialize_from_measurements()
            return
        H = np.zeros((2, self.filter.n))
        H[:, P_.start:P_.start + 2] = np.eye(2)
        gate = self.rf_cfg["gate_prob"]
        position_result = self.filter.update(xy - self.filter.p[:2], H, c, gate)
        q = msg.pose.pose.orientation
        rf_yaw = 2 * math.atan2(q.z, q.w)
        eskf_yaw = self.filter.R.as_euler("ZYX")[0]
        Hy = np.zeros((1, self.filter.n))
        Hy[0, TH.start + 2] = 1.0  # world-side attitude error: its z component is the yaw error
        heading_innovation = (rf_yaw - eskf_yaw + math.pi) % (2 * math.pi) - math.pi
        heading_result = self.filter.update(np.array([heading_innovation]), Hy, c6[5:6, 5:6], gate)
        self.rf_rejected_in_row = 0 if position_result.accepted else self.rf_rejected_in_row + 1
        reanchored = self.rf_rejected_in_row >= self.rf_cfg["reanchor_after"]
        if reanchored:
            # Reset the position and drop its correlations; the barometer carries the height
            self.filter.p[:2] = xy
            self.filter.P[P_, :] = 0.0
            self.filter.P[:, P_] = 0.0
            self.filter.P[P_, P_] = np.diag([0.0, 0.0, self.rf_cfg["start_height_sigma_m"] ** 2])
            self.filter.P[P_.start:P_.start + 2, P_.start:P_.start + 2] = c
            self.filter.P[V_, V_] += np.eye(3) * self.rf_cfg["reanchor_velocity_sigma_mps"] ** 2
            self.rf_rejected_in_row = 0
            self.get_logger().warn(f"RF: {self.rf_cfg['reanchor_after']} fixes rejected in a row; "
                                   f"position reset to the RF fix")
        self.rf_updates += int(position_result.accepted)
        self.rf_rejections += int(not position_result.accepted)
        self.status_pub.publish(String(data=json.dumps({"event": "rf_update", "stamp": t,
            "accepted": position_result.accepted, "nis": position_result.nis, "reason": position_result.reason,
            "sigma_m": math.sqrt(trace / 2), "heading_accepted": heading_result.accepted,
            "heading_innovation_deg": math.degrees(heading_innovation),
            "reanchored": reanchored, "updates": self.rf_updates, "rejections": self.rf_rejections})))

    def try_initialize_from_measurements(self):
        n = self.gnss_cfg["attitude_init_samples"]
        if self.filter is not None or (self.last_gnss is None and self.last_rf is None) or len(self.startup_accel) < n:
            return
        # Reject the spawn/landing transient: only initialize from a settled gravity and gyro window.
        if not self.time_history or self.time_history[-1] < 1.0:
            return
        accel = np.asarray(self.startup_accel[-n:])
        gyro = np.asarray(self.startup_gyro[-n:])
        mean_accel = np.mean(accel, axis=0)
        if (np.std(np.linalg.norm(accel, axis=1)) > 0.25 or
                not 9.0 <= np.linalg.norm(mean_accel) <= 10.6 or
                np.linalg.norm(np.mean(gyro, axis=0)) > 0.05):
            return
        if self.last_gnss is not None:
            source = "GNSS"
            t, position, covariance = self.last_gnss
        else:
            # No GNSS: horizontal position from the ships; height of the take-off pad
            source = "RF"
            t, xy, cov_xy = self.last_rf
            position = np.array([xy[0], xy[1], self.rf_cfg["start_height_m"]])
            covariance = np.diag([0.0, 0.0, self.rf_cfg["start_height_sigma_m"] ** 2])
            covariance[:2, :2] = cov_xy
        up_body = mean_accel / max(float(np.linalg.norm(mean_accel)), 1e-12)
        world_up = np.array([0.0, 0.0, 1.0])
        cross = np.cross(up_body, world_up)
        angle = math.acos(float(np.clip(np.dot(up_body, world_up), -1.0, 1.0)))
        align = Rotation.from_rotvec(cross / max(float(np.linalg.norm(cross)), 1e-12) * angle)
        q = align.as_quat()
        ba0 = mean_accel - align.inv().apply([0.0, 0.0, 9.80665])
        bg0 = np.mean(gyro, axis=0)
        self.filter = ESKF(position, [0.0, 0.0, 0.0], [q[3], q[0], q[1], q[2]],
                           [0.0, 0.0, -9.80665], "body", self.noise, ba0=ba0, bg0=bg0)
        # Keep velocity and IMU history indices aligned for camera intervals that
        # reference startup frames preceding filter initialization.
        self.vel_history = [self.filter.v.copy() for _ in self.time_history]
        self.t0 = t
        self.last_gnss_update_stamp = t
        if source == "GNSS":
            self.gnss_velocity_anchor = (t, position.copy(), covariance.copy())
        else:
            self.filter.P[P_, P_] = covariance  # metres of RF uncertainty, not the GNSS-aided 0.5 m
        self.get_logger().info(f"initialized from {source} at sim t={t:.3f}s; origin={self.world_origin}; "
                               f"yaw=0 ENU; settled IMU window (ba={ba0.tolist()}, bg={bg0.tolist()})")
        self.status_pub.publish(String(data=json.dumps({"event": f"{source.lower()}_initialized", "stamp": t,
            "enu_position_m": position.tolist(), "attitude_source": "settled IMU gravity; yaw=0 ENU",
            "initial_accel_bias": ba0.tolist(), "initial_gyro_bias": bg0.tolist()})))

    def on_camera_info(self, msg):
        if msg.width and msg.height:
            self.K = np.asarray(msg.k, dtype=float).reshape(3, 3)

    def publish_sensor_transforms(self):
        transforms = []
        for child, R_bc in (("camera_forward", gazebo_optical_to_flu()),
                            ("camera_down", gazebo_down_optical_to_flu())):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "sensor_link"
            tf.child_frame_id = child
            q = Rotation.from_matrix(R_bc).as_quat()
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q.tolist()
            transforms.append(tf)
        self.tf_static_broadcaster.sendTransform(transforms)

    def on_baro(self, msg):
        self.baro_latest = (msg_time(msg), float(msg.fluid_pressure))

    def on_imu(self, msg):
        t = msg_time(msg)
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if self.filter is None:
            self.startup_accel.append(a)
            self.startup_gyro.append(w)
            n = self.gnss_cfg["attitude_init_samples"]
            if len(self.startup_accel) > n:
                self.startup_accel.pop(0)
                self.startup_gyro.pop(0)
            self.accel_history.append(a)
            self.gyro_history.append(w)
            self.time_history.append(t)
            self.try_initialize_from_measurements()
            return
        if self.last_imu is None:
            self.last_imu = (t, a, w)
            self.accel_history.append(a)
            self.gyro_history.append(w)
            self.time_history.append(t)
            self.vel_history.append(self.filter.v.copy())
            self.sample_index = 0
            self.publish_state(t)
            return
        t0, a0, w0 = self.last_imu
        if t <= t0:
            return
        self.filter.predict(a0, a, w0, w, t - t0)
        self.last_imu = (t, a, w)
        self.sample_index += 1
        self.accel_history.append(a)
        self.gyro_history.append(w)
        self.time_history.append(t)
        self.vel_history.append(self.filter.v.copy())
        if self.sample_index % 20 == 0:
            self.update_baro(t)
        self.publish_state(t)

    def update_baro(self, t):
        if self.baro_latest is None:
            return
        if self.baro0 is None:
            self.baro0 = self.baro_latest[1]
            self.baro_reference_z = float(self.filter.p[2])
        pressure = max(self.baro_latest[1], 1.0)
        dz = 8434.5 * math.log(self.baro0 / pressure)
        elapsed = max(0.0, t - self.t0)
        b = self.cfg["barometer"]
        sigma = math.sqrt(b["white_std_m"]**2 + b["bias_walk_m_per_sqrt_s"]**2 * elapsed
                          + (b["drift_sigma_m_per_s"] * elapsed)**2
                          + (b["scale_error_rms"] * dz)**2)
        # Barometer altitude is relative to the fixed pressure baseline. Do not add the
        # cumulative dz to the current estimate at every update (that re-applies the climb).
        self.filter.update_altitude(self.baro_reference_z + dz, np.array([0.0, 0.0, 1.0]), sigma,
                                    self.cfg["forward_camera"]["gate_prob"])

    def on_image(self, msg):
        if self.filter is None or not self.time_history or not (self.vision_rotation or self.vision_direction):
            return
        h, w = msg.height, msg.width
        data = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()
        if enc in ("rgb8", "bgr8"):
            image = data.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
            gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY)
        elif enc == "mono8":
            gray = data.reshape(h, msg.step)[:, :w]
        else:
            return
        sample = int(np.clip(np.searchsorted(self.time_history, msg_time(msg)), 0, len(self.time_history) - 1))
        self.camera_frames += 1
        tr = self.tracker.process(gray, self.frame_index)
        local_for_frame = lambda _: sample
        measurements = measurements_from_tracks([tr], self.K if self.K is not None else
            pinhole_intrinsics(w, h, self.camera.hfov_deg), self.R_bc, local_for_frame,
            "essential", self.pose_cfg)
        m = measurements[0]
        if tr.new_keyframe:
            if self.filter.has_clone:
                self.filter.drop_clone()
            self.filter.clone_attitude()
            self.clone_index = sample
            self.clone_stamp = msg_time(msg)
        if tr.reanchor and self.filter.has_clone:
            rotation_result = None
            direction_result = None
            predicted_C = self.filter.R_clone.inv() * self.filter.R
            visual_C = None
            if m.valid and self.vision_rotation and m.body_rotation_candidates:
                visual_C = Rotation.from_matrix(m.body_rotation_candidates[0])
                rotation_result = self.filter.update_relative_rotation(m.body_rotation_candidates[0],
                    math.radians(self.fc["sigma_deg"]), self.fc["gate_prob"])
            if (m.valid and self.vision_direction and m.translation_dir_cam is not None and
                    sample > (self.clone_index if self.clone_index is not None else -1)):
                direction_result = self.update_direction(m, sample)
            self.camera_spans += 1
            visual_dir_body = (self.R_bc @ m.translation_dir_cam if m.translation_dir_cam is not None else None)
            visual_dir_world = (self.filter.R_clone.apply(visual_dir_body).tolist()
                                if visual_dir_body is not None else None)
            predicted_velocity = (np.mean(self.vel_history[max(0, self.clone_index or 0):sample + 1], axis=0)
                                  if sample > (self.clone_index if self.clone_index is not None else -1) else np.zeros(3))
            status = {"event": "visual_span", "frame": self.frame_index, "valid": m.valid,
                      "start_stamp": self.clone_stamp, "end_stamp": msg_time(msg),
                      "frame_dt": msg_time(msg) - self.clone_stamp if self.clone_stamp is not None else None,
                      "imu_state_stamp": self.time_history[sample],
                      "tracks": m.n_correspondences, "inliers": m.n_inliers,
                      "rotation_accepted": bool(rotation_result and rotation_result.accepted),
                      "direction_accepted": bool(direction_result and direction_result.accepted),
                      "rotation_nis": rotation_result.nis if rotation_result else None,
                      "rotation_reason": rotation_result.reason if rotation_result else "disabled_or_unavailable",
                      "direction_nis": direction_result.nis if direction_result else None,
                      "direction_reason": direction_result.reason if direction_result else "disabled_or_unavailable",
                      "direction_innovation_deg": self.last_direction_innovation_deg,
                      "visual_relative_rotation_body": visual_C.as_matrix().tolist() if visual_C else None,
                      "predicted_relative_rotation_body": predicted_C.as_matrix().tolist(),
                      "rotation_innovation_deg": (math.degrees((predicted_C.inv() * visual_C).magnitude())
                                                   if visual_C else None),
                      "visual_translation_direction_camera": (m.translation_dir_cam.tolist()
                                                               if m.translation_dir_cam is not None else None),
                      "visual_translation_direction_body": (visual_dir_body.tolist()
                                                            if visual_dir_body is not None else None),
                      "visual_translation_direction_world": visual_dir_world,
                      "predicted_velocity_world": predicted_velocity.tolist(),
                      "camera_frames": self.camera_frames, "visual_spans": self.camera_spans}
            self.status_pub.publish(String(data=json.dumps(status)))
            self.filter.drop_clone()
            self.filter.clone_attitude()
            self.clone_index = sample
            self.clone_stamp = msg_time(msg)
        self.frame_index += 1

    def update_direction(self, measurement, sample):
        first = max(0, self.clone_index or 0)
        v = np.mean(self.vel_history[first:sample + 1], axis=0)
        speed = float(np.linalg.norm(v))
        if speed < self.direction["min_speed_mps"]:
            return None
        d_vis = self.filter.R_clone.apply(self.R_bc @ measurement.translation_dir_cam)
        d_pred = v / speed
        self.last_direction_innovation_deg = math.degrees(math.acos(float(np.clip(d_vis @ d_pred, -1.0, 1.0))))
        try:
            residual, H, d_vis, d_pred = direction_update_terms(self.filter, v, d_vis)
        except ValueError as exc:
            return UpdateResult(False, math.nan, 2, str(exc))
        sigma = math.radians(self.direction["sigma_deg"])
        return self.filter.update(residual, H, np.eye(2) * sigma**2, self.direction["gate_prob"])

    def publish_state(self, t):
        R = self.filter.R
        q = R.as_quat()
        msg = Odometry()
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int((t - int(t)) * 1e9)
        msg.header.frame_id = "world"
        msg.child_frame_id = "sensor_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = self.filter.p.tolist()
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q.tolist()
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = self.filter.v.tolist()
        cov = self.filter.P
        for i in range(3):
            for j in range(3):
                msg.pose.covariance[i * 6 + j] = float(cov[i, j])
                msg.pose.covariance[(i + 3) * 6 + j + 3] = float(cov[TH.start + i, TH.start + j])
                msg.twist.covariance[i * 6 + j] = float(cov[V_.start + i, V_.start + j])
        self.pub.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = FrozenEskfAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

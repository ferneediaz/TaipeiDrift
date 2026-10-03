"""Record simulator topics directly as taipeidrift-replay/1."""
import argparse
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Image, Imu, NavSatFix

SCHEMA = "taipeidrift-replay/1"
CSV_COLUMNS = {
    "imu": ["t_s", "gx", "gy", "gz", "ax", "ay", "az"],
    "baro": ["t_s", "pressure_pa", "temperature_c", "alt_isa_m"],
    "gnss": ["t_s", "lat_deg", "lon_deg", "alt_m", "fix_type", "hacc_m", "vacc_m",
             "ve_mps", "vn_mps", "vu_mps", "nsat"],
    "images": ["t_s", "cam", "path"],
    "truth": ["t_s", "e_m", "n_m", "u_m", "lat_deg", "lon_deg", "alt_m", "qw", "qx", "qy", "qz"],
}
EARTH_RADIUS_M = 6_378_137.0


def stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def variance_sqrt(value):
    if not math.isfinite(value) or value < 0:
        return ""
    return math.sqrt(value)


def camera_transform_body_from_camera(y_m: float) -> list[list[float]]:
    """Pose of the Gazebo camera frame in the body FLU frame (sensor SDF pitch = +90 deg)."""
    return [
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 1.0, 0.0, y_m],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


class ReplayRecorder(Node):
    def __init__(self, args):
        super().__init__(
            "t_replay_recorder",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.args = args
        self.t0_ns = None
        self.latest_ns = None
        self.last_flush_ns = None
        self.done = False
        self.image_period_ns = round(1_000_000_000 / args.image_rate_hz)
        self.last_image_ns = {}
        self.image_reorder_window_ns = 250_000_000
        self.pending_image_rows = []
        self.latest_image_ns = None
        self.counts = {key: 0 for key in CSV_COLUMNS}
        self.counts["images"] = {"cam0": 0}
        if args.stereo:
            self.counts["images"]["cam1"] = 0

        world_path = Path(__file__).resolve().parents[1] / "worlds" / f"{args.world}.sdf"
        world = ET.parse(world_path)
        spherical = world.find(".//spherical_coordinates")
        if spherical is None:
            raise ValueError(f"{world_path} has no spherical_coordinates origin")
        self.origin = {
            "lat_deg": float(spherical.findtext("latitude_deg")),
            "lon_deg": float(spherical.findtext("longitude_deg")),
            "alt_m": float(spherical.findtext("elevation")),
        }

        root = Path(args.output_root)
        name = Path(args.name)
        if name.name != args.name or args.name in ("", ".", ".."):
            raise ValueError("recording name must be one directory name")
        self.out = root / name
        self.out.mkdir(parents=True, exist_ok=False)
        (self.out / "images" / "cam0").mkdir(parents=True)
        if args.stereo:
            (self.out / "images" / "cam1").mkdir(parents=True)

        self.files = {}
        self.writers = {}
        for key, columns in CSV_COLUMNS.items():
            path = self.out / ("images.csv" if key == "images" else f"{key}.csv")
            stream = path.open("w", newline="", encoding="utf-8")
            writer = csv.writer(stream)
            writer.writerow(columns)
            self.files[key] = stream
            self.writers[key] = writer

        self.meta = self.build_meta()
        self.write_meta()

        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(FluidPressure, "/air_pressure", self.on_baro, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/gps/fix", self.on_gnss, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/down/image_raw", lambda m: self.on_image(m, "cam0"), 5)
        if args.stereo:
            self.create_subscription(Image, "/camera/down_right/image_raw", lambda m: self.on_image(m, "cam1"), 5)

        self.get_logger().info(
            f"Recording {self.out}; image_rate_hz={args.image_rate_hz:g}; duration_s={args.duration_s}"
        )

    def build_meta(self):
        width = self.args.cam_res
        focal = width / 2.0
        k = [focal, 0.0, focal, 0.0, focal, focal, 0.0, 0.0, 1.0]
        cams = {
            "cam0": {
                "K": k,
                "width": width,
                "height": width,
                "rate_hz": 25.0,
                "recorded_rate_hz": self.args.image_rate_hz,
                "direction": "down; image top points body-forward",
                "T_body_cam": camera_transform_body_from_camera(0.0),
            }
        }
        if self.args.stereo:
            cams["cam1"] = {
                "K": k,
                "width": width,
                "height": width,
                "rate_hz": 25.0,
                "recorded_rate_hz": self.args.image_rate_hz,
                "direction": "down; image top points body-forward",
                "T_body_cam": camera_transform_body_from_camera(-self.args.stereo_baseline_m),
            }
        return {
            "schema": SCHEMA,
            "sequence": self.args.name,
            "evidence_label": "SIMULATED",
            "source": f"TaipeiDrift Gazebo Harmonic simulation; world={self.args.world}; origin read from spherical_coordinates in the world SDF",
            "origin": self.origin,
            "licence": "Synthetic output; simulator model and world assets retain their upstream licences.",
            "gnss_cut_s": self.args.gnss_cut_s,
            "clock": "t_s is message simulation time minus the first received message time; earlier startup samples are clamped to 0; gnss_cut_s is absolute simulation time",
            "sensors": {
                "imu": {
                    "rate_hz": 100.0,
                    "frame": "sensor_link body FLU (+X forward, +Y left, +Z up)",
                    "noise_model": "Mid-Air eq. 1 per-axis white noise plus random-walk bias; orientation is zeroed and orientation_covariance[0] = -1",
                    "provenance": "sim/nodes/sensor_noise.py; bounds from sim/config/sensor_noise.yaml",
                },
                "baro": {
                    "rate_hz": 50.0,
                    "altitude_semantics": "raw pressure from Gazebo AirPressure sensor + declared noise/drift",
                    "provenance": "Gazebo AirPressure sensor (10 Pa Gaussian noise in SDF) plus random-walk drift in sim/nodes/sensor_noise.py",
                },
                "gnss": {
                    "rate_hz": 1.0,
                    "alt_ref": "ellipsoid (Gazebo EARTH_WGS84 SphericalCoordinates)",
                    "provenance": "Gazebo NavSat sensor with SDF noise, bridged to /sim/gps_raw and gated to /gps/fix by sim/nodes/gnss_gate.py",
                },
                "camera": {"cams": cams},
                "truth": {
                    "source": "Gazebo OdometryPublisher /ground_truth/odom; pose in world ENU, q is body-to-ENU; geodetic coordinates approximated from the SDF origin",
                    "rate_hz": 100.0,
                    "evaluator_only": True,
                },
            },
            "counts": self.counts,
        }

    def write_meta(self):
        path = self.out / "meta.json"
        path.write_text(json.dumps(self.meta, indent=2) + "\n", encoding="utf-8")

    def relative_time(self, msg) -> tuple[int, float]:
        timestamp = stamp_ns(msg)
        if self.t0_ns is None:
            self.t0_ns = timestamp
            self.meta["first_message_sim_time_s"] = timestamp * 1e-9
        self.latest_ns = timestamp if self.latest_ns is None else max(self.latest_ns, timestamp)
        relative = max(0.0, (timestamp - self.t0_ns) * 1e-9)
        if self.args.duration_s is not None and relative >= self.args.duration_s:
            self.done = True
        return timestamp, relative

    def write_row(self, key: str, timestamp_ns: int, row) -> None:
        if key == "images":
            self.pending_image_rows.append((timestamp_ns, row))
            self.latest_image_ns = timestamp_ns if self.latest_image_ns is None else max(self.latest_image_ns, timestamp_ns)
            cutoff_ns = self.latest_image_ns - self.image_reorder_window_ns
            self.pending_image_rows.sort(key=lambda item: item[0])
            ready_count = 0
            while ready_count < len(self.pending_image_rows) and self.pending_image_rows[ready_count][0] <= cutoff_ns:
                image_ns, image_row = self.pending_image_rows[ready_count]
                self._write_row("images", image_ns, image_row)
                ready_count += 1
            del self.pending_image_rows[:ready_count]
            return
        self._write_row(key, timestamp_ns, row)

    def _write_row(self, key: str, timestamp_ns: int, row) -> None:
        self.writers[key].writerow(row)
        if key == "images":
            self.counts["images"][row[1]] += 1
        else:
            self.counts[key] += 1
        if self.last_flush_ns is None or timestamp_ns - self.last_flush_ns >= 1_000_000_000:
            self.flush()
            self.last_flush_ns = timestamp_ns

    def flush(self) -> None:
        for stream in self.files.values():
            stream.flush()

    def on_imu(self, msg: Imu) -> None:
        timestamp, t_s = self.relative_time(msg)
        w, a = msg.angular_velocity, msg.linear_acceleration
        self.write_row("imu", timestamp, [t_s, w.x, w.y, w.z, a.x, a.y, a.z])

    def on_baro(self, msg: FluidPressure) -> None:
        timestamp, t_s = self.relative_time(msg)
        pressure = msg.fluid_pressure
        alt_isa = 44330.769 * (1.0 - (pressure / 101325.0) ** 0.190263)
        self.write_row("baro", timestamp, [t_s, pressure, "", alt_isa])

    def on_gnss(self, msg: NavSatFix) -> None:
        timestamp, t_s = self.relative_time(msg)
        hacc = vacc = ""
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            covariance = msg.position_covariance
            hacc = variance_sqrt(max(covariance[0], covariance[4]))
            vacc = variance_sqrt(covariance[8])
        self.write_row("gnss", timestamp, [
            t_s, msg.latitude, msg.longitude, msg.altitude,
            3 if msg.status.status >= 0 else 0, hacc, vacc, "", "", "", "",
        ])

    def on_truth(self, msg: Odometry) -> None:
        timestamp, t_s = self.relative_time(msg)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        lat0 = math.radians(self.origin["lat_deg"])
        lat = self.origin["lat_deg"] + p.y / EARTH_RADIUS_M * 180.0 / math.pi
        lon = self.origin["lon_deg"] + p.x / (EARTH_RADIUS_M * math.cos(lat0)) * 180.0 / math.pi
        alt = self.origin["alt_m"] + p.z
        self.write_row("truth", timestamp, [t_s, p.x, p.y, p.z, lat, lon, alt, q.w, q.x, q.y, q.z])

    def on_image(self, msg: Image, cam: str) -> None:
        timestamp, t_s = self.relative_time(msg)
        previous = self.last_image_ns.get(cam)
        if previous is not None and timestamp - previous < self.image_period_ns:
            return
        channels_by_encoding = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
        channels = channels_by_encoding.get(msg.encoding)
        if channels is None:
            self.get_logger().error(f"Unsupported {cam} image encoding {msg.encoding!r}")
            return
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.step < msg.width * channels or data.size < msg.height * msg.step:
            self.get_logger().error(f"Malformed {cam} image buffer ({msg.width}x{msg.height}, step={msg.step})")
            return
        rows = data[:msg.height * msg.step].reshape(msg.height, msg.step)
        pixels = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
        if msg.encoding == "rgb8":
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "rgba8":
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGRA)
        filename = f"{timestamp}.png"
        relative_path = Path("images") / cam / filename
        if not cv2.imwrite(str(self.out / relative_path), pixels):
            self.get_logger().error(f"Failed to write {relative_path}")
            return
        self.last_image_ns[cam] = timestamp
        self.write_row("images", timestamp, [t_s, cam, relative_path.as_posix()])

    def close(self) -> None:
        self.pending_image_rows.sort(key=lambda item: item[0])
        for timestamp_ns, row in self.pending_image_rows:
            self._write_row("images", timestamp_ns, row)
        self.pending_image_rows.clear()
        self.flush()
        if self.t0_ns is not None:
            self.meta["first_message_sim_time_s"] = self.t0_ns * 1e-9
        if self.latest_ns is not None:
            self.meta["last_message_sim_time_s"] = self.latest_ns * 1e-9
            self.meta["duration_s"] = (self.latest_ns - self.t0_ns) * 1e-9
        self.meta["counts"] = self.counts
        self.write_meta()
        for stream in self.files.values():
            stream.close()
        self.get_logger().info(f"Recording stopped; sim_duration_s={self.meta.get('duration_s', 0):.3f}; counts={self.counts}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True, help="directory name under output-root")
    parser.add_argument("--world", default="terrain")
    parser.add_argument("--output-root", default="/ws/TaipeiDrift/recordings")
    parser.add_argument("--image-rate-hz", type=float, default=5.0)
    parser.add_argument("--duration-s", type=float, default=None, help="stop after this much relative simulation time")
    parser.add_argument("--gnss-cut-s", type=float, default=-1.0)
    parser.add_argument("--cam-res", type=int, default=512)
    parser.add_argument("--stereo", action="store_true")
    parser.add_argument("--stereo-baseline-m", type=float, default=0.30)
    args, ros_args = parser.parse_known_args()
    if args.image_rate_hz <= 0 or args.cam_res <= 0:
        parser.error("image-rate-hz and cam-res must be positive")
    if args.duration_s is not None and args.duration_s <= 0:
        parser.error("duration-s must be positive")

    rclpy.init(args=ros_args)
    node = ReplayRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

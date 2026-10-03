"""Record a flight in the simulator in the Mid-Air dataset's format, so code written for Mid-Air reads it unchanged.

    python3 sim/nodes/record_midair.py [--world islands] [--duration S] [--out data/sim] [--climate sunny]

Writes, as Mid-Air does (https://midair.ulg.ac.be/data_organization.html and the authors' M4Depth loader):

    data/sim/<world>/<climate>/
        color_down/trajectory_XXXX/000000.JPEG ...   down camera, 25 Hz, JPEG
        sensor_records.hdf5                         one group per trajectory:
            trajectory_XXXX/camera_data/color_down  25N paths, relative to the climate folder
            trajectory_XXXX/groundtruth/position    100N x 3, m, world frame, NED
                                       /velocity    100N x 3, m/s, world frame, NED
                                       /acceleration 100N x 3, m/s^2, world frame, NED
                                       /attitude    100N x 4, quaternion w x y z, body (FRD) to world (NED)
                                       /angular_velocity 100N x 3, rad/s, around the world NED axes (*)
            trajectory_XXXX/imu/accelerometer       100N x 3, m/s^2, body FRD, noisy; attrs init_bias_est
                               /gyroscope           100N x 3, rad/s, around the world NED axes (*), noisy;
                                                    attrs init_bias_est (drone axes FRD)
            trajectory_XXXX/gps/position            N x 3, m, world frame NED (from the noisy GNSS fix)
                               /velocity            N x 3, m/s, from consecutive fixes
                               /GDOP /PDOP /HDOP /VDOP /no_vis_sats   N x 1, fixed values: no satellites are simulated

(*) As in the Mid-Air files, not as their documentation says: docs/findings.md section 2.2 shows the files give
turn rates around the world axes. Code written for Mid-Air then works on these recordings unchanged. The rates
around the drone's own axes are R^-1 times these, with R the attitude.

N is the flight length in seconds; frame j goes with rows 4j of the 100 Hz records and row j // 25 of GPS.
The world origin, helipad A in the islands world, is the origin of every trajectory, so all flights share one
frame (in Mid-Air each flight starts at its own origin). The sensors sit at one point, so the body frame is the
IMU and camera position. Barometer readings, which Mid-Air does not have, go to trajectory_XXXX/barometer/pressure
(100N x 1, Pa).

Start it while the simulator runs, after the drone is flying. Ctrl+C or --duration ends the recording.

On a CPU the camera renders slower than the physics runs, so a free-running simulation skips frames. The recorder
therefore pauses the simulation and steps it one camera period (40 ms) at a time, waiting for each frame: every
frame is kept, and the simulation runs as fast as the camera renders. --free-run turns this off.
"""
import argparse
import json
import math
import queue
import re
import signal
import sys
import time
from pathlib import Path

import cv2
import h5py
import numpy as np
import rclpy
import rclpy.executors
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix
from std_msgs.msg import String

SIM = Path(__file__).resolve().parents[1]
CAM_HZ, GT_HZ = 25, 100
CAMERA_TOPICS = {"down": "/camera/down/image_raw", "forward": "/camera/forward/image_raw"}
WGS84_A, WGS84_E2 = 6378137.0, 6.69437999014e-3
# ENU world to NED world, and FRD body to FLU body: both swap or flip axes
ENU_TO_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def world_origin(world):
    text = (SIM / "worlds" / f"{world}.sdf").read_text()
    get = lambda tag: float(re.search(rf"<{tag}>([-\d.]+)</{tag}>", text).group(1))  # noqa: E731
    return get("latitude_deg"), get("longitude_deg"), get("elevation")


def ecef(lat, lon, alt):
    lat, lon = np.radians(lat), np.radians(lon)
    n = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat) ** 2)
    return np.stack([(n + alt) * np.cos(lat) * np.cos(lon), (n + alt) * np.cos(lat) * np.sin(lon),
                     (n * (1 - WGS84_E2) + alt) * np.sin(lat)], -1)


def geodetic_to_ned(lla, origin):
    """Latitude, longitude, altitude rows to metres north, east, down of the origin (WGS84, as Gazebo does)."""
    lat0, lon0, alt0 = origin
    d = ecef(lla[:, 0], lla[:, 1], lla[:, 2]) - ecef(lat0, lon0, alt0)
    la, lo = math.radians(lat0), math.radians(lon0)
    east = -math.sin(lo) * d[:, 0] + math.cos(lo) * d[:, 1]
    north = -math.sin(la) * math.cos(lo) * d[:, 0] - math.sin(la) * math.sin(lo) * d[:, 1] + math.cos(la) * d[:, 2]
    up = math.cos(la) * math.cos(lo) * d[:, 0] + math.cos(la) * math.sin(lo) * d[:, 1] + math.sin(la) * d[:, 2]
    return np.stack([north, east, -up], -1)


def interp(t, ts, values):
    """Linear interpolation of each column of `values` (sampled at ts) at times t."""
    return np.stack([np.interp(t, ts, values[:, k]) for k in range(values.shape[1])], -1)


class Recorder(Node):
    def __init__(self, folder, traj):
        # No use_sim_time: the stamps come from the messages, and /clock (1 kHz) would only flood the queue
        super().__init__("record_midair")
        self.executor_ = rclpy.executors.SingleThreadedExecutor()
        self.executor_.add_node(self)
        self.folder, self.traj = folder, traj
        self.img_dirs = {"down": folder / "color_down" / traj,
                         "forward": folder / "color_left" / traj}
        for path in self.img_dirs.values():
            path.mkdir(parents=True, exist_ok=True)
        self.frames = {name: {} for name in CAMERA_TOPICS}  # frame index -> stamp
        self.t0 = None
        self.armed = False  # frames count only once every subscription is connected
        self.odom, self.imu, self.gps, self.baro = [], [], [], []
        self.imu_params = None
        self.images = {name: queue.Queue() for name in CAMERA_TOPICS}  # filled from Gazebo transport
        deep = QoSProfile(depth=5000, reliability=QoSReliabilityPolicy.BEST_EFFORT)  # keep every 100 Hz reading
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_odom, deep)
        self.create_subscription(Imu, "/imu/data", self.on_imu, deep)
        self.create_subscription(NavSatFix, "/gps/fix", self.on_gps, deep)
        self.create_subscription(FluidPressure, "/air_pressure", self.on_baro, deep)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/imu/params", lambda m: setattr(self, "imu_params", json.loads(m.data)), latched)

    def take_images(self):
        """Save the camera frames that have arrived; the ROS readings are taken in between."""
        while True:
            received = False
            for name, images in self.images.items():
                try:
                    t, img = images.get_nowait()
                except queue.Empty:
                    continue
                received = True
                if self.armed:
                    self.on_image(name, t, img)
            if not received:
                return

    def spin(self, timeout):
        self.take_images()
        self.executor_.spin_once(timeout_sec=timeout)
        self.take_images()

    def on_image(self, name, t, img):
        if self.t0 is None:
            self.t0 = t
            self.get_logger().info(f"recording {self.traj}")
        k = round((t - self.t0) * CAM_HZ)
        cv2.imwrite(str(self.img_dirs[name] / f"{k:06d}.JPEG"), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.frames[name][k] = t
        if name == "down" and k % (10 * CAM_HZ) == 0:
            self.get_logger().info(f"{k / CAM_HZ:.0f} s recorded")

    def step_frame(self, world, steps):
        """Advance the paused simulation by one camera period and wait until that frame has arrived."""
        have = len(self.frames["down"])
        if not world.control(pause=True, multi_step=steps):
            return False  # the caller logs it and steps again; the frames so far are kept
        end = time.time() + 30
        while len(self.frames["down"]) == have and time.time() < end:
            self.spin(0.005)
        if len(self.frames["down"]) == have:
            return False
        # Take in the ground truth and IMU readings of this step before the next one. Their 100 Hz ticks can sit a
        # millisecond off the camera's, so wait for the latest tick at or before the frame, not one exactly at it.
        t = self.frames["down"][max(self.frames["down"])] - 1 / GT_HZ + 1e-6
        end = time.time() + 0.5
        while time.time() < end and not (self.odom and self.imu and self.odom[-1][0] >= t and self.imu[-1][0] >= t):
            self.spin(0.005)
        return True

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom.append((stamp(m), p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def on_imu(self, m):
        a, w = m.linear_acceleration, m.angular_velocity
        self.imu.append((stamp(m), a.x, a.y, a.z, w.x, w.y, w.z))

    def on_gps(self, m):
        self.gps.append((stamp(m), m.latitude, m.longitude, m.altitude))

    def on_baro(self, m):
        self.baro.append((stamp(m), m.fluid_pressure))

    def write(self, h5_path, origin):
        if not self.frames["down"]:
            self.get_logger().error("no camera frames received; is the simulator running?")
            return
        if not self.frames["forward"]:
            self.get_logger().error("no forward camera frames received; refusing to write an incomplete two-camera trajectory")
            return
        n_s = (max(self.frames["down"]) + 1) // CAM_HZ  # whole seconds, as in Mid-Air
        if n_s < 1:
            self.get_logger().error("less than one second recorded; nothing written")
            return
        n_frames = n_s * CAM_HZ
        for name, frames in self.frames.items():
            for k in [k for k in frames if k >= n_frames]:
                (self.img_dirs[name] / f"{k:06d}.JPEG").unlink()
            missing = [k for k in range(n_frames) if k not in frames]
            for k in missing:  # keep the 25 Hz time index stable if a camera drops a frame
                prior = [j for j in frames if j < k]
                prev = max(prior) if prior else min(frames)
                (self.img_dirs[name] / f"{k:06d}.JPEG").write_bytes(
                    (self.img_dirs[name] / f"{prev:06d}.JPEG").read_bytes())
            if missing:
                self.get_logger().warning(f"{name} camera: {len(missing)} of {n_frames} frames dropped and repeated")

        t = self.t0 + np.arange(n_s * GT_HZ) / GT_HZ
        for name, rows in (("ground truth", self.odom), ("IMU", self.imu)):
            have = {round((r[0] - self.t0) * GT_HZ) for r in rows}
            lost = [i for i in range(len(t)) if i not in have]
            got = len(t) - len(lost)
            msg = f"{name}: {got} of {len(t)} readings at 100 Hz received" + (
                f"; missing at {[round(i / GT_HZ, 2) for i in lost[:8]]} s (filled by interpolation)" if lost else "")
            (self.get_logger().info if got >= len(t) else self.get_logger().warning)(msg)
        od = np.array(sorted(self.odom))
        od = od[np.unique(od[:, 0], return_index=True)[1]]
        pos_ned = interp(t, od[:, 0], od[:, 1:4]) @ ENU_TO_NED.T
        rot_enu = Slerp(od[:, 0], Rotation.from_quat(od[:, 4:8]))(np.clip(t, od[0, 0], od[-1, 0]))
        rot_ned = Rotation.from_matrix(ENU_TO_NED @ rot_enu.as_matrix() @ FRD_TO_FLU)
        q = rot_ned.as_quat()[:, [3, 0, 1, 2]]  # w x y z
        q *= np.sign(q[:, :1] + 1e-12)
        vel = np.gradient(pos_ned, 1 / GT_HZ, axis=0)
        acc = np.gradient(vel, 1 / GT_HZ, axis=0)
        ang = (rot_ned[1:] * rot_ned[:-1].inv()).as_rotvec() * GT_HZ  # turn rates around the world axes, as Mid-Air
        ang = np.vstack([ang, ang[-1:]])

        imu = np.array(sorted(self.imu))
        accel = interp(t, imu[:, 0], imu[:, 1:4]) @ FRD_TO_FLU.T  # FLU to FRD: the same sign flips
        gyro = rot_ned.apply(interp(t, imu[:, 0], imu[:, 4:7]) @ FRD_TO_FLU.T)  # drone axes to world axes, as Mid-Air

        g = np.array(sorted(self.gps)) if self.gps else np.zeros((0, 4))
        gps_t = self.t0 + np.arange(n_s)
        if len(g):
            idx = np.clip(np.searchsorted(g[:, 0], gps_t + 1e-6) - 1, 0, len(g) - 1)  # latest fix at each second
            gps_pos = geodetic_to_ned(g[idx, 1:4], origin)
        else:
            gps_pos = np.full((n_s, 3), np.nan)
        gps_vel = np.gradient(gps_pos, 1.0, axis=0) if n_s > 1 else np.zeros_like(gps_pos)

        with h5py.File(h5_path, "a") as f:
            grp = f.create_group(self.traj)
            cam = grp.create_group("camera_data")
            cam.create_dataset("color_down", data=np.array(
                [f"color_down/{self.traj}/{k:06d}.JPEG" for k in range(n_frames)], dtype=h5py.string_dtype()))
            cam.create_dataset("color_left", data=np.array(
                [f"color_left/{self.traj}/{k:06d}.JPEG" for k in range(n_frames)], dtype=h5py.string_dtype()))
            gt = grp.create_group("groundtruth")
            for name, data in (("position", pos_ned), ("velocity", vel), ("acceleration", acc),
                               ("attitude", q), ("angular_velocity", ang)):
                gt.create_dataset(name, data=data.astype(np.float64))
            imu_g = grp.create_group("imu")
            for name, data, key in (("accelerometer", accel, "accel"), ("gyroscope", gyro, "gyro")):
                ds = imu_g.create_dataset(name, data=data)
                if self.imu_params:
                    ds.attrs["init_bias_est"] = np.array(self.imu_params[key]["initial_bias"]) * FRD_TO_FLU.diagonal()
            gps_g = grp.create_group("gps")
            gps_g.create_dataset("position", data=gps_pos)
            gps_g.create_dataset("velocity", data=gps_vel)
            for name, value in (("GDOP", 1.5), ("PDOP", 1.3), ("HDOP", 0.8), ("VDOP", 1.1), ("no_vis_sats", 10)):
                gps_g.create_dataset(name, data=np.full((n_s, 1), value, np.float64))
            gps_g.attrs["note"] = "DOP and satellite counts are fixed values; the simulator models no satellites"
            if self.baro:
                b = np.array(sorted(self.baro))
                grp.create_group("barometer").create_dataset("pressure", data=interp(t, b[:, 0], b[:, 1:2]))
            grp.attrs["source"] = "TaipeiDrift Gazebo simulator"
            grp.attrs["turn_rate_axes"] = "world NED, as in the Mid-Air files"
            grp.attrs["origin_lat_lon_alt"] = np.array(origin)
            grp.attrs["sim_time_start_s"] = self.t0
            if self.imu_params:
                grp.attrs["imu_noise_draw"] = json.dumps(self.imu_params)
        self.get_logger().info(f"wrote {self.traj}: {n_s} s, {n_frames} frames -> {self.folder}")


class GzWorld:
    """Camera frames and simulation stepping through Gazebo's own transport, not ROS. Over ROS, a lost piece of a
    3 MB image is only sent again after about 3 s, and stepping through the ROS bridge could stall as long; both made
    long recordings crawl."""

    def __init__(self, world, frames):
        sys.path += ["/opt/ros/jazzy/opt/gz_transport_vendor/lib/python", "/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python"]
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.image_pb2 import Image as GzImage
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.transport13 import Node as GzNode
        self.node, self.req_type, self.rep_type = GzNode(), WorldControl, Boolean
        self.service = f"/world/{world}/control"

        def on_image(name, m):  # Gazebo's thread: copy and hand over; files are written in the main thread
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[..., 2::-1]  # RGB to BGR for OpenCV
            frames[name].put((m.header.stamp.sec + m.header.stamp.nsec * 1e-9, img.copy()))

        for name, topic in CAMERA_TOPICS.items():
            self.node.subscribe(GzImage, topic, lambda m, name=name: on_image(name, m))

    def control(self, **fields):
        ok, rep = self.node.request(self.service, self.req_type(**fields), self.req_type, self.rep_type, 5000)
        return ok and rep.data


def physics_step(world):
    return float(re.search(r"<max_step_size>([\d.e-]+)</max_step_size>", (SIM / "worlds" / f"{world}.sdf").read_text()).group(1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="islands", help="the world that runs, for its origin and the folder name")
    ap.add_argument("--duration", type=float, default=0, help="seconds of simulation time to record; 0 records until Ctrl+C")
    ap.add_argument("--out", type=Path, default=SIM.parent / "data" / "sim")
    ap.add_argument("--climate", default="sunny", help="Mid-Air's weather folder name")
    ap.add_argument("--free-run", action="store_true", help="do not step the simulation; frames the camera skips are lost")
    args = ap.parse_args()

    folder = args.out / args.world / args.climate
    folder.mkdir(parents=True, exist_ok=True)
    h5_path = folder / "sensor_records.hdf5"
    done = set()
    if h5_path.exists():
        with h5py.File(h5_path, "r") as f:
            done = set(f.keys())
    done |= {p.name for p in (folder / "color_down").glob("trajectory_*")}
    traj = f"trajectory_{next(i for i in range(10000) if f'trajectory_{i:04d}' not in done):04d}"

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C ends the recording, handled below
    rec = Recorder(folder, traj)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    gz = GzWorld(args.world, rec.images)
    world = None if args.free_run else gz
    if world:
        steps = round(1 / CAM_HZ / physics_step(args.world))
        end = time.time() + 30  # Gazebo's transport needs a moment to find the service
        while not world.control(pause=True):
            if time.time() > end:
                raise SystemExit(f"no answer from {world.service}; is the {args.world} world running?")
            time.sleep(0.5)
    end = time.time() + 2  # let the subscriptions connect, so the first readings are not missed
    while time.time() < end:
        rec.spin(0.05)
    rec.armed = True
    try:
        while not stop and rclpy.ok():
            if world:
                if not rec.step_frame(world, steps):
                    rec.get_logger().warning("no camera frame after a step, or the step was refused; stepping again")
            else:
                rec.spin(0.1)
            if args.duration and rec.frames["down"] and max(rec.frames["down"]) >= args.duration * CAM_HZ:
                break
    finally:
        if world:
            world.control(pause=False)  # leave the simulation running
    rec.write(h5_path, world_origin(args.world))
    rec.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

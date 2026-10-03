"""Live sensor dashboard in the terminal.

    docker compose exec sim bash -ic "python3 sim/nodes/sensor_monitor.py [--world terrain|islands]"

Shows what the drone's sensors report, next to the ground truth, refreshed 5 times a second.
Rates are measured in simulation time.
"""
import argparse
import math
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, FluidPressure, Image, Imu, NavSatFix

WORLDS = Path(__file__).resolve().parents[1] / "worlds"
M_PER_DEG_LAT = 111_320.0

BOLD, DIM, GREEN, RED, RESET = "\033[1m", "\033[2m", "\033[32m", "\033[31m", "\033[0m"


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Rate:
    """Messages per second of simulation time, over the last couple of seconds."""

    def __init__(self):
        self.t = deque(maxlen=300)

    def tick(self, t):
        if t is not None:
            self.t.append(t)

    def hz(self):
        return (len(self.t) - 1) / (self.t[-1] - self.t[0]) if len(self.t) > 2 and self.t[-1] > self.t[0] else 0.0


def world_origin(name):
    """Latitude and longitude of the world origin, for turning GPS into metres east/north."""
    sc = ET.parse(WORLDS / f"{name}.sdf").find(".//spherical_coordinates")
    return float(sc.findtext("latitude_deg")), float(sc.findtext("longitude_deg"))


class Monitor(Node):
    def __init__(self, world):
        super().__init__("sensor_monitor")
        self.lat0, self.lon0 = world_origin(world)
        self.sim_t = None
        self.imu = self.baro = self.gps = self.truth = self.info = None
        self.p0 = None
        self.rates = {k: Rate() for k in ("imu", "baro", "gps", "cam", "truth")}
        self.wall0 = self.sim0 = None
        sd = qos_profile_sensor_data
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(Imu, "/imu/data", lambda m: self.keep("imu", m), sd)
        self.create_subscription(FluidPressure, "/air_pressure", self.on_baro, sd)
        self.create_subscription(NavSatFix, "/gps/fix", lambda m: self.keep("gps", m), sd)
        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: self.keep("truth", m), sd)
        self.create_subscription(CameraInfo, "/camera/down/camera_info", lambda m: setattr(self, "info", m), sd)
        # Images are counted without decoding them; Python is too slow to decode every frame
        self.create_subscription(Image, "/camera/down/image_raw", lambda _: self.rates["cam"].tick(self.sim_t), 10, raw=True)
        self.create_timer(0.2, self.draw, clock=rclpy.clock.Clock())  # wall-clock refresh

    def on_clock(self, m):
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.sim0 is None:
            self.wall0, self.sim0 = time.time(), self.sim_t

    def keep(self, name, msg):
        setattr(self, name, msg)
        self.rates[name].tick(stamp(msg))

    def on_baro(self, m):
        if self.p0 is None:
            self.p0 = m.fluid_pressure
        self.keep("baro", m)

    def draw(self):
        out = [f"{BOLD}TaipeiDrift sensor monitor{RESET}   Ctrl+C to quit"]
        if self.sim_t is None:
            out.append("waiting for the simulator (/clock)...")
            print("\033[H\033[J" + "\n".join(out), flush=True)
            return
        rtf = (self.sim_t - self.sim0) / max(1e-6, time.time() - self.wall0)
        out.append(f"sim time {self.sim_t:8.1f} s    real-time factor {rtf:4.2f}\n")
        r = {k: v.hz() for k, v in self.rates.items()}

        tz = None
        if self.truth:
            p, v, q = self.truth.pose.pose.position, self.truth.twist.twist.linear, self.truth.pose.pose.orientation
            roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2)))
            tz = p.z
            out.append(f"{BOLD}GROUND TRUTH{RESET} {DIM}({r['truth']:5.1f} Hz, not a sensor){RESET}")
            out.append(f"  position  E {p.x:8.2f}   N {p.y:8.2f}   U {p.z:7.2f}  m")
            out.append(f"  velocity  fwd {v.x:6.2f}  left {v.y:6.2f}  up {v.z:6.2f}  m/s   speed {math.hypot(v.x, v.y, v.z):5.2f}")
            out.append(f"  attitude  roll {roll:6.1f}  pitch {pitch:6.1f}  yaw {yaw:6.1f}  deg\n")

        if self.imu:
            w, a = self.imu.angular_velocity, self.imu.linear_acceleration
            out.append(f"{BOLD}IMU{RESET} {DIM}({r['imu']:5.1f} Hz, Mid-Air noise model){RESET}")
            out.append(f"  gyro      x {w.x:+8.4f}   y {w.y:+8.4f}   z {w.z:+8.4f}  rad/s")
            out.append(f"  accel     x {a.x:+8.3f}   y {a.y:+8.3f}   z {a.z:+8.3f}  m/s^2\n")

        if self.baro:
            p = self.baro.fluid_pressure
            alt = 44330.0 * (1.0 - (p / self.p0) ** (1 / 5.255))
            err = f"   error {alt - tz:+6.2f} m" if tz is not None else ""
            out.append(f"{BOLD}BAROMETER{RESET} {DIM}(air pressure sensor, {r['baro']:5.1f} Hz){RESET}")
            out.append(f"  pressure  {p:10.1f} Pa    altitude {alt:7.2f} m above start{err}\n")

        out.append(f"{BOLD}GPS{RESET} {DIM}({r['gps']:5.1f} Hz){RESET}")
        if self.gps and r["gps"] > 0 and self.sim_t - stamp(self.gps) < 3:
            e = (self.gps.longitude - self.lon0) * M_PER_DEG_LAT * math.cos(math.radians(self.lat0))
            n = (self.gps.latitude - self.lat0) * M_PER_DEG_LAT
            err = ""
            if self.truth:
                tp = self.truth.pose.pose.position
                err = f"   error {math.hypot(e - tp.x, n - tp.y):5.2f} m"
            out.append(f"  lat {self.gps.latitude:11.6f}  lon {self.gps.longitude:11.6f}  alt {self.gps.altitude:7.1f} m")
            out.append(f"  as metres E {e:8.2f}   N {n:8.2f}{err}\n")
        else:
            out.append(f"  {RED}no fix{RESET} (gps:=false, or no signal)\n")

        out.append(f"{BOLD}DOWN CAMERA{RESET}")
        if self.info:
            ok = GREEN if abs(r["cam"] - 25) < 2.5 else RED
            ground = 2 * tz if tz and tz > 0 else None  # 90 deg FOV: footprint = 2 x height
            fp = f"   footprint {ground:6.1f} m  ({100 * ground / self.info.width:5.1f} cm/px)" if ground else ""
            out.append(f"  {self.info.width}x{self.info.height}  fx {self.info.k[0]:.0f}  "
                       f"{ok}{r['cam']:5.1f} fps{RESET} (want 25){fp}")
        print("\033[H\033[J" + "\n".join(out), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="terrain", help="the world that runs, for the GPS origin")
    args = ap.parse_args()
    rclpy.init()
    try:
        rclpy.spin(Monitor(args.world))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

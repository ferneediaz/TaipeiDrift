"""Fly a short test pattern and report how the drone and its sensors behaved.

Run inside the container while sim.launch.py is running:
    python3 sim/scripts/smoke_flight.py [out_dir]
Climbs to about 30 m, flies forward, hovers, and saves one down-camera frame to out_dir
(default sim/bags/). Steps are timed in simulation time.
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Image
from std_msgs.msg import Bool

# (sim seconds, vx, vz, label)
PLAN = [(2, 0, 0, "arm"), (10, 0, 3, "climb"), (3, 0, 0, "hover"), (8, 5, 0, "forward 5 m/s"), (5, 0, 0, "hover")]


def pressure_altitude(p, p0):
    """Altitude change from pressure, ISA troposphere."""
    return 44330.0 * (1.0 - (p / p0) ** (1 / 5.255))


class Smoke(Node):
    def __init__(self):
        super().__init__("smoke_flight", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.enable = self.create_publisher(Bool, "/enable", 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = self.p0 = self.p = self.image = None
        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: setattr(self, "odom", m), qos_profile_sensor_data)
        self.create_subscription(FluidPressure, "/air_pressure", self.on_baro, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/down/image_raw", lambda m: setattr(self, "image", m), 5)

    def on_baro(self, m):
        self.p = m.fluid_pressure
        if self.p0 is None:
            self.p0 = m.fluid_pressure

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, seconds, vx=0.0, vz=0.0):
        end = self.now_s() + seconds
        while rclpy.ok() and self.now_s() < end:
            self.cmd.publish(Twist(linear=type(Twist().linear)(x=float(vx), z=float(vz))))
            rclpy.spin_once(self, timeout_sec=0.05)

    def report(self, label):
        o = self.odom.pose.pose
        q = o.orientation
        roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2)))
        pitch = math.degrees(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))))
        v = self.odom.twist.twist.linear
        baro = pressure_altitude(self.p, self.p0) if self.p else float("nan")
        print(f"{label:15s} t={self.now_s():6.1f}s  pos=({o.position.x:6.1f}, {o.position.y:6.1f}, {o.position.z:5.1f}) m"
              f"  baro alt={baro:5.1f} m  roll={roll:5.1f} pitch={pitch:5.1f} deg"
              f"  v=({v.x:4.1f}, {v.y:4.1f}, {v.z:4.1f}) m/s", flush=True)


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "bags")
    rclpy.init()
    n = Smoke()
    while n.odom is None or n.p is None or n.now_s() == 0:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.report("start")
    for seconds, vx, vz, label in PLAN:
        if label == "arm":
            for _ in range(5):
                n.enable.publish(Bool(data=True))
                rclpy.spin_once(n, timeout_sec=0.05)
        n.wait(seconds, vx, vz)
        n.report(label)
    if n.image is not None:
        img = np.frombuffer(n.image.data, np.uint8).reshape(n.image.height, n.image.width, -1)
        out.mkdir(parents=True, exist_ok=True)
        path = out / "smoke_down_camera.png"
        cv2.imwrite(str(path), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        print(f"saved {path} ({n.image.width}x{n.image.height}, {n.image.encoding})")


if __name__ == "__main__":
    main()

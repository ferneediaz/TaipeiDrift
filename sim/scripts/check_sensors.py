"""Check that the simulator publishes every sensor at the right rate, with Mid-Air intrinsics.

Run inside the container while sim.launch.py is running:
    python3 sim/scripts/check_sensors.py [seconds]
Rates are counted over simulation time, so they hold even when Gazebo runs slower than real time.
"""
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, FluidPressure, Image, Imu, NavSatFix

EXPECTED = {  # topic: (type, Hz)
    "/imu/data": (Imu, 100),
    "/air_pressure": (FluidPressure, 50),
    "/gps/fix": (NavSatFix, 1),
    "/ground_truth/odom": (Odometry, 100),
    "/camera/down/image_raw": (Image, 25),
    "/camera/down/camera_info": (CameraInfo, 25),
}


class Check(Node):
    def __init__(self):
        super().__init__("check_sensors")
        self.stamps = {t: [] for t in EXPECTED}
        self.last = {}
        self.sim_now = None
        self.raw_images = []  # sim time at arrival; images are counted raw, Python is too slow to decode them all
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        for topic, (msg_type, _) in EXPECTED.items():
            if msg_type is Image:
                self.create_subscription(msg_type, topic, lambda _: self.raw_images.append(self.sim_now), 50, raw=True)
            else:
                self.create_subscription(msg_type, topic, lambda m, t=topic: self.on_msg(t, m), qos_profile_sensor_data)

    def on_clock(self, msg):
        self.sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_msg(self, topic, msg):
        if hasattr(msg, "header"):
            self.stamps[topic].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.last[topic] = msg


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 15
    rclpy.init()
    node = Check()
    wall0 = time.time()
    sim0 = None
    while time.time() - wall0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
        if sim0 is None and node.sim_now is not None:
            sim0 = node.sim_now
    sim_span = (node.sim_now or 0) - (sim0 or 0)
    print(f"wall {seconds:.0f} s, sim {sim_span:.1f} s, real-time factor {sim_span / seconds:.2f}")

    ok = True
    for topic, (_, hz) in EXPECTED.items():
        s = node.stamps[topic] if EXPECTED[topic][0] is not Image else [t for t in node.raw_images if t is not None]
        rate = (len(s) - 1) / (s[-1] - s[0]) if len(s) > 1 and s[-1] > s[0] else 0.0
        good = abs(rate - hz) <= 0.1 * hz
        ok &= good
        frame = node.last[topic].header.frame_id if topic in node.last else "(raw)"
        print(f"{'OK ' if good else 'BAD'} {topic:28s} {rate:6.1f} Hz (want {hz})  frame '{frame}'")

    info = node.last.get("/camera/down/camera_info")
    if info:
        fx, cx, fy, cy = info.k[0], info.k[2], info.k[4], info.k[5]
        want = info.width / 2
        good = all(abs(v - want) < 1 for v in (fx, cx, fy, cy))
        ok &= good
        print(f"{'OK ' if good else 'BAD'} intrinsics {info.width}x{info.height} fx={fx:.1f} fy={fy:.1f} "
              f"cx={cx:.1f} cy={cy:.1f} (Mid-Air: all = width/2 = {want:.0f})")
    gps = node.last.get("/gps/fix")
    if gps:
        print(f"    gps lat {gps.latitude:.6f} lon {gps.longitude:.6f} alt {gps.altitude:.1f}")
    baro = node.last.get("/air_pressure")
    if baro:
        print(f"    air pressure {baro.fluid_pressure:.1f} Pa")
    print("ALL OK" if ok else "SOME CHECKS FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

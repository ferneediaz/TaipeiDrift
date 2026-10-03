"""Fly the drone in a continuous demo pattern: take off, then circle at constant height. In the islands world it
instead flies from the helipad on island A across the open sea to island B and back, again and again.

Started by `sim.launch.py demo:=true`, or by hand while the simulator runs:
    python3 sim/nodes/demo_flight.py [--world terrain|islands]
It also points the Gazebo window's camera at the drone; `--camera-only` does only that. Height is held from the ground
truth; this stands in for an autopilot and is not part of any navigation method.
"""
import argparse
import json
import math
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

HEIGHT = 40.0     # m above the ground
SPEED = 6.0       # m/s forward
YAW_RATE = 0.15   # rad/s: a circle of SPEED / YAW_RATE = 40 m radius
ROUTE_SPEED = 8.0  # m/s between the islands
LAYOUT = Path(__file__).resolve().parents[1] / "models" / "{world}" / "layout.json"


def follow_in_gui(timeout_s=180):
    """Chase-camera view of the drone in the Gazebo window, once the window is up (it can take a minute)."""
    end = time.time() + timeout_s
    while time.time() < end:
        topics = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True).stdout
        if "/gui/track" in topics:
            break
        time.sleep(2)
    else:
        return
    # Camera 1.2 m behind, 0.4 m to the side and 0.5 m above the drone, looking at it.
    # The gains make it keep up at flight speed; Gazebo's default lags far behind.
    # Sent repeatedly for two minutes: the window ignores it until its scene has loaded the drone.
    for _ in range(24):
        subprocess.run(["gz", "topic", "-t", "/gui/track", "-m", "gz.msgs.CameraTrack", "-p",
                        'track_mode: FOLLOW_LOOK_AT, follow_target: {name: "midair_quad"}, '
                        'track_target: {name: "midair_quad"}, follow_offset: {x: -1.2, y: -0.4, z: 0.5}, '
                        "follow_pgain: 0.8, track_pgain: 0.8"], capture_output=True)
        time.sleep(5)


def route_for(world):
    """Waypoints (x, y) to fly between, or None to fly circles. The islands world lists its helipads in layout.json."""
    layout = Path(str(LAYOUT).format(world=world))
    if not layout.exists():
        return None
    pads = json.loads(layout.read_text())["pads"]
    return [tuple(pads[k]) for k in sorted(pads)[1:] + sorted(pads)[:1]]  # b, then back to a


class Demo(Node):
    def __init__(self, route=None):
        super().__init__("demo_flight", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.enable = self.create_publisher(Bool, "/enable", 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.z = self.xy = self.yaw = None
        self.route, self.leg = route, 0
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.05, self.step)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.z, self.xy = p.z, (p.x, p.y)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    def step(self):
        if self.z is None:
            return
        self.enable.publish(Bool(data=True))
        cmd = Twist()
        cmd.linear.z = max(-2.0, min(3.0, 0.8 * (HEIGHT - self.z)))
        if self.z > HEIGHT - 3 and self.route:  # fly to the next waypoint once near the target height
            tx, ty = self.route[self.leg]
            dist = math.hypot(tx - self.xy[0], ty - self.xy[1])
            if dist < 15:
                self.leg = (self.leg + 1) % len(self.route)
                self.get_logger().info(f"waypoint reached, next {self.route[self.leg]}")
            err = math.atan2(ty - self.xy[1], tx - self.xy[0]) - self.yaw
            err = math.atan2(math.sin(err), math.cos(err))
            cmd.angular.z = max(-0.5, min(0.5, 1.2 * err))  # turn towards it, then fly
            cmd.linear.x = min(ROUTE_SPEED, 0.4 * dist + 1.0) * max(0.0, math.cos(err)) ** 4
        elif self.z > HEIGHT - 3:  # circle once near the target height
            cmd.linear.x = SPEED
            cmd.angular.z = YAW_RATE
        self.cmd.publish(cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="terrain", help="the world that runs; islands flies between the islands")
    ap.add_argument("--camera-only", action="store_true", help="only point the Gazebo window at the drone")
    args = ap.parse_args()
    if args.camera_only:
        follow_in_gui()  # about two minutes
        return
    rclpy.init()
    threading.Thread(target=follow_in_gui, daemon=True).start()
    try:
        rclpy.spin(Demo(route_for(args.world)))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

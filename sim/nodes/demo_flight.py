"""Fly the drone in a continuous demo pattern: take off, then circle at constant height. In the islands world, and
the strait world that shares its scenery, it instead flies from the helipad on island A across the open sea to
island B and back, again and again.
With --route survey it flies back-and-forth lines over each island in turn, so the down camera sees all of them.

Started by `sim.launch.py demo:=true`, or by hand while the simulator runs:
    python3 sim/nodes/demo_flight.py [--world terrain|islands|strait] [--route pads|crossing|survey] [--spacing M] [--height M] [--once] [--land] [--from-waypoint N]

--once flies the route a single time, then hovers; record_islands_set.sh uses it to end each recorded flight.
--route crossing flies to the helipad of island B only, and --land then settles onto it (the demo flight).
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

import numpy as np
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
SCENERY = {"strait": "islands"}  # worlds that reuse another world's scenery


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


def route_for(world, kind="pads", spacing=60.0):
    """Waypoints (x, y) to fly between, or None to fly circles. The islands world lists its helipads in layout.json.

    pads: to the helipad on island B and back. crossing: to the helipad on island B only. survey: back-and-forth
    lines `spacing` m apart over each island, then back to the start pad.
    """
    layout = Path(str(LAYOUT).format(world=SCENERY.get(world, world)))
    if not layout.exists():
        if world == "city":
            # A varied loop through the city, across several intersections, then back to the plaza.
            return [(175.0, 0.0), (175.0, 150.0), (70.0, 150.0), (70.0, -150.0),
                    (-70.0, -150.0), (-70.0, 70.0), (0.0, 70.0), (0.0, 0.0)]
        return None
    layout = json.loads(layout.read_text())
    pads = layout["pads"]
    if kind == "pads":
        return [tuple(pads[k]) for k in sorted(pads)[1:] + sorted(pads)[:1]]  # b, then back to a
    if kind == "crossing":
        return [tuple(pads[k]) for k in sorted(pads)[1:]]  # b only
    route = []
    for name in sorted(layout["islands"]):
        (cx, cy), r = layout["islands"][name]["centre"], 1.3 * layout["islands"][name]["radius"]  # coast within 1.3 r
        ys = np.arange(cy - r, cy + r + 1e-6, spacing)
        for i, y in enumerate(ys):
            route += [(cx - r, y), (cx + r, y)][::1 if i % 2 == 0 else -1]
    return route + [tuple(pads[sorted(pads)[0]])]


class Demo(Node):
    def __init__(self, route=None, height=HEIGHT, once=False, start=0, land=False):
        super().__init__("demo_flight", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.enable = self.create_publisher(Bool, "/enable", 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.z = self.xy = self.yaw = None
        self.route, self.leg = route, start
        self.height, self.once, self.at_height, self.done = height, once, False, False
        self.land, self.landed = land, False
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
        cmd.linear.z = max(-2.0, min(3.0, 0.8 * (self.height - self.z)))
        if self.z > self.height - 3 and not self.at_height:
            self.at_height = True
            self.get_logger().info(f"at height {self.height:.0f} m")
        if self.done and self.land:  # --once --land: come down onto the last waypoint, centring on the way
            if self.landed:
                return
            tx, ty = self.route[-1]
            dist = math.hypot(tx - self.xy[0], ty - self.xy[1])
            err = math.atan2(ty - self.xy[1], tx - self.xy[0]) - self.yaw
            err = math.atan2(math.sin(err), math.cos(err))
            cmd.angular.z = max(-0.5, min(0.5, 1.2 * err))
            cmd.linear.x = min(2.0, 0.5 * dist) * max(0.0, math.cos(err)) ** 4
            # down at up to 2.5 m/s, slower near the ground; below 6 m only once over the pad (8 m wide)
            cmd.linear.z = -max(0.4, min(2.5, 0.4 * self.z)) if dist < 3.0 or self.z > 6.0 else 0.0
            if self.z < 0.2 and dist < 4.0:
                self.landed = True
                self.get_logger().info(f"landed, {dist:.1f} m from the pad centre")
                self.cmd.publish(Twist())
                self.enable.publish(Bool(data=False))
                return
        elif self.done:  # --once: the route is flown, hover
            pass
        elif self.z > self.height - 3 and self.route:  # fly to the next waypoint once near the target height
            tx, ty = self.route[self.leg]
            dist = math.hypot(tx - self.xy[0], ty - self.xy[1])
            if dist < 15:
                if self.once and self.leg == len(self.route) - 1:
                    self.done = True
                    self.get_logger().info("route done")
                    self.cmd.publish(Twist())
                    return
                self.leg = (self.leg + 1) % len(self.route)
                self.get_logger().info(f"waypoint reached, next {self.route[self.leg]}")
            err = math.atan2(ty - self.xy[1], tx - self.xy[0]) - self.yaw
            err = math.atan2(math.sin(err), math.cos(err))
            cmd.angular.z = max(-0.5, min(0.5, 1.2 * err))  # turn towards it, then fly
            cmd.linear.x = min(ROUTE_SPEED, 0.4 * dist + 1.0) * max(0.0, math.cos(err)) ** 4
        elif self.z > self.height - 3:  # circle once near the target height
            cmd.linear.x = SPEED
            cmd.angular.z = YAW_RATE
        self.cmd.publish(cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="terrain", help="the world that runs; islands and strait fly between the islands")
    ap.add_argument("--camera-only", action="store_true", help="only point the Gazebo window at the drone")
    ap.add_argument("--route", choices=["pads", "crossing", "survey", "city_loop"], default="pads",
                    help="islands and strait: pads/crossing/survey; city: city_loop")
    ap.add_argument("--spacing", type=float, default=60.0, help="m between survey lines")
    ap.add_argument("--height", type=float, default=HEIGHT, help="flight height above the start point, m")
    ap.add_argument("--once", action="store_true", help="fly the route once, then hover")
    ap.add_argument("--land", action="store_true", help="with --once: settle onto the last waypoint (a helipad)")
    ap.add_argument("--from-waypoint", type=int, default=0,
                    help="start the route at this waypoint (0 is the first), to continue an interrupted flight")
    args = ap.parse_args()
    if args.camera_only:
        follow_in_gui()  # about two minutes
        return
    rclpy.init()
    threading.Thread(target=follow_in_gui, daemon=True).start()
    try:
        rclpy.spin(Demo(route_for(args.world, args.route, args.spacing), args.height, args.once or args.land,
                        args.from_waypoint, args.land))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

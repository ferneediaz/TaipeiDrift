"""Fly a planned route with the simulator's velocity commands: climb, then follow the waypoints.

The route comes from sim/scripts/plan_route.py (x east, y north, metres from the world origin). The
drone steers by the simulator's true position, the way an autopilot with working GNSS would fly a
plan; the navigator under test never sees that truth, only the recorded sensors.

Steering: the drone looks at a point 40 m ahead on the route and turns its nose towards it (the
down camera's image top points forward). It slows down while its nose is far off that direction,
so the turn at the north end is flown on the spot.

Run in the container, with the simulator and the recorder already running:
    python3 sim/scripts/route_flight.py [--route sim/scenarios/wufeng_corridor.json]
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

LOOKAHEAD_M = 40.0
WINDOW_M = 80.0  # the nearest route point is searched only this far ahead of the progress so far
MAX_YAW_RATE = 0.5  # rad/s


class RouteFlight(Node):
    def __init__(self, route: np.ndarray, altitude: float, speed: float):
        super().__init__("route_flight", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.enable = self.create_publisher(Bool, "/enable", 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = None
        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: setattr(self, "odom", m), qos_profile_sensor_data)
        self.route, self.altitude, self.speed = route, altitude, speed
        self.along = np.r_[0.0, np.cumsum(np.hypot(*np.diff(route, axis=0).T))]
        self.progress = 0.0

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self) -> tuple[float, float, float, float]:
        p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        return p.x, p.y, p.z, yaw

    def send(self, vx=0.0, vz=0.0, yaw_rate=0.0) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.z, msg.angular.z = float(vx), float(vz), float(yaw_rate)
        self.cmd.publish(msg)

    def hold_height(self, z: float) -> float:
        return max(-2.0, min(3.0, 0.8 * (self.altitude - z)))

    def point_at(self, s: float) -> np.ndarray:
        s = min(max(s, 0.0), self.along[-1])
        return np.array([np.interp(s, self.along, self.route[:, 0]), np.interp(s, self.along, self.route[:, 1])])

    def step(self) -> bool:
        """One control step. Returns False at the end of the route."""
        x, y, z, yaw = self.pose()
        ahead = (self.along >= self.progress) & (self.along <= self.progress + WINDOW_M)
        if ahead.any():
            idx = np.where(ahead)[0]
            near = idx[np.argmin(np.hypot(self.route[idx, 0] - x, self.route[idx, 1] - y))]
            self.progress = max(self.progress, float(self.along[near]))
        if self.progress >= self.along[-1] - 5.0:
            return False
        target = self.point_at(self.progress + LOOKAHEAD_M)
        wanted = math.atan2(target[1] - y, target[0] - x)
        error = math.atan2(math.sin(wanted - yaw), math.cos(wanted - yaw))
        vx = self.speed * max(0.15, math.cos(error))
        self.send(vx, self.hold_height(z), max(-MAX_YAW_RATE, min(MAX_YAW_RATE, 1.2 * error)))
        return True

    def report(self, label: str) -> None:
        x, y, z, yaw = self.pose()
        print(f"{label}: sim_t={self.now_s():.1f} s, position ({x:.1f}, {y:.1f}, {z:.1f}) m, "
              f"heading {math.degrees(yaw):.0f} deg, {self.progress:.0f} of {self.along[-1]:.0f} m", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--route", default=str(Path(__file__).resolve().parents[1] / "scenarios" / "wufeng_corridor.json"))
    args = ap.parse_args()
    plan = json.loads(Path(args.route).read_text())
    route = np.array(plan["waypoints_xy_m"], float)

    rclpy.init()
    node = RouteFlight(route, plan["altitude_m"], plan["speed_mps"])
    try:
        while rclpy.ok() and (node.odom is None or node.now_s() == 0):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.report("start")
        for _ in range(10):
            node.enable.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.05)

        while rclpy.ok() and node.pose()[2] < node.altitude - 2.0:  # climb on the spot
            node.send(vz=node.hold_height(node.pose()[2]))
            rclpy.spin_once(node, timeout_sec=0.05)
        node.report("at flight height")

        last = node.now_s()
        while rclpy.ok() and node.step():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.now_s() - last >= 20.0:
                node.report("flying")
                last = node.now_s()
        node.report("route complete")
        end = node.now_s() + 5.0
        while rclpy.ok() and node.now_s() < end:  # hover
            node.send(vz=node.hold_height(node.pose()[2]))
            rclpy.spin_once(node, timeout_sec=0.05)
        node.report("done")
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # report and stop the drone
        print(f"route flight failed: {exc}", file=sys.stderr, flush=True)
        raise
    finally:
        node.send()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

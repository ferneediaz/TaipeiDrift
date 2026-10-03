"""Sail the ships of config/rf.yaml along their waypoints and publish where they are.

Started by `sim.launch.py` in the strait world (or with ships:=true), or by hand while the simulator runs:
    python3 sim/nodes/ship_traffic.py --world strait [--config sim/config/rf.yaml]

Ships are kinematic: their pose is a function of simulation time (constant speed around the waypoint loop) and is
written into Gazebo through the bridged /world/<world>/set_pose service at 10 Hz. The same pose is published as
ground truth on /ships/<name>/odom; sim/nodes/rf_sensor.py transmits from there.
"""
import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

CONFIG = Path(__file__).resolve().parents[1] / "config" / "rf.yaml"
KNOT = 1852.0 / 3600.0
RATE_HZ = 10.0


class Route:
    """Constant speed around a closed loop of waypoints, starting at the first."""

    def __init__(self, waypoints, speed_mps):
        self.p = np.asarray(waypoints, float)
        seg = np.roll(self.p, -1, axis=0) - self.p
        self.len = np.hypot(seg[:, 0], seg[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.len)])
        self.seg = seg
        self.speed = speed_mps

    def at(self, t):
        """(x, y, heading) at time t."""
        s = (self.speed * t) % self.cum[-1]
        i = min(int(np.searchsorted(self.cum, s, side="right")) - 1, len(self.len) - 1)
        f = (s - self.cum[i]) / self.len[i]
        x, y = self.p[i] + f * self.seg[i]
        return x, y, math.atan2(self.seg[i][1], self.seg[i][0])


def yaw_quaternion(yaw):
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class ShipTraffic(Node):
    def __init__(self, world, cfg):
        super().__init__("ship_traffic", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.z = cfg["sea_level_z"]
        self.ships = [(s["name"], Route(s["waypoints"], s["speed_kn"] * KNOT)) for s in cfg["ships"]]
        self.pubs = {name: self.create_publisher(Odometry, f"/ships/{name}/odom", qos_profile_sensor_data)
                     for name, _ in self.ships}
        self.set_pose = self.create_client(SetEntityPose, f"/world/{world}/set_pose")
        self.pending = {}
        self.create_timer(1.0 / RATE_HZ, self.step)

    def step(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if t <= 0:  # no /clock yet
            return
        for name, route in self.ships:
            x, y, yaw = route.at(t)
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = "world"
            odom.child_frame_id = name
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = x, y, self.z
            odom.pose.pose.orientation = yaw_quaternion(yaw)
            odom.twist.twist.linear.x = route.speed
            self.pubs[name].publish(odom)

            # Gazebo is only told when the previous move is done, so requests never pile up
            if self.set_pose.service_is_ready() and (name not in self.pending or self.pending[name].done()):
                req = SetEntityPose.Request()
                req.entity = Entity(name=name, type=Entity.MODEL)
                req.pose = odom.pose.pose
                self.pending[name] = self.set_pose.call_async(req)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="strait")
    ap.add_argument("--config", default=str(CONFIG))
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    try:
        rclpy.spin(ShipTraffic(args.world, yaml.safe_load(open(args.config))))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

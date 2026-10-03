"""Fly a 60 m GNSS-cut scenario using the simulator's velocity-command interface."""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


class Scenario(Node):
    def __init__(self):
        super().__init__(
            "t_scenario",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.enable = self.create_publisher(Bool, "/enable", 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = None
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        self.odom = msg

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def yaw_rad(self):
        q = self.odom.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    def command(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        self.cmd.publish(msg)

    def wait_sim(self, seconds, vx=0.0, vy=0.0, altitude_m=None, target_yaw=None):
        end = self.now_s() + seconds
        while rclpy.ok() and self.now_s() < end:
            vz = 0.0
            yaw_rate = 0.0
            if self.odom is not None:
                if altitude_m is not None:
                    error_z = altitude_m - self.odom.pose.pose.position.z
                    vz = max(-2.0, min(3.0, 0.8 * error_z))
                if target_yaw is not None:
                    yaw_error = target_yaw - self.yaw_rad()
                    error_yaw = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
                    yaw_rate = max(-0.5, min(0.5, 1.2 * error_yaw))
            self.command(vx, vy, vz, yaw_rate)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.command()

    def report(self, label):
        p = self.odom.pose.pose.position
        yaw = math.degrees(self.yaw_rad())
        print(f"{label}: sim_t={self.now_s():.1f}s position=({p.x:.1f}, {p.y:.1f}, {p.z:.1f}) m yaw={yaw:.1f} deg", flush=True)

    def climb_to(self, target_m=60.0, max_s=45.0):
        start = self.now_s()
        deadline = start + max_s
        while rclpy.ok() and self.odom is not None and self.odom.pose.pose.position.z < target_m and self.now_s() < deadline:
            self.command(vz=3.0)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.command()
        if self.odom is None or self.odom.pose.pose.position.z < target_m:
            altitude = -1.0 if self.odom is None else self.odom.pose.pose.position.z
            raise RuntimeError(f"failed to reach {target_m:.1f} m within {max_s:.1f} s (z={altitude:.1f} m)")


def main():
    rclpy.init()
    node = Scenario()
    try:
        while rclpy.ok() and (node.odom is None or node.now_s() == 0):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.report("start")
        for _ in range(10):
            node.enable.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.05)

        node.climb_to()
        node.report("climb complete")
        node.wait_sim(5.0, altitude_m=60.0)
        node.report("hover at 60 m")

        first_heading = node.yaw_rad()
        node.wait_sim(120.0, vx=8.0, altitude_m=60.0, target_yaw=first_heading)
        node.report("first 8 m/s leg complete")
        second_heading = math.atan2(math.sin(first_heading + math.pi / 2), math.cos(first_heading + math.pi / 2))
        node.wait_sim(8.0, altitude_m=60.0, target_yaw=second_heading)
        node.report("90 degree turn complete")
        node.wait_sim(2.0, altitude_m=60.0, target_yaw=second_heading)
        node.wait_sim(60.0, vx=8.0, altitude_m=60.0, target_yaw=second_heading)
        node.report("second 8 m/s leg complete")
        node.wait_sim(30.0, altitude_m=60.0, target_yaw=second_heading)
        node.report("final hover")
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"scenario failed: {exc}", file=sys.stderr, flush=True)
        raise
    finally:
        node.command()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

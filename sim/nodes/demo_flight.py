"""Fly the drone in a continuous demo pattern: take off, then circle at constant height.

Started by `sim.launch.py demo:=true`, or by hand while the simulator runs:
    python3 sim/nodes/demo_flight.py
It also points the Gazebo window's camera at the drone; `--camera-only` does only that. Height is held from the ground
truth; this stands in for an autopilot and is not part of any navigation method.
"""
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

HEIGHT = 40.0     # m above the ground
SPEED = 6.0       # m/s forward
YAW_RATE = 0.15   # rad/s: a circle of SPEED / YAW_RATE = 40 m radius


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


class Demo(Node):
    def __init__(self):
        super().__init__("demo_flight", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.enable = self.create_publisher(Bool, "/enable", 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.z = None
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.05, self.step)

    def on_odom(self, m):
        self.z = m.pose.pose.position.z

    def step(self):
        if self.z is None:
            return
        self.enable.publish(Bool(data=True))
        cmd = Twist()
        cmd.linear.z = max(-2.0, min(3.0, 0.8 * (HEIGHT - self.z)))
        if self.z > HEIGHT - 3:  # circle once near the target height
            cmd.linear.x = SPEED
            cmd.angular.z = YAW_RATE
        self.cmd.publish(cmd)


def main():
    if "--camera-only" in sys.argv:
        follow_in_gui()  # about two minutes
        return
    rclpy.init()
    threading.Thread(target=follow_in_gui, daemon=True).start()
    try:
        rclpy.spin(Demo())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

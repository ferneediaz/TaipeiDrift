"""Republish simulated GNSS until the configured simulation-time cutoff."""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


def stamp_s(msg: NavSatFix) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class GnssGate(Node):
    def __init__(self):
        super().__init__(
            "gnss_gate",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.cut_s = float(self.declare_parameter("gnss_cut_s", -1.0).value)
        self.cut_logged = False
        self.publisher = self.create_publisher(NavSatFix, "/gps/fix", qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/sim/gps_raw", self.on_fix, qos_profile_sensor_data)
        if self.cut_s < 0:
            self.get_logger().info("GNSS gate open; no cutoff configured")

    def on_fix(self, msg: NavSatFix) -> None:
        if self.cut_s < 0 or stamp_s(msg) < self.cut_s:
            self.publisher.publish(msg)
        elif not self.cut_logged:
            self.get_logger().info(
                f"GNSS cut reached at sim t={stamp_s(msg):.3f} s; cutoff={self.cut_s:.3f} s; dropping fixes"
            )
            self.cut_logged = True


def main():
    rclpy.init()
    node = GnssGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

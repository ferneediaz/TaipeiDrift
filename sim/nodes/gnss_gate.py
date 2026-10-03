"""Republish simulated GNSS until the configured simulation-time cutoff."""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from gnss_policy import gnss_available


def stamp_s(msg: NavSatFix) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class GnssGate(Node):
    def __init__(self):
        super().__init__(
            "gnss_gate",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.cut_s = float(self.declare_parameter("gnss_cut_s", -1.0).value)
        self.start_s = None
        self.cut_logged = False
        self.publisher = self.create_publisher(NavSatFix, "/gps/fix", qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(Bool, "/nav/gnss_available", latched)
        self.available = None
        self.create_subscription(NavSatFix, "/sim/gps_raw", self.on_fix, qos_profile_sensor_data)
        self.create_timer(0.25, self.publish_status)
        if self.cut_s < 0:
            self.get_logger().info("GNSS gate open; no cutoff configured")

    def publish_status(self):
        if self.start_s is not None:
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.start_s
            self.set_available(gnss_available(self.cut_s, elapsed))

    def set_available(self, available):
        if self.available != available:
            self.available = available
            self.status_pub.publish(Bool(data=available))

    def on_fix(self, msg: NavSatFix) -> None:
        t = stamp_s(msg)
        if self.start_s is None:
            self.start_s = t
            self.get_logger().info(f"GNSS experiment clock started at sim t={t:.3f} s")
        elapsed = t - self.start_s
        available = gnss_available(self.cut_s, elapsed)
        self.set_available(available)
        if available:
            self.publisher.publish(msg)
        elif not self.cut_logged:
            self.get_logger().info(
                f"GNSS cut reached at sim t={t:.3f} s (experiment +{elapsed:.3f} s); "
                f"cutoff={self.cut_s:.3f} s; dropping fixes"
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

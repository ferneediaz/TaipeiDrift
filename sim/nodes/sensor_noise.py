"""Add the Mid-Air sensor noise to the noise-free Gazebo IMU, and drift to the air pressure.

IMU, per axis (Mid-Air paper, eq. 1):
    m = m_true + N(0, n) + b_t,   b_t = b_{t-1} + N(0, b0 * sqrt(dt / t_a)),   b_0 ~ N(0, b0)
n, b0 and t_a are drawn per axis at startup from the bounds in config/sensor_noise.yaml,
as Mid-Air draws them per flight. The draw and the initial bias are logged and published
once on /imu/params (latched), as Mid-Air logs its initial bias.

/sim/imu_raw          -> /imu/data
/sim/air_pressure_raw -> /air_pressure   (Gazebo Air Pressure sensor = the barometer)
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import String


class AxisModel:
    """Mid-Air eq. 1 for three axes."""

    def __init__(self, rng: np.random.Generator, n, b0, ta, scale: float):
        self.rng = rng
        self.n = rng.uniform(*n, 3) * scale
        self.b0 = rng.uniform(*b0, 3) * scale
        self.ta = rng.uniform(*ta, 3)
        self.bias = rng.normal(0.0, self.b0)
        self.initial_bias = self.bias.copy()

    def apply(self, true: np.ndarray, dt: float) -> np.ndarray:
        if dt > 0:
            self.bias = self.bias + self.rng.normal(0.0, self.b0 * np.sqrt(dt / self.ta))
        return true + self.rng.normal(0.0, self.n) + self.bias

    def describe(self) -> dict:
        return {k: getattr(self, k).tolist() for k in ("n", "b0", "ta", "initial_bias")}


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class SensorNoise(Node):
    def __init__(self):
        super().__init__("sensor_noise")
        p = lambda name, default: self.declare_parameter(name, default).value
        seed = p("seed", -1)
        scale = p("scale", 1.0)
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.gyro = AxisModel(self.rng, p("gyro_n", [0.0005, 0.005]), p("gyro_b0", [0.001, 0.01]),
                              p("gyro_ta", [100.0, 1000.0]), scale)
        self.accel = AxisModel(self.rng, p("accel_n", [0.005, 0.05]), p("accel_b0", [0.01, 0.1]),
                               p("accel_ta", [100.0, 1000.0]), scale)
        self.baro_rate = p("baro_drift_pa_per_sqrt_s", 0.65) * scale
        self.baro_drift = 0.0
        self.last_imu = None
        self.last_baro = None

        params = {"seed": seed, "scale": scale, "gyro": self.gyro.describe(), "accel": self.accel.describe(),
                  "baro_drift_pa_per_sqrt_s": self.baro_rate}
        self.get_logger().info(f"IMU noise draw: {json.dumps(params)}")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(String, "/imu/params", latched).publish(String(data=json.dumps(params)))

        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        self.baro_pub = self.create_publisher(FluidPressure, "/air_pressure", qos_profile_sensor_data)
        self.create_subscription(Imu, "/sim/imu_raw", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(FluidPressure, "/sim/air_pressure_raw", self.on_baro, qos_profile_sensor_data)

    def on_imu(self, msg: Imu):
        t = stamp_s(msg)
        dt = 0.0 if self.last_imu is None else t - self.last_imu
        self.last_imu = t
        w, a = msg.angular_velocity, msg.linear_acceleration
        w.x, w.y, w.z = self.gyro.apply(np.array([w.x, w.y, w.z]), dt)
        a.x, a.y, a.z = self.accel.apply(np.array([a.x, a.y, a.z]), dt)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        # The filter must not see the true orientation that Gazebo fills in.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance = list(np.diag(self.gyro.n**2).ravel())
        msg.linear_acceleration_covariance = list(np.diag(self.accel.n**2).ravel())
        msg.header.frame_id = "sensor_link"
        self.imu_pub.publish(msg)

    def on_baro(self, msg: FluidPressure):
        t = stamp_s(msg)
        if self.last_baro is not None and t > self.last_baro:
            self.baro_drift += self.rng.normal(0.0, self.baro_rate * np.sqrt(t - self.last_baro))
        self.last_baro = t
        msg.fluid_pressure += self.baro_drift
        msg.header.frame_id = "sensor_link"
        self.baro_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SensorNoise())


if __name__ == "__main__":
    main()

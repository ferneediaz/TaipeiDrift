"""Where is the drone, without GNSS? Triangulation (resection) from the AIS ships it hears.

Started by `sim.launch.py` with the ships (the strait world), or by hand while the simulator runs:
    python3 sim/nodes/rf_nav.py --world strait

Every decoded AIS packet on /rf/detections gives a ship's reported position and the bearing to it in the drone's
body frame. A bearing in the body frame depends on the drone's position and its heading, and without GNSS the
heading is unknown too, so three ships are needed for a fix: three bearings, three unknowns (x, y, heading).

1. Start: once three different ships have been heard within START_WINDOW_S, solve x, y and heading from their
   bearings by least squares (resection), searching a grid first so it does not settle on a wrong solution.
2. Then an extended Kalman filter, state [x, y, vx, vy, heading, gyro_bias] in the world frame (ENU):
   predict with the noisy gyro's yaw rate (/imu/data) and a constant-velocity model, correct with each bearing.
   Ships transmit every 6-10 s, so a bearing arrives every few seconds.

Only drone sensors and decoded AIS data are used, never ground truth.

/rf_nav/odom  nav_msgs/Odometry: position (z is not estimated, 0), heading as orientation, covariance;
              twist.linear is the velocity in the world frame (ENU), not the body frame.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import rf_model as rf

WORLDS = Path(__file__).resolve().parents[1] / "worlds"
START_WINDOW_S = 12.0     # the first fix uses one bearing per ship from this long a window
START_SIGMA_M = 40.0      # the drone moves during that window: position uncertainty of the first fix, at least
ACCEL_SIGMA = 0.8         # m/s^2, white-noise acceleration of the constant-velocity model (the demo turns at 0.5)
GYRO_BIAS_WALK = 1e-4     # rad/s per sqrt(s)
BEARING_BIAS_DEG = 1.0    # the direction finder's mounting error is fixed, not white: widen each bearing's sigma
GATE = 16.0               # reject a bearing whose squared normalised innovation exceeds this (4 sigma)
SHIP_POSITION_SIGMA_M = 3.0  # error of the position a ship reports (its own GNSS), as assumed by the navigator
X, Y, VX, VY, PSI, BG = range(6)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def resection(ships, bearings, sigmas):
    """x, y, heading and their covariance from body-frame bearings to known positions (ships: N x 2)."""
    w = 1 / np.asarray(sigmas) ** 2

    def heading_at(px, py):
        """For a candidate position, the heading that best fits: circular mean of world minus body bearings."""
        d = np.arctan2(ships[:, 1, None] - py, ships[:, 0, None] - px) - bearings[:, None]
        return np.arctan2((w[:, None] * np.sin(d)).sum(0), (w[:, None] * np.cos(d)).sum(0))

    def residuals(px, py, psi):
        return np.angle(np.exp(1j * (np.arctan2(ships[:, 1, None] - py, ships[:, 0, None] - px)
                                     - psi - bearings[:, None])))

    # coarse grid over the ships' area, then Gauss-Newton from the best point
    lo, hi = ships.min(0) - 3000, ships.max(0) + 3000
    gx, gy = np.meshgrid(np.arange(lo[0], hi[0], 25.0), np.arange(lo[1], hi[1], 25.0))
    px, py = gx.ravel(), gy.ravel()
    cost = (w[:, None] * residuals(px, py, heading_at(px, py)) ** 2).sum(0)
    i = int(np.argmin(cost))
    s = np.array([px[i], py[i], heading_at(px[i:i + 1], py[i:i + 1])[0]])
    for _ in range(20):
        dx, dy = ships[:, 0] - s[0], ships[:, 1] - s[1]
        r2 = dx * dx + dy * dy
        res = residuals(s[0:1], s[1:2], s[2])[:, 0]
        jac = np.column_stack([dy / r2, -dx / r2, -np.ones(len(ships))])
        info = jac.T @ (w[:, None] * jac)
        step = np.linalg.solve(info, jac.T @ (w * -res))
        s += step
        if np.abs(step[:2]).max() < 0.01:
            break
    return s, np.linalg.inv(info)


class RfNav(Node):
    def __init__(self, world):
        super().__init__("rf_nav", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.origin = rf.world_origin(WORLDS / f"{world}.sdf")
        self.x = None          # state, None until the first fix
        self.P = None
        self.t = None          # time of the state
        self.gyro_z = 0.0      # last yaw rate (body z, rad/s)
        self.gyro_noise = 5e-4  # angle random walk, rad/sqrt(s); replaced by the IMU's own figure
        self.heard = {}        # mmsi: last detection, for the first fix
        self.odom_pub = self.create_publisher(Odometry, "/rf_nav/odom", 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, "/rf/detections", self.on_detection, 50)

    def predict(self, t):
        if self.x is None or t <= self.t:
            return
        dt = t - self.t
        F = np.eye(6)
        F[X, VX] = F[Y, VY] = dt
        F[PSI, BG] = -dt
        self.x = F @ self.x
        self.x[PSI] = wrap(self.x[PSI] + self.gyro_z * dt)
        q = ACCEL_SIGMA ** 2
        Q = np.zeros((6, 6))
        for p, v in ((X, VX), (Y, VY)):
            Q[p, p], Q[p, v], Q[v, p], Q[v, v] = q * dt ** 3 / 3, q * dt ** 2 / 2, q * dt ** 2 / 2, q * dt
        Q[PSI, PSI] = (2 * self.gyro_noise) ** 2 * dt  # gyro white noise, integrated
        Q[BG, BG] = GYRO_BIAS_WALK ** 2 * dt
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.gyro_noise = math.sqrt(max(m.angular_velocity_covariance[8], 1e-10)) * 0.1  # per-sample sigma at 100 Hz
        self.predict(t)
        self.gyro_z = m.angular_velocity.z
        if self.x is not None:
            self.publish(m.header.stamp)

    def ship_xy(self, d):
        return np.array(rf.latlon_to_enu(d["lat"], d["lon"], *self.origin))

    def sigma(self, d, ship):
        """Bearing sigma: the direction finder's own, its fixed bias, and the ship's reported position error."""
        r = max(np.hypot(*(ship - self.x[:2])), 50.0) if self.x is not None else 1000.0
        return math.sqrt(d["azimuth_std_rad"] ** 2 + math.radians(BEARING_BIAS_DEG) ** 2 + (SHIP_POSITION_SIGMA_M / r) ** 2)

    def on_detection(self, m):
        d = json.loads(m.data)
        if self.x is None:
            self.start(d)
            return
        self.predict(d["t"])
        ship = self.ship_xy(d)
        dx, dy = ship - self.x[:2]
        r2 = dx * dx + dy * dy
        innov = wrap(d["azimuth_body_rad"] - (math.atan2(dy, dx) - self.x[PSI]))
        H = np.zeros(6)
        H[X], H[Y], H[PSI] = dy / r2, -dx / r2, -1.0
        S = H @ self.P @ H + self.sigma(d, ship) ** 2
        if innov ** 2 / S > GATE:
            self.get_logger().warn(f"bearing to {d['mmsi']} rejected: {math.degrees(innov):+.1f} deg off")
            return
        K = self.P @ H / S
        self.x = self.x + K * innov
        self.x[PSI] = wrap(self.x[PSI])
        I_KH = np.eye(6) - np.outer(K, H)
        self.P = I_KH @ self.P @ I_KH.T + np.outer(K, K) * self.sigma(d, ship) ** 2  # Joseph form

    def start(self, d):
        self.heard[d["mmsi"]] = d
        recent = [h for h in self.heard.values() if d["t"] - h["t"] <= START_WINDOW_S]
        if len(recent) < 3:
            return
        ships = np.array([self.ship_xy(h) for h in recent])
        s, cov = resection(ships, np.array([h["azimuth_body_rad"] for h in recent]),
                           [self.sigma(h, None) for h in recent])
        self.x = np.array([s[0], s[1], 0.0, 0.0, s[2], 0.0])
        self.P = np.diag([0.0, 0.0, 5.0 ** 2, 5.0 ** 2, 0.0, math.radians(0.5) ** 2])
        self.P[np.ix_([X, Y, PSI], [X, Y, PSI])] += cov * 4
        self.P[X, X] += START_SIGMA_M ** 2
        self.P[Y, Y] += START_SIGMA_M ** 2
        self.t = d["t"]
        self.get_logger().info(f"first fix from {len(recent)} ships: x {s[0]:.0f} m, y {s[1]:.0f} m, "
                               f"heading {math.degrees(s[2]):.1f} deg")

    def publish(self, stamp):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "world"
        o.child_frame_id = "rf_nav"
        o.pose.pose.position.x, o.pose.pose.position.y = float(self.x[X]), float(self.x[Y])
        o.pose.pose.orientation.z = math.sin(self.x[PSI] / 2)
        o.pose.pose.orientation.w = math.cos(self.x[PSI] / 2)
        cov = np.zeros((6, 6))  # x y z roll pitch yaw
        cov[np.ix_([0, 1], [0, 1])] = self.P[np.ix_([X, Y], [X, Y])]
        cov[2, 2] = cov[3, 3] = cov[4, 4] = 1e6
        cov[5, 5] = self.P[PSI, PSI]
        cov[0, 5] = cov[5, 0] = self.P[X, PSI]
        cov[1, 5] = cov[5, 1] = self.P[Y, PSI]
        o.pose.covariance = cov.ravel().tolist()
        o.twist.twist.linear.x, o.twist.twist.linear.y = float(self.x[VX]), float(self.x[VY])
        self.odom_pub.publish(o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="strait", help="for the latitude and longitude of the world origin")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    try:
        rclpy.spin(RfNav(args.world))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

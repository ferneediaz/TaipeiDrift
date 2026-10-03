"""AIS transmissions from the ships, as heard by the drone's receiver and direction finder.

Started by `sim.launch.py` in the strait world (or with ships:=true), or by hand while the simulator runs:
    python3 sim/nodes/rf_sensor.py --world strait [--config sim/config/rf.yaml]

Each ship transmits one 256-bit GMSK packet per AIS reporting interval, alternating between AIS 1 and AIS 2.
For every transmission and every receiver the link model of rf_model.py decides whether the packet is decoded.
A decoded packet gives the receiver the ship's MMSI and the position the ship reports (AIS position reports carry
the ship's own GNSS fix; only the drone is GNSS-denied), plus the RSSI and bearing the direction finder measures.

/rf/detections  std_msgs/String, JSON, one per decoded packet (what the drone knows):
    t, receiver, mmsi, channel (1 or 2), freq_hz, lat, lon, rssi_dbm, snr_db, azimuth_body_rad, azimuth_std_rad
    lat, lon: the ship's reported position (its AIS antenna, with the ship's GNSS error), degrees.
    azimuth_body_rad: bearing to the ship in the drone's body frame (x forward, y left), counter-clockwise from
    forward, in (-pi, pi]. Turning it into a world bearing needs the drone's heading, which is the navigator's job.
/rf/truth       std_msgs/String, JSON, one per transmission and receiver, decoded or not (for scoring only):
    t, receiver, ship, mmsi, ship_xy, antenna_xy, antenna_height_m, drone_xyz, range_m, azimuth_body_rad,
    azimuth_world_rad, p_rx_dbm, snr_db, per, decoded
/rf/params      std_msgs/String, JSON, latched: the implementation loss calibration and the bearing bias draw
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

import rf_model as rf

# A square pseudo-Doppler array is unambiguous while its side is under 0.35 wavelength (half a wavelength across
# the diagonal); 0.22 works best (Gerhard and Tokekar, arXiv 2003.00386, section III)
MAX_ARRAY_SIDE_WAVELENGTHS = 0.35
CONFIG = Path(__file__).resolve().parents[1] / "config" / "rf.yaml"
WORLDS = Path(__file__).resolve().parents[1] / "worlds"


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def to_body(q, v):
    """Rotate world vector v into the body frame of orientation q (x, y, z, w): R(q)^T v."""
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return r.T @ v


class RfSensor(Node):
    def __init__(self, cfg, world):
        super().__init__("rf_sensor", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.cfg = cfg
        self.origin = rf.world_origin(WORLDS / f"{world}.sdf")
        seed = cfg.get("seed", -1)
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        ais, rx, df = cfg["ais"], cfg["receiver"], cfg["direction_finder"]
        self.impl_loss = rf.calibrate_impl_loss(rx["sensitivity_dbm"], rx["per_at_sensitivity"], ais["packet_bits"],
                                                ais["bit_rate"], rx["noise_figure_db"], ais["ber_alpha"])
        self.noise_floor = rf.noise_floor_dbm(ais["bandwidth_hz"], rx["noise_figure_db"])
        # The pseudo-Doppler array: unambiguous only if the square is small against the wavelength, and it must
        # rotate electronically often enough within one packet to measure the Doppler tone's phase
        wavelength = rf.wavelength(sum(ais["channels_hz"]) / len(ais["channels_hz"]))
        side = 2 * df["array_radius_m"] * math.sin(math.pi / df["elements"])
        rotations = df["rotation_hz"] * ais["packet_bits"] / ais["bit_rate"]
        if side > MAX_ARRAY_SIDE_WAVELENGTHS * wavelength:
            raise SystemExit(f"direction_finder: array side {side:.2f} m is over {MAX_ARRAY_SIDE_WAVELENGTHS} "
                             f"wavelength ({MAX_ARRAY_SIDE_WAVELENGTHS * wavelength:.2f} m); the bearing would be ambiguous")
        if rotations < df["min_rotations_per_packet"]:
            raise SystemExit(f"direction_finder.rotation_hz {df['rotation_hz']} gives {rotations:.1f} rotations per "
                             f"packet, under {df['min_rotations_per_packet']}")
        self.bias = {r["name"]: float(self.rng.normal(0.0, math.radians(df["bias_sigma_deg"])))
                     for r in cfg["receivers"]}
        params = {"seed": seed, "impl_loss_db": self.impl_loss, "noise_floor_dbm": self.noise_floor,
                  "azimuth_bias_rad": self.bias, "array_side_wavelengths": side / wavelength,
                  "rotations_per_packet": rotations}
        self.get_logger().info(f"RF model: {json.dumps(params)}")
        if self.impl_loss < 0:
            self.get_logger().warn("Sensitivity is better than an ideal GMSK receiver allows; check receiver config")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(String, "/rf/params", latched).publish(String(data=json.dumps(params)))
        self.det_pub = self.create_publisher(String, "/rf/detections", 50)
        self.truth_pub = self.create_publisher(String, "/rf/truth", 50)

        self.drones = {}
        for r in cfg["receivers"]:
            self.create_subscription(Odometry, r["odom_topic"],
                                     lambda m, n=r["name"]: self.drones.__setitem__(n, m), qos_profile_sensor_data)
        self.ships = {s["name"]: s for s in cfg["ships"]}
        self.ship_odom = {}
        for name in self.ships:
            self.create_subscription(Odometry, f"/ships/{name}/odom",
                                     lambda m, n=name: self.ship_odom.__setitem__(n, m), qos_profile_sensor_data)
        self.next_tx = {}     # ship: sim time of its next transmission
        self.channel = {name: int(self.rng.integers(2)) for name in self.ships}
        self.create_timer(0.02, self.step)

    def interval(self, ship, speed):
        jitter = self.cfg["ais"]["report_jitter"]
        return rf.ais_report_interval_s(ship["ais_class"], speed) * self.rng.uniform(1 - jitter, 1 + jitter)

    def step(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if t < self.cfg["ais"].get("start_after_s", 0.0):
            return
        for name, odom in self.ship_odom.items():
            ship, speed = self.ships[name], odom.twist.twist.linear.x
            if name not in self.next_tx:  # first report at a random point of the first interval
                self.next_tx[name] = t + self.rng.uniform(0, 1) * self.interval(ship, speed)
            if t >= self.next_tx[name]:
                self.transmit(t, ship, odom)
                self.next_tx[name] = t + self.interval(ship, speed)

    def transmit(self, t, ship, odom):
        ais, prop, rx, df = self.cfg["ais"], self.cfg["propagation"], self.cfg["receiver"], self.cfg["direction_finder"]
        ch = self.channel[ship["name"]]
        self.channel[ship["name"]] = 1 - ch
        freq = float(ais["channels_hz"][ch])
        sp, q = odom.pose.pose.position, odom.pose.pose.orientation
        yaw = 2 * math.atan2(q.z, q.w)  # ships only turn about z
        fwd, left = ship.get("antenna_xy_m", [0.0, 0.0])
        h_tx = ship["antenna_height_m"]
        antenna = np.array([sp.x + fwd * math.cos(yaw) - left * math.sin(yaw),
                            sp.y + fwd * math.sin(yaw) + left * math.cos(yaw), self.cfg["sea_level_z"] + h_tx])
        for rname, drone in self.drones.items():
            dp = drone.pose.pose.position
            h_rx = max(dp.z - self.cfg["sea_level_z"], 0.1)
            v = antenna - np.array([dp.x, dp.y, dp.z])
            d_h = math.hypot(v[0], v[1])
            p_rx = rf.link(d_h, h_tx, h_rx, freq, ais["class_power_dbm"][ship["ais_class"]], ship["antenna_gain_db"],
                           ship["cable_loss_db"], rx["antenna_gain_db"], prop["sea_reflection"],
                           self.rng.normal(0.0, prop["shadowing_db"]))
            if math.isfinite(p_rx):
                ber = rf.gmsk_ber(rf.ebn0_db(p_rx, ais["bit_rate"], rx["noise_figure_db"], self.impl_loss),
                                  ais["ber_alpha"])
                per = float(rf.packet_error_rate(ber, ais["packet_bits"]))
            else:
                per = 1.0
            decoded = bool(self.rng.uniform() >= per)
            snr = p_rx - self.noise_floor
            vb = to_body(drone.pose.pose.orientation, v)
            az_body = math.atan2(vb[1], vb[0])
            truth = {"t": t, "receiver": rname, "ship": ship["name"], "mmsi": ship["mmsi"], "ship_xy": [sp.x, sp.y],
                     "antenna_xy": antenna[:2].tolist(),
                     "antenna_height_m": h_tx, "drone_xyz": [dp.x, dp.y, dp.z], "range_m": float(np.linalg.norm(v)),
                     "azimuth_body_rad": az_body, "azimuth_world_rad": math.atan2(v[1], v[0]),
                     "p_rx_dbm": p_rx if math.isfinite(p_rx) else None,
                     "snr_db": snr if math.isfinite(snr) else None, "per": per, "decoded": decoded}
            self.truth_pub.publish(String(data=json.dumps(truth)))
            if not decoded:
                continue
            sigma = rf.bearing_sigma_rad(snr, df["sigma_floor_deg"], df["sigma_at_ref_deg"], df["snr_ref_db"])
            reported = antenna[:2] + self.rng.normal(0.0, ais["position_sigma_m"], 2)
            lat, lon = rf.enu_to_latlon(reported[0], reported[1], *self.origin)
            det = {"t": t, "receiver": rname, "mmsi": ship["mmsi"], "channel": ch + 1, "freq_hz": freq,
                   "lat": lat, "lon": lon,
                   "rssi_dbm": float(round(p_rx + self.rng.normal(0.0, rx["rssi_noise_db"]))),
                   "snr_db": round(snr, 1),
                   "azimuth_body_rad": wrap(az_body + self.bias[rname] + self.rng.normal(0.0, sigma)),
                   "azimuth_std_rad": sigma}
            self.det_pub.publish(String(data=json.dumps(det)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--world", default="strait", help="for the latitude and longitude of the world origin")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    try:
        rclpy.spin(RfSensor(yaml.safe_load(open(args.config)), args.world))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

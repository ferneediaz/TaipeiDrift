"""Live sensor dashboard in the terminal.

    docker compose exec sim bash -ic "python3 sim/nodes/sensor_monitor.py [--world terrain|islands]"

Shows what the drone's sensors report, next to the ground truth, refreshed 5 times a second.
Rates are measured in simulation time.

At the top, NAVIGATION compares the position estimates that run on this flight, each against ground truth:
    RF only    nodes/rf_nav.py: the ships' bearings (strait world)
    ESKF       nodes/eskf_ros_adapter.py: IMU + barometer + forward camera, GNSS until the cutoff
    ESKF + RF  the same ESKF, also fusing the RF position fix (strait world)
"""
import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, FluidPressure, Image, Imu, NavSatFix
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

WORLDS = Path(__file__).resolve().parents[1] / "worlds"
M_PER_DEG_LAT = 111_320.0

BOLD, DIM, GREEN, RED, RESET = "\033[1m", "\033[2m", "\033[32m", "\033[31m", "\033[0m"
YELLOW, CYAN = "\033[33m", "\033[36m"
# The estimates compared in NAVIGATION: key, odometry topic, status topic, name, what it uses, whose
ESTIMATORS = [
    ("rf", "/rf_nav/odom", None, "RF only", "ships' bearings + gyro", "rf_nav, Dan"),
    ("eskf", "/nav/odom", "/nav/estimator_status", "ESKF", "IMU+baro+camera, GNSS", "Alessandro"),
    ("eskf_rf", "/nav_rf/odom", "/nav_rf/estimator_status", "ESKF + RF", "the ESKF + the RF fix", "combined"),
]


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))


GOOD_M, FAIR_M = 20.0, 100.0   # error bar colours: green below GOOD_M, yellow below FAIR_M, red beyond
BAR_DECADES = 3                 # the bar spans 1 m to 10^BAR_DECADES m


def error_bar(err, width=12):
    """Log scale, 1 m (empty) to 1 km (full): the eye sees 10 m against 100 m at once."""
    n = round(width * min(max(math.log10(max(err, 1.0)) / BAR_DECADES, 0.0), 1.0))
    col = GREEN if err < GOOD_M else YELLOW if err < FAIR_M else RED
    return f"{col}{'█' * n}{RESET}{DIM}{'·' * (width - n)}{RESET}"


class Estimate:
    """One estimator's latest output, its update counts and its error since the GNSS cutoff."""

    def __init__(self):
        self.odom = None
        self.counts = {}       # source: [accepted, rejected]
        self.sq, self.n, self.max = 0.0, 0, 0.0

    def on_status(self, m):
        d = json.loads(m.data)
        ev = d.get("event")
        if ev in ("gnss_update", "rf_update"):
            c = self.counts.setdefault(ev.split("_")[0], [0, 0])
            c[0 if d["accepted"] else 1] += 1
            if d.get("reanchored"):
                self.counts.setdefault("reset to RF", [0, 0])[0] += 1
        elif ev == "visual_span":
            c = self.counts.setdefault("vision", [0, 0])
            c[0 if d.get("rotation_accepted") or d.get("direction_accepted") else 1] += 1


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Rate:
    """Messages per second of simulation time, over the last couple of seconds."""

    def __init__(self):
        self.t = deque(maxlen=300)

    def tick(self, t):
        if t is not None:
            self.t.append(t)

    def hz(self):
        return (len(self.t) - 1) / (self.t[-1] - self.t[0]) if len(self.t) > 2 and self.t[-1] > self.t[0] else 0.0


def world_origin(name):
    """Latitude and longitude of the world origin, for turning GPS into metres east/north."""
    sc = ET.parse(WORLDS / f"{name}.sdf").find(".//spherical_coordinates")
    return float(sc.findtext("latitude_deg")), float(sc.findtext("longitude_deg"))


class Monitor(Node):
    def __init__(self, world):
        super().__init__("sensor_monitor")
        self.lat0, self.lon0 = world_origin(world)
        self.sim_t = None
        self.imu = self.baro = self.gps = self.truth = self.info = None
        self.p0 = None
        self.rates = {k: Rate() for k in ("imu", "baro", "gps", "cam", "truth")}
        self.wall0 = self.sim0 = None
        sd = qos_profile_sensor_data
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(Imu, "/imu/data", lambda m: self.keep("imu", m), sd)
        self.create_subscription(FluidPressure, "/air_pressure", self.on_baro, sd)
        self.create_subscription(NavSatFix, "/gps/fix", lambda m: self.keep("gps", m), sd)
        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: self.keep("truth", m), sd)
        self.create_subscription(CameraInfo, "/camera/down/camera_info", lambda m: setattr(self, "info", m), sd)
        # Images are counted without decoding them; Python is too slow to decode every frame
        self.create_subscription(Image, "/camera/down/image_raw", lambda _: self.rates["cam"].tick(self.sim_t), 10, raw=True)
        # AIS from the ships (nodes/rf_sensor.py): last truth and detection per MMSI, and decoded/sent counts
        self.rf = {}
        self.create_subscription(String, "/rf/truth", self.on_rf_truth, 50)
        self.create_subscription(String, "/rf/detections", self.on_rf_detection, 50)
        self.rf_nav = None  # drone position from the ships' bearings (nodes/rf_nav.py)
        self.create_subscription(Odometry, "/rf_nav/odom", lambda m: setattr(self, "rf_nav", m), 10)
        self.est = {key: Estimate() for key, *_ in ESTIMATORS}
        for key, odom, status, *_ in ESTIMATORS:
            self.create_subscription(Odometry, odom, lambda m, k=key: setattr(self.est[k], "odom", m), sd)
            if status:
                self.create_subscription(String, status, self.est[key].on_status, 50)
        self.gnss_on = None    # the GNSS gate's state (nodes/gnss_gate.py)
        self.cut_t = None      # sim time when it closed
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, "/nav/gnss_available", self.on_gnss_available, latched)
        self.create_timer(0.2, self.draw, clock=rclpy.clock.Clock())  # wall-clock refresh

    def on_clock(self, m):
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.sim0 is None:
            self.wall0, self.sim0 = time.time(), self.sim_t

    def keep(self, name, msg):
        setattr(self, name, msg)
        self.rates[name].tick(stamp(msg))

    def on_baro(self, m):
        if self.p0 is None:
            self.p0 = m.fluid_pressure
        self.keep("baro", m)

    def on_gnss_available(self, m):
        if self.gnss_on and not m.data and self.sim_t is not None:
            self.cut_t = self.sim_t
        self.gnss_on = m.data

    def navigation(self):
        """The comparison table: who knows best where the drone is, right now and since the GNSS cutoff."""
        live = [(e, self.est[e[0]]) for e in ESTIMATORS if self.est[e[0]].odom is not None]
        if not live or not self.truth:
            return []
        if self.gnss_on is None:
            gnss = f"{DIM}GNSS gate not running{RESET}"
        elif self.gnss_on:
            gnss = f"{GREEN}GNSS ON{RESET}"
        else:
            since = (f" {self.sim_t - self.cut_t:.0f} s ago" if self.cut_t is not None
                     else " before this monitor started (RMS counts from its start)")
            gnss = f"{RED}GNSS CUT{since}{RESET}: the estimates are on their own"
        tp = self.truth.pose.pose.position
        tyaw = quat_yaw(self.truth.pose.pose.orientation)
        out = [f"{BOLD}NAVIGATION{RESET}  {gnss}",
               f"  {DIM}error = how far the estimate is from the true position; bar: log scale, "
               f"green < {GOOD_M:.0f} m, yellow < {FAIR_M:.0f} m, red beyond{RESET}",
               f"  {DIM}{'estimate':<11}{'uses':<23}{'error now':>10}  {f'error 1m…{10 ** BAR_DECADES / 1000:.0f}km':<12} {'its 2σ':>7}"
               f" {'RMS since cut':>14} {'height':>7} {'heading':>8}  fused (✓ used / ✗ rejected){RESET}"]
        errs = {}
        for (key, _, _, name, uses, who), e in live:
            o = e.odom
            p, c = o.pose.pose.position, o.pose.covariance
            err = math.hypot(p.x - tp.x, p.y - tp.y)
            errs[key] = err
            two_sigma = 2 * math.sqrt(max(c[0] + c[7], 0.0))
            if self.gnss_on is False:
                e.sq, e.n, e.max = e.sq + err * err, e.n + 1, max(e.max, err)
            rms = f"{math.sqrt(e.sq / e.n):7.1f} m" if e.n else "       —"
            inside = GREEN if err <= two_sigma else RED  # the estimate is honest when the truth is inside its 2σ
            height = f"{p.z - tp.z:+6.1f}m" if key != "rf" else "     — "
            heading = (math.degrees(quat_yaw(o.pose.pose.orientation) - tyaw) + 180) % 360 - 180
            fused = "  ".join(f"{src} {a}✓" + (f" {r}✗" if r else "") for src, (a, r) in e.counts.items())
            if key == "rf":  # rf_nav reports no status: count the bearings it was given
                fused = f"bearings {sum(r['decoded'] for r in self.rf.values())}"
            out.append(f"  {BOLD}{name:<11}{RESET}{uses:<23}{err:8.1f} m  {error_bar(err)} "
                       f"{inside}{two_sigma:5.0f} m{RESET} {rms:>14} {height:>7} {heading:+7.1f}°  {DIM}{fused}{RESET}")
            out.append(f"  {DIM}{'(' + who + ')':<11}{RESET}")
        if self.gnss_on is False and "eskf_rf" in errs:
            mine = max(errs["eskf_rf"], 0.1)  # m; keeps the ratio finite when the error is near zero
            parts = []
            for k, n in (("eskf", "the ESKF alone"), ("rf", "RF alone")):
                if k in errs:
                    ratio = errs[k] / mine
                    parts.append(f"{ratio:.1f}x closer than {n}" if ratio >= 1 else
                                 f"{1 / ratio:.1f}x farther than {n}")
            out.append(f"  {CYAN}→ ESKF + RF is {', '.join(parts)} right now{RESET}")
        elif self.gnss_on:
            out.append(f"  {DIM}→ with GNSS every estimate is near the truth; the difference shows after the cut{RESET}")
        return out + [""]

    def on_rf_truth(self, m):
        t = json.loads(m.data)
        r = self.rf.setdefault(t["mmsi"], {"ship": t["ship"], "sent": 0, "decoded": 0, "det": None})
        r["sent"] += 1
        r["decoded"] += t["decoded"]
        r["truth"] = t

    def on_rf_detection(self, m):
        d = json.loads(m.data)
        if d["mmsi"] in self.rf:
            self.rf[d["mmsi"]]["det"] = d

    def draw(self):
        out = [f"{BOLD}TaipeiDrift sensor monitor{RESET}   Ctrl+C to quit"]
        if self.sim_t is None:
            out.append("waiting for the simulator (/clock)...")
            print("\033[H\033[J" + "\n".join(out), flush=True)
            return
        rtf = (self.sim_t - self.sim0) / max(1e-6, time.time() - self.wall0)
        out.append(f"sim time {self.sim_t:8.1f} s    real-time factor {rtf:4.2f}\n")
        out += self.navigation()
        r = {k: v.hz() for k, v in self.rates.items()}

        tz = None
        if self.truth:
            p, v, q = self.truth.pose.pose.position, self.truth.twist.twist.linear, self.truth.pose.pose.orientation
            roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2)))
            tz = p.z
            out.append(f"{BOLD}GROUND TRUTH{RESET} {DIM}({r['truth']:5.1f} Hz, not a sensor){RESET}")
            out.append(f"  position  E {p.x:8.2f}   N {p.y:8.2f}   U {p.z:7.2f}  m")
            out.append(f"  velocity  fwd {v.x:6.2f}  left {v.y:6.2f}  up {v.z:6.2f}  m/s   speed {math.hypot(v.x, v.y, v.z):5.2f}")
            out.append(f"  attitude  roll {roll:6.1f}  pitch {pitch:6.1f}  yaw {yaw:6.1f}  deg\n")

        if self.imu:
            w, a = self.imu.angular_velocity, self.imu.linear_acceleration
            out.append(f"{BOLD}IMU{RESET} {DIM}({r['imu']:5.1f} Hz, Mid-Air noise model){RESET}")
            out.append(f"  gyro      x {w.x:+8.4f}   y {w.y:+8.4f}   z {w.z:+8.4f}  rad/s")
            out.append(f"  accel     x {a.x:+8.3f}   y {a.y:+8.3f}   z {a.z:+8.3f}  m/s^2\n")

        if self.baro:
            p = self.baro.fluid_pressure
            alt = 44330.0 * (1.0 - (p / self.p0) ** (1 / 5.255))
            err = f"   error {alt - tz:+6.2f} m" if tz is not None else ""
            out.append(f"{BOLD}BAROMETER{RESET} {DIM}(air pressure sensor, {r['baro']:5.1f} Hz){RESET}")
            out.append(f"  pressure  {p:10.1f} Pa    altitude {alt:7.2f} m above start{err}\n")

        out.append(f"{BOLD}GPS{RESET} {DIM}({r['gps']:5.1f} Hz){RESET}")
        if self.gps and r["gps"] > 0 and self.sim_t - stamp(self.gps) < 3:
            e = (self.gps.longitude - self.lon0) * M_PER_DEG_LAT * math.cos(math.radians(self.lat0))
            n = (self.gps.latitude - self.lat0) * M_PER_DEG_LAT
            err = ""
            if self.truth:
                tp = self.truth.pose.pose.position
                err = f"   error {math.hypot(e - tp.x, n - tp.y):5.2f} m"
            out.append(f"  lat {self.gps.latitude:11.6f}  lon {self.gps.longitude:11.6f}  alt {self.gps.altitude:7.1f} m")
            out.append(f"  as metres E {e:8.2f}   N {n:8.2f}{err}\n")
        else:
            out.append(f"  {RED}no fix{RESET} (gps:=false, or no signal)\n")

        out.append(f"{BOLD}DOWN CAMERA{RESET}")
        if self.info:
            ok = GREEN if abs(r["cam"] - 25) < 2.5 else RED
            ground = 2 * tz if tz and tz > 0 else None  # 90 deg FOV: footprint = 2 x height
            fp = f"   footprint {ground:6.1f} m  ({100 * ground / self.info.width:5.1f} cm/px)" if ground else ""
            out.append(f"  {self.info.width}x{self.info.height}  fx {self.info.k[0]:.0f}  "
                       f"{ok}{r['cam']:5.1f} fps{RESET} (want 25){fp}")

        if self.rf and not self.rf_nav:
            out.append(f"\n{BOLD}RF NAVIGATION{RESET} {DIM}waiting for bearings to three ships...{RESET}")

        if self.rf:
            out.append(f"\n{BOLD}AIS RECEIVER{RESET} {DIM}(GMSK 9600 bit/s, 162 MHz; angle of arrival, error against truth){RESET}")
            for mmsi, r in sorted(self.rf.items()):
                t, d = r["truth"], r["det"]
                line = f"  {r['ship']:<20} {mmsi}  range {t['range_m'] / 1e3:6.2f} km  decoded {r['decoded']:3d}/{r['sent']}"
                if d:
                    # the last decoded packet; its truth is the one sent at the same time
                    err = math.degrees((d["azimuth_body_rad"] - t["azimuth_body_rad"] + math.pi) % (2 * math.pi) - math.pi) \
                        if abs(d["t"] - t["t"]) < 1e-6 else None
                    e = f"{err:+5.1f} deg" if err is not None else "  (old)  "
                    line += (f"\n    AoA {math.degrees(d['azimuth_body_rad']):+6.1f} deg from the nose"
                             f"  error {e} (sigma {math.degrees(d['azimuth_std_rad']):.1f})")
                out.append(line)
        print("\033[H\033[J" + "\n".join(out), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="terrain", help="the world that runs, for the GPS origin")
    args = ap.parse_args()
    rclpy.init()
    try:
        rclpy.spin(Monitor(args.world))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

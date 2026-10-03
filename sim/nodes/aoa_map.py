"""Live map of the drone's position from the ships' AIS signals, without GNSS: rf_nav's estimate, which combines the
angles of arrival (AoA) of the packets, the gyro and a Kalman filter, with the bearings it uses.

Started by `sim.launch.py` with the ships and demo:=true (the strait world), or by hand while the simulator runs:
    python3 sim/nodes/aoa_map.py --world strait
The window opens when the first ship's bearing arrives, so it appears when the ships start transmitting
(ais.start_after_s in config/rf.yaml).

The window has three parts, written for someone who has not seen it before:
- Left, the map of the strait: the three ships, the line from each ship along its last bearing, the drone's estimated
  position (red dot), the true position (gold star) and their tracks.
- Top right, a close-up around the drone: the estimate, the area it says the drone is in (its 95 % ellipse), the true
  position, and the one-shot fix from the last three bearings alone.
- Bottom right, the numbers in words: how far off the estimate is, how far off it says it might be, whether the truth
  is inside that, the heading error, and whether GPS is on.

The math
- rf_nav's estimate (/rf_nav/odom, nodes/rf_nav.py: AoA + gyro + extended Kalman filter). Its 95 % ellipse comes from
  its own covariance: the points with squared Mahalanobis distance under chi2.ppf(0.95, 2) = 5.99. Its error is the
  horizontal distance to the true position at the estimate's own time, the terminal's "RF (AoA)" row.
- The bearings: each ship's last packet within START_WINDOW_S. A bearing b_i is measured from the drone's nose at its
  own time t_i, and the drone turns in between, so it is referred to the estimate's time t with the gyro,
      b_i' = b_i - (yaw_gyro(t) - yaw_gyro(t_i)),
  and drawn from the ship at the world angle psi + b_i' + 180 deg, with psi the filter's heading, dashed at +-2 sigma
  (rf_nav's bearing sigma: the direction finder's own, its fixed bias, the ship's reported-position error).
- The one-shot fix: the last three bearings alone, no filter and no memory, solved by rf_nav's own resection
  (x, y and heading from three bearings), with START_SIGMA_M for the drone's motion while they arrived.
The true position is for scoring only. RSSI is not used. Nothing here feeds the navigation.
"""
import argparse
import bisect
import json
import math
from pathlib import Path

import matplotlib
import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

matplotlib.use("QtAgg")
matplotlib.rcParams["toolbar"] = "None"  # a display, not a plotting tool
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Ellipse, Patch  # noqa: E402
from scipy.stats import chi2  # noqa: E402

import rf_model as rf  # noqa: E402
from rf_nav import (BEARING_BIAS_DEG, SHIP_POSITION_SIGMA_M, START_SIGMA_M, START_WINDOW_S,  # noqa: E402
                    resection, wrap)

SIM = Path(__file__).resolve().parents[1]
WORLDS = SIM / "worlds"
RF_CONFIG = SIM / "config" / "rf.yaml"
MIN_SHIPS = 3             # three bearings for three unknowns (x, y, heading)
AREA = (-1100.0, 2000.0, -900.0, 1250.0)  # x0, x1, y0, y1 of the map, world metres
ZOOM_M = 250.0            # half-width of the close-up, metres; widened when the ellipse or the truth is farther out
MIN_RANGE_M = 10.0        # below this a ship's range is clamped in the bearing sigma
GYRO_HISTORY_S = 2 * START_WINDOW_S  # integrated gyro yaw kept for referring bearings to one time
TRACK_S = 60.0            # length of the drawn tracks, sim seconds
ELLIPSE_95 = chi2.ppf(0.95, 2)  # squared Mahalanobis radius of the 95 % ellipse in 2D (5.99)
COLOURS = ["tab:blue", "tab:orange", "tab:green", "tab:purple", "tab:brown"]
EST, TRUE, SNAP = "tab:red", "gold", "0.3"
WINDOW = (720, 470)       # width and height of the window, in the browser desktop's top-right corner
PLACE_DRAWS = 8           # place it on each of the first draws (0.5 s apart)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def stamp_of(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def pretty(name):
    """carrier_tai_shan -> Carrier Tai Shan"""
    return name.replace("_", " ").title()


class AoaMap(Node):
    def __init__(self, world):
        super().__init__("aoa_map")
        self.origin = rf.world_origin(WORLDS / f"{world}.sdf")
        self.names = {s["mmsi"]: pretty(s["name"]) for s in yaml.safe_load(RF_CONFIG.read_text())["ships"]}
        self.last = {}            # mmsi: last detection
        self.truth = self.nav = None
        self.gyro_t, self.gyro_yaw = [], []  # integrated body yaw rate, sim time
        self.truth_hist = []      # (t, x, y, yaw), for the truth at the estimate's own time
        self.nav_track, self.truth_track = [], []  # (t, x, y), the last TRACK_S
        self.gnss_on, self.cut_t = None, None
        sd = qos_profile_sensor_data
        self.create_subscription(String, "/rf/detections", self.on_detection, 50)
        self.create_subscription(Imu, "/imu/data", self.on_imu, sd)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, sd)
        self.create_subscription(Odometry, "/rf_nav/odom", self.on_nav, 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, "/nav/gnss_available", self.on_gnss, latched)

    def open_window(self):
        """Opened by main() once the first ship is heard: there is nothing to show before the RF is on."""
        plt.ion()
        self.fig = plt.figure(figsize=(7.2, 4.7), dpi=100)
        outer = self.fig.add_gridspec(1, 2, width_ratios=[1.45, 1], left=0.085, right=0.985, top=0.95, bottom=0.03,
                                      wspace=0.14)
        left = outer[0].subgridspec(2, 1, height_ratios=[1, 0.27], hspace=0.22)
        right = outer[1].subgridspec(2, 1, height_ratios=[1.05, 1], hspace=0.2)
        self.ax = self.fig.add_subplot(left[0])          # the strait
        self.key = self.fig.add_subplot(left[1])         # the legend, under the map so it covers nothing
        self.zoom = self.fig.add_subplot(right[0])       # close-up around the drone
        self.panel = self.fig.add_subplot(right[1])      # the numbers in words
        self.placed = 0           # draws so far that placed the window
        try:
            self.fig.canvas.manager.set_window_title("RF Navigation: AIS angle of arrival")
        except AttributeError:
            pass

    # --- inputs -------------------------------------------------------------------------------------------------
    def on_imu(self, m):
        t = stamp_of(m)
        if self.gyro_t and t <= self.gyro_t[-1]:
            return
        yaw = self.gyro_yaw[-1] + m.angular_velocity.z * (t - self.gyro_t[-1]) if self.gyro_t else 0.0
        self.gyro_t.append(t)
        self.gyro_yaw.append(yaw)
        k = bisect.bisect_left(self.gyro_t, t - GYRO_HISTORY_S)
        if k:
            del self.gyro_t[:k], self.gyro_yaw[:k]

    def on_truth(self, m):
        self.truth = m
        p = m.pose.pose.position
        t = stamp_of(m)
        self.truth_hist.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation)))
        while self.truth_hist and t - self.truth_hist[0][0] > GYRO_HISTORY_S:
            self.truth_hist.pop(0)
        self.add_track(self.truth_track, m)

    def on_nav(self, m):
        self.nav = m
        self.add_track(self.nav_track, m)

    def on_gnss(self, m):
        if self.gnss_on and not m.data and self.truth is not None:
            self.cut_t = stamp_of(self.truth)
        self.gnss_on = m.data

    def on_detection(self, m):
        d = json.loads(m.data)
        self.last[d["mmsi"]] = d

    @staticmethod
    def add_track(track, m):
        t = stamp_of(m)
        if not track or t - track[-1][0] >= 0.5:
            track.append((t, m.pose.pose.position.x, m.pose.pose.position.y))
        while track and t - track[0][0] > TRACK_S:
            track.pop(0)

    def truth_at(self, t):
        """True x, y, yaw at sim time t, from the recent history (nearest sample)."""
        if not self.truth_hist:
            return None
        return min(self.truth_hist, key=lambda h: abs(h[0] - t))[1:]

    def gyro_at(self, t):
        return float(np.interp(t, self.gyro_t, self.gyro_yaw)) if self.gyro_t else 0.0

    # --- drawing ------------------------------------------------------------------------------------------------
    def gps_line(self):
        if self.gnss_on is None:
            return "—", "0.4"
        if self.gnss_on:
            return "AVAILABLE", "tab:green"
        if self.cut_t is not None and self.truth is not None:
            return f"DENIED  T+{stamp_of(self.truth) - self.cut_t:.0f} s", "tab:red"
        return "DENIED", "tab:red"

    def draw(self):
        for a in (self.ax, self.key, self.zoom, self.panel):
            a.clear()
        self.frame(self.ax, AREA)
        self.panel.axis("off")
        self.key.axis("off")
        self.legend()
        self.ax.set_title("AIS lines of position", fontsize=8, loc="left", fontweight="bold")
        if self.nav is None:
            self.waiting()
            self.show()
            return

        o = self.nav
        t_nav = stamp_of(o)
        nx, ny, psi = o.pose.pose.position.x, o.pose.pose.position.y, yaw_of(o.pose.pose.orientation)
        c = o.pose.covariance
        cov = np.array([[c[0], c[1]], [c[6], c[7]]])
        fresh = sorted((d for d in self.last.values() if t_nav - d["t"] <= START_WINDOW_S), key=lambda d: d["mmsi"])

        # the bearings, referred to the estimate's time with the gyro
        ships = np.array([rf.latlon_to_enu(d["lat"], d["lon"], *self.origin) for d in fresh]).reshape(-1, 2)
        turned = np.array([self.gyro_at(t_nav) - self.gyro_at(d["t"]) for d in fresh])
        bearings = np.array([wrap(d["azimuth_body_rad"] - dpsi) for d, dpsi in zip(fresh, turned)])
        r2 = np.maximum(((ships - [nx, ny]) ** 2).sum(1), MIN_RANGE_M ** 2)
        base2 = np.array([d["azimuth_std_rad"] ** 2 for d in fresh]) + math.radians(BEARING_BIAS_DEG) ** 2
        sig = np.sqrt(base2 + SHIP_POSITION_SIGMA_M ** 2 / r2)            # rf_nav's bearing sigma

        # the one-shot fix from the last three bearings alone, for comparison
        snap = None
        if len(fresh) >= MIN_SHIPS:
            snap, _ = resection(ships, bearings,
                                np.sqrt(base2 + (SHIP_POSITION_SIGMA_M ** 2 + START_SIGMA_M ** 2) / r2))

        then = self.truth_at(t_nav)
        for ax, full in ((self.ax, True), (self.zoom, False)):
            self.draw_scene(ax, full, fresh, ships, bearings, sig, psi, nx, ny, cov, snap)
        half = max(ZOOM_M, 1.3 * math.sqrt(ELLIPSE_95 * max(np.linalg.eigvalsh(cov)[1], 0.0)),
                   1.3 * math.hypot(nx - then[0], ny - then[1]) if then else 0.0)
        self.frame(self.zoom, (nx - half, nx + half, ny - half, ny + half), small=True)
        self.zoom.set_title(f"Close-up  ±{half:.0f} m", fontsize=8, loc="left", fontweight="bold")
        self.write_panel(then, nx, ny, psi, cov, snap, fresh)
        self.show()

    def draw_scene(self, ax, full, fresh, ships, bearings, sig, psi, nx, ny, cov, snap):
        reach = math.hypot(AREA[1] - AREA[0], AREA[3] - AREA[2])
        for i, d in enumerate(fresh):
            col = COLOURS[i % len(COLOURS)]
            sx, sy = ships[i]
            back = psi + bearings[i] + math.pi  # from the ship, back along its line of position
            for off, lw, ls in ((0.0, 1.4, "-"), (-2 * sig[i], 0.6, "--"), (2 * sig[i], 0.6, "--")):
                ax.plot([sx, sx + reach * math.cos(back + off)], [sy, sy + reach * math.sin(back + off)],
                        color=col, lw=lw, ls=ls, alpha=0.8)
            if full:
                ax.plot(sx, sy, "^", color=col, ms=10, mec="k", zorder=6)
                ax.annotate(self.names.get(d["mmsi"], str(d["mmsi"])), (sx, sy), textcoords="offset points",
                            xytext=(7, 5), fontsize=7, color=col, fontweight="bold")
        for track, colour, ls in ((self.truth_track, "0.25", ":"), (self.nav_track, EST, "-")):
            if len(track) > 1:
                tr = np.array(track)
                ax.plot(tr[:, 1], tr[:, 2], color=colour, lw=1.1, ls=ls, zorder=3)
        vals, vecs = np.linalg.eigh(cov)
        vals = np.maximum(vals, 0.0)
        ax.add_patch(Ellipse((nx, ny), 2 * math.sqrt(ELLIPSE_95 * vals[1]), 2 * math.sqrt(ELLIPSE_95 * vals[0]),
                             angle=math.degrees(math.atan2(vecs[1, 1], vecs[0, 1])), facecolor=EST, alpha=0.2,
                             edgecolor=EST, lw=1.2, zorder=4))
        if snap is not None:
            ax.plot(snap[0], snap[1], "x", color=SNAP, ms=8 if full else 11, mew=2, zorder=5)
        ax.plot(nx, ny, "o", color=EST, ms=7 if full else 10, mec="k", zorder=6)
        if self.truth:
            p = self.truth.pose.pose.position
            ax.plot(p.x, p.y, "*", color=TRUE, ms=12 if full else 16, mec="k", zorder=7)

    def legend(self):
        handles = [
            Line2D([], [], marker="o", color=EST, mec="k", ls="", ms=7, label="EKF position estimate"),
            Patch(facecolor=EST, alpha=0.2, edgecolor=EST, label="95 % confidence region"),
            Line2D([], [], marker="*", color=TRUE, mec="k", ls="", ms=11, label="Ground truth (simulation)"),
            Line2D([], [], marker="x", color=SNAP, ls="", ms=7, mew=2, label="Snapshot fix (3 lines, no filter)"),
            Line2D([], [], color="0.4", lw=1.4, label="Line of position, ±2σ dashed"),
            Line2D([], [], color=EST, lw=1.1, label=f"Track, {TRACK_S:.0f} s (dotted: truth)"),
        ]
        self.key.legend(handles=handles, loc="upper center", ncol=2, fontsize=6.5, frameon=False,
                        handletextpad=0.4, columnspacing=1.0, borderaxespad=0.0)

    def rows(self, rows, footer):
        """An instrument-style readout: grey field names, bold values."""
        p = self.panel
        y = 1.0
        for name, value, colour, big in rows:
            p.text(0.0, y, name, fontsize=7, color="0.4", va="top", transform=p.transAxes, family="monospace")
            p.text(0.47, y + (0.02 if big else 0.0), value, fontsize=12 if big else 8, fontweight="bold",
                   color=colour, va="top", transform=p.transAxes, family="monospace")
            y -= 0.14 if big else 0.105
        p.text(0.0, 0.0, footer, fontsize=6.3, color="0.45", va="bottom", transform=p.transAxes)

    def write_panel(self, then, nx, ny, psi, cov, snap, fresh):
        gps, gps_col = self.gps_line()
        two_sigma = 2 * math.sqrt(max(cov[0, 0] + cov[1, 1], 0.0))
        rows = [("GNSS", gps, gps_col, False),
                ("SOURCE", "AIS AoA + GYRO", "0.15", False),
                (f"SHIPS ({START_WINDOW_S:.0f} s)", f"{len(fresh)}", "0.15", False)]
        if then:
            err = math.hypot(nx - then[0], ny - then[1])
            d = np.array([nx - then[0], ny - then[1]])
            within = np.linalg.det(cov) > 0 and d @ np.linalg.solve(cov, d) <= ELLIPSE_95
            rows += [("POSITION ERROR", f"{err:.0f} m", EST, True),
                     ("EST. ACCURACY 2σ", f"{two_sigma:.0f} m", "0.15", False),
                     ("INTEGRITY", "WITHIN 95 % BOUND" if within else "BOUND EXCEEDED",
                      "tab:green" if within else "tab:orange", False),
                     ("HEADING ERROR", f"{abs(math.degrees(wrap(psi - then[2]))):.1f}°", "0.15", False)]
            if snap is not None:
                rows.append(("SNAPSHOT ERROR", f"{math.hypot(snap[0] - then[0], snap[1] - then[1]):.0f} m",
                             SNAP, False))
        self.rows(rows, "Pseudo-Doppler AoA, 4-element array\nEKF with gyro · errors vs. simulation truth")

    def waiting(self):
        gps, gps_col = self.gps_line()
        self.zoom.axis("off")
        self.rows([("GNSS", gps, gps_col, False),
                   ("SOURCE", "AIS AoA + GYRO", "0.15", False),
                   ("STATUS", "ACQUIRING", "tab:orange", True),
                   ("SHIPS HEARD", f"{len(self.last)} / {MIN_SHIPS} needed", "0.15", False)],
                  f"First fix: bearings to {MIN_SHIPS} ships within {START_WINDOW_S:.0f} s\n(position and heading)")

    @staticmethod
    def frame(ax, area, small=False):
        ax.set_xlim(area[0], area[1])
        ax.set_ylim(area[2], area[3])
        ax.set_aspect("equal", adjustable="box")
        ax.tick_params(labelsize=6 if small else 7)
        if not small:
            ax.set_xlabel("east, m", fontsize=7)
            ax.set_ylabel("north, m", fontsize=7)
        ax.grid(alpha=0.25)

    def show(self):
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        try:
            w = self.fig.canvas.manager.window
            # matplotlib turns on Qt's high-DPI scaling and the dashboard does not: size in screen pixels, so the
            # two windows are the same width (at a 1.04 scale, 720 Qt pixels drew 750 on the screen)
            size = [round(n / w.devicePixelRatioF()) for n in WINDOW]
            drifted = [w.width(), w.height()] != size
        except AttributeError:
            return
        if self.placed < PLACE_DRAWS or drifted:  # the first draws: the window manager may move it after it appears
            self.placed += 1
            try:  # the top-right corner of the browser desktop, whatever the window manager adds around it
                screen = w.screen().availableGeometry()
                # never larger than the screen: a maximise once asked Qt for a 65535 x 65535 canvas (17 GB)
                w.setMaximumSize(screen.width(), screen.height())
                w.resize(*size)
                frame = w.frameGeometry()
                w.move(screen.right() - frame.width() + 1, screen.top())
            except AttributeError:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="strait", help="for the latitude and longitude of the world origin")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = AoaMap(args.world)
    try:
        while not node.last:  # no window until a ship's bearing arrives (the ships start transmitting late)
            rclpy.spin_once(node, timeout_sec=0.5)
        node.open_window()
        while plt.fignum_exists(node.fig.number):
            for _ in range(500):  # the IMU alone is 100 Hz of sim time: drain the queue each frame
                rclpy.spin_once(node, timeout_sec=0.0)
            node.draw()
            plt.pause(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

"""Live map: where the drone could be, from the signal strength (RSSI) of the ships' AIS packets alone.

Started by `sim.launch.py` with the ships and demo:=true (the strait world), or by hand while the simulator runs:
    python3 sim/nodes/rssi_map.py --world strait

The math, per ship (its position comes from its own AIS message):
    distance  d = 10^((EIRP - RSSI - 20 log10(4 pi / lambda)) / 20)       free space, inverted
    EIRP is what a Class A installation nominally sends: 41 dBm, +2.15 dBi antenna, -2 dB cable.
    The receiver cannot know the real fading, so the distance is uncertain by SIGMA_DB:
    the ring d * 10^(-SIGMA_DB/20) ... d * 10^(+SIGMA_DB/20) holds the drone with about 68 % probability.
Nothing is drawn until three different ships have been heard within the last 30 s (MIN_SHIPS): with fewer, the
rings do not pin the drone down, so the map stays empty and says how many ships it has.
Where could the drone be? Every point of a 10 m grid is scored against all three rings:
    cost(x) = sum over ships of (20 log10(|x - ship| / d) / SIGMA_DB)^2
and the regions cost - min < 2.30 and < 6.18 are drawn (68 % and 95 % for two unknowns), with the best point.

The bearing estimate (nodes/rf_nav.py) and the true position are drawn too, for comparison only: the RSSI fix uses
neither. Nothing here feeds the navigation.
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib
import numpy as np
from matplotlib.patches import Wedge
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt  # noqa: E402

import rf_model as rf  # noqa: E402

WORLDS = Path(__file__).resolve().parents[1] / "worlds"
# What a nominal Class A installation sends: 41 dBm, +2.15 dBi antenna, -2 dB cable. All the ships here are Class A;
# the detections do not say the class, so a Class B ship (33 dBm) would look about 8 dB, 2.5 times, farther away.
EIRP_DBM = 41.0 + 2.15 - 2.0
SIGMA_DB = math.sqrt(3.0 ** 2 + 1.0 ** 2 + 2.0 ** 2)  # fading, RSSI error, model error
MAX_AGE_S = 30.0          # use each ship's last packet if it is this recent
MIN_SHIPS = 3             # hard requirement: nothing is drawn until this many ships have been heard recently
AREA = (-1100.0, 2000.0, -900.0, 1250.0)  # x0, x1, y0, y1 of the map, world metres
GRID_M = 10.0
COLOURS = ["tab:blue", "tab:orange", "tab:green", "tab:purple", "tab:brown"]


def rssi_to_distance(rssi_dbm, freq_hz, eirp_dbm):
    return 10 ** ((eirp_dbm - rssi_dbm - 20 * math.log10(4 * math.pi / rf.wavelength(freq_hz))) / 20)


class RssiMap(Node):
    def __init__(self, world):
        super().__init__("rssi_map")
        self.origin = rf.world_origin(WORLDS / f"{world}.sdf")
        self.last = {}            # mmsi: last detection
        self.truth = self.nav = None
        self.create_subscription(String, "/rf/detections", self.on_detection, 50)
        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: setattr(self, "truth", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/rf_nav/odom", lambda m: setattr(self, "nav", m), 10)
        gx = np.arange(AREA[0], AREA[1], GRID_M)
        gy = np.arange(AREA[2], AREA[3], GRID_M)
        self.gx, self.gy = np.meshgrid(gx, gy)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7.2, 5.0), dpi=100)
        try:  # the top-right slot of the browser desktop
            self.fig.canvas.manager.set_window_title("RSSI map")
            self.fig.canvas.manager.window.setGeometry(1200, 30, 720, 470)
        except AttributeError:
            pass

    def on_detection(self, m):
        d = json.loads(m.data)
        self.last[d["mmsi"]] = d

    def draw(self):
        ax = self.ax
        ax.clear()
        t_now = max((d["t"] for d in self.last.values()), default=0.0)
        fresh = [d for d in self.last.values() if t_now - d["t"] <= MAX_AGE_S]
        self.frame(ax)
        if len(fresh) < MIN_SHIPS:
            heard = ", ".join(f"{str(d['mmsi'])[-3:]} ({t_now - d['t']:.0f} s ago)"
                              for d in sorted(fresh, key=lambda d: d["mmsi"])) or "none yet"
            ax.set_title(f"RSSI only: waiting for {MIN_SHIPS} ships", fontsize=10)
            ax.text(0.5, 0.5, f"Heard {len(fresh)} of {MIN_SHIPS} ships in the last {MAX_AGE_S:.0f} s\n"
                    f"{heard}\n\nNothing is shown until {MIN_SHIPS} ships are heard:\n"
                    "fewer rings do not pin the position down.",
                    transform=ax.transAxes, ha="center", va="center", fontsize=10,
                    bbox=dict(facecolor="white", alpha=0.9, lw=0.5, edgecolor="0.6"))
            self.show()
            return
        lines = []
        cost = np.zeros_like(self.gx)
        for i, d in enumerate(sorted(fresh, key=lambda d: d["mmsi"])):
            c = COLOURS[i % len(COLOURS)]
            sx, sy = rf.latlon_to_enu(d["lat"], d["lon"], *self.origin)
            dist = rssi_to_distance(d["rssi_dbm"], d["freq_hz"], EIRP_DBM)
            lo, hi = dist * 10 ** (-SIGMA_DB / 20), dist * 10 ** (SIGMA_DB / 20)
            ax.add_patch(Wedge((sx, sy), hi, 0, 360, width=hi - lo, color=c, alpha=0.15, lw=0))
            ax.add_patch(plt.Circle((sx, sy), dist, fill=False, color=c, lw=1.5))
            ax.plot(sx, sy, "^", color=c, ms=9, mec="k")
            ax.annotate(str(d["mmsi"])[-3:], (sx, sy), textcoords="offset points", xytext=(6, 6), fontsize=7)
            lines.append(f"{str(d['mmsi'])[-3:]}: {d['rssi_dbm']:.0f} dBm -> {dist:.0f} m "
                         f"({lo:.0f}-{hi:.0f})")
            r = np.maximum(np.hypot(self.gx - sx, self.gy - sy), 1.0)
            cost += (20 * np.log10(r / dist) / SIGMA_DB) ** 2

        title = f"RSSI only, {len(fresh)} ships: where the drone could be"
        dc = cost - cost.min()
        ax.contourf(self.gx, self.gy, dc, levels=[0, 2.30, 6.18], colors=["tab:red", "tab:red"],
                    alpha=0.35, antialiased=True)
        ax.contour(self.gx, self.gy, dc, levels=[2.30, 6.18], colors="tab:red", linewidths=[1.2, 0.6])
        k = np.unravel_index(np.argmin(cost), cost.shape)
        best = (self.gx[k], self.gy[k])
        ax.plot(*best, "x", color="tab:red", ms=12, mew=3, label="RSSI fix (most likely)")
        if self.truth:
            p = self.truth.pose.pose.position
            title += f"   error {math.hypot(best[0] - p.x, best[1] - p.y):.0f} m"
        if self.nav:
            p = self.nav.pose.pose.position
            ax.plot(p.x, p.y, "o", color="k", mfc="none", ms=9, mew=2, label="bearing fix (rf_nav)")
        if self.truth:
            p = self.truth.pose.pose.position
            ax.plot(p.x, p.y, "*", color="gold", ms=14, mec="k", label="true position")

        ax.set_title(title, fontsize=10)
        ax.legend(loc="lower left", fontsize=7, framealpha=0.8)
        ax.text(0.01, 0.99, "\n".join(lines) + f"\nring = distance +-{SIGMA_DB:.1f} dB; red = 68 % / 95 %",
                transform=ax.transAxes, va="top", fontsize=7, family="monospace",
                bbox=dict(facecolor="white", alpha=0.8, lw=0))
        self.show()

    def frame(self, ax):
        ax.set_xlim(AREA[0], AREA[1])
        ax.set_ylim(AREA[2], AREA[3])
        ax.set_aspect("equal")
        ax.set_xlabel("east, m", fontsize=8)
        ax.set_ylabel("north, m", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.25)

    def show(self):
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="strait", help="for the latitude and longitude of the world origin")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = RssiMap(args.world)
    try:
        while plt.fignum_exists(node.fig.number):
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.0)
            node.draw()
            plt.pause(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

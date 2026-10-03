"""Live map: where the drone could be, from the angle of arrival (AoA) of the ships' AIS packets alone.

Started by `sim.launch.py` with the ships and demo:=true (the strait world), or by hand while the simulator runs:
    python3 sim/nodes/aoa_map.py --world strait

The math. Each decoded packet gives a ship's reported position s_i and the direction finder's bearing b_i to it,
counter-clockwise from the drone's nose. The drone's heading psi is unknown without GNSS, so a candidate position x
and heading must satisfy, for every ship,
    atan2(s_i - x) - psi = b_i                                             (one line of position per ship)
Three ships are needed: three bearings, three unknowns (x, y, psi). The bearings arrive at different times, and the
drone turns in between, so each one is first referred to the newest packet's time with the gyro:
    b_i' = b_i - (yaw_gyro(t_now) - yaw_gyro(t_i))
Only packets from the last START_WINDOW_S are used (as rf_nav's first fix); the drone also moves in that time, which
is added to each bearing's uncertainty as START_SIGMA_M across the line, beside the direction finder's own sigma, its
fixed mounting bias and the ship's reported position error (the same sigma as rf_nav):
    sigma_i(x)^2 = azimuth_std^2 + bias^2 + (SHIP_POSITION_SIGMA_M^2 + START_SIGMA_M^2) / |s_i - x|^2
Every point of a 10 m grid is scored with its best-fitting heading (weighted circular mean of the residuals):
    cost(x) = min over psi of  sum_i  (wrap(atan2(s_i - x) - psi - b_i') / sigma_i(x))^2
and the regions cost - min < 2.30 and < 6.18 are drawn (68 % and 95 % for the two position unknowns). The best point
is refined by rf_nav's own resection, and each ship's line of position is drawn with that heading.

rf_nav's filtered estimate and the true position are drawn too, for comparison only: the AoA snapshot uses neither,
and RSSI is not used for the position. Nothing here feeds the navigation.
"""
import argparse
import bisect
import json
import math
from pathlib import Path

import matplotlib
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt  # noqa: E402

import rf_model as rf  # noqa: E402
from rf_nav import (BEARING_BIAS_DEG, SHIP_POSITION_SIGMA_M, START_SIGMA_M, START_WINDOW_S,  # noqa: E402
                    resection, wrap)

WORLDS = Path(__file__).resolve().parents[1] / "worlds"
MIN_SHIPS = 3             # hard requirement: three bearings for three unknowns (x, y, heading)
AREA = (-1100.0, 2000.0, -900.0, 1250.0)  # x0, x1, y0, y1 of the map, world metres
GRID_M = 10.0
GYRO_HISTORY_S = 2 * START_WINDOW_S  # integrated gyro yaw kept for referring bearings to one time
COLOURS = ["tab:blue", "tab:orange", "tab:green", "tab:purple", "tab:brown"]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class AoaMap(Node):
    def __init__(self, world):
        super().__init__("aoa_map")
        self.origin = rf.world_origin(WORLDS / f"{world}.sdf")
        self.last = {}            # mmsi: last detection
        self.truth = self.nav = None
        self.gyro_t, self.gyro_yaw = [], []  # integrated body yaw rate, sim time
        self.create_subscription(String, "/rf/detections", self.on_detection, 50)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.truth_hist = []      # (t, x, y, yaw): the fix is scored against the truth at its own time
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/rf_nav/odom", lambda m: setattr(self, "nav", m), 10)
        gx = np.arange(AREA[0], AREA[1], GRID_M)
        gy = np.arange(AREA[2], AREA[3], GRID_M)
        self.gx, self.gy = np.meshgrid(gx, gy)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7.2, 5.0), dpi=100)
        try:  # the top-right slot of the browser desktop
            self.fig.canvas.manager.set_window_title("AoA map")
            self.fig.canvas.manager.window.setGeometry(1200, 30, 720, 470)
        except AttributeError:
            pass

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
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
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.truth_hist.append((t, p.x, p.y, yaw_of(m.pose.pose.orientation)))
        while self.truth_hist and t - self.truth_hist[0][0] > GYRO_HISTORY_S:
            self.truth_hist.pop(0)

    def truth_at(self, t):
        """True x, y, yaw at sim time t, from the recent history (nearest sample)."""
        if not self.truth_hist:
            return None
        return min(self.truth_hist, key=lambda h: abs(h[0] - t))[1:]

    def gyro_at(self, t):
        return float(np.interp(t, self.gyro_t, self.gyro_yaw)) if self.gyro_t else 0.0

    def on_detection(self, m):
        d = json.loads(m.data)
        self.last[d["mmsi"]] = d

    def draw(self):
        ax = self.ax
        ax.clear()
        t_now = max((d["t"] for d in self.last.values()), default=0.0)
        fresh = sorted((d for d in self.last.values() if t_now - d["t"] <= START_WINDOW_S), key=lambda d: d["mmsi"])
        self.frame(ax)
        if len(fresh) < MIN_SHIPS:
            heard = ", ".join(f"{str(d['mmsi'])[-3:]} ({t_now - d['t']:.0f} s ago)" for d in fresh) or "none yet"
            ax.set_title(f"AoA only: waiting for {MIN_SHIPS} ships", fontsize=10)
            ax.text(0.5, 0.5, f"Heard {len(fresh)} of {MIN_SHIPS} ships in the last {START_WINDOW_S:.0f} s\n"
                    f"{heard}\n\nNothing is shown until {MIN_SHIPS} ships are heard:\n"
                    "with the heading unknown, fewer bearings do not pin the position down.",
                    transform=ax.transAxes, ha="center", va="center", fontsize=10,
                    bbox=dict(facecolor="white", alpha=0.9, lw=0.5, edgecolor="0.6"))
            self.show()
            return

        ships = np.array([rf.latlon_to_enu(d["lat"], d["lon"], *self.origin) for d in fresh])
        yaw_now = self.gyro_at(t_now)
        turned = np.array([yaw_now - self.gyro_at(d["t"]) for d in fresh])  # how far the drone turned since
        bearings = np.array([wrap(d["azimuth_body_rad"] - dpsi) for d, dpsi in zip(fresh, turned)])
        sd2 = np.array([d["azimuth_std_rad"] ** 2 + math.radians(BEARING_BIAS_DEG) ** 2 for d in fresh])
        pos2 = SHIP_POSITION_SIGMA_M ** 2 + START_SIGMA_M ** 2

        # cost over the grid, with the heading that fits best at each point
        px, py = self.gx.ravel(), self.gy.ravel()
        world = np.arctan2(ships[:, 1, None] - py, ships[:, 0, None] - px)        # ship i seen from point
        r2 = np.maximum((ships[:, 0, None] - px) ** 2 + (ships[:, 1, None] - py) ** 2, GRID_M ** 2)
        w = 1 / (sd2[:, None] + pos2 / r2)
        diff = world - bearings[:, None]
        psi = np.arctan2((w * np.sin(diff)).sum(0), (w * np.cos(diff)).sum(0))
        cost = (w * np.angle(np.exp(1j * (diff - psi))) ** 2).sum(0).reshape(self.gx.shape)
        dc = cost - cost.min()
        ax.contourf(self.gx, self.gy, dc, levels=[0, 2.30, 6.18], colors=["tab:red", "tab:red"],
                    alpha=0.35, antialiased=True)
        ax.contour(self.gx, self.gy, dc, levels=[2.30, 6.18], colors="tab:red", linewidths=[1.2, 0.6])

        # refine the best grid point with rf_nav's resection, at the sigmas of that point
        k = int(np.argmin(cost))
        sig = np.sqrt(sd2 + pos2 / r2[:, k])
        s, cov = resection(ships, bearings, sig)
        best, heading = s[:2], s[2]
        ax.plot(*best, "x", color="tab:red", ms=12, mew=3, zorder=6, label="AoA fix (most likely)")

        lines = []
        reach = math.hypot(AREA[1] - AREA[0], AREA[3] - AREA[2])
        for i, d in enumerate(fresh):
            c = COLOURS[i % len(COLOURS)]
            sx, sy = ships[i]
            back = heading + bearings[i] + math.pi  # from the ship, back along the line of position
            sigma = sig[i]  # all of the bearing's uncertainty, at the fix: as in the cost and the resection
            for off, lw, ls in ((0.0, 1.5, "-"), (-2 * sigma, 0.6, "--"), (2 * sigma, 0.6, "--")):
                a = back + off
                ax.plot([sx, sx + reach * math.cos(a)], [sy, sy + reach * math.sin(a)], color=c, lw=lw, ls=ls)
            ax.plot(sx, sy, "^", color=c, ms=9, mec="k")
            ax.annotate(str(d["mmsi"])[-3:], (sx, sy), textcoords="offset points", xytext=(6, 6), fontsize=7)
            lines.append(f"{str(d['mmsi'])[-3:]}: AoA {math.degrees(d['azimuth_body_rad']):+6.1f} deg "
                         f"+-{math.degrees(sigma):.1f}, {t_now - d['t']:2.0f} s ago, "
                         f"gyro {math.degrees(-turned[i]):+5.1f}")

        title = f"AoA only, {len(fresh)} ships: where the drone could be"
        then = self.truth_at(t_now)
        if then:
            herr = math.degrees(wrap(heading - then[2]))
            title += f"   error {math.hypot(best[0] - then[0], best[1] - then[1]):.0f} m, heading {herr:+.1f} deg"
        if self.nav:
            p = self.nav.pose.pose.position
            ax.plot(p.x, p.y, "o", color="k", mfc="none", ms=9, mew=2, label="rf_nav (AoA + gyro, filtered)")
        if self.truth:
            p = self.truth.pose.pose.position
            ax.plot(p.x, p.y, "*", color="gold", ms=14, mec="k", label="true position, now")

        ax.set_title(title, fontsize=10)
        ax.legend(loc="lower left", fontsize=7, framealpha=0.8)
        two_sigma = 2 * math.sqrt(max(cov[0, 0] + cov[1, 1], 0.0))
        ax.text(0.01, 0.99, "\n".join(lines) + f"\nline of position +-2 sigma (dashed); red = 68 % / 95 %"
                f"\nsnapshot 2 sigma {two_sigma:.0f} m; RSSI not used",
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
    node = AoaMap(args.world)
    try:
        while plt.fignum_exists(node.fig.number):
            for _ in range(500):  # the IMU alone is 100 Hz of sim time: drain the queue each frame
                rclpy.spin_once(node, timeout_sec=0.0)
            node.draw()
            plt.pause(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

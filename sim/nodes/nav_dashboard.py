"""Navigation dashboard: the terminal monitor's information in a window, with tabs.

Started by `sim.launch.py` with demo:=true, or by hand while the simulator runs:
    python3 sim/nodes/nav_dashboard.py --world strait [--headline eskf_rf|rf|eskf]

Until the first ship is heard there is no RF navigation display, and the window fills the whole right side of the
desktop, its Overview with the live sensor readings and the AIS receiver's state below the rows. When a ship's
bearing arrives (the ships start transmitting at ais.start_after_s, config/rf.yaml) the RF navigation display opens
in the top right (nodes/aoa_map.py) and this window shrinks to the lower-right corner under it.

Tabs
    Overview      GNSS state; each estimator's position error against the truth, its own 2σ and whether the error is
                  within it, its heading and height error; sensor health
    Navigation    the three estimators side by side (the terminal monitor's NAVIGATION table)
    Sensors       each sensor's rate and latest values against the truth
    AIS           each ship heard: range, angle of arrival and its error, packets decoded

The data and the math are the terminal monitor's (nodes/sensor_monitor.py, whose ROS node this window reuses), so the
two always agree: the error is the horizontal distance from an estimate to the simulation's ground truth, the 2σ bound
is 2·sqrt(var_x + var_y) from the estimate's own covariance, and an estimate is within its bound when the error is
under that 2σ.
"""
import argparse
import math

import rclpy
from PyQt5 import QtCore, QtGui, QtWidgets

from sensor_monitor import ESTIMATORS, FAIR_M, GOOD_M, M_PER_DEG_LAT, Monitor, quat_yaw, stamp

WIDTH = 720               # the RF display's width (aoa_map.WINDOW), in the browser desktop's lower-right corner
RF_HEIGHT = 470           # the RF display's height above it: the dashboard takes the rest of the desktop's height
                          # (before the RF display opens: all of it)
REFRESH_MS = 250          # screen refresh, wall clock
ROW_COLUMNS = (118, 82, 100, 76, 92, 120)  # Overview rows: name, error, bar, 2σ, bound, heading and height (px)
PLACE_AFTER_MS = (300, 1500, 4000)  # place the window again after it appears: the window manager may move it
NOMINAL_HZ = {"imu": 100.0, "baro": 50.0, "cam": 25.0, "gps": 1.0}  # the sensors' rates in the drone model
RATE_OK = 0.8             # a sensor is healthy at this share of its nominal rate or more
AIS_FRESH_S = 30.0        # a ship counts as in view if a packet from it was decoded this recently
NAMES = {key: name for key, _, _, name, _ in ESTIMATORS}
SOURCES = {key: uses for key, _, _, _, uses in ESTIMATORS}

GREEN, AMBER, RED, GREY = "#2e9d57", "#d08a12", "#c8372d", "#8a8f98"
INK, MUTED, CARD, PAGE = "#1d2430", "#5b6472", "#ffffff", "#eef1f5"
STYLE = f"""
QWidget {{ background: {PAGE}; color: {INK}; font-family: 'DejaVu Sans'; font-size: 11px; }}
QTabWidget::pane {{ border: 0; }}
QTabBar::tab {{ background: #dde2e9; padding: 6px 12px; margin-right: 2px; border-top-left-radius: 4px;
               border-top-right-radius: 4px; color: {MUTED}; }}
QTabBar::tab:selected {{ background: {CARD}; color: {INK}; border-bottom: 2px solid {INK}; }}
QFrame#card {{ background: {CARD}; border-radius: 6px; }}
QLabel#caption {{ color: {MUTED}; font-size: 10px; background: transparent; }}
QLabel#big {{ font-size: 24px; font-weight: bold; background: transparent; }}
QLabel#mid {{ font-size: 15px; font-weight: bold; background: transparent; }}
QLabel#banner {{ font-size: 15px; font-weight: bold; color: white; border-radius: 6px; padding: 8px 12px; }}
QLabel {{ background: transparent; }}
QTableWidget {{ background: {CARD}; gridline-color: #e3e7ed; border: 0; }}
QHeaderView::section {{ background: #dde2e9; color: {MUTED}; padding: 4px; border: 0; font-weight: bold; }}
"""


def colour_for(err):
    return GREEN if err < GOOD_M else AMBER if err < FAIR_M else RED


def metres(x):
    return f"{x / 1000:.1f} km" if x >= 1000 else f"{x:.0f} m"


class Feed(Monitor):
    """The terminal monitor's node, without its terminal output: this window draws instead."""

    def draw(self):
        pass


def card(*widgets, stretch=False):
    frame = QtWidgets.QFrame(objectName="card")
    lay = QtWidgets.QVBoxLayout(frame)
    lay.setContentsMargins(12, 8, 12, 8)
    lay.setSpacing(2)
    for w in widgets:
        lay.addWidget(w)
    if stretch:
        lay.addStretch(1)
    return frame


def label(text="", name=None, wrap=False):
    w = QtWidgets.QLabel(text)
    if name:
        w.setObjectName(name)
    w.setWordWrap(wrap)
    return w


class Bar(QtWidgets.QWidget):
    """Error on a log scale, 1 m to 1 km, coloured like the terminal's bar."""

    def __init__(self):
        super().__init__()
        self.err = None
        self.setFixedHeight(10)

    def set(self, err):
        self.err = err
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        r = self.rect()
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor("#e3e7ed"))
        p.drawRoundedRect(r, 4, 4)
        if self.err is not None:
            share = min(max(math.log10(max(self.err, 1.0)) / 3.0, 0.02), 1.0)
            p.setBrush(QtGui.QColor(colour_for(self.err)))
            p.drawRoundedRect(QtCore.QRectF(0, 0, r.width() * share, r.height()), 4, 4)


class Chip(QtWidgets.QLabel):
    """A sensor's health: name, tick or cross, and its rate."""

    def set(self, name, ok, detail):
        col = GREEN if ok is True else RED if ok is False else GREY
        mark = "✓" if ok is True else "✗" if ok is False else "–"
        self.setText(f"<b>{name}</b> {mark}<br><span style='font-size:9px'>{detail}</span>")
        self.setStyleSheet(f"background:{col}; color:white; border-radius:5px; padding:4px 6px;")
        self.setAlignment(QtCore.Qt.AlignCenter)


class Dashboard(QtWidgets.QWidget):
    def __init__(self, feed, headline):
        super().__init__()
        self.feed, self.headline = feed, headline
        self.setWindowTitle("Navigation dashboard")
        self.setStyleSheet(STYLE)
        self.tabs = tabs = QtWidgets.QTabWidget()
        tabs.addTab(self.overview_tab(), "Overview")
        tabs.addTab(self.table_tab(), "Navigation")
        self.repeated = [tabs.addTab(self.sensors_tab(), "Sensors"),  # on the Overview too while the window is tall
                         tabs.addTab(self.ais_tab(), "AIS")]
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.addWidget(tabs)
        self.tall = None          # True while the window fills the right side (no ship heard yet)
        self.set_tall(True)
        self.timer = QtCore.QTimer(self, timeout=self.tick)
        self.timer.start(REFRESH_MS)

    # --- size -----------------------------------------------------------------------------------------------
    def set_tall(self, tall):
        """The whole right side before the RF display opens; the lower-right corner, under it, after."""
        if tall == self.tall:
            return
        self.tall = tall
        self.extra.setVisible(tall)
        for i in self.repeated:  # a tab only for what the Overview does not show
            self.tabs.setTabVisible(i, not tall)
        if tall and self.tabs.currentIndex() in self.repeated:
            self.tabs.setCurrentIndex(0)
        screen = QtWidgets.QApplication.primaryScreen().availableGeometry()
        self.setMaximumSize(screen.width(), screen.height())  # a maximise must not ask Qt for an enormous canvas
        for ms in (0,) + PLACE_AFTER_MS:  # again later: the window manager may move it after a resize
            QtCore.QTimer.singleShot(ms, lambda: self.place(screen))

    def place(self, screen):
        frame_h = self.frameGeometry().height() - self.height()   # the title bar the window manager adds
        height = screen.height() - frame_h - (0 if self.tall else RF_HEIGHT + frame_h)
        self.resize(WIDTH, height)
        frame = self.frameGeometry()
        self.move(screen.right() - frame.width() + 1, screen.bottom() - frame.height() + 1)

    # --- tabs -----------------------------------------------------------------------------------------------
    def overview_tab(self):
        page = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(page)
        lay.setSpacing(6)
        self.banner = label("", "banner")
        lay.addWidget(self.banner)
        self.cards = {}
        # one row per estimator, the headline first
        for key in [self.headline] + [k for k, *_ in ESTIMATORS if k != self.headline]:
            c = dict(err=label("—", "big"), bar=Bar(), sigma=label("—", "mid"), bound=label("", "mid"),
                     att=label("—", "mid"))
            c["bar"].setFixedWidth(100)
            frame = QtWidgets.QFrame(objectName="card")
            if key == self.headline:
                frame.setStyleSheet(f"QFrame#card {{ border: 2px solid {INK}; }}")
            g = QtWidgets.QGridLayout(frame)
            g.setContentsMargins(10, 4, 10, 4)
            g.setHorizontalSpacing(10)
            g.setVerticalSpacing(0)
            name = label(NAMES[key], "mid")
            g.addWidget(name, 0, 0)
            g.addWidget(label(SOURCES[key], "caption"), 1, 0)
            for col, (title, widget) in enumerate((("POSITION ERROR", c["err"]), ("", c["bar"]), ("2σ", c["sigma"]),
                                                   ("BOUND", c["bound"]), ("HEADING · HEIGHT ERR.", c["att"])), start=1):
                g.addWidget(label(title, "caption"), 0, col)
                g.addWidget(widget, 1, col)
            for col, width in enumerate(ROW_COLUMNS):  # the same widths in every row, so the columns line up
                g.setColumnMinimumWidth(col, width)
            g.setColumnStretch(len(ROW_COLUMNS) - 1, 1)
            self.cards[key] = c
            lay.addWidget(frame, 1)
        chips = QtWidgets.QHBoxLayout()
        self.chips = {k: Chip() for k in ("imu", "baro", "cam", "gps", "ais")}
        for c in self.chips.values():
            chips.addWidget(c)
        holder = QtWidgets.QFrame(objectName="card")
        hl = QtWidgets.QVBoxLayout(holder)
        hl.setContentsMargins(10, 6, 10, 8)
        hl.addWidget(label("SENSORS", "caption"))
        hl.addLayout(chips)
        lay.addWidget(holder)
        # shown only while the window is tall, before the RF navigation display opens
        self.ais_state = label("", "mid")
        self.live = QtWidgets.QLabel()
        self.live.setTextFormat(QtCore.Qt.RichText)
        self.live.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        self.live.setStyleSheet("font-size: 13px;")
        self.extra = card(label("AIS RECEIVER", "caption"), self.ais_state, label("", "caption"),
                          label("LIVE SENSORS", "caption"), self.live, stretch=True)
        lay.addWidget(self.extra, 4)
        return page

    def table_tab(self):
        page = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(page)
        self.table = QtWidgets.QTableWidget(len(ESTIMATORS), 7)
        self.table.setHorizontalHeaderLabels(["Error", "2σ", "Bound", "RMS denied", "Height", "Heading", "Updates"])
        self.table.setVerticalHeaderLabels([f"{name}\n{uses}" for _, _, _, name, uses in ESTIMATORS])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        lay.addWidget(self.table)
        lay.addWidget(label("Error and height, heading: against simulation truth · 2σ: 2·√(var x + var y) from the "
                            "estimator's covariance · RMS denied: since the GNSS cut · ✓ accepted ✗ rejected",
                            "caption", wrap=True))
        return page

    def sensors_tab(self):
        page = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(page)
        self.sensors = QtWidgets.QLabel()
        self.sensors.setTextFormat(QtCore.Qt.RichText)
        self.sensors.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        lay.addWidget(card(self.sensors, stretch=True))
        return page

    def ais_tab(self):
        page = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(page)
        self.ais = QtWidgets.QTableWidget(0, 6)
        self.ais.setHorizontalHeaderLabels(["Ship", "Range", "AoA (nose)", "AoA error", "AoA σ", "Packets"])
        self.ais.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.ais.verticalHeader().setVisible(False)
        self.ais.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        lay.addWidget(self.ais)
        lay.addWidget(label("AoA: angle of arrival from the drone's nose, pseudo-Doppler array · AoA error: against "
                            "simulation truth · range: simulation truth, not measured · packets: decoded / sent",
                            "caption", wrap=True))
        return page

    # --- refresh --------------------------------------------------------------------------------------------
    def tick(self):
        for _ in range(400):  # drain the ROS queue: the IMU alone is 100 messages per simulated second
            rclpy.spin_once(self.feed, timeout_sec=0.0)
        f = self.feed
        self.set_tall(not any(r["det"] for r in f.rf.values()))
        self.update_banner()
        rows = self.navigation_rows()
        self.update_overview(rows)
        self.update_table(rows)
        self.update_sensors()
        self.update_ais()

    def update_banner(self):
        f = self.feed
        if f.gnss_on is None:
            text, col = "GNSS  —", GREY
        elif f.gnss_on:
            text, col = "GNSS AVAILABLE", GREEN
        else:
            since = f"   T+{f.sim_t - f.cut_t:.0f} s" if f.cut_t is not None and f.sim_t is not None else ""
            text, col = f"GNSS DENIED{since}", RED
        if f.sim_t is not None:
            text += f"      sim {f.sim_t:.0f} s"
        self.banner.setText(text)
        self.banner.setStyleSheet(f"background:{col};")

    def navigation_rows(self):
        """Each estimator's error, own 2σ, height and heading error against the truth, as the terminal computes."""
        f = self.feed
        rows = {}
        if not f.truth:
            return rows
        tp, tyaw = f.truth.pose.pose.position, quat_yaw(f.truth.pose.pose.orientation)
        for key, *_ in ESTIMATORS:
            e = f.est[key]
            if e.odom is None:
                continue
            p, c = e.odom.pose.pose.position, e.odom.pose.covariance
            err = math.hypot(p.x - tp.x, p.y - tp.y)
            if f.gnss_on is False:
                e.sq, e.n = e.sq + err * err, e.n + 1
            rows[key] = dict(err=err, two_sigma=2 * math.sqrt(max(c[0] + c[7], 0.0)),
                             rms=math.sqrt(e.sq / e.n) if e.n else None,
                             height=p.z - tp.z if key != "rf" else None,
                             heading=(math.degrees(quat_yaw(e.odom.pose.pose.orientation) - tyaw) + 180) % 360 - 180,
                             counts=e.counts)
        return rows

    def update_overview(self, rows):
        for key, c in self.cards.items():
            r = rows.get(key)
            if r is None:
                c["err"].setText("—")
                c["sigma"].setText("acquiring" if key == "rf" else "—")
                continue
            c["err"].setText(metres(r["err"]))
            c["err"].setStyleSheet(f"color:{colour_for(r['err'])};")
            c["bar"].set(r["err"])
            within = r["err"] <= r["two_sigma"]
            c["sigma"].setText(f"±{metres(r['two_sigma'])}")
            c["bound"].setText("WITHIN" if within else "EXCEEDED")
            c["bound"].setStyleSheet(f"color:{GREEN if within else AMBER};")
            height = f" · {r['height']:+.1f} m" if r["height"] is not None else " · —"
            c["att"].setText(f"{r['heading']:+.1f}°{height}")
        self.update_chips()

    def update_chips(self):
        f = self.feed
        hz = {k: v.hz() for k, v in f.rates.items()}
        for key, name in (("imu", "IMU"), ("baro", "BARO"), ("cam", "CAMERA")):
            ok = hz[key] >= RATE_OK * NOMINAL_HZ[key] if f.sim_t else None
            self.chips[key].set(name, ok, f"{hz[key]:.0f} Hz" if key != "cam" else f"{hz[key]:.0f} fps")
        if f.gnss_on is False:
            self.chips["gps"].set("GPS", False, "denied")
        else:
            gps_ok = bool(f.gps) and hz["gps"] >= RATE_OK * NOMINAL_HZ["gps"]
            self.chips["gps"].set("GPS", gps_ok if f.gps else None, f"{hz['gps']:.1f} Hz")
        heard = sum(1 for r in f.rf.values() if r["det"] and f.sim_t is not None and f.sim_t - r["det"]["t"] <= AIS_FRESH_S)
        self.chips["ais"].set("AIS", heard >= 3 if f.rf else None, f"{heard} ship{'s' if heard != 1 else ''}")

    def update_table(self, rows):
        for i, (key, *_) in enumerate(ESTIMATORS):
            r = rows.get(key)
            cells = ["—"] * 7
            colours = [None] * 7
            if r:
                within = r["err"] <= r["two_sigma"]
                cells = [metres(r["err"]), metres(r["two_sigma"]), "✓ yes" if within else "✗ no",
                         metres(r["rms"]) if r["rms"] is not None else "—",
                         f"{r['height']:+.1f} m" if r["height"] is not None else "—", f"{r['heading']:+.1f}°",
                         "  ".join(f"{s} {a}✓" + (f" {x}✗" if x else "") for s, (a, x) in r["counts"].items())
                         or (f"bearings {sum(v['decoded'] for v in self.feed.rf.values())}" if key == "rf" else "—")]
                colours[0] = colour_for(r["err"])
                colours[2] = GREEN if within else AMBER
            for j, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                if colours[j]:
                    item.setForeground(QtGui.QColor(colours[j]))
                    item.setFont(QtGui.QFont("DejaVu Sans", 10, QtGui.QFont.Bold))
                self.table.setItem(i, j, item)

    def update_sensors(self):
        f = self.feed
        hz = {k: v.hz() for k, v in f.rates.items()}
        out = []
        tz = None
        if f.truth:
            p, v = f.truth.pose.pose.position, f.truth.twist.twist.linear
            tz = p.z
            out.append(f"<b>Ground truth</b> <span style='color:{MUTED}'>(simulation reference, {hz['truth']:.0f} Hz)"
                       f"</span><br>position E {p.x:.1f} m · N {p.y:.1f} m · height {p.z:.1f} m · speed "
                       f"{math.hypot(v.x, v.y, v.z):.1f} m/s")
        if f.imu:
            w, a = f.imu.angular_velocity, f.imu.linear_acceleration
            out.append(f"<b>IMU</b> <span style='color:{MUTED}'>({hz['imu']:.0f} Hz)</span><br>rotation rate "
                       f"x {w.x:+.4f} · y {w.y:+.4f} · z {w.z:+.4f} rad/s<br>acceleration x {a.x:+.2f} · y {a.y:+.2f}"
                       f" · z {a.z:+.2f} m/s²")
        if f.baro and f.p0:
            alt = 44330.0 * (1.0 - (f.baro.fluid_pressure / f.p0) ** (1 / 5.255))
            err = f" · error {alt - tz:+.2f} m" if tz is not None else ""
            out.append(f"<b>Barometer</b> <span style='color:{MUTED}'>({hz['baro']:.0f} Hz)</span><br>pressure "
                       f"{f.baro.fluid_pressure:.1f} Pa · altitude {alt:.1f} m above start{err}")
        if f.gps and hz["gps"] > 0 and f.sim_t is not None and f.sim_t - stamp(f.gps) < 3:
            e = (f.gps.longitude - f.lon0) * M_PER_DEG_LAT * math.cos(math.radians(f.lat0))
            n = (f.gps.latitude - f.lat0) * M_PER_DEG_LAT
            err = ""
            if f.truth:
                tp = f.truth.pose.pose.position
                err = f" · error {math.hypot(e - tp.x, n - tp.y):.1f} m"
            out.append(f"<b>GPS</b> <span style='color:{MUTED}'>({hz['gps']:.1f} Hz)</span><br>lat "
                       f"{f.gps.latitude:.6f} · lon {f.gps.longitude:.6f}{err}")
        else:
            out.append(f"<b>GPS</b><br><span style='color:{RED}'>no fix (denied)</span>" if f.gnss_on is False else
                       f"<b>GPS</b><br><span style='color:{MUTED}'>waiting for a fix</span>")
        if f.info:
            fp = f" · ground footprint {2 * tz:.0f} m" if tz and tz > 0 else ""  # 90° field of view
            out.append(f"<b>Down camera</b> <span style='color:{MUTED}'>({hz['cam']:.0f} fps)</span><br>"
                       f"{f.info.width}×{f.info.height} px{fp}")
        html = "<br><br>".join(out) or "Waiting for the sensors…"
        self.sensors.setText(html)
        if self.tall:
            self.live.setText(html)
            sent = sum(r["sent"] for r in f.rf.values())
            self.ais_state.setText("Listening on 161.975 and 162.025 MHz · no ship heard yet" +
                                   (f" ({sent} packets too weak to decode)" if sent else ""))

    def update_ais(self):
        f = self.feed
        ships = sorted(f.rf.items())
        self.ais.setRowCount(len(ships))
        for i, (mmsi, r) in enumerate(ships):
            t, d = r.get("truth"), r["det"]
            aoa = err = sigma = "—"
            if d:
                aoa = f"{math.degrees(d['azimuth_body_rad']):+.1f}°"
                sigma = f"{math.degrees(d['azimuth_std_rad']):.1f}°"
                if t and abs(d["t"] - t["t"]) < 1e-6:  # the truth of the same packet
                    err = f"{math.degrees((d['azimuth_body_rad'] - t['azimuth_body_rad'] + math.pi) % (2 * math.pi) - math.pi):+.1f}°"
            cells = [r["ship"].replace("_", " ").title(), f"{t['range_m'] / 1000:.2f} km" if t else "—", aoa, err,
                     sigma, f"{r['decoded']} / {r['sent']}"]
            for j, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.ais.setItem(i, j, item)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="strait", help="the world that runs, for the GPS origin")
    ap.add_argument("--headline", default="eskf_rf", choices=[k for k, *_ in ESTIMATORS],
                    help="the estimator the Overview leads with")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    app = QtWidgets.QApplication([])
    window = Dashboard(Feed(args.world), args.headline)
    window.show()
    try:
        app.exec_()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

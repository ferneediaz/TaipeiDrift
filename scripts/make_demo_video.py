"""Turn the desktop frames grabbed by sim/scripts/capture_desktop.py into the demo video.

The frames are evenly spaced in simulated time (frames.csv gives each frame's time), so the video plays at flight
speed whatever the simulator's own pace was. Parts of the flight can be played faster:

    python scripts/make_demo_video.py outputs/demo/take1 --out outputs/demo/take1/pitch.mp4 \
        --from 4 --speed 0:19:4 19:85:1.5 85:999:4 --insert outputs/demo/clips/felix_position_video.mp4

--speed START:END:FACTOR, in simulated seconds; anything not listed plays at 1x. --from / --to trim the flight.

If the run folder holds estimators.csv (sim/scripts/log_two_estimators.py), the video gets
  - a caption bar that says in plain words what happens in each part of the flight, and
  - a small map with the true path, our estimate, and the estimate of a filter that has only the inertial sensors
    and the barometer ("without us"), each with its distance from the truth in metres at that moment.
Every number on screen is read from that log, so it belongs to the flight shown. --plain leaves both out.
--insert CLIP appends a clip of real data, sped up by --insert-speed, under its own caption.
"""
import argparse
import csv
import json
import math
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
FPS = 30
W, H = 1920, 1080
BAR = 130                                    # caption bar under the 3D view: covers the simulator's buttons
VIEW_RIGHT = 1198                            # the 3D view ends here; the dashboard is to its right
TASK_BAR = 23                                # the desktop's task bar at the very bottom, painted over
MAP_W, MAP_H = 400, 246                      # the small map, pixels
MAP_AREA = (-130.0, 650.0, -185.0, 295.0)    # east from, east to, north from, north to; metres
PAD_B = (480.0, 110.0)                       # the second helipad of the strait world (sim/scripts/make_islands.py)
COAST_A_M, COAST_B_M = 183.0, 419.0          # metres along the route: island A ends, island B begins
WHITE, GREY, DARK = (255, 255, 255), (198, 206, 218), (15, 19, 27)
GREEN, RED, LAND, SEA = (62, 201, 110), (240, 82, 70), (78, 100, 72), (24, 46, 72)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    names = (["/System/Library/Fonts/Supplemental/Arial Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"]
             if bold else ["/System/Library/Fonts/Supplemental/Arial.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"])
    for name in names:
        if Path(name).exists():
            return ImageFont.truetype(name, size)
    return ImageFont.load_default()


def fitted(draw: ImageDraw.ImageDraw, text: str, size: int, bold: bool, max_width: float) -> ImageFont.FreeTypeFont:
    """The largest font up to `size` in which `text` fits into `max_width` pixels."""
    while size > 14 and draw.textlength(text, font=font(size, bold)) > max_width:
        size -= 1
    return font(size, bold)


class Flight:
    """The logged flight: the true path, two estimates with their errors, and the moments the captions hang on."""

    def __init__(self, log: Path, ours: str, baseline: str, ships: str = "ships"):
        rows = list(csv.DictReader(log.open()))

        def track(name: str) -> dict:
            r = [x for x in rows if x["name"] == name]
            if not r:
                raise SystemExit(f"{log} has no rows named '{name}'")
            d = {k: np.array([float(x[c]) for x in r]) for k, c in (("t", "t"), ("x", "est_x"), ("y", "est_y"),
                 ("gx", "gt_x"), ("gy", "gt_y"), ("gz", "gt_z"), ("gnss", "gnss_available"))}
            d["err"] = np.hypot(d["x"] - d["gx"], d["y"] - d["gy"])
            return d

        self.ours, self.base = track(ours), track(baseline)
        o = self.ours
        along = (o["gx"] * PAD_B[0] + o["gy"] * PAD_B[1]) / math.hypot(*PAD_B)

        def first(mask: np.ndarray) -> float:
            return float(o["t"][np.argmax(mask)]) if mask.any() else float("inf")

        self.lost = first(o["gnss"] == 0)
        self.coast_a, self.coast_b = first(along > COAST_A_M), first(along > COAST_B_M)
        self.pad = first(along > math.hypot(*PAD_B) - 15.0)
        # the first position from the ships' bearings after the drone has left island A (else 8 s after the coast)
        fixes = [float(x["t"]) for x in rows if x["name"] == ships and float(x["t"]) >= self.coast_a]
        self.first_fix = min(fixes[0], self.coast_b) if fixes else self.coast_a + 8.0
        # how far off the ships' bearings alone were over the water, for the caption
        radio = [math.hypot(float(x["est_x"]) - float(x["gt_x"]), float(x["est_y"]) - float(x["gt_y"]))
                 for x in rows if x["name"] == ships and self.coast_a <= float(x["t"]) < self.coast_b]
        self.radio_median = float(np.median(radio)) if radio else None
        blind = (o["t"] >= self.lost) & (o["t"] <= self.pad)
        self.worst = float(o["err"][blind].max()) if blind.any() else float("nan")
        self.median = float(np.median(o["err"][blind])) if blind.any() else float("nan")
        self.base_at_pad = self.error(self.base, self.pad)
        self.ours_at_pad = self.error(self.ours, self.pad)

    @staticmethod
    def error(track: dict, t: float) -> float:
        return float(np.interp(t, track["t"], track["err"]))

    def captions(self) -> list[tuple[float, list[str]]]:
        """(from this simulated second on, [headline, further lines])."""
        return [
            (0.0, ["Take-off with satellite navigation (GNSS)",
                   "Simulated flight: 490 m from one island to the next, three ships in the strait"]),
            (self.lost, ["GNSS is lost. The drone keeps navigating on its own.",
                         "Over land a downward camera and a range finder measure its speed over the ground"]),
            (self.coast_a, ["Over open water the camera finds little to hold on to",
                            "The drone listens for the ships' AIS radio: the RF window opens at the top right"]),
            (self.first_fix, ["Bearings to three ships give a position that does not drift",
                              "Each fix is rough, " + ("some tens of metres" if self.radio_median is None else
                                                       f"about {self.radio_median:.0f} m here")
                              + ", but its error does not grow with time"]),
            ((self.first_fix + self.coast_b) / 2, ["Camera, inertial sensors and radio are fused into one estimate",
                                                   "Dashboard, first row: the fused estimate and its distance from "
                                                   "the true position"]),
            (self.coast_b, ["Land again: the camera has ground to track",
                            "Approach to the helipad on the second island"]),
            (self.pad, [f"{self.pad - self.lost:.0f} s without GNSS: our estimate was typically {self.median:.0f} m off, "
                        f"{self.worst:.0f} m at worst",
                        "Red line: the same flight with the inertial sensors alone, no camera and no radio",
                        "The autopilot flies on the simulator's true position; our estimate is scored against it"]),
        ]


def base_map() -> Image.Image:
    """Land and sea of the strait world from its height map (sim/maps/islands_dem.tif, 1 m per pixel)."""
    try:
        meta = json.loads((ROOT / "sim/maps/islands_dem.json").read_text())["pixel_centres_world_m"]
        dem = np.array(Image.open(ROOT / "sim/maps/islands_dem.tif"))
    except FileNotFoundError:
        return Image.new("RGB", (MAP_W, MAP_H), SEA)
    x0, x1, y0, y1 = MAP_AREA
    cols = np.clip(np.round(np.linspace(x0, x1, MAP_W) - meta["x_first"]).astype(int), 0, dem.shape[1] - 1)
    rows = np.clip(np.round(meta["y_top"] - np.linspace(y1, y0, MAP_H)).astype(int), 0, dem.shape[0] - 1)
    land = dem[np.ix_(rows, cols)] > 0
    return Image.fromarray(np.where(land[..., None], LAND, SEA).astype(np.uint8))


def map_px(x: np.ndarray, y: np.ndarray) -> list[tuple[float, float]]:
    x0, x1, y0, y1 = MAP_AREA
    return list(zip(((x - x0) / (x1 - x0) * MAP_W).tolist(), ((y1 - y) / (y1 - y0) * MAP_H).tolist()))


class Overlay:
    def __init__(self, flight: Flight):
        self.f, self.caps, self.map = flight, flight.captions(), base_map()
        self.tracks = []                     # (times, map points, colour), drawn in this order
        for d, xk, yk, colour in ((flight.ours, "gx", "gy", WHITE), (flight.base, "x", "y", RED),
                                  (flight.ours, "x", "y", GREEN)):
            keep = np.concatenate(([True], np.diff(np.floor(d["t"] / 0.2)) > 0))      # five points per second
            self.tracks.append((d["t"][keep], map_px(d[xk][keep], d[yk][keep]), colour))
        self.legend_font, self.tag_font, self.note_font = font(21, True), font(20, True), font(16)

    def draw(self, frame: Image.Image, t: float, speed: float) -> None:
        d = ImageDraw.Draw(frame)
        lines = next(lines for start, lines in reversed(self.caps) if t >= start)
        bar(d, lines, GREEN if t < self.f.lost else RED, VIEW_RIGHT)
        d.rectangle([VIEW_RIGHT, H - TASK_BAR, W, H], fill=DARK)
        tag = "SIMULATION, " + ("real time" if speed == 1 else f"played at {speed:g}x")
        tw = d.textlength(tag, font=self.tag_font)
        d.rectangle([VIEW_RIGHT - tw - 34, 82, VIEW_RIGHT - 10, 118], fill=DARK)
        d.text((VIEW_RIGHT - tw - 22, 89), tag, font=self.tag_font, fill=WHITE)

        pad, line = 8, 29
        bw, bh = MAP_W + 2 * pad, MAP_H + 3 * pad + 3 * line
        bx, by = VIEW_RIGHT - bw - 10, H - BAR - bh - 10
        d.rectangle([bx, by, bx + bw, by + bh], fill=DARK)
        # the comparison ends when the drone arrives over the helipad: the landing is the autopilot's part, and the
        # touchdown bump throws the inertial-only estimate about
        arrived = t >= self.f.pad
        t = min(t, self.f.pad)
        small = self.map.copy()
        m = ImageDraw.Draw(small)
        if arrived:
            m.text((8, 6), "at arrival over the helipad", font=self.note_font, fill=GREY)
        for times, pts, colour in self.tracks:
            k = int(np.searchsorted(times, t, side="right"))
            if k >= 2:
                m.line(pts[:k], fill=colour, width=3, joint="curve")
            if k >= 1:
                u, v = pts[k - 1]
                m.ellipse([u - 5, v - 5, u + 5, v + 5], fill=colour, outline=DARK)
        frame.paste(small, (bx + pad, by + pad))
        t_shown = t if arrived else math.floor(t * 2) / 2     # numbers change twice a second, not every frame
        rows = (("True path", WHITE),
                (f"Our estimate: {Flight.error(self.f.ours, t_shown):.0f} m off", GREEN),
                (f"Inertial sensors alone: {Flight.error(self.f.base, t_shown):.0f} m off", RED))
        for i, (text, colour) in enumerate(rows):
            y = by + MAP_H + 2 * pad + i * line
            d.line([bx + pad + 2, y + 13, bx + pad + 30, y + 13], fill=colour, width=4)
            d.text((bx + pad + 40, y), text, font=self.legend_font, fill=colour)


def bar(d: ImageDraw.ImageDraw, lines: list[str], colour: tuple, width: int, tag: str = "") -> None:
    """The caption bar: a headline and one or two further lines, centred in the bar's height."""
    d.rectangle([0, H - BAR, width, H], fill=DARK)
    d.rectangle([0, H - BAR, width, H - BAR + 4], fill=colour)
    sizes = [36] + [24] * (len(lines) - 1)
    gap = 9
    y = H - BAR + 4 + (BAR - 4 - sum(sizes) - gap * (len(lines) - 1)) / 2 - 3
    for text, size in zip(lines, sizes):
        d.text((28, y), text, font=fitted(d, text, size, size == 36, width - 56), fill=WHITE if size == 36 else GREY)
        y += size + gap
    if tag:
        f = font(20, True)
        d.text((width - 28 - d.textlength(tag, font=f), H - 40), tag, font=f, fill=GREY)


def insert_frames(clip: Path, speed: float):
    """The frames of a clip, sped up, scaled to sit above the caption bar on a white page."""
    h = H - BAR
    w = int(round(1920 * h / 1080 / 2)) * 2
    proc = subprocess.Popen(["ffmpeg", "-v", "error", "-i", str(clip), "-vf",
                             f"setpts=PTS/{speed},fps={FPS},scale={w}:{h}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                            stdout=subprocess.PIPE)
    while True:
        raw = proc.stdout.read(w * h * 3)
        if len(raw) < w * h * 3:
            break
        page = Image.new("RGB", (W, H), WHITE)
        page.paste(Image.frombytes("RGB", (w, h), raw), ((W - w) // 2, 0))
        yield page
    proc.wait()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path, help="folder holding frames/ with frames.csv, and estimators.csv")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--speed", nargs="*", default=[], help="START:END:FACTOR in simulated seconds")
    ap.add_argument("--from", dest="start", type=float, default=0.0)
    ap.add_argument("--to", dest="end", type=float, default=1e9)
    ap.add_argument("--hold", type=float, default=3.0, help="seconds the last picture of the flight stays")
    ap.add_argument("--plain", action="store_true", help="no caption bar and no map")
    ap.add_argument("--ours", default="ours", help="name of our estimate in estimators.csv")
    ap.add_argument("--baseline", default="inertial", help="name of the inertial-only estimate in estimators.csv")
    ap.add_argument("--insert", type=Path, help="a clip of real data to append")
    ap.add_argument("--insert-speed", type=float, default=6.0)
    ap.add_argument("--insert-head", default="The camera part on real hardware")
    ap.add_argument("--insert-sub", default="A downward camera on a cart, 64 cm above the floor, tracks its own path "
                                            "around a 5.8 m loop")
    ap.add_argument("--crf", type=int, default=20)
    args = ap.parse_args()
    frames = [(float(r["sim_s"]), int(r["frame"])) for r in csv.DictReader((args.run / "frames/frames.csv").open())]
    frames = [f for f in frames if args.start <= f[0] <= args.end]
    spans = [tuple(float(v) for v in s.split(":")) for s in args.speed]

    def factor(t: float) -> float:
        return next((f for a, b, f in spans if a <= t < b), 1.0)

    # walk through simulated time in steps of factor / FPS and take the frame nearest each step
    times = [f[0] for f in frames]
    chosen, t, k = [], times[0], 0
    while t <= times[-1]:
        while k + 1 < len(times) and abs(times[k + 1] - t) <= abs(times[k] - t):
            k += 1
        chosen.append((t, frames[k][1]))
        t += factor(t) / FPS

    overlay = None
    if not args.plain and (args.run / "estimators.csv").exists():
        flight = Flight(args.run / "estimators.csv", args.ours, args.baseline)
        overlay = Overlay(flight)
        print(f"GNSS lost {flight.lost:.1f} s, coast A {flight.coast_a:.1f} s, first ships' fix "
              f"{flight.first_fix:.1f} s, coast B {flight.coast_b:.1f} s, "
              f"over pad B {flight.pad:.1f} s; from the loss to pad B our estimate: median {flight.median:.1f} m, "
              f"worst {flight.worst:.1f} m, at pad B {flight.ours_at_pad:.1f} m; inertial only at pad B "
              f"{flight.base_at_pad:.0f} m")
    encoder = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                                "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", str(args.crf),
                                "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.out)], stdin=subprocess.PIPE)
    count, last = 0, None
    for t, n in chosen:
        frame = Image.open(args.run / "frames" / f"f{n:05d}.jpg").convert("RGB")
        if frame.size != (W, H):
            frame = frame.resize((W, H))
        if overlay:
            overlay.draw(frame, t, factor(t))
        last = frame.tobytes()
        encoder.stdin.write(last)
        count += 1
    for _ in range(int(args.hold * FPS) if last else 0):
        encoder.stdin.write(last)
        count += 1
    flight_s = count / FPS
    if args.insert:
        for page in insert_frames(args.insert, args.insert_speed):
            bar(ImageDraw.Draw(page), [args.insert_head, args.insert_sub], GREEN, W,
                f"REAL DATA, played at {args.insert_speed:g}x")
            encoder.stdin.write(page.tobytes())
            count += 1
    encoder.stdin.close()
    if encoder.wait() != 0:
        raise SystemExit("ffmpeg failed")
    print(f"{args.out}: {count / FPS:.1f} s ({flight_s:.1f} s for {times[-1] - times[0]:.1f} s of flight"
          + (f", {count / FPS - flight_s:.1f} s of real data" if args.insert else "") + ")")


if __name__ == "__main__":
    main()

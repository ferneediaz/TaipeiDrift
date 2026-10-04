"""Grab the simulator's desktop (X display :1) inside the container, for the demo video.

The browser view (noVNC) is compressed and the simulator runs slower than real time, so a screen recording of
the browser stutters. This saves the desktop itself as JPEG frames, one every 1/30 s of SIMULATED time, each
with its simulated time in frames.csv; scripts/make_demo_video.py turns them into a video at flight speed.

    docker compose exec -d sim bash -ic "python3 sim/scripts/capture_desktop.py --out outputs/demo/run1/frames"

Stop it with pkill -f capture_desktop.py, or let --max-s end it (wall seconds).
"""
import argparse
import csv
import time
from pathlib import Path

import rclpy
from PIL import ImageGrab
from rclpy.node import Node


class Capture(Node):
    def __init__(self, out: Path, fps: float, max_s: float, quality: int):
        super().__init__("capture_desktop", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        out.mkdir(parents=True, exist_ok=True)
        self.out, self.max_s, self.quality = out, max_s, quality
        self.log = (out / "frames.csv").open("w", newline="")
        self.writer = csv.writer(self.log)
        self.writer.writerow(["frame", "sim_s", "wall_s"])
        self.start, self.count = time.time(), 0
        self.create_timer(1.0 / fps, self.grab)  # the node's clock is the simulation's: evenly spaced in flight time

    def grab(self):
        wall = time.time() - self.start
        if wall > self.max_s:
            raise SystemExit
        sim = self.get_clock().now().nanoseconds * 1e-9
        try:
            ImageGrab.grab(xdisplay=":1").save(self.out / f"f{self.count:05d}.jpg", quality=self.quality)
        except OSError as e:  # the display is not up yet
            self.get_logger().warning(f"no frame: {e}", throttle_duration_sec=5.0)
            return
        self.writer.writerow([self.count, f"{sim:.3f}", f"{wall:.3f}"])
        self.log.flush()
        self.count += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--fps", type=float, default=30.0, help="frames per second of simulated time")
    ap.add_argument("--max-s", type=float, default=600.0, help="stop after this many wall seconds")
    ap.add_argument("--quality", type=int, default=88)
    args = ap.parse_args()
    rclpy.init()
    node = Capture(args.out, args.fps, args.max_s, args.quality)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.log.close()


if __name__ == "__main__":
    main()

"""Turn the desktop frames grabbed by sim/scripts/capture_desktop.py into a video.

The frames are evenly spaced in simulated time (frames.csv gives each frame's time), so the video plays at flight
speed whatever the simulator's own pace was. Parts of the flight can be played faster:

    python scripts/make_demo_video.py outputs/demo/run3 --out outputs/demo/run3/cut.mp4 \
        --speed 0:19:4 19:85:1.5 85:999:4

--speed START:END:FACTOR, in simulated seconds; anything not listed plays at 1x. --from / --to trim the flight.
"""
import argparse
import csv
import subprocess
import tempfile
from pathlib import Path

FPS = 30


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path, help="folder holding frames/ with frames.csv")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--speed", nargs="*", default=[], help="START:END:FACTOR in simulated seconds")
    ap.add_argument("--from", dest="start", type=float, default=0.0)
    ap.add_argument("--to", dest="end", type=float, default=1e9)
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
        chosen.append(frames[k][1])
        t += factor(t) / FPS
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as listing:
        for n in chosen:
            listing.write(f"file '{(args.run / 'frames' / f'f{n:05d}.jpg').resolve()}'\nduration {1 / FPS:.6f}\n")
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-f", "concat", "-safe", "0", "-i", listing.name, "-r", str(FPS),
                    "-c:v", "libx264", "-preset", "fast", "-crf", str(args.crf), "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(args.out)], check=True)
    print(f"{args.out}: {len(chosen) / FPS:.1f} s from {times[-1] - times[0]:.1f} s of flight")


if __name__ == "__main__":
    main()

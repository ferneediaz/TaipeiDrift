"""The mentor's experiment: an iPhone camera used as a sun compass, measured against known turns.

Put the photos into data/raw/phone_sun/:

    chessboard/   15 to 20 photos of the chessboard (chessboard_9x6_inner_corners.png on a screen)
    sun/          photos taken with the phone face down on a level table, back camera up, sun in view,
                  turning the phone in 90-degree steps between photos
    compass.csv   optional: file,compass_deg  (what the iPhone Compass app showed for each sun photo)

Then, from the repository root:

    python baseline/scripts/phone_sun_compass.py

It calibrates the camera, finds the sun in every photo, and prints the heading each photo gives.
The turns between photos were multiples of 90 degrees, so the scatter of the headings around those
steps is the error of the sun compass, with no other reference needed. Results go to
outputs/phone_sun/: calibration.json and headings.csv.
"""
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

BASELINE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = BASELINE_DIR.parent
sys.path.insert(0, str(BASELINE_DIR))

from src.sensors.heading import sun_position  # noqa: E402
from src.sensors.sun_compass import calibrate, find_sun, heading_from_sun, read_gray  # noqa: E402

PHOTO_SUFFIXES = {".jpg", ".jpeg", ".heic", ".png"}


def as_jpeg(path: Path, cache: Path) -> Path:
    """iPhone photos are HEIC, which OpenCV cannot read; macOS converts them with sips."""
    if path.suffix.lower() != ".heic":
        return path
    target = cache / (path.stem + ".jpg")
    if not target.is_file():
        cache.mkdir(parents=True, exist_ok=True)
        subprocess.run(["sips", "-s", "format", "jpeg", str(path), "--out", str(target)], check=True, capture_output=True)
    return target


def exif_time_and_place(path: Path) -> tuple[datetime, float | None, float | None]:
    """When and where the photo was taken, from its metadata."""
    from PIL import Image

    exif = Image.open(path).getexif()
    detail = exif.get_ifd(0x8769)  # the Exif block: original time and its time zone
    stamp = detail.get(36867) or exif.get(306)
    offset = detail.get(36881) or "+08:00"
    when = datetime.strptime(stamp, "%Y:%m:%d %H:%M:%S")
    sign = -1 if offset.startswith("-") else 1
    hours, minutes = (int(v) for v in offset.strip("+-").split(":"))
    when = when.replace(tzinfo=timezone(sign * timedelta(hours=hours, minutes=minutes)))
    gps = exif.get_ifd(0x8825)
    if not gps:
        return when, None, None

    def degrees(values, ref) -> float:
        d, m, s = (float(v) for v in values)
        value = d + m / 60 + s / 3600
        return -value if ref in ("S", "W") else value

    return when, degrees(gps[2], gps[1]), degrees(gps[4], gps[3])


def photos(folder: Path) -> list[Path]:
    return sorted(p for p in folder.iterdir() if p.suffix.lower() in PHOTO_SUFFIXES)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--folder", type=Path, default=REPO_ROOT / "data" / "raw" / "phone_sun")
    p.add_argument("--output-dir", type=Path, default=REPO_ROOT / "outputs" / "phone_sun")
    p.add_argument("--lat", type=float, default=25.0174, help="used when a photo carries no GPS (default: NTU, Taipei)")
    p.add_argument("--lon", type=float, default=121.5398)
    args = p.parse_args()
    cache = REPO_ROOT / "data" / "processed" / "phone_sun_jpg"
    args.output_dir.mkdir(parents=True, exist_ok=True)

    boards = [read_gray(as_jpeg(f, cache)) for f in photos(args.folder / "chessboard")]
    model, used = calibrate(boards)
    fx, fy, cx, cy = model.matrix[0, 0], model.matrix[1, 1], model.matrix[0, 2], model.matrix[1, 2]
    print(f"calibration: {used} of {len(boards)} chessboard photos usable, corners fit to {model.rms_px:.2f} px; "
          f"focal length {fx:.0f} x {fy:.0f} px, centre ({cx:.0f}, {cy:.0f}) in a {model.image_size[0]} x {model.image_size[1]} image")
    (args.output_dir / "calibration.json").write_text(json.dumps({
        "matrix": model.matrix.tolist(), "distortion": model.distortion.ravel().tolist(),
        "image_size": model.image_size, "rms_px": model.rms_px, "photos_used": used}, indent=2))

    compass = {}
    if (args.folder / "compass.csv").is_file():
        with open(args.folder / "compass.csv") as fh:
            compass = {row["file"]: float(row["compass_deg"]) for row in csv.DictReader(fh)}

    rows = []
    for f in photos(args.folder / "sun"):
        jpg = as_jpeg(f, cache)
        gray = read_gray(jpg)
        if (gray.shape[1], gray.shape[0]) != model.image_size:
            print(f"{f.name}: image size {gray.shape[1]} x {gray.shape[0]} differs from the calibration; skipped")
            continue
        when, lat, lon = exif_time_and_place(jpg)
        lat, lon = (lat, lon) if lat is not None else (args.lat, args.lon)
        azimuth, elevation = sun_position(lat, lon, when)
        sx, sy, area = find_sun(gray)
        heading, measured_elevation = heading_from_sun(*model.normalised(sx, sy), azimuth)
        rows.append({"file": f.name, "time": when.isoformat(), "lat": lat, "lon": lon, "sun_azimuth": azimuth, "sun_elevation": elevation,
                     "sun_x_px": sx, "sun_y_px": sy, "sun_area_px": area, "measured_elevation": measured_elevation,
                     "heading": heading, "compass": compass.get(f.name)})
    if not rows:
        print("no sun photos found")
        return 1

    first = rows[0]["heading"]
    print(f"\n{'photo':24s} {'time':8s} {'sun elev.':>9s} {'measured':>9s} {'heading':>8s} {'turn':>7s} {'off 90-step':>11s} {'compass':>8s}")
    offsets = []
    for r in rows:
        turn = (r["heading"] - first) % 360
        off = (turn + 45) % 90 - 45  # distance to the nearest multiple of 90 degrees
        offsets.append(off)
        r["turn_from_first"], r["off_90_step"] = turn, off
        comp = "" if r["compass"] is None else f"{r['compass']:.0f}"
        print(f"{r['file']:24s} {r['time'][11:19]:8s} {r['sun_elevation']:9.1f} {r['measured_elevation']:9.1f} {r['heading']:8.1f} "
              f"{turn:7.1f} {off:11.2f} {comp:>8s}")
    offsets = np.array(offsets[1:])
    if len(offsets):
        print(f"\nscatter of the turns around the 90-degree steps: {np.sqrt(np.mean(offsets**2)):.2f} degrees (root mean square), "
              f"largest {np.abs(offsets).max():.2f}")
    print("elevation measured minus true (a level and calibration check): "
          f"median {np.median([r['measured_elevation'] - r['sun_elevation'] for r in rows]):+.2f} degrees")
    with open(args.output_dir / "headings.csv", "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"results in {args.output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

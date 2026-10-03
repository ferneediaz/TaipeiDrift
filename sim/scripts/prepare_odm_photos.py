"""Unpack a drone photo set for OpenDroneMap at a reduced size, keeping the photos' GPS and camera tags.

The Tuniu River sets (OpenDroneMap's ODMdata, photos by Yu-Huang Wang) hold 20-megapixel photos.
Half the width (2736 x 1824) still resolves about 5.5 cm on the ground from 100 m, far finer than our
simulated down camera (about 30 cm per pixel), and needs a quarter of the memory and disk to process.
The EXIF block (GPS, focal length, camera model) and the DJI XMP block (gimbal angles, RTK accuracy)
are copied unchanged; the focal length in millimetres stays true, since the sensor is the same.

    python sim/scripts/prepare_odm_photos.py data/raw/odm_tuniu/tuniu_tw_2.zip data/raw/odm_tuniu/tuniu_tw_2/images
"""
from __future__ import annotations

import argparse
import io
import sys
import zipfile
from pathlib import Path

from PIL import Image


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("zip", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--quality", type=int, default=92)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    zf = zipfile.ZipFile(args.zip)
    names = [n for n in zf.namelist() if n.upper().endswith(".JPG")]
    for k, name in enumerate(names):
        target = args.out / Path(name).name
        if target.exists():
            continue
        image = Image.open(io.BytesIO(zf.read(name)))
        exif, xmp = image.info.get("exif"), image.info.get("xmp")
        small = image.resize((round(image.width * args.scale), round(image.height * args.scale)), Image.Resampling.LANCZOS)
        extra = {"exif": exif} if exif else {}
        if xmp:
            extra["xmp"] = xmp
        small.save(target, "JPEG", quality=args.quality, **extra)
        if k % 50 == 0:
            print(f"{k + 1} of {len(names)}", flush=True)
    print(f"done: {len(names)} photos in {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

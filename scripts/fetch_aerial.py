"""Fetch two aerial images of the same site in Wufeng, Taichung, at reduced resolution.

The originals on OpenAerialMap (CC BY 4.0) are 115 MB and 385 MB. They are cloud-optimised
GeoTIFFs, so one overview level can be read over HTTP without downloading the full file.

Usage: python scripts/fetch_aerial.py [factor] [out_dir]
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("GDAL_DISABLE_READDIR_ON_OPEN", "EMPTY_DIR")
os.environ.setdefault("CPL_VSIL_CURL_ALLOWED_EXTENSIONS", ".tif")
os.environ.setdefault("GDAL_HTTP_TIMEOUT", "60")
os.environ.setdefault("GDAL_HTTP_MAX_RETRY", "5")
os.environ.setdefault("GDAL_HTTP_RETRY_DELAY", "2")

import numpy as np
import rasterio
from rasterio.enums import Resampling
from rasterio.errors import RasterioIOError
from rasterio.windows import Window

STRIP_ROWS = 2048  # output rows per request, so one failed tile only repeats one strip
ATTEMPTS = 5

BASE = "https://oin-hotosm-temp.s3.amazonaws.com"
IMAGES = {
    "wufeng_2018-05-03": f"{BASE}/5b3597c92b6a08001185f8b3/0/5b3597c92b6a08001185f8b4.tif",
    "wufeng_2020-03-23": f"{BASE}/5f70a17f09916800053797b1/0/5f70a17f09916800053797b2.tif",
}


def read_strip(src: rasterio.DatasetReader, window: Window, out_shape: tuple[int, int, int]) -> np.ndarray:
    for attempt in range(1, ATTEMPTS + 1):
        try:
            return src.read(window=window, out_shape=out_shape, resampling=Resampling.average)
        except RasterioIOError:
            if attempt == ATTEMPTS:
                raise
            print(f"  read failed at row {window.row_off}, retry {attempt}", flush=True)
    raise AssertionError("unreachable")


def fetch(name: str, url: str, factor: int, out_dir: Path) -> Path:
    out_path = out_dir / f"{name}_x{factor}.tif"
    if out_path.exists():
        return out_path
    with rasterio.open(f"/vsicurl/{url}") as src:
        height, width = src.height // factor, src.width // factor
        data = np.zeros((src.count, height, width), dtype=src.dtypes[0])
        for row in range(0, height, STRIP_ROWS):
            rows = min(STRIP_ROWS, height - row)
            window = Window(0, row * factor, width * factor, rows * factor)
            data[:, row : row + rows, :] = read_strip(src, window, (src.count, rows, width))
        transform = src.transform * src.transform.scale(src.width / width, src.height / height)
        profile = {
            "driver": "GTiff",
            "dtype": data.dtype,
            "count": src.count,
            "height": height,
            "width": width,
            "crs": src.crs,
            "transform": transform,
            "tiled": True,
            "compress": "jpeg",
            "photometric": "ycbcr",
        }
    with rasterio.open(out_path, "w", **profile) as dst:
        dst.write(data)
    return out_path


if __name__ == "__main__":
    factor = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    out_dir = Path(sys.argv[2] if len(sys.argv) > 2 else "data/raw/aerial")
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, url in IMAGES.items():
        path = fetch(name, url, factor, out_dir)
        with rasterio.open(path) as ds:
            size_mb = path.stat().st_size / 1e6
            print(f"{path}  {ds.width} x {ds.height} px  {ds.res[0] * 100:.1f} cm/px  {size_mb:.0f} MB")

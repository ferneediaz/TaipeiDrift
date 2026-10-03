"""Plan the demo flight along the Wufeng corridor: where both aerial images hold imagery.

The simulator's ground is the 2020 image; the navigator's map is the 2018 image of the same place.
Only part of each rectangle holds imagery (a motorway corridor, about 3.7 km long), so the flight
follows the corridor's centreline: the line furthest from the edge of the ground both images cover.

The route: climb at the start point (the simulator's origin, the centre of the 2020 image), fly
north along the centreline to near its north end, turn, and fly south to its south end. GNSS is
lost after the first few hundred metres; that cut is applied afterwards, by distance (see
baseline/src/data/sim_replay.py), so the recording itself keeps GNSS.

Coordinates: the simulator's x is east and y is north, in metres from the centre of the 2020 image
(EPSG:3826, TWD97, which is in true metres).

Run on the host (needs rasterio):
    python sim/scripts/plan_route.py            # writes sim/scenarios/wufeng_corridor.json and a preview
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
GROUND_TIF = REPO / "data/raw/aerial/wufeng_2020-03-23_x4.tif"  # what the simulated camera sees
MAP_TIF = REPO / "data/raw/aerial/wufeng_2018-05-03_x4.tif"  # what the navigator carries as its map
OUT = REPO / "sim/scenarios/wufeng_corridor.json"
GRID_M = 4.0  # metres per cell of the coverage grid


def covered(tif: Path, west: float, north: float, width: int, height: int) -> np.ndarray:
    """Cells of the grid where the image holds imagery (neither black nor white), at GRID_M metres per cell."""
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.transform import from_origin
    from rasterio.warp import reproject

    with rasterio.open(tif) as src:
        small = (src.height // 16, src.width // 16)
        scale = src.transform * src.transform.scale(src.width / small[1], src.height / small[0])
        total = np.zeros((height, width), np.float32)
        for band in (1, 2, 3):
            out = np.zeros((height, width), np.float32)
            reproject(src.read(band, out_shape=small, resampling=Resampling.average).astype(np.float32), out,
                      src_transform=scale, src_crs=src.crs, dst_transform=from_origin(west, north, GRID_M, GRID_M),
                      dst_crs=src.crs, resampling=Resampling.average, dst_nodata=0)
            total += out
    return (total > 30) & (total < 3 * 250)


def image_centre(tif: Path) -> tuple[float, float]:
    import rasterio

    with rasterio.open(tif) as src:
        b = src.bounds
    return (b.left + b.right) / 2, (b.bottom + b.top) / 2


def centreline(mask: np.ndarray, west: float, north: float, origin: tuple[float, float]) -> np.ndarray:
    """The corridor's centreline from north to south: per 20 m of northing, the cell furthest from the edge.

    Returns rows of (east, north, distance to the edge, distance along the line), in simulator metres.
    """
    dist = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5) * GRID_M
    points = []
    for y in np.where(mask.any(1))[0][::5]:
        x = int(np.argmax(dist[y] * mask[y]))
        if dist[y, x] > 0:
            points.append((x, y))
    points = np.array(points, float)
    k = 7  # smooth over 140 m
    xs = np.convolve(np.pad(points[:, 0], k // 2, mode="edge"), np.ones(k) / k, mode="valid")
    ys = points[:, 1]
    east = west + (xs + 0.5) * GRID_M - origin[0]
    north_m = north - (ys + 0.5) * GRID_M - origin[1]
    edge = np.array([dist[int(round(y)), int(round(x))] for x, y in zip(xs, ys)])
    along = np.r_[0.0, np.cumsum(np.hypot(np.diff(east), np.diff(north_m)))]
    return np.c_[east, north_m, edge, along]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--altitude", type=float, default=100.0, help="flight height above the ground, m")
    ap.add_argument("--speed", type=float, default=10.0, help="ground speed, m/s")
    ap.add_argument("--north-margin", type=float, default=300.0, help="turn this far before the corridor's north end, m")
    ap.add_argument("--south-margin", type=float, default=100.0, help="stop this far before its south end, m")
    ap.add_argument("--south-first", action="store_true", help="fly south first, turn near the south end, then north")
    ap.add_argument("--out", default=str(OUT), help="route file to write (a preview PNG is written next to it)")
    args = ap.parse_args()
    out = Path(args.out)

    origin = image_centre(GROUND_TIF)
    west, north, east, south = 216090.0, 2662690.0, 218240.0, 2659760.0  # a box around both images
    width, height = int((east - west) / GRID_M), int((north - south) / GRID_M)
    both = covered(GROUND_TIF, west, north, width, height) & covered(MAP_TIF, west, north, width, height)
    both = cv2.morphologyEx(both.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(both)
    corridor = labels == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    line = centreline(corridor, west, north, origin)

    start = int(np.argmin(np.hypot(line[:, 0], line[:, 1])))  # the centreline point nearest the start
    turn = int(np.searchsorted(line[:, 3], args.north_margin))
    stop = int(np.searchsorted(line[:, 3], line[-1, 3] - args.south_margin))
    if args.south_first:  # out to the south end, back north to the turn point of the northern margin
        route = np.r_[line[start:stop + 1, :2], line[stop - 1:turn - 1:-1, :2]]
    else:
        route = np.r_[line[start:turn - 1:-1, :2], line[turn:stop + 1, :2]]
    length = float(np.sum(np.hypot(*np.diff(route, axis=0).T)))

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "about": "Demo route along the Wufeng corridor; written by sim/scripts/plan_route.py",
        "crs": "EPSG:3826", "origin_easting_m": origin[0], "origin_northing_m": origin[1],
        "altitude_m": args.altitude, "speed_mps": args.speed, "length_m": round(length, 1),
        "waypoints_xy_m": [[round(float(x), 1), round(float(y), 1)] for x, y in route],
    }, indent=1) + "\n")

    preview = np.dstack([corridor * 120] * 3).astype(np.uint8)
    to_px = lambda x, y: (int((x + origin[0] - west) / GRID_M), int((north - (y + origin[1])) / GRID_M))
    for (x0, y0), (x1, y1) in zip(route[:-1], route[1:]):
        cv2.line(preview, to_px(x0, y0), to_px(x1, y1), (0, 140, 255), 2)
    cv2.circle(preview, to_px(0.0, 0.0), 6, (0, 0, 255), 2)
    cv2.imwrite(str(out.with_suffix(".png")), preview)
    narrow = line[start:stop + 1, 2]
    print(f"route: {len(route)} waypoints, {length:.0f} m at {args.altitude:.0f} m and {args.speed:.0f} m/s "
          f"(about {length / args.speed / 60:.1f} min); imagery reaches {np.median(narrow):.0f} m to the side "
          f"(median), {narrow.min():.0f} m at the narrowest -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

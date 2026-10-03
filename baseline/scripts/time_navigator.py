"""How much computing the camera navigator needs: time per camera frame, time per map fix, map size.

Measured on the simulated Wufeng flight (512 px camera, frames turned north up to 360 px, map at
0.5 m per pixel), on one processor core of whatever machine runs it. The navigator itself is single
threaded; OpenCV is limited to one thread here so the numbers stand for one core.

From the repository root:

    python baseline/scripts/time_navigator.py

Prints a table and writes outputs/timing/timing.json.
"""
from __future__ import annotations

import json
import platform
import sys
import time
from pathlib import Path

import cv2
import numpy as np

BASELINE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = BASELINE_DIR.parent
sys.path.insert(0, str(BASELINE_DIR))

from src.data.camera_flight import prepare  # noqa: E402
from src.data.sim_replay import SimReplayConfig, load_sim_flight  # noqa: E402
from src.estimation.image_motion import image_shift  # noqa: E402
from src.estimation.map_matching import KEEP, search_area  # noqa: E402

RADII_M = (60.0, 150.0, 300.0, 600.0)  # 60 m is the smallest search, 600 m the cap
ZOOMS = np.arange(0.70, 0.9001, 0.05)  # five zooms around the learned one, as navigate tries (zoom_reach 0.10, step 0.05)
ANGLES = np.arange(-5.0, 5.1, 5.0)  # the angles tried around the learned one


def best_of(fn, repeats: int) -> float:
    """Median time of ``repeats`` calls, in milliseconds."""
    times = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        fn()
        times.append(time.perf_counter() - t0)
    return 1000.0 * float(np.median(times))


def main() -> int:
    cv2.setNumThreads(1)
    flight = load_sim_flight(SimReplayConfig(recording=str(REPO_ROOT / "recordings/wufeng_corridor_100m"),
                                             route=str(REPO_ROOT / "sim/scenarios/wufeng_corridor.json"),
                                             map_tif=str(REPO_ROOT / "data/raw/aerial/wufeng_2018-05-03_x4.tif"),
                                             cache_dir=str(REPO_ROOT / "data/processed")))
    ground = flight.ground_map
    ground.prepared(), ground.covered_float()  # built once at start-up, not per fix
    frames = [flight.frame(k) for k in range(600, 660)]
    load_ms = best_of(lambda: flight.frame(600), 20)
    flow_ms = best_of(lambda: image_shift(frames[0], frames[1]), 30)
    prep_ms = best_of(lambda: prepare(frames[0]), 30)
    fix_ms = {}
    for radius in RADII_M:
        centre = flight.position_gt[620]
        fix_ms[radius] = best_of(lambda: search_area(prepare(frames[20]), ground, centre, radius, ZOOMS, ANGLES, KEEP), 5)

    km2 = ground.covered.sum() * ground.metres_per_pixel ** 2 / 1e6
    out = {
        "machine": f"{platform.machine()}, {platform.processor() or platform.platform()}",
        "one_core": True,
        "camera_frame_ms": {"read_and_turn_north_up": round(load_ms, 1), "optical_flow": round(flow_ms, 1)},
        "map_fix_ms_by_search_radius_m": {f"{r:g}": round(v, 1) for r, v in fix_ms.items()},
        "fix_tries": {"zooms": len(ZOOMS), "angles": len(ANGLES)},
        "map": {"metres_per_pixel": ground.metres_per_pixel, "covered_km2": round(km2, 2),
                "grey_bytes_per_km2": int(1e6 / ground.metres_per_pixel ** 2)},
    }
    print(f"one core of {out['machine']}")
    print(f"per camera frame: read and turn north up {load_ms:.1f} ms, optical flow {flow_ms:.1f} ms "
          f"-> {1000 / (load_ms + flow_ms):.0f} frames per second possible (the camera gives 25)")
    print(f"preparing a frame for matching {prep_ms:.1f} ms")
    for r, v in fix_ms.items():
        print(f"one map fix, search radius {r:4.0f} m: {v:7.1f} ms ({len(ZOOMS)} zooms x {len(ANGLES)} angles)")
    print(f"map: {ground.metres_per_pixel} m per pixel, {out['map']['grey_bytes_per_km2'] / 1e6:.1f} MB per km2 as raw grey; "
          f"this map covers {km2:.2f} km2")
    folder = REPO_ROOT / "outputs" / "timing"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "timing.json").write_text(json.dumps(out, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

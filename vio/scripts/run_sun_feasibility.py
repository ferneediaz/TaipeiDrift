"""Sun-visibility feasibility on Mid-Air forward-camera frames (evaluation uses ground truth).

    python vio/scripts/run_sun_feasibility.py sunny:0 sunny:1 cloudy:3000 cloudy:3001 sunset:1004 ...

For every frame: sky fraction, best sun candidate, its body ray, and (EVALUATION ONLY) its world
ray using the ground-truth attitude. Real sun detections must agree on one world direction,
because the simulator's lighting is static; clouds and highlights scatter. Writes per flight
outputs/sun_orientation/feasibility/<cond>_<traj>.csv and prints a summary.
"""
from __future__ import annotations

import csv
import sys
import zipfile
from pathlib import Path

import cv2
import numpy as np
import yaml

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(VIO_DIR / "scripts"))

from run_visual_rotation_diagnostic import load_midair  # noqa: E402
from src.data.trajectory import quat_wxyz_to_rotation  # noqa: E402
from vio.sun.detector import azimuth_elevation_ned, camera_to_body_ray, detect_sun, pixel_to_camera_ray  # noqa: E402
from vio.vision.camera import pinhole_intrinsics  # noqa: E402

OUT = REPO_ROOT / "outputs" / "sun_orientation" / "feasibility"


def run(spec: str, step: int = 1):
    cfg = yaml.safe_load((VIO_DIR / "configs" / "midair_vio.yaml").read_text())
    traj, cond, frames_dir = load_midair(spec)
    name = traj.metadata["trajectory"]
    R_bc = np.asarray(cfg["cameras"]["left"]["R_bc"], float)
    K = pinhole_intrinsics(512, 512, 90.0)
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    z = zipfile.ZipFile(Path(frames_dir) / "color_left" / name / "frames.zip")
    names = sorted(n for n in z.namelist() if n.endswith(".JPEG"))
    rows = []
    for i in range(0, min(len(names), (len(traj) - 1) // 4 + 1), step):
        img = cv2.resize(cv2.imdecode(np.frombuffer(z.read(names[i]), np.uint8), cv2.IMREAD_COLOR), (512, 512), interpolation=cv2.INTER_AREA)
        det, skyf, ncand = detect_sun(img)
        row = {"frame": i, "t": i * 0.04, "sky_fraction": skyf, "n_saturated_regions": ncand, "detected": int(det is not None)}
        if det is not None:
            sb = camera_to_body_ray(pixel_to_camera_ray(det.u, det.v, K), R_bc)
            sw = R[4 * i].apply(sb)  # EVALUATION ONLY
            az, el = azimuth_elevation_ned(sw)
            row.update({"u": det.u, "v": det.v, "confidence": det.confidence, "area": det.area, "circularity": det.circularity,
                        "ring": det.ring_brightness, "sky_adj": det.sky_adjacency, "sb_x": sb[0], "sb_y": sb[1], "sb_z": sb[2],
                        "sw_x": sw[0], "sw_y": sw[1], "sw_z": sw[2], "world_az_deg": az, "world_el_deg": el})
        rows.append(row)
    OUT.mkdir(parents=True, exist_ok=True)
    keys = sorted({k for r in rows for k in r}, key=lambda k: list(rows[-1]).index(k) if k in rows[-1] else 99)
    with open(OUT / f"{cond}_{name}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["frame", "t", "sky_fraction", "n_saturated_regions", "detected", "u", "v", "confidence", "area",
                                          "circularity", "ring", "sky_adj", "sb_x", "sb_y", "sb_z", "sw_x", "sw_y", "sw_z", "world_az_deg", "world_el_deg"])
        w.writeheader()
        w.writerows(rows)
    det = [r for r in rows if r["detected"]]
    msg = f"{cond}/{name}: {len(rows)} frames | sky visible (>5%) {np.mean([r['sky_fraction'] > 0.05 for r in rows]) * 100:.0f}% | detections {len(det)} ({len(det) / len(rows) * 100:.1f}%)"
    if det:
        sw = np.array([[r["sw_x"], r["sw_y"], r["sw_z"]] for r in det])
        med = np.median(sw, axis=0)
        med /= np.linalg.norm(med)
        ang = np.degrees(np.arccos(np.clip(sw @ med, -1, 1)))
        az, el = azimuth_elevation_ned(med)
        msg += f" | world dir median az {az:.1f} el {el:.1f} | within 3 deg {np.mean(ang < 3) * 100:.0f}%, within 10 deg {np.mean(ang < 10) * 100:.0f}%"
    print(msg, flush=True)


if __name__ == "__main__":
    for s in sys.argv[1:]:
        run(s)

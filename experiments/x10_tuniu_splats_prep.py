#!/usr/bin/env python3
"""Prepare OpenDroneMap's Tuniu reconstruction for Gaussian-splat training (Brush), in COLMAP text format.

Uses only OpenDroneMap outputs, unchanged: the undistorted photos and camera poses of
data/processed/x_tuniu_survey_odm_full3d/opensfm/undistorted/ (reconstruction.json, images/*.tif) and its sparse
points. This is a format conversion (OpenSfM -> COLMAP) plus a resize of the photos; no pose is re-estimated.

OpenSfM shot: x_cam = R(rotation) x_world + translation, camera axes x right / y down / z forward, the same as
COLMAP, so qvec = quaternion(R) and tvec = translation. Focal is normalised by max(width, height).

Output: data/processed/x_tuniu_splats/colmap/{images/*.jpg, sparse/0/{cameras,images,points3D}.txt}

    .venv/bin/python experiments/x10_tuniu_splats_prep.py --max-px 1600
    ~/dev/tools/brush/brush-app-aarch64-apple-darwin/brush_app data/processed/x_tuniu_splats/colmap \
        --with-viewer --max-resolution 1600 --total-steps 15000 --export-path data/processed/x_tuniu_splats/out

Photos by Yu-Huang Wang (licence unknown): local use only.
"""
from __future__ import annotations

import argparse
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
UND = ROOT / "data/processed/x_tuniu_survey_odm_full3d/opensfm/undistorted"
OUT = ROOT / "data/processed/x_tuniu_splats/colmap"


def quat_from_rotvec(v) -> np.ndarray:
    v = np.asarray(v, float)
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    a = v / th
    return np.concatenate([[np.cos(th / 2)], np.sin(th / 2) * a])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max-px", type=int, default=1600, help="long side of the exported photos")
    args = ap.parse_args()
    rec = json.loads((UND / "reconstruction.json").read_text())[0]
    (OUT / "images").mkdir(parents=True, exist_ok=True)
    (OUT / "sparse/0").mkdir(parents=True, exist_ok=True)

    cams, cam_id = [], {}
    for i, (key, c) in enumerate(rec["cameras"].items(), 1):
        if c["projection_type"] != "perspective" or c.get("k1", 0) or c.get("k2", 0):
            raise ValueError(f"expected undistorted pinhole cameras, got {c}")
        s = args.max_px / max(c["width"], c["height"])
        w, h = round(c["width"] * s), round(c["height"] * s)
        f = c["focal"] * max(w, h)
        cams.append(f"{i} PINHOLE {w} {h} {f:.6f} {f:.6f} {w / 2:.6f} {h / 2:.6f}")
        cam_id[key] = (i, w, h)
    (OUT / "sparse/0/cameras.txt").write_text("# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n" + "\n".join(cams) + "\n")

    def convert(name: str, size) -> None:
        dst = OUT / "images" / (Path(name).stem.replace(".JPG", "") + ".jpg")
        if not dst.exists():
            with Image.open(UND / "images" / name) as im:
                im.convert("RGB").resize(size, Image.Resampling.LANCZOS).save(dst, quality=92)

    lines, jobs = [], []
    for j, (name, shot) in enumerate(sorted(rec["shots"].items()), 1):
        cid, w, h = cam_id[shot["camera"]]
        q = quat_from_rotvec(shot["rotation"])
        t = shot["translation"]
        jpg = Path(name).stem.replace(".JPG", "") + ".jpg"
        lines += [f"{j} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} {cid} {jpg}", ""]
        jobs.append((name, (w, h)))
    (OUT / "sparse/0/images.txt").write_text("# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n" + "\n".join(lines) + "\n")
    with ThreadPoolExecutor(4) as ex:
        list(ex.map(lambda a: convert(*a), jobs))

    pts = [f"{k} {p['coordinates'][0]:.4f} {p['coordinates'][1]:.4f} {p['coordinates'][2]:.4f} "
           f"{int(p['color'][0])} {int(p['color'][1])} {int(p['color'][2])} 0"
           for k, p in enumerate(rec["points"].values(), 1)]
    (OUT / "sparse/0/points3D.txt").write_text("# POINT3D_ID X Y Z R G B ERROR TRACK[]\n" + "\n".join(pts) + "\n")
    print(f"{len(jobs)} images, {len(pts)} points, cameras: {cams} -> {OUT}")


if __name__ == "__main__":
    main()

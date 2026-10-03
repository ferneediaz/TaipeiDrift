"""Multi-site, multi-date map localization benchmark with camera-effect conditions.

Evidence: real orthophotos of different dates (map = older date, camera = newer date);
the camera image is SIMULATED from the newer orthophoto by a declared pinhole generator
(yaw, altitude/scale error, roll/pitch tilt, blur, haze, low light, JPEG). Ground truth
is used only inside the generator and for scoring; the estimators receive the query
image, a map window around a noisy prior and (for rectified conditions) a noisy attitude.

This script only measures raw scores and errors. Acceptance thresholds, agreement
rules and false-accept bounds are computed out-of-sample in r_integrity.py.

Run from repo root:
  .venv/bin/python experiments/r_map_benchmark.py --workers 8
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import sys
import time
from dataclasses import dataclass, field
from itertools import combinations
from multiprocessing import get_context
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import rasterio

QUERY = 160          # camera image, px (1 m/px at the nominal altitude)
SEARCH = 384         # map window around the prior, px = m
TEMPLATE = 96        # central template for the ZNCC family
FOCAL = 160.         # px; with ALTITUDE gives 1 m/px at nadir, 53 deg FOV
ALTITUDE = 160.      # m above flat ground (generator assumption)
PRIOR_ERROR = 60     # uniform +-60 m prior error per axis
SPACING = 160        # px between query centres (non-overlapping nadir footprints)
NEG_MIN_DIST = 320   # Chebyshev px between negative source and prior
ROOT = Path(__file__).resolve().parents[1]

# name: generator parameters. yaw deg, scale = true altitude / assumed altitude,
# tilt deg (random direction), rectify = use IMU attitude with 1 deg noise to nadir-warp.
CONDITIONS = {
    "aligned": dict(),
    "legacy_degraded": dict(yaw=15, scale=1.07, blur=1.2, gain=.65, offset=12),
    "yaw10": dict(yaw=10), "yaw30": dict(yaw=30), "yaw60": dict(yaw=60),
    "yaw90": dict(yaw=90), "yaw180": dict(yaw=180),
    "scale0.8": dict(scale=.8), "scale1.25": dict(scale=1.25),
    "tilt5": dict(tilt=5), "tilt10": dict(tilt=10), "tilt20": dict(tilt=20),
    "tilt10_rect": dict(tilt=10, rectify=True), "tilt20_rect": dict(tilt=20, rectify=True),
    "motion9": dict(motion=9), "motion21": dict(motion=21),
    "haze": dict(haze=.55),
    "lowlight_noise": dict(gain=.4, offset=8, noise=5.),
    "patch_shadow": dict(shadow=True),
    "jpeg20": dict(jpeg=20, noise=3.),
}
HEADING_CONDITIONS = {"yaw30", "yaw60", "yaw90", "yaw180"}
BASE_METHODS = ["zncc", "zncc_yaw_scale", "xfeat_affine", "xfeat_homography"]
HEADING_METHODS = ["zncc_heading", "xfeat_rot4"]
PAIRS_MODE = "default"
GRID_OFFSET = 0


# ----------------------------------------------------------------------------- data

def ensure_wufeng() -> None:
    """Materialize the existing OAM Wufeng pair on the shared 1 m grid (once)."""
    out = ROOT / "data/raw/aerial_pairs/oam_wufeng"
    if (out / "2020-03-23_oam.tif").exists():
        return
    sys.path.insert(0, str(ROOT / "experiments"))
    import rasterio as rio
    from o_map_benchmark import load_maps  # noqa: E402  (same 1 m grid as the first benchmark)
    out.mkdir(parents=True, exist_ok=True)
    paths = [ROOT / f"data/raw/aerial/wufeng_{d}_x4.tif" for d in ("2018-05-03", "2020-03-23")]
    with rio.open(paths[0]) as a, rio.open(paths[1]) as b:
        left, right = max(a.bounds.left, b.bounds.left), min(a.bounds.right, b.bounds.right)
        bottom, top = max(a.bounds.bottom, b.bounds.bottom), min(a.bounds.top, b.bounds.top)
        transform = rio.transform.from_origin(left, top, 1, 1)
        shape = (int(top - bottom), int(right - left))
        from rasterio.warp import Resampling, reproject
        for ds, date in ((a, "2018-05-03"), (b, "2020-03-23")):
            rgb = np.zeros((3, *shape), dtype=np.uint8)
            for band in range(3):
                reproject(rio.band(ds, band + 1), rgb[band], dst_transform=transform,
                          dst_crs=ds.crs, resampling=Resampling.average)
            with rio.open(out / f"{date}_oam.tif", "w", driver="GTiff", width=shape[1], height=shape[0],
                          count=3, dtype="uint8", crs=ds.crs, transform=transform, nodata=0,
                          compress="deflate") as dst:
                dst.write(rgb)


def discover_sites(root: Path) -> list[dict]:
    manifest = {}
    for path in sorted(root.glob("manifest_*.json")):
        for entry in json.loads(path.read_text()):
            if "file" in entry:
                manifest["/".join(Path(entry["file"]).parts[-2:])] = entry
    sites = []
    for directory in sorted(p for p in root.iterdir() if p.is_dir()):
        files = sorted(directory.glob("*.tif"))
        if len(files) < 2:
            continue
        meta = manifest.get(f"{directory.name}/{files[0].name}", {})
        land = meta.get("land_cover", "corridor_mixed" if directory.name == "oam_wufeng" else "unknown")
        shapes = set()
        for f in files:
            with rasterio.open(f) as ds:
                shapes.add((ds.width, ds.height, tuple(ds.transform)[:6], ds.crs.to_epsg()))
        if len(shapes) != 1:
            print(f"skip {directory.name}: dates are not on one grid", flush=True)
            continue
        sites.append(dict(site=directory.name, land_cover=land, files=[str(f) for f in files],
                          source=files[0].stem.split("_", 1)[-1]))
    return sites


_CACHE: dict[str, tuple[np.ndarray, np.ndarray]] = {}


def load(path: str):
    if path not in _CACHE:
        with rasterio.open(path) as ds:
            rgb = ds.read([1, 2, 3]).transpose(1, 2, 0)
        grey = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2GRAY)
        valid = (rgb.max(axis=2) > 0).astype(np.uint8)
        _CACHE[path] = (grey, valid)
    return _CACHE[path]


# ------------------------------------------------------------------------ generator

def rotation(yaw, roll, pitch):
    """Camera-to-map rotation. Map frame: x = column, y = row, z = into the ground."""
    cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    cr, sr = np.cos(np.radians(roll)), np.sin(np.radians(roll))
    cp, sp = np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cr, 0, sr], [0, 1, 0], [-sr, 0, cr]])   # roll tilts the x axis
    rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])   # pitch tilts the y axis
    return rz @ rx @ ry


def ground_to_image(centre, altitude, rot):
    k = np.array([[FOCAL, 0, QUERY / 2], [0, FOCAL, QUERY / 2], [0, 0, 1]])
    ext = np.array([[1, 0, -centre[0]], [0, 1, -centre[1]], [0, 0, altitude]], float)
    return k @ rot.T @ ext


def apply(h, point):
    p = h @ np.array([point[0], point[1], 1.])
    return p[:2] / p[2]


@dataclass
class Shot:
    image: np.ndarray
    image_to_ground: np.ndarray     # truth geometry, scoring only
    drone_xy: np.ndarray            # truth drone position, scoring only
    valid: bool
    meta: dict = field(default_factory=dict)


def render(grey, valid, centre, cond: dict, rng) -> Shot:
    yaw = cond.get("yaw", 0.)
    tilt = cond.get("tilt", 0.)
    direction = rng.uniform(0, 2 * np.pi)
    roll, pitch = tilt * np.cos(direction), tilt * np.sin(direction)
    altitude = ALTITUDE * cond.get("scale", 1.)
    rot = rotation(yaw, roll, pitch)
    g2i = ground_to_image(centre, altitude, rot)
    image = cv2.warpPerspective(grey, g2i, (QUERY, QUERY), flags=cv2.INTER_LINEAR)
    inside = cv2.warpPerspective(valid, g2i, (QUERY, QUERY), flags=cv2.INTER_NEAREST, borderValue=0)
    ok = bool(inside.min() > 0)
    i2g = np.linalg.inv(g2i)
    meta = dict(roll=roll, pitch=pitch, nav_offset=np.zeros(2))
    if tilt:
        # Synthetic IMU attitude: truth + 1 deg sigma per axis (labelled generator noise).
        # The estimator uses it to convert the image-centre ground point into a drone position.
        roll_e, pitch_e = roll + rng.normal(0, 1), pitch + rng.normal(0, 1)
        est = ground_to_image((0, 0), ALTITUDE, rotation(yaw, roll_e, pitch_e))
        offset = apply(np.linalg.inv(est), (QUERY / 2, QUERY / 2))   # principal-ray ground point
        meta.update(roll_imu=roll_e, pitch_imu=pitch_e, nav_offset=offset)
        if cond.get("rectify"):
            # Virtual nadir camera centred on the principal-ray ground point (estimated attitude).
            nadir = ground_to_image(offset, ALTITUDE, rotation(yaw, 0, 0))
            m = nadir @ np.linalg.inv(est)
            image = cv2.warpPerspective(image, m, (QUERY, QUERY), flags=cv2.INTER_LINEAR)
            inside = cv2.warpPerspective(inside, m, (QUERY, QUERY), flags=cv2.INTER_NEAREST)
            h = TEMPLATE // 2
            ok = ok and bool(inside[QUERY//2-h:QUERY//2+h, QUERY//2-h:QUERY//2+h].min() > 0)
            meta["rect_valid"] = float(inside.mean())
            image[inside == 0] = int(np.median(image[inside > 0])) if inside.any() else 0
            i2g = i2g @ np.linalg.inv(m)
    image = photometric(image, cond, rng)
    return Shot(image, i2g, np.asarray(centre, float), ok, meta)


def photometric(image, cond, rng):
    img = image.astype(np.float32)
    if cond.get("blur"):
        img = cv2.GaussianBlur(img, (0, 0), cond["blur"])
    if cond.get("motion"):
        n = cond["motion"]
        kernel = np.zeros((n, n), np.float32)
        kernel[n // 2, :] = 1
        kernel = cv2.warpAffine(kernel, cv2.getRotationMatrix2D((n / 2 - .5, n / 2 - .5), rng.uniform(0, 180), 1), (n, n))
        img = cv2.filter2D(img, -1, kernel / kernel.sum())
    if cond.get("haze"):
        t = cond["haze"]
        img = img * t + 210 * (1 - t)
    if cond.get("shadow"):
        # Crude: smooth random 50 % darkening patches over ~30 % of the image. Not sun geometry.
        noise = cv2.GaussianBlur(rng.normal(size=(QUERY, QUERY)).astype(np.float32), (0, 0), 12)
        mask = (noise > np.quantile(noise, .7)).astype(np.float32)
        mask = cv2.GaussianBlur(mask, (0, 0), 2)
        img = img * (1 - .5 * mask)
    if "gain" in cond:
        img = img * cond["gain"] + cond.get("offset", 0)
    if cond.get("noise"):
        img = img + rng.normal(0, cond["noise"], img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    if cond.get("jpeg"):
        _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, cond["jpeg"]])
        img = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
    return img


# -------------------------------------------------------------------------- methods

def crop(image, centre, size):
    x, y = np.rint(centre).astype(int)
    h = size // 2
    if y - h < 0 or x - h < 0 or y + h > image.shape[0] or x + h > image.shape[1]:
        raise ValueError("crop leaves image")
    return image[y-h:y+h, x-h:x+h]


def zncc(query, reference, valid, angles=(0.,), scales=(1.,)):
    """Translation (+ optional yaw/scale hypotheses) ZNCC; returns point + integrity stats."""
    win = cv2.matchTemplate(valid.astype(np.float32), np.ones((TEMPLATE, TEMPLATE), np.float32), cv2.TM_CCORR)
    bad = win < .99 * TEMPLATE ** 2
    best = dict(score=-1., point=np.array([np.nan, np.nan]))
    for angle in angles:
        for scale in scales:
            m = cv2.getRotationMatrix2D((QUERY / 2, QUERY / 2), angle, scale)
            warped = cv2.warpAffine(query, m, (QUERY, QUERY))
            t = crop(warped, (QUERY / 2, QUERY / 2), TEMPLATE)
            if t.std() < 2:
                continue
            s = cv2.matchTemplate(reference, t, cv2.TM_CCOEFF_NORMED)
            s[bad] = -1
            _, peak, _, loc = cv2.minMaxLoc(s)
            if peak > best["score"]:
                best = dict(score=float(peak), point=np.array(loc, float) + TEMPLATE / 2, surface=s, loc=loc,
                            angle=angle, scale=scale, warped=warped)
    if "surface" not in best:
        return best["point"], dict(score=0.)
    s, (x, y) = best.pop("surface"), best.pop("loc")
    good = s[s > -1]
    masked = s.copy()
    masked[max(0, y-12):y+13, max(0, x-12):x+13] = -1
    second = float(masked.max())
    stats = dict(score=best["score"], second=second, peak_ratio=(1 - second) / max(1e-6, 1 - best["score"]),
                 margin=best["score"] - second,
                 z=(best["score"] - float(good.mean())) / max(1e-6, float(good.std())),
                 hyp_angle=best["angle"], hyp_scale=best["scale"])
    stats.update(quad_consensus(best.pop("warped"), reference, valid, best["point"]))
    return best["point"], stats


def quad_consensus(warped, reference, valid, point, size=56, tol=4.):
    """Integrity check: four disjoint sub-templates matched independently must agree with the full fix."""
    win = cv2.matchTemplate(valid.astype(np.float32), np.ones((size, size), np.float32), cv2.TM_CCORR)
    bad = win < .99 * size ** 2
    c, h = QUERY // 2, size // 2
    agree, peaks, dists = 0, [], []
    for dx in (-h, h):
        for dy in (-h, h):
            t = warped[c + dy - h:c + dy + h, c + dx - h:c + dx + h]
            if t.std() < 2:
                dists.append(np.inf)
                continue
            s = cv2.matchTemplate(reference, t, cv2.TM_CCOEFF_NORMED)
            s[bad] = -1
            _, peak, _, loc = cv2.minMaxLoc(s)
            estimate = np.array(loc, float) + h - (dx, dy)
            peaks.append(peak)
            dists.append(float(np.linalg.norm(estimate - point)))
            agree += int(dists[-1] <= tol)
    # Sorted sub-template distances to the full fix, so other tolerances can be evaluated offline.
    d = sorted(dists)
    return dict(quad_n=agree, quad_peak=float(np.mean(peaks)) if peaks else 0.,
                quad_d1=d[0], quad_d2=d[1], quad_d3=d[2], quad_d4=d[3])


_XFEAT = None


def xfeat():
    global _XFEAT
    if _XFEAT is None:
        import torch
        torch.set_num_threads(1)
        root = ROOT / "data/raw/models/accelerated_features"
        sys.path.insert(0, str(root))
        from modules.xfeat import XFeat
        _XFEAT = XFeat(weights=str(root / "weights/xfeat.pt"), top_k=2048)
    return _XFEAT


def xfeat_points(query, reference):
    try:
        a, b = xfeat().match_xfeat(cv2.cvtColor(query, cv2.COLOR_GRAY2RGB),
                                   cv2.cvtColor(reference, cv2.COLOR_GRAY2RGB), top_k=2048)
    except (IndexError, RuntimeError):   # no keypoints detected (e.g. featureless water): no fix
        return np.zeros((0, 2), np.float32), np.zeros((0, 2), np.float32)
    return np.asarray(a, np.float32), np.asarray(b, np.float32)


def geometric_fix(a, b, model="affine", reference_shape=(SEARCH, SEARCH)):
    """RANSAC + the geometric gates of o_map_benchmark. Returns point, stats (score=inliers or 0)."""
    nan = np.array([np.nan, np.nan])
    stats = dict(score=0., matches=len(a), inliers=0, inlier_ratio=0., spread=0., fit_scale=np.nan, gate="few")
    if len(a) < 6:
        return nan, stats
    if model == "affine":
        m, mask = cv2.estimateAffinePartial2D(a, b, method=cv2.RANSAC, ransacReprojThreshold=3., maxIters=3000, confidence=.995)
        if m is None:
            return nan, stats
        h = np.vstack([m, [0, 0, 1]])
    else:
        h, mask = cv2.findHomography(a, b, cv2.USAC_MAGSAC, 3., maxIters=3000, confidence=.995)
        if h is None:
            return nan, stats
    keep = mask.ravel().astype(bool)
    scale = float(np.sqrt(abs(np.linalg.det(h[:2, :2]))))
    spread = float(np.linalg.eigvalsh(np.cov(a[keep].T)).min()) if keep.sum() >= 3 else 0.
    stats.update(inliers=int(keep.sum()), inlier_ratio=float(keep.mean()), spread=spread, fit_scale=scale)
    centre = apply(h, (QUERY / 2, QUERY / 2))
    if not (.6 < scale < 1.5):
        stats["gate"] = "scale"
    elif keep.mean() < .2 or keep.sum() < 6:
        stats["gate"] = "inliers"
    elif spread < 25:
        stats["gate"] = "spread"
    elif not np.all(np.isfinite(centre)) or np.any(centre < 0) or np.any(centre >= np.array(reference_shape[::-1])):
        stats["gate"] = "outside"
    else:
        stats.update(gate="ok", score=float(keep.sum()))
        return centre, stats
    return nan, stats   # gate failed: no fix, but keep the diagnostics


def run_method(method, query, reference, valid):
    if method == "zncc":
        return zncc(query, reference, valid)
    if method == "zncc_yaw_scale":
        return zncc(query, reference, valid, (-20., -10., 0., 10., 20.), (.9, 1., 1.1))
    if method == "zncc_heading":
        return zncc(query, reference, valid, tuple(np.arange(0., 360., 15.)), (1.,))
    if method in ("xfeat_affine", "xfeat_homography"):
        a, b = xfeat_points(query, reference)
        return geometric_fix(a, b, method.split("_")[1], reference.shape)
    if method == "xfeat_rot4":
        best = (np.array([np.nan, np.nan]), dict(score=0.))
        for k in range(4):
            q = np.rot90(query, k).copy()
            a, b = xfeat_points(q, reference)
            # Map rotated-image points back to the original query frame before fitting.
            for _ in range(k):
                a = np.stack([QUERY - 1 - a[:, 1], a[:, 0]], axis=1) if len(a) else a
            point, stats = geometric_fix(a, b, "affine", reference.shape)
            stats["hyp_angle"] = 90 * k
            if stats["score"] > best[1]["score"]:
                best = (point, stats)
        return best
    if method.startswith("lib:"):
        sys.path.insert(0, str(ROOT / "experiments"))
        import r_matchers
        a, b, _ = r_matchers.match(method[4:], query, reference)
        return geometric_fix(np.asarray(a, np.float32), np.asarray(b, np.float32), "affine", reference.shape)
    raise KeyError(method)


def texture(image):
    gx = cv2.Sobel(image, cv2.CV_32F, 1, 0)
    gy = cv2.Sobel(image, cv2.CV_32F, 0, 1)
    return dict(q_std=float(image.std()), q_grad=float(np.hypot(gx, gy).mean()))


# --------------------------------------------------------------------------- worker

def seed_of(*parts) -> int:
    return int.from_bytes(hashlib.sha256("|".join(map(str, parts)).encode()).digest()[:8], "little")


def unit_rows(unit):
    cv2.setNumThreads(1)
    rng = np.random.default_rng(unit["seed"])
    map_grey, map_valid = load(unit["map_file"])
    centre = np.array(unit["centre"], float)
    prior = centre + rng.integers(-PRIOR_ERROR, PRIOR_ERROR + 1, 2)
    reference = crop(map_grey, prior, SEARCH)
    valid = crop(map_valid, prior, SEARCH)
    rows = []
    sources = [("positive", unit["query_file"], unit["centre"], unit["site"])] + \
              [(kind, f, c, s) for kind, f, c, s in unit["negatives"]]
    for cname in unit["conditions"]:
        cond = CONDITIONS[cname]
        methods = list(unit["methods"]) + (HEADING_METHODS if cname in HEADING_CONDITIONS else [])
        for kind, qfile, qcentre, qsite in sources:
            grey, vmask = load(qfile)
            crng = np.random.default_rng(seed_of(unit["seed"], cname, kind, qcentre))
            shot = render(grey, vmask, np.array(qcentre, float), cond, crng)
            if not shot.valid:
                continue
            truth = apply(shot.image_to_ground, (QUERY / 2, QUERY / 2))
            for method in methods:
                start = time.perf_counter()
                try:
                    point, stats = run_method(method, shot.image, reference, valid)
                except (IndexError, RuntimeError, ValueError) as exc:   # e.g. no keypoints: no fix
                    point, stats = np.array([np.nan, np.nan]), dict(score=0., gate=f"error:{type(exc).__name__}")
                latency = (time.perf_counter() - start) * 1000
                estimate = point + prior - SEARCH / 2
                fixed = bool(np.all(np.isfinite(point)))
                row = dict(site=unit["site"], land_cover=unit["land_cover"], pair=unit["pair"],
                           centre_id=unit["centre_id"], condition=cname, kind=kind, query_site=qsite,
                           present=kind == "positive", method=method, latency_ms=latency, fixed=fixed,
                           error_m=float(np.linalg.norm(estimate - truth)) if fixed else np.nan,
                           nav_error_m=float(np.linalg.norm(estimate - shot.meta["nav_offset"] - shot.drone_xy)) if fixed else np.nan,
                           prior_offset_m=float(np.linalg.norm(prior - centre)),
                           truth_x=float(truth[0]), truth_y=float(truth[1]),
                           est_x=float(estimate[0]), est_y=float(estimate[1]), **texture(shot.image))
                row.update({k: v for k, v in stats.items()})
                rows.append(row)
    return rows


def build_units(sites, methods, conditions, max_centres, rng, neg_same=2, neg_other=1):
    units = []
    for site in sites:
        grey, valid = load(site["files"][0])
        h, w = grey.shape
        margin = SEARCH // 2 + PRIOR_ERROR + 8
        joint = np.ones_like(valid)
        for f in site["files"]:
            joint &= load(f)[1]
        centres = []
        for y in range(margin + GRID_OFFSET, h - margin, SPACING):
            for x in range(margin + GRID_OFFSET, w - margin, SPACING):
                # Nadir query footprint valid on every date; map window mostly valid (renders re-check).
                if crop(joint, (x, y), QUERY).mean() < .999:
                    continue
                if crop(valid, (x, y), SEARCH).mean() < .6:
                    continue
                centres.append((x, y))
        rng.shuffle(centres)
        centres = centres[:max_centres]
        site["n_centres"] = len(centres)
        n = len(site["files"])
        # Many-date sites: oldest map vs middle and newest camera dates only (bounded cost),
        # unless PAIRS_MODE == "oldest": oldest map vs every later date (date-gap study).
        if PAIRS_MODE == "oldest":
            pairs = [(0, j) for j in range(1, n)]
        else:
            pairs = list(combinations(range(n), 2)) if n <= 3 else [(0, n // 2), (0, n - 1)]
        for (i, j) in pairs:
            map_file, query_file = site["files"][i], site["files"][j]   # sorted by date: older = map
            pair = f"{Path(map_file).stem}->{Path(query_file).stem}"
            for cid, c in enumerate(centres):
                far = [p for p in centres if max(abs(p[0] - c[0]), abs(p[1] - c[1])) > NEG_MIN_DIST + PRIOR_ERROR]
                negs = []
                for n, k in enumerate(rng.permutation(len(far))[:neg_same]):
                    negs.append((f"neg_same_site{n}", query_file, far[k], site["site"]))
                others = [s for s in sites if s["site"] != site["site"] and s.get("centres")]
                for n in range(neg_other if others else 0):
                    o = others[rng.integers(len(others))]
                    negs.append((f"neg_other_site{n}", o["files"][-1], o["centres"][rng.integers(len(o["centres"]))], o["site"]))
                units.append(dict(site=site["site"], land_cover=site["land_cover"], pair=pair, centre_id=cid,
                                  centre=c, map_file=map_file, query_file=query_file, negatives=negs,
                                  methods=methods, conditions=conditions,
                                  seed=seed_of(site["site"], pair, cid)))
        site["centres"] = centres
    return units


def _init():
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    cv2.setNumThreads(1)
    import torch
    torch.set_num_threads(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pairs", type=Path, default=ROOT / "data/raw/aerial_pairs")
    parser.add_argument("--output", type=Path, default=ROOT / "data/processed/r_map_benchmark")
    parser.add_argument("--methods", default=",".join(BASE_METHODS))
    parser.add_argument("--conditions", default=",".join(CONDITIONS))
    parser.add_argument("--sites", default="", help="comma list; default all discovered")
    parser.add_argument("--max-centres", type=int, default=40)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument("--limit-units", type=int, default=0, help="smoke test")
    parser.add_argument("--neg-same", type=int, default=2, help="negatives per unit from the same site")
    parser.add_argument("--neg-other", type=int, default=1, help="negatives per unit from other sites")
    parser.add_argument("--pairs-mode", default="default", choices=("default", "oldest"))
    parser.add_argument("--grid-offset", type=int, default=0, help="px shift of the centre grid (fresh scenes)")
    args = parser.parse_args()
    global PAIRS_MODE, GRID_OFFSET
    PAIRS_MODE, GRID_OFFSET = args.pairs_mode, args.grid_offset
    ensure_wufeng()
    sites = discover_sites(args.pairs)
    if args.sites:
        keep = set(args.sites.split(","))
        sites = [s for s in sites if s["site"] in keep]
    rng = np.random.default_rng(args.seed)
    # Two passes so cross-site negatives can draw from every site's centres.
    build_units(sites, [], [], args.max_centres, np.random.default_rng(args.seed))
    units = build_units(sites, args.methods.split(","), args.conditions.split(","), args.max_centres, rng,
                        args.neg_same, args.neg_other)
    if args.limit_units:
        units = units[:args.limit_units]
    print(f"{len(sites)} sites, {len(units)} units: " + ", ".join(f"{s['site']}={s['n_centres']}" for s in sites), flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    start = time.time()
    with get_context("spawn").Pool(args.workers, initializer=_init) as pool:
        for i, part in enumerate(pool.imap_unordered(unit_rows, units, chunksize=1)):
            rows.extend(part)
            if (i + 1) % 50 == 0:
                print(f"{i+1}/{len(units)} units, {time.time()-start:.0f} s", flush=True)
    df = pd.DataFrame(rows)
    df.to_csv(args.output / "matches.csv.gz", index=False)
    meta = dict(protocol=__doc__.strip().split("\n\n")[1], generator=dict(query_px=QUERY, search_px=SEARCH,
                focal_px=FOCAL, altitude_m=ALTITUDE, prior_error_m=PRIOR_ERROR, spacing_px=SPACING,
                conditions=CONDITIONS), seed=args.seed, platform=platform.platform(),
                opencv=cv2.__version__, numpy=np.__version__,
                sites=[{k: v for k, v in s.items() if k != "centres"} for s in sites],
                files_sha256={f: hashlib.sha256(Path(f).read_bytes()).hexdigest()[:16] for s in sites for f in s["files"]},
                rows=len(df), seconds=time.time() - start)
    (args.output / "run.json").write_text(json.dumps(meta, indent=2, default=str) + "\n")
    print(df.groupby(["condition", "method"]).apply(
        lambda g: pd.Series(dict(pos_10m=(g[g.present].error_m <= 10).mean(), n=g.present.sum())),
        include_groups=False).unstack("method").round(2).to_string())


if __name__ == "__main__":
    main()

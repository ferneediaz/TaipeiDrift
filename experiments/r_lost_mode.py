"""Lost-mode recovery without a position prior: how map size changes wrong matches.

Real multi-date orthophotos (map = oldest date, camera = newest date of each site),
camera image SIMULATED by the r_map_benchmark pinhole generator. The camera is
searched over regions of growing size that contain the truth (384 m .. full site)
and over a mosaic of all sites (the "big map"). Negatives: camera scenes from a
site that is removed from the mosaic. Heading is assumed known to the generator
condition (aligned, yaw 10 deg error, or legacy degradation).

Pipelines (no truth used):
  zncc_top1      best ZNCC peak in the region
  zncc_quad      top1 + >= 3 of 4 disjoint sub-templates agree
  zncc_xfeat_k5  5 best ZNCC peaks (NMS 40 px); each verified by XFeat+RANSAC on a
                 384 px window; accept the best verified candidate whose XFeat
                 centre agrees with its ZNCC peak within 10 m

  .venv/bin/python experiments/r_lost_mode.py --workers 8
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from multiprocessing import get_context
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import r_map_benchmark as B  # noqa: E402

SIZES = [384, 768, 1536]
CONDS = ["aligned", "yaw10"]
GAP = 64


def build_mosaic(sites):
    maps = [(s["site"], *B.load(s["files"][0])) for s in sites]
    cols = int(np.ceil(np.sqrt(len(maps))))
    hmax = max(m[1].shape[0] for m in maps)
    wmax = max(m[1].shape[1] for m in maps)
    rows = int(np.ceil(len(maps) / cols))
    grey = np.zeros((rows * (hmax + GAP), cols * (wmax + GAP)), np.uint8)
    valid = np.zeros_like(grey)
    offsets, bounds = {}, {}
    for i, (name, g, v) in enumerate(maps):
        y, x = (i // cols) * (hmax + GAP), (i % cols) * (wmax + GAP)
        grey[y:y + g.shape[0], x:x + g.shape[1]] = g
        valid[y:y + g.shape[0], x:x + g.shape[1]] = v
        offsets[name] = np.array([x, y], float)
        bounds[name] = (x, y, g.shape[1], g.shape[0], int(v.sum()))
    return grey, valid, offsets, bounds


def box_sum(valid, size):
    """Sum of valid pixels in every size x size window (top-left anchored), via an integral image."""
    s = cv2.integral(valid.astype(np.uint8), sdepth=cv2.CV_32S)
    return (s[size:, size:] - s[:-size, size:] - s[size:, :-size] + s[:-size, :-size]).astype(np.float32)


def peaks(reference, valid, template, k, nms=40):
    win = box_sum(valid, template.shape[0])
    scores = cv2.matchTemplate(reference, template, cv2.TM_CCOEFF_NORMED)
    scores[win < .99 * template.size] = -1
    out = []
    for _ in range(k):
        _, score, _, loc = cv2.minMaxLoc(scores)
        if score <= -1:
            break
        out.append((float(score), np.array(loc, float) + template.shape[0] / 2))
        x, y = loc
        scores[max(0, y - nms):y + nms + 1, max(0, x - nms):x + nms + 1] = -1
    return out


def prepare_coarse(reference, valid, factor=8):
    h, w = reference.shape
    cw, ch = max(1, round(w / factor)), max(1, round(h / factor))
    small = cv2.resize(reference, (cw, ch), interpolation=cv2.INTER_AREA)
    valid_fraction = cv2.resize(valid.astype(np.float32), (cw, ch), interpolation=cv2.INTER_AREA)
    small_valid = (valid_fraction >= .99).astype(np.uint8)
    return small, small_valid, w / cw, h / ch


def coarse_to_fine_peaks(reference, valid, template, coarse, k=5, exclude_bounds=None):
    small, small_valid, sx, sy = coarse
    if exclude_bounds is not None:
        small_valid = small_valid.copy()
        ex, ey, ew, eh, _ = exclude_bounds
        pad = template.shape[0] / 2
        x0, x1 = int(max(0, np.floor((ex - pad) / sx))), int(np.ceil((ex + ew + pad) / sx))
        y0, y1 = int(max(0, np.floor((ey - pad) / sy))), int(np.ceil((ey + eh + pad) / sy))
        small_valid[y0:y1, x0:x1] = 0

    small_size = max(4, int(round(template.shape[0] / max(sx, sy))))
    small_template = cv2.resize(template, (small_size, small_size), interpolation=cv2.INTER_AREA)
    coarse_nms = max(1, int(round(40 / max(sx, sy))))
    coarse_candidates = peaks(small, small_valid, small_template, 40, nms=coarse_nms)
    refined = []
    h, w = reference.shape
    radius = 64
    th, tw = template.shape
    for _, coarse_point in coarse_candidates:
        centre = coarse_point * (sx, sy)
        if exclude_bounds is not None:
            ex, ey, ew, eh, _ = exclude_bounds
            if ex <= centre[0] < ex + ew and ey <= centre[1] < ey + eh:
                continue
        x0 = max(0, int(np.floor(centre[0] - tw / 2 - radius)))
        y0 = max(0, int(np.floor(centre[1] - th / 2 - radius)))
        x1 = min(w, int(np.ceil(centre[0] + tw / 2 + radius)))
        y1 = min(h, int(np.ceil(centre[1] + th / 2 + radius)))
        local = peaks(reference[y0:y1, x0:x1], valid[y0:y1, x0:x1], template, 1)
        if not local:
            continue
        score, point = local[0]
        point += (x0, y0)
        if exclude_bounds is not None:
            ex, ey, ew, eh, _ = exclude_bounds
            if ex <= point[0] < ex + ew and ey <= point[1] < ey + eh:
                continue
        refined.append((score, point))

    out = []
    for candidate in sorted(refined, key=lambda item: item[0], reverse=True):
        if any(np.linalg.norm(candidate[1] - point) <= 40 for _, point in out):
            continue
        out.append(candidate)
        if len(out) == k:
            break
    return out


def search(query, reference, valid, coarse=None, exclude_bounds=None):
    template = B.crop(query, (B.QUERY / 2, B.QUERY / 2), B.TEMPLATE)
    start = time.perf_counter()
    if coarse is None:
        candidates = peaks(reference, valid, template, 5)
    else:
        candidates = coarse_to_fine_peaks(reference, valid, template, coarse, exclude_bounds=exclude_bounds)
    zncc_ms = (time.perf_counter() - start) * 1000
    result = dict(zncc_ms=zncc_ms)
    if not candidates:
        return result
    score, top = candidates[0]
    x0 = int(np.clip(top[0] - B.SEARCH / 2, 0, reference.shape[1] - B.SEARCH))
    y0 = int(np.clip(top[1] - B.SEARCH / 2, 0, reference.shape[0] - B.SEARCH))
    q = B.quad_consensus(query, reference[y0:y0 + B.SEARCH, x0:x0 + B.SEARCH],
                         valid[y0:y0 + B.SEARCH, x0:x0 + B.SEARCH], top - (x0, y0))
    result.update(top1=top, top1_score=score, quad_n=q["quad_n"],
                  second_score=candidates[1][0] if len(candidates) > 1 else -1.)
    start = time.perf_counter()
    best = None
    for _, point in candidates:
        x0 = int(np.clip(point[0] - B.SEARCH / 2, 0, reference.shape[1] - B.SEARCH))
        y0 = int(np.clip(point[1] - B.SEARCH / 2, 0, reference.shape[0] - B.SEARCH))
        window = reference[y0:y0 + B.SEARCH, x0:x0 + B.SEARCH]
        a, b = B.xfeat_points(query, window)
        estimate, stats = B.geometric_fix(a, b, "affine")
        if not np.all(np.isfinite(estimate)):
            continue
        estimate += (x0, y0)
        if np.linalg.norm(estimate - point) <= 10 and (best is None or stats["score"] > best[1]):
            best = (estimate, stats["score"])
    result["verify_ms"] = (time.perf_counter() - start) * 1000
    if best is not None:
        result.update(verified=best[0], verified_inliers=best[1])
    return result


def job(task):
    cv2.setNumThreads(1)
    sites = task["sites"]
    site = next(s for s in sites if s["site"] == task["site"])
    rng = np.random.default_rng(task["seed"])
    grey_q, valid_q = B.load(site["files"][-1])
    grey_m, valid_m = B.load(site["files"][0])
    shot = B.render(grey_q, valid_q, np.array(task["centre"], float), B.CONDITIONS[task["condition"]], rng)
    if not shot.valid:
        return []
    truth = B.apply(shot.image_to_ground, (B.QUERY / 2, B.QUERY / 2))
    regions = []
    for size in SIZES:
        if size > min(grey_m.shape):
            continue
        lo = np.maximum(truth - size + 100, 0)
        hi = np.minimum(truth - 100, np.array(grey_m.shape[::-1]) - size)
        if np.any(hi < lo):
            continue
        origin = np.floor(rng.uniform(lo, hi)).astype(int)
        regions.append((f"window{size}", origin, grey_m, valid_m, size, True, None, None))

    full_mosaic, full_valid, offsets, bounds = _MOSAICS["full"]
    regions.extend([
        ("site", np.zeros(2, int), grey_m, valid_m, None, True, None, None),
        ("mosaic", np.zeros(2, int), full_mosaic, full_valid, None, True, None, _COARSE_MOSAIC),
        ("mosaic_without_site", np.zeros(2, int), full_mosaic, full_valid, None, False,
         bounds[task["site"]], _COARSE_MOSAIC),
    ])
    rows = []
    for name, origin, grey, valid, size, present, exclude_bounds, coarse in regions:
        if size:
            reference = grey[origin[1]:origin[1] + size, origin[0]:origin[0] + size]
            mask = valid[origin[1]:origin[1] + size, origin[0]:origin[0] + size]
        else:
            reference, mask = grey, valid
        target = truth - origin
        if name == "mosaic":
            target = truth + offsets[task["site"]]
        result = search(shot.image, reference, mask, coarse, exclude_bounds)
        area = float(mask.sum()) / 1e6
        if exclude_bounds is not None:
            area -= exclude_bounds[4] / 1e6

        def err(key):
            return float(np.linalg.norm(result[key] - target)) if key in result and present else np.nan

        rows.append(dict(site=task["site"], land_cover=site["land_cover"], centre_id=task["centre_id"],
                         condition=task["condition"], region=name, area_km2=area, present=present,
                         top1_score=result.get("top1_score", np.nan), second_score=result.get("second_score", np.nan),
                         quad_n=result.get("quad_n", 0), top1_found="top1" in result,
                         top1_err=err("top1"), verified="verified" in result, verified_err=err("verified"),
                         verified_inliers=result.get("verified_inliers", 0.),
                         zncc_ms=result.get("zncc_ms", np.nan), verify_ms=result.get("verify_ms", np.nan)))
    return rows


_MOSAICS = {}
_COARSE_MOSAIC = None


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pairs", type=Path, default=B.ROOT / "data/raw/aerial_pairs")
    p.add_argument("--output", type=Path, default=B.ROOT / "data/processed/r_lost_mode")
    p.add_argument("--per-site", type=int, default=12)
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--exclude", default="oam_e9d0dc", help="oam_e9d0dc duplicates oam_wufeng (same ground)")
    a = p.parse_args()
    cv2.setNumThreads(1)
    B.xfeat()  # Initialize PyTorch before fork; workers inherit the read-only matcher.
    sites = [s for s in B.discover_sites(a.pairs) if s["site"] not in set(a.exclude.split(","))]
    rng = np.random.default_rng(a.seed)
    B.build_units(sites, [], [], 1000, np.random.default_rng(a.seed))
    _MOSAICS["full"] = full = build_mosaic(sites)
    global _COARSE_MOSAIC
    _COARSE_MOSAIC = prepare_coarse(full[0], full[1])
    tasks = []
    for site in sites:
        for cid in rng.permutation(len(site["centres"]))[:a.per_site]:
            for condition in CONDS:
                tasks.append(dict(sites=sites, site=site["site"], centre=site["centres"][cid],
                                  centre_id=int(cid), condition=condition,
                                  seed=B.seed_of(site["site"], cid, condition)))
    print(f"{len(sites)} sites, mosaic {full[0].shape}, valid {full[1].sum()/1e6:.1f} km2, {len(tasks)} tasks", flush=True)
    start = time.perf_counter()
    rows = []
    with get_context("fork").Pool(a.workers) as pool:
        for i, part in enumerate(pool.imap_unordered(job, tasks)):
            rows.extend(part)
            if (i + 1) % 50 == 0:
                print(f"{i+1}/{len(tasks)} {time.perf_counter()-start:.0f} s", flush=True)
    df = pd.DataFrame(rows)
    a.output.mkdir(parents=True, exist_ok=True)
    df.to_csv(a.output / "lost_mode.csv", index=False)
    summary = []
    for (region, condition), group in df.groupby(["region", "condition"]):
        present = bool(group.present.iloc[0])
        summary.append(dict(region=region, condition=condition, n=len(group),
                            area_km2=round(group.area_km2.median(), 2),
                            top1_correct=float((group.top1_err <= 10).mean()) if present else np.nan,
                            quad_accept_correct=float(((group.quad_n >= 3) & (group.top1_err <= 10)).mean())
                            if present else np.nan,
                            quad_accept_wrong=int(((group.quad_n >= 3) & ~(group.top1_err <= 25)).sum()),
                            verified_correct=float((group.verified & (group.verified_err <= 10)).mean())
                            if present else np.nan,
                            verified_wrong=int((group.verified & ~(group.verified_err <= 25)).sum()),
                            zncc_ms_p50=float(group.zncc_ms.median()),
                            verify_ms_p50=float(group.verify_ms.median())))
    result = pd.DataFrame(summary)
    result.to_csv(a.output / "summary.csv", index=False)
    (a.output / "run.json").write_text(json.dumps(dict(
        protocol=__doc__.strip().split("\n\n")[1], sites=[site["site"] for site in sites],
        seed=a.seed, per_site=a.per_site, conditions=CONDS, excluded=a.exclude.split(","),
        workers=a.workers, mosaic_shape=full[0].shape, mosaic_valid_km2=float(full[1].sum() / 1e6),
        coarse_factor=8, coarse_peaks=40, refine_radius_px=64,
        seconds=time.perf_counter() - start), indent=2) + "\n")
    pd.set_option("display.width", 220)
    print(result.to_string(index=False))



if __name__ == "__main__":
    main()

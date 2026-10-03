"""Cross-date Wufeng localization with spatial calibration/test separation.

Real orthophotos, synthetic camera crops/priors; NOT real flight validation.
Run from repo root. Optional XFeat source/checkpoint must already be local.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rasterio
from rasterio.transform import from_origin
from rasterio.warp import Resampling, reproject

QUERY = 160
SEARCH = 384
SOURCE = 240


def load_maps(directory: Path):
    paths = [directory / f"wufeng_{date}_x4.tif" for date in ("2018-05-03", "2020-03-23")]
    with rasterio.open(paths[0]) as a, rasterio.open(paths[1]) as b:
        assert a.crs == b.crs and a.crs.to_epsg() == 3826
        left, right = max(a.bounds.left, b.bounds.left), min(a.bounds.right, b.bounds.right)
        bottom, top = max(a.bounds.bottom, b.bounds.bottom), min(a.bounds.top, b.bounds.top)
        transform = from_origin(left, top, 1, 1)
        shape = (int(top - bottom), int(right - left))
        maps, masks = [], []
        for ds in (a, b):
            rgb = np.zeros((3, *shape), dtype=np.uint8)
            for band in range(3):
                reproject(rasterio.band(ds, band + 1), rgb[band], dst_transform=transform,
                          dst_crs=ds.crs, resampling=Resampling.average)
            maps.append(cv2.cvtColor(rgb.transpose(1, 2, 0), cv2.COLOR_RGB2GRAY))
            masks.append((rgb.sum(axis=0) > 0).astype(np.uint8))
    return maps, masks, transform, paths


def crop(image, centre, size):
    x, y = np.rint(centre).astype(int)
    h = size // 2
    if y - h < 0 or x - h < 0 or y + h > image.shape[0] or x + h > image.shape[1]:
        raise ValueError(f"crop of {size} px at {(x, y)} leaves the {image.shape} image")
    return image[y-h:y+h, x-h:x+h]


def render(image, centre, angle=0., scale=1., degraded=False):
    source = crop(image, centre, SOURCE)
    matrix = cv2.getRotationMatrix2D((SOURCE/2, SOURCE/2), angle, scale)
    matrix[:, 2] += (QUERY - SOURCE) / 2
    query = cv2.warpAffine(source, matrix, (QUERY, QUERY))
    if degraded:
        query = np.clip(cv2.GaussianBlur(query, (0, 0), 1.2) * .65 + 12, 0, 255).astype(np.uint8)
    return query


def template_match(query, reference, valid, augmented=False):
    """No location/angle truth accepted. Search translation, optionally yaw/scale."""
    best_score, best_point = -1., np.array([np.nan, np.nan])
    angles = (-20., -10., 0., 10., 20.) if augmented else (0.,)
    scales = (.9, 1., 1.1) if augmented else (1.,)
    size = 96  # central region avoids rotation padding for all hypotheses
    valid_windows = cv2.matchTemplate(valid.astype(np.float32), np.ones((size, size), np.float32), cv2.TM_CCORR)
    for angle in angles:
        for scale in scales:
            warped = cv2.warpAffine(query, cv2.getRotationMatrix2D((QUERY/2, QUERY/2), angle, scale), (QUERY, QUERY))
            template = crop(warped, (QUERY/2, QUERY/2), size)
            if template.std() < 2:
                continue
            scores = cv2.matchTemplate(reference, template, cv2.TM_CCOEFF_NORMED)
            scores[valid_windows < .99 * size**2] = -1
            _, score, _, point = cv2.minMaxLoc(scores)
            if score > best_score:
                best_score, best_point = float(score), np.array(point, float) + size / 2
    return best_point, best_score


def load_xfeat(root: Path):
    import torch
    torch.set_num_threads(1)
    sys.path.insert(0, str(root.resolve()))
    from modules.xfeat import XFeat
    return XFeat(weights=str(root / "weights/xfeat.pt"), top_k=2048)


def learned_match(model, query, reference):
    a, b = model.match_xfeat(cv2.cvtColor(query, cv2.COLOR_GRAY2RGB),
                             cv2.cvtColor(reference, cv2.COLOR_GRAY2RGB), top_k=2048)
    if len(a) < 6:
        return np.array([np.nan, np.nan]), 0.
    matrix, mask = cv2.estimateAffinePartial2D(a, b, method=cv2.RANSAC,
                                             ransacReprojThreshold=3., maxIters=3000, confidence=.995)
    if matrix is None or mask is None:
        return np.array([np.nan, np.nan]), 0.
    keep = mask.ravel().astype(bool)
    scale = np.linalg.norm(matrix[:, 0])
    if not (.6 < scale < 1.5) or keep.mean() < .2 or keep.sum() < 6:
        return np.array([np.nan, np.nan]), 0.
    # Reject a line or tiny texture cluster, not merely a high inlier count.
    spread = np.linalg.eigvalsh(np.cov(a[keep].T))
    if spread.min() < 25:
        return np.array([np.nan, np.nan]), 0.
    centre = matrix @ np.array([QUERY/2, QUERY/2, 1.])
    if np.any(centre < 0) or np.any(centre >= reference.shape[::-1]):
        return np.array([np.nan, np.nan]), 0.
    return centre, float(keep.sum())


def select_centres(masks, count, rng):
    h, w = masks[0].shape
    groups = {"calibration": [], "test": []}
    # Margin keeps the search window inside the map for a prior error of up to 64 px.
    for y in range(SEARCH//2+64, h-SEARCH//2-64, 80):
        # 600+m gap between query footprints; no query scene appears in both sets.
        split = "calibration" if y < h*.4-SEARCH/2 else "test" if y > h*.6+SEARCH/2 else None
        if split is None:
            continue
        for x in range(SEARCH//2+64, w-SEARCH//2-64, 80):
            if crop(masks[1], (x, y), SOURCE).mean() < .995:
                continue
            if crop(masks[0], (x, y), QUERY).mean() < .995:
                continue
            groups[split].append((x, y))
    for split, points in groups.items():
        rng.shuffle(points)
        groups[split] = points[:count]
        if len(groups[split]) < 8:
            raise ValueError(f"Only {len(groups[split])} valid {split} centres; inspect imagery coverage")
    return groups


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("data/raw/aerial"))
    parser.add_argument("--output", type=Path, default=Path("data/processed/map_benchmark"))
    parser.add_argument("--xfeat-root", type=Path)
    parser.add_argument("--count", type=int, default=30, help="Maximum independent centres per spatial split")
    parser.add_argument("--seed", type=int, default=20261003)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    cv2.setRNGSeed(args.seed)
    rng = np.random.default_rng(args.seed)
    maps, masks, transform, paths = load_maps(args.data)
    groups = select_centres(masks, args.count, rng)
    model = load_xfeat(args.xfeat_root) if args.xfeat_root else None
    methods = ["zncc", "zncc_yaw_scale"] + (["xfeat_affine"] if model is not None else [])
    rows = []
    examples = []
    for split, centres in groups.items():
        for index, centre_tuple in enumerate(centres):
            centre = np.array(centre_tuple)
            # Simulated imperfect prior; estimator never receives the true centre.
            prior = centre + rng.integers(-60, 61, 2)
            reference = crop(maps[0], prior, SEARCH)
            valid = crop(masks[0], prior, SEARCH)
            alternatives = [p for p in centres if np.linalg.norm(np.array(p)-prior) > SEARCH]
            if not alternatives:
                raise ValueError("Cannot generate a spatially distinct negative in this split")
            negative = np.array(alternatives[rng.integers(len(alternatives))])
            for condition in ("aligned", "yaw_scale_blur"):
                angle, scale = (0., 1.) if condition == "aligned" else (15., 1.07)
                for present, source in ((True, centre), (False, negative)):
                    query = render(maps[1], source, angle, scale, condition != "aligned")
                    for method in methods:
                        start = time.perf_counter()
                        if method == "xfeat_affine":
                            point, score = learned_match(model, query, reference)
                        else:
                            point, score = template_match(query, reference, valid, method != "zncc")
                        elapsed = (time.perf_counter()-start)*1000
                        estimate = point + prior - SEARCH/2
                        error = float(np.linalg.norm(estimate-source)) if np.isfinite(point).all() else None
                        rows.append(dict(split=split, centre_id=index, condition=condition, present=present,
                                         method=method, score=score, error_m=error, latency_ms=elapsed,
                                         truth_x=int(source[0]), truth_y=int(source[1]),
                                         prior_x=int(prior[0]), prior_y=int(prior[1]),
                                         estimate_x=float(estimate[0]), estimate_y=float(estimate[1])))
                    if split == "test" and index < 3 and present and condition != "aligned":
                        examples.append((query, reference, centre-prior+SEARCH/2))
        print(f"{split}: {len(centres)} centres completed", flush=True)
    df = pd.DataFrame(rows)
    summary = []
    thresholds = {}
    for method in methods:
        cal = df[(df.split == "calibration") & (df.method == method)]
        wrong = (~cal.present) | (cal.error_m > 25) | cal.error_m.isna()
        threshold = float(np.nextafter(cal.loc[wrong, "score"].max(), np.inf))
        thresholds[method] = threshold
        df.loc[df.method == method, "accepted"] = (df.score >= threshold) & df.error_m.notna()
        for condition in ("aligned", "yaw_scale_blur"):
            test = df[(df.split == "test") & (df.method == method) & (df.condition == condition)]
            pos = test[test.present]
            neg = test[~test.present]
            accepted = pos[pos.accepted == True]
            summary.append(dict(method=method, condition=condition, threshold=threshold,
                                positive_n=len(pos), negative_n=len(neg),
                                raw_success_10m=float((pos.error_m <= 10).mean()),
                                raw_median_error_m=float(pos.error_m.median()) if pos.error_m.notna().any() else None,
                                accepted_n=len(accepted), accepted_correct_10m=int((accepted.error_m <= 10).sum()),
                                accepted_wrong_25m=int((accepted.error_m > 25).sum()),
                                negative_accepted=int(neg.accepted.sum()),
                                latency_p50_ms=float(test.latency_ms.median()),
                                latency_p95_ms=float(test.latency_ms.quantile(.95))))
    # Pre-registered after the first exploratory run: cheapest method first, first calibrated acceptance wins.
    order = [m for m in ("zncc", "xfeat_affine", "zncc_yaw_scale") if m in methods]
    cascade = []
    for (_, condition, present), group in df[df.split == "test"].groupby(["centre_id", "condition", "present"]):
        group = group.set_index("method")
        latency, accepted, error = 0., False, np.nan
        for method in order:
            latency += group.at[method, "latency_ms"]
            if group.at[method, "accepted"]:
                accepted, error = True, group.at[method, "error_m"]
                break
        cascade.append(dict(condition=condition, present=present, accepted=accepted, error_m=error, latency_ms=latency))
    cascade = pd.DataFrame(cascade)
    for condition, group in cascade.groupby("condition"):
        pos, neg = group[group.present], group[~group.present]
        accepted = pos[pos.accepted]
        summary.append(dict(method="cascade:" + ">".join(order), condition=condition, threshold=None,
                            positive_n=len(pos), negative_n=len(neg), raw_success_10m=None, raw_median_error_m=None,
                            accepted_n=len(accepted), accepted_correct_10m=int((accepted.error_m <= 10).sum()),
                            accepted_wrong_25m=int((accepted.error_m > 25).sum()),
                            negative_accepted=int(neg.accepted.sum()),
                            latency_p50_ms=float(group.latency_ms.median()),
                            latency_p95_ms=float(group.latency_ms.quantile(.95))))
    df.to_csv(args.output / "matches.csv", index=False)
    report = dict(protocol="Real 2018/2020 orthophotos, synthetic nadir queries and prior; 1m/pixel. Spatial split, threshold strictly above every calibration false score. No flight, stereo, barometer or independent surveyed truth.",
                  seed=args.seed, platform=platform.platform(), python=platform.python_version(),
                  numpy=np.__version__, opencv=cv2.__version__,
                  source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
                  transform=list(transform), counts={k: len(v) for k, v in groups.items()}, thresholds=thresholds, results=summary)
    (args.output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    fig, axes = plt.subplots(len(examples), 2, figsize=(8, 3*len(examples)), squeeze=False)
    for axes_row, (query, reference, target) in zip(axes, examples):
        axes_row[0].imshow(query, cmap="gray"); axes_row[0].set_title("2020 synthetic query: yaw/scale/blur")
        axes_row[1].imshow(reference, cmap="gray"); axes_row[1].scatter(*target, marker="+", c="red"); axes_row[1].set_title("2018 map; red = evaluation truth only")
    fig.tight_layout(); fig.savefig(args.output / "examples.png", dpi=130); plt.close(fig)
    print(pd.DataFrame(summary).to_string(index=False))


if __name__ == "__main__":
    main()

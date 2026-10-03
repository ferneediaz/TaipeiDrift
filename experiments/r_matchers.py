#!/usr/bin/env python
"""CPU benchmark of learned / classic image matchers for GNSS-denied aerial localisation.

Compares seven matchers on the same multi-date orthophoto test pair (Wufeng, Taichung):

    xfeat_mnn, xfeat_lighterglue, disk_lightglue, aliked_lightglue,
    sift_lightglue, tiny_roma, roma_outdoor

Test pair: data/raw/aerial/wufeng_2020-03-23_x4.tif (query crop, 160x160 px @ 1 m/px)
vs data/raw/aerial/wufeng_2018-05-03_x4.tif (reference crop, 384x384 px @ 1 m/px),
both resampled to EPSG:3826 at 1 m/px, centred on the same geographic point.
10 centres are sampled from the overlapping footprint.

Usage:
    .venv/bin/python experiments/r_matchers.py --threads 1
    .venv/bin/python experiments/r_matchers.py --threads 10
    .venv/bin/python experiments/r_matchers.py --threads 1 --matchers xfeat_mnn disk_lightglue

Outputs:
    data/processed/r_matchers/timing.csv
    data/processed/r_matchers/timing.json

Only civil navigation estimation research is intended with this code.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import resource
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
MODELS = REPO / "data" / "raw" / "models"
XFEAT_REPO = MODELS / "accelerated_features"
os.environ.setdefault("TORCH_HOME", str(MODELS / "torch_hub"))
os.environ.setdefault("HF_HOME", str(MODELS / "hf"))
os.environ.setdefault("OMP_NUM_THREADS", os.environ.get("OMP_NUM_THREADS", "1"))

if not XFEAT_REPO.is_dir():
    raise SystemExit(
        f"XFeat source not found at {XFEAT_REPO}. Expected an XFeat repo clone with "
        "weights/xfeat.pt and weights/xfeat-lighterglue.pt"
    )
sys.path.insert(0, str(XFEAT_REPO))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rasterio  # noqa: E402
from rasterio.windows import from_bounds  # noqa: E402
import torch  # noqa: E402
from PIL import Image  # noqa: E402

torch.set_grad_enabled(False)
try:
    torch.set_float32_matmul_precision("highest")
except Exception:  # pragma: no cover
    pass

QUERY_TIF = REPO / "data" / "raw" / "aerial" / "wufeng_2020-03-23_x4.tif"
REFERENCE_TIF = REPO / "data" / "raw" / "aerial" / "wufeng_2018-05-03_x4.tif"
OUT_DIR = REPO / "data" / "processed" / "r_matchers"

QUERY_SIZE = 160
REFERENCE_SIZE = 384
GSD_M = 1.0
N_CENTRES = 10
TOP_K = 2048

MATCHERS = [
    "xfeat_mnn",
    "xfeat_lighterglue",
    "disk_lightglue",
    "aliked_lightglue",
    "sift_lightglue",
    "tiny_roma",
    "roma_outdoor",
]

# Model settings that materially change results are recorded with every row.
SETTINGS = {
    "xfeat_mnn": {"top_k": TOP_K},
    "xfeat_lighterglue": {"top_k": TOP_K, "min_conf": 0.1},
    "disk_lightglue": {"n": TOP_K, "disk_max": 2048},
    "aliked_lightglue": {"max_num_keypoints": TOP_K, "model": "aliked-n16"},
    # kornia's LightGlue default filter_threshold=0.1 rejects *every* SIFT match on
    # cross-date aerial crops this small (measured: 0.4 matches/pair, 0 inliers).
    # Disabling the threshold recovers 114 matches / 8 RANSAC inliers per pair.
    "sift_lightglue": {"num_features": TOP_K, "lightglue_filter_threshold": 0.0},
    "tiny_roma": {"sample_num": 2000},
    "roma_outdoor": {"coarse_res": 280, "upsample_res": 432, "sample_num": 2000},
}

# Licence of the code that produces the matches (weights are downloaded, see
# data/processed/r_matchers/weights_licences.txt).
LICENCES = {
    "xfeat_mnn": "Apache-2.0 (accelerated_features, verlab)",
    "xfeat_lighterglue": "Apache-2.0 (accelerated_features, verlab)",
    "disk_lightglue": "Apache-2.0 (kornia) + DISK weights: see kornia docs",
    "aliked_lightglue": "Apache-2.0 (kornia) + ALIKED weights: see kornia docs",
    "sift_lightglue": "Apache-2.0 (kornia) + LightGlue weights (cvg/LightGlue)",
    "tiny_roma": "romatch 0.1.2 PyPI metadata declares no licence; upstream Parskatt/RoMa is MIT",
    "roma_outdoor": "romatch 0.1.2 PyPI metadata declares no licence; upstream Parskatt/RoMa is MIT",
}

_CACHE: dict[str, object] = {}


# ----------------------------------------------------------------------------- images


def _read_gray_window(src: rasterio.DatasetReader, cx: float, cy: float, size_px: int) -> np.ndarray:
    """Read a size_px x size_px window (1 m/px) centred on projected point (cx, cy)."""
    half = size_px * GSD_M / 2.0
    win = from_bounds(cx - half, cy - half, cx + half, cy + half, transform=src.transform)
    data = src.read(window=win, boundless=True, fill_value=0)
    data = np.transpose(data[:3], (1, 2, 0))
    return cv2.resize(data, (size_px, size_px), interpolation=cv2.INTER_AREA)


def _window_valid_fraction(src: rasterio.DatasetReader, cx: float, cy: float, size_px: int) -> float:
    """Fraction of non-nodata pixels in a size_px window, probed on a coarse grid."""
    half = size_px * GSD_M / 2.0
    win = from_bounds(cx - half, cy - half, cx + half, cy + half, transform=src.transform)
    probe = max(16, size_px // 8)
    data = src.read(window=win, boundless=True, fill_value=0, out_shape=(3, probe, probe))
    return float((data.max(axis=0) > 0).mean())


def build_test_pairs(
    query_tif: Path,
    reference_tif: Path,
    n_centres: int = N_CENTRES,
    require_valid: float = 0.95,
    cols: int = 28,
    rows: int = 16,
):
    """Return [(name, query_gray_u8, reference_gray_u8, centre_xy)] aligned crops.

    Both scenes only partly overlap and carry nodata outside their flown footprint, so a
    dense candidate grid is scored for nodata-free fractions and the centres that pass
    ``require_valid`` are thinned by farthest-point sampling to stay geographically spread.
    """
    with rasterio.open(query_tif) as q, rasterio.open(reference_tif) as r:
        qb, rb = q.bounds, r.bounds
        lo_x = max(qb.left, rb.left)
        hi_x = min(qb.right, rb.right)
        lo_y = max(qb.bottom, rb.bottom)
        hi_y = min(qb.top, rb.top)
        margin = REFERENCE_SIZE * GSD_M / 2.0 + 20.0
        lon_lo, lon_hi = lo_x + margin, hi_x - margin
        lat_lo, lat_hi = lo_y + margin, hi_y - margin
        candidates = [
            (lon_lo + (lon_hi - lon_lo) * (i + 0.5) / cols, lat_hi - (lat_hi - lat_lo) * (j + 0.5) / rows)
            for j in range(rows)
            for i in range(cols)
        ]
        scored = []
        for cx, cy in candidates:
            v = min(
                _window_valid_fraction(q, cx, cy, QUERY_SIZE),
                _window_valid_fraction(r, cx, cy, REFERENCE_SIZE),
            )
            if v >= require_valid:
                scored.append((v, cx, cy))

        if len(scored) > n_centres:  # farthest-point thinning over the projected plane
            chosen = [max(scored)]
            while len(chosen) < n_centres:
                nx = max(
                    scored,
                    key=lambda c: min(np.hypot(c[1] - p[1], c[2] - p[2]) for p in chosen),
                )
                chosen.append(nx)
            scored = sorted(chosen, key=lambda c: (-c[1], -c[2]))

        pairs = []
        for v, cx, cy in scored:
            qc = _read_gray_window(q, cx, cy, QUERY_SIZE)
            rc = _read_gray_window(r, cx, cy, REFERENCE_SIZE)
            pairs.append((f"c{len(pairs):02d}", qc, rc, (cx, cy)))
        return pairs


def to_gray(img_rgb: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)


def _bchw(gray: np.ndarray) -> torch.Tensor:
    rgb = np.stack([gray] * 3, axis=-1)
    return torch.from_numpy(rgb).permute(2, 0, 1)[None].float() / 255.0


def _chw_gray(gray: np.ndarray) -> torch.Tensor:
    return torch.from_numpy(gray)[None, None].float() / 255.0


# ----------------------------------------------------------------------------- matchers


def _xfeat():
    if "xfeat" not in _CACHE:
        from modules.xfeat import XFeat

        _CACHE["xfeat"] = XFeat(
            weights=str(XFEAT_REPO / "weights" / "xfeat.pt"), top_k=TOP_K
        )
    return _CACHE["xfeat"]


def _xfeat_backbone():
    """A fresh verlab XFeatModel with the standard weights; safe for romatch to mutilate."""
    from modules.model import XFeatModel

    net = XFeatModel().eval()
    net.load_state_dict(torch.load(XFEAT_REPO / "weights" / "xfeat.pt", map_location="cpu"))
    for param in net.parameters():
        param.requires_grad_(False)
    return net


_LG_PRISTINE_CONF = None


def _restore_lightglue_conf():
    """Undo the global `LightGlue.default_conf` hijack done by verlab's lighterglue wrapper.

    kornia stores each feature profile behind a *class-level mutable* `LightGlue.default_conf`,
    and `data/raw/models/accelerated_features/modules/lighterglue.py:31` rebinds that class
    attribute to the xfeat profile. Every LightGlue built after it in the same process then
    inherits `descriptor_dim=96, n_layers=6, num_heads=1` and dies with a state_dict size
    mismatch. Snapshot the pristine dict once (before any matcher exists) and restore it
    before each construction.
    """
    global _LG_PRISTINE_CONF
    import kornia.feature as KF

    if _LG_PRISTINE_CONF is None:
        _LG_PRISTINE_CONF = dict(KF.LightGlue.default_conf)
    else:
        KF.LightGlue.default_conf.clear()
        KF.LightGlue.default_conf.update(_LG_PRISTINE_CONF)


def _lightglue(feature: str):
    key = f"lg_{feature}"
    if key not in _CACHE:
        import kornia.feature as KF

        _restore_lightglue_conf()
        # Only SIFT overrides the LightGlue match threshold; see SETTINGS["sift_lightglue"].
        matcher = KF.LightGlueMatcher(feature, params={"filter_threshold": 0.0} if feature == "sift" else None)
        matcher.eval()
        _CACHE[key] = matcher
    return _CACHE[key]


def _kornia_feature(name: str):
    import kornia.feature as KF

    key = f"feat_{name}"
    if key not in _CACHE:
        if name == "disk":
            model = KF.DISK.from_pretrained("depth")
        elif name == "aliked":
            model = KF.ALIKED.from_pretrained("aliked-n16", max_num_keypoints=TOP_K)
        elif name == "sift":
            model = KF.SIFTFeature(num_features=TOP_K)
        else:  # pragma: no cover
            raise KeyError(name)
        model.eval()
        _CACHE[key] = model
    return _CACHE[key]


def _cos_conf(d0: torch.Tensor, d1: torch.Tensor) -> np.ndarray:
    return (d0 * d1).sum(dim=-1).cpu().numpy()


def _xfeat_features(model, gray: np.ndarray):
    img = model.parse_input(_bchw(gray))
    return model.detectAndCompute(img, top_k=TOP_K)[0]


def run_xfeat_mnn(_model, q: np.ndarray, r: np.ndarray):
    model = _xfeat()
    d0 = _xfeat_features(model, q)
    d1 = _xfeat_features(model, r)
    i0, i1 = model.match(d0["descriptors"], d1["descriptors"])
    if len(i0) == 0:
        return np.zeros((0, 2), np.float32), np.zeros((0, 2), np.float32), np.zeros(0, np.float32)
    return (
        d0["keypoints"][i0].cpu().numpy().astype(np.float32),
        d1["keypoints"][i1].cpu().numpy().astype(np.float32),
        _cos_conf(d0["descriptors"][i0], d1["descriptors"][i1]).astype(np.float32),
    )


def run_xfeat_lighterglue(_model, q: np.ndarray, r: np.ndarray):
    model = _xfeat()
    d0 = _xfeat_features(model, q)
    d1 = _xfeat_features(model, r)
    d0["image_size"] = (QUERY_SIZE, QUERY_SIZE)
    d1["image_size"] = (REFERENCE_SIZE, REFERENCE_SIZE)
    if len(d0["keypoints"]) < 2 or len(d1["keypoints"]) < 2:
        return np.zeros((0, 2), np.float32), np.zeros((0, 2), np.float32), np.zeros(0, np.float32)
    pts0, pts1, idx = model.match_lighterglue(d0, d1, min_conf=0.1)
    conf = _cos_conf(
        d0["descriptors"][idx[:, 0].astype(np.int64)], d1["descriptors"][idx[:, 1].astype(np.int64)]
    )
    return pts0.astype(np.float32), pts1.astype(np.float32), conf.astype(np.float32)


def _lafs_from_keypoints(kpts: torch.Tensor) -> torch.Tensor:
    """Identity-scale LAFs (1, N, 2, 3) from (N, 2) keypoints for LightGlue."""
    import kornia.feature as KF

    n = kpts.shape[0]
    return KF.laf_from_center_scale_ori(
        kpts[None].float(), torch.ones(1, n, 1, 1), torch.zeros(1, n, 1)
    )


def _kornia_lg(feature: str, gray_q: np.ndarray, gray_r: np.ndarray, three_channel: bool):
    model = _kornia_feature(feature)
    iq = _bchw(gray_q) if three_channel else _chw_gray(gray_q)
    ir = _bchw(gray_r) if three_channel else _chw_gray(gray_r)
    f0 = model(iq)
    f1 = model(ir)
    if isinstance(f0, list):  # DISK / ALIKED return one feature object per image
        f0, f1 = f0[0], f1[0]
        lafs0 = _lafs_from_keypoints(f0.keypoints)
        lafs1 = _lafs_from_keypoints(f1.keypoints)
        desc0, desc1 = f0.descriptors, f1.descriptors
    else:  # SIFT returns (lafs, response, descriptors), zero-padded to num_features
        lafs0, resp0, desc0 = f0
        lafs1, resp1, desc1 = f1
        keep0 = (resp0[0] > 0).nonzero().flatten()
        keep1 = (resp1[0] > 0).nonzero().flatten()
        lafs0, desc0 = lafs0[:, keep0], desc0[:, keep0]
        lafs1, desc1 = lafs1[:, keep1], desc1[:, keep1]
    matcher = _lightglue(feature)
    hw0 = (gray_q.shape[1], gray_q.shape[0])  # (H, W)
    hw1 = (gray_r.shape[1], gray_r.shape[0])
    # LightGlueMatcher.forward expects (N, D) descriptors: it early-returns "no match"
    # when desc.shape[0] < 2, so a (1, N, D) batch dim makes every pair match zero.
    if desc0.dim() == 3:
        desc0 = desc0[0]
    if desc1.dim() == 3:
        desc1 = desc1[0]
    dists, idxs = matcher(desc0, desc1, lafs0, lafs1, hw0, hw1)
    if idxs.numel() == 0:
        return np.zeros((0, 2), np.float32), np.zeros((0, 2), np.float32), np.zeros(0, np.float32)
    import kornia.feature as KF

    k0 = KF.get_laf_center(lafs0)[0][idxs[:, 0]].cpu().numpy()
    k1 = KF.get_laf_center(lafs1)[0][idxs[:, 1]].cpu().numpy()
    return k0.astype(np.float32), k1.astype(np.float32), dists[:, 0].cpu().numpy().astype(np.float32)


def run_disk_lightglue(_m, q, r):
    return _kornia_lg("disk", q, r, three_channel=True)


def run_aliked_lightglue(_m, q, r):
    return _kornia_lg("aliked", q, r, three_channel=True)


def run_sift_lightglue(_m, q, r):
    return _kornia_lg("sift", q, r, three_channel=False)


def _roma(name: str):
    if name not in _CACHE:
        import romatch

        torch.set_float32_matmul_precision("highest")
        if name == "roma_outdoor":
            model = romatch.roma_outdoor(
                "cpu", coarse_res=SETTINGS["roma_outdoor"]["coarse_res"],
                upsample_res=SETTINGS["roma_outdoor"]["upsample_res"],
            )
        else:
            # TinyRoMa.__init__ deletes heatmap_head/keypoint_head/fine_matcher from the
            # XFeatModel it receives (romatch/models/tiny.py:41). Hand it a dedicated copy:
            # sharing the cached XFeat corrupts xfeat_mnn / xfeat_lighterglue for the rest of
            # the process (AttributeError: 'XFeatModel' object has no attribute 'heatmap_head').
            model = romatch.tiny_roma_v1_outdoor("cpu", xfeat=_xfeat_backbone())
        model.eval()
        _CACHE[name] = model
    return _CACHE[name]


def _run_roma(name: str, q: np.ndarray, r: np.ndarray):
    model = _roma(name)
    im_q = Image.fromarray(np.stack([q] * 3, axis=-1)).convert("RGB")
    im_r = Image.fromarray(np.stack([r] * 3, axis=-1)).convert("RGB")
    with torch.inference_mode():
        warp, certainty = model.match(im_q, im_r, batched=True)
        matches, cert = model.sample(warp, certainty, num=SETTINGS[name]["sample_num"])
        k0, k1 = model.to_pixel_coordinates(
            matches, QUERY_SIZE, QUERY_SIZE, REFERENCE_SIZE, REFERENCE_SIZE
        )
    return (
        k0.cpu().numpy().astype(np.float32),
        k1.cpu().numpy().astype(np.float32),
        cert.cpu().numpy().astype(np.float32),
    )


def run_tiny_roma(_m, q, r):
    return _run_roma("tiny_roma", q, r)


def run_roma_outdoor(_m, q, r):
    return _run_roma("roma_outdoor", q, r)


RUNNERS = {
    "xfeat_mnn": run_xfeat_mnn,
    "xfeat_lighterglue": run_xfeat_lighterglue,
    "disk_lightglue": run_disk_lightglue,
    "aliked_lightglue": run_aliked_lightglue,
    "sift_lightglue": run_sift_lightglue,
    "tiny_roma": run_tiny_roma,
    "roma_outdoor": run_roma_outdoor,
}


def match(name: str, query_gray: np.ndarray, reference_gray: np.ndarray):
    """Return (pts_query Nx2 f32, pts_reference Nx2 f32, confidence N f32)."""
    if name not in RUNNERS:
        raise KeyError(f"unknown matcher {name!r}; known: {sorted(RUNNERS)}")
    _restore_lightglue_conf()  # capture the pristine conf before any matcher can hijack it
    with torch.inference_mode():
        pts_q, pts_r, conf = RUNNERS[name](None, query_gray, reference_gray)
    n = min(len(pts_q), len(pts_r))
    return (
        np.asarray(pts_q, np.float32)[:n],
        np.asarray(pts_r, np.float32)[:n],
        np.asarray(conf, np.float32)[:n],
    )


def inlier_count(pts_q: np.ndarray, pts_r: np.ndarray, thresh_px: float = 3.0) -> int:
    if len(pts_q) < 3:
        return 0
    _, mask = cv2.estimateAffinePartial2D(
        pts_q.reshape(-1, 1, 2), pts_r.reshape(-1, 1, 2),
        method=cv2.RANSAC, ransacReprojThreshold=thresh_px, maxIters=5000, confidence=0.999,
    )
    return 0 if mask is None else int(mask.sum())


# Both crops are read from rasters on the same EPSG:3826 / 1 m grid and are centred on the
# same projected point, so the query centre must land on the reference centre.
QUERY_CENTRE = (QUERY_SIZE / 2.0 - 0.5, QUERY_SIZE / 2.0 - 0.5)
REFERENCE_CENTRE = (REFERENCE_SIZE / 2.0 - 0.5, REFERENCE_SIZE / 2.0 - 0.5)


def centre_error_m(pts_q: np.ndarray, pts_r: np.ndarray, thresh_px: float = 3.0):
    """Similarity fit from matches, then the localisation error of the crop centre.

    Returns (error_m, inliers) or (None, inliers) when the fit fails. 1 px == 1 m here.
    """
    if len(pts_q) < 3:
        return None, 0
    M, mask = cv2.estimateAffinePartial2D(
        pts_q.reshape(-1, 1, 2), pts_r.reshape(-1, 1, 2),
        method=cv2.RANSAC, ransacReprojThreshold=thresh_px, maxIters=5000, confidence=0.999,
    )
    ninl = 0 if mask is None else int(mask.sum())
    if M is None:
        return None, ninl
    pred = M @ np.array([QUERY_CENTRE[0], QUERY_CENTRE[1], 1.0])
    return float(np.hypot(pred[0] - REFERENCE_CENTRE[0], pred[1] - REFERENCE_CENTRE[1])), ninl


def rss_mb() -> float:
    rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return rss / (1024 * 1024) if platform.system() == "Darwin" else rss / 1024


# ----------------------------------------------------------------------------- driver


def benchmark(
    matchers: list[str],
    threads: int,
    query_tif: Path = QUERY_TIF,
    reference_tif: Path = REFERENCE_TIF,
    n_centres: int = N_CENTRES,
    repeats: int = 1,
) -> list[dict]:
    torch.set_num_threads(threads)
    pairs = build_test_pairs(query_tif, reference_tif, n_centres)
    if len(pairs) < n_centres:
        print(f"warning: only {len(pairs)} usable centres found", file=sys.stderr)

    rows = []
    for name in matchers:
        row = {
            "matcher": name,
            "threads": threads,
            "n_pairs": len(pairs),
            "status": "ok",
            "error": "",
            "median_ms": float("nan"),
            "p95_ms": float("nan"),
            "mean_matches": float("nan"),
            "mean_inliers": float("nan"),
            "rss_mb": float("nan"),
            "licence": LICENCES.get(name, ""),
            "settings": json.dumps(SETTINGS.get(name, {}), sort_keys=True),
            "per_pair_ms": [],
        }
        try:
            # warm-up (model load + first forward), not timed
            t_warm = time.perf_counter()
            pts_q, pts_r, _ = match(name, to_gray(pairs[0][1]), to_gray(pairs[0][2]))
            row["load_and_warmup_ms"] = round((time.perf_counter() - t_warm) * 1000, 1)
            row["warmup_matches"] = int(len(pts_q))

            times, nm, ni = [], [], []
            for rep in range(repeats):
                for pid, q_rgb, r_rgb, _centre in pairs:
                    t0 = time.perf_counter()
                    pts_q, pts_r, _ = match(name, to_gray(q_rgb), to_gray(r_rgb))
                    times.append((time.perf_counter() - t0) * 1000.0)
                    nm.append(len(pts_q))
                    ni.append(inlier_count(pts_q, pts_r))
            row["per_pair_ms"] = [round(t, 2) for t in times]
            row["median_ms"] = round(float(np.median(times)), 2)
            row["p95_ms"] = round(float(np.percentile(times, 95)), 2)
            row["mean_matches"] = round(float(np.mean(nm)), 1)
            row["mean_inliers"] = round(float(np.mean(ni)), 1)
            row["rss_mb"] = round(rss_mb(), 1)
            print(
                f"{name:20s} threads={threads:<3d} median={row['median_ms']:8.1f} ms "
                f"p95={row['p95_ms']:8.1f} ms matches={row['mean_matches']:7.1f} "
                f"inliers={row['mean_inliers']:6.1f}",
                flush=True,
            )
        except Exception as exc:  # keep the zoo running when one model fails
            row["status"] = "error"
            row["error"] = f"{type(exc).__name__}: {exc}"[:400]
            row["rss_mb"] = round(rss_mb(), 1)
            print(f"{name:20s} threads={threads:<3d} FAILED {row['error']}", file=sys.stderr, flush=True)
        rows.append(row)
    return rows


LOC_FIELDS = [
    "matcher", "threads", "status", "n_pairs", "n_localised", "median_err_m", "p90_err_m",
    "frac_within_1m", "frac_within_3m", "mean_matches", "mean_inliers", "licence", "settings",
    "error", "per_pair_err_m", "per_pair_inliers",
]


def loc_error_benchmark(
    matchers: list[str],
    threads: int,
    query_tif: Path = QUERY_TIF,
    reference_tif: Path = REFERENCE_TIF,
    n_centres: int = N_CENTRES,
) -> list[dict]:
    """How far does the matched correspondence field place the query centre from the truth?"""
    torch.set_num_threads(threads)
    pairs = build_test_pairs(query_tif, reference_tif, n_centres)
    rows = []
    for name in matchers:
        row = {k: "" for k in LOC_FIELDS}
        row.update({
            "matcher": name, "threads": threads, "status": "ok", "n_pairs": len(pairs),
            "n_localised": 0, "median_err_m": float("nan"), "p90_err_m": float("nan"),
            "frac_within_1m": float("nan"), "frac_within_3m": float("nan"),
            "mean_matches": float("nan"), "mean_inliers": float("nan"),
            "licence": LICENCES.get(name, ""),
            "settings": json.dumps(SETTINGS.get(name, {}), sort_keys=True),
            "per_pair_err_m": [], "per_pair_inliers": [],
        })
        try:
            errs, inls, nm = [], [], []
            for _pid, q_rgb, r_rgb, _centre in pairs:
                pts_q, pts_r, _ = match(name, to_gray(q_rgb), to_gray(r_rgb))
                err, ninl = centre_error_m(pts_q, pts_r)
                nm.append(len(pts_q))
                inls.append(ninl)
                errs.append(float("nan") if err is None else round(err, 2))
            ok = np.array([e for e in errs if not np.isnan(e)])
            row["per_pair_err_m"] = errs
            row["per_pair_inliers"] = inls
            row["n_localised"] = int(ok.size)
            if ok.size:
                row["median_err_m"] = round(float(np.median(ok)), 2)
                row["p90_err_m"] = round(float(np.percentile(ok, 90)), 2)
                row["frac_within_1m"] = round(float((ok <= 1.0).mean()), 2)
                row["frac_within_3m"] = round(float((ok <= 3.0).mean()), 2)
            row["mean_matches"] = round(float(np.mean(nm)), 1)
            row["mean_inliers"] = round(float(np.mean(inls)), 1)
            print(
                f"{name:20s} threads={threads:<3d} localised={row['n_localised']}/{len(pairs)} "
                f"median_err={row['median_err_m']:8.2f} m within3m={row['frac_within_3m']}",
                flush=True,
            )
        except Exception as exc:
            row["status"] = "error"
            row["error"] = f"{type(exc).__name__}: {exc}"[:400]
            print(f"{name:20s} FAILED {row['error']}", file=sys.stderr, flush=True)
        rows.append(row)
    return rows


def write_outputs(rows: list[dict], out_dir: Path = OUT_DIR, stem: str = "timing", fields: list[str] | None = None):
    out_dir.mkdir(parents=True, exist_ok=True)
    fields = fields or [
        "matcher", "threads", "status", "median_ms", "p95_ms", "mean_matches", "mean_inliers",
        "rss_mb", "n_pairs", "load_and_warmup_ms", "warmup_matches", "licence", "settings", "error",
    ]
    csv_path = out_dir / f"{stem}.csv"
    write_header = not csv_path.exists()
    with csv_path.open("a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        if write_header:
            w.writeheader()
        for row in rows:
            w.writerow(row)

    json_path = out_dir / f"{stem}.json"
    existing = []
    if json_path.exists():
        try:
            existing = json.loads(json_path.read_text())
        except json.JSONDecodeError:
            existing = []
    existing = [
        r for r in existing
        if not any(r["matcher"] == n["matcher"] and r["threads"] == n["threads"] for n in rows)
    ]
    existing.extend(rows)
    existing.sort(key=lambda r: (r["matcher"], r["threads"]))
    json_path.write_text(json.dumps(existing, indent=2))
    return csv_path, json_path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--matchers", nargs="+", default=MATCHERS, choices=MATCHERS)
    ap.add_argument("--threads", type=int, nargs="+", default=[1, 10])
    ap.add_argument("--query", type=Path, default=QUERY_TIF)
    ap.add_argument("--reference", type=Path, default=REFERENCE_TIF)
    ap.add_argument("--centres", type=int, default=N_CENTRES)
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument(
        "--loc-error",
        action="store_true",
        help="also fit each matcher's correspondence field and report crop-centre error in metres",
    )
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args(argv)

    n_cpu = os.cpu_count() or 1
    torch.set_num_threads(min(args.threads[0], n_cpu))
    all_rows = []
    for threads in args.threads:
        all_rows.extend(
            benchmark(args.matchers, threads, args.query, args.reference, args.centres, args.repeats)
        )
    csv_path, json_path = write_outputs(all_rows, args.out_dir)
    print(f"\nwrote {csv_path}\nwrote {json_path}")
    if args.loc_error:
        loc_rows = loc_error_benchmark(
            args.matchers, min(args.threads[0], n_cpu), args.query, args.reference, args.centres
        )
        lc, lj = write_outputs(loc_rows, args.out_dir, stem="loc_error", fields=LOC_FIELDS)
        print(f"wrote {lc}\nwrote {lj}")
    return all_rows


if __name__ == "__main__":
    main()

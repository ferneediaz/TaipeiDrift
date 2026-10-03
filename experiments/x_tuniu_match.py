"""Matching core for the Tuniu real-photo fixes: query construction, ZNCC (+ quad), learned matchers.

A *query* is a rectified north-up ground patch anchored at the camera nadir (x_tuniu_geo.rectify).
A *reference* is a map window covering every nadir position within +-SEARCH_M of the prior.
Every method returns the map position of the camera NADIR (EPSG:3826) plus integrity features.
"""
from __future__ import annotations

import math
import os
import time

import cv2
import numpy as np

import x_tuniu_geo as G

SEARCH_M = 45.0                 # prior is +-40 m per axis; 5 m margin
QUAD_TOL_M = 4.0                # quad >= 3: four disjoint sub-templates within 4 m of the full fix
ZNCC_ANGLES = (-4.0, -2.0, 0.0, 2.0, 4.0)
ZNCC_SCALES = (0.94, 1.0, 1.06)
MIN_VALID_OVERLAP = 0.99        # template positions overlapping > 1 % map no-data are not scored
LEARNED = ("aliked-lightglue", "disk-lightglue", "xfeat", "roma")
H_SCALE_RANGE = (0.8, 1.25)     # plausibility of the query->map homography (rectified, metric patches)
H_MAX_ROT_DEG = 12.0
H_MAX_ANISO = 1.3


# ----------------------------------------------------------------------------- queries

class QueryConfig:
    """Estimator inputs. height: baro_dem | const100 | rtk_dem(ORACLE); attitude: dji | nadir | yaw3 | yaw7;
    ground: dem_prior (flat plane at DEM under the prior) | takeoff (plane at take-off height) | dem_lifted."""

    def __init__(self, height="baro_dem", attitude="dji", ground="dem_prior", far_m=100.0):
        self.height, self.attitude, self.ground, self.far_m = height, attitude, ground, far_m

    @property
    def name(self):
        return f"h={self.height},att={self.attitude},gnd={self.ground}"


def boresight() -> dict:
    p = G.XOUT / "stage1_calibration.json"
    if p.exists():
        import json
        return json.loads(p.read_text())["boresight_deg"]
    return {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}


def build_query(img: np.ndarray, factor: int, att, cam: G.Camera, res: float, cfg: QueryConfig,
                baro_alt: float, prior_xy, rtk_alt: float | None, takeoff_alt_ell: float, yaw_err: float = 0.0,
                bs: dict | None = None) -> dict:
    """Rectified patch. `rtk_alt` is only used by the ORACLE height mode."""
    bs = bs or boresight()
    yaw = att.gimbal_yaw_deg + bs["yaw"] + yaw_err
    if cfg.attitude == "nadir":
        R = G.rot_enu_cam(yaw, -90.0, 0.0)
    else:
        R = G.rot_enu_cam(yaw, att.gimbal_pitch_deg + bs["pitch"], att.gimbal_roll_deg + bs["roll"])
    dem_prior = float(G.dem_ellipsoidal(prior_xy[0], prior_xy[1]))
    if cfg.height == "const100":
        H = 100.0
    elif cfg.height == "rtk_dem":
        H = rtk_alt - dem_prior
    elif cfg.ground == "takeoff":
        H = baro_alt - takeoff_alt_ell
    else:
        H = baro_alt - dem_prior
    ground = None
    if cfg.ground == "dem_lifted":
        def ground(X, Y):
            return G.dem_ellipsoidal(prior_xy[0] + X, prior_xy[1] + Y) - dem_prior
    q = G.rectify(img, factor, cam, R, H, res, far_m=cfg.far_m if cfg.attitude != "nadir" else 1e3, ground=ground)
    q["height_used"] = H
    return q


def reference_for(m: G.MapRaster, q: dict, prior_xy, search_m: float = SEARCH_M):
    """Map window holding the query for every nadir within +-search_m of prior_xy."""
    res = q["res"]
    rows, cols = q["patch"].shape
    x_min = prior_xy[0] + q["x0"] - search_m
    y_max = prior_xy[1] + q["y0"] + search_m
    W = cols + int(math.ceil(2 * search_m / res)) + 1
    Hh = rows + int(math.ceil(2 * search_m / res)) + 1
    gray, valid, (xc, yc) = m.window(x_min, y_max, W, Hh)
    return dict(gray=gray, valid=valid, xc=xc, yc=yc, res=res)


def ref_px_to_xy(ref, px):
    return np.array([ref["xc"] + px[0] * ref["res"], ref["yc"] - px[1] * ref["res"]])


# ----------------------------------------------------------------------------- ZNCC

def max_rect(mask: np.ndarray) -> tuple[int, int, int, int]:
    """Largest all-True axis-aligned rectangle (r0, c0, r1, c1), exclusive ends. Histogram method."""
    h, w = mask.shape
    heights = np.zeros(w, int)
    best = (0, 0, 0, 0, 0)
    for r in range(h):
        heights = np.where(mask[r], heights + 1, 0)
        stack = []
        for c in range(w + 1):
            hc = heights[c] if c < w else 0
            start = c
            while stack and stack[-1][1] >= hc:
                s, hs = stack.pop()
                area = hs * (c - s)
                if area > best[0]:
                    best = (area, r - hs + 1, s, r + 1, c)
                start = s
            stack.append((start, hc))
    return best[1:]


def template_rect(valid: np.ndarray, nadir, angles, scales) -> tuple[int, int, int, int]:
    """Template rectangle valid under every yaw/scale hypothesis (computed on a <=160 px proxy)."""
    rows, cols = valid.shape
    k = max(1, int(math.ceil(max(rows, cols) / 160)))
    small = cv2.resize(valid.astype(np.uint8), (max(1, cols // k), max(1, rows // k)), interpolation=cv2.INTER_AREA)
    centre = ((cols - 1) / 2 / k, (rows - 1) / 2 / k)
    inter = np.ones_like(small, bool)
    for a in angles:
        for s in scales:
            M = cv2.getRotationMatrix2D(centre, a, s)
            w = cv2.warpAffine(small, M, (small.shape[1], small.shape[0]), flags=cv2.INTER_NEAREST)
            inter &= w > 0
    inter = cv2.erode(inter.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    r0, c0, r1, c1 = max_rect(inter)
    return r0 * k, c0 * k, r1 * k, c1 * k


def _valid_scores(ref_valid: np.ndarray, th: int, tw: int) -> np.ndarray:
    ii = cv2.integral(ref_valid.astype(np.uint8))
    s = ii[th:, tw:] - ii[:-th, tw:] - ii[th:, :-tw] + ii[:-th, :-tw]
    return s >= MIN_VALID_OVERLAP * th * tw


def _match(ref_gray, tmpl, ok):
    s = cv2.matchTemplate(ref_gray, tmpl, cv2.TM_CCOEFF_NORMED)
    s[~ok] = -1
    s[~np.isfinite(s)] = -1
    _, peak, _, loc = cv2.minMaxLoc(s)
    return s, float(peak), np.array(loc, float)


def zncc_fix(q: dict, ref: dict, angles=ZNCC_ANGLES, scales=ZNCC_SCALES) -> dict:
    res = q["res"]
    patch, valid, nadir = q["patch"], q["valid"], q["nadir_px"]
    rows, cols = patch.shape
    r0, c0, r1, c1 = template_rect(valid, nadir, angles, scales)
    th, tw = r1 - r0, c1 - c0
    out = dict(fix=None, score=np.nan, second=np.nan, quad_n=0, tmpl_w_m=tw * res, tmpl_h_m=th * res)
    if th < 16 or tw < 16 or th >= ref["gray"].shape[0] or tw >= ref["gray"].shape[1]:
        return out
    ok = _valid_scores(ref["valid"], th, tw)
    centre = ((cols - 1) / 2, (rows - 1) / 2)
    best = None
    for a in angles:
        for s in scales:
            M = cv2.getRotationMatrix2D(centre, a, s)
            warped = cv2.warpAffine(patch, M, (cols, rows), flags=cv2.INTER_LINEAR)
            t = warped[r0:r1, c0:c1]
            if t.std() < 2:
                continue
            sc, peak, loc = _match(ref["gray"], t, ok)
            if best is None or peak > best["peak"]:
                best = dict(peak=peak, loc=loc, surface=sc, a=a, s=s, warped=warped, M=M)
    if best is None or best["peak"] <= -1:
        return out
    nadir_w = best["M"] @ np.array([nadir[0], nadir[1], 1.0])
    nadir_ref = best["loc"] + nadir_w - np.array([c0, r0])
    sc = best["surface"]
    x, y = best["loc"].astype(int)
    rad = int(math.ceil(3.0 / res))
    masked = sc.copy()
    masked[max(0, y - rad):y + rad + 1, max(0, x - rad):x + rad + 1] = -1
    good = sc[sc > -1]
    # quad: four disjoint sub-templates of the best hypothesis, matched independently
    hh, hw = th // 2, tw // 2
    agree, dists = 0, []
    ok_sub = _valid_scores(ref["valid"], hh, hw)
    for dy in (0, hh):
        for dx in (0, hw):
            t = best["warped"][r0 + dy:r0 + dy + hh, c0 + dx:c0 + dx + hw]
            if t.std() < 2:
                dists.append(np.inf)
                continue
            _, _, loc = _match(ref["gray"], t, ok_sub)
            d = float(np.linalg.norm(loc - (dx, dy) - best["loc"])) * res
            dists.append(d)
            agree += d <= QUAD_TOL_M
    out.update(fix=ref_px_to_xy(ref, nadir_ref), score=best["peak"], second=float(masked.max()),
               z=(best["peak"] - float(good.mean())) / max(1e-6, float(good.std())),
               hyp_angle=best["a"], hyp_scale=best["s"], quad_n=int(agree),
               quad_d=";".join(f"{d:.1f}" for d in sorted(dists)))
    return out


# ----------------------------------------------------------------------------- learned matchers

_MATCHERS: dict = {}


def matcher(name: str):
    if name not in _MATCHERS:
        os.environ.setdefault("TORCH_HOME", str(G.ROOT / "data/raw/models/torch_hub"))
        os.environ.setdefault("HF_HOME", str(G.ROOT / "data/raw/models/hf"))
        from vismatch import get_matcher
        m = get_matcher(name, device="cpu", max_num_keypoints=2048)
        m.skip_ransac = True
        ext = getattr(m, "extractor", None)
        if ext is not None and hasattr(ext, "preprocess_conf"):
            # LightGlue extractors resize the long edge to 1024 px by default (up- or down-sampling);
            # disabled so that each method sees the map resolution under test.
            ext.preprocess_conf = {**ext.preprocess_conf, "resize": None}
        _MATCHERS[name] = m
    return _MATCHERS[name]


def _t3(gray):
    import torch
    return torch.from_numpy(np.ascontiguousarray(np.stack([gray] * 3))).float() / 255.0


def learned_fix(name: str, q: dict, ref: dict) -> dict:
    res = q["res"]
    out = dict(fix=None, matches=0, inliers=0, inlier_ratio=0.0, h_scale=np.nan, h_rot=np.nan, gate="few")
    r = matcher(name)(_t3(q["patch"]), _t3(ref["gray"]))
    a, b = np.asarray(r["matched_kpts0"], np.float64), np.asarray(r["matched_kpts1"], np.float64)
    if len(a):
        qv = cv2.erode(q["valid"].astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        ia = np.clip(np.rint(a).astype(int), 0, [q["valid"].shape[1] - 1, q["valid"].shape[0] - 1])
        ib = np.clip(np.rint(b).astype(int), 0, [ref["valid"].shape[1] - 1, ref["valid"].shape[0] - 1])
        keep = qv[ia[:, 1], ia[:, 0]] & ref["valid"][ib[:, 1], ib[:, 0]]
        a, b = a[keep], b[keep]
    out["matches"] = len(a)
    if len(a) < 8:
        return out
    thr = max(3.0, 1.0 / res)
    Hm, mask = cv2.findHomography(a, b, cv2.USAC_MAGSAC, thr, maxIters=10000, confidence=0.999)
    if Hm is None or mask is None:
        out["gate"] = "nomodel"
        return out
    inl = mask.ravel().astype(bool)
    # local Jacobian of the homography at the inlier centroid (the patch is metric, so ~ rotation)
    p = a[inl].mean(0)
    w = Hm[2, 0] * p[0] + Hm[2, 1] * p[1] + Hm[2, 2]
    u = (Hm[:2, :2] @ p + Hm[:2, 2]) / w
    J = (Hm[:2, :2] - np.outer(u, Hm[2, :2])) / w
    sv = np.linalg.svd(J, compute_uv=False)
    scale = float(np.sqrt(abs(np.linalg.det(J))))
    rot = float(np.degrees(math.atan2(J[1, 0] - J[0, 1], J[0, 0] + J[1, 1])))
    n = Hm @ np.array([q["nadir_px"][0], q["nadir_px"][1], 1.0])
    out.update(inliers=int(inl.sum()), inlier_ratio=float(inl.mean()), h_scale=scale, h_rot=rot,
               h_aniso=float(sv[0] / max(sv[1], 1e-9)))
    if not (H_SCALE_RANGE[0] <= scale <= H_SCALE_RANGE[1]) or abs(rot) > H_MAX_ROT_DEG or out["h_aniso"] > H_MAX_ANISO \
            or abs(n[2]) < 1e-9:
        out["gate"] = "implausible"
        return out
    out["gate"] = "ok"
    out["fix"] = ref_px_to_xy(ref, n[:2] / n[2])
    return out


def run_method(method: str, q: dict, ref: dict) -> dict:
    t0 = time.perf_counter()
    if method == "zncc":
        r = zncc_fix(q, ref)
    else:
        r = learned_fix(method, q, ref)
    r["latency_s"] = time.perf_counter() - t0
    return r

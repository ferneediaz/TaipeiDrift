"""Step 3 - compute the camera ("drone") position for every frame.

Model
-----
The camera looks (almost) straight down from a known height h above a flat
floor.  A floor feature seen at pixel (u, v) is
    1. undistorted with the calibrated lens model -> normalised ray (x, y, 1)
    2. rotated by the fixed camera tilt R_tilt (roll/pitch w.r.t. the floor
       normal, estimated from the video itself, see below)
    3. intersected with the floor:  X = h * r_xy / r_z   [metres]
X is the feature's position on the floor relative to the point directly below
the camera (the nadir).  Between two frames the floor points must then move as
a *rigid* 2-D body (rotation + translation) - that motion IS the motion of
the camera over the ground, in metres, because h is known.

Estimation
----------
* camera tilt: the two angles that make all frame-to-frame floor motions most
  rigid (a wrong tilt makes the projected floor patch look trapezoidal).
* relative altitude per frame: similarity fits between frames give height
  ratios h_j / h_k; solved globally with the mean fixed to the measured height.
* edges: rigid fits (2-point RANSAC + least squares) between frame k and
  k+1, 2, 4, ..., 64 using the KLT tracks, plus SIFT keyframe matches for
  loop closure when the camera comes back to a place it has seen before.
* pose graph: all edges are combined in one robust non-linear least squares
  problem; the start position is the origin.

Output frame: origin = nadir point at frame 0, X axis = image right at
frame 0, Y axis = image up at frame 0 (right-handed, Z up).
"""
from __future__ import annotations

import argparse
import json
import time

import cv2
import numpy as np
import scipy.sparse as sp
from scipy.optimize import least_squares

from common import CACHE, OUT, ensure_dirs, load_calib, undistort_norm

SPANS = (1, 2, 4, 8, 16, 32, 64)
MAX_PTS = 300
SIGMA0 = 5e-5
RNG = np.random.default_rng(1)


# ----------------------------------------------------------------------------- geometry
def rot_tilt(a, b):
    ca, sa, cb, sb = np.cos(a), np.sin(a), np.cos(b), np.sin(b)
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    return Rx @ Ry


def rectify(xy, R, h):
    """normalised coords (...,2) -> floor coords in metres (...,2). h scalar or broadcastable."""
    r = xy[..., 0:1] * R[:, 0] + xy[..., 1:2] * R[:, 1] + R[:, 2]
    return np.asarray(h)[..., None] * r[..., :2] / r[..., 2:3] if np.ndim(h) else h * r[..., :2] / r[..., 2:3]


def kabsch(p, q, w=None, scale=False):
    """Least-squares q ~ s R(th) p + t for 2-D points. Batched over leading axis."""
    if w is None:
        w = np.ones(p.shape[:-1])
    ws = w.sum(-1, keepdims=True)
    pc = (w[..., None] * p).sum(-2) / ws
    qc = (w[..., None] * q).sum(-2) / ws
    P, Q = p - pc[..., None, :], q - qc[..., None, :]
    c = (w * (P[..., 0] * Q[..., 0] + P[..., 1] * Q[..., 1])).sum(-1)
    s_ = (w * (P[..., 0] * Q[..., 1] - P[..., 1] * Q[..., 0])).sum(-1)
    th = np.arctan2(s_, c)
    s = np.hypot(c, s_) / (w * (P ** 2).sum(-1)).sum(-1) if scale else np.ones_like(th)
    R = np.stack([np.stack([np.cos(th), -np.sin(th)], -1), np.stack([np.sin(th), np.cos(th)], -1)], -2)
    t = qc - s[..., None] * np.einsum("...ij,...j->...i", R, pc)
    return th, t, s


def apply(th, t, p, s=1.0):
    c, sn = np.cos(th), np.sin(th)
    return np.stack([s * (c * p[..., 0] - sn * p[..., 1]) + t[0], s * (sn * p[..., 0] + c * p[..., 1]) + t[1]], -1)


def ransac_rigid(p, q, thr, iters=128):
    n = len(p)
    if n < 6:
        return None
    i = RNG.integers(0, n, iters)
    j = RNG.integers(0, n, iters)
    keep = i != j
    i, j = i[keep], j[keep]
    dp, dq = p[j] - p[i], q[j] - q[i]
    th = np.arctan2(dp[:, 0] * dq[:, 1] - dp[:, 1] * dq[:, 0], (dp * dq).sum(1))
    c, s = np.cos(th), np.sin(th)
    t = q[i] - np.stack([c * p[i, 0] - s * p[i, 1], s * p[i, 0] + c * p[i, 1]], 1)
    pr = np.stack([c[:, None] * p[None, :, 0] - s[:, None] * p[None, :, 1],
                   s[:, None] * p[None, :, 0] + c[:, None] * p[None, :, 1]], -1) + t[:, None]
    err = np.linalg.norm(pr - q[None], axis=-1)
    inl = err < thr
    best = inl.sum(1).argmax()
    inl = inl[best]
    for _ in range(3):
        if inl.sum() < 6:
            return None
        th_, t_, _ = kabsch(p[inl], q[inl])
        err = np.linalg.norm(apply(th_, t_, p) - q, axis=1)
        inl = err < thr
    if inl.sum() < 6:
        return None
    th_, t_, _ = kabsch(p[inl], q[inl])
    res = np.linalg.norm(apply(th_, t_, p[inl]) - q[inl], axis=1)
    return th_, t_, inl, float(np.sqrt(np.mean(res ** 2)))


# ----------------------------------------------------------------------------- data
def load_tracks(K, dist):
    d = np.load(CACHE / "tracks.npz")
    f, ids, uv = d["frame"], d["id"], d["uv"]
    print(f"  undistorting {len(uv)} track observations ...")
    xy = np.concatenate([undistort_norm(uv[i:i + 500000], K, dist) for i in range(0, len(uv), 500000)])
    nF = len(d["times"])
    starts = np.searchsorted(f, np.arange(nF + 1))
    return dict(f=f, ids=ids, uv=uv, xy=xy, starts=starts, times=d["times"], nF=nF)


def pair_points(T, k, j, max_pts=MAX_PTS):
    a0, a1 = T["starts"][k], T["starts"][k + 1]
    b0, b1 = T["starts"][j], T["starts"][j + 1]
    _, ia, ib = np.intersect1d(T["ids"][a0:a1], T["ids"][b0:b1], assume_unique=True, return_indices=True)
    if len(ia) > max_pts:
        sel = RNG.choice(len(ia), max_pts, replace=False)
        ia, ib = ia[sel], ib[sel]
    return T["xy"][a0 + ia], T["xy"][b0 + ib]


def collect_pairs(T):
    pairs = []
    for k in range(T["nF"]):
        for d in SPANS:
            j = k + d
            if j >= T["nF"]:
                break
            pk, pj = pair_points(T, k, j)
            if len(pk) < 20:
                break  # longer spans share even fewer tracks
            pairs.append((k, j, pk, pj))
    return pairs


# ----------------------------------------------------------------------------- tilt
def estimate_tilt(pairs, h, n_per=60):
    sel = [p for p in pairs if p[1] - p[0] in (16, 32, 64) and p[0] % 4 == 0 and len(p[2]) >= n_per]
    # fixed outlier rejection with zero tilt first
    P, Q = [], []
    R0 = np.eye(3)
    for k, j, pk, pj in sel:
        r = ransac_rigid(rectify(pj, R0, h), rectify(pk, R0, h), thr=0.003)
        if r is None:
            continue
        idx = np.flatnonzero(r[2])
        if len(idx) < n_per:
            continue
        idx = RNG.choice(idx, n_per, replace=False)
        P.append(pj[idx]); Q.append(pk[idx])
    P, Q = np.array(P), np.array(Q)
    print(f"  tilt estimation from {len(P)} frame pairs")

    def resid(x):
        R = rot_tilt(x[0], x[1])
        p, q = rectify(P, R, h), rectify(Q, R, h)
        th, t, _ = kabsch(p, q)
        pr = np.stack([np.cos(th)[:, None] * p[..., 0] - np.sin(th)[:, None] * p[..., 1],
                       np.sin(th)[:, None] * p[..., 0] + np.cos(th)[:, None] * p[..., 1]], -1) + t[:, None]
        return (pr - q).ravel() * 1000  # mm

    r0 = resid([0, 0])
    sol = least_squares(resid, [0.0, 0.0], loss="soft_l1", f_scale=0.3, x_scale=0.01, diff_step=1e-6)
    J = sol.jac
    cov = np.linalg.inv(J.T @ J) * np.mean(sol.fun ** 2)
    print(f"  rigidity RMS: no tilt {np.sqrt(np.mean(r0**2)):.4f} mm -> with tilt {np.sqrt(np.mean(sol.fun**2)):.4f} mm")
    print(f"  tilt: roll {np.degrees(sol.x[0]):.3f}° pitch {np.degrees(sol.x[1]):.3f}°  "
          f"(± {np.degrees(np.sqrt(cov[0,0])):.3f}°, {np.degrees(np.sqrt(cov[1,1])):.3f}°)")
    return sol.x, np.sqrt(np.diag(cov)), float(np.sqrt(np.mean(r0 ** 2))), float(np.sqrt(np.mean(sol.fun ** 2)))


# ----------------------------------------------------------------------------- altitude
def estimate_altitude(pairs, R, h, nF):
    """Relative height per frame from similarity scale between frames."""
    rows, cols, vals, rhs, wts = [], [], [], [], []
    e = 0
    for k, j, pk, pj in pairs:
        if j - k < 4:
            continue  # short baselines: scale is noisy
        p, q = rectify(pj, R, h), rectify(pk, R, h)
        r = ransac_rigid(p, q, thr=0.002)
        if r is None or r[2].sum() < 30:
            continue
        _, _, s = kabsch(p[r[2]], q[r[2]], scale=True)
        # q (frame k) = s * R p (frame j);  s = h_j / h_k  -> log h_j - log h_k = log s
        rows += [e, e]; cols += [j, k]; vals += [1.0, -1.0]
        spread = np.sqrt(np.mean(np.sum((p[r[2]] - p[r[2]].mean(0)) ** 2, 1)))
        w = spread * np.sqrt(r[2].sum()) / max(r[3], 1e-5)
        rhs.append(np.log(s)); wts.append(w); e += 1
    # anchor: mean log height = 0, plus weak smoothness to bridge gaps
    rows += [e] * nF; cols += list(range(nF)); vals += [1.0 / nF] * nF; rhs.append(0.0); wts.append(1e4); e += 1
    for k in range(nF - 1):
        rows += [e, e]; cols += [k + 1, k]; vals += [1.0, -1.0]; rhs.append(0.0); wts.append(1.0); e += 1
    A = sp.csr_matrix((vals, (rows, cols)), shape=(e, nF))
    W = sp.diags(wts)
    from scipy.sparse.linalg import lsqr
    sol = lsqr(W @ A, W @ np.array(rhs), atol=1e-12, btol=1e-12, iter_lim=20000)[0]
    return h * np.exp(sol)


# ----------------------------------------------------------------------------- edges
def virtual_points(p, q, th, t, n, rms):
    """Summarise a rigid fit by 5 weighted point correspondences (centroid and
    +/- 1 sigma along the principal axes), which reproduces the information of
    the full point set in the pose graph."""
    c = p.mean(0)
    C = np.cov((p - c).T)
    ev, evec = np.linalg.eigh(C)
    v = [c] + [c + sg * np.sqrt(max(ev[i], 1e-8)) * evec[:, i] for i in (0, 1) for sg in (-1, 1)]
    v = np.array(v)
    # per-point noise / sqrt(n) plus a floor for errors shared by all points of the
    # edge (floor relief parallax, tracking correlation): 0.05 mm
    sigma = np.sqrt(max(rms, 2e-5) ** 2 / n + SIGMA0 ** 2)  # metres
    w = np.array([1.0, 0.5, 0.5, 0.5, 0.5]) / sigma
    return v, apply(th, t, v), w


def build_edges(pairs, R, hk, thr=0.0008):
    edges = []
    for k, j, pk, pj in pairs:
        p, q = rectify(pj, R, hk[j]), rectify(pk, R, hk[k])
        r = ransac_rigid(p, q, thr)
        if r is None or r[2].sum() < 15:
            continue
        th, t, inl, rms = r
        v, vk, w = virtual_points(p[inl], q[inl], th, t, inl.sum(), rms)
        edges.append(dict(k=k, j=j, th=th, t=t, v=v, vk=vk, w=w, n=int(inl.sum()), rms=rms, kind="klt"))
    return edges


def keyframe_edges(T, K, dist, R, hk, poses, kind, min_gap, max_dist, thr=0.001):
    d = np.load(CACHE / "keyframes.npz")
    frames, off, kp, desc = d["frames"], d["offsets"], d["kp"], d["desc"]
    xy = undistort_norm(kp, K, dist)
    bf = cv2.BFMatcher(cv2.NORM_L2)
    edges = []
    for a in range(len(frames)):
        for b in range(a + 1, len(frames)):
            fa, fb = frames[a], frames[b]
            gap = fb - fa
            if kind == "bridge" and gap != frames[1] - frames[0]:
                continue
            if kind == "loop":
                if gap < min_gap:
                    continue
                if np.hypot(*(poses[fa, :2] - poses[fb, :2])) > max_dist:
                    continue
            da, db = desc[off[a]:off[a + 1]].astype(np.float32), desc[off[b]:off[b + 1]].astype(np.float32)
            if len(da) < 20 or len(db) < 20:
                continue
            m = bf.knnMatch(db, da, k=2)
            good = [x[0] for x in m if len(x) == 2 and x[0].distance < 0.8 * x[1].distance]
            if len(good) < 20:
                continue
            ib = np.array([g.queryIdx for g in good]); ia = np.array([g.trainIdx for g in good])
            p = rectify(xy[off[b] + ib], R, hk[fb])
            q = rectify(xy[off[a] + ia], R, hk[fa])
            r = ransac_rigid(p, q, thr, iters=400)
            if r is None or r[2].sum() < 25 or r[2].mean() < 0.15:
                continue
            th, t, inl, rms = r
            v, vk, w = virtual_points(p[inl], q[inl], th, t, inl.sum(), rms)
            edges.append(dict(k=int(fa), j=int(fb), th=th, t=t, v=v, vk=vk, w=w, n=int(inl.sum()), rms=rms, kind=kind))
    return edges


def gap_edges(edges, nF, K, dist, R, hk, min_cov=3, reach=6, thr=0.001):
    """Bridge stretches where KLT tracking broke (motion blur, bumps): match the
    frames around the gap directly with SIFT."""
    from common import FLIGHT_VIDEO, iter_frames
    from track import rootsift
    cov = np.zeros(nF - 1, int)
    for e in edges:
        cov[e["k"]:e["j"]] += 1
    weak = np.flatnonzero(cov < min_cov)
    if not len(weak):
        return []
    groups = np.split(weak, np.flatnonzero(np.diff(weak) > 1) + 1)
    mask = cv2.imread(str(OUT / "mask.png"), cv2.IMREAD_GRAYSCALE)
    sift = cv2.SIFT_create(nfeatures=4000, contrastThreshold=0.02)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    bf = cv2.BFMatcher(cv2.NORM_L2)
    out = []
    for g in groups:
        lo, hi = max(0, g[0] - reach + 1), min(nF, g[-1] + reach + 1)
        feats = {}
        for f, _, frame in iter_frames(FLIGHT_VIDEO, start=lo, stop=hi):
            kp, desc = sift.detectAndCompute(clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)), mask)
            if desc is not None and len(kp) > 30:
                feats[f] = (undistort_norm(np.array([p.pt for p in kp]), K, dist), rootsift(desc).astype(np.float32))
        before = [f for f in feats if f <= g[0]]
        after = [f for f in feats if f > g[-1]]
        for fa in before:
            for fb in after:
                xa, da = feats[fa]; xb, db = feats[fb]
                m = bf.knnMatch(db, da, k=2)
                good = [x[0] for x in m if len(x) == 2 and x[0].distance < 0.8 * x[1].distance]
                if len(good) < 25:
                    continue
                ib = np.array([q.queryIdx for q in good]); ia = np.array([q.trainIdx for q in good])
                p, q = rectify(xb[ib], R, hk[fb]), rectify(xa[ia], R, hk[fa])
                r = ransac_rigid(p, q, thr, iters=400)
                if r is None or r[2].sum() < 25:
                    continue
                th, t, inl, rms = r
                v, vk, w = virtual_points(p[inl], q[inl], th, t, inl.sum(), rms)
                out.append(dict(k=int(fa), j=int(fb), th=th, t=t, v=v, vk=vk, w=w, n=int(inl.sum()), rms=rms, kind="gap"))
        print(f"  gap {g[0]}-{g[-1] + 1}: {sum(e['k'] >= lo for e in out)} bridging edges")
    return out


# ----------------------------------------------------------------------------- pose graph
def chain_init(edges, nF):
    """Dead-reckoning initial guess using the shortest available span per frame."""
    best = {}
    for e in edges:
        if e["j"] - e["k"] <= 16 and (e["j"] not in best or e["j"] - e["k"] < best[e["j"]]["j"] - best[e["j"]]["k"]):
            best[e["j"]] = e
    poses = np.zeros((nF, 3))
    for j in range(1, nF):
        e = best.get(j)
        if e is None or e["k"] >= j:
            poses[j] = poses[j - 1]
            continue
        k = e["k"]
        th = poses[k, 2] + e["th"]
        # pose of j: x_w = R(th_k) (R(dth) p + dt) + t_k
        tj = apply(poses[k, 2], poses[k, :2], e["t"][None])[0]
        poses[j] = [tj[0], tj[1], th]
    return poses


def optimise(edges, nF, init, loss="cauchy", f_scale=5.0):
    E = len(edges)
    K_ = np.array([e["k"] for e in edges]); J_ = np.array([e["j"] for e in edges])
    V = np.array([e["v"] for e in edges]); VK = np.array([e["vk"] for e in edges]); W = np.array([e["w"] for e in edges])

    def unpack(x):
        P = np.zeros((nF, 3)); P[1:] = x.reshape(-1, 3); return P

    def fun(x):
        P = unpack(x)
        a = rot2(P[K_, 2], VK) + P[K_, None, :2]
        b = rot2(P[J_, 2], V) + P[J_, None, :2]
        return ((a - b) * W[..., None]).ravel()

    def jac(x):
        P = unpack(x)
        dA = drot2(P[K_, 2], VK) * W[..., None]  # (E,5,2) derivative wrt th_k
        dB = -drot2(P[J_, 2], V) * W[..., None]
        rows, cols, vals = [], [], []
        ridx = np.arange(E * 10).reshape(E, 5, 2)
        for node, d_th, sign in ((K_, dA, 1.0), (J_, dB, -1.0)):
            valid = node > 0
            base = (node - 1) * 3
            for c in range(2):  # translation
                r = ridx[..., c][valid]
                rows.append(r.ravel()); cols.append(np.repeat(base[valid] + c, 5))
                vals.append((sign * W[valid]).ravel())
            r = ridx[valid]
            rows.append(r.ravel()); cols.append(np.repeat(base[valid] + 2, 10)); vals.append(d_th[valid].ravel())
        return sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
                             shape=(E * 10, (nF - 1) * 3))

    # Gauss-Newton with iteratively re-weighted (Cauchy) edges, sparse direct solve
    from scipy.sparse.linalg import spsolve
    x = init[1:].ravel().copy()
    for it in range(30):
        r = fun(x)
        e = np.sqrt((r.reshape(E, 5, 2) ** 2).sum(-1).mean(-1))  # per-edge error in sigma units
        rw = 1.0 / (1.0 + (e / f_scale) ** 2) if loss != "linear" else np.ones(E)
        Wr = sp.diags(np.repeat(rw, 10))
        Jm = jac(x)
        A = (Jm.T @ Wr @ Jm).tocsc()
        b = -(Jm.T @ (rw.repeat(10) * r))
        dx = spsolve(A + sp.identity(A.shape[0]) * 1e-9, b)
        x += dx
        if np.abs(dx).max() < 1e-9:
            break
    r = fun(x).reshape(E, 5, 2)
    return unpack(x), np.sqrt((r ** 2).sum(-1)).mean(-1)


def rot2(th, p):
    c, s = np.cos(th)[:, None], np.sin(th)[:, None]
    return np.stack([c * p[..., 0] - s * p[..., 1], s * p[..., 0] + c * p[..., 1]], -1)


def drot2(th, p):
    c, s = np.cos(th)[:, None], np.sin(th)[:, None]
    return np.stack([-s * p[..., 0] - c * p[..., 1], c * p[..., 0] - s * p[..., 1]], -1)


def edge_err(P, E):
    """Disagreement (mm) of each edge's centroid correspondence with poses P."""
    out = []
    for e in E:
        pred = apply(P[e["k"], 2], P[e["k"], :2], e["vk"][0][None])[0]
        act = apply(P[e["j"], 2], P[e["j"], :2], e["v"][0][None])[0]
        out.append(np.linalg.norm(pred - act) * 1000)
    return np.array(out)


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=0.641, help="camera height above floor [m]")
    ap.add_argument("--use-alt", action="store_true", help="use the per-frame altitude estimate instead of the constant height")
    ap.add_argument("--no-loop", action="store_true")
    ap.add_argument("--alt-smooth", type=int, default=31, help="smoothing window for altitude [frames]")
    ap.add_argument("--reuse", action="store_true", help="reuse cached motion edges")
    a = ap.parse_args()
    ensure_dirs()
    t0 = time.time()
    K, dist, cal = load_calib()
    h = a.height
    print("loading tracks ...")
    T = load_tracks(K, dist)
    nF = T["nF"]
    print("collecting frame pairs ...")
    pairs = collect_pairs(T)
    print(f"  {len(pairs)} frame pairs ({time.time()-t0:.0f}s)")

    print("estimating camera tilt ...")
    tilt, tilt_sd, rig0, rig1 = estimate_tilt(pairs, h)
    R = rot_tilt(*tilt)

    # altitude from image scale (diagnostic). The camera height is constant by
    # construction; tests showed the per-frame estimate does not improve the
    # loop closure, so positions use the measured height unless --use-alt.
    print("estimating relative altitude per frame (diagnostic) ...")
    h_est = estimate_altitude(pairs, R, h, nF)
    if a.alt_smooth > 1:
        from scipy.ndimage import median_filter, uniform_filter1d
        h_est = uniform_filter1d(median_filter(h_est, a.alt_smooth, mode="nearest"), a.alt_smooth, mode="nearest")
        h_est *= h / h_est.mean()
    print(f"  altitude from image scale: std {h_est.std()*1000:.2f} mm, "
          f"range {h_est.min()*100:.2f}..{h_est.max()*100:.2f} cm")
    hk = h_est if a.use_alt else np.full(nF, h)

    print("rigid motion fits ...")
    import pickle
    ec = CACHE / f"edges_{a.height:.4f}_{int(a.use_alt)}_{a.alt_smooth}.pkl"
    if a.reuse and ec.exists():
        edges = pickle.loads(ec.read_bytes())
    else:
        edges = build_edges(pairs, R, hk)
        edges += keyframe_edges(T, K, dist, R, hk, None, "bridge", 0, 0)
        edges += gap_edges(edges, nF, K, dist, R, hk)
        ec.write_bytes(pickle.dumps(edges))
    print(f"  {len(edges)} edges ({time.time()-t0:.0f}s)")
    covered = np.zeros(nF, bool)
    for e in edges:
        covered[e["k"] + 1:e["j"] + 1] = True
    print(f"  frames without any motion constraint: {(~covered[1:]).sum()}")

    # 1) pure dead reckoning (frame-to-frame chain only), for comparison
    odo = chain_init([e for e in edges if e["j"] - e["k"] == 1 or e["kind"] == "bridge"], nF)
    # 2) multi-span graph without loop closure
    init = chain_init(edges, nF)
    vo, _ = optimise(edges, nF, init)
    poses, res = vo, None
    loops = []
    if not a.no_loop:
        print("searching loop closures ...")
        loops = keyframe_edges(T, K, dist, R, hk, vo, "loop", min_gap=90, max_dist=0.35)
        print(f"  {len(loops)} loop-closure edges")
        if loops:
            # loop gap before closing: how far apart the VO solution puts revisited places
            gaps = []
            for e in loops:
                pred = apply(vo[e["k"], 2], vo[e["k"], :2], e["vk"][0][None])[0]
                act = apply(vo[e["j"], 2], vo[e["j"], :2], e["v"][0][None])[0]
                gaps.append(np.linalg.norm(pred - act))
            print(f"  loop mismatch of VO solution: median {np.median(gaps)*1000:.1f} mm, max {np.max(gaps)*1000:.1f} mm")
    all_edges = edges + loops
    poses, res = optimise(all_edges, nF, vo)
    # second round: drop gross outlier edges (> 5 mm disagreement) and re-solve
    bad = edge_err(poses, all_edges) > 5.0
    if bad.any():
        print(f"  removing {bad.sum()} outlier edges")
        loops = [e for e in loops if edge_err(poses, [e])[0] <= 5.0]
        all_edges = [e for e, b in zip(all_edges, bad) if not b]
        poses, res = optimise(all_edges, nF, poses)

    # frames with no motion constraint at all (e.g. completely blurred) -> interpolate
    seen = np.zeros(nF, bool)
    for e in all_edges:
        seen[e["k"]] = seen[e["j"]] = True
    seen[0] = True
    interp = ~seen
    if interp.any():
        idx = np.arange(nF)
        for c in range(3):
            src = np.unwrap(poses[seen, 2]) if c == 2 else poses[seen, c]
            poses[interp, c] = np.interp(idx[interp], idx[seen], src)
        print(f"  {interp.sum()} unconstrained frames interpolated: {idx[interp]}")

    err = edge_err(poses, all_edges)
    loop_err = edge_err(poses, loops) if loops else np.array([])

    # ---- write results (convert image axes x right / y down -> X right / Y up)
    t = T["times"] - T["times"][0]
    X, Y = poses[:, 0], -poses[:, 1]
    head = -np.degrees(np.unwrap(poses[:, 2]))
    # speed from a 0.25 s Savitzky-Golay fit: the raw 60 Hz positions carry ~0.6 mm
    # jitter from cart vibration, which would dominate a plain finite difference
    from scipy.signal import savgol_filter
    dt = np.median(np.diff(t))
    vx = savgol_filter(X, 15, 2, deriv=1, delta=dt)
    vy = savgol_filter(Y, 15, 2, deriv=1, delta=dt)
    speed = np.hypot(vx, vy)
    dist_trav = np.concatenate([[0], np.cumsum(np.hypot(np.diff(X), np.diff(Y)))])
    nedge = np.bincount(np.concatenate([[e["k"] for e in all_edges], [e["j"] for e in all_edges]]), minlength=nF)
    Xo, Yo = odo[:, 0], -odo[:, 1]
    Xv, Yv = vo[:, 0], -vo[:, 1]
    hdr = "frame,time_s,x_m,y_m,heading_deg,speed_m_s,distance_m,altitude_from_image_scale_m,n_constraints,interpolated,x_vo_m,y_vo_m,x_deadreckoning_m,y_deadreckoning_m"
    data = np.column_stack([np.arange(nF), t, X, Y, head, speed, dist_trav, h_est, nedge, interp, Xv, Yv, Xo, Yo])
    np.savetxt(OUT / "trajectory.csv", data, delimiter=",", header=hdr, comments="",
               fmt=["%d", "%.4f", "%.5f", "%.5f", "%.3f", "%.4f", "%.5f", "%.5f", "%d", "%d", "%.5f", "%.5f", "%.5f", "%.5f"])
    summary = dict(
        height_m=h, tilt_roll_deg=float(np.degrees(tilt[0])), tilt_pitch_deg=float(np.degrees(tilt[1])),
        tilt_sd_deg=np.degrees(tilt_sd).tolist(), rigidity_rms_mm_no_tilt=rig0, rigidity_rms_mm_tilt=rig1,
        altitude_used="per-frame estimate" if a.use_alt else "constant",
        altitude_est_std_mm=float(h_est.std() * 1000), altitude_est_min_m=float(h_est.min()), altitude_est_max_m=float(h_est.max()),
        n_frames=nF, duration_s=float(t[-1]), n_edges=len(all_edges), n_loop_edges=len(loops),
        edge_err_median_mm=float(np.median(err)), edge_err_p95_mm=float(np.percentile(err, 95)),
        loop_err_after_mm=float(np.median(loop_err)) if len(loop_err) else None,
        loop_err_before_mm=float(np.median(gaps)) * 1000 if loops else None,
        path_length_m=float(dist_trav[-1]),
        path_length_smoothed_m=float(np.sum(np.hypot(np.diff(savgol_filter(X, 15, 2)), np.diff(savgol_filter(Y, 15, 2))))),
        jitter_rms_mm=float(np.sqrt(np.mean((X - savgol_filter(X, 7, 2)) ** 2 + (Y - savgol_filter(Y, 7, 2)) ** 2)) * 1000), end_position_m=[float(X[-1]), float(Y[-1])],
        end_minus_start_deadreckoning_vs_final_mm=float(np.hypot(Xo[-1] - X[-1], Yo[-1] - Y[-1]) * 1000),
        gsd_mm_per_px=float(h / K[0, 0] * 1000),
    )
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2))
    np.savez(CACHE / "solution.npz", poses=poses, odo=odo, vo=vo, hk=hk, h_est=h_est, tilt=tilt, times=t,
             loops=np.array([[e["k"], e["j"]] for e in loops]).reshape(-1, 2))
    print(json.dumps(summary, indent=2))
    print(f"done in {time.time()-t0:.0f}s -> {OUT/'trajectory.csv'}")


if __name__ == "__main__":
    main()

"""Closed-loop GNSS-denied position filter with camera-to-map fixes over real multi-date maps.

SIMULATED flight over REAL orthophotos: the camera sees the newest date (r_map_benchmark
pinhole generator), the onboard map is the oldest date. Odometry is synthetic and
ALTO-like (constants copied from q_wufeng_closed_loop.py). GNSS 1 Hz until the cut.
Truth is used only by the generator (path, odometry, camera images, synthetic IMU
attitude) and by the scorer; `Estimator` never receives it (asserted on its API).

Decision variants (all except accept_all and dr_only also use a chi2 99 % gate):
  union              accept ZNCC quad>=3 or ZNCC-family x XFeat agreement within 10 m;
                     all passing estimates must agree
  union_driftgate    union plus a distance-dependent drift variance in the innovation gate
  union_adaptive     union_driftgate plus a 3-sigma search window, clipped to 384..1024 px
  xfeat              XFeat affine with geometric gates only
  zncc_quad          ZNCC quad>=3 only (cheapest)
  accept_all         first finite ZNCC output, no gate (ablation)
  dr_only            odometry only

  .venv/bin/python experiments/r_closed_loop.py --sweep default --workers 8
"""
from __future__ import annotations

import argparse
import inspect
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

DT = 1.0
SPEED = 15.0
DURATION = 420.0
GNSS_CUT = 20.0
GNSS_SIGMA = 3.0
SCALE_SIGMA, HEAD0_SIGMA, HEAD_RW, VEL_NOISE = .04, 1.0, .05, .3   # as q_wufeng_closed_loop.py
Q_PER_M, P_FLOOR = .10, .05 ** 2
R_FIX = 8.0
CHI2_99 = 9.21
WRONG = 25.0
KAPPA = .06          # declared odometry drift bound: fraction of distance since the last fix
MAX_WINDOW = 1024    # px; adaptive search window cap
VARIANTS = ("union", "union_driftgate", "union_adaptive", "xfeat", "zncc_quad", "accept_all", "dr_only")


# ------------------------------------------------------------------ generator side

def make_path(joint, rng, steps):
    """GENERATOR. Smooth random walk at SPEED inside the eroded jointly-valid area."""
    margin = int(B.QUERY * .8)
    inner = cv2.erode(joint, np.ones((2 * margin + 1, 2 * margin + 1), np.uint8))
    border = B.SEARCH // 2 + B.PRIOR_ERROR
    inner[:border], inner[-border:], inner[:, :border], inner[:, -border:] = 0, 0, 0, 0
    ys, xs = np.nonzero(inner)
    if len(xs) == 0:
        raise ValueError("no valid interior")
    i = rng.integers(len(xs))
    p, heading = np.array([xs[i], ys[i]], float), rng.uniform(0, 2 * np.pi)
    path = [p.copy()]
    def ok(q):
        x, y = int(round(q[0])), int(round(q[1]))
        return 0 <= y < inner.shape[0] and 0 <= x < inner.shape[1] and inner[y, x]
    for _ in range(steps - 1):
        heading += rng.normal(0, np.radians(3))
        for k in range(24):
            look = p + 60 * np.array([np.cos(heading), np.sin(heading)])
            if ok(look) and ok(p + SPEED * np.array([np.cos(heading), np.sin(heading)])):
                break
            heading += np.radians(15)
        else:
            raise ValueError("trapped")
        p = p + SPEED * np.array([np.cos(heading), np.sin(heading)])
        path.append(p.copy())
    return np.array(path)


def make_odometry(path, rng, mode="transit"):
    """GENERATOR. ALTO-like odometry: scale bias, heading-bias random walk, velocity noise.

    physical: errors act on the true increments, so they partly cancel when the path folds back
              on a small map (optimistic for a transit flight).
    transit:  same error magnitudes, but accumulated along a fixed random direction per run, as if
              the flight were straight (drift grows with distance travelled; conservative)."""
    n = len(path)
    scale = rng.normal(0, SCALE_SIGMA)
    bias = rng.normal(0, HEAD0_SIGMA) + np.concatenate([[0], np.cumsum(rng.normal(0, HEAD_RW, n - 1))])
    inc = np.diff(path, axis=0)
    noise = rng.normal(0, VEL_NOISE * DT, inc.shape)
    if mode == "physical":
        c, s = np.cos(np.radians(bias[:-1])), np.sin(np.radians(bias[:-1]))
        rot = np.stack([c * inc[:, 0] - s * inc[:, 1], s * inc[:, 0] + c * inc[:, 1]], axis=1)
        return (1 + scale) * rot + noise, bias
    angle = rng.uniform(0, 2 * np.pi)
    along = np.array([np.cos(angle), np.sin(angle)])
    perp = np.array([-along[1], along[0]])
    dist = np.linalg.norm(inc, axis=1)[:, None]
    b = np.radians(bias[:-1])[:, None]
    error = dist * (scale * along + np.sin(b) * perp) + dist * (np.cos(b) - 1) * along
    return inc + error + noise, bias


def make_shot(grey, valid, centre, cname, heading_error_deg, rng):
    """GENERATOR. Camera image at truth; residual yaw = estimator heading error + condition yaw."""
    cond = dict(B.CONDITIONS[cname])
    cond["yaw"] = cond.get("yaw", 0.) + heading_error_deg
    return B.render(grey, valid, centre, cond, rng)


# ------------------------------------------------------------------ estimator side

class Estimator:
    """Position-only KF. Inputs: initial GNSS, GNSS before the cut, odometry, camera images,
    IMU nav offset for tilted cameras, the onboard map. No truth."""

    def __init__(self, gnss0, map_grey, map_valid, variant):
        self.x = np.asarray(gnss0, float)
        self.P = np.eye(2) * GNSS_SIGMA ** 2
        self.map, self.valid, self.variant = map_grey, map_valid, variant
        self.adaptive = variant.endswith("_adaptive")
        self.drift_gate = self.adaptive or variant.endswith("_driftgate")
        self.dist = 0.   # odometry distance since the last accepted fix / GNSS update
        self.log = []

    def predict(self, increment):
        self.x = self.x + increment
        self.P = self.P + ((Q_PER_M * np.linalg.norm(increment)) ** 2 + P_FLOOR) * np.eye(2)
        self.dist += float(np.linalg.norm(increment))

    def update(self, z, sigma):
        S = self.P + np.eye(2) * sigma ** 2
        K = self.P @ np.linalg.inv(S)
        self.x, self.P = self.x + K @ (z - self.x), (np.eye(2) - K) @ self.P
        self.dist = 0.

    def gnss(self, z):
        self.update(np.asarray(z, float), GNSS_SIGMA)

    def drift_var(self):
        """Adaptive variants: bias-type drift bound from the odometry spec (KAPPA x distance)."""
        return (KAPPA * self.dist) ** 2 if self.drift_gate else 0.

    def camera(self, image, nav_offset, t):
        if self.variant == "dr_only":
            return
        prior = np.rint(self.x)
        if self.adaptive:   # window grows with uncertainty (only *_adaptive)
            sigma = np.sqrt(np.linalg.eigvalsh(self.P).max() + self.drift_var())
            size = int(np.clip(2 * np.ceil(3 * sigma) + B.QUERY, B.SEARCH, MAX_WINDOW)) // 2 * 2
        else:
            size = B.SEARCH
        size = min(size, min(self.map.shape) // 2 * 2)
        shape = np.array(self.map.shape[::-1])
        if np.any(prior + size / 2 < 0) or np.any(prior - size / 2 > shape):
            self.log.append(dict(t=t, status="outside_map"))
            return
        # Near the map edge the window is shifted inside the map (any implementation would do this).
        origin = np.clip(prior - size / 2, 0, shape - size).astype(int)
        ref = self.map[origin[1]:origin[1] + size, origin[0]:origin[0] + size]
        val = self.valid[origin[1]:origin[1] + size, origin[0]:origin[0] + size]
        start = time.perf_counter()
        z = self.decide(image, ref, val)
        latency = (time.perf_counter() - start) * 1000
        if z is None:
            self.log.append(dict(t=t, status="rejected", latency_ms=latency, window=size))
            return
        z = z + origin - nav_offset
        if self.variant == "accept_all":
            self.x = z.copy()
            self.log.append(dict(t=t, status="accepted", z=z, latency_ms=latency, window=size))
            return
        v = z - self.x
        d2 = float(v @ np.linalg.inv(self.P + np.eye(2) * (R_FIX ** 2 + self.drift_var())) @ v)
        if d2 > CHI2_99:
            self.log.append(dict(t=t, status="gated", z=z, latency_ms=latency, window=size))
            return
        if self.drift_gate:
            self.P = self.P + np.eye(2) * self.drift_var()   # admit the unmodelled bias before updating
        self.update(z, R_FIX)
        self.log.append(dict(t=t, status="accepted", z=z, latency_ms=latency, window=size))

    def decide(self, image, ref, val):
        v = self.variant
        if v == "accept_all":
            p, _ = B.run_method("zncc", image, ref, val)
            return p if np.all(np.isfinite(p)) else None
        if v == "zncc_quad":
            p, s = B.run_method("zncc", image, ref, val)
            return p if np.all(np.isfinite(p)) and s.get("quad_n", 0) >= 3 else None
        if v == "xfeat":
            p, _ = B.run_method("xfeat_affine", image, ref, val)
            return p if np.all(np.isfinite(p)) else None
        # union: cheapest checks first, stop at the first consistent acceptance set
        z0, s0 = B.run_method("zncc", image, ref, val)
        z1, s1 = B.run_method("zncc_yaw_scale", image, ref, val)
        fa, _ = B.run_method("xfeat_affine", image, ref, val)
        cands = []
        for z, s in ((z0, s0), (z1, s1)):
            if np.all(np.isfinite(z)) and s.get("quad_n", 0) >= 3:
                cands.append(z)
            if np.all(np.isfinite(z)) and np.all(np.isfinite(fa)) and np.linalg.norm(z - fa) <= 10:
                cands.append(fa)
        if not cands:
            return None
        c = np.array(cands)
        if np.ptp(c[:, 0]) > 10 or np.ptp(c[:, 1]) > 10:
            return None
        return c[0]


for _name, _f in inspect.getmembers(Estimator, inspect.isfunction):
    assert not any("truth" in p for p in inspect.signature(_f).parameters), _name


# ------------------------------------------------------------------ one run

def run(task):
    cv2.setNumThreads(1)
    site = task["site"]
    map_grey, map_valid = B.load(site["files"][0])
    cam_grey, cam_valid = B.load(site["files"][-1])
    joint = np.ones_like(map_valid)
    for f in site["files"]:
        joint &= B.load(f)[1]
    rng = np.random.default_rng(task["seed"])
    steps = int(DURATION / DT) + 1
    path = make_path(joint, rng, steps)
    odom, bias = make_odometry(path, rng, task["odometry"])
    gnss = path + rng.normal(0, GNSS_SIGMA, path.shape)
    period, blackout, cname = task["period"], task["blackout"], task["condition"]
    b0 = GNSS_CUT + 60
    attempts = [t for t in np.arange(GNSS_CUT + period, DURATION + 1e-9, period)
                if not (blackout and b0 <= t < b0 + blackout)]
    shots = {}
    for t in attempts:
        i = int(round(t / DT))
        shots[i] = make_shot(cam_grey, cam_valid, path[i], cname, float(bias[i]),
                             np.random.default_rng(B.seed_of(task["seed"], t, cname)))
    rows, series = [], []
    for variant in task["variants"]:
        est = Estimator(gnss[0], map_grey, map_valid, variant)
        err = np.zeros(steps)
        sigma = np.zeros(steps)
        for i in range(1, steps):
            est.predict(odom[i - 1])
            if i * DT < GNSS_CUT:
                est.gnss(gnss[i])
            if i in shots and shots[i].valid:
                est.camera(shots[i].image, shots[i].meta["nav_offset"], i * DT)
            err[i] = np.linalg.norm(est.x - path[i])
            sigma[i] = np.sqrt(np.trace(est.P) / 2)
        acc = [e for e in est.log if e["status"] == "accepted"]
        wrong = sum(np.linalg.norm(e["z"] - path[int(round(e["t"] / DT))]) > WRONG for e in acc)
        after = np.arange(steps) * DT >= GNSS_CUT
        tail = np.arange(steps) * DT >= DURATION - 60
        rows.append(dict(site=site["site"], land_cover=site["land_cover"], seed=task["seed"], condition=cname,
                         period_s=period, blackout_s=blackout, variant=variant,
                         path_m=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                         attempts=len(attempts), accepted=len(acc), wrong_accepted=int(wrong),
                         gated=sum(e["status"] == "gated" for e in est.log),
                         outside_map=sum(e["status"] == "outside_map" for e in est.log),
                         final_err_m=float(err[-1]), median_err_m=float(np.median(err[after])),
                         p95_err_m=float(np.quantile(err[after], .95)), max_err_m=float(err[after].max()),
                         last60_median_m=float(np.median(err[tail])),
                         nees_ok=float(np.mean(err[after] <= 3 * sigma[after])),
                         latency_p50_ms=float(np.median([e.get("latency_ms", np.nan) for e in est.log])) if est.log else np.nan))
        if task.get("series"):
            series.extend(dict(site=site["site"], seed=task["seed"], condition=cname, period_s=period,
                               blackout_s=blackout, variant=variant, t=i * DT, err=float(err[i]), sigma=float(sigma[i]))
                          for i in range(0, steps, 2))
    return rows, series


SWEEPS = {
    # one-factor-at-a-time around the default (aligned, 10 s, 60 s blackout)
    "default": [("aligned", 10, 60), ("legacy_degraded", 10, 60), ("tilt10_rect", 10, 60), ("motion9", 10, 60),
                ("scale1.25", 10, 60), ("patch_shadow", 10, 60),
                ("aligned", 5, 60), ("aligned", 20, 60), ("aligned", 40, 60),
                ("aligned", 10, 0), ("aligned", 10, 120), ("aligned", 10, 240)],
    "smoke": [("aligned", 10, 60)],
}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pairs", type=Path, default=B.ROOT / "data/raw/aerial_pairs")
    p.add_argument("--output", type=Path, default=B.ROOT / "data/processed/r_closed_loop")
    p.add_argument("--sweep", default="default", choices=sorted(SWEEPS))
    p.add_argument("--seeds", type=int, default=4, help="runs per site and sweep point")
    p.add_argument("--sites", default="")
    p.add_argument("--variants", default=",".join(VARIANTS))
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--odometry", default="transit", choices=("transit", "physical"))
    a = p.parse_args()
    sites = B.discover_sites(a.pairs)
    if a.sites:
        sites = [s for s in sites if s["site"] in set(a.sites.split(","))]
    tasks = []
    for site in sites:
        for (cname, period, blackout) in SWEEPS[a.sweep]:
            for k in range(a.seeds):
                tasks.append(dict(site=site, condition=cname, period=period, blackout=blackout,
                                  seed=B.seed_of("closed", site["site"], k), variants=a.variants.split(","),
                                  odometry=a.odometry, series=(cname, period, blackout) == ("aligned", 10, 60)))
    print(f"{len(sites)} sites, {len(tasks)} runs x {len(a.variants.split(','))} variants", flush=True)
    rows, series = [], []
    start = time.time()
    with get_context("spawn").Pool(a.workers) as pool:
        for i, (r, s) in enumerate(pool.imap_unordered(run, tasks)):
            rows.extend(r)
            series.extend(s)
            if (i + 1) % 20 == 0:
                print(f"{i+1}/{len(tasks)} {time.time()-start:.0f} s", flush=True)
    a.output.mkdir(parents=True, exist_ok=True)
    df = pd.DataFrame(rows)
    df.to_csv(a.output / "runs.csv", index=False)
    pd.DataFrame(series).to_csv(a.output / "series.csv.gz", index=False)
    g = df.groupby(["condition", "period_s", "blackout_s", "variant"])
    summary = g.agg(runs=("seed", "size"), median_err=("median_err_m", "median"), p95_err=("p95_err_m", "median"),
                    max_err_p90=("max_err_m", lambda x: x.quantile(.9)), final_err=("final_err_m", "median"),
                    accepted=("accepted", "sum"), attempts=("attempts", "sum"), wrong=("wrong_accepted", "sum"),
                    nees_ok=("nees_ok", "median")).reset_index()
    summary.to_csv(a.output / "summary.csv", index=False)
    (a.output / "run.json").write_text(json.dumps(dict(
        protocol=__doc__.strip().split("\n\n")[1], sweep=SWEEPS[a.sweep], seeds=a.seeds,
        sites=[s["site"] for s in sites], constants=dict(DT=DT, SPEED=SPEED, DURATION=DURATION, GNSS_CUT=GNSS_CUT,
        GNSS_SIGMA=GNSS_SIGMA, SCALE_SIGMA=SCALE_SIGMA, HEAD0_SIGMA=HEAD0_SIGMA, HEAD_RW=HEAD_RW,
        VEL_NOISE=VEL_NOISE, Q_PER_M=Q_PER_M, R_FIX=R_FIX, CHI2_99=CHI2_99, blackout_start_after_cut=60),
        seconds=time.time() - start), indent=2) + "\n")
    pd.set_option("display.width", 220)
    print(summary.round(1).to_string(index=False))


if __name__ == "__main__":
    main()

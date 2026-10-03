"""Step 5 - independent check of the metric scale with the tape measure.

The whole position solution scales with  height / focal length.  The tape
measure lying on the floor has millimetre graduations, i.e. a known ground
truth length.  For every frame that shows the tape we
  1. project the image onto the floor plane at 0.1 mm/px with the SAME model
     (calibration + tilt + 64.1 cm height) that the navigation uses,
  2. find the tape (bright, unsaturated strip) and rotate it upright,
  3. measure the period of the graduation pattern along the tape with a
     Fourier transform (refined peak -> sub-0.1 % precision).
If the model is right the period is exactly 1.000 mm. The ratio
true/measured is the scale error; height * ratio is the camera height implied
by the tape.
"""
from __future__ import annotations

import json

import cv2
import matplotlib
import numpy as np
from scipy.optimize import minimize_scalar

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common import CACHE, FLIGHT_VIDEO, OUT, iter_frames, load_calib
from navigate import rot_tilt
from render import ground_homography

RES = 1e-4  # 0.1 mm per pixel


def rectify_frame(frame, K, dist, R, h, m1, m2):
    W, H = frame.shape[1], frame.shape[0]
    und = cv2.remap(frame, m1, m2, cv2.INTER_CUBIC)
    G = ground_homography(K, R, h)
    c = G @ np.array([[0, 0, 1], [W, 0, 1], [W, H, 1], [0, H, 1]], float).T
    c = (c[:2] / c[2]).T
    lo, hi = c.min(0), c.max(0)
    M = np.array([[1 / RES, 0, -lo[0] / RES], [0, 1 / RES, -lo[1] / RES], [0, 0, 1]])
    size = (int((hi[0] - lo[0]) / RES), int((hi[1] - lo[1]) / RES))
    valid = cv2.warpPerspective(np.full((H, W), 255, np.uint8), M @ G, size, flags=cv2.INTER_NEAREST)
    return cv2.warpPerspective(und, M @ G, size, flags=cv2.INTER_CUBIC), valid


def tape_period(img, valid):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    white = ((hsv[..., 2] > 215) & (hsv[..., 1] < 60) & (valid > 0)).astype(np.uint8)
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((41, 41), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(white)
    if n < 2:
        return None
    i = 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])
    ys, xs = np.nonzero(lab == i)
    pts = np.stack([xs, ys], 1).astype(float)
    c = pts.mean(0)
    ev, evec = np.linalg.eigh(np.cov((pts - c).T))
    # uniform strip: full extent = sqrt(12) * sigma
    length, width = np.sqrt(12 * ev[1]), np.sqrt(12 * ev[0])
    if length * RES < 0.12 or not (0.010 < width * RES < 0.025):
        return None  # need >= 12 cm of tape, tape is ~17 mm wide
    ax = evec[:, 1]
    ang = np.degrees(np.arctan2(ax[1], ax[0]))
    Rm = cv2.getRotationMatrix2D(tuple(c), ang - 90, 1.0)
    rot = cv2.warpAffine(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), Rm, (img.shape[1], img.shape[0]), flags=cv2.INTER_CUBIC)
    vrot = cv2.warpAffine(valid, Rm, (img.shape[1], img.shape[0]), flags=cv2.INTER_NEAREST)
    # tape is now vertical through c. Find its two edges from the column profile.
    y0, y1 = max(0, int(c[1] - length * 0.48)), min(img.shape[0], int(c[1] + length * 0.48))
    xa, xb = int(c[0] - width), int(c[0] + width)
    reg = rot[y0:y1, xa:xb].astype(float)
    ok = vrot[y0:y1, xa:xb].min(1) > 0
    reg = reg[ok]
    if len(reg) < 1000:
        return None
    col = np.median(reg, 0)
    inside = col > (np.percentile(col, 90) + np.percentile(col, 10)) / 2
    idx = np.flatnonzero(inside)
    el, er = idx[0], idx[-1]
    out = {}
    for side, (a0, a1) in {"left": (el + 5, el + 25), "right": (er - 25, er - 5)}.items():
        prof = reg[:, a0:a1].mean(1)
        prof = prof - np.convolve(prof, np.ones(61) / 61, mode="same")
        prof = prof[40:-40] * np.hanning(len(prof) - 80)
        out[side] = (refined_period(prof), prof)
    return out, length * RES, (er - el) * RES


def refined_period(prof):
    """Strongest periodicity between 0.7 and 4 mm, refined to sub-0.01 % (mm, snr)."""
    x = np.arange(len(prof))

    def power(f):
        return -np.abs(np.sum(prof * np.exp(-2j * np.pi * f * x))) ** 2

    fs = np.linspace(1 / 40, 1 / 7, 3000)
    p = np.array([-power(f) for f in fs])
    f0 = fs[p.argmax()]
    df = fs[1] - fs[0]
    r = minimize_scalar(power, bounds=(f0 - df, f0 + df), method="bounded", options=dict(xatol=1e-12))
    return 1.0 / r.x * RES * 1000, p.max() / np.median(p)


def main():
    K, dist, cal = load_calib()
    S = np.load(CACHE / "solution.npz")
    summ = json.loads((OUT / "summary.json").read_text())
    h = summ["height_m"]
    R = rot_tilt(*S["tilt"])
    W, H = cal["image_size"]
    m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, K, (W, H), cv2.CV_32FC1)
    rows = []
    example = None
    for k, _, frame in iter_frames(FLIGHT_VIDEO, step=15):
        img, valid = rectify_frame(frame, K, dist, R, h, m1, m2)
        r = tape_period(img, valid)
        if r is None:
            continue
        o, length, width = r
        # the edge with ~1 mm ticks is the metric (cm/mm) edge; the other edge is
        # the Taiwanese 台尺 scale (1 分 = 3.0303 mm, half-分 ticks = 1.515 mm)
        per = {s_: o[s_][0] for s_ in o}
        mm_side = min(per, key=lambda s_: abs(per[s_][0] - 1.0))
        tw_side = "left" if mm_side == "right" else "right"
        p_mm, snr_mm = per[mm_side]
        p_tw, snr_tw = per[tw_side]
        if snr_mm < 50 or not 0.9 < p_mm < 1.15:
            continue
        rows.append((k, p_mm, snr_mm, p_tw, snr_tw, length))
        if example is None or snr_mm > example[1]:
            example = (k, snr_mm, o[mm_side][1])
        print(f"  frame {k:5d}: tape {length*100:4.1f} cm, mm-edge period {p_mm:.5f} mm (SNR {snr_mm:.0f}), "
              f"台尺-edge period {p_tw:.5f} mm")
    rows = np.array(rows)

    def robust_mean(v):
        med = np.median(v)
        mad = 1.4826 * np.median(np.abs(v - med))
        v = v[np.abs(v - med) < 5 * mad + 1e-9]
        return v.mean(), v.std(ddof=1) / np.sqrt(len(v)), len(v)

    pm, pm_se, n_mm = robust_mean(rows[:, 1])
    tw_ok = rows[(rows[:, 4] > 50) & (np.abs(rows[:, 3] / 1.51515 - pm) < 0.05)]
    pt, pt_se, n_tw = robust_mean(tw_ok[:, 3]) if len(tw_ok) > 3 else (np.nan, np.nan, 0)
    scale = 1.0 / pm  # true / model
    res = dict(n_frames_mm_edge=int(n_mm), mm_edge_period_mm=float(pm), mm_edge_period_se=float(pm_se),
               taiwan_edge_period_mm=float(pt), taiwan_edge_expected_mm=1.51515, n_frames_taiwan_edge=int(n_tw),
               scale_true_over_model=float(scale), scale_uncertainty=float(pm_se / pm),
               scale_from_taiwan_edge=float(1.51515 / pt) if n_tw else None,
               assumed_height_m=h, implied_height_m=float(h * scale),
               note="positions scale linearly with the height: x_true = x_model * scale_true_over_model")
    (OUT / "scale_check.json").write_text(json.dumps(res, indent=2))
    # tape-corrected trajectory: every length scales with the height
    hdr = (OUT / "trajectory.csv").read_text().splitlines()[0]
    d = np.loadtxt(OUT / "trajectory.csv", delimiter=",", skiprows=1)
    cols = hdr.split(",")
    for i, c in enumerate(cols):
        if c.endswith("_m") and c != "altitude_from_image_scale_m" or c == "speed_m_s":
            d[:, i] *= scale
    ai = cols.index("altitude_from_image_scale_m")
    d[:, ai] *= scale
    fmt = ["%.5f"] * len(cols)
    fmt[0] = fmt[cols.index("n_constraints")] = fmt[cols.index("interpolated")] = "%d"
    np.savetxt(OUT / "trajectory_tape_scaled.csv", d, delimiter=",", header=hdr, comments="", fmt=fmt)
    print(json.dumps(res, indent=2))

    fig, ax = plt.subplots(1, 2, figsize=(14, 4.2))
    ax[0].plot(rows[:, 0] / 60.0, rows[:, 1], "o", ms=4, label="mm edge (should be 1.000 mm)")
    if n_tw:
        ax[0].plot(tw_ok[:, 0] / 60.0, tw_ok[:, 3] / 1.51515, "s", ms=4, label="Taiwan-foot edge / 1.515 mm (should be 1.000)")
    ax[0].axhline(1.0, color="k", ls="--", label="ground truth")
    ax[0].axhline(pm, color="r", label=f"measured {pm:.4f}")
    ax[0].set_xlabel("time [s]"); ax[0].set_ylabel("measured / true length"); ax[0].legend(fontsize=8)
    ax[0].set_title(f"Tape-measure scale check (h = {h*100:.1f} cm assumed) -> implied h = {h*scale*100:.2f} cm")
    k, _, prof = example
    xs = np.arange(len(prof)) * RES * 1000
    mid = xs[len(xs) // 2]
    ax[1].plot(xs - mid, prof, lw=0.7)
    ax[1].set_xlim(-12.5, 12.5); ax[1].set_xlabel("position along tape in model metres [mm]")
    ax[1].set_title(f"mm-tick profile along the tape edge, frame {k}")
    fig.tight_layout(); fig.savefig(OUT / "scale_check.png", dpi=110)


if __name__ == "__main__":
    main()

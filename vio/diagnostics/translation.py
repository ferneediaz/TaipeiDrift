"""Monocular translation-direction diagnostics and the inertial-scale velocity test.

Monocular limitation: the essential matrix gives the translation DIRECTION only, t_vis ~ lambda t.
Metric scale is never taken from ground truth: the only scale source tested is the estimator's own
inertial speed, v_vis = ||v_est|| * t_hat_vis (a fused diagnostic, not independent vision).

Sign convention: cv2.recoverPose resolves the sign of t by cheirality (triangulated points in
front of both cameras), so the raw direction is used. The sign-agnostic error
min(e, 180 - e) is reported alongside to show how much of the error is a sign flip.
"""
from __future__ import annotations

import numpy as np


def direction_error_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Angle between direction vectors, row-wise (degrees). Zero-length rows give NaN."""
    a, b = np.atleast_2d(a).astype(float), np.atleast_2d(b).astype(float)
    na, nb = np.linalg.norm(a, axis=1), np.linalg.norm(b, axis=1)
    ok = (na > 1e-12) & (nb > 1e-12)
    out = np.full(len(a), np.nan)
    c = np.sum(a[ok] * b[ok], axis=1) / (na[ok] * nb[ok])
    out[ok] = np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))
    return out


def sign_agnostic(err_deg: np.ndarray) -> np.ndarray:
    return np.minimum(err_deg, 180.0 - err_deg)


def inertial_scaled_velocity(t_dir_world: np.ndarray, speed_est: np.ndarray) -> np.ndarray:
    """v_vis = ||v_est|| * t_hat. Scale from the ESTIMATOR'S speed only (never ground truth)."""
    d = np.atleast_2d(t_dir_world).astype(float)
    d = d / np.linalg.norm(d, axis=1, keepdims=True)
    return d * np.asarray(speed_est, float)[:, None]


def causal_slope(t: np.ndarray, y: np.ndarray, window: int = 5) -> np.ndarray:
    """Causal derivative: least-squares slope over the last ``window`` samples (NaN until enough).

    Works on (N,) or (N, k). A moving linear fit is the simplest smoother-differentiator: it
    averages noise over the window at the cost of a lag of about half the window.
    """
    t = np.asarray(t, float)
    Y = np.asarray(y, float)
    Y2 = Y[:, None] if Y.ndim == 1 else Y
    out = np.full(Y2.shape, np.nan)
    for k in range(window - 1, len(t)):
        tt = t[k - window + 1:k + 1]
        yy = Y2[k - window + 1:k + 1]
        tc = tt - tt.mean()
        den = float(tc @ tc)
        if den > 0 and np.all(np.isfinite(yy)):
            out[k] = tc @ (yy - yy.mean(0)) / den
    return out[:, 0] if Y.ndim == 1 else out

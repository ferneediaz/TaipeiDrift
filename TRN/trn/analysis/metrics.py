"""Navigation performance metrics for one run of one filter."""
from __future__ import annotations

import numpy as np


def _first_hold(below: np.ndarray, hold_n: int) -> int:
    """First index from which ``below`` stays True for ``hold_n`` samples AND until the end... (hold only)."""
    run = 0
    for i, b in enumerate(below):
        run = run + 1 if b else 0
        if run >= hold_n:
            return i - hold_n + 1
    return -1


def run_metrics(t: np.ndarray, err: np.ndarray, cov: np.ndarray, mc: dict) -> dict:
    """err (n,2) horizontal estimate-minus-truth, cov (n,2,2). Returns scalar metrics."""
    eh = np.hypot(err[:, 0], err[:, 1])
    dt = float(np.median(np.diff(t)))
    post = t >= mc["burn_in_s"]
    out = dict(rmse_all=float(np.sqrt(np.mean(eh ** 2))),
               rmse_post=float(np.sqrt(np.mean(eh[post] ** 2))),
               cep50=float(np.median(eh[post])), cep95=float(np.percentile(eh[post], 95)),
               final_err=float(eh[-1]), max_err_post=float(eh[post].max()))
    k = _first_hold(eh < mc["converge_threshold_m"], max(1, int(mc["converge_hold_s"] / dt)))
    out["conv_time"] = float(t[k]) if k >= 0 else np.nan
    covr = cov + np.eye(2)[None] * 1e-6
    inv = np.linalg.inv(covr)
    nees = np.einsum("ni,nij,nj->n", err, inv, err)
    out["nees_mean"] = float(np.mean(nees[post]))
    out["nees_median"] = float(np.median(nees[post]))
    out["sigma_mean"] = float(np.mean(np.sqrt(np.trace(cov[post], axis1=1, axis2=2))))
    ff = (np.sqrt(nees) > mc["false_fix_sigma"]) & post
    out["false_fix"] = bool(_first_hold(ff, max(1, int(mc["false_fix_duration_s"] / dt))) >= 0)
    out["diverged"] = bool(eh[-1] > mc["divergence_error_m"])
    return out

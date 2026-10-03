"""Reference-checked visual-rotation error: one interval table for any dataset, frozen analysis.

Residual (body frame, right perturbation; same for every dataset):
    dR_vis_body = R_bc C_cam R_bc^T          camera relative rotation in the body frame
    dR_ref      = R_i^T R_j                  reference body rotation (GT or IMU-derived, per dataset)
    e_body      = Log(dR_ref^T dR_vis_body)  visual error, body axes at j
    e_cam       = R_bc^T e_body              same error in camera axes
    e_world     = R_j e_body                 same error in world axes
    rate        = e / dt                     (deg/s on export)

Dataset-specific frame handling (gyro conventions, quaternion order, extrinsics, timing) stays in
the adapters; this module only sees body->world reference rotations, positions and the camera
measurement.

Frozen analysis rules (set on Mid-Air, applied unchanged to NTU):
- frame stability per sequence: mean vector m, covariance C; signal-to-dispersion
  ||m|| / sqrt(trace C); alignment ||m|| / mean(||e||); sign stability per component
  max(frac+, frac-). Most stable frame = highest median signal-to-dispersion over sequences.
- rotation magnitude: quartile bins (Q1-Q4) of the dataset's own GT rotation magnitude, plus
  the Mid-Air absolute edges for comparability.
- dominant axis: largest |component| of the reference body rotation vector.
- speed / translation / vision quality: Spearman and Pearson with the error-rate norm, plus
  quartile bins.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import pearsonr, spearmanr

FRAMES = ("cam", "body", "world")
SCHEMA = (["dataset", "condition", "sequence", "interval_start_s", "interval_end_s", "dt"]
          + [f"e_vis_{a}_deg" for a in "xyz"] + ["e_vis_norm_deg"]
          + [f"e_vis_rate_{a}_deg_s" for a in "xyz"] + ["e_vis_rate_norm_deg_s"]
          + [f"e_{f}_rate_{a}_deg_s" for f in FRAMES for a in "xyz"]
          + ["gt_rotation_magnitude_deg"] + [f"gt_rotation_axis_{a}" for a in "xyz"] + ["dominant_axis"]
          + ["mean_speed_m_s"] + [f"translation_{a}" for a in "xyz"] + ["translation_magnitude_m"]
          + [f"translation_dir_{a}" for a in "xyz"]
          + ["track_count", "ransac_inlier_count", "inlier_ratio", "median_flow_px", "track_spread_px"])


@dataclass
class VisualInterval:
    """One camera measurement between two timestamps (dataset adapter output)."""

    t_i: float
    t_j: float
    C_cam: np.ndarray  # R_ab in camera axes: orientation of camera at j expressed in camera at i
    track_count: int
    inliers: int
    inlier_ratio: float
    median_flow_px: float
    track_spread_px: float
    t_dir_cam: np.ndarray | None = None  # unit translation direction (camera i axes, cheirality-resolved sign); no scale
    parallax_px: float = float("nan")  # median flow left after removing the estimated rotation (translation signal)


def interval_row(dataset: str, condition: str, sequence: str, iv: VisualInterval, R_i: Rotation, R_j: Rotation,
                 p_i: np.ndarray, p_j: np.ndarray, R_bc: np.ndarray, eps: float = 1e-3) -> dict:
    """Build one table row. R_i, R_j: reference body->world attitudes; p: positions (world, m)."""
    dt = iv.t_j - iv.t_i
    vis_body = Rotation.from_matrix(R_bc @ iv.C_cam @ R_bc.T)
    ref = R_i.inv() * R_j
    e_body = (ref.inv() * vis_body).as_rotvec()
    e = {"body": e_body, "cam": R_bc.T @ e_body, "world": R_j.apply(e_body)}
    phi = ref.as_rotvec()
    mag = np.linalg.norm(phi)
    tr_body = R_i.inv().apply(np.asarray(p_j) - np.asarray(p_i))  # translation in body axes at i
    tmag = float(np.linalg.norm(tr_body))
    row = {"dataset": dataset, "condition": condition, "sequence": sequence,
           "interval_start_s": iv.t_i, "interval_end_s": iv.t_j, "dt": dt}
    deg = np.degrees(e_body)
    row.update({f"e_vis_{a}_deg": float(v) for a, v in zip("xyz", deg)})
    row["e_vis_norm_deg"] = float(np.linalg.norm(deg))
    row.update({f"e_vis_rate_{a}_deg_s": float(v / dt) for a, v in zip("xyz", deg)})
    row["e_vis_rate_norm_deg_s"] = float(np.linalg.norm(deg) / dt)
    for f in FRAMES:
        row.update({f"e_{f}_rate_{a}_deg_s": float(v) for a, v in zip("xyz", np.degrees(e[f]) / dt)})
    row["gt_rotation_magnitude_deg"] = float(np.degrees(mag))
    axis = phi / mag if mag > 1e-9 else np.zeros(3)
    row.update({f"gt_rotation_axis_{a}": float(v) for a, v in zip("xyz", axis)})
    row["dominant_axis"] = "xyz"[int(np.argmax(np.abs(phi)))] if mag > 1e-9 else "none"
    row["mean_speed_m_s"] = tmag / dt
    row.update({f"translation_{a}": float(v) for a, v in zip("xyz", tr_body)})
    row["translation_magnitude_m"] = tmag
    d = tr_body / tmag if tmag > eps else np.full(3, np.nan)
    row.update({f"translation_dir_{a}": float(v) for a, v in zip("xyz", d)})
    row.update({"track_count": iv.track_count, "ransac_inlier_count": iv.inliers, "inlier_ratio": iv.inlier_ratio,
                "median_flow_px": iv.median_flow_px, "track_spread_px": iv.track_spread_px})
    return row


def time_intervals(times: np.ndarray, interval_s: float) -> list[tuple[int, int]]:
    """Non-overlapping (i, j) index pairs, each spanning the sample closest to ``interval_s`` after i."""
    out, i = [], 0
    while i < len(times) - 1:
        j = int(np.argmin(np.abs(times - (times[i] + interval_s))))
        if j <= i or times[j] - times[i] < 0.75 * interval_s:  # no sample close enough: end of data
            break
        out.append((i, j))
        i = j
    return out


def sign_stability(x: np.ndarray) -> float:
    x = np.asarray(x)
    x = x[np.isfinite(x) & (x != 0)]
    return float(max((x > 0).mean(), (x < 0).mean())) if x.size else float("nan")


# ------------------------------------------------------------------ frozen analysis
def frame_stats(rates: np.ndarray) -> dict:
    """Systematic-component statistics of an (N, 3) error-rate set in one frame (deg/s)."""
    m = rates.mean(0)
    C = np.cov(rates.T) if len(rates) > 1 else np.zeros((3, 3))
    disp = float(np.sqrt(np.trace(C)))
    mean_norm = float(np.linalg.norm(rates, axis=1).mean())
    return {"mean": m.tolist(), "norm_mean": float(np.linalg.norm(m)), "mean_norm": mean_norm, "dispersion": disp,
            "signal_to_dispersion": float(np.linalg.norm(m) / disp) if disp > 0 else float("nan"),
            "alignment": float(np.linalg.norm(m) / mean_norm) if mean_norm > 0 else float("nan"),
            "sign_stability": [sign_stability(rates[:, k]) for k in range(3)]}


def sequence_summary(rows: list[dict]) -> list[dict]:
    out = []
    for seq in sorted({(r["condition"], r["sequence"]) for r in rows}):
        sel = [r for r in rows if (r["condition"], r["sequence"]) == seq]
        s = {"condition": seq[0], "sequence": seq[1], "n_intervals": len(sel),
             "median_rate_norm_deg_s": float(np.median([r["e_vis_rate_norm_deg_s"] for r in sel]))}
        for f in FRAMES:
            s[f] = frame_stats(np.array([[r[f"e_{f}_rate_{a}_deg_s"] for a in "xyz"] for r in sel]))
        out.append(s)
    return out


def _bins(rows, key, edges):
    out = []
    v = np.array([r[key] for r in rows])
    e = np.array([r["e_vis_rate_norm_deg_s"] for r in rows])
    for k in range(len(edges) - 1):
        m = (v >= edges[k]) & (v < edges[k + 1]) if k < len(edges) - 2 else (v >= edges[k]) & (v <= edges[k + 1])
        if m.any():
            out.append({"range": [float(edges[k]), float(edges[k + 1])], "n": int(m.sum()), "mean": float(e[m].mean()),
                        "median": float(np.median(e[m])), "p90": float(np.percentile(e[m], 90))})
        else:
            out.append({"range": [float(edges[k]), float(edges[k + 1])], "n": 0})
    return out


def quartile_edges(rows, key):
    v = np.array([r[key] for r in rows], dtype=float)
    v = v[np.isfinite(v)]
    return np.percentile(v, [0, 25, 50, 75, 100]).tolist()


def correlations(rows, key):
    x = np.array([r[key] for r in rows], dtype=float)
    y = np.array([r["e_vis_rate_norm_deg_s"] for r in rows], dtype=float)
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 5 or np.std(x[m]) == 0:
        return {"n": int(m.sum())}
    return {"n": int(m.sum()), "spearman": float(spearmanr(x[m], y[m]).statistic), "pearson": float(pearsonr(x[m], y[m]).statistic)}


def dataset_summary(rows: list[dict], rotation_edges_reference: list[float] | None = None) -> dict:
    """All frozen analyses for one dataset. ``rotation_edges_reference``: Mid-Air's absolute edges."""
    e = np.array([r["e_vis_rate_norm_deg_s"] for r in rows])
    seqs = sequence_summary(rows)
    stab = {f: float(np.nanmedian([s[f]["signal_to_dispersion"] for s in seqs])) for f in FRAMES}
    align = {f: float(np.nanmedian([s[f]["alignment"] for s in seqs])) for f in FRAMES}
    signs = {f: float(np.nanmedian([max(s[f]["sign_stability"]) for s in seqs])) for f in FRAMES}
    rot_q = quartile_edges(rows, "gt_rotation_magnitude_deg")
    out = {
        "n_sequences": len(seqs), "n_intervals": len(rows),
        "rate_norm_deg_s": {"median": float(np.median(e)), "mean": float(e.mean()), "p90": float(np.percentile(e, 90))},
        "frame_signal_to_dispersion_median": stab, "frame_alignment_median": align,
        "frame_best_component_sign_stability_median": signs,
        "most_stable_frame": max(stab, key=lambda f: stab[f]),
        "rotation_quartile_edges_deg": rot_q,
        "by_rotation_quartile": _bins(rows, "gt_rotation_magnitude_deg", rot_q),
        "by_rotation_reference_edges": _bins(rows, "gt_rotation_magnitude_deg", rotation_edges_reference) if rotation_edges_reference else None,
        "by_dominant_axis": {},
        "correlations": {k: correlations(rows, k) for k in ("gt_rotation_magnitude_deg", "mean_speed_m_s", "translation_magnitude_m",
                                                             "track_count", "ransac_inlier_count", "inlier_ratio", "median_flow_px", "track_spread_px")},
        "by_speed_quartile": _bins(rows, "mean_speed_m_s", quartile_edges(rows, "mean_speed_m_s")),
        "by_inlier_ratio_quartile": _bins(rows, "inlier_ratio", quartile_edges(rows, "inlier_ratio")),
        "by_track_count_quartile": _bins(rows, "track_count", quartile_edges(rows, "track_count")),
    }
    for ax in "xyz":
        sel = [r for r in rows if r["dominant_axis"] == ax]
        if sel:
            v = np.array([r["e_vis_rate_norm_deg_s"] for r in sel])
            comp = np.array([[r[f"e_body_rate_{a}_deg_s"] for a in "xyz"] for r in sel])
            out["by_dominant_axis"][ax] = {"n": len(sel), "median_rate_norm": float(np.median(v)), "p90": float(np.percentile(v, 90)),
                                           "median_abs_body_component_deg_s": np.median(np.abs(comp), axis=0).tolist()}
    # does the error direction change with rotation magnitude? mean body error vector per quartile
    out["mean_body_error_by_rotation_quartile"] = []
    for k in range(4):
        sel = [r for r in rows if rot_q[k] <= r["gt_rotation_magnitude_deg"] <= rot_q[k + 1]]
        out["mean_body_error_by_rotation_quartile"].append(
            np.mean([[r[f"e_body_rate_{a}_deg_s"] for a in "xyz"] for r in sel], axis=0).tolist() if sel else None)
    # translation direction: correlation of each body error component with each translation direction component
    out["translation_direction_vs_error_spearman"] = {
        f"e{a}_vs_t{b}": _sp(rows, f"e_body_rate_{a}_deg_s", f"translation_dir_{b}") for a in "xyz" for b in "xyz"}
    return out


def _sp(rows, ka, kb):
    x = np.array([r[ka] for r in rows], dtype=float)
    y = np.array([r[kb] for r in rows], dtype=float)
    m = np.isfinite(x) & np.isfinite(y)
    return float(spearmanr(x[m], y[m]).statistic) if m.sum() > 5 else float("nan")

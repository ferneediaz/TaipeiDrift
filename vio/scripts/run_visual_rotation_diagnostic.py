"""Visual-rotation error diagnostic: Mid-Air (discovery) then NTU VIRAL (transfer, rules frozen).

    python vio/scripts/run_visual_rotation_diagnostic.py midair-tracks --flights sunny:0 cloudy:3000   # cache tracks (parallelisable)
    python vio/scripts/run_visual_rotation_diagnostic.py midair
    python vio/scripts/run_visual_rotation_diagnostic.py ntu --sequences eee_03 sbs_01 rtp_01
    python vio/scripts/run_visual_rotation_diagnostic.py compare

Ground truth (Mid-Air) and the VN100 orientation (NTU) are used only as references for this
diagnostic. Outputs: outputs/visual_rotation_diagnostic/{midair,ntu,cross_dataset}/.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml

VIO_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "baseline"))
sys.path.insert(0, str(REPO_ROOT))

from vio.diagnostics import sources  # noqa: E402
from vio.diagnostics.visual_rotation import SCHEMA, dataset_summary, sequence_summary  # noqa: E402

OUT = REPO_ROOT / "outputs" / "visual_rotation_diagnostic"
MIDAIR_FLIGHTS = ["sunny:0", "sunny:1", "sunny:3", "sunny:6", "sunny:9",
                  "cloudy:3000", "cloudy:3001", "cloudy:3004", "cloudy:3007", "cloudy:3012",
                  "foggy:2002", "foggy:2005", "foggy:2008", "foggy:2011", "foggy:2014",
                  "sunset:1004", "sunset:1007", "sunset:1010", "sunset:1013", "sunset:1015"]
RESTORED = REPO_ROOT / "data" / "MidAir_3_trajecotry"  # images of 0000, 0001, 3000, 3001
# rtp_01's left camera is saturated in the bag (every pixel 255), so it uses the right camera.
NTU_CAMERA = {"eee_03": "left", "sbs_01": "left", "rtp_01": "right"}


def vio_cfg():
    return yaml.safe_load((VIO_DIR / "configs" / "midair_vio.yaml").read_text())


def load_midair(spec: str):
    from src.data.midair import MidAirConfig, load_midair_trajectory
    from vio.benchmark.discovery import resolve_sensor_file
    cond, tid = spec.split(":")
    name = f"trajectory_{int(tid):04d}"
    base = yaml.safe_load((REPO_ROOT / "baseline/configs/midair_baseline.yaml").read_text())["midair"]
    restored = RESTORED / "Kite_training" / cond
    if (restored / "color_left" / name / "frames.zip").is_file() and (restored / "sensor_records.hdf5").is_file():
        root, frames = RESTORED, restored
    else:
        cdir = REPO_ROOT / "data" / "MidAir" / "Kite_training" / cond
        sensor, _, problem = resolve_sensor_file(cdir, OUT.parent / "midair_benchmark" / "cache" / "sensor_records", "Kite_training", cond)
        if sensor is None:
            raise FileNotFoundError(problem)
        root, frames = sensor.parent.parent.parent, cdir
    traj = load_midair_trajectory(MidAirConfig.from_dict({**base, "condition": cond, "trajectory": name, "data_root": None}), data_root=str(root))
    return traj, cond, frames


def write_csv(path: Path, rows: list[dict], fields: list[str]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in r.items()})


def flat_sequence(s: dict) -> dict:
    out = {"condition": s["condition"], "sequence": s["sequence"], "n_intervals": s["n_intervals"],
           "median_rate_norm_deg_s": s["median_rate_norm_deg_s"]}
    for f in ("cam", "body", "world"):
        st = s[f]
        out.update({f"{f}_mean_{a}": v for a, v in zip("xyz", st["mean"])})
        out.update({f"{f}_{k}": st[k] for k in ("norm_mean", "mean_norm", "dispersion", "signal_to_dispersion", "alignment")})
        out.update({f"{f}_sign_stability_{a}": v for a, v in zip("xyz", st["sign_stability"])})
    return out


def run_dataset(name: str, rows: list[dict], extra: dict, ref_edges=None):
    d = OUT / name
    write_csv(d / "intervals.csv", rows, SCHEMA)
    seqs = sequence_summary(rows)
    flat = [flat_sequence(s) for s in seqs]
    write_csv(d / "sequence_summary.csv", flat, list(flat[0]))
    summ = dataset_summary(rows, ref_edges)
    summ.update(extra)
    (d / "dataset_summary.json").write_text(json.dumps(summ, indent=2, default=float))
    plots(rows, seqs, d, name)
    return summ


def plots(rows, seqs, d: Path, name: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    e = np.array([r["e_vis_rate_norm_deg_s"] for r in rows])
    for key, lab, fn in (("gt_rotation_magnitude_deg", "Reference rotation over the interval (deg)", "error_vs_rotation.png"),
                         ("mean_speed_m_s", "Mean speed (m/s)", "error_vs_speed.png")):
        fig, ax = plt.subplots(figsize=(6.5, 4.5))
        ax.scatter([r[key] for r in rows], e, s=5, alpha=0.4, color="#2a78d6", linewidths=0)
        ax.set_yscale("log"), ax.set_xlabel(lab), ax.set_ylabel("Visual rotation error rate (deg/s)")
        ax.set_title(f"{name}: error vs {lab.split(' (')[0].lower()}"), ax.grid(color="#e4e3df")
        fig.tight_layout(), fig.savefig(d / fn, dpi=110), plt.close(fig)
    fig, ax = plt.subplots(figsize=(6, 4.5))
    groups = [[r["e_vis_rate_norm_deg_s"] for r in rows if r["dominant_axis"] == a] for a in "xyz"]
    ax.boxplot([g if g else [np.nan] for g in groups], whis=(5, 95), showfliers=False)
    ax.set_xticks([1, 2, 3], [f"dominant {a} (n={len(g)})" for a, g in zip("xyz", groups)])
    ax.set_yscale("log"), ax.set_ylabel("Visual rotation error rate (deg/s)"), ax.set_title(f"{name}: error by dominant body rotation axis")
    ax.grid(color="#e4e3df"), fig.tight_layout(), fig.savefig(d / "error_by_axis.png", dpi=110), plt.close(fig)
    fig, ax = plt.subplots(figsize=(6, 4.5))
    for k, f in enumerate(("cam", "body", "world")):
        v = [s[f]["signal_to_dispersion"] for s in seqs]
        ax.scatter(np.full(len(v), k) + np.random.default_rng(0).uniform(-0.1, 0.1, len(v)), v, s=18, color="#2a78d6")
    ax.set_xticks([0, 1, 2], ["camera", "body", "world"]), ax.set_ylabel("|mean| / sqrt(trace cov), per sequence")
    ax.set_title(f"{name}: which frame holds a systematic component?"), ax.grid(color="#e4e3df")
    fig.tight_layout(), fig.savefig(d / "frame_stability.png", dpi=110), plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    for ax, (a, b) in zip(axes, (("x", "y"), ("x", "z"), ("y", "z"))):
        for s in seqs:
            m = s["body"]["mean"]
            ax.scatter(m["xyz".index(a)], m["xyz".index(b)], s=22, color="#2a78d6")
        ax.axhline(0, color="#52514e", lw=0.8), ax.axvline(0, color="#52514e", lw=0.8)
        ax.set_xlabel(f"mean e_{a} (deg/s, body)"), ax.set_ylabel(f"mean e_{b} (deg/s, body)"), ax.grid(color="#e4e3df")
    fig.suptitle(f"{name}: per-sequence mean visual error (body frame)"), fig.tight_layout()
    fig.savefig(d / "sequence_bias_vectors.png", dpi=110), plt.close(fig)


def cmd_midair_tracks(flights):
    cfg = vio_cfg()
    for spec in flights:
        t = time.time()
        traj, cond, frames = load_midair(spec)
        sources.midair_tracks(traj, frames, cfg, OUT / "cache" / "midair" / f"{cond}_{traj.metadata['trajectory']}.pkl")
        print(f"{spec}: tracks in {time.time() - t:.0f} s", flush=True)


def cmd_midair(flights):
    cfg = vio_cfg()
    rows = []
    for spec in flights:
        traj, cond, frames = load_midair(spec)
        tracks, age = sources.midair_tracks(traj, frames, cfg, OUT / "cache" / "midair" / f"{cond}_{traj.metadata['trajectory']}.pkl")
        rows += sources.midair_rows(traj, cond, tracks, age, cfg)
    summ = run_dataset("midair", rows, {"flights": flights, "interval_frames": sources.frames_for(25.0),
                                        "reference": "ground-truth attitude"})
    (OUT / "midair" / "frozen_rules.json").write_text(json.dumps({"rotation_edges_deg": summ["rotation_quartile_edges_deg"]}))
    print(json.dumps({k: summ[k] for k in ("n_sequences", "n_intervals", "rate_norm_deg_s", "most_stable_frame",
                                            "frame_signal_to_dispersion_median", "frame_alignment_median")}, indent=1))


def cmd_ntu(seqs, max_seconds, reference="orientation"):
    cfg = vio_cfg()
    frozen = json.loads((OUT / "midair" / "frozen_rules.json").read_text())
    rows, info = [], {}
    for seq in seqs:
        sdir = OUT / "cache" / "ntu" / seq
        bag = sdir / f"{seq}.bag"
        if not bag.is_file():
            print(f"{seq}: bag not extracted yet, skipped")
            continue
        t = time.time()
        cam = NTU_CAMERA.get(seq, "left")
        d = sources.ntu_read_bag(bag, OUT / "cache" / "ntu" / f"{seq}_{cam}_messages.npz", f"/{cam}/image_raw")
        cal = sources.ntu_calibration(sdir, cam)
        val = sources.ntu_validate_orientation(d)
        r, meta = sources.ntu_rows(seq, d, cal, cfg, max_seconds, reference)
        meta["camera"] = cam
        rows += r
        info[seq] = {**meta, "orientation_check_median_deg_per_0.5s": val, "n_intervals": len(r),
                     "duration_s": float(d["img_t"][-1] - d["img_t"][0])}
        print(f"{seq}: {len(r)} intervals, orientation check {val}, {time.time() - t:.0f} s", flush=True)
    # Primary: the VN100 orientation output (bias-compensated). The raw /imu/imu gyro carries a constant
    # body-frame bias of ~(0.40, 0.21, 0.04-0.14) deg/s in all three sequences, so gyro integration is
    # kept only as a secondary, labelled reference.
    name = "ntu" if reference == "orientation" else "ntu_gyro_reference"
    ref = ("VN100 orientation output, bias-compensated by the VN100 filter (no GT attitude in NTU VIRAL)" if reference == "orientation"
           else "raw body-frame gyro integration (SECONDARY: contains the uncompensated ~0.47 deg/s gyro bias)")
    summ = run_dataset(name, rows, {"sequences": info, "reference": ref}, frozen["rotation_edges_deg"])
    print(json.dumps({k: summ[k] for k in ("n_sequences", "n_intervals", "rate_norm_deg_s", "most_stable_frame")}, indent=1))


def cmd_compare():
    m = json.loads((OUT / "midair" / "dataset_summary.json").read_text())
    n = json.loads((OUT / "ntu" / "dataset_summary.json").read_text())

    def trend(s, key):  # Spearman with the error-rate norm
        c = s["correlations"].get(key, {})
        return c.get("spearman")

    def axis(s):
        return {a: v["median_rate_norm"] for a, v in s["by_dominant_axis"].items()}

    rows = []
    for label, f in [("n_sequences", lambda s: s["n_sequences"]), ("n_intervals", lambda s: s["n_intervals"]),
                     ("median_error_floor_deg_s", lambda s: s["rate_norm_deg_s"]["median"]),
                     ("p90_error_deg_s", lambda s: s["rate_norm_deg_s"]["p90"]),
                     ("best_component_sign_stability_median", lambda s: max(s["frame_best_component_sign_stability_median"].values())),
                     ("most_stable_frame", lambda s: s["most_stable_frame"]),
                     ("signal_to_dispersion_cam", lambda s: s["frame_signal_to_dispersion_median"]["cam"]),
                     ("signal_to_dispersion_body", lambda s: s["frame_signal_to_dispersion_median"]["body"]),
                     ("signal_to_dispersion_world", lambda s: s["frame_signal_to_dispersion_median"]["world"]),
                     ("alignment_body", lambda s: s["frame_alignment_median"]["body"]),
                     ("spearman_rotation_magnitude", lambda s: trend(s, "gt_rotation_magnitude_deg")),
                     ("spearman_speed", lambda s: trend(s, "mean_speed_m_s")),
                     ("spearman_translation_magnitude", lambda s: trend(s, "translation_magnitude_m")),
                     ("spearman_track_count", lambda s: trend(s, "track_count")),
                     ("spearman_inlier_ratio", lambda s: trend(s, "inlier_ratio")),
                     ("spearman_median_flow", lambda s: trend(s, "median_flow_px")),
                     ("median_error_by_dominant_axis", axis)]:
        rows.append({"metric": label, "midair": f(m), "ntu": f(n)})
    d = OUT / "cross_dataset"
    d.mkdir(parents=True, exist_ok=True)
    (d / "comparison.json").write_text(json.dumps(rows, indent=2, default=str))
    write_csv(d / "comparison.csv", [{k: json.dumps(v) if isinstance(v, dict) else v for k, v in r.items()} for r in rows], ["metric", "midair", "ntu"])
    for r in rows:
        print(f"{r['metric']:40s} {str(r['midair'])[:60]:60s} {str(r['ntu'])[:60]}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["midair-tracks", "midair", "ntu", "compare"])
    ap.add_argument("--flights", nargs="+", default=MIDAIR_FLIGHTS)
    ap.add_argument("--sequences", nargs="+", default=["eee_03", "sbs_01", "rtp_01"])
    ap.add_argument("--max-seconds", type=float, default=None, help="NTU: limit each sequence (time budget)")
    ap.add_argument("--reference", choices=["orientation", "gyro"], default="orientation", help="NTU rotation reference")
    a = ap.parse_args()
    if a.cmd == "midair-tracks":
        cmd_midair_tracks(a.flights)
    elif a.cmd == "midair":
        cmd_midair(a.flights)
    elif a.cmd == "ntu":
        cmd_ntu(a.sequences, a.max_seconds, a.reference)
    else:
        cmd_compare()
    return 0


if __name__ == "__main__":
    sys.exit(main())

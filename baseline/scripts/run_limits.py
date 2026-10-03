"""The limits: how the navigator copes when the camera picture gets worse (the brief's "noise rises").

On the ALTO validation flight, every camera frame after GNSS is lost is made worse in one of three
ways (src/data/degrade.py): less light, blur, haze, each at five levels. The frames before the jam
stay clean, so what the navigator learned with GNSS does not change. For each level the navigator
runs with map fixes every 300 m, and we record the error, the fixes accepted and refused, wrong
fixes used, and the status it reports.

From the repository root:

    python baseline/scripts/run_limits.py

Results go to outputs/limits/: limits.csv, limits.png (the chart) and examples.png (what the
camera sees at each level).
"""
from __future__ import annotations

import csv
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

BASELINE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = BASELINE_DIR.parent
sys.path.insert(0, str(BASELINE_DIR))

from src.data.alto import AltoConfig, load_alto_flight  # noqa: E402
from src.data.degrade import degrade  # noqa: E402
from src.estimation.camera_navigator import NavigatorConfig, calibrate, jam_index, navigate  # noqa: E402
from src.estimation.image_motion import shifts_for_flight  # noqa: E402
from src.evaluation.navigation_metrics import summarize_navigation  # noqa: E402

FRAME_METRES_PER_PIXEL = 0.85 * 0.60  # an ALTO frame shows 0.85 of a 500-pixel reference image at 0.6 m per pixel
LEVELS = {
    "light": [1 / 4, 1 / 16, 1 / 64, 1 / 256, 1 / 1024],  # share of the recorded light
    "blur": [1.0, 2.0, 4.0, 8.0, 16.0],  # metres on the ground
    "haze": [0.5, 0.25, 0.1, 0.05, 0.02],  # share of the contrast left
}
LABELS = {
    "light": ("Less light", lambda v: f"1/{round(1 / v)}"),
    "blur": ("Blur", lambda v: f"{v:g} m"),
    "haze": ("Haze", lambda v: f"{100 * v:g}%"),
}
OUT = REPO_ROOT / "outputs" / "limits"


def _setup():
    cfg = yaml.safe_load((BASELINE_DIR / "configs" / "alto_navigator.yaml").read_text())
    flight = load_alto_flight(AltoConfig(data_root=str(REPO_ROOT / "data/raw/alto"), ground_map=True, map_cache_dir=str(REPO_ROOT / "data/processed")))
    run = replace(NavigatorConfig(**cfg["navigator"]), **cfg["runs"]["map_every_300"])
    return flight, run


def degraded(flight, kind: str, level: float):
    jam = jam_index(flight, 300.0)
    original = flight.load_frame
    return replace(flight, load_frame=lambda i: degrade(original(i), kind, level, i, FRAME_METRES_PER_PIXEL) if i > jam else original(i))


def one_level(job: tuple[str, float]) -> dict:
    kind, level = job
    flight, run = _setup()
    clean_shifts = shifts_for_flight(flight, REPO_ROOT / "data/processed/alto_val_flow.npy")
    calibration = calibrate(flight, clean_shifts, run)
    t0 = time.time()
    if kind == "clean":
        worse, shifts = flight, clean_shifts
    else:
        worse = degraded(flight, kind, level)
        shifts = shifts_for_flight(worse, REPO_ROOT / "data/processed" / f"alto_val_flow_{kind}_{level:g}.npy")
    result = navigate(worse, shifts, calibration, run)
    s = summarize_navigation(result, flight)
    status = np.array(result.status)
    return {
        "kind": kind, "level": level, "median": s.median, "p90": s.p90, "worst": s.worst, "end": s.end,
        "fixes_used": s.fixes_used, "fixes_refused": s.fixes_rejected, "used_but_wrong": s.used_but_wrong,
        "within_3_sigma": s.within_3_sigma, "tracking": float(np.mean(status == "TRACKING")),
        "degraded": float(np.mean(status == "DEGRADED")), "lost": float(np.mean(status == "LOST")),
        "seconds": round(time.time() - t0, 1),
    }


def chart(rows: list[dict], path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    clean = next(r for r in rows if r["kind"] == "clean")
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.5), sharey="row")
    for col, kind in enumerate(LEVELS):
        name, fmt = LABELS[kind]
        mine = [clean] + sorted((r for r in rows if r["kind"] == kind), key=lambda r: -r["level"] if kind != "blur" else r["level"])
        ticks = ["as recorded"] + [fmt(r["level"]) for r in mine[1:]]
        x = np.arange(len(mine))
        ax = axes[0, col]
        ax.plot(x, [r["median"] for r in mine], "o-", color="#eb6834", label="median error")
        ax.plot(x, [r["p90"] for r in mine], "s--", color="#8a5cd6", label="90% of the time below")
        ax.axhline(472.4, color="#888", lw=1, ls=":", label="camera alone, no fixes (472 m)")
        ax.set_yscale("log")
        ax.set_title(name)
        ax.set_xticks(x, ticks)
        if col == 0:
            ax.set_ylabel("position error (m)")
            ax.legend(fontsize=8)
        ax = axes[1, col]
        used = np.array([r["fixes_used"] for r in mine])
        refused = np.array([r["fixes_refused"] for r in mine])
        wrong = np.array([r["used_but_wrong"] for r in mine])
        ax.bar(x, used - wrong, color="#1baf7a", label="fixes used, right")
        ax.bar(x, wrong, bottom=used - wrong, color="#d64a4a", label="fixes used, wrong (> 50 m)")
        ax.bar(x, refused, bottom=used, color="#cccccc", label="fixes refused")
        ax.set_xticks(x, ticks)
        if col == 0:
            ax.set_ylabel("fix attempts")
            ax.legend(fontsize=8)
    fig.suptitle("ALTO flight, GNSS lost after 300 m, fixes every 300 m: the camera picture made worse after the jam")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def examples(path: Path) -> None:
    import cv2

    flight, _ = _setup()
    k = jam_index(flight, 300.0) + 600
    frame = flight.frame(k)
    rows = []
    for kind, levels in LEVELS.items():
        tiles = [frame] + [degrade(frame, kind, v, k, FRAME_METRES_PER_PIXEL) for v in levels]
        rows.append(np.hstack([cv2.resize(t, (200, 200), interpolation=cv2.INTER_AREA) for t in tiles]))
    cv2.imwrite(str(path), np.vstack(rows))


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = [("clean", 1.0)] + [(kind, v) for kind, levels in LEVELS.items() for v in levels]
    t0 = time.time()
    rows = []
    with ProcessPoolExecutor(max_workers=8) as pool:
        for r in pool.map(one_level, jobs):
            rows.append(r)
            print(f"{r['kind']:5s} {r['level']:8.4g}: median {r['median']:6.1f} m, 90% below {r['p90']:6.1f}, "
                  f"fixes used {r['fixes_used']:2d} (wrong {r['used_but_wrong']}), refused {r['fixes_refused']:2d}, "
                  f"within 3 sigma {r['within_3_sigma']:.0%}, lost {r['lost']:.0%} ({r['seconds']:.0f} s)", flush=True)
    with open(OUT / "limits.csv", "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    chart(rows, OUT / "limits.png")
    examples(OUT / "examples.png")
    print(f"results in {OUT} ({time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

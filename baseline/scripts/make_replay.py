"""Make the demo videos: a navigator run replayed on the map.

From the repository root:

    python baseline/scripts/make_replay.py alto                 # ALTO validation flight, real camera motion
    python baseline/scripts/make_replay.py visloc --flight 04   # UAV-VisLoc flight 04, never tuned on
    python baseline/scripts/make_replay.py alto --frames 300    # a short test clip

Videos and still images for the slides go to outputs/replay/.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

BASELINE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = BASELINE_DIR.parent
sys.path.insert(0, str(BASELINE_DIR))
sys.path.insert(0, str(BASELINE_DIR / "scripts"))

from src.estimation.camera_navigator import NavigatorConfig, calibrate, navigate  # noqa: E402
from src.visualization.replay import make_replay  # noqa: E402

STILLS = (0.25, 0.5, 0.75, 1.0)


def alto(args):
    from src.data.alto import AltoConfig, load_alto_flight
    from src.estimation.image_motion import shifts_for_flight

    cfg = yaml.safe_load((BASELINE_DIR / "configs" / "alto_navigator.yaml").read_text())
    flight = load_alto_flight(AltoConfig(data_root=str(REPO_ROOT / "data/raw/alto"), ground_map=True, map_cache_dir=str(REPO_ROOT / "data/processed")))
    shifts = shifts_for_flight(flight, REPO_ROOT / "data/processed/alto_val_flow.npy")
    run = replace(NavigatorConfig(**cfg["navigator"]), **cfg["runs"]["map_every_300"])
    result = navigate(flight, shifts, calibrate(flight, shifts, run), run)
    title = "ALTO, USA: real helicopter flight. GNSS lost after 300 m; camera motion and map fixes every 300 m"
    return flight, result, title, "alto_val"


def visloc(args):
    from run_visloc_navigator import heading_reading

    from src.data.uav_visloc import VisLocConfig, load_visloc_flight, with_heading
    from src.sensors.dead_reckoning import simulated_steps

    cfg = yaml.safe_load((BASELINE_DIR / "configs" / "visloc_navigator.yaml").read_text())
    flight0 = load_visloc_flight(VisLocConfig(data_root=str(REPO_ROOT / cfg["data_root"]), flight=args.flight, cache_dir=str(REPO_ROOT / cfg["cache_dir"])))
    settings = dict(cfg["runs"][args.run])
    sensor = settings.pop("heading")
    run = replace(NavigatorConfig(**cfg["navigator"]), **settings)
    rng = np.random.default_rng(args.seed)
    reading = heading_reading(cfg, flight0, sensor, rng)
    flight = with_heading(flight0, reading)
    steps = simulated_steps(flight.position_gt, reading - flight0.metadata["true_heading_deg"], rng, **cfg["dead_reckoning"])
    result = navigate(flight, steps, calibrate(flight, steps, run), run)
    title = (f"UAV-VisLoc flight {args.flight}, China: never seen before. Real photos (2018) against a satellite map (2021-23); "
             f"GNSS lost after 1 km")
    return flight, result, title, f"visloc_{args.flight}_{args.run}_seed{args.seed}"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("source", choices=["alto", "visloc"])
    p.add_argument("--flight", default="04")
    p.add_argument("--run", default="confirm_body")
    p.add_argument("--seed", type=int, default=2, help="seed 2 is the median of the three held-out seeds on flight 04")
    p.add_argument("--every", type=int, default=None, help="draw every n-th frame (default: 3 for ALTO, 1 for UAV-VisLoc)")
    p.add_argument("--fps", type=int, default=None)
    p.add_argument("--frames", type=int, default=None, help="stop after this many frames of the run (for a test clip)")
    p.add_argument("--output-dir", type=Path, default=REPO_ROOT / "outputs" / "replay")
    args = p.parse_args()

    flight, result, title, name = (alto if args.source == "alto" else visloc)(args)
    if args.frames:
        result = replace(result, position=result.position[: args.frames], sigma=result.sigma[: args.frames], status=result.status[: args.frames])
        name += "_short"
    every = args.every or (3 if args.source == "alto" else 1)
    fps = args.fps or (20 if args.source == "alto" else 12)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    video = args.output_dir / f"{name}.mp4"
    stills = make_replay(flight, result, video, title, every=every, fps=fps, window_m=1500.0 if args.source == "alto" else 2500.0, stills=STILLS)
    print(f"video {video}")
    for s in stills:
        print(f"still {s}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

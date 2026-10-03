"""Run the camera navigator on the simulated Wufeng flight: camera motion and map fixes, GNSS lost after 450 m.

From the repository root, after a flight was recorded (sim/README.md):

    python baseline/scripts/run_sim_navigator.py
    python baseline/scripts/run_sim_navigator.py --only map_2018 --seeds 1

Results go to outputs/sim_navigator/: results.csv (one row per run and seed), metrics.json (medians
over the seeds), navigator.png and stanford.png.
"""
from __future__ import annotations

import argparse
import csv
import json
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

from src.data.sim_replay import SimReplayConfig, load_sim_flight, with_heading  # noqa: E402
from src.estimation.camera_navigator import NavigatorConfig, calibrate, navigate  # noqa: E402
from src.estimation.image_motion import shifts_for_flight  # noqa: E402
from src.evaluation.navigation_metrics import integrity_summary, navigation_errors, summarize_navigation  # noqa: E402
from src.sensors.heading import compass_heading  # noqa: E402

_FLIGHTS: dict[str, object] = {}  # one loaded flight per worker process and map


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=BASELINE_DIR / "configs" / "sim_navigator.yaml")
    p.add_argument("--seeds", nargs="+", type=int)
    p.add_argument("--only", nargs="+", help="names of the runs to do")
    p.add_argument("--output-dir", type=Path)
    p.add_argument("--workers", type=int, default=6)
    return p.parse_args()


def _path(cfg: dict, key: str) -> str:
    p = Path(cfg[key])
    return str(p if p.is_absolute() else REPO_ROOT / p)


def flight_for(cfg: dict, map_name: str):
    """The recorded flight with the chosen map, with the true heading (loaded once per process)."""
    if map_name not in _FLIGHTS:
        tif = Path(cfg["maps"][map_name])
        _FLIGHTS[map_name] = load_sim_flight(SimReplayConfig(
            recording=_path(cfg, "recording"), map_tif=str(tif if tif.is_absolute() else REPO_ROOT / tif),
            route=_path(cfg, "route"), cache_dir=_path(cfg, "cache_dir")))
    return _FLIGHTS[map_name]


def compass(cfg: dict, flight, seed: int) -> np.ndarray:
    return compass_heading(flight.metadata["true_heading_deg"], np.random.default_rng(seed), **cfg["heading"]["compass"])


def flow_path(cfg: dict, seed: int) -> Path:
    return Path(_path(cfg, "cache_dir")) / f"sim_flow_{Path(cfg['recording']).name}_compass_seed{seed}.npy"


def flow_job(job: tuple) -> str:
    """Camera motion for one compass draw: the frames are turned north up by its readings."""
    cfg, seed = job
    flight0 = flight_for(cfg, "2018")
    shifts_for_flight(with_heading(flight0, compass(cfg, flight0, seed)), flow_path(cfg, seed))
    return f"seed {seed}"


def one_run(job: tuple) -> dict:
    cfg, name, settings, seed, keep_result = job
    settings = dict(settings)
    flight0 = flight_for(cfg, settings.pop("map"))
    reading = compass(cfg, flight0, seed)
    flight = with_heading(flight0, reading)
    shifts = shifts_for_flight(flight, flow_path(cfg, seed))
    run = replace(NavigatorConfig(**cfg["navigator"]), **settings)
    t0 = time.time()
    calibration = calibrate(flight, shifts, run)
    result = navigate(flight, shifts, calibration, run)
    errors = navigation_errors(result, flight)
    out = {
        "run": name, "seed": seed,
        **summarize_navigation(result, flight).as_dict(),
        **{f"integrity_{k}": v for k, v in integrity_summary(errors, cfg["alert_limit_m"]).as_dict().items() if k != "alert_limit_m"},
        "camera_lost_track": float(np.mean(result.lost_track)) if result.lost_track is not None else 0.0,
        "run_time_s": round(time.time() - t0, 1),
    }
    if keep_result:
        out["_result"] = result
    return out


def main() -> int:
    args = parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    seeds = args.seeds or cfg["seeds"]
    runs = {n: s for n, s in cfg["runs"].items() if not args.only or n in args.only}
    out = Path(args.output_dir or REPO_ROOT / cfg["output_dir"])
    out.mkdir(parents=True, exist_ok=True)
    f = flight_for(cfg, "2018")
    print(f"{f.name}: {len(f)} frames, {f.travelled[-1] / 1000:.2f} km, {f.timestamp[-1]:.0f} s; map {f.ground_map.shape[1]} x {f.ground_map.shape[0]} px")

    t0 = time.time()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for done in pool.map(flow_job, [(cfg, s) for s in seeds]):
            print(f"  camera motion ready: {done}", flush=True)
        jobs = [(cfg, name, s, seed, name in cfg.get("plot", []) and seed == seeds[0]) for name, s in runs.items() for seed in seeds]
        rows = []
        for r in pool.map(one_run, jobs):
            rows.append(r)
            print(f"  done {len(rows)}/{len(jobs)}: {r['run']} seed {r['seed']}: median {r['median']:.1f} m, "
                  f"used {r['fixes_used']}, wrong used {r['used_but_wrong']}, {r['run_time_s']:.0f} s", flush=True)

    results = {r["run"]: r.pop("_result") for r in rows if "_result" in r}
    columns = [k for k in rows[0] if not k.startswith("_")]
    with open(out / "results.csv", "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    summary = {}
    print(f"\n{f.name} (median over seeds {seeds}; integrity with an alert limit of {cfg['alert_limit_m']:g} m)")
    for name in runs:
        mine = [r for r in rows if r["run"] == name]
        med = {k: float(np.median([r[k] for r in mine])) for k in columns if isinstance(mine[0][k], (int, float)) and k != "seed"}
        med["used_but_wrong_per_seed"] = [r["used_but_wrong"] for r in mine]
        summary[name] = med
        print(f"  {name:16s} median {med['median']:7.1f} m, 90% below {med['p90']:7.1f}, worst {med['worst']:7.1f}, end {med['end']:7.1f} | "
              f"used {med['fixes_used']:4.0f}, wrong among them {med['used_but_wrong_per_seed']} | "
              f"within 3 sigma {med['within_3_sigma']:.0%}, hazardous {med['integrity_hazardous']:.1%}")
    (out / "metrics.json").write_text(json.dumps({"config": cfg, "seeds": seeds, "summary": summary}, indent=2, default=str))

    from src.visualization.navigator_plot import plot_navigation, plot_stanford

    if results:
        plot_navigation(f, results, out / "navigator.png", title="Simulated flight over Wufeng: GNSS lost after 450 m, camera motion and map fixes")
        plot_stanford({k: navigation_errors(v, f) for k, v in results.items()}, cfg["alert_limit_m"], out / "stanford.png",
                      title="Simulated flight: true error against the bound the navigator states")
    print(f"\nresults in {out} ({time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

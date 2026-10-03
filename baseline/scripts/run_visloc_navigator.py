"""Run the camera navigator on UAV-VisLoc flights: real photos against a real satellite map.

From the repository root:

    python baseline/scripts/run_visloc_navigator.py                       # flight 03, where the method was developed
    python baseline/scripts/run_visloc_navigator.py --flights 01 04       # the held-out flights
    python baseline/scripts/run_visloc_navigator.py --only first_version confirm_body --seeds 1

Results go to outputs/visloc_navigator/: results.csv (one row per flight, run and seed),
metrics.json (medians over the seeds), and per flight navigator.png and stanford.png.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np
import yaml

BASELINE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = BASELINE_DIR.parent
sys.path.insert(0, str(BASELINE_DIR))

from src.data.uav_visloc import VisLocConfig, VisLocDataNotFound, load_visloc_flight, with_heading  # noqa: E402
from src.estimation.camera_navigator import NavigatorConfig, calibrate, navigate  # noqa: E402
from src.evaluation.navigation_metrics import integrity_summary, navigation_errors, summarize_navigation  # noqa: E402
from src.sensors.dead_reckoning import simulated_steps  # noqa: E402
from src.sensors.heading import compass_heading, sun_elevation, sun_heading  # noqa: E402

_FLIGHTS: dict[str, object] = {}  # one loaded flight per worker process and flight number


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=BASELINE_DIR / "configs" / "visloc_navigator.yaml")
    p.add_argument("--flights", nargs="+", help="flight numbers, e.g. 01 04")
    p.add_argument("--seeds", nargs="+", type=int)
    p.add_argument("--only", nargs="+", help="names of the runs to do")
    p.add_argument("--output-dir", type=Path)
    p.add_argument("--workers", type=int, default=8)
    return p.parse_args()


def _path(cfg: dict, key: str) -> str:
    p = Path(cfg[key])
    return str(p if p.is_absolute() else REPO_ROOT / p)


def _flight(cfg: dict, number: str):
    if number not in _FLIGHTS:
        _FLIGHTS[number] = load_visloc_flight(VisLocConfig(data_root=_path(cfg, "data_root"), flight=number, cache_dir=_path(cfg, "cache_dir")))
    return _FLIGHTS[number]


def heading_reading(cfg: dict, flight, sensor: str, rng: np.random.Generator) -> np.ndarray:
    """What the chosen heading sensor would read on this flight."""
    true = flight.metadata["true_heading_deg"]
    if sensor == "compass":
        return compass_heading(true, rng, **cfg["heading"]["compass"])
    if sensor == "sun":
        zone = timezone(timedelta(hours=cfg["time_zone_hours"]))
        elevation = np.array([
            sun_elevation(lat, lon, datetime.fromisoformat(t).replace(tzinfo=zone))
            for (lat, lon), t in zip(flight.metadata["lat_lon"], flight.metadata["date"])
        ])
        return sun_heading(true, elevation, rng, **cfg["heading"]["sun"])
    if sensor == "true":
        return np.array(true, dtype=float)
    raise ValueError(f"unknown heading sensor {sensor!r}")


def one_run(job: tuple) -> dict:
    """One flight, one run, one seed. Returns the numbers and, if asked, the result for plotting."""
    cfg, number, name, settings, seed, keep_result = job
    settings = dict(settings)
    sensor = settings.pop("heading")
    base = replace(NavigatorConfig(**cfg["navigator"]), **settings)
    flight0 = _flight(cfg, number)
    rng = np.random.default_rng(seed)
    reading = heading_reading(cfg, flight0, sensor, rng)
    flight = with_heading(flight0, reading)
    steps = simulated_steps(flight.position_gt, reading - flight0.metadata["true_heading_deg"], rng, **cfg["dead_reckoning"])
    t0 = time.time()
    calibration = calibrate(flight, steps, base)
    result = navigate(flight, steps, calibration, base)
    errors = navigation_errors(result, flight)
    out = {
        "flight": number, "run": name, "seed": seed, "heading": sensor,
        **summarize_navigation(result, flight).as_dict(),
        **{f"integrity_{k}": v for k, v in integrity_summary(errors, cfg["alert_limit_m"]).as_dict().items() if k != "alert_limit_m"},
        "heading_error_median_abs_deg": float(np.median(np.abs(reading - flight0.metadata["true_heading_deg"]))),
        "run_time_s": round(time.time() - t0, 1),
    }
    if keep_result:
        out["_result"] = result
    return out


def main() -> int:
    args = parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    flights = args.flights or cfg["flights"]
    seeds = args.seeds or cfg["seeds"]
    runs = {n: s for n, s in cfg["runs"].items() if not args.only or n in args.only}
    out = Path(args.output_dir or REPO_ROOT / cfg["output_dir"])
    out.mkdir(parents=True, exist_ok=True)
    try:
        for number in flights:
            f = _flight(cfg, number)
            print(f"flight {number}: {len(f)} photos, {f.travelled[-1] / 1000:.1f} km, map {f.ground_map.shape[1]} x {f.ground_map.shape[0]} m")
    except VisLocDataNotFound as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    jobs = [(cfg, n, name, s, seed, name in cfg.get("plot", []) and seed == seeds[0]) for n in flights for name, s in runs.items() for seed in seeds]
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(one_run, jobs))

    results = {(r["flight"], r["run"]): r.pop("_result") for r in rows if "_result" in r}
    columns = [k for k in rows[0] if not k.startswith("_")]
    with open(out / "results.csv", "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    summary = {}
    for n in flights:
        print(f"\nflight {n} (median over seeds {seeds}; integrity with an alert limit of {cfg['alert_limit_m']:g} m)")
        for name in runs:
            mine = [r for r in rows if r["flight"] == n and r["run"] == name]
            med = {k: float(np.median([r[k] for r in mine])) for k in columns if isinstance(mine[0][k], (int, float)) and k != "seed"}
            med["used_but_wrong_per_seed"] = [r["used_but_wrong"] for r in mine]
            summary.setdefault(n, {})[name] = med
            print(f"  {name:22s} median {med['median']:7.1f} m, 90% below {med['p90']:7.1f}, worst {med['worst']:7.1f} | "
                  f"used {med['fixes_used']:4.0f}, wrong among them {med['used_but_wrong_per_seed']} | "
                  f"within 3 sigma {med['within_3_sigma']:.0%}, hazardous {med['integrity_hazardous']:.1%}, unavailable {med['integrity_unavailable']:.0%}")
    (out / "metrics.json").write_text(json.dumps({"config": cfg, "flights": flights, "seeds": seeds, "summary": summary}, indent=2, default=str))

    from src.visualization.navigator_plot import plot_navigation, plot_stanford

    for n in flights:
        drawn = {name: results[(n, name)] for name in cfg.get("plot", []) if (n, name) in results}
        if drawn:
            f = _flight(cfg, n)
            plot_navigation(f, drawn, out / f"navigator_{n}.png", title=f"UAV-VisLoc flight {n}: GNSS lost after 1 km, then camera fixes against a satellite map")
            plot_stanford({k: navigation_errors(v, f) for k, v in drawn.items()}, cfg["alert_limit_m"], out / f"stanford_{n}.png",
                          title=f"Flight {n}: true error against the bound the navigator states")
    print(f"\nresults in {out} ({time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

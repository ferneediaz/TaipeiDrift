"""Run the camera navigator on the simulated Wufeng flight: camera motion and map fixes, GNSS lost after 450 m.

From the repository root, after a flight was recorded (sim/README.md):

    python baseline/scripts/run_sim_navigator.py
    python baseline/scripts/run_sim_navigator.py --only map_2018 --seeds 1

Results go to outputs/sim_navigator/: results.csv (one row per run and seed), metrics.json (medians
over the seeds), navigator.png and stanford.png.
"""
from __future__ import annotations

import argparse
import hashlib
import csv
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timedelta
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

BASELINE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = BASELINE_DIR.parent
sys.path.insert(0, str(BASELINE_DIR))

from src.data.camera_model import CameraModel  # noqa: E402
from src.data.sim_replay import SimReplayConfig, load_sim_flight, with_camera, with_heading  # noqa: E402
from src.estimation.camera_navigator import NavigatorConfig, calibrate, navigate  # noqa: E402
from src.estimation.image_motion import detail_for_flight, shifts_for_flight  # noqa: E402
from src.evaluation.navigation_metrics import integrity_summary, navigation_errors, summarize_navigation  # noqa: E402
from src.sensors.heading import compass_heading, sun_position  # noqa: E402
from src.sensors.sun_sensor import SunSensorModel, sun_heading_readings  # noqa: E402

_FLIGHTS: dict[str, object] = {}  # one loaded flight per worker process and map


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, default=BASELINE_DIR / "configs" / "sim_navigator.yaml")
    p.add_argument("--seeds", nargs="+", type=int)
    p.add_argument("--only", nargs="+", help="names of the runs to do")
    p.add_argument("--output-dir", type=Path)
    p.add_argument("--workers", type=int, default=6)
    p.add_argument("--recording", help="another recorded flight, e.g. recordings/wufeng_south_80m")
    p.add_argument("--route", help="its route file, e.g. sim/scenarios/wufeng_south_80m.json")
    p.add_argument("--camera", default="ideal", help="a camera from the config's cameras: ideal (the simulator's frames) or realistic")
    return p.parse_args()


def _path(cfg: dict, key: str) -> str:
    p = Path(cfg[key])
    return str(p if p.is_absolute() else REPO_ROOT / p)


def flight_for(cfg: dict, map_name: str):
    """The recorded flight with the chosen map, with the true heading (loaded once per process)."""
    key = f"{cfg['recording']}|{map_name}|{json.dumps(cfg.get('camera'), sort_keys=True)}"
    if key not in _FLIGHTS:
        tif = Path(cfg["maps"][map_name])
        flight = load_sim_flight(SimReplayConfig(
            recording=_path(cfg, "recording"), map_tif=str(tif if tif.is_absolute() else REPO_ROOT / tif),
            route=_path(cfg, "route"), cache_dir=_path(cfg, "cache_dir")))
        camera = cfg.get("camera")
        _FLIGHTS[key] = with_camera(flight, CameraModel(**camera) if camera else None)
    return _FLIGHTS[key]


def compass(cfg: dict, flight, seed: int) -> np.ndarray:
    return compass_heading(flight.metadata["true_heading_deg"], np.random.default_rng(seed), **cfg["heading"]["compass"])


def heading_reading(cfg: dict, flight, seed: int) -> np.ndarray:
    """The heading sensor's readings for one draw (seed): the compass, or a sun sensor (src/sensors/sun_sensor.py).

    A sun sensor is told the flight's date and time (``heading.when``) and the world's latitude and longitude
    (the recording's meta.json). Under the realistic camera's clouds, the sun is hidden when the drone's line
    to the sun passes through a cloud: traced down to the ground, it ends in that cloud's shadow.
    """
    source = cfg["heading"].get("source", "compass")
    if source == "compass":
        return compass(cfg, flight, seed)
    model = SunSensorModel(**cfg["heading"][source])
    meta = json.loads((Path(flight.metadata["recording"]) / "meta.json").read_text())
    lat, lon = meta["origin"]["lat_deg"], meta["origin"]["lon_deg"]
    start = datetime.fromisoformat(cfg["heading"]["when"])
    hidden = None
    camera = flight.metadata.get("camera")
    if camera is not None:
        az_el = np.array([sun_position(lat, lon, start + timedelta(seconds=float(s))) for s in flight.timestamp])
        a, zenith = np.radians(az_el[:, 0]), np.radians(90.0 - az_el[:, 1])
        reach = flight.metadata["height_m"] * np.tan(zenith)  # from the drone along the sun's ray to the ground
        north = flight.position_gt[:, 0] - reach * np.cos(a)
        east = flight.position_gt[:, 1] - reach * np.sin(a)
        hidden = camera.shadow_at(north, east, flight.timestamp) > 0.5 * camera.model.cloud_shadow
    reading, _ = sun_heading_readings(model, flight.metadata["attitude_q"], flight.timestamp, start, lat, lon,
                                      np.random.default_rng(seed), hidden)
    return reading


def flow_path(cfg: dict, seed: int) -> Path:
    """Where the camera motion of one heading-sensor draw is cached.

    The name carries a fingerprint of the recording's image list and of the heading sensor's settings, so
    a flight recorded again under the same name, or another sensor, never reuses old camera motion.
    """
    recording = Path(_path(cfg, "recording"))
    source = cfg["heading"].get("source", "compass")
    sensor = cfg["heading"]["compass"] if source == "compass" else {source: cfg["heading"][source], "when": cfg["heading"]["when"]}
    content = (recording / "images.csv").read_bytes() + json.dumps(sensor, sort_keys=True).encode()
    if cfg.get("camera"):  # the ideal camera keeps the fingerprint it always had
        content += json.dumps(cfg["camera"], sort_keys=True).encode()
    return Path(_path(cfg, "cache_dir")) / f"sim_flow_{recording.name}_{hashlib.sha1(content).hexdigest()[:10]}_{source}_seed{seed}.npy"


def detail_path(cfg: dict, seed: int) -> Path:
    """Where the picture detail of one draw is cached: next to its camera motion."""
    path = flow_path(cfg, seed)
    return path.with_name(path.stem + "_detail.npy")


def flow_job(job: tuple) -> str:
    """Camera motion for one heading-sensor draw: the frames are turned north up by its readings."""
    cfg, seed = job
    flight0 = flight_for(cfg, "2018")
    flight = with_heading(flight0, heading_reading(cfg, flight0, seed))
    shifts_for_flight(flight, flow_path(cfg, seed))
    if cfg["navigator"].get("detail_scaling"):
        detail_for_flight(flight, detail_path(cfg, seed))
    return f"seed {seed}"


def one_run(job: tuple) -> dict:
    cfg, name, settings, seed, keep_result = job
    settings = dict(settings)
    flight0 = flight_for(cfg, settings.pop("map"))
    reading = heading_reading(cfg, flight0, seed)
    flight = with_heading(flight0, reading)
    shifts = shifts_for_flight(flight, flow_path(cfg, seed))
    run = replace(NavigatorConfig(**cfg["navigator"]), **settings)
    detail = detail_for_flight(flight, detail_path(cfg, seed)) if run.detail_scaling else None
    t0 = time.time()
    calibration = calibrate(flight, shifts, run, detail)
    result = navigate(flight, shifts, calibration, run, detail)
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
    if args.recording:
        cfg["recording"], cfg["route"] = args.recording, args.route or cfg["route"]
        cfg["output_dir"] = f"{cfg['output_dir']}_{Path(args.recording).name}"
    cfg["camera"] = cfg.get("cameras", {}).get(args.camera)
    if args.camera != "ideal":
        if args.camera not in cfg.get("cameras", {}):
            raise SystemExit(f"no camera {args.camera!r} in {args.config}")
        cfg["output_dir"] = f"{cfg['output_dir']}_{args.camera}_camera"
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
        plot_navigation(f, results, out / "navigator.png", title=f"Simulated flight over Wufeng ({Path(cfg['recording']).name}): GNSS lost after 450 m, camera motion and map fixes")
        plot_stanford({k: navigation_errors(v, f) for k, v in results.items()}, cfg["alert_limit_m"], out / "stanford.png",
                      title="Simulated flight: true error against the bound the navigator states")
    print(f"\nresults in {out} ({time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

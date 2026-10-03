"""Rerun every stage of Saturday's improvement loop on the development flights, from the same recordings.

Each stage is the navigator as it stood at a save point; later stages add one change each. The settings
are written out below against today's configuration, so the table and figures in
docs/simulation-results.md can be reproduced exactly:

    python scripts/sim_progress.py          # writes outputs/sim_progress.json (about 10 minutes)

Stage 1 is save point 1 with the calibration zooms widened for heights from about 58 to 134 m, without
which the 65 m flight cannot run at all.
"""
from __future__ import annotations

import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "baseline"))
sys.path.insert(0, str(REPO / "baseline" / "scripts"))
sys.path.insert(0, str(REPO / "scripts"))

import run_sim_navigator as S  # noqa: E402
from sim_dev_check import flight_cfg, recorded  # noqa: E402

BEFORE_SP3 = {"wide_zooms": [0.60, 1.10], "zoom_limits": [0.5, 1.1], "confirm_above_radius_m": 150.0}
STAGES = {
    "1 save point 1": ({"scale_from_fixes": True, **BEFORE_SP3}, "compass"),
    "2 keep the scale": ({"scale_from_fixes": False, **BEFORE_SP3}, "compass"),
    "3 zooms of the heights flown, confirm from 100 m": ({}, "compass"),
    "4 heading from the sun": ({}, "sun_digital"),
}
RUNS = ["camera_alone", "map_2018", "map_2020_fresh"]


def main() -> int:
    cfg = yaml.safe_load((REPO / "baseline" / "configs" / "sim_navigator.yaml").read_text())
    flights = {n: r for n, r in cfg["flights"]["development"].items() if recorded(n)}
    jobs, flows, keys = [], [], []
    for stage, (overrides, heading) in STAGES.items():
        c = {**cfg, "heading": {**cfg["heading"], "source": heading}}
        for name, route in flights.items():
            fc = flight_cfg(c, name, route, "ideal", overrides)
            flows += [(fc, s) for s in cfg["seeds"]]
            for run in RUNS:
                for seed in cfg["seeds"]:
                    jobs.append((fc, run, cfg["runs"][run], seed, False))
                    keys.append((stage, name, run, seed))
    with ProcessPoolExecutor(max_workers=6) as pool:
        list(pool.map(S.flow_job, flows))
        rows = list(pool.map(S.one_run, jobs))

    out: dict = {}
    for (stage, name, run, seed), r in zip(keys, rows):
        out.setdefault(stage, {}).setdefault(name, {}).setdefault(run, []).append(
            {k: r[k] for k in ("seed", "median", "p90", "worst", "fixes_used", "used_but_wrong", "within_3_sigma", "integrity_hazardous")})
    for stage, by_flight in out.items():
        print(stage)
        for name, by_run in by_flight.items():
            m = by_run["map_2018"]
            works = sum(x["used_but_wrong"] for x in m) == 0 and min(x["within_3_sigma"] for x in m) >= 0.99 and max(x["integrity_hazardous"] for x in m) == 0
            print(f"  {name:22s} 2018 map median {np.median([x['median'] for x in m]):5.1f}, worst {np.median([x['worst'] for x in m]):6.1f}, "
                  f"wrong {sum(x['used_but_wrong'] for x in m)}, {'works' if works else 'fails'}")
    path = REPO / "outputs" / "sim_progress.json"
    path.write_text(json.dumps({"stages": list(STAGES), "flights": list(flights), "results": out}, indent=1))
    print(f"written {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

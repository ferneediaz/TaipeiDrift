"""Run a navigator change on the development flights and check it against what "works" means.

The improvement loop (Saturday 3 October): a change is kept only if it works on every development
flight, does not make them worse, and passes scripts/check_save_point.py (the real data). The sealed
flights are never run here; they run once, at the end. Flights and their roles are in
baseline/configs/sim_navigator.yaml under ``flights``.

"Works", fixed before the loop started, for the realistic map (2018) on every flight and seed:
- no wrong fix used (a used fix more than 50 m from the truth);
- the true error within the stated 3 sigma at least 99 percent of the time;
- never hazardous: never more than 50 m off while stating that it is within 50 m.

    python scripts/sim_dev_check.py                                  # as configured
    python scripts/sim_dev_check.py --set scale_from_fixes=false     # a candidate change
    python scripts/sim_dev_check.py --camera realistic --set scale_from_fixes=false
"""
from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "baseline"))
sys.path.insert(0, str(REPO / "baseline" / "scripts"))

import run_sim_navigator as S  # noqa: E402


def parse_value(text: str):
    """'false' -> False, '0.3' -> 0.3, 'none' -> None, anything else stays text."""
    low = text.lower()
    if low in ("true", "false"):
        return low == "true"
    if low in ("none", "null"):
        return None
    try:
        return float(text) if any(c in text for c in ".e") else int(text)
    except ValueError:
        return text


def recorded(name: str) -> bool:
    """The recorder writes the duration into meta.json when it stops: a flight still recording has none."""
    meta = REPO / "recordings" / name / "meta.json"
    return meta.is_file() and json.loads(meta.read_text()).get("duration_s") is not None


def flight_cfg(cfg: dict, name: str, route: str, camera: str, overrides: dict) -> dict:
    out = {**cfg, "recording": f"recordings/{name}", "route": route, "camera": cfg.get("cameras", {}).get(camera)}
    out["navigator"] = {**cfg["navigator"], **overrides}
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--set", nargs="*", default=[], help="navigator settings to change, as name=value")
    p.add_argument("--camera", default="ideal")
    p.add_argument("--seeds", nargs="+", type=int)
    p.add_argument("--runs", nargs="+", default=["camera_alone", "map_2018", "map_2020_fresh"])
    p.add_argument("--flights", nargs="+", help="development flights to use (default: all that are recorded)")
    p.add_argument("--workers", type=int, default=6)
    args = p.parse_args()

    cfg = yaml.safe_load((REPO / "baseline" / "configs" / "sim_navigator.yaml").read_text())
    overrides = {k: parse_value(v) for k, v in (s.split("=", 1) for s in args.set)}
    seeds = args.seeds or cfg["seeds"]
    flights = {n: r for n, r in cfg["flights"]["development"].items() if (not args.flights or n in args.flights) and recorded(n)}
    print(f"development flights: {', '.join(flights)}; camera {args.camera}; changes {overrides or 'none'}")

    jobs, flows = [], []
    for name, route in flights.items():
        fc = flight_cfg(cfg, name, route, args.camera, overrides)
        flows += [(fc, s) for s in seeds]
        jobs += [(fc, run, cfg["runs"][run], s, False) for run in args.runs for s in seeds]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        list(pool.map(S.flow_job, flows))
        rows = list(pool.map(S.one_run, jobs))

    works = True
    for name in flights:
        print(f"\n{name}")
        for run in args.runs:
            mine = [r for r, j in zip(rows, jobs) if j[0]["recording"].endswith(name) and r["run"] == run]
            med = lambda k: float(np.median([r[k] for r in mine]))  # noqa: E731
            wrong = [int(r["used_but_wrong"]) for r in mine]
            within = [r["within_3_sigma"] for r in mine]
            hazardous = [r["integrity_hazardous"] for r in mine]
            line = (f"  {run:16s} median {med('median'):6.1f}  90% {med('p90'):6.1f}  worst {med('worst'):6.1f}  "
                    f"used {med('fixes_used'):3.0f}  wrong {wrong}  within 3 sigma {min(within):.1%} (lowest seed)  hazardous {max(hazardous):.1%} (highest)")
            if run == "map_2018":
                ok = sum(wrong) == 0 and min(within) >= 0.99 and max(hazardous) == 0.0
                works &= ok
                line += "  WORKS" if ok else "  FAILS"
            print(line)
    print(f"\n{'every development flight works' if works else 'not every development flight works'} (map_2018: no wrong fix, 99% within 3 sigma, never hazardous)")
    return 0 if works else 1


if __name__ == "__main__":
    sys.exit(main())

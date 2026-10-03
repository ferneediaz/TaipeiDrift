"""Check that the code still gives the numbers recorded at the last save point.

A save point is a tagged commit where the tests passed and the key numbers below were recorded in
scripts/save_points.json. Before anything new goes into the demo, run this: it reruns the tests and
the key numbers (about five minutes) and compares them with the save point. A difference is not
always a mistake, but it has to be explained before the change is kept.

From the repository root:

    python scripts/check_save_point.py                    # compare with the latest save point
    python scripts/check_save_point.py --record sp1 --note "what works"   # record a new one (then tag the commit)

What is checked:
- python -m pytest: every test, including ALTO's numbers (baseline/tests/test_alto_navigator.py pins them).
- UAV-VisLoc flight 03, the frozen method (confirm_body), seed 1: real photos against a satellite map.
- The simulated Wufeng flight, seed 3 (the median seed): camera alone, map fixes with the 2018 map, with the 2020 map.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
STORE = REPO / "scripts" / "save_points.json"
sys.path.insert(0, str(REPO / "baseline"))
sys.path.insert(0, str(REPO / "baseline" / "scripts"))

KEYS = ("median", "p90", "worst", "fixes_used", "used_but_wrong")
TOLERANCE_M = 1.0  # metres; or 3 percent, whichever is larger
TOLERANCE_COUNT = 1


def run_tests() -> dict:
    out = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider"], cwd=REPO, capture_output=True, text=True)
    tail = out.stdout.strip().splitlines()[-1] if out.stdout.strip() else out.stderr.strip()[-300:]
    counts = {k: int(v) for v, k in re.findall(r"(\d+) (passed|failed|skipped|errors?)", tail)}
    return {"passed": counts.get("passed", 0), "failed": counts.get("failed", 0) + counts.get("error", 0) + counts.get("errors", 0),
            "skipped": counts.get("skipped", 0), "summary": tail}


def run_visloc() -> dict:
    import run_visloc_navigator as V

    cfg = yaml.safe_load((REPO / "baseline/configs/visloc_navigator.yaml").read_text())
    row = V.one_run((cfg, "03", "confirm_body", cfg["runs"]["confirm_body"], 1, False))
    return {k: round(float(row[k]), 1) for k in KEYS}


def run_sim() -> dict:
    import run_sim_navigator as S

    cfg = yaml.safe_load((REPO / "baseline/configs/sim_navigator.yaml").read_text())
    S.flow_job((cfg, 3))
    return {name: {k: round(float(row[k]), 1) for k in KEYS}
            for name in ("camera_alone", "map_2018", "map_2020_fresh")
            for row in [S.one_run((cfg, name, cfg["runs"][name], 3, False))]}


def measure() -> dict:
    t0 = time.time()
    numbers = {"tests": run_tests()}
    print(f"tests: {numbers['tests']['summary']}", flush=True)
    numbers["visloc_03_confirm_body_seed1"] = run_visloc()
    print(f"UAV-VisLoc 03: {numbers['visloc_03_confirm_body_seed1']}", flush=True)
    numbers["sim_wufeng_seed3"] = run_sim()
    for name, values in numbers["sim_wufeng_seed3"].items():
        print(f"simulated flight, {name}: {values}", flush=True)
    numbers["seconds"] = round(time.time() - t0)
    return numbers


def compare(saved: dict, now: dict) -> list[str]:
    problems = []
    if now["tests"]["failed"]:
        problems.append(f"{now['tests']['failed']} tests fail")
    if now["tests"]["passed"] < saved["tests"]["passed"]:
        problems.append(f"fewer tests pass: {now['tests']['passed']} instead of {saved['tests']['passed']}")

    def close(a: float, b: float, key: str) -> bool:
        if key in ("fixes_used", "used_but_wrong"):
            return abs(a - b) <= TOLERANCE_COUNT
        return abs(a - b) <= max(TOLERANCE_M, 0.03 * abs(b))

    def check(label: str, old: dict, new: dict) -> None:
        for key in KEYS:
            if not close(new[key], old[key], key):
                problems.append(f"{label} {key}: {new[key]} now, {old[key]} at the save point")

    check("UAV-VisLoc 03", saved["visloc_03_confirm_body_seed1"], now["visloc_03_confirm_body_seed1"])
    for name, old in saved["sim_wufeng_seed3"].items():
        check(f"simulated flight {name}", old, now["sim_wufeng_seed3"][name])
    return problems


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--record", metavar="NAME", help="record the numbers as a new save point with this name")
    p.add_argument("--note", default="", help="one line: what works at this save point")
    args = p.parse_args()
    store = json.loads(STORE.read_text()) if STORE.is_file() else {"save_points": []}
    commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=REPO, capture_output=True, text=True).stdout.strip())
    now = measure()
    if args.record:
        if dirty:
            print("the working tree has uncommitted changes: commit first, so the save point is a commit")
            return 2
        if now["tests"]["failed"]:
            print("tests fail: not a save point")
            return 1
        store["save_points"].append({"name": args.record, "commit": commit, "date": time.strftime("%Y-%m-%d %H:%M"), "note": args.note, **now})
        STORE.write_text(json.dumps(store, indent=1) + "\n")
        print(f"recorded save point {args.record} at {commit}; commit scripts/save_points.json and tag: git tag -a {args.record} -m '{args.note}'")
        return 0
    if not store["save_points"]:
        print("no save point recorded yet")
        return 2
    last = store["save_points"][-1]
    problems = compare(last, now)
    if problems:
        print(f"\nDIFFERS from save point {last['name']} ({last['commit']}):")
        for line in problems:
            print(f"  - {line}")
        return 1
    print(f"\nsame as save point {last['name']} ({last['commit']}, {last['date']}): tests and key numbers hold")
    return 0


if __name__ == "__main__":
    sys.exit(main())

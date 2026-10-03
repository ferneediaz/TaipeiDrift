"""Score flights of the demo route, logged by scripts/demo_flight_batch.sh or recorded as takes for the video.

    python scripts/score_demo_flights.py outputs/demo/batch/r1 outputs/demo/take2 ...

One row per flight: the horizontal position error of each estimate against the simulator's truth over the time
without GNSS. For a flight that lands, that time ends when the drone arrives over the second helipad (the landing
is the autopilot's part); otherwise it ends with the log. The estimates:
    ours      camera speed, inertial sensors and the ships' fix (the dashboard's first row)
    camera    the same filter without the ships
    ships     the ships' bearings alone
    inertial  IMU and barometer only: what the drone has without us
Flights with more than 100 s without GNSS also get the errors in windows of 30 s.
"""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

PAD_B = (480.0, 110.0)           # the second helipad of the strait world (sim/scripts/make_islands.py)
NAMES = ("ours", "camera", "ships", "inertial")


def load(run: Path) -> dict:
    rows = list(csv.DictReader((run / "estimators.csv").open()))
    out = {}
    for name in NAMES:
        r = [x for x in rows if x["name"] == name]
        if not r:
            continue
        col = lambda k: np.array([float(x[k]) for x in r])
        out[name] = dict(t=col("t"), err=np.hypot(col("est_x") - col("gt_x"), col("est_y") - col("gt_y")),
                         two_sigma=2 * np.sqrt(np.maximum(col("pos_var_x") + col("pos_var_y"), 0)),
                         gnss=col("gnss_available"), gx=col("gt_x"), gy=col("gt_y"), gz=col("gt_z"))
    return out


def readings(run: Path, lost: float, end: float) -> str:
    """Camera speed readings used, and the ships' fixes used and refused, by our filter without GNSS."""
    try:
        records = [json.loads(line) for line in (run / "status.jsonl").open()]
    except FileNotFoundError:
        return ""
    mine = [s for s in records if s.get("instance") == "ours" and lost <= float(s.get("stamp") or 0) <= end]
    flow = sum(1 for s in mine if s.get("event") == "metric_flow" and s.get("accepted") is True)
    fix = [s.get("accepted") for s in mine if s.get("event") == "rf_update"]
    return f"{flow:4d} | {sum(1 for a in fix if a):3d} / {sum(1 for a in fix if a is False):<2d}"


def main() -> None:
    runs = [Path(a) for a in sys.argv[1:]]
    print("flight         | s without | ours: median  90%  worst  at end | inside   | camera: median worst | ships  | inertial | camera   | ships' fixes")
    print("               |   GNSS    |                                   | 2 sigma  |                      | median | at end   | readings | used / refused")
    medians, long_runs = [], []
    for run in runs:
        d = load(run)
        o = d["ours"]
        lost = float(o["t"][np.argmax(o["gnss"] == 0)]) if (o["gnss"] == 0).any() else float("nan")
        along = (o["gx"] * PAD_B[0] + o["gy"] * PAD_B[1]) / math.hypot(*PAD_B)
        over_pad = along > math.hypot(*PAD_B) - 15.0
        landed = o["gz"][-1] < 0.5 and o["gz"].max() > 20.0
        end = float(o["t"][np.argmax(over_pad)]) if landed and over_pad.any() else float(o["t"][-1])

        def part(name):
            x = d.get(name)
            if x is None:
                return np.array([np.nan])
            k = (x["t"] >= lost) & (x["t"] <= end)
            return x["err"][k] if k.any() else np.array([np.nan])

        k = (o["t"] >= lost) & (o["t"] <= end)
        e, cam, ships, inert = part("ours"), part("camera"), part("ships"), part("inertial")
        inside = 100 * np.mean(o["err"][k] <= o["two_sigma"][k])
        print(f"{run.name:14s} | {end - lost:7.0f}   | {np.median(e):12.1f} {np.percentile(e, 90):5.0f} {e.max():6.0f} {e[-1]:7.0f} |"
              f" {inside:5.0f} %  | {np.median(cam):14.1f} {cam.max():5.0f} | {np.median(ships):6.0f} | {inert[-1]:8.0f} | {readings(run, lost, end)}")
        medians.append(float(np.median(e)))
        if end - lost > 100:
            long_runs.append((run.name, d, lost, end))
    if len(medians) > 1:
        print(f"\nours, median error per flight: {min(medians):.1f} to {max(medians):.1f} m over {len(medians)} flights "
              f"(middle value {np.median(medians):.1f} m)")
    for name, d, lost, end in long_runs:
        print(f"\n{name}: median error in windows of 30 s after the loss, m")
        print("   seconds  | " + " | ".join(f"{n:>8s}" for n in NAMES if n in d))
        for a in np.arange(lost, end, 30.0):
            cells = []
            for n in NAMES:
                if n not in d:
                    continue
                k = (d[n]["t"] >= a) & (d[n]["t"] < a + 30.0)
                cells.append(f"{np.median(d[n]['err'][k]):8.1f}" if k.any() else " " * 8)
            print(f"  {a - lost:4.0f}-{min(a + 30, end) - lost:4.0f} | " + " | ".join(cells))


if __name__ == "__main__":
    main()

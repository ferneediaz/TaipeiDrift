"""Parallel Monte Carlo runner.

Usage:
    python -m trn.experiments.monte_carlo configs/experiments/<exp>.yaml [--quick] [--runs N] [--processes P]

An experiment file defines ``name``, ``routes``, ``filters``, ``overrides`` (dotted keys) and an optional
``sweep`` (dotted key -> list of values; cartesian product). Each (route, sweep point, run) task simulates
the sensors once and runs every filter on the identical data (paired comparison). Results go to
``results/<name>/``: resolved config, per-run metrics (CSV), 1 Hz error series (npz), summary (CSV),
progress.log.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import multiprocessing as mp
import os
import time
from pathlib import Path

import numpy as np
import yaml

from trn.common.config import apply_overrides, load_base_config, load_yaml, project_path


def load_experiment(path: str) -> tuple[dict, dict]:
    exp = load_yaml(path)
    cfg = apply_overrides(load_base_config(), exp.get("overrides"))
    return exp, cfg


def sweep_points(exp: dict) -> list[dict]:
    sw = exp.get("sweep") or {}
    if not sw:
        return [{}]
    keys = list(sw)
    return [dict(zip(keys, vals)) for vals in itertools.product(*[sw[k] for k in keys])]


def point_id(point: dict) -> str:
    if not point:
        return "default"
    return "__".join(f"{k.split('.')[-1]}={v}" for k, v in point.items()).replace("/", "-").replace(" ", "")


def _init_worker() -> None:
    """One numba thread per worker process: parallelism comes from the process pool (no oversubscription)."""
    import numba
    numba.set_num_threads(1)


def _task(args):
    cfg, route, point, run_idx, filters = args
    from trn.experiments.runner import run_once
    c = apply_overrides(cfg, point)
    t0 = time.time()
    res = run_once(c, route, run_idx, filters=filters, salt=json.dumps(point, sort_keys=True))
    res["point"] = point
    res["wall_s"] = time.time() - t0
    return res


def run_experiment(path: str, quick: bool = False, runs: int | None = None, processes: int | None = None) -> Path:
    exp, cfg = load_experiment(path)
    mc = cfg["montecarlo"]
    n_runs = runs or (exp.get("quick_runs", mc["quick_runs"]) if quick else exp.get("runs", mc["runs"]))
    procs = processes or mc["processes"]
    name = exp["name"] + ("_quick" if quick else "")
    out = project_path(cfg["data"]["results_dir"]) / name
    out.mkdir(parents=True, exist_ok=True)
    routes = exp.get("routes", list(cfg["routes"]))
    filters = exp.get("filters", ["tercom", "mpf_baseline", "mpf_gated", "mpf_proposed"])
    points = sweep_points(exp)
    (out / "experiment.yaml").write_text(yaml.safe_dump(exp, sort_keys=False))
    (out / "config_resolved.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    tasks = [(cfg, r, p, i, filters) for r in routes for p in points for i in range(n_runs)]
    log = open(out / "progress.log", "w")
    msg = f"[{exp['name']}] {len(tasks)} runs = {len(routes)} routes x {len(points)} points x {n_runs} runs, {procs} processes"
    print(msg, flush=True); log.write(msg + "\n"); log.flush()
    rows, series = [], {}
    t0 = time.time()
    ctx = mp.get_context("spawn")
    with ctx.Pool(procs, initializer=_init_worker) as pool:
        for k, res in enumerate(pool.imap_unordered(_task, tasks), 1):
            pid = point_id(res["point"])
            key = (res["route"], pid)
            s = series.setdefault(key, {"t": res["t"], "runs": {}, "ins": {}, "point": res["point"]})
            s["ins"][res["run"]] = res["ins_err"]
            for fname, rec in res["filters"].items():
                row = dict(route=res["route"], point=pid, run=res["run"], filter=fname,
                           usable_ground_frac=res["usable_ground_frac"], **{kk: vv for kk, vv in res["point"].items()},
                           **rec["metrics"])
                rows.append(row)
                s["runs"].setdefault(fname, {})[res["run"]] = (rec["err"], rec["sig"], rec["pvis"])
            el = time.time() - t0
            eta = el / k * (len(tasks) - k)
            fm = " ".join(f"{f}={res['filters'][f]['metrics']['rmse_post']:.0f}m" for f in filters)
            msg = (f"  [{k}/{len(tasks)}] {res['route']} {pid} run {res['run']}: {fm} "
                   f"({res['wall_s']:.0f}s, elapsed {el/60:.1f} min, ETA {eta/60:.1f} min)")
            print(msg, flush=True); log.write(msg + "\n"); log.flush()
    _write_outputs(out, rows, series, filters)
    msg = f"done in {(time.time()-t0)/60:.1f} min -> {out}"
    print(msg); log.write(msg + "\n"); log.close()
    return out


def _write_outputs(out: Path, rows: list, series: dict, filters: list) -> None:
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(out / "metrics.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: (r["route"], str(r["point"]), r["filter"], r["run"])))
    for (route, pid), s in series.items():
        runs = sorted(s["ins"])
        arr = {"t": s["t"], "runs": np.array(runs), "ins_err": np.stack([s["ins"][r] for r in runs])}
        for fname in filters:
            d = s["runs"][fname]
            arr[f"{fname}_err"] = np.stack([d[r][0] for r in runs])
            arr[f"{fname}_sig"] = np.stack([d[r][1] for r in runs])
            arr[f"{fname}_pvis"] = np.stack([d[r][2] for r in runs])
        np.savez_compressed(out / f"series_{route}_{pid}.npz", **arr)
    from trn.analysis.summary import summarize
    summarize(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiment")
    ap.add_argument("--quick", action="store_true", help="few runs for development")
    ap.add_argument("--runs", type=int)
    ap.add_argument("--processes", type=int)
    a = ap.parse_args()
    run_experiment(a.experiment, a.quick, a.runs, a.processes)


if __name__ == "__main__":
    main()

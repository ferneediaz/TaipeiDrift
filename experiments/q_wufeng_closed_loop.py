"""Closed-loop GNSS-denied position estimation over real Wufeng imagery.

The camera flies the 2020 orthophoto (what it sees); the onboard map is the 2018 orthophoto.
A GNSS outage is cut 20 s into the run; afterwards the estimator must dead-reckon on synthetic
ALTO-like odometry and take camera-to-map fixes every 10 s. Truth is used ONLY by the generator
(queries, scoring) and by this script's scorer; the estimator function is asserted not to receive it.

Everything synthetic is declared below as named constants. No number here is a measurement of real
hardware; the odometry model is a loose calibration to the team's ALTO dead reckoning (~10%/distance).

Run from the repository root:
    .venv/bin/python experiments/q_wufeng_closed_loop.py --seeds 3
Outputs (default data/processed/closed_loop/): runs.csv, timeseries.csv, summary.json, closed_loop.png.
"""
from __future__ import annotations

import argparse
import inspect
import json
import platform
import sys
import time
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from o_map_benchmark import (  # noqa: E402  (reuse the existing contract)
    QUERY, SEARCH, SOURCE, crop, learned_match, load_maps, load_xfeat,
    render, select_centres, template_match,
)

# ---------------------------------------------------------------------------------------------------
# Declared assumptions (generator-side and filter-side). Units: metres, seconds, degrees.
# ---------------------------------------------------------------------------------------------------
SEED_BASE = 20261003          # master seed: path construction uses SEED_BASE, run r uses SEED_BASE + r
DT_S = 1.0                    # sample interval of the simulated trajectory / filter step
SPEED_MPS = 15.0              # constant ground speed of the simulated flight
PATH_POINT_STRIDE = 3         # polyline through every 3rd principal-axis-ordered test centre
SELECT_COUNT = 1000           # select_centres cap (the test split only yields ~30 valid centres)

GNSS_CUT_S = 20.0             # GNSS is available at 1 Hz until this time, then cut
GNSS_SIGMA_M = 3.0            # per-axis GNSS noise, both initial fix and pre-cut updates
FIX_PERIOD_S = 10.0           # camera-to-map fix attempt period, counted from the cut
BLACKOUT_DEFAULT_AFTER_CUT = (60.0, 120.0)  # default outage window, seconds AFTER the cut
DEGRADED_CONDITIONS = (False, True)         # render() 'degraded' flag (blur + low contrast)

# Odometry generator: loose calibration to ALTO camera dead reckoning (~10% drift per distance).
ALTO_SCALE_SIGMA = 0.04       # scale bias, fraction of distance, drawn per run
ALTO_HEADING_INIT_SIGMA_DEG = 1.0   # initial heading bias, deg, drawn per run
ALTO_HEADING_RW_SIGMA_DEG = 0.05    # heading-bias random-walk step sigma, deg/sqrt(s)
ALTO_VEL_NOISE_MPS = 0.3      # white velocity noise per axis, m/s

# Position-only Kalman filter.
Q_PER_M = 0.10                # process noise std per metre of odometry increment (fraction of |inc|)
P_FLOOR_M2 = (0.05) ** 2      # isotropic process-noise floor added every step, m^2
R_FIX_M = 8.0                 # fix measurement std, m
CHI2_99_2DOF = 9.21           # chi-square innovation gate, 99%, 2 degrees of freedom
WRONG_FIX_M = 25.0            # a truth-scored accepted fix is "wrong" above this error

# Path validity (crop() must not raise; mask coverage must be full where the matcher looks).
COVERAGE_MIN = 0.995          # SOURCE window must be this fraction valid in the 2018 map mask
MIN_VALID_TEMPLATE = 96       # template_match's central template needs a fully valid window this big
VARIANTS = ("fused", "dr_only", "accept_all")

THRESHOLDS_CONFIRM = Path("data/processed/map_benchmark_confirm/summary.json")
THRESHOLDS_FALLBACK = Path("data/processed/map_benchmark/summary.json")


def load_thresholds():
    """Calibrated acceptance thresholds, from the confirmation run if present."""
    for path in (THRESHOLDS_CONFIRM, THRESHOLDS_FALLBACK):
        if path.exists():
            return json.loads(path.read_text())["thresholds"], path
    raise FileNotFoundError("no calibrated map_benchmark summary.json found")


def principal_axis_order(points: np.ndarray) -> np.ndarray:
    centre = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centre, full_matrices=False)
    return points[np.argsort((points - centre) @ vt[0])]


def resample_polyline(vertices: np.ndarray, spacing: float) -> np.ndarray:
    """Walk the polyline, emitting a point every `spacing` metres (1 m/px here)."""
    path = [vertices[0]]
    for a, b in zip(vertices[:-1], vertices[1:]):
        distance = float(np.linalg.norm(b - a))
        count = max(1, int(np.ceil(distance / spacing)))
        path.extend(a + (b - a) * np.linspace(0.0, 1.0, count, endpoint=False)[:, None])
    return np.asarray(path, float)


def step_is_valid(maps, masks, point) -> bool:
    """Crop must not raise and mask coverage must be full where the run actually looks."""
    try:
        source = crop(masks[1], point, SOURCE)
        search = crop(masks[0], point, SEARCH).astype(np.float32)
    except ValueError:
        return False
    if source.mean() < COVERAGE_MIN:
        return False
    window = cv2.matchTemplate(search, np.ones((MIN_VALID_TEMPLATE,) * 2, np.float32), cv2.TM_CCORR)
    return bool(window.max() >= MIN_VALID_TEMPLATE ** 2 - 0.5)


def build_path(maps, masks, rng):
    """Principal-axis path through every PATH_POINT_STRIDE-th test centre, longest valid run."""
    points = np.asarray(select_centres(masks, SELECT_COUNT, rng)["test"], float)
    vertices = principal_axis_order(points)[::PATH_POINT_STRIDE]
    raw = resample_polyline(vertices, SPEED_MPS * DT_S)
    valid = np.array([step_is_valid(maps, masks, p) for p in raw])
    best = (0, 0)
    i = 0
    while i < len(valid):                     # longest contiguous valid run
        if valid[i]:
            j = i
            while j < len(valid) and valid[j]:
                j += 1
            if j - i > best[1] - best[0]:
                best = (i, j)
            i = j
        else:
            i += 1
    path = raw[best[0]:best[1]]
    if len(path) < 3:
        raise ValueError(f"valid run too short ({len(path)} steps); inspect imagery coverage")
    heading = np.zeros(len(path))
    delta = np.diff(path, axis=0)
    heading[:-1] = np.degrees(np.arctan2(delta[:, 1], delta[:, 0]))
    heading[-1] = heading[-2]
    return path, heading


def draw_run(path, heading_deg, rng):
    """GENERATOR ONLY. Draw the odometry biases/noise and return measured increments + bias trace."""
    n = len(path)
    scale_bias = float(rng.normal(0.0, ALTO_SCALE_SIGMA))
    bias_deg = np.empty(n)
    bias_deg[0] = rng.normal(0.0, ALTO_HEADING_INIT_SIGMA_DEG)
    steps = rng.normal(0.0, ALTO_HEADING_RW_SIGMA_DEG, n - 1) * np.sqrt(DT_S)
    bias_deg[1:] = bias_deg[0] + np.cumsum(steps)
    true_increments = np.diff(path, axis=0)
    rotation = np.radians(bias_deg[:-1])
    cos, sin = np.cos(rotation), np.sin(rotation)
    rotated = np.empty_like(true_increments)
    rotated[:, 0] = cos * true_increments[:, 0] - sin * true_increments[:, 1]
    rotated[:, 1] = sin * true_increments[:, 0] + cos * true_increments[:, 1]
    noise = rng.normal(0.0, ALTO_VEL_NOISE_MPS * DT_S, true_increments.shape)
    measured = (1.0 + scale_bias) * rotated + noise
    return measured, bias_deg, scale_bias


def make_fix_queries(maps, path, bias_deg, degraded, blackout):
    """GENERATOR ONLY. Render one query per scheduled attempt; returns {t: query}. Truth is the source."""
    events = {}
    t = GNSS_CUT_S + FIX_PERIOD_S
    while t < len(path):
        if not (blackout[0] <= t <= blackout[1]):
            events[int(round(t))] = render(maps[1], path[int(round(t))], float(bias_deg[int(round(t))]),
                                           1.0, degraded)
        t += FIX_PERIOD_S
    return events


def assert_no_truth_parameter(function):
    """The estimator must not be handed the truth anywhere in its signature."""
    names = list(inspect.signature(function).parameters)
    assert not any("truth" in name.lower() for name in names), f"estimator signature leaks truth: {names}"


def kalman_update(state, covariance, measurement, measurement_cov):
    innovation = measurement - state
    gain = covariance @ np.linalg.inv(covariance + measurement_cov)
    return state + gain @ innovation, (np.eye(2) - gain) @ covariance


def match_once(query, reference, valid, order, model, thresholds, fused):
    """Cascade over `order`; fused=threshold acceptance, else first finite point. Returns (point, method)."""
    for method in order:
        if method == "xfeat_affine":
            point, score = learned_match(model, query, reference)
        else:
            point, score = template_match(query, reference, valid, method != "zncc")
        if not np.isfinite(point).all():
            continue
        if fused and score < thresholds[method]:
            continue
        return np.asarray(point, float), method, float(score)
    return None, None, float("nan")


def run_estimator(gnss0, gnss_events, odom_increments, fix_queries, maps, masks,
                  order, model, thresholds, variant, degraded):
    """POSITION-ONLY estimator. Receives initial GNSS, odometry increments and fix query images only.

    variant: 'fused'      -> threshold acceptance + chi-square innovation gate
             'dr_only'    -> dead reckoning only, fixes ignored
             'accept_all' -> every finite cascade output accepted, no threshold, no gate
    """
    n = len(odom_increments) + 1
    state = np.asarray(gnss0, float).copy()
    covariance = np.eye(2) * GNSS_SIGMA_M ** 2
    estimates = np.zeros((n, 2))
    estimates[0] = state
    accepted_points = []
    counters = dict(attempted=0, accepted=0, gated=0, outside_map=0)
    gnss_after_cut = 0
    for i in range(1, n):
        t = i * DT_S
        increment = odom_increments[i - 1]
        state = state + increment
        covariance = covariance + (Q_PER_M * float(np.linalg.norm(increment))) ** 2 * np.eye(2) \
            + P_FLOOR_M2 * np.eye(2)
        if t in gnss_events:
            assert t < GNSS_CUT_S, "GNSS used after the cut"
            state, covariance = kalman_update(state, covariance, np.asarray(gnss_events[t], float),
                                              np.eye(2) * GNSS_SIGMA_M ** 2)
        if t in fix_queries and variant != "dr_only":
            counters["attempted"] += 1
            query = fix_queries[t]
            prior = np.rint(state)
            try:
                reference = crop(maps[0], prior, SEARCH)
                valid = crop(masks[0], prior, SEARCH)
            except ValueError:
                counters["outside_map"] += 1
            else:
                point, _, _ = match_once(query, reference, valid, order, model, thresholds,
                                         fused=(variant == "fused"))
                if point is not None:
                    measurement = point + prior - SEARCH / 2
                    if variant == "fused":
                        innovation = measurement - state
                        gate = float(innovation @ np.linalg.inv(covariance + np.eye(2) * R_FIX_M ** 2)
                                     @ innovation)
                        if gate > CHI2_99_2DOF:
                            counters["gated"] += 1
                        else:
                            state, covariance = kalman_update(state, covariance, measurement,
                                                              np.eye(2) * R_FIX_M ** 2)
                            counters["accepted"] += 1
                            accepted_points.append((t, measurement))
                    else:
                        state = measurement
                        counters["accepted"] += 1
                        accepted_points.append((t, measurement))
        estimates[i] = state
    assert gnss_after_cut == 0
    return estimates, counters, accepted_points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("data/raw/aerial"))
    parser.add_argument("--output", type=Path, default=Path("data/processed/closed_loop"))
    parser.add_argument("--xfeat-root", type=Path, default=Path("data/raw/models/accelerated_features"))
    parser.add_argument("--no-xfeat", action="store_true", help="cascade without the learned matcher")
    parser.add_argument("--seeds", type=int, default=20, help="independent runs over the same path")
    parser.add_argument("--blackout", type=float, nargs=2, metavar=("START", "END"), default=None,
                        help="fix blackout window in seconds after start (default 60-120 s after the cut)")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    blackout = tuple(args.blackout) if args.blackout is not None else \
        (GNSS_CUT_S + BLACKOUT_DEFAULT_AFTER_CUT[0], GNSS_CUT_S + BLACKOUT_DEFAULT_AFTER_CUT[1])

    thresholds, thresholds_path = load_thresholds()
    maps, masks, transform, paths = load_maps(args.data)
    path, heading_deg = build_path(maps, masks, np.random.default_rng(SEED_BASE))
    print(f"path: {len(path)} steps, {len(path) - 1} m, {len(path) * DT_S:.0f} s, "
          f"GNSS cut at {GNSS_CUT_S:.0f} s, blackout {blackout[0]:.0f}-{blackout[1]:.0f} s", flush=True)
    if blackout[0] > len(path) * DT_S:
        print("WARNING: blackout lies beyond the path end; no fix attempt is suppressed", flush=True)

    model = None if args.no_xfeat else load_xfeat(args.xfeat_root)
    order = [m for m in ("zncc", "xfeat_affine", "zncc_yaw_scale")
             if m in thresholds and (m != "xfeat_affine" or model is not None)]
    assert_no_truth_parameter(run_estimator)
    truth = path
    time_grid = np.arange(len(path)) * DT_S

    rows, series = [], []
    errors = {(v, d): [] for v in VARIANTS for d in DEGRADED_CONDITIONS}
    start_time = time.perf_counter()
    for run in range(args.seeds):
        rng = np.random.default_rng(SEED_BASE + run)
        odom, bias_deg, scale_bias = draw_run(path, heading_deg, rng)
        gnss0 = truth[0] + rng.normal(0.0, GNSS_SIGMA_M, 2)
        gnss_events = {i: truth[i] + rng.normal(0.0, GNSS_SIGMA_M, 2)
                       for i in range(1, len(path)) if i * DT_S < GNSS_CUT_S}
        for degraded in DEGRADED_CONDITIONS:
            queries = make_fix_queries(maps, path, bias_deg, degraded, blackout)
            for variant in VARIANTS:
                estimates, counters, accepted = run_estimator(
                    gnss0, gnss_events, odom, queries, maps, masks, order, model,
                    thresholds, variant, degraded)
                error = np.linalg.norm(estimates - truth, axis=1)
                after_cut = time_grid >= GNSS_CUT_S
                wrong = 0
                for fix_time, fix_point in accepted:
                    if np.linalg.norm(fix_point - truth[int(round(fix_time))]) > WRONG_FIX_M:
                        wrong += 1
                rows.append(dict(seed=SEED_BASE + run, degraded=degraded, variant=variant,
                                 scale_bias=scale_bias, steps=len(estimates),
                                 final_error_m=float(error[-1]), max_error_m=float(error.max()),
                                 median_error_after_cut_m=float(np.median(error[after_cut])),
                                 fixes_attempted=counters["attempted"], fixes_accepted=counters["accepted"],
                                 fixes_gated=counters["gated"], fixes_outside_map=counters["outside_map"],
                                 accepted_wrong_25m=wrong))
                for t, e in zip(time_grid, error):
                    series.append(dict(seed=SEED_BASE + run, degraded=degraded, variant=variant,
                                       t_s=float(t), error_m=float(e)))
                errors[(variant, degraded)].append(error)
    runtime = time.perf_counter() - start_time

    runs = pd.DataFrame(rows)
    runs.to_csv(args.output / "runs.csv", index=False)
    pd.DataFrame(series).to_csv(args.output / "timeseries.csv", index=False)

    per_variant = {}
    for variant in VARIANTS:
        for degraded in DEGRADED_CONDITIONS:
            group = runs[(runs.variant == variant) & (runs.degraded == degraded)]
            per_variant[f"{variant}|degraded={degraded}"] = dict(
                final_error_median_m=float(group.final_error_m.median()),
                final_error_p90_m=float(group.final_error_m.quantile(.9)),
                max_error_median_m=float(group.max_error_m.median()),
                max_error_p90_m=float(group.max_error_m.quantile(.9)),
                median_after_cut_median_m=float(group.median_error_after_cut_m.median()),
                median_after_cut_p90_m=float(group.median_error_after_cut_m.quantile(.9)),
                fixes_attempted_sum=int(group.fixes_attempted.sum()),
                fixes_accepted_sum=int(group.fixes_accepted.sum()),
                fixes_gated_sum=int(group.fixes_gated.sum()),
                fixes_outside_map_sum=int(group.fixes_outside_map.sum()),
                accepted_wrong_25m_sum=int(group.accepted_wrong_25m.sum()))
    constants = dict(SEED_BASE=SEED_BASE, DT_S=DT_S, SPEED_MPS=SPEED_MPS,
                     PATH_POINT_STRIDE=PATH_POINT_STRIDE, SELECT_COUNT=SELECT_COUNT,
                     GNSS_CUT_S=GNSS_CUT_S, GNSS_SIGMA_M=GNSS_SIGMA_M, FIX_PERIOD_S=FIX_PERIOD_S,
                     BLACKOUT_DEFAULT_AFTER_CUT=list(BLACKOUT_DEFAULT_AFTER_CUT),
                     DEGRADED_CONDITIONS=list(DEGRADED_CONDITIONS),
                     ALTO_SCALE_SIGMA=ALTO_SCALE_SIGMA,
                     ALTO_HEADING_INIT_SIGMA_DEG=ALTO_HEADING_INIT_SIGMA_DEG,
                     ALTO_HEADING_RW_SIGMA_DEG=ALTO_HEADING_RW_SIGMA_DEG,
                     ALTO_VEL_NOISE_MPS=ALTO_VEL_NOISE_MPS, Q_PER_M=Q_PER_M,
                     P_FLOOR_M2=P_FLOOR_M2, R_FIX_M=R_FIX_M, CHI2_99_2DOF=CHI2_99_2DOF,
                     WRONG_FIX_M=WRONG_FIX_M, COVERAGE_MIN=COVERAGE_MIN,
                     MIN_VALID_TEMPLATE=MIN_VALID_TEMPLATE, QUERY=QUERY, SEARCH=SEARCH, SOURCE=SOURCE)
    report = dict(protocol=("Real 2018/2020 Wufeng orthophotos (1 m/px). Sick camera flies the 2020 image; "
                            "onboard map is 2018. Synthetic ALTO-like odometry, synthetic GNSS cut, "
                            "synthetic query degradation. Position-only filter. Truth used only by the "
                            "generator and the scorer. Not a flight test."),
                  constants=constants, seeds=dict(count=args.seeds,
                                                  values=[SEED_BASE + r for r in range(args.seeds)]),
                  thresholds=thresholds, thresholds_source=str(thresholds_path),
                  blackout=list(blackout), cascade_order=order, xfeat=not args.no_xfeat,
                  path=dict(steps=len(path), length_m=len(path) - 1, duration_s=float(len(path) * DT_S)),
                  runtime_s=runtime, platform=platform.platform(), python=platform.python_version(),
                  numpy=np.__version__, opencv=cv2.__version__,
                  median_p90_per_variant_and_condition=per_variant)
    (args.output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")

    figure, axes = plt.subplots(len(DEGRADED_CONDITIONS), 1, figsize=(8, 8), sharex=True, squeeze=False)
    for axis, degraded in zip(axes[:, 0], DEGRADED_CONDITIONS):
        for variant in VARIANTS:
            stack = np.asarray(errors[(variant, degraded)])
            axis.plot(time_grid, np.median(stack, axis=0), label=f"{variant} median")
            axis.plot(time_grid, np.quantile(stack, .9, axis=0), ls="--", lw=1,
                      label=f"{variant} p90")
        axis.axvspan(blackout[0], blackout[1], color="0.85", label="blackout")
        axis.axvline(GNSS_CUT_S, color="k", ls=":", lw=1)
        axis.set_ylabel(f"error (m), degraded={degraded}")
        axis.legend(fontsize=7)
    axes[-1, 0].set_xlabel("time (s)")
    figure.tight_layout()
    figure.savefig(args.output / "closed_loop.png", dpi=130)
    plt.close(figure)

    printable = per_variant
    print(f"runtime {runtime:.1f} s, seeds {args.seeds}, thresholds {thresholds_path}", flush=True)
    print(pd.DataFrame(printable).T.to_string(), flush=True)


if __name__ == "__main__":
    main()

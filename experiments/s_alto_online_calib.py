"""Test truth-free online scale and heading calibration on the ALTO validation flight.

Run from the repository root:
    .venv/bin/python experiments/s_alto_online_calib.py

The first 300 m use GNSS only for the team's camera-motion calibration and three
map fixes. After the jam, truth is used only to score trajectories and report the
whole-flight least-squares oracle reference.
"""
from __future__ import annotations

import json
import math
import time
import zipfile
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path.cwd()
ZIP = ROOT / "data/raw/alto/Val.zip"
FLOW_CACHE = ROOT / "data/processed/u2_baro_dem_scale/alto_flow.npy"
OUTPUT_DIR = ROOT / "data/processed/alto_online_calib"
LOCAL_FLOW_CACHE = OUTPUT_DIR / "alto_flow.npy"
FIGURE = OUTPUT_DIR / "online_calib.png"
REF_MPP = 0.60  # metres per pixel of the reference images, measured in f_alto_matching.py
JAM_AT_M = 300.0
FIX_SIGMA = 15.0  # metres, accuracy of one fix
DRIFT_RATE = 0.10  # share of distance since the last fix
KEEP = 0.8  # share of the camera frame used as the template
MIN_SCORE = 0.33  # from i_alto_zoom_check.py: no match at a wrong place scored above 0.32
FIX_SPACINGS_M = (300, 500, 1000, 2000)
ONLINE_LS_RW_PER_SQRT_KM = 0.02
ONLINE_PSI_RW_PER_SQRT_KM = math.radians(1.0)
ONLINE_ZOOM_SIGMA = 0.057
BLACKOUT_START_M = 2000.0


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def image_motion(archive: zipfile.ZipFile, query: pd.DataFrame) -> tuple[np.ndarray, str]:
    """Median Farneback image shift; extend the identical u2 cache if truncated."""
    n_frames = len(query)
    caches = (LOCAL_FLOW_CACHE, FLOW_CACHE)
    best_cache: np.ndarray | None = None
    best_path: Path | None = None
    for path in caches:
        if path.exists():
            candidate = np.load(path)
            if candidate.ndim == 2 and candidate.shape[1] == 2 and len(candidate) > (len(best_cache) if best_cache is not None else 0):
                best_cache, best_path = candidate, path
    if best_cache is not None and len(best_cache) >= n_frames:
        return np.asarray(best_cache[:n_frames], dtype=np.float64), f"{best_path} ({n_frames}/{len(best_cache)} rows)"

    flow = np.zeros((n_frames, 2), dtype=np.float64)
    copied = 0
    if best_cache is not None:
        copied = min(len(best_cache), n_frames)
        flow[:copied] = best_cache[:copied]

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    previous = None
    if copied:
        previous_name = query.name.iloc[copied - 1]
        raw = np.frombuffer(archive.read(f"Val/query_images/{previous_name}"), np.uint8)
        previous = cv2.resize(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE), (250, 250), interpolation=cv2.INTER_AREA)
    for k in range(copied, n_frames):
        name = query.name.iloc[k]
        raw = np.frombuffer(archive.read(f"Val/query_images/{name}"), np.uint8)
        frame = cv2.resize(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE), (250, 250), interpolation=cv2.INTER_AREA)
        if previous is not None:
            field = cv2.calcOpticalFlowFarneback(previous, frame, None, 0.5, 4, 21, 3, 7, 1.5, 0)
            flow[k] = np.median(field[60:190, 60:190].reshape(-1, 2), axis=0) * 2
        previous = frame
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    np.save(LOCAL_FLOW_CACHE, flow)
    source = f"{best_path} ({copied}/{n_frames} cached; recomputed suffix)" if best_path else f"computed and cached at {LOCAL_FLOW_CACHE}"
    return flow, source


def load_alto() -> dict:
    archive = zipfile.ZipFile(ZIP)
    query = pd.read_csv(archive.open("Val/query.csv"))
    ref = pd.read_csv(archive.open("Val/reference.csv"))
    ref = ref[ref.name.str.startswith("offset_0_None")].reset_index(drop=True)
    truth = query[["easting", "northing"]].to_numpy()
    ref_xy = ref[["easting", "northing"]].to_numpy()
    jam = 0
    jam_distance = 0.0
    for k in range(1, len(truth)):
        jam_distance += float(np.linalg.norm(truth[k] - truth[k - 1]))
        if jam_distance >= JAM_AT_M:
            jam = k
            break
    return {"archive": archive, "query": query, "ref": ref, "truth": truth, "ref_xy": ref_xy, "jam": jam, "jam_distance_m": jam_distance}


def make_matcher(archive: zipfile.ZipFile, query: pd.DataFrame, ref: pd.DataFrame, ref_xy: np.ndarray):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    images: dict[str, np.ndarray] = {}

    def load(name: str) -> np.ndarray:
        if name not in images:
            raw = np.frombuffer(archive.read(name), np.uint8)
            images[name] = clahe.apply(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)).astype(np.float32)
        return images[name]

    def template(frame: np.ndarray, zoom: float, angle: float) -> np.ndarray:
        n = int(round(500 * zoom))
        resized = cv2.resize(frame, (n, n), interpolation=cv2.INTER_AREA)
        rotated = cv2.warpAffine(resized, cv2.getRotationMatrix2D((n / 2, n / 2), angle, 1.0), (n, n))
        c = int(n * KEEP)
        o = (n - c) // 2
        return rotated[o : o + c, o : o + c]

    def nearest(estimate: np.ndarray, n: int) -> np.ndarray:
        return np.argsort(np.linalg.norm(ref_xy - estimate, axis=1))[:n]

    def fix(k: int, zooms: np.ndarray, angles: np.ndarray, candidates: np.ndarray) -> tuple:
        frame = load(f"Val/query_images/{query.name[k]}")
        best = (-1.0, None, 0.0, 0.0)
        for zoom in zooms:
            for angle in angles:
                t = template(frame, zoom, angle)
                for ri in candidates:
                    result = cv2.matchTemplate(load(f"Val/reference_images/{ref.name[ri]}"), t, cv2.TM_CCOEFF_NORMED)
                    _, score, _, loc = cv2.minMaxLoc(result)
                    if score > best[0]:
                        centre = np.array(loc) + t.shape[0] / 2
                        offset = np.array([(centre[0] - 250) * REF_MPP, -(centre[1] - 250) * REF_MPP])
                        best = (score, ref_xy[ri] + offset, zoom, angle)
        return best

    return nearest, fix


def calibrate(data: dict, flow: np.ndarray, nearest, fix) -> dict:
    truth, jam = data["truth"], data["jam"]
    steps = truth[1 : jam + 1] - truth[:jam]
    A0, *_ = np.linalg.lstsq(flow[1 : jam + 1], steps, rcond=None)
    calib_frames = (jam // 3, 2 * jam // 3, jam)
    calib = [fix(k, np.arange(0.60, 1.01, 0.05), np.arange(-10, 36, 5), nearest(truth[k], 7)) for k in calib_frames]
    zoom0 = float(np.median([c[2] for c in calib]))
    angle0 = float(np.median([c[3] for c in calib]))
    offset0 = np.median([c[1] - truth[k] for c, k in zip(calib, calib_frames)], axis=0)
    return {"A0": A0, "zoom0": zoom0, "angle0": angle0, "offset0": offset0, "calib_frames": calib_frames}


def scalar_kalman_update(state: float, variance: float, measurement: float, measurement_sigma: float, angle: bool = False) -> tuple[float, float, bool]:
    innovation = wrap_angle(measurement - state) if angle else measurement - state
    innovation_variance = variance + measurement_sigma**2
    if abs(innovation) > 3.0 * math.sqrt(innovation_variance):
        return state, variance, False
    gain = variance / innovation_variance
    state = wrap_angle(state + gain * innovation) if angle else state + gain * innovation
    return state, (1.0 - gain) * variance, True


def run(data: dict, flow: np.ndarray, calibration: dict, nearest, fix, variant: str, fix_every_m: float | None, blackout_after_m: float | None = None) -> dict:
    truth, ref_xy, jam = data["truth"], data["ref_xy"], data["jam"]
    A0, zoom0, angle0, offset0 = calibration["A0"], calibration["zoom0"], calibration["angle0"], calibration["offset0"]
    online = variant in ("online", "online_plus_zoom")
    plus_zoom = variant == "online_plus_zoom"
    estimate = truth[jam].copy()
    path = [estimate.copy()]
    distance_path = [0.0]
    flown_since_jam = 0.0
    position_variance = 3.0**2
    since_fix = 0.0
    since_try = 0.0
    zoom = zoom0
    team_scale = 1.0
    ls = 0.0
    psi_b = 0.0
    p_ls = 0.15**2
    p_psi = math.radians(5.0) ** 2
    anchor_frame: int | None = None
    anchor_position: np.ndarray | None = None
    accepted = rejected = wrong_50 = ls_updates = psi_updates = zoom_updates = 0
    log: list[dict] = []
    for k in range(jam + 1, len(truth)):
        raw_step = flow[k] @ A0
        if online:
            c, s = math.cos(psi_b), math.sin(psi_b)
            step = np.array([[c, -s], [s, c]]) @ raw_step * math.exp(ls)
        else:
            step = raw_step * team_scale
        estimate = estimate + step
        step_distance = float(np.linalg.norm(step))
        flown_since_jam += step_distance
        since_fix += step_distance
        since_try += step_distance
        if online:
            p_ls += ONLINE_LS_RW_PER_SQRT_KM**2 * step_distance / 1000.0
            p_psi += ONLINE_PSI_RW_PER_SQRT_KM**2 * step_distance / 1000.0

        due = fix_every_m is not None and since_try >= fix_every_m
        if due:
            since_try = 0.0
            distance_after_jam = flown_since_jam
            if blackout_after_m is None or distance_after_jam <= blackout_after_m:
                predicted_var = position_variance + (DRIFT_RATE * since_fix) ** 2
                zooms = np.clip(zoom + np.arange(-0.10, 0.11, 0.05), 0.5, 1.1)
                candidates = nearest(estimate, 7)
                radius = max(60.0, 3.0 * math.sqrt(predicted_var))
                inside = np.where(np.linalg.norm(ref_xy - estimate, axis=1) <= radius)[0]
                candidates = inside if len(inside) >= 7 else candidates
                if since_fix > 400:
                    zooms = np.arange(0.60, 1.101, 0.05)
                score, matched_position, new_zoom, _ = fix(k, zooms, np.array([angle0 - 5, angle0, angle0 + 5]), candidates)
                position = matched_position - offset0
                agrees = np.linalg.norm(position - estimate) <= 3.0 * math.sqrt(predicted_var + FIX_SIGMA**2)
                use = bool(agrees and score >= MIN_SCORE)
                fix_error = float(np.linalg.norm(position - truth[k]))
                log.append({"frame": int(k), "distance_m": float(distance_after_jam), "score": float(score), "fix_error_m": fix_error, "used": use})
                if use:
                    accepted += 1
                    wrong_50 += int(fix_error > 50.0)
                    if online:
                        if anchor_frame is not None and anchor_position is not None:
                            D_fix = position - anchor_position
                            fix_distance = float(np.linalg.norm(D_fix))
                            if fix_distance >= 150.0:
                                D_raw = np.sum(flow[anchor_frame + 1 : k + 1] @ A0, axis=0)
                                raw_distance = float(np.linalg.norm(D_raw))
                                if raw_distance > 1e-9:
                                    measurement_sigma = math.sqrt(2.0) * FIX_SIGMA / fix_distance
                                    ls_meas = math.log(fix_distance / raw_distance)
                                    ls, p_ls, did_update = scalar_kalman_update(ls, p_ls, ls_meas, measurement_sigma)
                                    ls_updates += int(did_update)
                                    psi_meas = wrap_angle(math.atan2(D_fix[1], D_fix[0]) - math.atan2(D_raw[1], D_raw[0]))
                                    psi_b, p_psi, did_update = scalar_kalman_update(psi_b, p_psi, psi_meas, measurement_sigma, angle=True)
                                    psi_updates += int(did_update)
                        anchor_frame, anchor_position = k, position.copy()
                        if plus_zoom:
                            ls_meas_zoom = math.log(new_zoom / zoom0)
                            ls, p_ls, did_update = scalar_kalman_update(ls, p_ls, ls_meas_zoom, ONLINE_ZOOM_SIGMA)
                            zoom_updates += int(did_update)
                    gain = predicted_var / (predicted_var + FIX_SIGMA**2)
                    estimate = estimate + gain * (position - estimate)
                    position_variance = (1.0 - gain) * predicted_var
                    if variant == "team_sized":
                        team_scale = new_zoom / zoom0
                        ls = math.log(team_scale)
                    zoom = new_zoom
                    since_fix = 0.0
                else:
                    rejected += 1
        path.append(estimate.copy())
        distance_path.append(flown_since_jam)

    path_array = np.asarray(path)
    errors = np.linalg.norm(path_array - truth[jam:], axis=1)
    final_ls = math.log(team_scale) if variant == "team_sized" else ls
    return {
        "variant": variant,
        "fix_spacing_m": fix_every_m,
        "path": path_array,
        "error": errors,
        "distance_m": np.asarray(distance_path),
        "fix_log": log,
        "fixes_used": accepted,
        "fixes_rejected": rejected,
        "used_wrong_over_50_m": wrong_50,
        "ls_final": float(final_ls),
        "psi_b_final_rad": float(psi_b),
        "ls_updates": ls_updates,
        "psi_updates": psi_updates,
        "zoom_updates": zoom_updates,
    }


def run_row(result: dict) -> dict:
    error = result["error"]
    return {
        "variant": result["variant"],
        "fix_spacing_m": result["fix_spacing_m"],
        "median_error_m": float(np.median(error)),
        "p90_error_m": float(np.percentile(error, 90)),
        "worst_error_m": float(error.max()),
        "end_error_m": float(error[-1]),
        "fixes_used": int(result["fixes_used"]),
        "fixes_rejected": int(result["fixes_rejected"]),
        "used_wrong_over_50_m": int(result["used_wrong_over_50_m"]),
        "final_ls": float(result["ls_final"]),
        "final_psi_b_deg": float(math.degrees(result["psi_b_final_rad"])),
        "ls_updates": int(result["ls_updates"]),
        "psi_updates": int(result["psi_updates"]),
        "zoom_updates": int(result["zoom_updates"]),
    }


def first_exceed_distance(result: dict, threshold_m: float, blackout_start_m: float) -> float | str:
    distance = result["distance_m"]
    error = result["error"]
    start = int(np.searchsorted(distance, blackout_start_m, side="left"))
    hits = np.flatnonzero(error[start:] > threshold_m)
    if len(hits) == 0:
        return "never within the route"
    index = start + int(hits[0])
    return float(max(0.0, distance[index] - blackout_start_m))


def oracle_reference(truth: np.ndarray, flow: np.ndarray, jam: int, A0: np.ndarray) -> dict:
    steps = np.diff(truth, axis=0)
    A_full, *_ = np.linalg.lstsq(flow[1:], steps, rcond=None)
    relative = np.linalg.solve(A0, A_full)
    left, singular_values, right_t = np.linalg.svd(relative)
    rotation_row = left @ right_t
    if np.linalg.det(rotation_row) < 0:
        left[:, -1] *= -1.0
        rotation_row = left @ right_t
    scale = float(np.mean(singular_values))
    psi = float(math.atan2(rotation_row[0, 1], rotation_row[0, 0]))
    return {
        "A_full": A_full.tolist(),
        "relative_A_full_over_A0": relative.tolist(),
        "relative_singular_values": singular_values.tolist(),
        "scale": scale,
        "ls": math.log(scale),
        "psi_b_rad": psi,
        "psi_b_deg": math.degrees(psi),
    }


def main() -> None:
    started = time.time()
    data = load_alto()
    flow, flow_source = image_motion(data["archive"], data["query"])
    nearest, fix = make_matcher(data["archive"], data["query"], data["ref"], data["ref_xy"])
    calibration = calibrate(data, flow, nearest, fix)
    print(f"jam after {data['jam_distance_m']:.0f} m (frame {data['jam']}). Learned before the jam: zoom {calibration['zoom0']:.2f}, rotation {calibration['angle0']:.0f} deg, fix offset {calibration['offset0'].round(1)} m")
    print(f"Flow: {flow_source}")

    results: dict[tuple[str, int], dict] = {}
    rows: list[dict] = []
    for spacing in FIX_SPACINGS_M:
        for variant in ("team_sized", "online", "online_plus_zoom"):
            print(f"Running {variant}, fixes every {spacing} m...", flush=True)
            result = run(data, flow, calibration, nearest, fix, variant, float(spacing))
            results[(variant, spacing)] = result
            rows.append(run_row(result))

    no_fix_result = run(data, flow, calibration, nearest, fix, "uncalibrated", None)
    blackout_rows: list[dict] = []
    blackout_results: dict[tuple[str, int], dict] = {}
    for spacing in (300, 500):
        for variant in ("team_sized", "online", "online_plus_zoom"):
            print(f"Running blackout {variant}, fixes every {spacing} m until {BLACKOUT_START_M:.0f} m...", flush=True)
            result = run(data, flow, calibration, nearest, fix, variant, float(spacing), BLACKOUT_START_M)
            blackout_results[(variant, spacing)] = result
        blackout_results[("uncalibrated", spacing)] = no_fix_result
        for variant in ("team_sized", "online", "online_plus_zoom", "uncalibrated"):
            result = blackout_results[(variant, spacing)]
            blackout_rows.append({
                "variant": variant,
                "fix_spacing_m": spacing if variant != "uncalibrated" else None,
                "blackout_start_m_after_jam": BLACKOUT_START_M,
                "fixes_used": result["fixes_used"],
                "fixes_rejected": result["fixes_rejected"],
                "distance_to_50m_error_m": first_exceed_distance(result, 50.0, BLACKOUT_START_M),
                "distance_to_100m_error_m": first_exceed_distance(result, 100.0, BLACKOUT_START_M),
                "end_error_m": float(result["error"][-1]),
            })

    run_table = pd.DataFrame(rows)
    blackout_table = pd.DataFrame(blackout_rows)
    expected = {
        300: {"median_error_m": 31.0, "worst_error_m": 73.0, "end_error_m": 38.0},
        1000: {"median_error_m": 56.0, "worst_error_m": 278.0, "end_error_m": 10.0},
        2000: {"median_error_m": 116.0, "worst_error_m": 578.0, "end_error_m": 197.0},
    }
    reproduction_rows = []
    print("\nTeam sized-search reproduction against docs/findings.md 3.4 (rounded published values):")
    for spacing, published in expected.items():
        measured = next(row for row in rows if row["variant"] == "team_sized" and row["fix_spacing_m"] == spacing)
        diffs = {key: abs(measured[key] - value) for key, value in published.items()}
        within = all(delta <= 2.0 for delta in diffs.values())
        reproduction_rows.append({"spacing_m": spacing, **{f"measured_{key}": measured[key] for key in published}, "max_abs_delta_m": max(diffs.values()), "within_2m": within})
        print(f"  {spacing:4d} m: median/worst/end {measured['median_error_m']:.1f}/{measured['worst_error_m']:.1f}/{measured['end_error_m']:.1f} m; published {published['median_error_m']:.0f}/{published['worst_error_m']:.0f}/{published['end_error_m']:.0f} m; {'MATCH' if within else 'DIFF'} (max |Δ|={max(diffs.values()):.1f} m)")

    oracle = oracle_reference(data["truth"], flow, data["jam"], calibration["A0"])
    print("\nWhole-path truth oracle (least-squares A_full relative to A0, polar similarity decomposition):")
    print(f"  scale={oracle['scale']:.6f}, ls={oracle['ls']:.6f}, psi_b={oracle['psi_b_deg']:.3f} deg; relative singular values={np.array(oracle['relative_singular_values']).round(6)}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    run_table.to_csv(OUTPUT_DIR / "runs.csv", index=False)
    blackout_table.to_csv(OUTPUT_DIR / "blackout.csv", index=False)
    figure, axis = plt.subplots(figsize=(10, 5.8))
    colors = {"team_sized": "tab:purple", "online": "tab:blue", "online_plus_zoom": "tab:green"}
    labels = {"team_sized": "team_sized", "online": "online", "online_plus_zoom": "online_plus_zoom"}
    for variant in ("team_sized", "online", "online_plus_zoom"):
        result = results[(variant, 1000)]
        axis.plot(result["distance_m"], result["error"], label=labels[variant], color=colors[variant], linewidth=1.2)
    axis.set_xlabel("distance flown since the jam (m)")
    axis.set_ylabel("position error (m)")
    axis.set_yscale("log")
    axis.grid(alpha=0.3)
    axis.legend()
    axis.set_title("ALTO online scale and heading calibration, 1,000 m fix spacing")
    figure.tight_layout()
    figure.savefig(FIGURE, dpi=120)
    plt.close(figure)

    runtime_s = time.time() - started
    summary = {
        "flow_source": flow_source,
        "jam_frame": int(data["jam"]),
        "jam_distance_m": float(data["jam_distance_m"]),
        "calibration": {
            "A0": calibration["A0"].tolist(),
            "zoom0": calibration["zoom0"],
            "angle0_deg": calibration["angle0"],
            "offset0_m": calibration["offset0"].tolist(),
        },
        "oracle_reference": oracle,
        "published_reproduction": reproduction_rows,
        "runs": rows,
        "blackout": blackout_rows,
        "runtime_seconds": runtime_s,
    }
    (OUTPUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")

    print("\nRuns table (errors in m; final psi_b in degrees):")
    display = run_table.copy()
    for column in ("median_error_m", "p90_error_m", "worst_error_m", "end_error_m", "final_ls", "final_psi_b_deg"):
        display[column] = display[column].map(lambda value: f"{value:.2f}")
    print(display.to_string(index=False))
    print("\nBlackout table (distances measured after the 2,000 m blackout start):")
    print(blackout_table.to_string(index=False))
    print(f"\nOnline final states versus oracle:")
    for spacing in FIX_SPACINGS_M:
        for variant in ("online", "online_plus_zoom"):
            row = next(row for row in rows if row["variant"] == variant and row["fix_spacing_m"] == spacing)
            print(f"  {variant:16s} {spacing:4d} m: ls={row['final_ls']:+.5f}, psi_b={row['final_psi_b_deg']:+.3f} deg")
    print(f"  oracle: ls={oracle['ls']:+.5f}, psi_b={oracle['psi_b_deg']:+.3f} deg")
    print(f"Run time: {runtime_s:.1f} s")
    print(f"Saved: {OUTPUT_DIR / 'summary.json'}, {OUTPUT_DIR / 'runs.csv'}, {OUTPUT_DIR / 'blackout.csv'}, {FIGURE}")
    data["archive"].close()


if __name__ == "__main__":
    main()

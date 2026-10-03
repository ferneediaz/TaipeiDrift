"""Held-out refactor of the ALTO end-to-end navigation experiment.

Run from the repository root with ``.venv/bin/python experiments/t_alto_heldout.py``.
The Val reproduction is a hard gate before any Round 2 Train results are computed.
"""
from __future__ import annotations

import argparse
import csv
import time
import zipfile
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

# Frozen values copied from experiments/h_alto_end_to_end.py and preregistration.md.
REF_MPP = 0.60
JAM_AT_M = 300.0
FIX_SIGMA = 15.0
DRIFT_RATE = 0.10
KEEP = 0.8
MIN_SCORE = 0.33
CALIB_ZOOMS = np.arange(0.60, 1.01, 0.05)
CALIB_ANGLES = np.arange(-10, 36, 5)
CALIB_NEAREST = 7
OUT_DIR = Path("data/processed/t_alto_heldout")
VAL_ZIP = Path("data/raw/alto/Val.zip")
TRAIN_ZIP = Path("data/raw/alto/UAV_Round2_Train.zip")
CACHE_IMAGES = 2000


@dataclass(frozen=True)
class RunConfig:
    name: str
    fix_every_m: float | None
    sized_search: bool = False
    min_score: float = 0.0


@dataclass
class PreparedRange:
    i0: int
    i1: int
    truth: np.ndarray
    travelled: np.ndarray
    jam: int
    A0: np.ndarray
    zoom0: float
    angle0: float
    offset0: np.ndarray
    calibration: list[tuple[float, np.ndarray, float, float]]


@dataclass
class RunResult:
    path: np.ndarray
    error: np.ndarray
    # absolute frame index, score, fix error in metres, accepted flag
    log: np.ndarray


class AltoDataset:
    """Zip-backed dataset and bounded decoded-image cache for one zip/prefix pair."""

    def __init__(self, zip_path: str | Path, prefix: str):
        self.zip_path = Path(zip_path)
        self.prefix = prefix
        self.archive = zipfile.ZipFile(self.zip_path)
        self.query = pd.read_csv(self.archive.open(f"{prefix}/query.csv"))
        self.reference_all = pd.read_csv(self.archive.open(f"{prefix}/reference.csv"))
        self.reference = self.reference_all[
            self.reference_all.name.str.startswith("offset_0_None")
        ].reset_index(drop=True)
        self.truth = self.query[["easting", "northing"]].to_numpy()
        self.ref_xy = self.reference[["easting", "northing"]].to_numpy()
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.images: OrderedDict[str, np.ndarray] = OrderedDict()
        self.flow: np.ndarray | None = None
        self.flow_cache = OUT_DIR / f"flow_{prefix}_{self.zip_path.stem}.npy"

    def close(self) -> None:
        self.archive.close()

    def load_image(self, name: str) -> np.ndarray:
        cached = self.images.pop(name, None)
        if cached is not None:
            self.images[name] = cached
            return cached
        raw = np.frombuffer(self.archive.read(name), np.uint8)
        image = self.clahe.apply(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)).astype(np.float32)
        self.images[name] = image
        if len(self.images) > CACHE_IMAGES:
            self.images.popitem(last=False)
        return image

    def nearest(self, estimate: np.ndarray, n: int) -> np.ndarray:
        return np.argsort(np.linalg.norm(self.ref_xy - estimate, axis=1))[:n]

    def image_motion(self) -> np.ndarray:
        """Median consecutive-frame image shift, computed/cached over every query frame."""
        if self.flow is not None:
            return self.flow
        if self.flow_cache.exists():
            try:
                cached = np.load(self.flow_cache, allow_pickle=False)
                if cached.shape == (len(self.query), 2):
                    self.flow = cached
                    print(f"flow cache: {self.flow_cache}", flush=True)
                    return cached
                print(f"ignoring wrong-shaped flow cache: {self.flow_cache}", flush=True)
            except (OSError, ValueError) as exc:
                print(f"ignoring unreadable flow cache ({exc}): {self.flow_cache}", flush=True)
        started = time.time()
        flow = np.zeros((len(self.query), 2))
        previous = None
        for k, name in enumerate(self.query.name):
            raw = np.frombuffer(
                self.archive.read(f"{self.prefix}/query_images/{name}"), np.uint8
            )
            frame = cv2.resize(
                cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE),
                (250, 250),
                interpolation=cv2.INTER_AREA,
            )
            if previous is not None:
                f = cv2.calcOpticalFlowFarneback(
                    previous, frame, None, 0.5, 4, 21, 3, 7, 1.5, 0
                )
                flow[k] = np.median(f[60:190, 60:190].reshape(-1, 2), axis=0) * 2
            previous = frame
            if (k + 1) % 1000 == 0 or k + 1 == len(self.query):
                print(
                    f"flow {self.prefix}: {k + 1}/{len(self.query)} frames "
                    f"({time.time() - started:.0f} s)",
                    flush=True,
                )
        self.flow_cache.parent.mkdir(parents=True, exist_ok=True)
        np.save(self.flow_cache, flow)
        self.flow = flow
        print(f"saved flow cache: {self.flow_cache}", flush=True)
        return flow



def template(frame: np.ndarray, zoom: float, angle: float) -> np.ndarray:
    """Original resize, rotation, and centered KEEP crop."""
    n = int(round(500 * zoom))
    resized = cv2.resize(frame, (n, n), interpolation=cv2.INTER_AREA)
    rotated = cv2.warpAffine(
        resized,
        cv2.getRotationMatrix2D((n / 2, n / 2), float(angle), 1.0),
        (n, n),
    )
    c = int(n * KEEP)
    o = (n - c) // 2
    return rotated[o : o + c, o : o + c]


def match_fix(
    dataset: AltoDataset,
    absolute_k: int,
    zooms: np.ndarray,
    angles: np.ndarray,
    candidates: np.ndarray,
) -> tuple[float, np.ndarray, float, float]:
    """Faithful matching loop factored for shared section-level calibration."""
    query_name = f"{dataset.prefix}/query_images/{dataset.query.name.iloc[absolute_k]}"
    frame = dataset.load_image(query_name)
    best = (-1.0, None, 0.0, 0.0)
    for zoom in zooms:
        for angle in angles:
            t = template(frame, float(zoom), float(angle))
            for ri in candidates:
                reference_name = (
                    f"{dataset.prefix}/reference_images/{dataset.reference.name.iloc[ri]}"
                )
                result = cv2.matchTemplate(
                    dataset.load_image(reference_name), t, cv2.TM_CCOEFF_NORMED
                )
                _, score, _, loc = cv2.minMaxLoc(result)
                if score > best[0]:
                    centre = np.array(loc) + t.shape[0] / 2
                    offset = np.array(
                        [(centre[0] - 250) * REF_MPP, -(centre[1] - 250) * REF_MPP]
                    )
                    best = (
                        score,
                        dataset.ref_xy[ri] + offset,
                        float(zoom),
                        float(angle),
                    )
    return best


def prepare_range(
    dataset: AltoDataset, frame_range: tuple[int, int]
) -> PreparedRange:
    """Calibrate a [i0, i1) section; travelled distance is reset to zero at i0."""
    i0, i1 = frame_range
    if dataset.flow is None:
        dataset.image_motion()
    if not (0 <= i0 < i1 <= len(dataset.query)):
        raise ValueError(f"invalid frame range [{i0}, {i1}) for {len(dataset.query)} frames")
    truth = dataset.truth[i0:i1]
    travelled = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(truth, axis=0), axis=1))]
    jam = int(np.searchsorted(travelled, JAM_AT_M))
    if jam < 1 or jam >= len(truth):
        raise ValueError(f"range [{i0}, {i1}) does not contain a 300 m GNSS phase")
    steps = np.diff(truth, axis=0)
    A0, *_ = np.linalg.lstsq(
        dataset.flow[i0 + 1 : i0 + jam + 1], steps[:jam], rcond=None
    )
    calib_frames = (jam // 3, 2 * jam // 3, jam)
    calibration = [
        match_fix(
            dataset,
            i0 + k,
            CALIB_ZOOMS,
            CALIB_ANGLES,
            dataset.nearest(truth[k], CALIB_NEAREST),
        )
        for k in calib_frames
    ]
    zoom0 = float(np.median([c[2] for c in calibration]))
    angle0 = float(np.median([c[3] for c in calibration]))
    offset0 = np.median(
        [c[1] - truth[k] for c, k in zip(calibration, calib_frames)], axis=0
    )
    print(
        f"{dataset.prefix} [{i0},{i1}): jam after {travelled[jam]:.0f} m "
        f"(local frame {jam}, global frame {i0 + jam}); calibration "
        f"zoom {zoom0:.2f}, rotation {angle0:.0f} deg, "
        f"offset {offset0.round(1)} m",
        flush=True,
    )
    return PreparedRange(
        i0, i1, truth, travelled, jam, A0, zoom0, angle0, offset0, calibration
    )


def run_config(
    dataset: AltoDataset, section: PreparedRange, config: RunConfig
) -> RunResult:
    """Run one frozen policy on a prepared range, post-jam truth used only for scoring."""
    estimate = section.truth[section.jam].copy()
    path = [estimate.copy()]
    variance = 3.0**2
    since_fix = 0.0
    since_try = 0.0
    zoom = section.zoom0
    scale = 1.0
    log: list[tuple[int, float, float, float]] = []
    for k in range(section.jam + 1, len(section.truth)):
        step = dataset.flow[section.i0 + k] @ section.A0 * scale
        estimate = estimate + step
        since_fix += np.linalg.norm(step)
        since_try += np.linalg.norm(step)
        if config.fix_every_m and since_try >= config.fix_every_m:
            since_try = 0.0
            predicted_var = variance + (DRIFT_RATE * since_fix) ** 2
            zooms = np.clip(zoom + np.arange(-0.10, 0.11, 0.05), 0.5, 1.1)
            candidates = dataset.nearest(estimate, CALIB_NEAREST)
            if config.sized_search:
                radius = max(60.0, 3 * np.sqrt(predicted_var))
                inside = np.where(
                    np.linalg.norm(dataset.ref_xy - estimate, axis=1) <= radius
                )[0]
                candidates = inside if len(inside) >= CALIB_NEAREST else candidates
                if since_fix > 400:
                    zooms = np.arange(0.60, 1.101, 0.05)
            score, position, new_zoom, _ = match_fix(
                dataset,
                section.i0 + k,
                zooms,
                np.array([section.angle0 - 5, section.angle0, section.angle0 + 5]),
                candidates,
            )
            position = position - section.offset0
            agrees = np.linalg.norm(position - estimate) <= 3 * np.sqrt(
                predicted_var + FIX_SIGMA**2
            )
            use = bool(agrees and score >= config.min_score)
            log.append(
                (
                    section.i0 + k,
                    score,
                    np.linalg.norm(position - section.truth[k]),
                    float(use),
                )
            )
            if use:
                gain = predicted_var / (predicted_var + FIX_SIGMA**2)
                estimate = estimate + gain * (position - estimate)
                variance = (1 - gain) * predicted_var
                scale, zoom, since_fix = new_zoom / section.zoom0, new_zoom, 0.0
        path.append(estimate.copy())
    path_array = np.asarray(path)
    errors = np.linalg.norm(path_array - section.truth[section.jam :], axis=1)
    return RunResult(path_array, errors, np.asarray(log, dtype=float).reshape(-1, 4))


def error_counts(result: RunResult) -> tuple[int, int, int, int]:
    if not len(result.log):
        return 0, 0, 0, 0
    accepted = result.log[:, 3] == 1
    used = int(accepted.sum())
    rejected = int((~accepted).sum())
    wrong = int((result.log[accepted, 2] > 50).sum())
    correct_rejected = int((result.log[~accepted, 2] <= 30).sum())
    return used, rejected, wrong, correct_rejected


def print_report(label: str, result: RunResult) -> None:
    errors = result.error
    line = (
        f"{label:44s} median {np.median(errors):6.1f} m, "
        f"90% below {np.percentile(errors, 90):6.1f}, "
        f"worst {errors.max():6.1f}, end {errors[-1]:6.1f}"
    )
    if len(result.log):
        used, rejected, wrong, correct_rejected = error_counts(result)
        line += (
            f" | fixes used {used:2d}, rejected {rejected:2d}, "
            f"used but wrong by over 50 m: {wrong}, "
            f"rejected although within 30 m: {correct_rejected}"
        )
    print(line, flush=True)


def val_configurations() -> list[RunConfig]:
    return [
        RunConfig("camera only", None),
        *[RunConfig(f"fixed {m}", float(m)) for m in (100, 200, 300, 400, 500, 600, 800, 1000)],
        RunConfig("fixed checked 300", 300.0, min_score=MIN_SCORE),
        RunConfig("fixed checked 1000", 1000.0, min_score=MIN_SCORE),
        RunConfig("sized 300", 300.0, sized_search=True, min_score=MIN_SCORE),
        RunConfig("sized 1000", 1000.0, sized_search=True, min_score=MIN_SCORE),
        RunConfig("sized 2000", 2000.0, sized_search=True, min_score=MIN_SCORE),
    ]


VAL_EXPECTED = {
    "camera only": (472, 657, 608, None, None, None),
    "fixed 100": (26, 50, 26, 39, 0, 0),
    "fixed 200": (30, 71, 45, 19, 0, 0),
    "fixed 300": (31, 83, 16, 13, 0, 0),
    "fixed 400": (285, 898, 898, 7, 0, 5),
    "fixed 500": (401, 1247, 1247, 5, 1, 4),
    "fixed 600": (609, 1334, 1334, 4, 0, 4),
    "fixed 800": (661, 949, 949, 4, 0, 4),
    "fixed 1000": (426, 805, 805, 3, 0, 3),
    "fixed checked 300": (32, 83, 38, 12, 1, 0),
    "fixed checked 1000": (472, 657, 608, 0, 3, 0),
    "sized 300": (31, 73, 38, 12, 1, 0),
    "sized 1000": (56, 278, 10, 4, 0, 0),
    "sized 2000": (116, 578, 197, 1, 0, 0),
}


def _val_row(config: RunConfig, result: RunResult, runtime_s: float) -> dict:
    used, rejected, wrong, correct_rejected = error_counts(result)
    has_fixes = config.fix_every_m is not None
    return {
        "config": config.name,
        "median": float(np.median(result.error)),
        "worst": float(result.error.max()),
        "end": float(result.error[-1]),
        "used": used if has_fixes else np.nan,
        "rejected": rejected if has_fixes else np.nan,
        "wrong_gt50": wrong if has_fixes else np.nan,
        "rejected_correct_le30": correct_rejected if has_fixes else np.nan,
        "runtime_s": runtime_s,
    }


def run_val_reproduction() -> tuple[bool, float]:
    phase_started = time.time()
    dataset = AltoDataset(VAL_ZIP, "Val")
    dataset.image_motion()
    section = prepare_range(dataset, (0, len(dataset.query)))
    print(
        "fixed search (7 nearest reference images), no score check:", flush=True
    )
    configs = val_configurations()
    results: dict[str, RunResult] = {}
    rows = []
    for index, config in enumerate(configs):
        if index == 0:
            print("camera only, no fixes:", flush=True)
        elif index == 9:
            print(f"fixed search, fix used only if its score is at least {MIN_SCORE}:", flush=True)
        elif index == 11:
            print(
                f"search sized by the uncertainty, fix used only if its score is at least {MIN_SCORE}:",
                flush=True,
            )
        label = config.name
        started = time.time()
        result = run_config(dataset, section, config)
        runtime_s = time.time() - started
        results[label] = result
        shown_label = {
            "camera only": "camera only, no fixes",
            "fixed checked 300": "  fix every 300 m",
            "fixed checked 1000": "  fix every 1000 m",
            "sized 300": "  fix every 300 m",
            "sized 1000": "  fix every 1000 m",
            "sized 2000": "  fix every 2000 m",
        }.get(label, f"  fix every {label.removeprefix('fixed ')} m")
        print_report(shown_label, result)
        rows.append(_val_row(config, result, runtime_s))
    distances = section.travelled[section.jam :] - section.travelled[section.jam]
    camera_error = results["camera only"].error
    at = lambda m: camera_error[np.searchsorted(distances, m)]
    print(
        "  camera only, error at 300 / 500 / 1000 / 2000 m after the jam: "
        f"{at(300):.0f} / {at(500):.0f} / {at(1000):.0f} / {at(2000):.0f} m",
        flush=True,
    )
    frame = pd.DataFrame(rows)
    pass_rows = []
    for row in rows:
        expected = VAL_EXPECTED[row["config"]]
        metric_match = all(
            abs(row[key] - target) <= 1.0
            for key, target in zip(("median", "worst", "end"), expected[:3])
        )
        if expected[3] is None:
            count_match = np.isnan(row["used"]) and np.isnan(row["rejected"])
        else:
            count_match = tuple(
                int(row[key]) for key in ("used", "rejected", "wrong_gt50")
            ) == expected[3:]
        passed = bool(metric_match and count_match)
        pass_rows.append(passed)
        print(
            f"Val compare {row['config']}: "
            f"median/worst/end {row['median']:.1f}/{row['worst']:.1f}/{row['end']:.1f} m; "
            f"counts {'match' if count_match else 'MISMATCH'}; "
            f"{'PASS' if passed else 'FAIL'}",
            flush=True,
        )
    frame["reproduction_match"] = pass_rows
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    frame.to_csv(OUT_DIR / "val_reproduction.csv", index=False)
    print(f"Val reproduction CSV: {OUT_DIR / 'val_reproduction.csv'}", flush=True)
    print(f"Val phase runtime: {time.time() - phase_started:.1f} s", flush=True)
    dataset.close()
    return bool(all(pass_rows)), time.time() - phase_started


def existing_val_pass() -> bool:
    path = OUT_DIR / "val_reproduction.csv"
    if not path.exists():
        print("held-out run blocked: Val reproduction CSV is missing", flush=True)
        return False
    data = pd.read_csv(path)
    passed = len(data) == 14 and bool(data.reproduction_match.all())
    if not passed:
        print("held-out run blocked: saved Val reproduction did not pass all 14 rows", flush=True)
    else:
        print("Val reproduction gate: prior 14/14 rows passed", flush=True)
    return passed


def leak_check() -> dict:
    val = AltoDataset(VAL_ZIP, "Val")
    train = AltoDataset(TRAIN_ZIP, "Train")
    val_query_tree = cKDTree(val.truth)
    val_reference_tree = cKDTree(
        val.reference_all[["easting", "northing"]].to_numpy()
    )
    query_distance = val_query_tree.query(train.truth, k=1, workers=-1)[0]
    reference_distance = val_reference_tree.query(train.truth, k=1, workers=-1)[0]
    nearest_any = np.minimum(query_distance, reference_distance)
    summary = {
        "train_frames": len(train.truth),
        "val_query_frames": len(val.truth),
        "val_reference_rows": len(val.reference_all),
        "min_to_val_query_m": float(query_distance.min()),
        "min_to_val_reference_m": float(reference_distance.min()),
        "min_any_m": float(nearest_any.min()),
        "train_frames_with_any_under_500m": int((nearest_any < 500).sum()),
    }
    pd.DataFrame([summary]).to_csv(OUT_DIR / "leak_check.csv", index=False)
    print(
        "Leak check (UTM easting/northing): "
        f"min Train→Val query {summary['min_to_val_query_m']:.2f} m; "
        f"min Train→any Val reference {summary['min_to_val_reference_m']:.2f} m; "
        f"combined minimum {summary['min_any_m']:.2f} m; "
        f"frames <500 m {summary['train_frames_with_any_under_500m']}/"
        f"{summary['train_frames']} "
        f"({'OVERLAP' if summary['train_frames_with_any_under_500m'] else 'no overlap'}).",
        flush=True,
    )
    val.close()
    train.close()
    return summary


def section_ranges(travelled: np.ndarray) -> list[tuple[int, int]]:
    """Split by consecutive 4.6 km traveled-distance thresholds."""
    n = len(travelled)
    total = float(travelled[-1])
    boundaries = [0]
    target = 4600.0
    while target < total:
        index = int(np.searchsorted(travelled, target))
        if index <= boundaries[-1] or index >= n:
            break
        boundaries.append(index)
        target += 4600.0
    if boundaries[-1] != n:
        boundaries.append(n)
    if len(boundaries) > 2 and total - travelled[boundaries[-2]] <= 2000.0:
        boundaries.pop(-2)
    return list(zip(boundaries[:-1], boundaries[1:]))


def train_section_configurations() -> list[RunConfig]:
    return [
        RunConfig("none", None),
        RunConfig("nearest_100", 100.0),
        RunConfig("nearest_200", 200.0),
        RunConfig("nearest_300", 300.0),
        RunConfig("nearest_gate_300", 300.0, min_score=MIN_SCORE),
        RunConfig("nearest_gate_1000", 1000.0, min_score=MIN_SCORE),
        RunConfig("sized_gate_300", 300.0, sized_search=True, min_score=MIN_SCORE),
        RunConfig("sized_gate_1000", 1000.0, sized_search=True, min_score=MIN_SCORE),
        RunConfig("sized_gate_2000", 2000.0, sized_search=True, min_score=MIN_SCORE),
    ]


def full_configurations() -> list[RunConfig]:
    return [
        RunConfig("none", None),
        RunConfig("sized_gate_300", 300.0, sized_search=True, min_score=MIN_SCORE),
        RunConfig("sized_gate_1000", 1000.0, sized_search=True, min_score=MIN_SCORE),
        RunConfig("sized_gate_2000", 2000.0, sized_search=True, min_score=MIN_SCORE),
    ]


def append_row(writer: csv.DictWriter, handle, row: dict) -> None:
    writer.writerow(row)
    handle.flush()


def calibration_row(section_name: str, section: PreparedRange, km: float) -> dict:
    return {
        "section": section_name,
        "i0": section.i0,
        "i1": section.i1,
        "km": km,
        "jam_local_frame": section.jam,
        "jam_global_frame": section.i0 + section.jam,
        "jam_m": float(section.travelled[section.jam]),
        "zoom0": section.zoom0,
        "angle0_deg": section.angle0,
        "offset_e_m": float(section.offset0[0]),
        "offset_n_m": float(section.offset0[1]),
    }


def summarize_fix_scores(score_rows: list[dict]) -> None:
    score_frame = pd.DataFrame(score_rows)
    if score_frame.empty:
        pd.DataFrame(
            columns=["section", "config", "accepted", "n", "min", "q25", "median", "q75", "max"]
        ).to_csv(OUT_DIR / "fix_score_summary.csv", index=False)
        return
    summary = (
        score_frame.groupby(["section", "config", "accepted"], as_index=False)
        .score.agg(
            n="count",
            min="min",
            q25=lambda values: values.quantile(0.25),
            median="median",
            q75=lambda values: values.quantile(0.75),
            max="max",
        )
    )
    summary.to_csv(OUT_DIR / "fix_score_summary.csv", index=False)
    print("Fix-score distribution pooled by section and decision:", flush=True)
    for section_name, group in score_frame.groupby("section", sort=False):
        print(f"  {section_name}:", end="", flush=True)
        for accepted, label in ((True, "accepted"), (False, "rejected")):
            scores = group.loc[group.accepted == accepted, "score"]
            if len(scores):
                print(
                    f" {label} n={len(scores)} median={scores.median():.3f} "
                    f"IQR=[{scores.quantile(.25):.3f},{scores.quantile(.75):.3f}]",
                    end=";",
                    flush=True,
                )
            else:
                print(f" {label} n=0;", end="", flush=True)
        print(flush=True)
    print(f"Detailed score distributions: {OUT_DIR / 'fix_score_summary.csv'}", flush=True)


def run_train_heldout() -> tuple[dict, float]:
    phase_started = time.time()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    leak = leak_check()
    dataset = AltoDataset(TRAIN_ZIP, "Train")
    dataset.image_motion()
    full_travelled = np.r_[
        0.0,
        np.cumsum(np.linalg.norm(np.diff(dataset.truth, axis=0), axis=1)),
    ]
    ranges = section_ranges(full_travelled)
    if len(ranges) != 8:
        dataset.close()
        raise RuntimeError(f"protocol expected 8 sections; generated {len(ranges)}")
    print(
        f"Train frames={len(dataset.query)}, traveled={full_travelled[-1] / 1000:.2f} km, "
        f"sections={len(ranges)}",
        flush=True,
    )
    prepared_sections: list[tuple[str, PreparedRange, float]] = []
    calibrations = []
    for number, (i0, i1) in enumerate(ranges, 1):
        km = float((full_travelled[i1 - 1] - full_travelled[i0]) / 1000)
        section = prepare_range(dataset, (i0, i1))
        name = f"section_{number}"
        prepared_sections.append((name, section, km))
        calibrations.append(calibration_row(name, section, km))
        print(
            f"  {name}: {i0}:{i1}, {km:.2f} km; zoom0 {section.zoom0:.2f} "
            f"({section.zoom0 - .85:+.2f} vs Val 0.85), angle0 {section.angle0:.0f} deg "
            f"({section.angle0 - 10:+.0f} vs Val 10)",
            flush=True,
        )
    full_km = float(full_travelled[-1] / 1000)
    full_section = prepare_range(dataset, (0, len(dataset.query)))
    calibrations.append(calibration_row("full", full_section, full_km))
    pd.DataFrame(calibrations).to_csv(OUT_DIR / "calibration.csv", index=False)

    result_path = OUT_DIR / "results.csv"
    score_path = OUT_DIR / "fix_scores.csv"
    result_columns = [
        "section", "i0", "i1", "km", "config", "median", "worst", "end",
        "used", "rejected", "wrong_gt50", "runtime_s",
    ]
    score_columns = ["section", "config", "frame", "score", "fix_error_m", "accepted"]
    score_rows: list[dict] = []
    with result_path.open("w", newline="") as result_handle, score_path.open(
        "w", newline=""
    ) as score_handle:
        result_writer = csv.DictWriter(result_handle, fieldnames=result_columns)
        score_writer = csv.DictWriter(score_handle, fieldnames=score_columns)
        result_writer.writeheader()
        score_writer.writeheader()
        for section_name, section, km in prepared_sections:
            for config in train_section_configurations():
                started = time.time()
                result = run_config(dataset, section, config)
                runtime_s = time.time() - started
                used, rejected, wrong, _ = error_counts(result)
                row = {
                    "section": section_name,
                    "i0": section.i0,
                    "i1": section.i1,
                    "km": km,
                    "config": config.name,
                    "median": float(np.median(result.error)),
                    "worst": float(result.error.max()),
                    "end": float(result.error[-1]),
                    "used": used,
                    "rejected": rejected,
                    "wrong_gt50": wrong,
                    "runtime_s": runtime_s,
                }
                append_row(result_writer, result_handle, row)
                print(
                    f"{section_name} {config.name}: median={row['median']:.1f} m, "
                    f"worst={row['worst']:.1f} m, end={row['end']:.1f} m; "
                    f"fixes {used} used/{rejected} rejected/{wrong} wrong>50; "
                    f"{runtime_s:.1f} s",
                    flush=True,
                )
                for frame, score, fix_error, accepted in result.log:
                    score_row = {
                        "section": section_name,
                        "config": config.name,
                        "frame": int(frame),
                        "score": float(score),
                        "fix_error_m": float(fix_error),
                        "accepted": bool(accepted),
                    }
                    append_row(score_writer, score_handle, score_row)
                    score_rows.append(score_row)
        for config in full_configurations():
            started = time.time()
            result = run_config(dataset, full_section, config)
            runtime_s = time.time() - started
            used, rejected, wrong, _ = error_counts(result)
            row = {
                "section": "full",
                "i0": 0,
                "i1": len(dataset.query),
                "km": full_km,
                "config": config.name,
                "median": float(np.median(result.error)),
                "worst": float(result.error.max()),
                "end": float(result.error[-1]),
                "used": used,
                "rejected": rejected,
                "wrong_gt50": wrong,
                "runtime_s": runtime_s,
            }
            append_row(result_writer, result_handle, row)
            print(
                f"full {config.name}: median={row['median']:.1f} m, "
                f"worst={row['worst']:.1f} m, end={row['end']:.1f} m; "
                f"fixes {used} used/{rejected} rejected/{wrong} wrong>50; "
                f"{runtime_s:.1f} s",
                flush=True,
            )
            for frame, score, fix_error, accepted in result.log:
                score_row = {
                    "section": "full",
                    "config": config.name,
                    "frame": int(frame),
                    "score": float(score),
                    "fix_error_m": float(fix_error),
                    "accepted": bool(accepted),
                }
                append_row(score_writer, score_handle, score_row)
                score_rows.append(score_row)
    summarize_fix_scores(score_rows)
    result_frame = pd.read_csv(result_path)
    section_frame = result_frame[result_frame.section != "full"]
    summary_rows = []
    for config_name, group in section_frame.groupby("config", sort=False):
        summary_rows.append(
            {
                "config": config_name,
                "median": float(group["median"].median()),
                "min": float(group["median"].min()),
                "max": float(group["median"].max()),
                "median_worst": float(group["worst"].median()),
                "median_end": float(group["end"].median()),
                "total_used": int(group["used"].sum()),
                "total_rejected": int(group["rejected"].sum()),
                "total_wrong_gt50": int(group["wrong_gt50"].sum()),
            }
        )
    summary_frame = pd.DataFrame(summary_rows)
    summary_frame.to_csv(OUT_DIR / "summary.csv", index=False)
    print("Section summary (median error across sections; min/max are section medians):", flush=True)
    print(summary_frame.to_string(index=False, float_format=lambda value: f"{value:.1f}"), flush=True)
    summarize_predictions(section_frame, score_rows)
    make_sections_figure(section_frame)
    print(f"Results: {result_path}", flush=True)
    print(f"Fix scores: {score_path}", flush=True)
    print(f"Calibration: {OUT_DIR / 'calibration.csv'}", flush=True)
    print(f"Figure: {OUT_DIR / 'sections.png'}", flush=True)
    runtime_s = time.time() - phase_started
    print(f"Train phase total runtime: {runtime_s:.1f} s", flush=True)
    dataset.close()
    return leak, runtime_s


def summarize_predictions(section_frame: pd.DataFrame, score_rows: list[dict]) -> None:
    medians = section_frame.pivot(index="section", columns="config", values="median")
    p1_counts = {
        config: int((medians[config] < 60).sum())
        for config in ("nearest_100", "nearest_200", "nearest_300")
    }
    p1 = all(count >= 6 for count in p1_counts.values())
    sized_1000 = section_frame[section_frame.config == "sized_gate_1000"]
    p2_count = int((sized_1000.wrong_gt50 == 0).sum())
    p2 = p2_count >= 7
    camera = medians["none"]
    p3_count = int(((camera >= 472 / 2) & (camera <= 472 * 2)).sum())
    p3 = p3_count >= 5
    gated_round2 = [
        "nearest_gate_300", "nearest_gate_1000", "sized_gate_300",
        "sized_gate_1000", "sized_gate_2000",
    ]
    details = pd.DataFrame(score_rows)
    per_section_correct_rejects = []
    for section_name in medians.index:
        selected = details[
            (details.section == section_name)
            & details.config.isin(gated_round2)
            & (~details.accepted)
            & (details.fix_error_m <= 30)
        ]
        per_section_correct_rejects.append(int(len(selected)))
    gated_val = [
        "fixed checked 300", "fixed checked 1000", "sized 300",
        "sized 1000", "sized 2000",
    ]
    val_data = pd.read_csv(OUT_DIR / "val_reproduction.csv").set_index("config")
    val_correct_rejects = int(
        val_data.loc[gated_val, "rejected_correct_le30"].fillna(0).sum()
    )
    p4_median = float(np.median(per_section_correct_rejects))
    p4 = p4_median > val_correct_rejects
    print("Pre-registered predictions:", flush=True)
    print(
        f"P1 {'TRUE' if p1 else 'FALSE'} — sections with median <60 m: "
        + ", ".join(f"{name} {count}/8" for name, count in p1_counts.items())
        + "; all three frequencies must reach 6/8.",
        flush=True,
    )
    print(
        f"P2 {'TRUE' if p2 else 'FALSE'} — sized+gate 1000 m has zero wrong (>50 m) "
        f"used fixes in {p2_count}/8 sections (required 7/8).",
        flush=True,
    )
    print(
        f"P3 {'TRUE' if p3 else 'FALSE'} — camera-only section medians within "
        f"236–944 m: {p3_count}/8 (required at least 5/8).",
        flush=True,
    )
    print(
        f"P4 {'TRUE' if p4 else 'FALSE'} — correct rejected fixes (<=30 m), pooled over "
        f"the five gated policies: Val {val_correct_rejects}; Round 2 median per "
        f"4.6-km section {p4_median:.1f}, range "
        f"{min(per_section_correct_rejects)}–{max(per_section_correct_rejects)}. "
        "Compared like-length section totals to the single Val trajectory.",
        flush=True,
    )
    print(f"P4 Round 2 per-section counts: {per_section_correct_rejects}", flush=True)


def make_sections_figure(section_frame: pd.DataFrame) -> None:
    val_values = {
        "none": 472,
        "nearest_100": 26,
        "nearest_200": 30,
        "nearest_300": 31,
        "nearest_gate_300": 32,
        "nearest_gate_1000": 472,
        "sized_gate_300": 31,
        "sized_gate_1000": 56,
        "sized_gate_2000": 116,
    }
    config_order = list(val_values)
    section_names = list(dict.fromkeys(section_frame.section.tolist()))
    fig, ax = plt.subplots(figsize=(12, 6.5))
    x = np.arange(len(config_order))
    offsets = np.linspace(-0.18, 0.18, len(section_names))
    colors = plt.get_cmap("tab10")(np.arange(len(section_names)))
    for section_index, section_name in enumerate(section_names):
        group = section_frame[section_frame.section == section_name].set_index("config")
        values = [group.loc[config, "median"] for config in config_order]
        ax.scatter(
            x + offsets[section_index], values, s=26, color=colors[section_index],
            label=section_name.replace("section_", "S"), zorder=3,
        )
    for position, config in zip(x, config_order):
        ax.hlines(
            val_values[config], position - 0.30, position + 0.30,
            color="black", linewidth=2, zorder=2,
        )
    ax.set_yscale("log")
    ax.set_ylabel("Section median position error (m, log scale)")
    ax.set_xticks(x, config_order, rotation=30, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(ncol=4, fontsize=8, title="Round 2 section")
    ax.set_title("Round 2 held-out sections; black bars show Val median")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "sections.png", dpi=140)
    plt.close(fig)

def finalize_existing_outputs() -> None:
    """Build reports from completed result/score CSVs without rerunning simulations."""
    result_path = OUT_DIR / "results.csv"
    score_path = OUT_DIR / "fix_scores.csv"
    result_frame = pd.read_csv(result_path)
    if len(result_frame) != 76:
        raise RuntimeError(f"expected 76 completed results; found {len(result_frame)}")
    section_frame = result_frame[result_frame.section != "full"]
    score_frame = pd.read_csv(score_path)
    score_rows = score_frame.to_dict(orient="records")
    summarize_fix_scores(score_rows)
    summary_rows = []
    for config_name, group in section_frame.groupby("config", sort=False):
        summary_rows.append(
            {
                "config": config_name,
                "median": float(group["median"].median()),
                "min": float(group["median"].min()),
                "max": float(group["median"].max()),
                "median_worst": float(group["worst"].median()),
                "median_end": float(group["end"].median()),
                "total_used": int(group["used"].sum()),
                "total_rejected": int(group["rejected"].sum()),
                "total_wrong_gt50": int(group["wrong_gt50"].sum()),
            }
        )
    summary_frame = pd.DataFrame(summary_rows)
    summary_frame.to_csv(OUT_DIR / "summary.csv", index=False)
    print("Section summary (median error across sections; min/max are section medians):", flush=True)
    print(summary_frame.to_string(index=False, float_format=lambda value: f"{value:.1f}"), flush=True)
    summarize_predictions(section_frame, score_rows)
    make_sections_figure(section_frame)
    print(f"Results: {result_path}", flush=True)
    print(f"Fix scores: {score_path}", flush=True)
    print(f"Calibration: {OUT_DIR / 'calibration.csv'}", flush=True)
    print(f"Figure: {OUT_DIR / 'sections.png'}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("val", "train", "all", "finalize"), default="all")
    args = parser.parse_args()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    overall_started = time.time()
    if args.phase == "finalize":
        if not existing_val_pass():
            raise SystemExit(2)
        finalize_existing_outputs()
        return
    if args.phase in ("val", "all"):
        reproduced, _ = run_val_reproduction()
        if not reproduced:
            print("STOP: Val reproduction failed; Round 2 held-out runs were not started.", flush=True)
            raise SystemExit(2)
        print("Val reproduction PASS: all 14 configurations match findings.md.", flush=True)
    if args.phase == "train" and not existing_val_pass():
        raise SystemExit(2)
    if args.phase in ("train", "all"):
        leak, train_runtime = run_train_heldout()
        print(
            f"Completed requested phases in this process: {time.time() - overall_started:.1f} s "
            f"(Train phase {train_runtime:.1f} s).",
            flush=True,
        )
        if leak["train_frames_with_any_under_500m"]:
            print("LEAK CHECK: at least one Train frame is within 500 m of Val data.", flush=True)



if __name__ == "__main__":
    main()

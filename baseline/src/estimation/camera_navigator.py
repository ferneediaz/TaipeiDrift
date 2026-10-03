"""Camera navigator: GNSS, then jamming, then the camera alone.

1. While GNSS works: learn how image shift maps to ground motion, and the zoom, the rotation
   and the offset of the camera against the reference images (``calibrate``).
2. After the jam: carry the position forward with the image shift alone (dead reckoning).
3. Every so many metres: match the camera frame against reference images near the estimate.
   The fix is blended into the estimate only if it passes the check (``navigate``).

What it reads: the camera frames, the map (reference images with coordinates, or one map of the
whole area) and the true position up to the jam. It reads no altitude, no orientation and no
camera calibration. The true position after the jam is never read here;
``src.evaluation.navigation_metrics`` uses it to measure the error.

Three ways to search for a fix:

- ``nearest`` and ``sized``: the reference images of the dataset closest to the estimate. On ALTO
  these images are centred on the true path, so this search knows where the path runs.
- ``area``: one map of the whole area, searched in a circle around the estimate. This search knows
  nothing about the true path; it is the one to report.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from src.data.camera_flight import CameraFlight, prepare
from src.estimation.image_motion import shifts_for_flight
from src.data.ground_map import GroundMap
from src.estimation.map_matching import KEEP, Match, match, search_area
from src.estimation.navigator_core import (
    FRAMES_DISAGREE,
    OFF_MAP,
    agreeing_fixes,
    allowed_distance,
    blend,
    fit_motion_matrix,
    fix_decision,
    predicted_variance,
    status,
)


@dataclass
class NavigatorConfig:
    """Settings of one run. The defaults are those of ``experiments/h_alto_end_to_end.py``."""

    jam_after_m: float = 300.0  # GNSS is lost after this distance
    fix_every_m: float | None = 300.0  # estimated distance between position fixes; None: no fixes
    search: str = "nearest"  # "nearest": a fixed number of reference images; "sized": grows with the uncertainty;
    # "area": one map of the whole area, in a circle around the estimate that grows with the uncertainty
    nearest_images: int = 7
    min_search_radius_m: float = 60.0  # sized and area search: within max(this, 3 sigma)
    calibration_radius_m: float = 60.0  # area search before the jam: around the GNSS position
    min_score: float = 0.0  # a fix with a lower matching score is not used
    fix_sigma_m: float = 15.0  # accuracy of one fix
    drift_rate: float = 0.10  # dead reckoning error as a share of the distance since the last fix
    gate_sigmas: float = 3.0  # a fix further from the estimate than this many sigmas is not used
    start_sigma_m: float = 3.0  # uncertainty at the moment of the jam
    keep: float = KEEP
    zoom_step: float = 0.05
    zoom_reach: float = 0.10  # zooms tried around the last zoom
    zoom_limits: tuple[float, float] = (0.5, 1.1)
    angle_reach_deg: float = 5.0  # angles tried: the learned angle and this much to either side
    wide_zoom_after_m: float = 400.0  # sized search: after this distance without a fix, try all zooms
    wide_zooms: tuple[float, float] = (0.60, 1.10)
    calibration_zooms: tuple[float, float] = (0.60, 1.00)
    calibration_angles_deg: tuple[float, float, float] = (-10.0, 35.0, 5.0)  # from, to, step
    degraded_above_m: float = 30.0  # status thresholds on sigma
    lost_above_m: float = 100.0
    agreement_frames: int = 1  # frames matched for one fix; 1 switches the agreement check off
    agreement_spacing: int = 5  # frames between them, about 14 m on ALTO
    agreement_radius_m: float = 10.0  # fixes agree if they land this close after removing the motion in between
    agreement_needed: int = 2  # this many frames have to agree


@dataclass
class Calibration:
    """What the navigator learned while GNSS still worked."""

    jam_index: int  # last frame with GNSS
    motion_matrix: np.ndarray  # (2, 2); image shift in pixels @ matrix = ground step (north, east) in m
    zoom: float
    angle: float  # degrees
    fix_offset: np.ndarray  # (2,) north, east in m; a fix lands this far from the true position


@dataclass
class FixRecord:
    """One attempted position fix."""

    frame: int
    score: float
    position: np.ndarray  # (2,) north, east in m, after removing the learned offset
    zoom: float
    angle: float
    reference_index: int
    candidates: int  # number of reference images searched
    distance: float  # m between the fix and the estimate before blending
    allowed: float  # largest distance at which the fix would be believed, m
    used: bool
    reason: str  # OK, LOW_SCORE, DISAGREES_WITH_ESTIMATE, FRAMES_DISAGREE or OFF_MAP
    frames_agreeing: int = 1  # with the agreement check: how many of the matched frames agreed
    search_radius_m: float = 0.0  # sized and area search: radius of the search around the estimate


@dataclass
class NavigatorResult:
    """Estimated path from the jam to the end of the flight."""

    start_index: int  # index into the flight of the first row, the frame of the jam
    position: np.ndarray  # (M, 2) north, east in m
    sigma: np.ndarray  # (M,) uncertainty the navigator states for its estimate, m
    status: list[str]  # (M,) TRACKING, DEGRADED or LOST
    fixes: list[FixRecord]
    calibration: Calibration
    config: NavigatorConfig = field(repr=False)


def jam_index(flight: CameraFlight, jam_after_m: float) -> int:
    """First frame at which the true distance flown reaches ``jam_after_m``."""
    return int(np.searchsorted(flight.travelled, jam_after_m))


def calibrate(flight: CameraFlight, shifts: np.ndarray, cfg: NavigatorConfig) -> Calibration:
    """Learn the motion matrix, zoom, rotation and fix offset from the stretch with GNSS."""
    jam = jam_index(flight, cfg.jam_after_m)
    if jam < 3 or jam >= len(flight):
        raise ValueError(f"the jam falls on frame {jam} of {len(flight)}; the flight needs GNSS before it and frames after it")
    truth = flight.position_gt[: jam + 1]
    matrix = fit_motion_matrix(shifts[1 : jam + 1], np.diff(truth, axis=0))

    zooms = np.arange(cfg.calibration_zooms[0], cfg.calibration_zooms[1] + 0.01, cfg.zoom_step)
    start, stop, step = cfg.calibration_angles_deg
    angles = np.arange(start, stop + 1.0, step)
    frames = (jam // 3, 2 * jam // 3, jam)
    if cfg.search == "area":
        ground = _ground_map(flight)
        found = [search_area(prepare(flight.frame(k)), ground, truth[k], cfg.calibration_radius_m, zooms, angles, cfg.keep) for k in frames]
        if any(f is None for f in found):
            raise ValueError("the map holds no imagery around a frame before the jam")
        fixes: list[Match] = found  # type: ignore[assignment]
    else:
        fixes = [
            match(prepare(flight.frame(k)), flight.reference, flight.reference.nearest(truth[k], cfg.nearest_images), zooms, angles, cfg.keep)
            for k in frames
        ]
    return Calibration(
        jam_index=jam,
        motion_matrix=matrix,
        zoom=float(np.median([f.zoom for f in fixes])),
        angle=float(np.median([f.angle for f in fixes])),
        fix_offset=np.median([f.position - truth[k] for f, k in zip(fixes, frames)], axis=0),
    )


def _ground_map(flight: CameraFlight) -> GroundMap:
    if flight.ground_map is None:
        raise ValueError(f"search 'area' needs a map of the whole area; flight {flight.name} has none")
    return flight.ground_map


def navigate(flight: CameraFlight, shifts: np.ndarray, calibration: Calibration, cfg: NavigatorConfig) -> NavigatorResult:
    """Estimate the path after the jam from image shift and position fixes."""
    if cfg.search not in ("nearest", "sized", "area"):
        raise ValueError(f"unknown search {cfg.search!r}")
    jam = calibration.jam_index
    reference = flight.reference
    ground = _ground_map(flight) if cfg.search == "area" else None
    fix_variance = cfg.fix_sigma_m**2

    estimate = flight.position_gt[jam].copy()  # the last position GNSS gave
    variance = cfg.start_sigma_m**2
    since_fix = 0.0  # estimated distance since the last fix that was used
    since_try = 0.0  # estimated distance since the last attempt
    zoom = calibration.zoom
    scale = 1.0  # change of the height above ground since the calibration, read from the zoom

    path = [estimate.copy()]
    sigma = [float(np.sqrt(variance))]
    fixes: list[FixRecord] = []
    for k in range(jam + 1, len(flight)):
        step = shifts[k] @ calibration.motion_matrix * scale
        estimate = estimate + step
        since_fix += float(np.linalg.norm(step))
        since_try += float(np.linalg.norm(step))

        if cfg.fix_every_m and since_try >= cfg.fix_every_m:
            since_try = 0.0
            predicted = predicted_variance(variance, since_fix, cfg.drift_rate)
            zooms = np.clip(zoom + np.arange(-cfg.zoom_reach, cfg.zoom_reach + 0.01, cfg.zoom_step), *cfg.zoom_limits)
            radius = max(cfg.min_search_radius_m, cfg.gate_sigmas * float(np.sqrt(predicted)))
            candidates = np.array([], dtype=int)
            if cfg.search in ("nearest", "sized"):
                candidates = reference.nearest(estimate, cfg.nearest_images)
            if cfg.search == "sized":
                inside = reference.within(estimate, radius)
                if len(inside) >= cfg.nearest_images:
                    candidates = inside
            if cfg.search in ("sized", "area") and since_fix > cfg.wide_zoom_after_m:
                zooms = np.arange(cfg.wide_zooms[0], cfg.wide_zooms[1] + 0.001, cfg.zoom_step)
            angles = np.array([calibration.angle - cfg.angle_reach_deg, calibration.angle, calibration.angle + cfg.angle_reach_deg])

            def find(j: int) -> Match | None:
                image = prepare(flight.frame(j))
                if ground is not None:
                    return search_area(image, ground, estimate, radius, zooms, angles, cfg.keep)
                return match(image, reference, candidates, zooms, angles, cfg.keep)

            # match this frame, and with the agreement check also the frames just before it,
            # each moved to the present by the dead-reckoned motion since then
            frames = [k - i * cfg.agreement_spacing for i in range(cfg.agreement_frames - 1, -1, -1)]
            frames = [j for j in frames if j > jam]
            found_all, moved, matched = [], [], []
            for j in frames:
                found_j = find(j)
                if found_j is None:
                    continue
                found_all.append(found_j)
                matched.append(j)
                moved.append(found_j.position - calibration.fix_offset + (estimate - path[j - jam]) if j < k else found_j.position - calibration.fix_offset)
            allowed = allowed_distance(predicted, fix_variance, cfg.gate_sigmas)
            searched = radius if cfg.search != "nearest" else 0.0
            if not matched or matched[-1] != k:
                nothing = np.full(2, np.nan)
                fixes.append(FixRecord(k, float("nan"), nothing, float("nan"), float("nan"), -1, 0, float("nan"), allowed, False, OFF_MAP, 0, searched))
                path.append(estimate.copy())
                sigma.append(float(np.sqrt(predicted_variance(variance, since_fix, cfg.drift_rate))))
                continue
            found = found_all[-1]
            position, score, agreeing = moved[-1], found.score, 1
            frames_agree = True
            if len(frames) > 1:
                group = agreeing_fixes(np.array(moved), cfg.agreement_radius_m, min(cfg.agreement_needed, len(frames)))
                frames_agree = group is not None
                if frames_agree:
                    best = max(group, key=lambda i: found_all[i].score)
                    found = found_all[best]
                    position = np.mean([moved[i] for i in group], axis=0)
                    score = float(np.mean([found_all[i].score for i in group]))
                    agreeing = len(group)

            distance = float(np.linalg.norm(position - estimate))
            if frames_agree:
                use, reason = fix_decision(score, distance, allowed, cfg.min_score)
            else:
                use, reason = False, FRAMES_DISAGREE
            fixes.append(
                FixRecord(k, score, position, found.zoom, found.angle, found.reference_index, len(candidates), distance, allowed, use, reason, agreeing, searched)
            )
            if use:
                estimate, variance, _ = blend(estimate, predicted, position, fix_variance)
                zoom = found.zoom
                scale = zoom / calibration.zoom
                since_fix = 0.0

        path.append(estimate.copy())
        sigma.append(float(np.sqrt(predicted_variance(variance, since_fix, cfg.drift_rate))))

    return NavigatorResult(
        start_index=jam,
        position=np.array(path),
        sigma=np.array(sigma),
        status=[status(s, cfg.degraded_above_m, cfg.lost_above_m) for s in sigma],
        fixes=fixes,
        calibration=calibration,
        config=cfg,
    )


def run_camera_navigator(
    flight: CameraFlight,
    cfg: NavigatorConfig | None = None,
    shifts: np.ndarray | None = None,
    cache_path: str | Path | None = None,
    calibration: Calibration | None = None,
) -> NavigatorResult:
    """Calibrate before the jam and navigate after it.

    ``shifts`` and ``calibration`` can be passed in when several runs share one flight.
    """
    cfg = cfg or NavigatorConfig()
    if shifts is None:
        shifts = shifts_for_flight(flight, cache_path)
    if calibration is None:
        calibration = calibrate(flight, shifts, cfg)
    return navigate(flight, shifts, calibration, cfg)

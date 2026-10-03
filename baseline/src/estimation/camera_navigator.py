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
from src.estimation.map_matching import KEEP, Match, match, quad_agreement, search_area
from src.estimation.navigator_core import (
    FRAMES_DISAGREE,
    OFF_MAP,
    QUARTERS_DISAGREE,
    UNCONFIRMED,
    fixes_agree,
    agreeing_fixes,
    allowed_distance,
    blend,
    fit_motion_matrix,
    fit_rotation_scale,
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
    max_search_radius_m: float | None = None  # area search: never wider than this; a compute budget, and
    # wider searches meet more look-alike places. The navigator still reports its full uncertainty.
    calibration_radius_m: float = 60.0  # area search before the jam: around the GNSS position
    offset_frame: str = "world"  # "world": the fix offset is learned in north and east; "body": in the drone's
    # forward and right directions, turned with the heading sensor (needs flight.heading_deg)
    confirm_jumps: bool = False  # a fix from a wide search that would move the estimate by more than
    # jump_limit_m is held until the next attempt, over different ground, agrees with it; then both are used
    jump_limit_m: float = 30.0  # twice the accuracy of one fix: a wrong fix closer than this does little harm
    confirm_above_radius_m: float = 150.0  # only searches wider than this need confirming: look-alikes come from wide searches
    camera_motion_floor: float | None = None  # a camera step shorter than this share of the cruising step is not
    # believed (the camera has lost track; a flying drone does not stop): the navigator then flies on at the
    # cruising speed learned with GNSS, along the last believable direction. None switches the check off.
    fallback_drift_rate: float = 0.30  # uncertainty growth while flying on like that, as a share of the distance
    motion_fit: str = "least_squares"  # how image shift becomes ground motion, learned before the jam:
    # "least_squares": a general 2 x 2 matrix (ALTO, a straight flight with a steady helicopter);
    # "rotation_scale": one turn and one scale from medians, for a quadcopter whose fixed camera tilts in turns
    quad_check: bool = False  # area search: the four quarters of the frame, searched alone, must land with the whole
    quad_needed: int = 3  # this many of the four quarters (Ilhan's rule "quad >= 3")
    quad_tolerance_share: float = 4.0 / 56.0  # Ilhan's 4 pixels on 56-pixel quarters, as a share of the quarter
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
    fix_offset: np.ndarray  # (2,) a fix lands this far from the true position: north, east in m, or
    # forward, right in m when offset_frame is "body"
    offset_frame: str = "world"
    cruise_speed_mps: float = 0.0  # mean ground speed while GNSS worked
    last_direction: np.ndarray | None = None  # (2,) unit vector (north, east) of the motion just before the jam

    def offset_at(self, heading_deg: float | None) -> np.ndarray:
        """The fix offset in north and east at a heading (bearing from north, degrees).

        Example: an offset of 10 m forward is 10 m north at heading 0 and 10 m east at heading 90.
        """
        if self.offset_frame == "world":
            return self.fix_offset
        a = np.radians(heading_deg)
        forward, right = self.fix_offset
        return np.array([forward * np.cos(a) - right * np.sin(a), forward * np.sin(a) + right * np.cos(a)])


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
    quarters_agreeing: int = -1  # with the quad check: how many quarters of the frame agreed; -1 if not checked


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
    lost_track: np.ndarray | None = None  # (M,) frames where the camera lost track and the navigator flew on at cruising speed


def jam_index(flight: CameraFlight, jam_after_m: float) -> int:
    """First frame at which the true distance flown reaches ``jam_after_m``."""
    return int(np.searchsorted(flight.travelled, jam_after_m))


def calibrate(flight: CameraFlight, shifts: np.ndarray, cfg: NavigatorConfig) -> Calibration:
    """Learn the motion matrix, zoom, rotation and fix offset from the stretch with GNSS."""
    jam = jam_index(flight, cfg.jam_after_m)
    if jam < 3 or jam >= len(flight):
        raise ValueError(f"the jam falls on frame {jam} of {len(flight)}; the flight needs GNSS before it and frames after it")
    truth = flight.position_gt[: jam + 1]
    fit = {"least_squares": fit_motion_matrix, "rotation_scale": fit_rotation_scale}[cfg.motion_fit]
    matrix = fit(shifts[1 : jam + 1], np.diff(truth, axis=0))

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
    zoom = float(np.median([f.zoom for f in fixes]))
    if zoom <= zooms[0] + 1e-9 or zoom >= zooms[-1] - 1e-9:
        # the best of the zooms tried may not be the right one: the camera's scale (its height) lies
        # outside what calibration_zooms covers, and every fix after the jam would be searched at the wrong scale
        raise ValueError(f"the camera's scale lies at the edge of the zooms tried ({zoom:.2f} of {zooms[0]:.2f} to "
                         f"{zooms[-1]:.2f}): the flight height is outside what calibration_zooms covers; widen it")
    offsets = np.array([f.position - truth[k] for f, k in zip(fixes, frames)])
    if cfg.offset_frame == "body":
        heading = _heading(flight)
        a = np.radians(heading[list(frames)])
        # north, east to forward, right: the inverse of Calibration.offset_at
        offsets = np.column_stack([offsets[:, 0] * np.cos(a) + offsets[:, 1] * np.sin(a), -offsets[:, 0] * np.sin(a) + offsets[:, 1] * np.cos(a)])
    elif cfg.offset_frame != "world":
        raise ValueError(f"unknown offset frame {cfg.offset_frame!r}")
    elapsed = float(flight.timestamp[jam] - flight.timestamp[0])
    last = truth[jam] - truth[max(0, jam - 5)]
    return Calibration(
        jam_index=jam,
        motion_matrix=matrix,
        zoom=zoom,
        angle=float(np.median([f.angle for f in fixes])),
        fix_offset=np.median(offsets, axis=0),
        offset_frame=cfg.offset_frame,
        cruise_speed_mps=float(flight.travelled[jam] / elapsed) if elapsed > 0 else 0.0,
        last_direction=last / max(float(np.linalg.norm(last)), 1e-9),
    )


def _heading(flight: CameraFlight) -> np.ndarray:
    if flight.heading_deg is None:
        raise ValueError(f"offset_frame 'body' needs the drone's heading; flight {flight.name} has none")
    return np.asarray(flight.heading_deg, dtype=float)


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
    heading = _heading(flight) if calibration.offset_frame == "body" else None

    def offset(j: int) -> np.ndarray:
        return calibration.offset_at(None if heading is None else heading[j])
    fix_variance = cfg.fix_sigma_m**2

    estimate = flight.position_gt[jam].copy()  # the last position GNSS gave
    variance = cfg.start_sigma_m**2
    since_fix = 0.0  # estimated distance since the last fix that was used
    since_try = 0.0  # estimated distance since the last attempt
    zoom = calibration.zoom
    scale = 1.0  # change of the height above ground since the calibration, read from the zoom

    pending: tuple[np.ndarray, int] | None = None  # a large jump waiting for the next fix: position, frame
    since_pending = 0.0

    # The uncertainty grows with a drift budget: 10 percent of every believable camera step, more for a
    # step flown on at cruising speed. With only camera steps this is drift_rate * since_fix, as before.
    drift_since_fix = 0.0
    direction = calibration.last_direction if calibration.last_direction is not None else np.array([1.0, 0.0])
    lost_track = []  # per frame: did the camera lose track of the motion?

    path = [estimate.copy()]
    sigma = [float(np.sqrt(variance))]
    fixes: list[FixRecord] = []
    for k in range(jam + 1, len(flight)):
        step = shifts[k] @ calibration.motion_matrix * scale
        rate = cfg.drift_rate
        length = float(np.linalg.norm(step))
        expected = calibration.cruise_speed_mps * float(flight.timestamp[k] - flight.timestamp[k - 1])
        if cfg.camera_motion_floor is not None and expected > 0 and length < cfg.camera_motion_floor * expected:
            step, rate, length = direction * expected, cfg.fallback_drift_rate, expected
            lost_track.append(True)
        else:
            if length > 0:
                direction = step / length
            lost_track.append(False)
        estimate = estimate + step
        since_fix += length
        since_try += length
        since_pending += length
        drift_since_fix += rate * length

        if cfg.fix_every_m and since_try >= cfg.fix_every_m:
            since_try = 0.0
            predicted = predicted_variance(variance, drift_since_fix, 1.0)
            zooms = np.clip(zoom + np.arange(-cfg.zoom_reach, cfg.zoom_reach + 0.01, cfg.zoom_step), *cfg.zoom_limits)
            radius = max(cfg.min_search_radius_m, cfg.gate_sigmas * float(np.sqrt(predicted)))
            if cfg.search == "area" and cfg.max_search_radius_m is not None:
                radius = min(radius, cfg.max_search_radius_m)
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
                moved.append(found_j.position - offset(j) + (estimate - path[j - jam]) if j < k else found_j.position - offset(j))
            allowed = allowed_distance(predicted, fix_variance, cfg.gate_sigmas)
            searched = radius if cfg.search != "nearest" else 0.0
            if not matched or matched[-1] != k:
                nothing = np.full(2, np.nan)
                fixes.append(FixRecord(k, float("nan"), nothing, float("nan"), float("nan"), -1, 0, float("nan"), allowed, False, OFF_MAP, 0, searched))
                pending = None
                path.append(estimate.copy())
                sigma.append(float(np.sqrt(predicted_variance(variance, drift_since_fix, 1.0))))
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
            # the four quarters of the frame, each searched alone, must land where the whole frame did
            quarters = -1
            if use and cfg.quad_check and ground is not None:
                quarters = quad_agreement(prepare(flight.frame(k)), ground, found, estimate, radius, cfg.keep, cfg.quad_tolerance_share)
                if quarters < cfg.quad_needed:
                    use, reason = False, QUARTERS_DISAGREE
            # a large jump out of a wide search needs the next fix, over different ground, to point the same way
            earlier = None  # a held fix that this one confirms: (its position carried to now, its variance now)
            if use and cfg.confirm_jumps and distance > cfg.jump_limit_m and radius > cfg.confirm_above_radius_m:
                if pending is not None:
                    carried = pending[0] + (estimate - path[pending[1] - jam])
                    if fixes_agree(pending[0], position, estimate - path[pending[1] - jam], since_pending, fix_variance, cfg.drift_rate, cfg.gate_sigmas):
                        earlier = (carried, fix_variance + (cfg.drift_rate * since_pending) ** 2)
                if earlier is None:
                    use, reason = False, UNCONFIRMED
            pending = (position.copy(), k) if reason == UNCONFIRMED else None
            since_pending = 0.0
            fixes.append(
                FixRecord(k, score, position, found.zoom, found.angle, found.reference_index, len(candidates), distance, allowed, use, reason, agreeing, searched, quarters)
            )
            if use:
                if earlier is not None:  # the confirmed earlier fix counts too
                    estimate, predicted, _ = blend(estimate, predicted, earlier[0], earlier[1])
                estimate, variance, _ = blend(estimate, predicted, position, fix_variance)
                zoom = found.zoom
                scale = zoom / calibration.zoom
                since_fix = 0.0
                drift_since_fix = 0.0

        path.append(estimate.copy())
        sigma.append(float(np.sqrt(predicted_variance(variance, drift_since_fix, 1.0))))

    return NavigatorResult(
        start_index=jam,
        position=np.array(path),
        sigma=np.array(sigma),
        status=[status(s, cfg.degraded_above_m, cfg.lost_above_m) for s in sigma],
        fixes=fixes,
        lost_track=np.array([False] + lost_track),
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

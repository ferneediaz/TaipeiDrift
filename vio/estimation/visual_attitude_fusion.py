"""Visual-attitude-assisted dead reckoning: the first visual-inertial estimator.

Same inertial mechanisation as the IMU-only baseline (baseline/src/estimation),
with one addition: whenever the camera provides a valid relative rotation
between a keyframe j and the current frame k, the attitude is pulled towards it.

Fusion (a complementary correction on SO(3))
---------------------------------------------
Let ``R_j`` and ``R_k`` be the estimator's own body->world attitudes at the
two frames (``R_j`` already includes earlier corrections) and ``C`` the visual
relative body rotation from j to k. The gyro predicts ``P = R_j^T R_k``. The
visual estimate of the current attitude is ``R_j C = R_k (P^T C)``, so

    r   = Log(P^T C)                 residual rotation vector, body frame at k
    R_k <- R_k Exp(gain * r)          gain in [0, 1]

``gain = 0`` reproduces the IMU-only baseline exactly; ``gain = 1`` replaces
the gyro's relative rotation by the camera's. A measurement is rejected (and
the gyro alone is used) if the camera failed, the span is shorter than
``min_keyframe_gap`` frames, or camera and gyro disagree by more than
``max_disagreement_deg``. With several candidates (homography), the one
closest to the gyro prediction is used. All of this uses only the IMU, the
camera and the estimator's own state: never ground truth or GNSS.

Velocity and position are integrated from the (corrected) attitude and the
accelerometer exactly as in the baseline. Monocular translation is not used:
it has no metric scale.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from src.data.trajectory import quat_wxyz_to_rotation, rotation_to_quat_wxyz
from src.estimation.inertial_dead_reckoning import DeadReckoningResult, NavState
from vio.vision.measurements import VisualMeasurement


@dataclass
class FusionConfig:
    """Strength and gating of the visual attitude correction."""

    gain: float = 0.1  # fraction of the visual residual applied per accepted measurement
    max_disagreement_deg: float = 2.0  # reject when camera and gyro differ by more than this
    min_keyframe_gap: int = 1  # frames; ignore measurements over shorter spans


@dataclass
class FusionEvent:
    """What the fusion did with one visual measurement."""

    frame_index: int
    imu_index: int
    accepted: bool
    disagreement_deg: float  # angle between camera and gyro relative rotation; nan if not compared
    correction_deg: float  # size of the applied correction
    reason: str  # why it was rejected; empty when accepted


def integrate_translation(
    timestamp: np.ndarray,
    accelerometer: np.ndarray,
    attitude: Rotation,
    initial: NavState,
    gravity_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Trapezoidal velocity and position integration, identical to the baseline's."""
    dt = np.diff(timestamp)[:, None]
    accel_world = attitude.apply(accelerometer) + gravity_world
    velocity = np.vstack([initial.velocity, initial.velocity + np.cumsum(0.5 * (accel_world[:-1] + accel_world[1:]) * dt, axis=0)])
    position = np.vstack([initial.position, initial.position + np.cumsum(0.5 * (velocity[:-1] + velocity[1:]) * dt, axis=0)])
    return position, velocity


def run_visual_attitude_fusion(
    timestamp: np.ndarray,
    accelerometer: np.ndarray,
    gyroscope: np.ndarray,
    initial: NavState,
    gravity_world: np.ndarray,
    gyroscope_frame: str,
    measurements: list[VisualMeasurement],
    start_index: int,
    cfg: FusionConfig | None = None,
) -> tuple[DeadReckoningResult, list[FusionEvent]]:
    """Dead-reckon from the GNSS cutoff with visual attitude corrections.

    Args:
        timestamp, accelerometer, gyroscope: IMU from the cutoff onwards (index 0 = cutoff).
        initial: state at the cutoff, the only ground truth the estimator gets.
        gravity_world: gravity vector in the world frame.
        gyroscope_frame: "body" or "world", as in the baseline.
        measurements: visual measurements; ``imu_index`` values are absolute
            indices into the flight, converted with ``start_index``.
        start_index: flight index of ``timestamp[0]``.
        cfg: fusion strength and gating.

    Returns:
        the estimate (same type as the baseline's) and one event per measurement.
    """
    cfg = cfg or FusionConfig()
    if gyroscope_frame not in ("body", "world"):
        raise ValueError(f"gyroscope_frame must be 'body' or 'world', got {gyroscope_frame!r}")
    if not 0.0 <= cfg.gain <= 1.0:
        raise ValueError(f"gain must be in [0, 1], got {cfg.gain}")
    n = len(timestamp)
    dt = np.diff(timestamp)
    if np.any(dt <= 0):
        raise ValueError("timestamps must be strictly increasing")
    increments = Rotation.from_rotvec(0.5 * (gyroscope[:-1] + gyroscope[1:]) * dt[:, None])
    body = gyroscope_frame == "body"
    max_dis = np.deg2rad(cfg.max_disagreement_deg)

    by_sample: dict[int, list[VisualMeasurement]] = {}
    for m in measurements:
        by_sample.setdefault(m.imu_index - start_index, []).append(m)

    events: list[FusionEvent] = []
    quats = np.empty((n, 4))  # scipy order while integrating
    r = quat_wxyz_to_rotation(initial.attitude)
    quats[0] = r.as_quat()

    def handle(m: VisualMeasurement, k: int, r: Rotation) -> Rotation:
        kf = m.keyframe_imu_index - start_index
        if not m.valid or not m.body_rotation_candidates:
            events.append(FusionEvent(m.frame_index, m.imu_index, False, np.nan, 0.0, m.reason or "invalid"))
            return r
        if kf < 0 or kf > k:
            events.append(FusionEvent(m.frame_index, m.imu_index, False, np.nan, 0.0, "keyframe before cutoff"))
            return r
        if m.frame_index - m.keyframe_index < cfg.min_keyframe_gap:
            events.append(FusionEvent(m.frame_index, m.imu_index, False, np.nan, 0.0, "span too short"))
            return r
        predicted = Rotation.from_quat(quats[kf]).inv() * r
        residuals = [(predicted.inv() * Rotation.from_matrix(C)).as_rotvec() for C in m.body_rotation_candidates]
        res = min(residuals, key=np.linalg.norm)
        dis = float(np.linalg.norm(res))
        if dis > max_dis:
            events.append(FusionEvent(m.frame_index, m.imu_index, False, np.degrees(dis), 0.0, "disagrees with gyro"))
            return r
        events.append(FusionEvent(m.frame_index, m.imu_index, True, np.degrees(dis), np.degrees(cfg.gain * dis), ""))
        return r * Rotation.from_rotvec(cfg.gain * res)

    for m in by_sample.get(0, []):  # a measurement exactly at the cutoff has no span; log it only
        events.append(FusionEvent(m.frame_index, m.imu_index, False, np.nan, 0.0, m.reason or "at cutoff"))
    for k in range(n - 1):
        r = r * increments[k] if body else increments[k] * r
        for m in by_sample.get(k + 1, []):
            r = handle(m, k + 1, r)
        quats[k + 1] = r.as_quat()

    attitude = Rotation.from_quat(quats)
    position, velocity = integrate_translation(timestamp, accelerometer, attitude, initial, gravity_world)
    result = DeadReckoningResult(timestamp, position, velocity, rotation_to_quat_wxyz(attitude),
                                 start_index=start_index, t0=float(timestamp[0]))
    return result, events

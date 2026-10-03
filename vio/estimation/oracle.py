"""ORACLE ABLATION: dead reckoning with the GROUND-TRUTH attitude. NOT AN ESTIMATOR.

This answers one diagnostic question: how much position drift would remain if
the attitude were perfect? The measured accelerometer is rotated into the
world frame with the TRUE attitude after the GNSS cutoff, then integrated as
in the baseline. It uses ground truth after t0 by construction, so it can never
be deployed and must never be reported as an estimator. It bounds how much any
attitude correction (visual or otherwise) can help.
"""
from __future__ import annotations

from src.data.trajectory import Trajectory, quat_wxyz_to_rotation
from src.estimation.inertial_dead_reckoning import DeadReckoningResult, NavState
from vio.estimation.visual_attitude_fusion import integrate_translation

ORACLE_LABEL = "ORACLE: ground-truth attitude (not an estimator)"


def oracle_ground_truth_attitude(traj: Trajectory, gnss_cutoff: float) -> DeadReckoningResult:
    """Integrate the measured accelerometer with the TRUE attitude after the cutoff. Diagnostic only."""
    k0 = traj.index_at(gnss_cutoff)
    t = traj.timestamp[k0:]
    initial = NavState(traj.position_gt[k0].copy(), traj.velocity_gt[k0].copy(), traj.attitude_gt[k0].copy())
    attitude_gt = traj.attitude_gt[k0:].copy()  # <- the oracle part: ground truth after t0
    position, velocity = integrate_translation(t, traj.accelerometer[k0:], quat_wxyz_to_rotation(attitude_gt),
                                               initial, traj.gravity_world)
    return DeadReckoningResult(t, position, velocity, attitude_gt, start_index=k0, t0=float(t[0]))

"""Linearization helpers for live visual updates that use a cloned keyframe attitude."""
import numpy as np

from vio.estimation.eskf import N_ERR, V_, skew


def direction_update_terms(filter_state, velocity_world, direction_world):
    """Build the tangent-plane residual/Jacobian for camera displacement direction.

    The direction is rotated into world using ``R_clone``; therefore its attitude
    error belongs to the cloned keyframe block, not the current attitude block.
    """
    if not filter_state.has_clone:
        raise ValueError("direction update requires the camera keyframe attitude clone")
    v = np.asarray(velocity_world, dtype=float)
    d_vis = np.asarray(direction_world, dtype=float)
    speed = float(np.linalg.norm(v))
    if speed <= 0 or np.linalg.norm(d_vis) <= 0:
        raise ValueError("direction update requires nonzero velocity and visual direction")
    d, d_vis = v / speed, d_vis / np.linalg.norm(d_vis)
    if float(d @ d_vis) <= 0.0:
        raise ValueError("visual direction is in the opposite hemisphere from predicted velocity")
    axis = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(d, axis)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(d, e1)
    B = np.column_stack([e1, e2])
    residual = B.T @ d_vis
    H = np.zeros((2, filter_state.n))
    H[:, V_] = B.T / speed
    H[:, N_ERR:N_ERR + 3] = B.T @ skew(d)
    return residual, H, d_vis, d

"""Camera model: pinhole intrinsics and the camera-to-body rotation.

Frames
------
- Camera frame (OpenCV): x right in the image, y down in the image, z along the optical axis.
- Body frame: the frame of the Mid-Air attitude quaternion (body -> NED world).
- ``R_bc`` rotates camera-frame vectors into the body frame: ``v_body = R_bc @ v_cam``.

A rotation measured by the camera (camera b in camera a, ``R_ab^cam``) maps to
the body as

    R_ab^body = R_bc @ R_ab^cam @ R_bc^T.

The translation between camera and body origins does not enter relative rotation.
"""
from __future__ import annotations

import numpy as np


def pinhole_intrinsics(width: int, height: int, hfov_deg: float) -> np.ndarray:
    """Intrinsic matrix of an ideal pinhole camera with square pixels and centred principal point."""
    f = (width / 2) / np.tan(np.deg2rad(hfov_deg) / 2)
    return np.array([[f, 0.0, width / 2], [0.0, f, height / 2], [0.0, 0.0, 1.0]])


def camera_to_body(R_bc: np.ndarray, rotation_cam: np.ndarray) -> np.ndarray:
    """Express a relative rotation measured in the camera frame in the body frame."""
    R_bc = np.asarray(R_bc, dtype=float)
    return R_bc @ np.asarray(rotation_cam, dtype=float) @ R_bc.T


def check_rotation_matrix(R: np.ndarray, name: str = "R_bc") -> np.ndarray:
    """Validate a proper rotation matrix (orthonormal, det +1)."""
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3) or not np.allclose(R @ R.T, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(R), 1.0, atol=1e-6):
        raise ValueError(f"{name} must be a 3x3 rotation matrix with determinant +1, got {R.tolist()}")
    return R

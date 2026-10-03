"""Shared navigation math (numba): rotations, Earth rates, normal gravity, latitude model."""
from __future__ import annotations

import math

import numpy as np
from numba import njit


@njit(cache=True)
def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


@njit(cache=True)
def exp_so3(w):
    """Rotation matrix exp([w x]) (Rodrigues)."""
    th = math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2])
    K = skew(w)
    if th < 1e-8:
        return np.eye(3) + K + 0.5 * K @ K
    return np.eye(3) + (math.sin(th) / th) * K + ((1.0 - math.cos(th)) / (th * th)) * (K @ K)


@njit(cache=True)
def log_so3(R):
    """Rotation vector w with exp([w x]) = R."""
    c = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)
    c = min(1.0, max(-1.0, c))
    th = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-8:
        return 0.5 * v
    return (th / (2.0 * math.sin(th))) * v


@njit(cache=True)
def latitude(n, lat_ref, n_ref, R):
    return lat_ref + (n - n_ref) / R


@njit(cache=True)
def gravity(lat, h):
    s = math.sin(lat)
    s2 = math.sin(2.0 * lat)
    return 9.780327 * (1.0 + 0.0053024 * s * s - 0.0000058 * s2 * s2) - 3.086e-6 * h


@njit(cache=True)
def nav_rates(lat, h, v, R, omega):
    """Earth rate and transport rate in ENU, plus gravity vector."""
    wie = np.array([0.0, omega * math.cos(lat), omega * math.sin(lat)])
    Rh = R + h
    wen = np.array([-v[1] / Rh, v[0] / Rh, v[0] * math.tan(lat) / Rh])
    g = np.array([0.0, 0.0, -gravity(lat, h)])
    return wie, wen, g


def euler_to_C(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Body(FLU)->nav(ENU) DCMs C = Rz(yaw) Ry(-pitch) Rx(roll). yaw = math angle from East, CCW.

    Positive roll = right wing down, positive pitch = nose up.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(-pitch), np.sin(-pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    n = roll.shape[0]
    Rx = np.zeros((n, 3, 3)); Rx[:, 0, 0] = 1; Rx[:, 1, 1] = cr; Rx[:, 1, 2] = -sr; Rx[:, 2, 1] = sr; Rx[:, 2, 2] = cr
    Ry = np.zeros((n, 3, 3)); Ry[:, 1, 1] = 1; Ry[:, 0, 0] = cp; Ry[:, 0, 2] = sp; Ry[:, 2, 0] = -sp; Ry[:, 2, 2] = cp
    Rz = np.zeros((n, 3, 3)); Rz[:, 2, 2] = 1; Rz[:, 0, 0] = cy; Rz[:, 0, 1] = -sy; Rz[:, 1, 0] = sy; Rz[:, 1, 1] = cy
    return Rz @ Ry @ Rx

import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim" / "nodes"))

from metric_flow import estimate_metric_velocity, flow_update_due, range_jump_detected  # noqa: E402
from frame_conversions import gazebo_down_optical_to_flu  # noqa: E402
from vio.vision.optical_flow import FlowPair  # noqa: E402


def projected_pair(pa, pb, Ra, Rb, dt=0.1):
    rng = np.random.default_rng(7)
    xy = rng.uniform(-0.65, 0.65, (400, 2))
    rays_a = np.c_[xy, np.ones(len(xy))]
    Rbc = gazebo_down_optical_to_flu()
    Rwa, Rwb = Ra @ Rbc, Rb @ Rbc
    Pw = pa + ((-pa[2] / (rays_a @ Rwa.T)[:, 2])[:, None] * (rays_a @ Rwa.T))
    def project(p, Rw):
        q = (Pw - p) @ Rw
        return q[:, :2] / q[:, 2:3]
    return FlowPair(1, 1, 0, project(pa, Rwa), project(pb, Rwb), 400, 1.0, True)


def estimate(pair, Rab, R0, range_m=5.0):
    Rbc = gazebo_down_optical_to_flu()
    ncam = (R0 @ Rbc).T @ np.array([0.0, 0.0, -1.0])
    return estimate_metric_velocity(pair, Rab, ncam, range_m, 0.1, 256.0,
                                    range_std_m=0.02)


def test_translation_sign_axes_and_range_scale():
    Ra = Rb = np.eye(3)
    pa, pb = np.array([0.0, 0.0, 5.0]), np.array([0.4, -0.2, 5.0])
    pair = projected_pair(pa, pb, Ra, Rb)
    out = estimate(pair, np.eye(3), Ra)
    assert out["valid"]
    expected_camera = gazebo_down_optical_to_flu().T @ (pb - pa) / 0.1
    np.testing.assert_allclose(out["velocity"], expected_camera[:2], atol=1e-3)
    scaled = estimate(pair, np.eye(3), Ra, range_m=10.0)
    np.testing.assert_allclose(scaled["velocity"], out["velocity"] * 2, atol=1e-3)


def test_pure_camera_rotation_is_compensated():
    Ra = Rotation.from_euler("xyz", [2, -3, 5], degrees=True).as_matrix()
    Rb = Ra @ Rotation.from_euler("xyz", [0.4, -0.2, 0.7], degrees=True).as_matrix()
    p = np.array([0.0, 0.0, 5.0])
    pair = projected_pair(p, p, Ra, Rb)
    Rbc = gazebo_down_optical_to_flu()
    Rab = Rbc.T @ Ra.T @ Rb @ Rbc
    out = estimate(pair, Rab, Ra)
    assert out["valid"]
    assert np.linalg.norm(out["velocity"]) < 1e-5


def test_invalid_range_and_discontinuity_are_gated():
    pair = FlowPair(1, 1, 0, np.zeros((50, 2)), np.zeros((50, 2)), 50, 1.0, True)
    assert estimate(pair, np.eye(3), np.eye(3), range_m=np.nan)["reason"] == "invalid range"
    assert range_jump_detected(3.0, 4.0, 0.75)
    assert not range_jump_detected(3.0, 3.2, 0.75)
    assert [flow_update_due(i, 5) for i in range(10)] == [True, False, False, False, False] * 2


def test_zero_translation_returns_near_zero():
    Ra = Rb = np.eye(3)
    p = np.array([0.0, 0.0, 5.0])
    out = estimate(projected_pair(p, p, Ra, Rb), np.eye(3), Ra)
    assert out["valid"]
    assert np.linalg.norm(out["velocity"]) < 1e-8


def test_production_estimator_path_has_no_ground_truth_subscription():
    source = (ROOT / "sim" / "nodes" / "eskf_ros_adapter.py").read_text(encoding="utf-8")
    assert '"/ground_truth/odom"' not in source
    assert "ground_truth" not in source

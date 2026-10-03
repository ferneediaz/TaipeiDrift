import math
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "baseline"))
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim" / "nodes"))
from gnss_policy import gnss_available
from run_policy import gps_csv_values
from gnss_projection import geodetic_to_enu, geodetic_covariance_to_enu
from frame_conversions import gazebo_optical_to_flu, gazebo_down_optical_to_flu
from visual_update_math import direction_update_terms
from vio.estimation.eskf import ESKF


def test_gnss_gate_forwards_until_cutoff():
    assert gnss_available(20.0, 19.999)
    assert not gnss_available(20.0, 20.0)
    assert not gnss_available(20.0, 21.0)


def test_negative_cutoff_keeps_gnss_open():
    assert gnss_available(-1.0, 0.0)
    assert gnss_available(-1.0, 100000.0)


def test_csv_gps_denial_representation_is_nan():
    denied_gps = gps_csv_values(False, [19.0, 25.0, 121.0, 12.0], 21.0)
    assert all(math.isnan(v) for v in denied_gps)
    available_gps = gps_csv_values(True, [19.0, 25.0, 121.0, 12.0], 20.0)
    assert available_gps == [25.0, 121.0, 12.0]
    stale_gps = gps_csv_values(True, [17.0, 25.0, 121.0, 12.0], 20.0)
    assert all(math.isnan(v) for v in stale_gps)


def test_geodetic_projection_is_explicit_enu():
    origin = (25.033, 121.5654, 10.0)
    np.testing.assert_allclose(geodetic_to_enu(*origin, origin), np.zeros(3), atol=1e-8)
    east = geodetic_to_enu(origin[0], origin[1] + 0.00001, origin[2], origin)
    north = geodetic_to_enu(origin[0] + 0.00001, origin[1], origin[2], origin)
    assert east[0] > 0 and abs(east[1]) < 0.01
    assert north[1] > 0 and abs(north[0]) < 0.01


def test_empty_gazebo_gnss_covariance_uses_sensor_noise_assumption():
    cov = geodetic_covariance_to_enu([0.0] * 9, 25.0)
    np.testing.assert_allclose(cov, np.diag([2.25, 2.25, 9.0]))


def test_eskf_cartesian_gnss_update_corrects_position():
    f = ESKF([0, 0, 0], [0, 0, 0], [1, 0, 0, 0], [0, 0, -9.80665], "body")
    result = f.update_position(np.array([4.0, -2.0, 1.0]), np.diag([2.25, 2.25, 9.0]), None)
    assert result.accepted
    assert np.linalg.norm(f.p) > 0
    assert np.linalg.norm(f.p - np.array([4.0, -2.0, 1.0])) < np.linalg.norm(np.array([4.0, -2.0, 1.0]))


def test_gazebo_camera_extrinsic_basis_mapping():
    R_bc = gazebo_optical_to_flu()
    np.testing.assert_allclose(R_bc @ [1, 0, 0], [0, -1, 0])
    np.testing.assert_allclose(R_bc @ [0, 1, 0], [0, 0, -1])
    np.testing.assert_allclose(R_bc @ [0, 0, 1], [1, 0, 0])
    np.testing.assert_allclose(R_bc.T @ R_bc, np.eye(3), atol=1e-12)
    R_down = gazebo_down_optical_to_flu()
    np.testing.assert_allclose(R_down @ [0, 0, 1], [0, 0, -1])
    np.testing.assert_allclose(R_down @ [0, 1, 0], [-1, 0, 0])
    np.testing.assert_allclose(R_down.T @ R_down, np.eye(3), atol=1e-12)


def test_eskf_cartesian_gnss_velocity_update_corrects_velocity():
    f = ESKF([0, 0, 0], [0, 0, 0], [1, 0, 0, 0], [0, 0, -9.80665], "body")
    result = f.update_velocity(np.array([2.0, -1.0, 0.5]), np.diag([0.25, 0.25, 1.0]), None)
    assert result.accepted
    assert np.linalg.norm(f.v) > 0
    assert np.linalg.norm(f.v - np.array([2.0, -1.0, 0.5])) < np.linalg.norm(np.array([2.0, -1.0, 0.5]))


def test_live_direction_jacobian_uses_keyframe_clone_not_current_attitude():
    from scipy.spatial.transform import Rotation
    f = ESKF([0, 0, 0], [5.0, 0, 0], [1, 0, 0, 0], [0, 0, -9.80665], "body")
    f.clone_attitude()
    observed = Rotation.from_euler("z", 10, degrees=True).apply([1.0, 0, 0])
    residual, H, _, _ = direction_update_terms(f, f.v, observed)
    assert np.all(H[:, 6:9] == 0)  # current attitude is not the camera-keyframe attitude
    assert np.linalg.norm(H[:, 15:18]) > 0
    result = f.update(residual, H, np.eye(2) * np.radians(1.0) ** 2, None)
    assert result.accepted
    assert f.R_clone.magnitude() > 0 and f.R.magnitude() > 0  # cloned/current states are correlated


def test_direction_residual_rejects_antipodal_measurement():
    import pytest
    f = ESKF([0, 0, 0], [5.0, 0, 0], [1, 0, 0, 0], [0, 0, -9.80665], "body")
    f.clone_attitude()
    with pytest.raises(ValueError, match="opposite hemisphere"):
        direction_update_terms(f, f.v, [-1.0, 0.0, 0.0])


def test_production_adapter_has_no_ground_truth_subscription():
    adapter = (ROOT / "sim" / "nodes" / "eskf_ros_adapter.py").read_text(encoding="utf-8")
    assert '"/ground_truth/odom"' not in adapter
    assert 'create_subscription(NavSatFix, "/gps/fix"' in adapter

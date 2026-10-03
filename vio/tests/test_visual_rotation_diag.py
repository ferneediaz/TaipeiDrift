"""Visual-rotation diagnostic: schema, residual and frame conventions, sign metric, intervals, aggregation."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vio.diagnostics.sources import frames_for, ntu_calibration, ntu_validate_orientation
from vio.diagnostics.visual_rotation import SCHEMA, VisualInterval, dataset_summary, interval_row, sign_stability, time_intervals

R_BC_MIDAIR = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
R_BC_NTU = np.array([[0.02183084, -0.01312053, 0.99967558], [0.99975965, 0.00230088, -0.02180248],
                     [-0.00201407, 0.99991127, 0.01316761]])


def _row(R_bc, err_body=np.zeros(3), Ri=None, rel=None, dataset="midair"):
    Ri = Ri or Rotation.from_euler("xyz", [5, -10, 40], degrees=True)
    rel = rel or Rotation.from_rotvec([0.02, -0.03, 0.05])
    Rj = Ri * rel
    vis_body = rel * Rotation.from_rotvec(err_body)  # visual = true * error (body, right)
    U, _, Vt = np.linalg.svd(R_bc)
    R_bc = U @ Vt  # exact rotation
    C_cam = R_bc.T @ vis_body.as_matrix() @ R_bc
    iv = VisualInterval(1.0, 1.5, C_cam, 200, 180, 0.9, 5.0, 100.0)
    return interval_row(dataset, "c", "s", iv, Ri, Rj, np.zeros(3), np.array([3.0, 4.0, 0.0]), R_bc), R_bc, Rj


def test_schema_is_common():
    row, _, _ = _row(R_BC_MIDAIR)
    assert list(row) == SCHEMA


@pytest.mark.parametrize("R_bc,dataset", [(R_BC_MIDAIR, "midair"), (R_BC_NTU, "ntu")])
def test_residual_convention_and_frames(R_bc, dataset):
    zero, _, _ = _row(R_bc, dataset=dataset)
    assert zero["e_vis_norm_deg"] < 1e-9
    err = np.deg2rad([0.1, -0.2, 0.3])
    row, Rbc, Rj = _row(R_bc, err, dataset=dataset)
    e_body = np.array([row[f"e_body_rate_{a}_deg_s"] for a in "xyz"]) * row["dt"]
    np.testing.assert_allclose(e_body, np.degrees(err), atol=1e-9)
    np.testing.assert_allclose([row[f"e_cam_rate_{a}_deg_s"] * row["dt"] for a in "xyz"], Rbc.T @ np.degrees(err), atol=1e-9)
    np.testing.assert_allclose([row[f"e_world_rate_{a}_deg_s"] * row["dt"] for a in "xyz"], Rj.apply(np.degrees(err)), atol=1e-9)
    assert row["e_vis_rate_norm_deg_s"] == pytest.approx(np.linalg.norm(np.degrees(err)) / 0.5)


def test_motion_columns():
    row, _, _ = _row(R_BC_MIDAIR)
    assert row["translation_magnitude_m"] == pytest.approx(5.0)
    assert row["mean_speed_m_s"] == pytest.approx(10.0)
    assert row["dominant_axis"] == "z"


def test_sign_stability():
    assert sign_stability(np.array([1, 2, 3, 4.0])) == 1.0
    assert sign_stability(np.array([1, -1, 1, -1.0])) == 0.5
    assert sign_stability(np.array([1, 1, 1, -1.0])) == 0.75


def test_time_based_intervals():
    assert frames_for(10.0) == 5 and frames_for(25.0) == 13
    t = np.arange(0, 3.0, 0.04)
    iv = time_intervals(t, 0.5)
    assert all(abs(t[j] - t[i] - 0.5) <= 0.0201 for i, j in iv)
    assert all(iv[k][1] == iv[k + 1][0] for k in range(len(iv) - 1))  # non-overlapping, contiguous


def test_aggregation_finds_body_fixed_error():
    """A constant body-frame error with varying attitude is stable in body/camera, dispersed in world."""
    rng = np.random.default_rng(0)
    rows = []
    for seq in ("a", "b"):
        for k in range(60):
            Ri = Rotation.from_euler("z", rng.uniform(0, 360), degrees=True)
            r, _, _ = _row(R_BC_MIDAIR, np.deg2rad([0.05, 0.0, 0.0]) + rng.normal(0, 1e-4, 3), Ri=Ri)
            r["sequence"] = seq
            rows.append(r)
    s = dataset_summary(rows)
    sd = s["frame_signal_to_dispersion_median"]
    assert sd["body"] > 5 * sd["world"] and s["most_stable_frame"] in ("body", "cam")
    assert s["n_sequences"] == 2 and s["n_intervals"] == 120


def test_ntu_calibration_parsing(tmp_path):
    (tmp_path / "camera_left.yaml").write_text(
        "%YAML:1.0\nimage_width: 752\nimage_height: 480\ndistortion_parameters:\n   k1: -0.28\n   k2: 0.07\n   p1: 0.0007\n   p2: -0.0002\n"
        "projection_parameters:\n   fx: 425.0\n   fy: 426.8\n   cx: 386.0\n   cy: 241.9\nT_Body_Cam: !!opencv-matrix\n   rows: 4\n   cols: 4\n"
        "   dt: d\n   data: [0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1]\nt_shift: -0.02\n")
    c = ntu_calibration(tmp_path)
    np.testing.assert_array_equal(c["R_bc"], R_BC_MIDAIR)
    assert c["K"][0, 0] == 425.0 and c["t_shift"] == -0.02 and c["size"] == (752, 480)


def test_ntu_orientation_validation_detects_body_gyro():
    t = np.arange(0, 20, 0.005)
    wb = np.array([0.3, -0.2, 0.4])
    R = Rotation.from_euler("xyz", [10, 20, 30], degrees=True) * Rotation.from_rotvec(t[:, None] * wb)
    v = ntu_validate_orientation({"imu_t": t, "imu_q_xyzw": R.as_quat(), "imu_w": np.tile(wb, (len(t), 1))})
    assert v["body_right"] < 1e-6 < v["world_left"]

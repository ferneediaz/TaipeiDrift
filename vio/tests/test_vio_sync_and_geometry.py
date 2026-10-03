"""Image/IMU synchronisation, camera-body transformation and the relative-rotation convention."""
import h5py
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vio.data.midair_camera import FrameSource, frame_to_imu_index, open_frames, stream_name
from vio.pipeline import CameraSetup, frame_range
from vio.vision.camera import camera_to_body, check_rotation_matrix, pinhole_intrinsics
from vio.vision.relative_pose import PoseConfig, estimate_relative_pose, homography_rotation_candidates

R_BC_LEFT = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
R_BC_DOWN = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
K = pinhole_intrinsics(512, 512, 90.0)


# ---------- synchronisation ----------

def test_frame_i_is_imu_sample_4i():
    np.testing.assert_array_equal(frame_to_imu_index(np.arange(4), 100.0, 25.0), [0, 4, 8, 12])
    assert frame_to_imu_index(2204, 100.0, 25.0) == 8816  # last Mid-Air frame, inside the 8818 IMU samples
    np.testing.assert_array_equal(frame_to_imu_index(np.arange(3), 100.0, 25.0, offset_samples=2), [2, 6, 10])


def test_non_integer_rate_ratio_is_rejected():
    with pytest.raises(ValueError, match="integer multiple"):
        frame_to_imu_index(1, 100.0, 30.0)


def test_frame_range_starts_at_cutoff():
    setup = CameraSetup("left", "color_left", 90.0, 25.0, 0, R_BC_LEFT, "essential", 2)
    frames = FrameSource([f"{i:06d}.JPEG" for i in range(2205)], None, None)
    rng = frame_range(frames, setup, 100.0, start_index=500, n_imu=8818)
    assert rng.start == 125 and rng.stop - 1 == 2204
    rng = frame_range(frames, setup, 100.0, start_index=501, n_imu=8818)  # cutoff between frames
    assert rng.start == 126


def test_stream_aliases():
    assert stream_name("down") == "color_down"
    assert stream_name("left") == "color_left"
    assert stream_name("color_right") == "color_right"


def test_open_frames_reports_missing_images(tmp_path):
    sensor = tmp_path / "sensor_records.hdf5"
    with h5py.File(sensor, "w") as f:
        f.create_dataset("trajectory_0000/camera_data/color_down", data=np.array([b"color_down/trajectory_0000/000000.JPEG"]))
    with pytest.raises(FileNotFoundError, match="frames.zip"):
        open_frames(sensor, "trajectory_0000", "down")
    with pytest.raises(KeyError, match="color_left"):
        open_frames(sensor, "trajectory_0000", "left")


# ---------- camera / body ----------

def test_camera_axes_map_to_body_axes():
    # left camera: optical axis forward, image right = body right, image down = body down
    np.testing.assert_array_equal(R_BC_LEFT @ [0, 0, 1], [1, 0, 0])
    np.testing.assert_array_equal(R_BC_LEFT @ [1, 0, 0], [0, 1, 0])
    # down camera: optical axis = body down, image up (-y) = body forward
    np.testing.assert_array_equal(R_BC_DOWN @ [0, 0, 1], [0, 0, 1])
    np.testing.assert_array_equal(R_BC_DOWN @ [0, -1, 0], [1, 0, 0])


def test_rotation_about_optical_axis_is_body_roll_for_forward_camera():
    cam_rot = Rotation.from_rotvec([0, 0, 0.1]).as_matrix()  # about camera z
    body = camera_to_body(R_BC_LEFT, cam_rot)
    np.testing.assert_allclose(Rotation.from_matrix(body).as_rotvec(), [0.1, 0, 0], atol=1e-12)


def test_rotation_about_optical_axis_is_body_yaw_for_down_camera():
    body = camera_to_body(R_BC_DOWN, Rotation.from_rotvec([0, 0, 0.1]).as_matrix())
    np.testing.assert_allclose(Rotation.from_matrix(body).as_rotvec(), [0, 0, 0.1], atol=1e-12)


def test_invalid_mounting_is_rejected():
    with pytest.raises(ValueError):
        check_rotation_matrix([[1, 0, 0], [0, 1, 0], [0, 0, -1]])  # reflection, det -1


# ---------- relative rotation convention ----------

def _project(points_w, R_wc, c_w):
    pc = (points_w - c_w) @ R_wc  # world -> camera: R_wc^T (p - c)
    uv = pc[:, :2] / pc[:, 2:3]
    return uv * K[0, 0] + K[:2, 2]


def _scene(rng, n=300, depth=(8, 30)):
    z = rng.uniform(*depth, n)
    xy = rng.uniform(-0.8, 0.8, (n, 2)) * z[:, None]
    return np.column_stack([xy, z])


def test_essential_matrix_returns_camera_b_in_camera_a():
    rng = np.random.default_rng(0)
    pts = _scene(rng)
    R_wc_a = np.eye(3)
    R_ab_true = Rotation.from_euler("xyz", [2.0, -3.0, 1.5], degrees=True).as_matrix()
    c_b = np.array([0.6, 0.1, 0.2])
    ua, ub = _project(pts, R_wc_a, np.zeros(3)), _project(pts, R_wc_a @ R_ab_true, c_b)
    pose = estimate_relative_pose(ua, ub, K, PoseConfig(ransac_threshold_px=0.5))
    assert pose.valid
    assert Rotation.from_matrix(pose.rotation.T @ R_ab_true).magnitude() < np.deg2rad(0.05)
    np.testing.assert_allclose(pose.translation_dir, c_b / np.linalg.norm(c_b), atol=1e-3)
    assert np.isclose(np.linalg.norm(pose.translation_dir), 1.0)  # direction only: monocular scale unknown


def test_homography_candidates_contain_true_rotation_for_ground_plane():
    rng = np.random.default_rng(1)
    xy = rng.uniform(-20, 20, (300, 2))
    pts = np.column_stack([xy, np.full(300, 30.0)])  # flat ground 30 m below a down camera
    R_ab_true = Rotation.from_euler("xyz", [1.0, -2.0, 4.0], degrees=True).as_matrix()
    ua = _project(pts, np.eye(3), np.zeros(3))
    ub = _project(pts, R_ab_true, np.array([0.5, 0.2, 0.0]))
    cands, pose = homography_rotation_candidates(ua, ub, K)
    assert pose.valid and cands
    assert min(Rotation.from_matrix(C.T @ R_ab_true).magnitude() for C in cands) < np.deg2rad(0.05)


@pytest.mark.parametrize("n", [0, 3, 10])
def test_too_few_points_is_invalid_not_an_exception(n):
    pts = np.random.default_rng(2).uniform(0, 512, (n, 2))
    assert not estimate_relative_pose(pts, pts + 1.0, K).valid
    cands, pose = homography_rotation_candidates(pts, pts + 1.0, K)
    assert not pose.valid and cands == []


def test_random_correspondences_are_rejected():
    rng = np.random.default_rng(3)
    pose = estimate_relative_pose(rng.uniform(0, 512, (200, 2)), rng.uniform(0, 512, (200, 2)), K)
    assert not pose.valid

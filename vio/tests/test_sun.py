"""Sun cue: ray geometry, solar-vector convention, detector, heading correction, observability, gating, no leakage."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src.data.synthetic import ImuNoise, make_synthetic_trajectory
from src.data.trajectory import quat_wxyz_to_rotation
from vio.bias_experiment import DEG, run_on_trajectory
from vio.estimation.gyro_bias_kf import BiasKFConfig, SunObservation, run_bias_kf
from vio.evaluation.attitude_metrics import tilt_heading_error_deg
from vio.sun.detector import azimuth_elevation_ned, camera_to_body_ray, detect_sun, pixel_to_camera_ray, sun_vector_ned
from vio.vision.camera import pinhole_intrinsics

K = pinhole_intrinsics(512, 512, 90.0)
R_BC = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
SUN_W = sun_vector_ned(60.0, 20.0)


# ---------------------------------------------------------------- geometry
def test_principal_point_ray():
    np.testing.assert_allclose(pixel_to_camera_ray(256, 256, K), [0, 0, 1], atol=1e-12)


def test_known_pixel_ray():
    s = pixel_to_camera_ray(256 + K[0, 0], 256, K)  # one focal length right: 45 deg
    np.testing.assert_allclose(s, [np.sqrt(0.5), 0, np.sqrt(0.5)], atol=1e-12)


def test_camera_to_body_ray():
    np.testing.assert_allclose(camera_to_body_ray([0, 0, 1], R_BC), [1, 0, 0])  # optical axis = body forward
    np.testing.assert_allclose(camera_to_body_ray([0, -1, 0], R_BC), [0, 0, -1])  # image up = body up (NED: -z)


def test_solar_vector_convention_ned():
    np.testing.assert_allclose(sun_vector_ned(0, 0), [1, 0, 0], atol=1e-12)  # north on the horizon
    np.testing.assert_allclose(sun_vector_ned(90, 0), [0, 1, 0], atol=1e-12)  # east
    np.testing.assert_allclose(sun_vector_ned(0, 90), [0, 0, -1], atol=1e-12)  # zenith is -z in NED
    az, el = azimuth_elevation_ned(sun_vector_ned(237.0, 12.5))
    assert az == pytest.approx(237.0) and el == pytest.approx(12.5)


# ---------------------------------------------------------------- detector
def _scene():
    img = np.zeros((512, 512, 3), np.uint8)
    img[:256] = (230, 170, 120)  # blue sky (BGR)
    img[256:] = (40, 120, 60)  # green ground
    return img


def test_detector_finds_sun_bloom_in_sky():
    import cv2
    img = _scene()
    cv2.circle(img, (300, 100), 40, (235, 235, 235), -1)  # bloom
    cv2.circle(img, (300, 100), 14, (255, 255, 255), -1)  # clipped core
    det, sky, _ = detect_sun(img)
    assert det is not None and abs(det.u - 300) < 3 and abs(det.v - 100) < 3 and sky > 0.4


def test_detector_rejects_small_cloud_and_ground_highlight():
    import cv2
    img = _scene()
    cv2.circle(img, (100, 80), 2, (255, 255, 255), -1)  # tiny clipped cloud fragment
    cv2.circle(img, (300, 400), 20, (255, 255, 255), -1)  # bright highlight on the ground, no bloom, no sky
    det, _, _ = detect_sun(img)
    assert det is None


# ---------------------------------------------------------------- filter
def _flight(bias_deg_s, frame="world", duration=60.0):
    return make_synthetic_trajectory("constant_velocity", duration=duration, gyroscope_frame=frame,
                                     noise=ImuNoise(gyro_bias=tuple(np.asarray(bias_deg_s) * DEG)))


def _sun_obs(traj, sun_w=SUN_W, error_deg=0.0, noise_deg=0.0, every=20, seed=0):
    rng = np.random.default_rng(seed)
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    obs = []
    for k in range(traj.index_at(5.0) + every, len(traj), every):
        s = R[k].inv().apply(sun_w)
        if error_deg or noise_deg:
            axis = np.cross(s, [0, 0, 1.0]) if abs(s[2]) < 0.9 else np.cross(s, [1.0, 0, 0])
            s = Rotation.from_rotvec(axis / np.linalg.norm(axis) * np.radians(error_deg + rng.normal(0, noise_deg))).apply(s)
        obs.append(SunObservation(k, s, 1.0))
    return obs


def _run(traj, grav=False, sun=False, obs=None, sun_w=SUN_W):
    from vio.estimation.gyro_bias_kf import BiasKFConfig as C
    k0 = traj.index_at(5.0)
    from src.estimation.inertial_dead_reckoning import NavState
    cfg = C(gravity_update=grav, gravity_sigma_deg=1.0, sun_update=sun, sun_sigma_deg=1.0, gyro_noise_density=0.0, bias_walk=1e-6)
    out = run_bias_kf(traj.timestamp[k0:], traj.accelerometer[k0:], traj.gyroscope[k0:], traj.gyroscope_frame, traj.gravity_world,
                      NavState(traj.position_gt[k0], traj.velocity_gt[k0], traj.attitude_gt[k0]), k0, [], cfg,
                      sun_observations=obs if obs is not None else (_sun_obs(traj, sun_w) if sun else None), sun_world=sun_w)
    tilt, head = tilt_heading_error_deg(out.result.attitude, traj.attitude_gt[k0:], traj.gravity_world)
    return out, tilt, np.abs(head)


def test_sun_fixes_heading_drift_gravity_cannot():
    traj = _flight([0, 0, 0.1])  # heading drift only
    _, _, h_none = _run(traj)
    _, _, h_grav = _run(traj, grav=True)
    _, _, h_sun = _run(traj, sun=True)
    assert h_none[-1] > 4.0 and h_grav[-1] > 0.9 * h_none[-1]
    assert h_sun[-1] < 0.15 * h_none[-1]  # one vector: 2 DOF; some error leaks through the unobserved axis


def test_sun_does_not_destabilise_tilt():
    traj = _flight([0.1, 0, 0])  # tilt drift only
    _, t_grav, _ = _run(traj, grav=True)
    _, t_both, _ = _run(traj, grav=True, sun=True)
    assert t_grav[-1] < 0.1 and t_both[-1] < 0.1


def test_gravity_plus_sun_beats_either_alone():
    traj = _flight([0.08, -0.06, 0.1])
    tot = lambda r: np.hypot(r[1][-1], r[2][-1])  # noqa: E731
    g, s, b = _run(traj, grav=True), _run(traj, sun=True), _run(traj, grav=True, sun=True)
    assert tot(b) < tot(g) and tot(b) <= tot(s) + 1e-6 and tot(b) < 0.3


def test_sun_unavailable_falls_back():
    traj = _flight([0.08, -0.06, 0.1])
    a, _, _ = _run(traj, grav=True)
    b, _, _ = _run(traj, grav=True, sun=True, obs=[])
    np.testing.assert_array_equal(a.result.attitude, b.result.attitude)


def test_bad_sun_is_rejected():
    traj = _flight([0, 0, 0.1])
    a, _, _ = _run(traj, grav=True)
    b, _, _ = _run(traj, grav=True, sun=True, obs=_sun_obs(traj, error_deg=40.0))
    assert b.sun and not any(x.accepted for x in b.sun)
    np.testing.assert_allclose(a.result.attitude, b.result.attitude, atol=1e-12)


def test_low_confidence_sun_is_skipped():
    traj = _flight([0, 0, 0.1])
    obs = [SunObservation(o.imu_index, o.s_body, 0.1) for o in _sun_obs(traj)]
    b, _, _ = _run(traj, sun=True, obs=obs)
    assert {x.reason for x in b.sun} == {"low confidence"}


def test_sun_parallel_to_gravity_gives_no_heading_and_stays_stable():
    traj = _flight([0, 0, 0.1])
    zenith = np.array([0, 0, -1.0])
    _, _, h_none = _run(traj)
    out, t, h = _run(traj, grav=True, sun=True, sun_w=zenith)
    assert np.all(np.isfinite(out.result.attitude)) and t[-1] < 0.1
    assert h[-1] > 0.9 * h_none[-1]  # rotation about the (vertical) sun ray is unobservable


def test_sun_updates_attitude_not_position():
    """No position/velocity state: an accepted update cannot move the position at that instant."""
    traj = _flight([0, 0, 0.1])
    a, _, _ = _run(traj)
    b, _, _ = _run(traj, sun=True)
    first = next(x.imu_index for x in b.sun if x.accepted) - b.result.start_index
    np.testing.assert_allclose(a.result.position[: first + 1], b.result.position[: first + 1], atol=1e-9)
    assert not np.allclose(a.result.attitude[first:], b.result.attitude[first:])


def test_no_ground_truth_after_cutoff():
    traj = _flight([0.08, -0.06, 0.1], duration=30.0)
    obs = _sun_obs(traj)
    a, _, _ = _run(traj, grav=True, sun=True, obs=obs)
    k0 = traj.index_at(5.0)
    traj.attitude_gt[k0 + 1:] = [1.0, 0, 0, 0]
    traj.position_gt[k0 + 1:] += 10.0
    b, _, _ = _run(traj, grav=True, sun=True, obs=obs)
    np.testing.assert_array_equal(a.result.attitude, b.result.attitude)

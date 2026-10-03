"""Dataset adapters for the visual-rotation diagnostic. Frame handling stays here.

Common front end (identical parameters): Shi-Tomasi + Lucas-Kanade keyframe tracker
(vio/vision/feature_tracker.py, tracker settings from midair_vio.yaml) with the keyframe age set
to the frame count closest to 0.5 s, then essential matrix + RANSAC on the keyframe-to-frame
correspondences (vio/vision/relative_pose.py, same PoseConfig). Only full-length spans are used:
non-overlapping intervals of ~0.5 s. Camera-specific differences: intrinsics, distortion (NTU
points are undistorted before the essential matrix), image size and frame rate.

Mid-Air: color_left (forward), 25 Hz, 1024 px read at 512 px, 90 deg pinhole, no distortion.
  Reference: ground-truth attitude (w, x, y, z), body->world, NED. Frame i at IMU sample 4i.
  (Mid-Air's world-frame gyro is irrelevant here: the reference is the GT attitude.)
NTU VIRAL: /left/image_raw, 10 Hz, 752x480 mono, radtan distortion, T_Body_Cam from
  camera_left.yaml, t_imu = t_cam + t_shift. NO ground-truth attitude exists (the Leica tracker
  measures prism position only). Reference rotation: the VN100 orientation output (/imu/imu).
  Checked: it is body->world and the gyro is body-frame (body-side integration matches it, world
  side does not), but the RAW gyro carries a constant ~(0.40, 0.21, 0.1) deg/s body-frame bias that
  the orientation output has removed, so the orientation output is the reference. rtp_01's left
  camera is saturated in the bag; its right camera is used instead.
  Position: Leica prism (/leica/pose/relative); the 0.4 m prism lever arm is ignored for speed.
"""
from __future__ import annotations

import pickle
from dataclasses import replace
from itertools import islice
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

from vio.diagnostics.visual_rotation import VisualInterval
from vio.vision.feature_tracker import KeyframeTracker, TrackerConfig
from vio.vision.relative_pose import PoseConfig, estimate_relative_pose

INTERVAL_S = 0.5


def frames_for(rate_hz: float, interval_s: float = INTERVAL_S) -> int:
    """Frame count closest to the interval (time-based)."""
    return max(1, int(np.floor(interval_s * rate_hz + 0.5)))


def intervals_from_tracks(tracks, times: np.ndarray, K: np.ndarray, pose_cfg: PoseConfig, age: int,
                          undistort=None) -> list[VisualInterval]:
    """Full-length keyframe spans -> relative camera rotations. ``times[frame_index]`` in seconds."""
    out = []
    for tr in tracks:
        if getattr(tr, "reanchor", "") != "age" or tr.frame_index - tr.keyframe_index != age:
            continue
        a, b = tr.keyframe_points.astype(np.float64), tr.current_points.astype(np.float64)
        if undistort is not None and len(a):
            a, b = undistort(a), undistort(b)
        pose = estimate_relative_pose(a, b, K, pose_cfg)
        if not pose.valid:
            continue
        spread = float(np.sqrt(np.trace(np.cov(b.T)))) if len(b) > 2 else float("nan")
        # translational parallax: flow left after removing the estimated rotation (pixels)
        Kinv = np.linalg.inv(K)
        na, nb = (np.c_[a, np.ones(len(a))] @ Kinv.T)[:, :2], (np.c_[b, np.ones(len(b))] @ Kinv.T)[:, :2]
        rays = np.c_[na, np.ones(len(na))] @ pose.rotation  # rotate rays of frame i into frame j: R_ab^T x
        par = float(np.median(np.linalg.norm(nb - rays[:, :2] / rays[:, 2:3], axis=1)) * K[0, 0])
        out.append(VisualInterval(float(times[tr.keyframe_index]), float(times[tr.frame_index]), pose.rotation,
                                  pose.n_correspondences, pose.n_inliers, pose.inlier_ratio, pose.median_flow_px, spread,
                                  pose.translation_dir, par))
    return out


# ------------------------------------------------------------------ Mid-Air
def midair_tracks(traj, frames_dir: Path, vio_cfg: dict, cache: Path):
    """Track the whole flight with keyframe age = 0.5 s at 25 Hz. Cached per flight."""
    from vio.data.midair_camera import open_frames
    age = frames_for(25.0)
    cache.parent.mkdir(parents=True, exist_ok=True)
    if cache.is_file():
        return pickle.loads(cache.read_bytes()), age
    tcfg = replace(TrackerConfig(**vio_cfg["tracker"]), max_keyframe_age=age)
    src = open_frames(Path(traj.metadata["file"]), traj.metadata["trajectory"], "color_left", frames_dir)
    n = min(len(src), (len(traj) - 1) // 4 + 1)
    tracker = KeyframeTracker(tcfg)
    tracks = [tracker.process(img, i) for i, img in islice(src.iter_gray(0, 2), n)]
    cache.write_bytes(pickle.dumps(tracks))
    return tracks, age


def midair_rows(traj, cond: str, tracks, age: int, vio_cfg: dict):
    from vio.diagnostics.visual_rotation import interval_row
    from src.data.trajectory import quat_wxyz_to_rotation
    from vio.vision.camera import pinhole_intrinsics
    K = pinhole_intrinsics(512, 512, vio_cfg["cameras"]["left"]["hfov_deg"])
    R_bc = np.asarray(vio_cfg["cameras"]["left"]["R_bc"], dtype=float)
    times = np.arange(len(tracks) + 1) * 0.04
    ivs = intervals_from_tracks(tracks, times, K, PoseConfig(**vio_cfg["pose"]), age)
    R = quat_wxyz_to_rotation(traj.attitude_gt)
    rows = []
    for iv in ivs:
        i, j = int(round(iv.t_i * 100)), int(round(iv.t_j * 100))
        if j >= len(traj):
            continue
        rows.append(interval_row("midair", cond, traj.metadata["trajectory"], iv, R[i], R[j],
                                 traj.position_gt[i], traj.position_gt[j], R_bc))
    return rows


# ------------------------------------------------------------------ NTU VIRAL
def ntu_calibration(seq_dir: Path, camera: str = "left") -> dict:
    def load(name):
        txt = (seq_dir / name).read_text()
        txt = "\n".join(l for l in txt.splitlines() if not l.startswith("%YAML")).replace("!!opencv-matrix", "")
        return yaml.safe_load(txt)
    cam = load(f"camera_{camera}.yaml")
    T = np.array((cam.get("T_Body_Cam") or cam.get("T_Body2Cam"))["data"], dtype=float).reshape(4, 4)
    d, p = cam["distortion_parameters"], cam["projection_parameters"]
    return {"K": np.array([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1.0]]),
            "D": np.array([d["k1"], d["k2"], d["p1"], d["p2"]]), "R_bc": T[:3, :3], "t_shift": float(cam["t_shift"]),
            "size": (int(cam["image_width"]), int(cam["image_height"]))}


def ntu_read_bag(bag: Path, cache: Path, image_topic: str = "/left/image_raw") -> dict:
    """IMU orientation + gyro, Leica positions, image timestamps; images streamed to an .npz cache."""
    if cache.is_file():
        d = np.load(cache, allow_pickle=False)
        return {k: d[k] for k in d.files}
    from rosbags.highlevel import AnyReader
    imu_t, imu_q, imu_w, imu_a, pos_t, pos, img_t, imgs = [], [], [], [], [], [], [], []
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic in ("/imu/imu", "/leica/pose/relative", image_topic)]
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if conn.topic == "/imu/imu":
                o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
                imu_t.append(t), imu_q.append([o.x, o.y, o.z, o.w]), imu_w.append([w.x, w.y, w.z]), imu_a.append([a.x, a.y, a.z])
            elif conn.topic == "/leica/pose/relative":
                pt = msg.pose.position if hasattr(msg, "pose") else msg.point  # PoseStamped in the bags, PointStamped in the docs
                pos_t.append(t), pos.append([pt.x, pt.y, pt.z])
            else:
                img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
                img_t.append(t), imgs.append(img)
    out = {"imu_t": np.array(imu_t), "imu_q_xyzw": np.array(imu_q), "imu_w": np.array(imu_w), "imu_a": np.array(imu_a),
           "pos_t": np.array(pos_t), "pos": np.array(pos), "img_t": np.array(img_t), "imgs": np.stack(imgs)}
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez(cache, **out)
    return out


def ntu_validate_orientation(d: dict) -> dict:
    """Is /imu/imu orientation body->world, and is the gyro in body axes? Compare 0.5 s rotations."""
    t, R, w = d["imu_t"], Rotation.from_quat(d["imu_q_xyzw"]), d["imu_w"]
    res = {"body_right": [], "world_left": []}
    step = int(round(0.5 / np.median(np.diff(t))))
    for i in range(0, len(t) - step, step * 4):
        Rb, Rw = R[i], R[i]
        for k in range(i, i + step):
            inc = Rotation.from_rotvec(0.5 * (w[k] + w[k + 1]) * (t[k + 1] - t[k]))
            Rb, Rw = Rb * inc, inc * Rw
        res["body_right"].append((Rb.inv() * R[i + step]).magnitude())
        res["world_left"].append((Rw.inv() * R[i + step]).magnitude())
    return {k: float(np.degrees(np.median(v))) for k, v in res.items()}


def gyro_integrated_attitude(t: np.ndarray, w: np.ndarray, R0: Rotation) -> Rotation:
    """Body-frame gyro integrated from R0 (midpoint rule). Relative rotations between any two
    times are pure gyro integration, independent of R0."""
    quats = np.empty((len(t), 4))
    R = R0
    quats[0] = R.as_quat()
    incs = Rotation.from_rotvec(0.5 * (w[:-1] + w[1:]) * np.diff(t)[:, None])
    for k in range(len(t) - 1):
        R = R * incs[k]
        quats[k + 1] = R.as_quat()
    return Rotation.from_quat(quats)


def ntu_visual_intervals(d: dict, cal: dict, vio_cfg: dict, max_seconds: float | None = None):
    """Same front end as Mid-Air on the NTU images (points undistorted before the essential matrix).
    Returns (intervals with times on the IMU clock, image rate, frames per interval, frames processed)."""
    img_t = d["img_t"] + cal["t_shift"]  # camera stamps -> IMU clock
    rate = 1.0 / np.median(np.diff(img_t))
    age = frames_for(rate)
    tracker = KeyframeTracker(replace(TrackerConfig(**vio_cfg["tracker"]), max_keyframe_age=age))
    n = len(img_t) if max_seconds is None else int(np.searchsorted(img_t, img_t[0] + max_seconds))
    tracks = [tracker.process(d["imgs"][i], i) for i in range(n)]
    K, D = cal["K"], cal["D"]
    undist = lambda p: cv2.undistortPoints(p.reshape(-1, 1, 2), K, D, P=K).reshape(-1, 2)  # noqa: E731
    return intervals_from_tracks(tracks, img_t, K, PoseConfig(**vio_cfg["pose"]), age, undistort=undist), rate, age, n


def ntu_rows(seq: str, d: dict, cal: dict, vio_cfg: dict, max_seconds: float | None = None, reference: str = "orientation"):
    """``reference``: 'orientation' (VN100 output, bias-compensated; primary) or 'gyro' (raw gyro
    integration; carries an uncompensated ~0.47 deg/s bias in NTU VIRAL, secondary)."""
    from vio.diagnostics.visual_rotation import interval_row
    ivs, rate, age, n = ntu_visual_intervals(d, cal, vio_cfg, max_seconds)
    q_t, Rq = d["imu_t"], Rotation.from_quat(d["imu_q_xyzw"])
    slerp = Slerp(q_t, Rq)
    gslerp = Slerp(q_t, gyro_integrated_attitude(q_t, d["imu_w"], Rq[0])) if reference == "gyro" else None
    p_t, P = d["pos_t"], d["pos"]
    rows = []
    for iv in ivs:
        if not (q_t[0] <= iv.t_i and iv.t_j <= q_t[-1] and p_t[0] <= iv.t_i and iv.t_j <= p_t[-1]):
            continue
        Ri, Rj = slerp([iv.t_i, iv.t_j])  # VN100 orientation: absolute frame for the world-axes view
        if gslerp is not None:  # relative rotation from the gyro, anchored at the VN100 attitude at t_i
            Gi, Gj = gslerp([iv.t_i, iv.t_j])
            Rj = Ri * (Gi.inv() * Gj)
        pi = np.array([np.interp(iv.t_i, p_t, P[:, k]) for k in range(3)])
        pj = np.array([np.interp(iv.t_j, p_t, P[:, k]) for k in range(3)])
        rows.append(interval_row("ntu", "viral", seq, iv, Ri, Rj, pi, pj, cal["R_bc"]))
    return rows, {"image_rate_hz": float(rate), "frames_per_interval": age, "frames_processed": n}

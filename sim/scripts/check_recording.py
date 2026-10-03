"""Check a Mid-Air-format recording: shapes, files, and that every sensor agrees with the ground truth and the pictures.

    python3 sim/scripts/check_recording.py data/sim/islands/sunny [trajectory_XXXX ...]

Each line says what was compared and prints PASS or FAIL with the number behind it. Run it in the container.
"""
import json
import sys
from pathlib import Path

import cv2
import h5py
import numpy as np
from scipy.spatial.transform import Rotation

SIM = Path(__file__).resolve().parents[1]
G = np.array([0, 0, 9.81])


def rms(x):
    return float(np.sqrt(np.mean(np.sum(np.atleast_2d(x) ** 2, 1))))


def check(ok, text):
    print(f"  {'PASS' if ok else 'FAIL'}  {text}")
    return ok


def over_land(north, east):
    """True where the islands world's elevation grid is above the sea at these points."""
    meta = json.loads((SIM / "maps" / "islands_dem.json").read_text())
    dem = cv2.imread(str(SIM / "maps" / "islands_dem.tif"), cv2.IMREAD_UNCHANGED)
    c = meta["pixel_centres_world_m"]
    col = np.clip(np.round((east - c["x_first"]) / meta["resolution_m"]).astype(int), 0, dem.shape[1] - 1)
    row = np.clip(np.round((c["y_top"] - north) / meta["resolution_m"]).astype(int), 0, dem.shape[0] - 1)
    return dem[row, col] > 0.3


def main():
    folder = Path(sys.argv[1]).resolve()
    with h5py.File(folder / "sensor_records.hdf5", "r") as f:
        names = sys.argv[2:] or sorted(f)
        bad = 0
        for name in names:
            t = f[name]
            print(f"\n{name}")
            paths = [p.decode() if isinstance(p, bytes) else p for p in t["camera_data/color_down"][:]]
            n_frames = len(paths)
            n_s = n_frames // 25
            pos, vel, acc = (t[f"groundtruth/{k}"][:] for k in ("position", "velocity", "acceleration"))
            q, ang = t["groundtruth/attitude"][:], t["groundtruth/angular_velocity"][:]
            accel, gyro = t["imu/accelerometer"][:], t["imu/gyroscope"][:]
            gps = t["gps/position"][:]

            # Shapes and files, as Mid-Air: 25N pictures, 100N rows at 100 Hz, N GPS rows
            shapes = [len(x) for x in (pos, vel, acc, q, ang, accel, gyro)]
            ok = n_frames % 25 == 0 and all(s == 4 * n_frames for s in shapes) and len(gps) == n_s
            bad += not check(ok, f"{n_s} s: {n_frames} pictures, {shapes[0]} rows at 100 Hz, {len(gps)} GPS rows")
            missing = [p for p in paths if not (folder / p).exists()]
            bad += not check(not missing, f"every listed picture exists ({len(missing)} missing)")
            on_disk = len(list((folder / "color_down" / name).glob("*.JPEG")))
            bad += not check(on_disk == n_frames, f"no extra pictures on disk ({on_disk} files)")
            nan = sum(int(np.isnan(t[k][:]).sum()) for k in ("groundtruth/position", "groundtruth/attitude", "imu/accelerometer", "imu/gyroscope", "gps/position"))
            bad += not check(nan == 0, f"no missing values ({nan} NaN)")

            # Ground truth is consistent with itself
            bad += not check(np.abs(np.linalg.norm(q, axis=1) - 1).max() < 1e-6, "attitude quaternions have length 1")
            v_num = np.gradient(pos, 0.01, axis=0)
            bad += not check(rms(v_num - vel) < 0.05, f"velocity is the rate of change of position ({rms(v_num - vel):.3f} m/s)")
            speed = np.linalg.norm(vel[:, :2], axis=1)
            bad += not check(speed.max() < 12, f"horizontal speed {speed.mean():.1f} m/s on average, at most {speed.max():.1f}")
            height = -pos[:, 2]
            print(f"        height above helipad A {height.min():.1f} to {height.max():.1f} m")

            # IMU against the ground truth, using the Mid-Air rules (docs/findings.md 2.2)
            rot = Rotation.from_quat(q[:, [1, 2, 3, 0]])
            accel_err = rms(accel - rot.inv().apply(acc - G))
            bad += not check(accel_err < 0.5, f"accelerometer matches the true motion in the drone's axes ({accel_err:.3f} m/s^2 off, noise)")
            gyro_err = rms(gyro - ang)
            bad += not check(gyro_err < 0.05, f"gyroscope matches the true turn rate ({gyro_err:.4f} rad/s off, noise)")
            world = (rot[1:] * rot[:-1].inv()).as_rotvec() * 100
            drone = (rot[:-1].inv() * rot[1:]).as_rotvec() * 100
            w_err, d_err = rms(ang[:-1] - world), rms(ang[:-1] - drone)
            bad += not check(w_err < d_err or d_err < 1e-3, f"turn rates are around the world axes, as in the Mid-Air files ({w_err:.4f} vs {d_err:.4f} for the drone's axes)")
            r = rot[0]
            for k in range(0, len(ang) - 1):
                r = Rotation.from_rotvec(ang[k] * 0.01) * r
            drift = np.degrees((r * rot[-1].inv()).magnitude())
            bad += not check(drift < 1, f"Mid-Air's turn rule on the true rates rebuilds the attitude ({drift:.4f} deg off at the end)")

            # GPS against the ground truth
            err = gps - pos[::100][:len(gps)]
            h = np.linalg.norm(err[:, :2], axis=1)
            bad += not check(np.median(h) < 6, f"GPS horizontal error {np.median(h):.1f} m median, {h.max():.1f} m at most (1 Hz fix, sigma 1.5 m)")
            bad += not check(np.median(np.abs(err[:, 2])) < 8, f"GPS vertical error {np.median(np.abs(err[:, 2])):.1f} m median")

            # Barometer: pressure falls by about 12 Pa per metre of climb
            if "barometer" in t:
                p = t["barometer/pressure"][:, 0]
                slope = np.polyfit(height, p, 1)[0] if np.ptp(height) > 2 else -12.0
                bad += not check(-14 < slope < -10, f"barometer falls {-slope:.1f} Pa per metre of height")

            # Bias attributes
            bad += not check("init_bias_est" in t["imu/accelerometer"].attrs and "init_bias_est" in t["imu/gyroscope"].attrs,
                             "IMU initial bias estimates present")

            # Pictures against the position: water in the picture where the map says sea
            idx = np.linspace(0, n_frames - 1, min(200, n_frames)).astype(int)
            water = []
            for k in idx:
                img = cv2.imread(str(folder / paths[k]))
                c = img[img.shape[0] // 2 - 40:img.shape[0] // 2 + 40, img.shape[1] // 2 - 40:img.shape[1] // 2 + 40].reshape(-1, 3).mean(0)
                water.append(c[0] > c[2] + 25)  # BGR: clearly bluer than red
            land = over_land(pos[4 * idx, 0], pos[4 * idx, 1])
            agree = np.mean(np.array(water) != land)
            bad += not check(agree > 0.9, f"pictures show sea where the map has sea under the drone ({100 * agree:.0f}% of {len(idx)} pictures; "
                                          f"{100 * np.mean(land):.0f}% over land)")
        print(f"\n{'all checks passed' if not bad else f'{bad} checks failed'}")
        sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()

"""How large are the IMU errors in Mid-Air?

Compares the IMU readings with the true motion for every flight in one sensor file.
Run from the repository root:  python experiments/e_midair_imu_noise.py [path to sensor_records.hdf5]

The accelerometer measures along the drone's own axes and includes gravity, so the
true value is  R^T (a_world - g)  with g = (0, 0, 9.81) in North, East, Down.
"""
import sys

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

PATH = sys.argv[1] if len(sys.argv) > 1 else "data/raw/midair/MidAir/Kite_training/sunny/sensor_records.hdf5"
GRAVITY = np.array([0.0, 0.0, 9.81])
WINDOW = 200  # samples, 2 s at 100 Hz: slower than this counts as offset, faster as white noise


def split(error: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split an error signal into its slow part (moving mean) and the rest."""
    c = np.cumsum(np.vstack([np.zeros(3), error]), axis=0)
    slow = (c[WINDOW:] - c[:-WINDOW]) / WINDOW
    fast = error[WINDOW // 2 : WINDOW // 2 + len(slow)] - slow
    return slow, fast


rows = []
with h5py.File(PATH, "r") as f:
    for name in sorted(f.keys()):
        t = f[name]
        q = t["groundtruth/attitude"][:]  # w, x, y, z
        body_to_world = Rotation.from_quat(q[:, [1, 2, 3, 0]])
        true_accel = body_to_world.inv().apply(t["groundtruth/acceleration"][:] - GRAVITY)
        accel_error = t["imu/accelerometer"][:] - true_accel
        gyro_error = t["imu/gyroscope"][:] - t["groundtruth/angular_velocity"][:]
        accel_slow, accel_fast = split(accel_error)
        gyro_slow, gyro_fast = split(gyro_error)
        rows.append(
            [
                np.abs(accel_error.mean(0)).max(),
                accel_fast.std(0).mean(),
                np.abs(accel_slow[-1] - accel_slow[0]).max(),
                np.abs(gyro_error.mean(0)).max(),
                gyro_fast.std(0).mean(),
                np.abs(gyro_slow[-1] - gyro_slow[0]).max(),
            ]
        )

r = np.array(rows)
labels = [
    ("accelerometer constant offset, worst axis", "m/s^2"),
    ("accelerometer white noise per sample", "m/s^2"),
    ("accelerometer offset change over the flight", "m/s^2"),
    ("gyroscope constant offset, worst axis", "rad/s"),
    ("gyroscope white noise per sample", "rad/s"),
    ("gyroscope offset change over the flight", "rad/s"),
]
print(f"{PATH}: {len(r)} flights")
for i, (label, unit) in enumerate(labels):
    print(f"{label:46s} min {r[:, i].min():.4f}  median {np.median(r[:, i]):.4f}  max {r[:, i].max():.4f} {unit}")

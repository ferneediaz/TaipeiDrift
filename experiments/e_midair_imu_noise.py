"""How large are the IMU errors in Mid-Air?

Compares the IMU readings with the true motion for every flight in one sensor file.
Run from the repository root:  python experiments/e_midair_imu_noise.py [path to sensor_records.hdf5]

The accelerometer measures along the drone's own axes and includes gravity, so the
true value is  R^T (a_world - g)  with g = (0, 0, 9.81) in North, East, Down.

The script also checks which axes the turn rates are given in. In Mid-Air the gyroscope
and the true angular velocity fit the attitude only as rates around the map's axes.
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


def rms(x: np.ndarray) -> float:
    return float(np.sqrt((x**2).mean()))


rows, axes = [], []
with h5py.File(PATH, "r") as f:
    for name in sorted(f.keys()):
        t = f[name]
        q = t["groundtruth/attitude"][:]  # w, x, y, z
        body_to_world = Rotation.from_quat(q[:, [1, 2, 3, 0]])
        true_accel = body_to_world.inv().apply(t["groundtruth/acceleration"][:] - GRAVITY)
        accel_error = t["imu/accelerometer"][:] - true_accel
        gyro_error = t["imu/gyroscope"][:] - t["groundtruth/angular_velocity"][:]
        # Which axes are the turn rates given in? Compare with the change of the true attitude
        # from one sample to the next, once expressed in the drone's axes and once in map axes.
        in_drone_axes = (body_to_world[:-1].inv() * body_to_world[1:]).as_rotvec() / 0.01
        in_map_axes = (body_to_world[1:] * body_to_world[:-1].inv()).as_rotvec() / 0.01
        axes.append(
            [
                rms(t["groundtruth/angular_velocity"][:-1] - in_drone_axes),
                rms(t["groundtruth/angular_velocity"][:-1] - in_map_axes),
                rms(t["imu/gyroscope"][:-1] - in_drone_axes),
                rms(t["imu/gyroscope"][:-1] - in_map_axes),
                rms(accel_error),
                rms(t["imu/accelerometer"][:] - (t["groundtruth/acceleration"][:] - GRAVITY)),
                rms(in_drone_axes),
            ]
        )
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

a = np.median(np.array(axes), axis=0)
print(f"\nWhich axes? Difference to the change of the true attitude (rms, median over flights; typical turn rate {a[6]:.2f} rad/s):")
print(f"true turn rate:  as rates in the drone's axes {a[0]:.4f}, as rates in map axes {a[1]:.4f} rad/s")
print(f"gyroscope:       as rates in the drone's axes {a[2]:.4f}, as rates in map axes {a[3]:.4f} rad/s")
print(f"accelerometer against true acceleration minus gravity: in the drone's axes {a[4]:.3f}, in map axes {a[5]:.3f} m/s^2")

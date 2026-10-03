"""IMU-only dead reckoning on Mid-Air: how fast does the position drift once GNSS is lost?

Scenario of the plan: GNSS works for the first 10 seconds of a flight, then it is jammed.
From that moment the position is computed from the IMU alone:
  1. the gyroscope turns the attitude forward,
  2. the attitude turns the accelerometer reading into north, east, down, and gravity is added back,
  3. acceleration is summed into speed, and speed into position.
Position, speed and attitude at the moment of the jam are the true ones.
Run from the repository root:  python experiments/j_midair_imu_only.py [path to sensor_records.hdf5]
"""
import os
import sys

import h5py
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

PATH = sys.argv[1] if len(sys.argv) > 1 else "data/raw/midair/MidAir/Kite_training/sunny/sensor_records.hdf5"
FIGURE = "docs/figures/midair_imu_only.png"
GRAVITY = np.array([0.0, 0.0, 9.81])  # North, East, Down
DT = 0.01
JAM_S = 10.0
SHOW = "trajectory_0003"


def dead_reckon(accel: np.ndarray, gyro: np.ndarray, attitude0: np.ndarray, position0: np.ndarray, velocity0: np.ndarray) -> np.ndarray:
    rot = Rotation.from_quat(attitude0[[1, 2, 3, 0]])  # Mid-Air stores w, x, y, z
    p, v = position0.copy(), velocity0.copy()
    path = np.empty((len(accel), 3))
    path[0] = p
    a = rot.apply(accel[0]) + GRAVITY
    for k in range(len(accel) - 1):
        # Mid-Air gives the turn rate in map axes, so the new rotation is applied from the left.
        # With the textbook formula for rates in the drone's axes (rot * step) the result is wrong.
        rot = Rotation.from_rotvec(0.5 * (gyro[k] + gyro[k + 1]) * DT) * rot
        a_next = rot.apply(accel[k + 1]) + GRAVITY
        a_mean = 0.5 * (a + a_next)  # average of two neighbouring samples, not one sample per step
        p = p + v * DT + 0.5 * a_mean * DT * DT
        v = v + a_mean * DT
        a = a_next
        path[k + 1] = p
    return path


j = int(JAM_S / DT)
errors, shares, check_errors, shown = [], [], [], None
with h5py.File(PATH, "r") as f:
    for name in sorted(f.keys()):
        t = f[name]
        position = t["groundtruth/position"][:]
        velocity = t["groundtruth/velocity"][:]
        attitude = t["groundtruth/attitude"][:]
        n = min(len(position), len(t["imu/accelerometer"]))
        estimate = dead_reckon(t["imu/accelerometer"][j:n], t["imu/gyroscope"][j:n], attitude[j], position[j], velocity[j])
        error = np.linalg.norm(estimate[:, :2] - position[j:n, :2], axis=1)
        flown = np.sum(np.linalg.norm(np.diff(position[j:n, :2], axis=0), axis=1))
        errors.append(error)
        shares.append(error[-1] / flown * 100)
        # the same code fed with the true motion: shows how much error the code itself adds
        true_force = Rotation.from_quat(attitude[j:n][:, [1, 2, 3, 0]]).inv().apply(t["groundtruth/acceleration"][j:n] - GRAVITY)
        check = dead_reckon(true_force, t["groundtruth/angular_velocity"][j:n], attitude[j], position[j], velocity[j])
        check_errors.append(np.linalg.norm(check[-1, :2] - position[n - 1, :2]))
        if name == SHOW:
            shown = (position[:n], estimate)

print(f"{PATH}: {len(errors)} flights, GNSS jammed after {JAM_S:.0f} s, IMU only for about {len(errors[0]) * DT:.0f} s")
print(f"check, true motion fed into the same code: error at the end median {np.median(check_errors):.2f} m, max {np.max(check_errors):.2f} m")
for seconds in (5, 10, 30, 60):
    e = np.array([err[int(seconds / DT)] for err in errors])
    print(f"error {seconds:2d} s after the jam: median {np.median(e):7.1f} m, smallest {e.min():6.1f} m, largest {e.max():7.1f} m")
e = np.array([err[-1] for err in errors])
print(f"error at the end of the flight: median {np.median(e):7.1f} m, smallest {e.min():6.1f} m, largest {e.max():7.1f} m")
print(f"error at the end as a share of the distance flown since the jam: median {np.median(shares):.0f}%")
passes = np.array([np.argmax(err > 50) * DT if (err > 50).any() else np.nan for err in errors])
print(f"time until the error passes 50 m: median {np.nanmedian(passes):.0f} s, fastest {np.nanmin(passes):.0f} s, slowest {np.nanmax(passes):.0f} s; flights that stay below 50 m: {np.isnan(passes).sum()}")

os.makedirs(os.path.dirname(FIGURE), exist_ok=True)
fig, (a, b) = plt.subplots(1, 2, figsize=(12, 5))
truth, estimate = shown
a.plot(truth[:, 1], truth[:, 0], color="black", lw=2, label="true path")
a.plot(estimate[:, 1], estimate[:, 0], color="tab:red", label="IMU only after the jam")
a.plot(truth[j, 1], truth[j, 0], "o", color="gray", label="GNSS jammed here")
a.set_xlabel("east (m)"), a.set_ylabel("north (m)"), a.set_aspect("equal", adjustable="datalim"), a.legend()
a.set_title(f"Mid-Air {SHOW}, sunny")
for err in errors:
    b.plot(np.arange(len(err)) * DT, err, color="gray", lw=0.6, alpha=0.6)
shortest = min(len(err) for err in errors)
b.plot(np.arange(shortest) * DT, np.median([err[:shortest] for err in errors], axis=0), color="tab:red", lw=2.5, label="median of 30 flights")
b.axhline(50, color="black", ls=":", label="50 m")
b.set_xlabel("seconds since the jam"), b.set_ylabel("position error (m)"), b.set_yscale("log"), b.set_ylim(0.1, 20000), b.grid(alpha=0.3), b.legend()
b.set_title("IMU-only error, every sunny flight")
fig.tight_layout()
fig.savefig(FIGURE, dpi=110)
print("figure:", FIGURE)

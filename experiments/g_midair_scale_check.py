"""Data check 1 of the plan: can a barometer give the scale for camera speed on Mid-Air?

Speed over ground = image motion x distance to the ground / focal length.
Mid-Air has no sensor for the distance to the ground. This script measures what that
distance would have to be (from the true speed), and tests the scenario of the plan:
learn the distance while GNSS still works (first 10 s), then follow it with the barometer.
Run from the repository root:  python experiments/g_midair_scale_check.py [flight numbers]
"""
import sys
import zipfile

import cv2
import h5py
import numpy as np

ROOT = "data/raw/midair/MidAir/Kite_training/sunny"
FLIGHTS = sys.argv[1:] or ["0000", "0001", "0002"]
FOCAL = 128.0  # pixels, for the 1024 px image read at quarter size (90 degree field of view)
FPS = 25.0
GNSS_SECONDS = 10.0  # GNSS is available this long, then jammed

sensors = h5py.File(f"{ROOT}/sensor_records.hdf5", "r")
for flight in FLIGHTS:
    t = sensors[f"trajectory_{flight}"]
    z = zipfile.ZipFile(f"{ROOT}/color_down/trajectory_{flight}/frames.zip")
    names = sorted(n for n in z.namelist() if n.endswith(".JPEG"))
    velocity = t["groundtruth/velocity"][::4]  # truth is at 100 Hz, the camera at 25 Hz
    down = t["groundtruth/position"][::4, 2]  # positive is down
    turn_rate = np.linalg.norm(t["groundtruth/angular_velocity"][::4, :2], axis=1)

    flow = np.full(len(names), np.nan)
    previous = None
    for k, name in enumerate(names):
        raw = np.frombuffer(z.read(name), np.uint8)
        frame = cv2.imdecode(raw, cv2.IMREAD_REDUCED_GRAYSCALE_4)
        if previous is not None:
            f = cv2.calcOpticalFlowFarneback(previous, frame, None, 0.5, 4, 21, 3, 7, 1.5, 0)
            centre = f[64:192, 64:192].reshape(-1, 2)
            flow[k] = np.linalg.norm(np.median(centre, axis=0)) * FPS  # pixels per second
        previous = frame

    n = min(len(names), len(velocity))
    flow, speed, height, turn = flow[:n], np.linalg.norm(velocity[:n, :2], axis=1), -down[:n], turn_rate[:n]
    time = np.arange(n) / FPS
    usable = (speed > 2.0) & (turn < 0.05) & (flow > 1.0)  # moving, not rotating
    distance = FOCAL * speed / flow  # the distance to the ground that makes the camera speed correct

    learn = usable & (time < GNSS_SECONDS)
    test = usable & (time >= GNSS_SECONDS)
    if learn.sum() < 10 or test.sum() < 50:
        print(f"flight {flight}: too few usable samples (learn {learn.sum()}, test {test.sum()})")
        continue
    d0, h0 = np.median(distance[learn]), np.median(height[learn])
    with_baro = flow * (d0 + height - h0) / FOCAL
    constant = flow * d0 / FOCAL
    err_baro = np.abs(with_baro[test] - speed[test]) / speed[test] * 100
    err_const = np.abs(constant[test] - speed[test]) / speed[test] * 100
    lo, mid, hi = np.percentile(distance[usable], [10, 50, 90])
    print(f"flight {flight}: {n / FPS:.0f} s, speed {np.median(speed):.1f} m/s, usable samples {usable.sum()} of {n}")
    print(f"  distance to the ground implied by the camera: median {mid:.0f} m, 10% to 90% range {lo:.0f} to {hi:.0f} m")
    print(f"  altitude change during the flight: {height.min() - height[0]:+.0f} to {height.max() - height[0]:+.0f} m")
    print(f"  speed error after the jam, distance learned in the first {GNSS_SECONDS:.0f} s then followed by barometer: median {np.median(err_baro):.0f}%, 90% below {np.percentile(err_baro, 90):.0f}%")
    print(f"  speed error after the jam, distance kept constant: median {np.median(err_const):.0f}%, 90% below {np.percentile(err_const, 90):.0f}%")

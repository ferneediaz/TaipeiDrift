"""Print every sensor value recorded with one down-camera picture of a Mid-Air-format recording.

    python3 sim/scripts/frame_info.py data/sim/islands/sunny/color_down/trajectory_0000/000123.JPEG

Picture j goes with row 4j of the 100 Hz records (ground truth, IMU, barometer) and row j // 25 of the 1 Hz GPS.
Works on the real Mid-Air dataset too. Run it in the container, which has h5py.
"""
import argparse
import math
from pathlib import Path

import h5py
import numpy as np


def euler_deg(q):
    """Roll, pitch, yaw in degrees from a w x y z quaternion (body FRD to world NED); yaw 0 is north, 90 east."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


def fmt(v, unit=""):
    return "  ".join(f"{x:10.4f}" for x in np.ravel(v)) + (f"  {unit}" if unit else "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("picture", type=Path, help=".../<climate>/color_down/trajectory_XXXX/NNNNNN.JPEG")
    args = ap.parse_args()
    pic = args.picture.resolve()
    frame, traj, climate = int(pic.stem), pic.parent.name, pic.parents[2]
    h5 = climate / "sensor_records.hdf5"
    if not h5.exists():
        raise SystemExit(f"no {h5} yet: it is written when the flight's recording ends")
    with h5py.File(h5, "r") as f:
        if traj not in f:
            raise SystemExit(f"{traj} is not in {h5} yet: it is written when that flight's recording ends")
        t = f[traj]
        row, gps_row = 4 * frame, frame // 25
        path = t["camera_data/color_down"][frame]
        print(f"{pic.name}: {traj}, frame {frame}, t = {frame / 25:.2f} s after the start of the recording")
        print(f"  listed as {path.decode() if isinstance(path, bytes) else path}")
        print(f"\nGround truth, row {row}  (north east down, m, from the world origin)")
        gt = t["groundtruth"]
        print(f"  position          {fmt(gt['position'][row], 'm')}")
        print(f"  velocity          {fmt(gt['velocity'][row], 'm/s')}")
        print(f"  acceleration      {fmt(gt['acceleration'][row], 'm/s^2')}")
        q = gt["attitude"][row]
        print(f"  attitude w x y z  {fmt(q)}")
        print(f"  roll pitch yaw    {fmt(euler_deg(q), 'deg (yaw 0 = north, 90 = east)')}")
        print(f"  angular velocity  {fmt(gt['angular_velocity'][row], 'rad/s, around north east down')}")
        print(f"\nIMU, row {row}  (accelerometer in the drone's axes: forward right down)")
        print(f"  accelerometer     {fmt(t['imu/accelerometer'][row], 'm/s^2')}")
        print(f"  gyroscope         {fmt(t['imu/gyroscope'][row], 'rad/s, around north east down, as Mid-Air')}")
        if "barometer" in t:
            print(f"\nBarometer, row {row}\n  pressure          {fmt(t['barometer/pressure'][row], 'Pa')}")
        gp = t["gps/position"][gps_row]
        err = np.linalg.norm(gp[:2] - gt["position"][100 * gps_row][:2])
        print(f"\nGPS, row {gps_row}  (the latest fix, from second {gps_row}; it updates once per second)")
        print(f"  position          {fmt(gp, 'm')}")
        print(f"  velocity          {fmt(t['gps/velocity'][gps_row], 'm/s')}")
        print(f"  horizontal error  {err:10.2f}  m against the ground truth at that second")
        if "origin_lat_lon_alt" in t.attrs:
            lat0, lon0, alt0 = t.attrs["origin_lat_lon_alt"]
            n, e, d = gt["position"][row]
            s2 = math.sin(math.radians(lat0)) ** 2  # WGS84 radii of curvature at the origin; good to well under a metre over a few km
            m_r = 6378137.0 * (1 - 6.69437999014e-3) / (1 - 6.69437999014e-3 * s2) ** 1.5
            n_r = 6378137.0 / math.sqrt(1 - 6.69437999014e-3 * s2)
            lat = lat0 + math.degrees(n / m_r)
            lon = lon0 + math.degrees(e / (n_r * math.cos(math.radians(lat0))))
            print(f"\nTrue location  {lat:.6f} N  {lon:.6f} E  {alt0 - d:.1f} m above the sea")


if __name__ == "__main__":
    main()

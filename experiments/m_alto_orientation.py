"""What do the orientation values in the ALTO data mean?

Needed for two things: turning the camera frame to north before matching when the aircraft
turns, and knowing how far the camera looks away from straight down.

The file gives one quaternion per camera frame, "with respect to the ECEF reference frame"
(axes fixed to the Earth). This script tries both readings of that sentence and reports, for
each camera axis, how far it points above the horizon and in which compass direction.
Run from the repository root:  python experiments/m_alto_orientation.py
"""
import zipfile

import numpy as np
import pandas as pd
from pyproj import Transformer
from scipy.spatial.transform import Rotation

ZIP = "data/raw/alto/Val.zip"

z = zipfile.ZipFile(ZIP)
query = pd.read_csv(z.open("Val/query.csv"))
xy = query[["easting", "northing"]].to_numpy()
lon, lat = Transformer.from_crs("EPSG:32617", "EPSG:4326", always_xy=True).transform(xy[:, 0], xy[:, 1])
la, lo = np.radians(lat), np.radians(lon)
# east, north and up at each position, written in Earth-fixed axes
east = np.stack([-np.sin(lo), np.cos(lo), np.zeros_like(lo)], axis=1)
north = np.stack([-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)], axis=1)
up = np.stack([np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)], axis=1)

rotation = Rotation.from_quat(query[["orient_x", "orient_y", "orient_z", "orient_w"]].to_numpy())
course = np.degrees(np.arctan2(np.gradient(xy[:, 0]), np.gradient(xy[:, 1])))
print(f"course over ground: median {np.median(course):.1f} degrees, from {course.min():.1f} to {course.max():.1f}")

readings = {
    "the quaternion turns camera axes into Earth axes": rotation,
    "the quaternion turns Earth axes into camera axes": rotation.inv(),
}
for label, r in readings.items():
    print(f"\nif {label}:")
    for name, axis in (("x", [1, 0, 0]), ("y", [0, 1, 0]), ("z", [0, 0, 1])):
        v = r.apply(axis)
        e, n, u = (v * east).sum(1), (v * north).sum(1), (v * up).sum(1)
        compass = np.degrees(np.arctan2(e, n))
        elevation = np.degrees(np.arcsin(np.clip(u, -1, 1)))
        spread = np.degrees(np.std(np.unwrap(np.radians(compass))))
        print(f"   camera {name}: {np.median(elevation):+6.1f} degrees above the horizon (spread {elevation.std():.1f}), compass direction {np.median(compass):+7.1f} degrees (spread {spread:.1f})")

# The first reading is the consistent one: x forward, y to the right, z down.
forward = rotation.apply([1, 0, 0])
down = rotation.apply([0, 0, 1])
heading = np.degrees(np.arctan2((forward * east).sum(1), (forward * north).sum(1)))
off_nadir = np.degrees(np.arccos(np.clip(-(down * up).sum(1), -1, 1)))
print(f"\nheading of the aircraft: median {np.median(heading):.1f} degrees, from {heading.min():.1f} to {heading.max():.1f}")
print(f"course minus heading (sideways drift in the wind): median {np.median(course - heading):+.1f} degrees")
print(f"the camera looks {np.median(off_nadir):.1f} degrees away from straight down (median), up to {off_nadir.max():.1f}")
print(f"expected rotation of the camera frame against north-up reference images, if the top of the frame is the aircraft's left: {np.median(heading) - 90:+.1f} degrees")

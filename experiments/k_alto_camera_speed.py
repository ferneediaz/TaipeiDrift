"""Camera speed on the real ALTO flight: does image motion give the distance flown,
and how long does a scale stay valid?

Ground step = image shift in pixels x metres per pixel. The metres per pixel (the scale)
depend on the height above ground, which ALTO does not contain. This script measures the
scale that would be correct at every frame, and then how wrong the speed gets when the
scale is taken from 100 m, 300 m or 1000 m earlier, as it would be after a position fix.
Run from the repository root:  python experiments/k_alto_camera_speed.py
"""
import os
import time
import zipfile

import cv2
import numpy as np
import pandas as pd

ZIP = "data/raw/alto/Val.zip"

z = zipfile.ZipFile(ZIP)
query = pd.read_csv(z.open("Val/query.csv"))
xy = query[["easting", "northing"]].to_numpy()
step = np.r_[np.nan, np.linalg.norm(np.diff(xy, axis=0), axis=1)]

flow = np.full(len(query), np.nan)
previous = None
t0 = time.time()
for k, name in enumerate(query.name):
    raw = np.frombuffer(z.read(f"Val/query_images/{name}"), np.uint8)
    frame = cv2.resize(cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE), (250, 250), interpolation=cv2.INTER_AREA)
    if previous is not None:
        f = cv2.calcOpticalFlowFarneback(previous, frame, None, 0.5, 4, 21, 3, 7, 1.5, 0)
        flow[k] = np.linalg.norm(np.median(f[60:190, 60:190].reshape(-1, 2), axis=0)) * 2  # pixels at full size
    previous = frame
ms = (time.time() - t0) / len(query) * 1000

scale = step / flow  # metres per pixel that makes the camera step correct
ok = np.isfinite(scale) & (flow > 0.5)
print(f"frames {ok.sum()} of {len(query)}, {ms:.0f} ms per frame")
print(f"image motion per frame: median {np.nanmedian(flow):.1f} px; true step {np.nanmedian(step):.2f} m")
lo, mid, hi = np.percentile(scale[ok], [10, 50, 90])
print(f"metres per pixel: median {mid:.3f}, 10% to 90% range {lo:.3f} to {hi:.3f}, so one frame covers {mid * 500:.0f} m")

smooth = pd.Series(scale).rolling(21, center=True, min_periods=5).median().to_numpy()  # the scale as a fix would deliver it
for lag in (36, 107, 357):  # frames; about 100 m, 300 m and 1000 m of flight
    estimate, true = flow[lag:] * smooth[:-lag], step[lag:]
    m = np.isfinite(estimate) & np.isfinite(true)
    error = np.abs(estimate[m] - true[m]) / true[m] * 100
    print(f"scale taken from {lag * 2.8:5.0f} m earlier: speed error median {np.median(error):.1f}%, 90% below {np.percentile(error, 90):.1f}%")
total_true, total_estimate = np.nansum(step), np.nansum(flow * np.nanmedian(smooth))
print(f"whole section with one constant scale: {total_estimate:.0f} m estimated against {total_true:.0f} m true ({(total_estimate / total_true - 1) * 100:+.1f}%)")

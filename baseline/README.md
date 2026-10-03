# Baseline package

Two parts share this package: the IMU-only baseline on Mid-Air, and the camera navigator on ALTO, further below.

# IMU-only baseline

How fast does a plain inertial estimate drift away from the true path once GNSS is lost at time t0? This folder answers that for Mid-Air flights, and for synthetic flights until the Mid-Air files are in place. It is the baseline later methods are compared against.

## Run

From the repository root:

```bash
python baseline/scripts/run_midair_baseline.py --synthetic --gnss-cutoff 5.0             # noisy synthetic circle
python baseline/scripts/run_midair_baseline.py --synthetic --no-noise --scenario spinning_hover
python baseline/scripts/run_midair_baseline.py --data-root data/raw/midair/MidAir \
    --environment Kite_training --condition sunny --trajectory 0 --gnss-cutoff 5.0
python -m pytest
```

The Mid-Air root can also come from `MID_AIR_ROOT` or `midair.data_root` in [configs/midair_baseline.yaml](configs/midair_baseline.yaml). Results go to `outputs/midair_baseline/<run_name>/`, which is not committed: `metrics.json`, `errors.csv`, `trajectory_2d.png`, `error_vs_time.png`, `trajectory_3d.png`.

## Method

At t0 the estimator takes the true position, velocity and attitude once. After that it reads only the IMU: the gyroscope turns the attitude, the accelerometer is rotated into the world frame, gravity is added back, and the result is integrated twice. GNSS is never read, and ground truth after t0 serves only for evaluation. Details are in [src/estimation/inertial_dead_reckoning.py](src/estimation/inertial_dead_reckoning.py).

## Conventions

Every loader converts its data to these ([src/data/trajectory.py](src/data/trajectory.py)):

| Quantity | Convention |
|---|---|
| Time | seconds |
| World frame | North, East, Down in metres (ENU also supported) |
| Gravity | (0, 0, +9.81) m/s² in NED |
| Attitude | quaternion (w, x, y, z), rotates body vectors into the world |
| Accelerometer | specific force in the body frame: f = Rᵀ(a − g). Level and still: (0, 0, −9.81) |
| Gyroscope | body rate relative to the world, in the body frame, rad/s |

Mid-Air is the exception for the gyroscope: its files store the turn rate around the world axes. The Mid-Air loader sets `gyroscope_frame = "world"`, and the estimator then applies the turn step on the left. With the body-frame rule, flight 0003 ends 3,758 m off instead of 247 m. The measurement behind this is in `docs/findings.md`, section 2.2, on `main`.

Each run prints how well the IMU matches the ground truth. If the frame, the gravity sign or the quaternion order is wrong, the accelerometer mismatch is of the order of g, and the run warns.

# Camera navigator on ALTO

The camera a drone already has, used as a position sensor once GNSS is jammed. GNSS works for the first 300 m. After that the position is carried forward by how the ground slides through the image, and every few hundred metres the camera frame is matched against aerial reference images that have coordinates. A check decides whether each of these position fixes is believed.

## Run

```bash
python baseline/scripts/run_alto_navigator.py                    # ALTO validation section
python baseline/scripts/run_alto_navigator.py --section Train    # the section the settings were not tuned on
python baseline/scripts/run_alto_navigator.py --synthetic        # generated flight, no download needed
python baseline/scripts/run_alto_navigator.py --only camera_only every_300
```

The ALTO zip files go into `data/raw/alto/` (see `data/README.md`). The runs and their settings are in [configs/alto_navigator.yaml](configs/alto_navigator.yaml). Results go to `outputs/alto_navigator/<flight>/`, which is not committed: `metrics.json`, `errors.csv` (error and stated uncertainty per frame), `fixes.csv` (every attempted fix, used or not and why) and `navigator.png`. The image shifts are computed once per flight and kept in `data/processed/`. The whole validation section takes about a minute.

## Method

1. **While GNSS works,** the navigator learns everything it needs: a 2 by 2 matrix that turns image shift in pixels into ground steps in metres, and the zoom, rotation and offset of the camera against the reference images. It reads no altitude, no orientation and no camera calibration.
2. **After the jam,** the image shift alone carries the position forward. The navigator states its uncertainty, which grows by 10 percent of the distance flown since the last fix.
3. **Every so many metres** it matches the frame against the reference images near its estimate, with normalised correlation of brightness patterns. A fix is used only if its score is high enough and it lies within 3 sigma of the estimate. A used fix is blended in according to the two uncertainties.
4. **The search** covers one map of the whole area, in a circle around the estimate of 60 m or 3 sigma, whichever is larger, so it grows when the navigator is less certain (`search: area`). On ALTO the map is built from all five reference folders, a strip about 380 m wide, and kept in `data/processed/alto_val_map/`. The older searches over the dataset's own reference images (`nearest`, `sized`) stay for comparison: those images are centred on the true path, so that search knows where the path runs (`docs/findings.md`, section 3.8).

The logic is in [src/estimation/navigator_core.py](src/estimation/navigator_core.py), with a worked example for each function. The true position after the jam is read only by [src/evaluation/navigation_metrics.py](src/evaluation/navigation_metrics.py); a test checks that the navigator gives the same result when it is hidden.

## Results on the validation section

GNSS lost after 300 m, then 4.3 km with the camera alone. Errors in metres.

Searching the map around the estimate (the runs to report):

| Run | Median | Worst | End | Fixes used / rejected |
|---|---|---|---|---|
| Camera alone | 472.4 | 657.0 | 608.2 | |
| Fix every 100 m, no check | 25.2 | 49.9 | 27.7 | 39 / 0 |
| Fix every 300 m, score check | 31.1 | 72.9 | 37.8 | 12 / 1 |
| Fix every 400 m, no check | 36.0 | 279.9 | 279.9 | 9 / 0, of which 1 wrong |
| Fix every 1,000 m, score check | 56.1 | 278.1 | 10.3 | 4 / 0 |

Searching the dataset's reference images, which are centred on the true path (for comparison):

| Run | Median | Worst | End | Fixes used / rejected |
|---|---|---|---|---|
| Fix every 100 m, 7 nearest images | 25.5 | 50.3 | 26.3 | 39 / 0 |
| Fix every 300 m, larger search, score check | 30.9 | 72.8 | 37.7 | 12 / 1 |
| Fix every 400 m, 7 nearest images, no check | 285.1 | 897.9 | 897.9 | 7 / 0, of which 5 wrong |
| Fix every 1,000 m, 7 nearest images, score check | 472.4 | 657.0 | 608.2 | 0 / 3 |
| Fix every 1,000 m, larger search, score check | 56.3 | 278.1 | 10.0 | 4 / 0 |

The second table is `experiments/h_alto_end_to_end.py`, reproduced exactly; [tests/test_alto_navigator.py](tests/test_alto_navigator.py) checks both tables whenever the data is present. The settings were chosen on this same section, so they still need a test on data they were not tuned on.

**A known limit, shown by a test:** if the map's coordinates are 200 m off, the first fixes are rejected, but after 600 m without a fix the allowed distance has grown past 200 m and a confident wrong fix is believed. The distance check alone cannot catch a wrong place that lies inside the stated uncertainty.

## Layout

- `src/data/trajectory.py`: the common `Trajectory` and the convention check
- `src/data/synthetic.py`: four exact flights (stationary, constant velocity, banked climbing circle, spinning hover) with optional IMU noise
- `src/data/midair.py`: the Mid-Air adapter. Every assumption about the file format is listed at its top
- `src/data/camera_flight.py`, `alto.py`, `synthetic_camera.py`: camera flights with reference images, the ALTO adapter and a generated flight for tests
- `src/data/ground_map.py`: one north-up map of the area with coordinates, and a mosaic of reference images into it
- `src/estimation/inertial_dead_reckoning.py`: the IMU estimator
- `src/estimation/image_motion.py`, `map_matching.py`, `navigator_core.py`, `camera_navigator.py`: the camera navigator
- `src/evaluation/`, `src/visualization/`: metrics and plots for both
- `scripts/run_midair_baseline.py`, `scripts/run_alto_navigator.py`: the command lines
- `tests/`: frame, gravity, attitude, integration-order and metric tests; navigator tests on the generated flight; the ALTO regression

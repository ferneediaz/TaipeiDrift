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

Each run prints how well the IMU matches the ground truth. If the frame, the gravity sign or the quaternion order is wrong, the accelerometer mismatch is of the order of g, and the run warns.

## Layout

- `src/data/trajectory.py`: the common `Trajectory` and the convention check
- `src/data/synthetic.py`: four exact flights (stationary, constant velocity, banked climbing circle, spinning hover) with optional IMU noise
- `src/data/midair.py`: the Mid-Air adapter. Every assumption about the file format is listed at its top
- `src/estimation/`, `src/evaluation/`, `src/visualization/`: estimator, metrics, plots
- `scripts/run_midair_baseline.py`: the command line
- `tests/`: frame, gravity, attitude, integration-order and metric tests

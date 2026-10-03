"""Alessandro's metric optical flow (sim/nodes/metric_flow.py) on made-up points: which cause gives speed errors of
the size his city run measured (4.45 m/s in the median, at 8 m/s)?

The camera is level and looks straight down, 512 px and 90 degrees (focal length 256 px), and flies 8 m/s, as in
his run (city world, 80 m above the street, buildings 16 to 63 m high, pictures 0.04 s apart). No simulator and
no recording is needed:

    python scripts/metric_flow_check.py

1. tracking noise alone, over flat ground with the right range;
2. the range finder and the tracked points at different depths (a roof and the street);
3. half the points on roofs, the range on the street.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim" / "nodes"))
from frame_conversions import gazebo_down_optical_to_flu  # noqa: E402
from metric_flow import estimate_metric_velocity  # noqa: E402
from vio.vision.optical_flow import FlowConfig, FlowPair  # noqa: E402

FOCAL_PX, SPEED_MPS = 256.0, 8.0
R_BC = gazebo_down_optical_to_flu()
# the settings his adapter uses (sim/nodes/eskf_ros_adapter.py, down_flow_cfg)
CFG = FlowConfig(min_tracks=20, min_inlier_ratio=0.50, ransac_threshold_px=1.0, max_residual_px=2.0)


def pair(depths: np.ndarray, dt: float, noise_px: float, rng: np.random.Generator) -> tuple[FlowPair, np.ndarray]:
    """Points at the given depths below the camera, seen before and after flying SPEED_MPS * dt forward."""
    n = len(depths)
    xy = rng.uniform(-0.9, 0.9, (n, 2))                    # normalised image coordinates, inside the 90-degree view
    points = np.c_[xy, np.ones(n)] * depths[:, None]       # in the first camera's axes (z points down)
    step = R_BC.T @ np.array([SPEED_MPS * dt, 0.0, 0.0])   # the drone flies along body x
    after = points - step
    xa = xy + rng.normal(0, noise_px / FOCAL_PX, (n, 2))
    xb = after[:, :2] / after[:, 2:3] + rng.normal(0, noise_px / FOCAL_PX, (n, 2))
    return FlowPair(1, 1, 0, xa, xb, n, 1.0, True), step[:2] / dt


def speed_error(depths, range_m: float, dt: float, noise_px: float, trials: int = 300) -> tuple[float, float]:
    """Median error of the measured speed (m/s) and the share of pairs the fit rejects."""
    rng, errors, rejected = np.random.default_rng(0), [], 0
    for _ in range(trials):
        flow_pair, true_velocity = pair(depths(rng), dt, noise_px, rng)
        out = estimate_metric_velocity(flow_pair, np.eye(3), np.array([0.0, 0.0, 1.0]), range_m, dt, FOCAL_PX, CFG)
        if out["valid"]:
            errors.append(np.linalg.norm(out["velocity"] - true_velocity))
        else:
            rejected += 1
    return (float(np.median(errors)) if errors else float("nan")), rejected / trials


def flat(count: int, depth_m: float):
    return lambda rng: np.full(count, depth_m)


def main() -> None:
    print("The picture moves %.2f px between two frames 0.04 s apart and %.2f px between frames 0.20 s apart "
          "(street 80 m below, 8 m/s)." % (SPEED_MPS * 0.04 / 80 * FOCAL_PX, SPEED_MPS * 0.20 / 80 * FOCAL_PX))

    print("\n1. Flat ground 80 m below, range correct, tracking noise only: median speed error, m/s")
    print("   points  noise px | frames 0.04 s apart | frames 0.20 s apart")
    for count in (25, 100, 400):
        for noise in (0.1, 0.25, 0.5):
            near, _ = speed_error(flat(count, 80.0), 80.0, 0.04, noise)
            far, _ = speed_error(flat(count, 80.0), 80.0, 0.20, noise)
            print(f"   {count:6d}  {noise:8.2f} | {near:19.2f} | {far:19.2f}")

    print("\n2. No noise; the range finder and the tracked points at different depths (frames 0.04 s apart)")
    for name, depth_m, range_m in (("points on the street (80 m), beam on a 58 m roof (22 m)", 80.0, 22.0),
                                   ("points on a 58 m roof (22 m), beam on the street (80 m)", 22.0, 80.0),
                                   ("points on an 18 m roof (62 m), beam on the street (80 m)", 62.0, 80.0),
                                   ("points on the street (80 m), beam on an 18 m roof (62 m)", 80.0, 62.0)):
        error, rejected = speed_error(flat(200, depth_m), range_m, 0.04, 0.0, trials=20)
        print(f"   {name}: {error:5.2f} m/s off at {SPEED_MPS:.0f} m/s, rejected {rejected:.0%}")

    print("\n3. No noise; half the points on the street (80 m), half on roofs (22 to 62 m below), beam on the street")
    mixed = lambda rng: np.r_[np.full(100, 80.0), rng.choice([22.0, 35.0, 50.0, 62.0], 100)]  # noqa: E731
    for dt in (0.04, 0.20):
        error, rejected = speed_error(mixed, 80.0, dt, 0.0, trials=100)
        print(f"   frames {dt:.2f} s apart: {error:5.2f} m/s off, rejected {rejected:.0%}")


if __name__ == "__main__":
    main()

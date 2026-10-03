"""Dead reckoning between photos, as an error model, for flights whose photos are too far apart.

UAV-VisLoc has one photo every 95 m, too far apart to measure the motion from one photo to the
next with optical flow. A drone would film continuously and measure it. To test the position
fixes on such a flight, the steps between photos are made from the true steps plus the errors a
camera dead reckoning makes, as measured on ALTO (``docs/findings.md`` 3.4):

- **Direction:** the step is turned by the error of the heading sensor at that moment.
- **Length:** a scale error, with an offset for the flight and a slow random walk. On ALTO the
  scale went 13 percent stale over 4.3 km without fixes.
- **Noise:** independent per step.

The fixes themselves stay real: real photos matched against a real map.
"""
from __future__ import annotations

import numpy as np


def simulated_steps(
    position_gt: np.ndarray,
    heading_error_deg: np.ndarray,
    rng: np.random.Generator,
    scale_offset_sd: float = 0.03,
    scale_walk_per_sqrt_km: float = 0.03,
    noise_m_per_sqrt_100m: float = 1.0,
) -> np.ndarray:
    """Steps (north, east) between consecutive frames as the dead reckoning would report them.

    Row 0 is zero, as for image shifts. A heading error of +a degrees turns a step clockwise by a:
    a step of 100 m north with an error of +3 degrees becomes 99.9 m north and 5.2 m east.
    """
    position_gt = np.asarray(position_gt, dtype=float)
    true = np.diff(position_gt, axis=0)
    length = np.linalg.norm(true, axis=1)
    scale = 1.0 + rng.normal(0.0, scale_offset_sd) + np.cumsum(rng.normal(0.0, 1.0, len(true)) * scale_walk_per_sqrt_km * np.sqrt(length / 1000.0))
    a = np.radians(np.asarray(heading_error_deg, dtype=float)[1:])
    turned = np.column_stack([true[:, 0] * np.cos(a) - true[:, 1] * np.sin(a), true[:, 1] * np.cos(a) + true[:, 0] * np.sin(a)])
    noise = rng.normal(0.0, 1.0, true.shape) * noise_m_per_sqrt_100m * np.sqrt(length / 100.0)[:, None]
    steps = turned * scale[:, None] + noise
    return np.vstack([np.zeros((1, 2)), steps])

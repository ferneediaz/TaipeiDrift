"""Shared matplotlib style and colours (one consistent visual system for all figures)."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {"tercom": "#8c6d31", "mpf_baseline": "#d62728", "mpf_gated": "#ff7f0e", "mpf_proposed": "#1f77b4",
          "ins": "#7f7f7f", "truth": "#000000"}
LABELS = {"tercom": "TERCOM", "mpf_baseline": "MPF baseline (last echo)", "mpf_gated": "MPF gated",
          "mpf_proposed": "MPF proposed (obscuration-aware)", "ins": "INS only"}
ROUTE_LABELS = {"A_mountain_crossing": "A: Central Mountain crossing", "B_coastal_plain": "B: Western coastal plain",
                "C_foothills": "C: Taichung foothills"}


def setup() -> None:
    plt.rcParams.update({"figure.dpi": 110, "savefig.dpi": 130, "font.size": 9, "axes.titlesize": 10,
                         "axes.grid": True, "grid.alpha": 0.3, "axes.spines.top": False, "axes.spines.right": False,
                         "legend.frameon": False, "savefig.bbox": "tight"})


def hillshade(z, dx: float, az: float = 315.0, alt: float = 45.0):
    """Simple hillshade for map backgrounds."""
    import numpy as np
    gy, gx = np.gradient(np.nan_to_num(z), dx)
    slope = np.pi / 2 - np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    a, al = np.radians(az), np.radians(alt)
    return np.sin(al) * np.sin(slope) + np.cos(al) * np.cos(slope) * np.cos(a - aspect)

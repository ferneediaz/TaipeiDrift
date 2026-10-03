"""TRUTH world for the simulator. Never imported by ``trn.filters`` (enforced by tests/test_isolation.py).

Top surface = MOI DSM 2024 at native 20 m (canopy, buildings). Ground layer = MOI DEM 2025 at native
20 m, used only to place ground echoes through canopy gaps (PLAN.md Q1, option a).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from trn.common.config import project_path
from trn.terrain.grid import Grid


@dataclass
class TruthMap:
    """Truth surfaces of one route tile."""

    top: Grid
    ground: Grid
    sea: np.ndarray

    @classmethod
    def load(cls, cfg: dict, route: str) -> "TruthMap":
        folder = project_path(cfg["data"]["processed_dir"]) / route
        return cls(top=Grid.load(folder, "dsm20"), ground=Grid.load(folder, "dem20"),
                   sea=np.load(folder / "sea.npy", mmap_mode="r"))

    def is_sea(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Nearest-cell sea flag."""
        r, c = self.top.xy_to_rc(x, y)
        r = np.clip(np.rint(r).astype(int), 0, self.sea.shape[0] - 1)
        c = np.clip(np.rint(c).astype(int), 0, self.sea.shape[1] - 1)
        return self.sea[r, c].astype(bool)

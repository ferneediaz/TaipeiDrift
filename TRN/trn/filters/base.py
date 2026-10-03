"""Common filter interface. Filters only ever receive an OnboardMap and sensor measurements."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

from trn.terrain.onboard_map import OnboardMap


@dataclass
class Measurement:
    """Everything a navigation filter sees at one laser pulse (no truth)."""

    t: float
    ins_pos: np.ndarray        # (3,) INS position E, N, U
    ins_vel: np.ndarray        # (3,)
    ins_C: np.ndarray          # (3,3) INS body->nav
    ins_fn: np.ndarray         # (3,) mean nav-frame specific force over the NEXT interval
    ranges: np.ndarray         # (B, M) echo ranges, NaN = no echo, sorted ascending
    baro: float                # baro altitude or NaN


@dataclass
class Estimate:
    """Filter output at one step."""

    pos: np.ndarray            # (3,) estimated true position
    cov: np.ndarray            # (2,2) horizontal covariance
    neff: float = np.nan
    reinit: bool = False
    extra: dict = field(default_factory=dict)


class NavFilter(ABC):
    """Shared interface: ``initialize`` once with the first measurement, then ``step`` every pulse."""

    name: str = "filter"

    def __init__(self, onboard: OnboardMap):
        if not isinstance(onboard, OnboardMap):
            raise TypeError("navigation filters accept only an OnboardMap (truth-map isolation)")
        self.map = onboard

    @abstractmethod
    def initialize(self, m: Measurement) -> Estimate: ...

    @abstractmethod
    def step(self, m: Measurement) -> Estimate: ...

"""Reproducible random streams: one SeedSequence per run, named child streams per component."""
from __future__ import annotations

import zlib

import numpy as np

STREAMS = ("ins", "laser", "atmosphere", "filter", "map", "baro")


def run_rngs(base_seed: int, run_index: int, salt: str = "") -> dict[str, np.random.Generator]:
    """Independent generators per component for one Monte Carlo run.

    ``salt`` (e.g. the route name) decorrelates runs of different configurations that share an index.
    The same (base_seed, run_index, salt) always reproduces the same run.
    """
    ss = np.random.SeedSequence([base_seed, run_index, zlib.crc32(salt.encode())])
    children = ss.spawn(len(STREAMS))
    return {name: np.random.default_rng(c) for name, c in zip(STREAMS, children)}

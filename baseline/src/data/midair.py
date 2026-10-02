"""Mid-Air dataset adapter (https://midair.ulg.ac.be/).

Every assumption about the downloaded files lives in this module and in the
``midair`` section of ``configs/midair_baseline.yaml``. Correct them here once
the real files have been inspected.

Assumptions and where they come from
------------------------------------
- File location: ``<root>/<environment>/<condition>/sensor_records.hdf5``,
  with ``<root>`` optionally followed by ``MidAir/``. Source: data/README.md
  and scripts/fetch_midair.sh (the download unpacks to
  ``MidAir/<set>/<condition>/sensor_records.zip``).
- One HDF5 group per flight, holding ``imu/accelerometer``, ``imu/gyroscope``
  and ``groundtruth/attitude``. Source: experiments/e_midair_imu_noise.py,
  which has been run on the real files.
- ``groundtruth/position`` and ``groundtruth/velocity``: named after the
  dataset documentation, NOT yet verified on the files.
- Attitude quaternion order (w, x, y, z), world frame NED, gravity 9.81 m/s^2,
  accelerometer as specific force: taken from e_midair_imu_noise.py. The
  loader's consistency check (``imu_consistency``) verifies them per flight.
- No timestamp dataset is assumed: time is sample index / ``imu_rate_hz``
  (100 Hz per the dataset documentation) unless ``keys.timestamp`` is set.
- Flight group names: a bare number such as ``3`` is expanded to
  ``trajectory_0003``, the naming used by the camera folders in the download.
  Full group names are passed through unchanged.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import h5py
import numpy as np

from src.data.trajectory import TimedObservations, Trajectory, gravity_vector

ENV_VAR = "MID_AIR_ROOT"


class MidAirDataNotFound(FileNotFoundError):
    """The Mid-Air files are not where the configuration says."""


@dataclass
class MidAirKeys:
    """HDF5 dataset paths inside one flight group."""

    accelerometer: str = "imu/accelerometer"
    gyroscope: str = "imu/gyroscope"
    position: str = "groundtruth/position"
    velocity: str = "groundtruth/velocity"
    attitude: str = "groundtruth/attitude"
    gps_position: str | None = None  # not read until its layout is checked
    timestamp: str | None = None  # None: build time from imu_rate_hz


@dataclass
class MidAirConventions:
    """How the files encode frames and quantities."""

    quaternion_order: str = "wxyz"  # "wxyz" or "xyzw"
    world_frame: str = "NED"  # "NED" or "ENU"
    gravity: float = 9.81  # m/s^2
    accelerometer: str = "specific_force"  # only specific force is supported


@dataclass
class MidAirConfig:
    """What to load and how to interpret it."""

    data_root: str | None = None
    environment: str = "Kite_training"
    condition: str = "sunny"
    trajectory: str = "trajectory_0000"
    camera_stream: str = "color_down"  # recorded in metadata; images not loaded yet
    sensor_file: str = "sensor_records.hdf5"
    imu_rate_hz: float = 100.0
    gps_rate_hz: float = 1.0
    keys: MidAirKeys = field(default_factory=MidAirKeys)
    conventions: MidAirConventions = field(default_factory=MidAirConventions)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "MidAirConfig":
        """Build from the ``midair`` section of the YAML config."""
        d = dict(d)
        keys = MidAirKeys(**d.pop("keys", {}) or {})
        conventions = MidAirConventions(**d.pop("conventions", {}) or {})
        cfg = cls(keys=keys, conventions=conventions, **d)
        cfg.trajectory = normalize_trajectory_name(cfg.trajectory)
        return cfg


def normalize_trajectory_name(name: str | int) -> str:
    """Expand a bare flight number to ``trajectory_XXXX``; leave other names unchanged."""
    s = str(name).strip()
    return f"trajectory_{int(s):04d}" if s.isdigit() else s


def resolve_data_root(cli_root: str | None, config_root: str | None) -> Path:
    """Pick the dataset root: command line, then $MID_AIR_ROOT, then the config file."""
    root = cli_root or os.environ.get(ENV_VAR) or config_root
    if not root:
        raise MidAirDataNotFound(
            "No Mid-Air data root configured. Do one of:\n"
            "  - pass --data-root <folder>\n"
            f"  - set the environment variable {ENV_VAR}=<folder>\n"
            "  - set midair.data_root in baseline/configs/midair_baseline.yaml\n"
            "The folder is the one holding the environment folders (e.g. Kite_training/), "
            "or its parent containing MidAir/. With scripts/fetch_midair.sh that is "
            "data/raw/midair/MidAir.\n"
            "To run without the dataset, use --synthetic."
        )
    path = Path(root).expanduser()
    if not path.is_dir():
        raise MidAirDataNotFound(
            f"Mid-Air data root {path} does not exist. Check --data-root, {ENV_VAR} "
            "or midair.data_root in the config."
        )
    return path


def find_sensor_file(root: Path, cfg: MidAirConfig) -> Path:
    """Locate the sensor-records HDF5 file for the selected environment and condition."""
    candidates = [
        root / cfg.environment / cfg.condition / cfg.sensor_file,
        root / "MidAir" / cfg.environment / cfg.condition / cfg.sensor_file,
    ]
    for c in candidates:
        if c.is_file():
            return c
    zipped = [c.with_suffix(".zip") for c in candidates if c.with_suffix(".zip").is_file()]
    if zipped:
        raise MidAirDataNotFound(
            f"Found {zipped[0]} but not the unpacked {cfg.sensor_file}. Unzip it in place first."
        )
    present = sorted(p.name for p in root.iterdir() if p.is_dir())
    tried = "\n  ".join(str(c) for c in candidates)
    raise MidAirDataNotFound(
        f"Mid-Air sensor file not found. Tried:\n  {tried}\n"
        f"Folders present in {root}: {present or 'none'}.\n"
        "Check environment/condition, or point the data root at the folder that contains them."
    )


def _list_datasets(group: h5py.Group) -> list[str]:
    names: list[str] = []
    group.visititems(lambda n, obj: names.append(n) if isinstance(obj, h5py.Dataset) else None)
    return names


def _read(group: h5py.Group, key: str) -> np.ndarray:
    if key not in group:
        raise KeyError(
            f"dataset {key!r} not found in {group.name}. Datasets present: {_list_datasets(group)}. "
            "Update midair.keys in the config."
        )
    return np.asarray(group[key][:], dtype=float)


def _to_wxyz(q: np.ndarray, order: str) -> np.ndarray:
    if order == "wxyz":
        return q
    if order == "xyzw":
        return q[:, [3, 0, 1, 2]]
    raise ValueError(f"unsupported quaternion order {order!r}, expected 'wxyz' or 'xyzw'")


def load_midair_trajectory(cfg: MidAirConfig, data_root: str | None = None) -> Trajectory:
    """Load one Mid-Air flight into the common ``Trajectory`` representation.

    Args:
        cfg: selection, HDF5 keys and conventions.
        data_root: overrides ``$MID_AIR_ROOT`` and ``cfg.data_root``.
    """
    conv = cfg.conventions
    if conv.accelerometer != "specific_force":
        raise NotImplementedError(
            "Only accelerometer readings that include gravity (specific force) are supported. "
            "Gravity-removed readings would need the true attitude after GNSS loss to undo."
        )
    root = resolve_data_root(data_root, cfg.data_root)
    path = find_sensor_file(root, cfg)

    with h5py.File(path, "r") as f:
        if cfg.trajectory not in f:
            raise KeyError(
                f"flight {cfg.trajectory!r} not in {path}. Available: {sorted(f.keys())}"
            )
        g = f[cfg.trajectory]
        k = cfg.keys
        accel = _read(g, k.accelerometer)
        gyro = _read(g, k.gyroscope)
        position = _read(g, k.position)
        velocity = _read(g, k.velocity)
        attitude = _to_wxyz(_read(g, k.attitude), conv.quaternion_order)
        timestamp = _read(g, k.timestamp).ravel() if k.timestamp else None
        gps_pos = _read(g, k.gps_position) if k.gps_position else None

    lengths = {"accelerometer": len(accel), "gyroscope": len(gyro), "position": len(position),
               "velocity": len(velocity), "attitude": len(attitude)}
    if len(set(lengths.values())) != 1:
        raise ValueError(
            f"IMU and ground truth have different lengths {lengths}. They are assumed to share "
            "one clock at imu_rate_hz; resampling is not implemented yet."
        )
    n = len(accel)
    if timestamp is None:
        timestamp = np.arange(n) / cfg.imu_rate_hz

    gps = None
    if gps_pos is not None:
        gps = TimedObservations(timestamp=np.arange(len(gps_pos)) / cfg.gps_rate_hz, values=gps_pos)

    return Trajectory(
        timestamp=timestamp,
        position_gt=position,
        velocity_gt=velocity,
        attitude_gt=attitude / np.linalg.norm(attitude, axis=1, keepdims=True),
        accelerometer=accel,
        gyroscope=gyro,
        world_frame=conv.world_frame.upper(),
        gravity_world=gravity_vector(conv.world_frame, conv.gravity),
        name=f"midair_{cfg.environment}_{cfg.condition}_{cfg.trajectory}",
        gps=gps,
        metadata={
            "source": "midair",
            "file": str(path),
            "environment": cfg.environment,
            "condition": cfg.condition,
            "trajectory": cfg.trajectory,
            "camera_stream": cfg.camera_stream,
        },
    )

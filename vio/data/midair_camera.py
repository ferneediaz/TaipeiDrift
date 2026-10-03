"""Mid-Air camera frames and their alignment with the 100 Hz sensor records.

Verified on the downloaded Kite_training files (see vio/README.md for the evidence):

- ``camera_data/<stream>`` in each flight group of ``sensor_records.hdf5``
  lists one relative path per frame, e.g. ``color_down/trajectory_0000/000000.JPEG``.
- The JPEGs are in ``<root>/<environment>/<condition>/<stream>/<trajectory>/frames.zip``,
  named ``000000.JPEG`` upwards, 1024 x 1024 RGB. An already extracted folder
  with the same file names is also accepted.
- Frames run at 25 Hz: 2205 frames against 8818 IMU samples. Frame ``i`` is
  taken at IMU sample ``4 i`` (offset 0, measured by matching visual and
  ground-truth rotation; ``frame_offset_samples`` keeps it configurable).
"""
from __future__ import annotations

import zipfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import h5py
import numpy as np

STREAM_ALIASES = {"down": "color_down", "left": "color_left", "right": "color_right"}


def stream_name(camera: str) -> str:
    """Map a short camera name ('down', 'left', 'right') to the Mid-Air stream name."""
    return STREAM_ALIASES.get(camera, camera)


def frame_to_imu_index(frame_index: np.ndarray | int, imu_rate_hz: float, camera_rate_hz: float,
                       offset_samples: int = 0) -> np.ndarray:
    """IMU sample index at which a camera frame was taken."""
    ratio = imu_rate_hz / camera_rate_hz
    if abs(ratio - round(ratio)) > 1e-9:
        raise ValueError(f"IMU rate {imu_rate_hz} is not an integer multiple of camera rate {camera_rate_hz}")
    return np.asarray(frame_index) * int(round(ratio)) + offset_samples


@dataclass
class FrameSource:
    """Lazy access to one flight's frames of one camera stream."""

    names: list[str]  # member file names, in frame order
    zip_path: Path | None
    folder: Path | None

    def __len__(self) -> int:
        return len(self.names)

    def read_gray(self, i: int, downscale: int = 1) -> np.ndarray:
        """Frame ``i`` as an 8-bit grayscale image, optionally downscaled by an integer factor."""
        if self.zip_path is not None:
            with zipfile.ZipFile(self.zip_path) as z:  # reopened per call: cheap next to JPEG decoding
                buf = np.frombuffer(z.read(self.names[i]), np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        else:
            img = cv2.imread(str(self.folder / self.names[i]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise IOError(f"could not decode frame {i} ({self.names[i]})")
        if downscale > 1:
            img = cv2.resize(img, (img.shape[1] // downscale, img.shape[0] // downscale), interpolation=cv2.INTER_AREA)
        return img

    def iter_gray(self, start: int = 0, downscale: int = 1):
        """Yield (index, grayscale frame) from ``start`` to the end, keeping the zip open."""
        if self.zip_path is None:
            for i in range(start, len(self)):
                yield i, self.read_gray(i, downscale)
            return
        with zipfile.ZipFile(self.zip_path) as z:
            for i in range(start, len(self)):
                img = cv2.imdecode(np.frombuffer(z.read(self.names[i]), np.uint8), cv2.IMREAD_GRAYSCALE)
                if img is None:
                    raise IOError(f"could not decode frame {i} ({self.names[i]})")
                if downscale > 1:
                    img = cv2.resize(img, (img.shape[1] // downscale, img.shape[0] // downscale),
                                     interpolation=cv2.INTER_AREA)
                yield i, img


def open_frames(sensor_file: Path, trajectory: str, camera: str, frames_dir: Path | None = None) -> FrameSource:
    """Find the frames of one flight and camera stream.

    By default they sit next to the sensor file (``<condition>/<stream>/<trajectory>``).
    ``frames_dir`` overrides the ``<condition>`` folder, for when the sensor file was
    extracted elsewhere (the benchmark reads it from a cache, never from inside data/).
    """
    stream = stream_name(camera)
    with h5py.File(sensor_file, "r") as f:
        key = f"camera_data/{stream}"
        if key not in f[trajectory]:
            raise KeyError(f"{key} not in {trajectory}; streams: {sorted(f[trajectory]['camera_data'])}")
        rel = [p.decode() if isinstance(p, bytes) else str(p) for p in f[trajectory][key][:]]
    names = [Path(p).name for p in rel]
    base = Path(frames_dir or sensor_file.parent) / stream / trajectory
    zip_path = base / "frames.zip"
    if zip_path.is_file():
        with zipfile.ZipFile(zip_path) as z:
            members = set(z.namelist())
        missing = [n for n in names if n not in members]
        if missing:
            raise FileNotFoundError(f"{len(missing)} frames listed in the HDF5 are missing from {zip_path}, e.g. {missing[:3]}")
        return FrameSource(names, zip_path, None)
    if base.is_dir() and (base / names[0]).is_file():
        return FrameSource(names, None, base)
    raise FileNotFoundError(
        f"No frames for {stream}/{trajectory}: expected {zip_path} or extracted JPEGs in {base}. "
        "Only some flights' images are in the downloaded subset."
    )

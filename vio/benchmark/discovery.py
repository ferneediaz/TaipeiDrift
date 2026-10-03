"""Discover which Mid-Air trajectories on disk are complete enough to benchmark.

Downloads may still be running, so every file is checked before use and
nothing in the dataset folder is ever written, renamed or repaired:

- ``<env>/<condition>/sensor_records.hdf5`` is used if present. Otherwise
  ``sensor_records.zip`` is CRC-checked and extracted to a cache OUTSIDE the
  dataset (atomically: temp file, then rename). A zip that cannot be opened is
  treated as still downloading.
- Each flight group needs the IMU and ground-truth datasets with equal, sensible
  lengths and finite values. That makes it IMU-valid.
- For the visual estimator it also needs ``<condition>/<stream>/<trajectory>/frames.zip``
  that opens, contains every frame listed in ``camera_data/<stream>``, and whose
  first and last listed frames decode. That makes it visual-valid.

A trajectory without usable images still takes part in the IMU-only and oracle
runs; the visual estimator is marked unavailable for it, with the reason.
"""
from __future__ import annotations

import json
import os
import shutil
import zipfile
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import h5py
import numpy as np

REQUIRED_KEYS = ("imu/accelerometer", "imu/gyroscope", "groundtruth/position", "groundtruth/velocity",
                 "groundtruth/attitude")
SENSOR_FILE = "sensor_records.hdf5"
SENSOR_ZIP = "sensor_records.zip"


@dataclass
class TrajectoryEntry:
    environment: str
    condition: str
    trajectory: str
    imu_valid: bool = False
    imu_reason: str = ""
    visual_valid: bool = False
    visual_reason: str = ""
    n_samples: int = 0
    duration_s: float = 0.0
    n_frames_listed: int = 0
    sensor_file: str = ""  # HDF5 actually opened (dataset or cache)
    frames_dir: str = ""  # <env>/<condition> folder in the dataset
    signature: dict = field(default_factory=dict)  # sizes and mtimes, for resume

    @property
    def key(self) -> str:
        return f"{self.environment}/{self.condition}/{self.trajectory}"

    @property
    def name(self) -> str:
        return f"{self.environment}_{self.condition}_{self.trajectory}"


@dataclass
class DiscoveryReport:
    data_root: str
    camera_stream: str
    entries: list[TrajectoryEntry]
    condition_issues: list[dict]  # conditions whose sensor file could not be used at all
    orphan_camera_folders: list[str]  # image folders with no matching flight in the sensor file

    def counts(self) -> dict:
        e = self.entries
        return {"discovered": len(e) + sum(c.get("n_camera_folders", 0) for c in self.condition_issues),
                "imu_valid": sum(x.imu_valid for x in e), "visual_valid": sum(x.visual_valid for x in e),
                "imu_invalid": sum(not x.imu_valid for x in e),
                "visual_unavailable": sum(x.imu_valid and not x.visual_valid for x in e),
                "conditions_unusable": len(self.condition_issues)}

    def to_json(self) -> dict:
        return {"data_root": self.data_root, "camera_stream": self.camera_stream, "counts": self.counts(),
                "conditions": sorted({f"{x.environment}/{x.condition}" for x in self.entries}),
                "condition_issues": self.condition_issues, "orphan_camera_folders": self.orphan_camera_folders,
                "entries": [asdict(x) for x in self.entries]}


def _file_sig(p: Path) -> dict:
    st = p.stat()
    return {"path": str(p), "size": st.st_size, "mtime": int(st.st_mtime)}


def resolve_sensor_file(cond_dir: Path, cache_root: Path, env: str, cond: str) -> tuple[Path | None, dict, str]:
    """Return (usable HDF5 path, signature, problem). Extracts the zip into the cache if needed."""
    direct = cond_dir / SENSOR_FILE
    if direct.is_file():
        return direct, _file_sig(direct), ""
    z = cond_dir / SENSOR_ZIP
    if not z.is_file():
        return None, {}, "no sensor_records.hdf5 or sensor_records.zip"
    sig = _file_sig(z)
    target = cache_root / env / cond / SENSOR_FILE
    stamp = target.with_suffix(".source.json")
    if target.is_file() and stamp.is_file():
        try:
            if json.loads(stamp.read_text()) == {"size": sig["size"], "mtime": sig["mtime"]}:
                return target, sig, ""
        except (OSError, json.JSONDecodeError):
            pass
    try:
        with zipfile.ZipFile(z) as zf:
            if SENSOR_FILE not in zf.namelist():
                return None, sig, f"{SENSOR_ZIP} does not contain {SENSOR_FILE}"
            bad = zf.testzip()
            if bad is not None:
                return None, sig, f"{SENSOR_ZIP}: CRC error in {bad} (incomplete or corrupt download?)"
            target.parent.mkdir(parents=True, exist_ok=True)
            tmp = target.with_suffix(".partial")
            with zf.open(SENSOR_FILE) as src, open(tmp, "wb") as dst:
                shutil.copyfileobj(src, dst, 1 << 20)
            os.replace(tmp, target)
            stamp.write_text(json.dumps({"size": sig["size"], "mtime": sig["mtime"]}))
    except (zipfile.BadZipFile, OSError, EOFError) as e:
        return None, sig, f"{SENSOR_ZIP} unreadable ({type(e).__name__}: {e}); still downloading?"
    return target, sig, ""


def check_flight(group: h5py.Group, min_samples: int) -> tuple[bool, str, int]:
    """IMU and ground truth present, equal lengths, finite, long enough."""
    missing = [k for k in REQUIRED_KEYS if k not in group]
    if missing:
        return False, f"missing datasets {missing}", 0
    lengths = {k: group[k].shape[0] for k in REQUIRED_KEYS}
    if len(set(lengths.values())) != 1:
        return False, f"unequal lengths {lengths}", 0
    n = next(iter(lengths.values()))
    if n < min_samples:
        return False, f"too short: {n} samples < {min_samples} needed", n
    for k in REQUIRED_KEYS:
        if not np.all(np.isfinite(group[k][:])):
            return False, f"non-finite values in {k}", n
    return True, "", n


def check_frames(cond_dir: Path, stream: str, trajectory: str, listed: list[str], min_frames: int) -> tuple[bool, str, dict]:
    """Camera archive opens, holds every listed frame, first and last listed frames decode."""
    fz = cond_dir / stream / trajectory / "frames.zip"
    if not fz.is_file():
        return False, f"no {stream}/{trajectory}/frames.zip (not downloaded)", {}
    sig = _file_sig(fz)
    if len(listed) < min_frames:
        return False, f"only {len(listed)} frames listed in the sensor file", sig
    try:
        with zipfile.ZipFile(fz) as zf:
            members = set(zf.namelist())
            names = [Path(p).name for p in listed]
            missing = [n for n in names if n not in members]
            if missing:
                return False, f"{len(missing)} listed frames missing from frames.zip, e.g. {missing[:2]}", sig
            for n in (names[0], names[-1]):
                img = cv2.imdecode(np.frombuffer(zf.read(n), np.uint8), cv2.IMREAD_GRAYSCALE)
                if img is None or img.size == 0:
                    return False, f"frame {n} does not decode", sig
    except (zipfile.BadZipFile, OSError, EOFError, zipfile.LargeZipFile) as e:
        return False, f"frames.zip unreadable ({type(e).__name__}); still downloading?", sig
    return True, "", sig


def discover(data_root: Path, cache_root: Path, camera_stream: str = "color_left", environments: list[str] | None = None,
             conditions: list[str] | None = None, trajectories: list[str] | None = None, imu_rate_hz: float = 100.0,
             min_duration_s: float = 10.0, min_frames: int = 50) -> DiscoveryReport:
    """Scan the dataset once. The result is the frozen trajectory list for one benchmark run."""
    data_root = Path(data_root)
    if (data_root / "MidAir").is_dir() and not any((data_root / d).is_dir() and d.endswith("_training") for d in os.listdir(data_root)):
        data_root = data_root / "MidAir"
    entries: list[TrajectoryEntry] = []
    issues: list[dict] = []
    orphans: list[str] = []
    env_dirs = sorted(p for p in data_root.iterdir() if p.is_dir()) if data_root.is_dir() else []
    for env_dir in env_dirs:
        env = env_dir.name
        if environments and env not in environments:
            continue
        for cond_dir in sorted(p for p in env_dir.iterdir() if p.is_dir()):
            cond = cond_dir.name
            if conditions and cond not in conditions:
                continue
            cam_dir = cond_dir / camera_stream
            cam_folders = sorted(p.name for p in cam_dir.iterdir() if p.is_dir()) if cam_dir.is_dir() else []
            sensor, sensor_sig, problem = resolve_sensor_file(cond_dir, cache_root, env, cond)
            if sensor is None:
                issues.append({"environment": env, "condition": cond, "reason": problem,
                               "n_camera_folders": len(cam_folders)})
                continue
            try:
                f = h5py.File(sensor, "r")
            except OSError as e:
                issues.append({"environment": env, "condition": cond, "reason": f"HDF5 unreadable: {e}",
                               "n_camera_folders": len(cam_folders)})
                continue
            with f:
                flights = sorted(k for k in f.keys() if k.startswith("trajectory_"))
                orphans += [f"{env}/{cond}/{camera_stream}/{c}" for c in cam_folders if c not in flights]
                for name in flights:
                    if trajectories and name not in trajectories and name.split("_")[-1] not in trajectories \
                            and str(int(name.split("_")[-1])) not in trajectories:
                        continue
                    e = TrajectoryEntry(env, cond, name, sensor_file=str(sensor), frames_dir=str(cond_dir))
                    e.signature = {"sensor": sensor_sig}
                    ok, why, n = check_flight(f[name], int(min_duration_s * imu_rate_hz))
                    e.imu_valid, e.imu_reason, e.n_samples = ok, why, n
                    e.duration_s = n / imu_rate_hz
                    key = f"camera_data/{camera_stream}"
                    listed = [p.decode() if isinstance(p, bytes) else str(p) for p in f[name][key][:]] if key in f[name] else []
                    e.n_frames_listed = len(listed)
                    if not ok:
                        e.visual_reason = "IMU/ground truth invalid"
                    elif not listed:
                        e.visual_reason = f"sensor file lists no {camera_stream} frames"
                    else:
                        v_ok, v_why, fsig = check_frames(cond_dir, camera_stream, name, listed, min_frames)
                        e.visual_valid, e.visual_reason = v_ok, v_why
                        if fsig:
                            e.signature["frames"] = fsig
                    entries.append(e)
    return DiscoveryReport(str(data_root), camera_stream, entries, issues, orphans)

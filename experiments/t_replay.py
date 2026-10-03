"""TaipeiDrift common replay format (schema `taipeidrift-replay/1`): loader and validator.

One directory per sequence:

  meta.json    provenance per sensor, frames, origin, licence, evidence label
  imu.csv      t_s, gx, gy, gz [rad/s], ax, ay, az [m/s^2]  (body frame named in meta.sensors.imu.frame)
  baro.csv     t_s, pressure_pa, temperature_c, alt_isa_m
                 alt_isa_m = ISA altitude computed by the exporter from pressure_pa (p0 = 101325 Pa).
                 When the source only gives an altitude, pressure_pa is empty and
                 meta.sensors.baro.altitude_semantics says what the altitude is.
  gnss.csv     t_s, lat_deg, lon_deg, alt_m, fix_type, hacc_m, vacc_m, ve_mps, vn_mps, vu_mps, nsat
                 alt_m reference (msl / ellipsoid) in meta.sensors.gnss.alt_ref; empty cells = not provided.
  images.csv   t_s, cam, path   (path relative to the sequence directory)
  truth.csv    t_s, e_m, n_m, u_m, lat_deg, lon_deg, alt_m, qw, qx, qy, qz
                 EVALUATOR ONLY. e/n/u: local ENU tangent plane at meta.origin. q: body -> ENU.

Clock: t_s = seconds on one common clock, 0 = first sample of the sequence.
Missing optional columns are empty; missing files mean the sensor does not exist.

Rule enforced here: `load(seq, cut_s=...)` drops every GNSS row with t_s >= cut_s and does not
return truth; truth needs the explicit `load_truth(seq)` call, used only by evaluators.

CLI:
  .venv/bin/python experiments/t_replay.py validate DIR [DIR ...]
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

SCHEMA = "taipeidrift-replay/1"
COLUMNS = {
    "imu.csv": ["t_s", "gx", "gy", "gz", "ax", "ay", "az"],
    "baro.csv": ["t_s", "pressure_pa", "temperature_c", "alt_isa_m"],
    "gnss.csv": ["t_s", "lat_deg", "lon_deg", "alt_m", "fix_type", "hacc_m", "vacc_m",
                 "ve_mps", "vn_mps", "vu_mps", "nsat"],
    "images.csv": ["t_s", "cam", "path"],
    "truth.csv": ["t_s", "e_m", "n_m", "u_m", "lat_deg", "lon_deg", "alt_m", "qw", "qx", "qy", "qz"],
}
SENSOR_FILE = {"imu": "imu.csv", "baro": "baro.csv", "gnss": "gnss.csv", "camera": "images.csv",
               "truth": "truth.csv"}
LABELS = {"MEASURED", "SIMULATED"}


def isa_altitude_m(pressure_pa: np.ndarray, p0: float = 101325.0) -> np.ndarray:
    """International Standard Atmosphere altitude (troposphere)."""
    return 44330.769 * (1.0 - (np.asarray(pressure_pa, float) / p0) ** 0.190263)


@dataclass
class Sequence:
    root: Path
    meta: dict
    imu: pd.DataFrame | None = None
    baro: pd.DataFrame | None = None
    gnss: pd.DataFrame | None = None
    images: pd.DataFrame | None = None
    cut_s: float | None = None
    extra: dict = field(default_factory=dict)

    def image_path(self, row) -> Path:
        return self.root / row["path"]


def _read(root: Path, name: str) -> pd.DataFrame | None:
    p = root / name
    if not p.exists():
        return None
    df = pd.read_csv(p)
    missing = [c for c in COLUMNS[name] if c not in df.columns]
    if missing:
        raise ValueError(f"{p}: missing columns {missing}")
    return df


def load(root: str | Path, cut_s: float | None = None) -> Sequence:
    """Estimator-side loader. GNSS after `cut_s` is removed; truth is never loaded."""
    root = Path(root)
    meta = json.loads((root / "meta.json").read_text())
    if meta.get("schema") != SCHEMA:
        raise ValueError(f"{root}: schema {meta.get('schema')!r} != {SCHEMA!r}")
    seq = Sequence(root, meta, _read(root, "imu.csv"), _read(root, "baro.csv"),
                   _read(root, "gnss.csv"), _read(root, "images.csv"), cut_s)
    if cut_s is not None and seq.gnss is not None:
        seq.gnss = seq.gnss[seq.gnss.t_s < cut_s].reset_index(drop=True)
    return seq


def load_truth(root: str | Path) -> pd.DataFrame:
    """Evaluator-only. Never feed this to an estimator after the GNSS cut."""
    df = _read(Path(root), "truth.csv")
    if df is None:
        raise FileNotFoundError(f"{root}/truth.csv")
    return df


def validate(root: str | Path) -> list[str]:
    """Return a list of problems; empty list = valid. Also prints a one-line summary per file."""
    root = Path(root)
    errs: list[str] = []
    try:
        meta = json.loads((root / "meta.json").read_text())
    except Exception as e:  # noqa: BLE001
        return [f"meta.json unreadable: {e}"]
    for k in ("schema", "sequence", "evidence_label", "source", "origin", "sensors", "licence"):
        if k not in meta:
            errs.append(f"meta.json missing '{k}'")
    if meta.get("schema") != SCHEMA:
        errs.append(f"schema {meta.get('schema')!r} != {SCHEMA!r}")
    if meta.get("evidence_label") not in LABELS:
        errs.append(f"evidence_label must be one of {sorted(LABELS)}")
    sensors = meta.get("sensors", {})
    if "baro" in sensors and "altitude_semantics" not in sensors["baro"]:
        errs.append("sensors.baro.altitude_semantics missing")
    if "truth" in sensors and not sensors["truth"].get("evaluator_only", False):
        errs.append("sensors.truth.evaluator_only must be true")
    for s, fname in SENSOR_FILE.items():
        p = root / fname
        if (s in sensors) != p.exists():
            errs.append(f"sensor '{s}' in meta: {s in sensors}, file {fname} exists: {p.exists()}")
        if not p.exists():
            continue
        df = pd.read_csv(p)
        missing = [c for c in COLUMNS[fname] if c not in df.columns]
        if missing:
            errs.append(f"{fname}: missing columns {missing}")
            continue
        t = df.t_s.to_numpy(float)
        if len(t) and (np.any(~np.isfinite(t)) or np.any(np.diff(t) < 0)):
            errs.append(f"{fname}: t_s not finite and non-decreasing")
        rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else float("nan")
        print(f"  {fname:11s} rows={len(df):7d} t=[{t.min() if len(t) else 0:9.2f}, "
              f"{t.max() if len(t) else 0:9.2f}] s  mean rate {rate:7.2f} Hz")
        if fname == "images.csv":
            miss = [r for r in df.path.head(2000) if not (root / r).exists()]
            if miss:
                errs.append(f"images.csv: {len(miss)} of first 2000 paths missing, e.g. {miss[0]}")
    return errs


def main(argv: list[str]) -> int:
    if len(argv) < 2 or argv[0] != "validate":
        print(__doc__)
        return 2
    bad = 0
    for d in argv[1:]:
        print(d)
        errs = validate(d)
        for e in errs:
            print("  ERROR", e)
        print("  OK" if not errs else f"  {len(errs)} problem(s)")
        bad += bool(errs)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

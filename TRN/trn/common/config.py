"""YAML configuration loading, merging and dot-key overrides.

Every experiment is defined by the base configs in ``configs/`` plus an experiment file
(``configs/experiments/*.yaml``) whose ``overrides`` replace individual values.
"""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "configs"
SECTIONS = ("data", "routes", "trajectory", "imu", "laser", "atmosphere", "onboard_map", "filters", "montecarlo")


def load_yaml(path: str | Path) -> dict[str, Any]:
    """Load one YAML file (relative paths are resolved against the project root)."""
    p = Path(path)
    if not p.is_absolute():
        p = PROJECT_ROOT / p
    with open(p) as f:
        return yaml.safe_load(f) or {}


def load_base_config() -> dict[str, Any]:
    """Load all base configuration sections into one nested dict."""
    return {s: load_yaml(CONFIG_DIR / f"{s}.yaml") for s in SECTIONS}


def set_dotted(cfg: dict[str, Any], key: str, value: Any) -> None:
    """Set ``cfg['a']['b']['c'] = value`` for key ``'a.b.c'`` (the path must exist up to the last level)."""
    parts = key.split(".")
    d = cfg
    for p in parts[:-1]:
        if p not in d:
            raise KeyError(f"override key '{key}': '{p}' not found")
        d = d[p]
    d[parts[-1]] = value


def get_dotted(cfg: dict[str, Any], key: str) -> Any:
    """Read a dotted key."""
    d = cfg
    for p in key.split("."):
        d = d[p]
    return d


def apply_overrides(cfg: dict[str, Any], overrides: dict[str, Any] | None) -> dict[str, Any]:
    """Return a deep copy of ``cfg`` with dotted-key overrides applied."""
    out = copy.deepcopy(cfg)
    for k, v in (overrides or {}).items():
        set_dotted(out, k, v)
    return out


def project_path(rel: str | Path) -> Path:
    """Resolve a project-relative path."""
    p = Path(rel)
    return p if p.is_absolute() else PROJECT_ROOT / p

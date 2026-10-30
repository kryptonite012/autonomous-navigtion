"""Configuration loading for the robot runtime."""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any

import yaml

ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = ROOT_DIR / "config" / "nexus.yaml"


def _deep_merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def resolve_path(path: str | os.PathLike) -> Path:
    p = Path(path)
    return p if p.is_absolute() else ROOT_DIR / p


def load_config(path: str | os.PathLike | None = None, overrides: dict | None = None) -> dict[str, Any]:
    """Load the YAML config, apply overrides and environment variables."""
    cfg_path = resolve_path(path or os.environ.get("NEXUS_CONFIG", DEFAULT_CONFIG))
    with open(cfg_path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh) or {}

    mode = os.environ.get("NEXUS_MODE")
    if mode:
        cfg.setdefault("robot", {})["mode"] = mode

    if overrides:
        cfg = _deep_merge(cfg, overrides)

    if cfg["robot"]["mode"] not in ("simulation", "hardware"):
        raise ValueError(f"invalid robot.mode {cfg['robot']['mode']!r}")
    return cfg


def save_section(path: str | os.PathLike | None, section: str, value: Any) -> None:
    """Persist one top level section back to the YAML file.

    The file is rewritten without its comments, so the previous version is
    kept as ``<name>.yaml.bak``.
    """
    cfg_path = resolve_path(path or os.environ.get("NEXUS_CONFIG", DEFAULT_CONFIG))
    with open(cfg_path, "r", encoding="utf-8") as fh:
        original = fh.read()
    cfg = yaml.safe_load(original) or {}
    cfg_path.with_suffix(".yaml.bak").write_text(original, encoding="utf-8")
    cfg[section] = value
    tmp = cfg_path.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as fh:
        yaml.safe_dump(cfg, fh, sort_keys=False)
    tmp.replace(cfg_path)

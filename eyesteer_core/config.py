"""Config loading shared by every runtime (standalone, ROS nodes, emulators)."""
from __future__ import annotations

import json
import os
from functools import lru_cache
from pathlib import Path

import yaml

ROOT = Path(os.environ.get("EYESTEER_ROOT", Path(__file__).resolve().parents[1]))
CONFIG = ROOT / "config"
ROBOT_FILE = {"field_robot": "robot_field.yaml", "walker": "robot_walker.yaml"}


def _load(p: Path) -> dict:
    return yaml.safe_load(Path(p).read_text())


@lru_cache(maxsize=None)
def hardware() -> dict:
    return _load(CONFIG / "hardware.yaml")


@lru_cache(maxsize=None)
def bci() -> dict:
    return _load(CONFIG / "bci.yaml")


@lru_cache(maxsize=None)
def robot(name: str) -> dict:
    return _load(CONFIG / ROBOT_FILE[name])


@lru_cache(maxsize=None)
def geometry(name: str) -> dict:
    """Generated geometry (footprint polygon etc.), see tools/generate_models.py."""
    return json.loads((CONFIG / "generated" / f"{name}_geometry.json").read_text())


def frequency_table(refresh_hz: float | None = None) -> dict:
    """The served frequency table (config/generated/frequencies.json) or a recomputation."""
    p = CONFIG / "generated" / "frequencies.json"
    if refresh_hz is None and p.exists():
        return json.loads(p.read_text())
    from eyesteer_core.world import load_frequency_table
    return load_frequency_table(refresh_hz)


def scenario(name: str) -> dict:
    return _load(CONFIG / "scenarios" / f"{name}.yaml")

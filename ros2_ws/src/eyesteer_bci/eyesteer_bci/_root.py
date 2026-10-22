"""Locate the eyesteer repository root so nodes can import the shared ROS-free core
(eyesteer_core) and the shared MuJoCo world (standalone.sim). Order: $EYESTEER_ROOT, then walk up
from this file's real path (colcon --symlink-install), then the root recorded at build time."""
import os
import sys
from pathlib import Path


def find_root() -> Path:
    env = os.environ.get("EYESTEER_ROOT")
    if env and (Path(env) / "eyesteer_core").is_dir():
        return Path(env)
    for p in Path(os.path.realpath(__file__)).parents:
        if (p / "eyesteer_core").is_dir() and (p / "config").is_dir():
            return p
    try:
        from ament_index_python.packages import get_package_share_directory
        rec = Path(get_package_share_directory("eyesteer_bringup")) / "eyesteer_root.txt"
        if rec.exists():
            return Path(rec.read_text().strip())
    except Exception:
        pass
    raise RuntimeError("cannot locate the eyesteer repository: set EYESTEER_ROOT")


ROOT = find_root()
os.environ.setdefault("EYESTEER_ROOT", str(ROOT))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

"""World + marker geometry shared by the standalone sim, the MuJoCo bridge and the map generator.

ROS-free. The map frame equals the sim world frame (origin at the world center).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"


@dataclass
class Marker:
    id: int
    label: str
    slot: object                 # 1..5 or "CANCEL"
    x: float
    y: float
    z: float = 1.4
    facing: float = 0.0
    cancel: bool = False
    frequency: float = 0.0       # filled from the frequency table

    def to_dict(self) -> dict:
        return {k: getattr(self, k) for k in ("id", "label", "slot", "x", "y", "z", "facing", "cancel", "frequency")}


@dataclass
class World:
    name: str
    bounds: tuple
    fence: dict
    obstacles: list
    people: list
    start_pose: tuple
    field_polygon: list
    default_markers: str
    raw: dict = field(default_factory=dict)


def load_yaml(p: Path) -> dict:
    return yaml.safe_load(Path(p).read_text(encoding="utf-8"))


def load_world(name: str) -> World:
    w = load_yaml(CONFIG / "worlds" / f"{name}.yaml")
    return World(name=w["name"], bounds=tuple(w["bounds"]), fence=w["fence"], obstacles=w["obstacles"],
                 people=w.get("people", []), start_pose=tuple(w["start_pose"]), field_polygon=w["field_polygon"],
                 default_markers=w["default_markers"], raw=w)


def load_frequency_table(refresh_hz: float | None = None) -> dict:
    """The single frequency table (D6); recomputed if a different refresh rate is requested."""
    import sys
    sys.path.insert(0, str(ROOT / "tools"))
    from compute_frequencies import compute, configured_refresh, load_rules  # noqa: E402
    return compute(refresh_hz or configured_refresh(), load_rules())


def load_markers(name: str, table: dict | None = None) -> list[Marker]:
    m = load_yaml(CONFIG / "markers" / f"{name}.yaml")
    table = table or load_frequency_table()
    by_slot = {str(s["slot"]): s["frequency_exact"] for s in table["slots"]}
    out = []
    for d in m["markers"]:
        mk = Marker(**{k: d[k] for k in ("id", "label", "slot", "x", "y")}, z=d.get("z", 1.4),
                    facing=d.get("facing", 0.0), cancel=bool(d.get("cancel", False)))
        mk.frequency = by_slot.get(str(mk.slot), 0.0)
        if mk.frequency == 0.0:
            continue  # slot not available at this refresh rate (reduced set)
        out.append(mk)
    return out


# ------------------------------------------------------------------ obstacle geometry
def wall_boxes(w: World) -> list[dict]:
    x0, y0, x1, y1 = w.bounds
    t, h = w.fence["thickness"], w.fence["height"]
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    return [
        {"name": "wall_s", "type": "box", "x": cx, "y": y0 - t / 2, "sx": x1 - x0 + 2 * t, "sy": t, "h": h, "yaw": 0.0},
        {"name": "wall_n", "type": "box", "x": cx, "y": y1 + t / 2, "sx": x1 - x0 + 2 * t, "sy": t, "h": h, "yaw": 0.0},
        {"name": "wall_w", "type": "box", "x": x0 - t / 2, "y": cy, "sx": t, "sy": y1 - y0, "h": h, "yaw": 0.0},
        {"name": "wall_e", "type": "box", "x": x1 + t / 2, "y": cy, "sx": t, "sy": y1 - y0, "h": h, "yaw": 0.0},
    ]


def static_shapes(w: World) -> list[dict]:
    return wall_boxes(w) + list(w.obstacles)


def shape_sdf(shape: dict, px: np.ndarray, py: np.ndarray) -> np.ndarray:
    """Signed distance (m) from points to a static 2D shape (negative inside)."""
    if shape["type"] == "cylinder":
        return np.hypot(px - shape["x"], py - shape["y"]) - shape["r"]
    c, s = math.cos(-shape.get("yaw", 0.0)), math.sin(-shape.get("yaw", 0.0))
    dx, dy = px - shape["x"], py - shape["y"]
    lx, ly = c * dx - s * dy, s * dx + c * dy
    qx, qy = np.abs(lx) - shape["sx"] / 2, np.abs(ly) - shape["sy"] / 2
    outside = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
    inside = np.minimum(np.maximum(qx, qy), 0)
    return outside + inside


def static_sdf(w: World, px, py) -> np.ndarray:
    px, py = np.asarray(px, float), np.asarray(py, float)
    d = np.full(np.broadcast(px, py).shape, np.inf)
    for sh in static_shapes(w):
        d = np.minimum(d, shape_sdf(sh, px, py))
    return d


@dataclass
class Grid:
    origin: tuple          # (x, y) of cell (0, 0) lower-left corner
    resolution: float
    occ: np.ndarray        # bool [ny, nx], True = occupied

    def world_to_cell(self, x, y):
        return int((y - self.origin[1]) / self.resolution), int((x - self.origin[0]) / self.resolution)

    def cell_to_world(self, r, c):
        return self.origin[0] + (c + 0.5) * self.resolution, self.origin[1] + (r + 0.5) * self.resolution

    def in_bounds(self, r, c):
        return 0 <= r < self.occ.shape[0] and 0 <= c < self.occ.shape[1]


def rasterize(w: World, resolution: float = 0.05, margin: float = 0.5) -> Grid:
    x0, y0, x1, y1 = w.bounds
    x0, y0, x1, y1 = x0 - margin, y0 - margin, x1 + margin, y1 + margin
    nx, ny = int(round((x1 - x0) / resolution)), int(round((y1 - y0) / resolution))
    xs = x0 + (np.arange(nx) + 0.5) * resolution
    ys = y0 + (np.arange(ny) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    sdf = static_sdf(w, gx, gy)
    occ = sdf <= resolution * 0.5
    # outside the fence counts as occupied too (unknown space beyond the walls)
    t = w.fence["thickness"]
    occ |= (gx < w.bounds[0] - t) | (gx > w.bounds[2] + t) | (gy < w.bounds[1] - t) | (gy > w.bounds[3] + t)
    return Grid(origin=(x0, y0), resolution=resolution, occ=occ)


def write_ros_map(grid: Grid, stem: Path, name: str) -> None:
    """Write a map_server-compatible PGM + YAML (0 = occupied, 254 = free)."""
    stem.parent.mkdir(parents=True, exist_ok=True)
    img = np.where(grid.occ, 0, 254).astype(np.uint8)[::-1]  # PGM row 0 = top = max y
    h, wd = img.shape
    with open(stem.with_suffix(".pgm"), "wb") as f:
        f.write(f"P5\n# eyesteer {name}\n{wd} {h}\n255\n".encode())
        f.write(img.tobytes())
    meta = {"image": stem.with_suffix(".pgm").name, "mode": "trinary", "resolution": grid.resolution,
            "origin": [round(grid.origin[0], 4), round(grid.origin[1], 4), 0.0], "negate": 0,
            "occupied_thresh": 0.65, "free_thresh": 0.25}
    stem.with_suffix(".yaml").write_text(yaml.safe_dump(meta, sort_keys=False))


# ------------------------------------------------------------------ marker goals
def marker_goal(marker: Marker, approach: float, clearance_fn=None, min_clearance: float = 0.0):
    """Goal pose ~approach m in front of the anchored point, facing it.

    If clearance_fn(x, y, yaw) < min_clearance, search nearby poses (rings of increasing radius)
    for the nearest free one, keeping the heading towards the marker.
    """
    gx = marker.x + approach * math.cos(marker.facing)
    gy = marker.y + approach * math.sin(marker.facing)
    yaw = math.atan2(marker.y - gy, marker.x - gx)
    if clearance_fn is None or clearance_fn(gx, gy, yaw) >= min_clearance:
        return gx, gy, yaw
    for rad in np.arange(0.05, 1.5, 0.05):
        best = None
        for a in np.linspace(0, 2 * math.pi, 24, endpoint=False):
            x, y = gx + rad * math.cos(a), gy + rad * math.sin(a)
            yw = math.atan2(marker.y - y, marker.x - x)
            c = clearance_fn(x, y, yw)
            if c >= min_clearance and (best is None or c > best[0]):
                best = (c, x, y, yw)
        if best:
            return best[1], best[2], best[3]
    return gx, gy, yaw

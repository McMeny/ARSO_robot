"""Reads the objects (boxes and cylinders) out of a Gazebo world .sdf file.

Shared by world_markers (RViz), hazard_tracker (which objects are hazards) and the
VR web server (which draws the same store in the headset).
"""
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass
class WorldObject:
    name: str
    kind: str      # 'box' or 'cylinder'
    x: float       # centre, world (= map) frame
    y: float
    z: float
    yaw: float
    size_x: float  # footprint; for a cylinder both are the diameter
    size_y: float
    height: float
    colour: tuple  # (r, g, b), 0..1

    def distance_to(self, px, py):
        """Distance from a point on the floor to the edge of this object's footprint (0 if inside)."""
        dx, dy = px - self.x, py - self.y
        if self.kind == 'cylinder':
            return max(math.hypot(dx, dy) - self.size_x / 2, 0.0)
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        lx, ly = c * dx - s * dy, s * dx + c * dy
        return math.hypot(max(abs(lx) - self.size_x / 2, 0.0), max(abs(ly) - self.size_y / 2, 0.0))


def load_world(world_file):
    objects = []
    for model in ET.parse(world_file).getroot().iter('model'):
        geometry = model.find('.//visual/geometry')
        pose = model.find('pose')
        if geometry is None or pose is None:
            continue  # e.g. the ground plane
        x, y, z, _, _, yaw = (float(v) for v in pose.text.split())
        box, cylinder = geometry.find('box'), geometry.find('cylinder')
        if box is not None:
            sx, sy, h = (float(v) for v in box.find('size').text.split())
            kind = 'box'
        elif cylinder is not None:
            sx = sy = 2 * float(cylinder.find('radius').text)
            h = float(cylinder.find('length').text)
            kind = 'cylinder'
        else:
            continue
        diffuse = model.find('.//visual/material/diffuse')
        colour = tuple(float(v) for v in diffuse.text.split()[:3]) if diffuse is not None else (0.7, 0.7, 0.7)
        objects.append(WorldObject(model.get('name', ''), kind, x, y, z, yaw, sx, sy, h, colour))
    return objects

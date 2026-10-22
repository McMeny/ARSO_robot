"""Headset (WebXR local-floor) <-> robot map frame calibration (Section 14.4). Mirrored in webxr/js/calib.js.

WebXR local-floor: x right, y up, z towards the viewer (right-handed), y = 0 on the floor.
Map (REP-103): x forward, y left, z up. On the ground plane we use u = -z_xr, v = -x_xr, which is a
right-handed (forward, left) pair with up = y_xr, so headset -> map is a pure 2D rotation + translation:
    [x_map, y_map] = R(yaw) [u, v] + [tx, ty],     z_map = y_xr - floor_height
Two-point co-location: the user touches the robot's front and rear decals (known positions in the
robot frame) while the robot is localized and stationary.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class XrCalibration:
    yaw: float
    tx: float
    ty: float
    floor_height: float
    residual: float


def xr_plane(p) -> tuple[float, float]:
    return -p[2], -p[0]


def decal_in_map(robot_pose, decal_xy) -> tuple[float, float]:
    x, y, yaw = robot_pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * decal_xy[0] - s * decal_xy[1], y + s * decal_xy[0] + c * decal_xy[1]


def solve(h_front, h_rear, robot_pose, decal_front, decal_rear, plate_height: float) -> XrCalibration:
    """h_*: headset points (x, y, z); decal_*: (x, y) in the robot frame; robot_pose: (x, y, yaw) in map."""
    hf, hr = xr_plane(h_front), xr_plane(h_rear)
    df, dr = decal_in_map(robot_pose, decal_front), decal_in_map(robot_pose, decal_rear)
    yaw = math.atan2(df[1] - dr[1], df[0] - dr[0]) - math.atan2(hf[1] - hr[1], hf[0] - hr[0])
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    c, s = math.cos(yaw), math.sin(yaw)
    hm = ((hf[0] + hr[0]) / 2, (hf[1] + hr[1]) / 2)
    dm = ((df[0] + dr[0]) / 2, (df[1] + dr[1]) / 2)
    tx = dm[0] - (c * hm[0] - s * hm[1])
    ty = dm[1] - (s * hm[0] + c * hm[1])
    floor = (h_front[1] + h_rear[1]) / 2 - plate_height
    # residual: decal spacing mismatch + per-point fit error + height disagreement
    len_h = math.hypot(hf[0] - hr[0], hf[1] - hr[1])
    len_d = math.hypot(df[0] - dr[0], df[1] - dr[1])
    cal = XrCalibration(yaw, tx, ty, floor, 0.0)
    errs = []
    for h, d in ((h_front, df), (h_rear, dr)):
        m = headset_to_map(h, cal)
        errs.append(math.hypot(m[0] - d[0], m[1] - d[1]))
    cal.residual = max(abs(len_h - len_d), max(errs), abs(h_front[1] - h_rear[1]))
    return cal


def headset_to_map(p, cal: XrCalibration) -> tuple[float, float, float]:
    u, v = xr_plane(p)
    c, s = math.cos(cal.yaw), math.sin(cal.yaw)
    return c * u - s * v + cal.tx, s * u + c * v + cal.ty, p[1] - cal.floor_height


def map_to_headset(p, cal: XrCalibration) -> tuple[float, float, float]:
    c, s = math.cos(cal.yaw), math.sin(cal.yaw)
    dx, dy = p[0] - cal.tx, p[1] - cal.ty
    u, v = c * dx + s * dy, -s * dx + c * dy
    return -v, p[2] + cal.floor_height, -u


def yaw_headset_to_map(yaw_xr: float, cal: XrCalibration) -> float:
    """A heading in the XR ground plane (radians, about +y, 0 = -z) -> map yaw."""
    return math.atan2(math.sin(yaw_xr + cal.yaw), math.cos(yaw_xr + cal.yaw))

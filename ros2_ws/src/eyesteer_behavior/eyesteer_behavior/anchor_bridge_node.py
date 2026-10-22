"""anchor_bridge_node: virtual markers between the headset and the robot's map frame.

 - /markers (latched, map frame): the marker set every node uses (frequencies from the served table)
 - /markers_viz: RViz overlay (translucent panels + labels, never obstacles)
 - /xr/anchors (headset frame, from the WebXR app) -> transformed with the calibration -> /markers
 - /xr/compute_calibration (ComputeHeadsetCalibration): two decal points + robot map pose -> transform
 - persistence: ~/.eyesteer/markers/<map>.yaml (reloaded on startup); falls back to the default layout
"""
from __future__ import annotations

import math
import os
from pathlib import Path

from eyesteer_behavior import _root  # noqa: F401

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker as VizMarker
from visualization_msgs.msg import MarkerArray as VizMarkerArray

from eyesteer_core import config
from eyesteer_core.world import load_markers
from eyesteer_core.xr_calibration import XrCalibration, headset_to_map, solve

from eyesteer_interfaces.msg import HeadsetCalibration, Marker, MarkerArray
from eyesteer_interfaces.srv import ComputeHeadsetCalibration

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
SERIES = [(0.22, 0.53, 0.9), (0.85, 0.35, 0.15), (0.1, 0.62, 0.44), (0.79, 0.52, 0.0), (0.84, 0.32, 0.51)]


class AnchorBridge(Node):
    def __init__(self):
        super().__init__("anchor_bridge")
        self.map_name = self.declare_parameter("map_name", "field").value
        self.default_set = self.declare_parameter("default_markers", "").value or (
            "supermarket_default" if self.map_name == "supermarket" else "field_default")
        self.robot = self.declare_parameter("robot", "field_robot").value
        self.persist = self.declare_parameter("persist", True).value
        self.store = Path(os.path.expanduser("~/.eyesteer/markers")) / f"{self.map_name}.yaml"
        self.table = config.frequency_table()
        self.freq = {str(s["slot"]): s["frequency_exact"] for s in self.table["slots"]}
        self.cal: XrCalibration | None = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(MarkerArray, "/markers", LATCHED)
        self.pub_viz = self.create_publisher(VizMarkerArray, "/markers_viz", LATCHED)
        self.pub_cal = self.create_publisher(HeadsetCalibration, "/xr/calibration", LATCHED)
        self.create_subscription(MarkerArray, "/xr/anchors", self.on_anchors, 10)
        self.create_subscription(HeadsetCalibration, "/xr/calibration_set", self.on_cal_set, 10)
        self.create_service(ComputeHeadsetCalibration, "/xr/compute_calibration", self.on_compute)
        self.markers = self.load()
        self.publish()

    # ------------------------------------------------------------------ persistence
    def load(self) -> list[dict]:
        if self.persist and self.store.exists():
            data = yaml.safe_load(self.store.read_text())
            self.get_logger().info(f"loaded {len(data['markers'])} markers from {self.store}")
            return data["markers"]
        return [m.to_dict() for m in load_markers(self.default_set, self.table)]

    def save(self):
        if not self.persist:
            return
        self.store.parent.mkdir(parents=True, exist_ok=True)
        self.store.write_text(yaml.safe_dump({"map": self.map_name, "frame_id": "map", "markers": self.markers}, sort_keys=False))

    def publish(self):
        ma = MarkerArray()
        ma.header.frame_id = "map"
        ma.header.stamp = self.get_clock().now().to_msg()
        ma.table_hash = self.table["hash"]
        viz = VizMarkerArray()
        for i, d in enumerate(self.markers):
            f = self.freq.get(str(d["slot"]), 0.0)
            if f == 0.0:
                continue
            m = Marker(id=int(d["id"]), label=str(d["label"]), slot=str(d["slot"]), frequency=float(f),
                       position=Point(x=float(d["x"]), y=float(d["y"]), z=float(d.get("z", 1.4))),
                       facing=float(d.get("facing", 0.0)), cancel=bool(d.get("cancel", False)))
            ma.markers.append(m)
            col = (0.75, 0.75, 0.7) if m.cancel else SERIES[(int(d["id"]) - 1) % 5]
            pv = VizMarker()
            pv.header = ma.header
            pv.ns, pv.id, pv.type, pv.action = "mr_markers", m.id, VizMarker.CUBE, VizMarker.ADD
            pv.pose.position = m.position
            pv.pose.orientation.z, pv.pose.orientation.w = math.sin(m.facing / 2), math.cos(m.facing / 2)
            pv.scale.x, pv.scale.y, pv.scale.z = 0.02, 0.3, 0.3
            pv.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=0.6)
            tv = VizMarker()
            tv.header = ma.header
            tv.ns, tv.id, tv.type, tv.action = "mr_labels", m.id, VizMarker.TEXT_VIEW_FACING, VizMarker.ADD
            tv.pose.position = Point(x=m.position.x, y=m.position.y, z=m.position.z + 0.3)
            tv.scale.z = 0.18
            tv.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            tv.text = f"{m.label} {m.frequency:.2f} Hz"
            viz.markers += [pv, tv]
        self.pub.publish(ma)
        self.pub_viz.publish(viz)

    # ------------------------------------------------------------------ headset input
    def on_anchors(self, msg: MarkerArray):
        if msg.header.frame_id not in ("map", "headset"):
            self.get_logger().warn(f"anchors in unknown frame {msg.header.frame_id}")
            return
        if msg.header.frame_id == "headset" and self.cal is None:
            self.get_logger().warn("anchors received before headset calibration: ignored")
            return
        out = []
        for m in msg.markers:
            if msg.header.frame_id == "headset":
                x, y, z = headset_to_map((m.position.x, m.position.y, m.position.z), self.cal)
                facing = math.atan2(math.sin(m.facing + self.cal.yaw), math.cos(m.facing + self.cal.yaw))
            else:
                x, y, z, facing = m.position.x, m.position.y, m.position.z, m.facing
            out.append({"id": int(m.id), "label": m.label, "slot": m.slot if m.slot == "CANCEL" else int(m.slot),
                        "x": round(x, 3), "y": round(y, 3), "z": round(z, 3), "facing": round(facing, 4),
                        "cancel": bool(m.cancel or m.slot == "CANCEL")})
        self.markers = out
        self.save()
        self.publish()
        self.get_logger().info(f"{len(out)} markers placed from the headset ({msg.header.frame_id} frame) -> map")

    def robot_pose(self):
        t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def on_compute(self, req, resp):
        try:
            pose = self.robot_pose()
        except Exception as e:
            resp.success, resp.message = False, f"robot not localized (map->base_footprint unavailable): {e}"
            return resp
        dec = config.robot(self.robot)["calibration_decals"]
        cal = solve((req.front.x, req.front.y, req.front.z), (req.rear.x, req.rear.y, req.rear.z), pose,
                    (dec["front"]["x"], dec["front"]["y"]), (dec["rear"]["x"], dec["rear"]["y"]), dec["plate_height"])
        self.cal = cal
        m = HeadsetCalibration(yaw=cal.yaw, tx=cal.tx, ty=cal.ty, floor_height=cal.floor_height,
                               residual=float(cal.residual), session=req.session)
        m.stamp = self.get_clock().now().to_msg()
        self.pub_cal.publish(m)
        resp.success = cal.residual <= 0.05
        resp.message = f"residual {cal.residual * 100:.1f} cm" + ("" if resp.success else " > 5 cm: redo the calibration")
        resp.calibration = m
        self.get_logger().info(f"headset calibration: yaw {math.degrees(cal.yaw):.1f} deg, t=({cal.tx:.2f}, {cal.ty:.2f}), {resp.message}")
        return resp

    def on_cal_set(self, m: HeadsetCalibration):
        self.cal = XrCalibration(m.yaw, m.tx, m.ty, m.floor_height, m.residual)
        self.pub_cal.publish(m)


def main():
    rclpy.init()
    n = AnchorBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

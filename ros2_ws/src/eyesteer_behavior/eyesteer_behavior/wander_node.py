"""wander_node: field robot idle behaviour (proposal 'autonomous state'). When /wander/enable is true,
sends random reachable NavigateToPose goals >= 2 m away inside the field polygon. Generic obstacle
avoidance is Nav2's job. The walker never enables this (D11)."""
from __future__ import annotations

import math
import random

from eyesteer_behavior import _root  # noqa: F401

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from eyesteer_core.control.footprint import point_in_poly
from eyesteer_core.world import load_world

from eyesteer_behavior.mode_manager_node import MapClearance

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class WanderNode(Node):
    def __init__(self):
        super().__init__("wander")
        world = self.declare_parameter("world", "field").value
        poly = self.declare_parameter("field_polygon", "").value
        self.speed_note = self.declare_parameter("min_goal_distance", 2.0).value
        self.poly = np.array(eval(poly) if poly else load_world(world).field_polygon, float)  # noqa: S307 (param from launch)
        # SLAM mapping: the map frame is the start pose (ox, oy, oyaw in the world) -> polygon into that frame
        ox, oy, oyaw = self.declare_parameter("slam_origin", [0.0, 0.0, 0.0]).value
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        d = self.poly - [ox, oy]
        self.poly = np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], 1)
        self.explore = self.declare_parameter("explore", False).value   # unknown cells count as free
        self.rng = random.Random(self.declare_parameter("seed", 3).value)
        self.enabled = False
        self.clear = None
        self.gh = None
        self.busy = False
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(Bool, "/wander/enable", self.on_enable, LATCHED)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "clear", MapClearance(m, self.explore)), LATCHED)
        self.create_timer(0.5, self.tick)

    def on_enable(self, m: Bool):
        self.enabled = m.data
        if not m.data and self.gh is not None:
            self.gh.cancel_goal_async()
            self.gh = None
        self.busy = False if not m.data else self.busy

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def tick(self):
        if not self.enabled or self.busy or self.clear is None:
            return
        p = self.pose()
        if p is None or not self.nav.server_is_ready():
            return
        lo, hi = self.poly.min(0), self.poly.max(0)
        for _ in range(300):
            gx, gy = self.rng.uniform(lo[0], hi[0]), self.rng.uniform(lo[1], hi[1])
            if math.hypot(gx - p[0], gy - p[1]) < self.speed_note:
                continue
            if not point_in_poly(np.array([[gx, gy]]), self.poly)[0]:
                continue
            if float(self.clear.sdf(np.array([gx]), np.array([gy]))[0]) < 0.6:
                continue
            break
        else:
            return
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
        yaw = math.atan2(gy - p[1], gx - p[0])
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.busy = True
        self.nav.send_goal_async(g).add_done_callback(self.on_resp)

    def on_resp(self, fut):
        gh = fut.result()
        if not gh.accepted or not self.enabled:
            self.busy = False
            if gh.accepted:
                gh.cancel_goal_async()
            return
        self.gh = gh
        gh.get_result_async().add_done_callback(lambda f: setattr(self, "busy", False))


def main():
    rclpy.init()
    n = WanderNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""mode_manager_node: wraps eyesteer_core.fsm.ModeManager. Selections in -> Nav2 NavigateToPose goals out.
Never outputs velocities (D7). Publishes /robot/status (10 Hz) for the headset, RViz and the dashboard."""
from __future__ import annotations

import json
import math
import time

from eyesteer_behavior import _root  # noqa: F401

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import distance_transform_edt
from std_msgs.msg import Bool, Empty, String

from eyesteer_core import config
from eyesteer_core.control.footprint import Footprint
from eyesteer_core.fsm.mode_manager import ModeManager
from eyesteer_core.world import Marker, marker_goal

from eyesteer_interfaces.msg import BciStatus, MarkerArray, RobotStatus, Selection, XrTiming

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class MapClearance:
    """Footprint clearance against the /map occupancy grid (works identically in sim and real)."""

    def __init__(self, grid: OccupancyGrid):
        info = grid.info
        occ = np.array(grid.data, np.int16).reshape(info.height, info.width)
        self.res = info.resolution
        self.ox, self.oy = info.origin.position.x, info.origin.position.y
        self.dist = distance_transform_edt(~((occ > 50) | (occ < 0))) * self.res

    def sdf(self, px, py):
        c = ((np.asarray(px) - self.ox) / self.res).astype(int)
        r = ((np.asarray(py) - self.oy) / self.res).astype(int)
        inside = (r >= 0) & (r < self.dist.shape[0]) & (c >= 0) & (c < self.dist.shape[1])
        out = np.zeros(np.shape(px))
        out[inside] = self.dist[r[inside], c[inside]]
        return out


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__("mode_manager")
        self.robot = self.declare_parameter("robot", "field_robot").value
        self.lost_cov = self.declare_parameter("localization_lost_cov", 1.0).value
        self.headset_timeout = self.declare_parameter("headset_timeout", 3.0).value
        self.rcfg = config.robot(self.robot)
        self.fp = Footprint(config.geometry(self.robot)["footprint"])
        self.fsm = ModeManager(idle_mode=self.rcfg["idle_mode"])
        self.markers: dict[int, Marker] = {}
        self.clear = None
        self.goal_handle = None
        self.goal_seq = 0
        self.mux_source = "NONE"
        self.safety = "OK"
        self.deadman = False
        self.estop = False
        self.motion_locked = True
        self.eeg_source = "SIMULATED"
        self.bci_ok = None
        self.headset_seen = None
        self.headset_ok = None
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.pub_lat = self.create_publisher(String, "/eyesteer/latency", 10)
        self.lat = {"selection": None, "accepted": None, "moving_reported": True}
        self.last_display = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.loc_bad_since = None
        self.loc_ok = True
        self.create_subscription(LaserScan, "/scan", self.on_scan, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        # clock-offset ping from the headset (NTP-style), so latencies across machines are measurable
        self.create_subscription(String, "/eyesteer/ping", lambda m: self.event("pong", ping=m.data), 10)
        # the headset acknowledges each state it displays: change -> ack on ONE clock (conservative upper bound)
        self.state_t = {}
        self.create_subscription(String, "/eyesteer/status_ack", self.on_ack, 10)
        self.pub_status = self.create_publisher(RobotStatus, "/robot/status", 10)
        self.pub_wander = self.create_publisher(Bool, "/wander/enable", LATCHED)
        self.create_subscription(Selection, "/bci/selection", self.on_selection, 10)
        self.create_subscription(MarkerArray, "/markers", self.on_markers, LATCHED)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "clear", MapClearance(m)), LATCHED)
        self.create_subscription(Bool, "/estop", self.on_estop, 10)
        self.create_subscription(Bool, "/lock/estop", lambda m: setattr(self, "estop", m.data), 10)
        self.create_subscription(Empty, "/system/reset", self.on_reset, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, LATCHED)
        self.create_subscription(Bool, "/localization/ok", lambda m: self.run_cmds(self.fsm.on_localization(self.now(), m.data)), 10)
        self.create_subscription(BciStatus, "/bci/status", self.on_bci, 10)
        self.create_subscription(XrTiming, "/xr/timing", self.on_xr, 10)
        self.create_subscription(String, "/mux/source", lambda m: setattr(self, "mux_source", m.data), 10)
        self.create_subscription(String, "/collision_monitor/state", lambda m: setattr(self, "safety", m.data), 10)
        self.create_subscription(Bool, "/lock/deadman", lambda m: setattr(self, "deadman", m.data), 10)
        self.create_subscription(Bool, "/lock/health", lambda m: setattr(self, "motion_locked", m.data), 10)
        self.create_subscription(String, "/bci/eeg_source", lambda m: setattr(self, "eeg_source", m.data), LATCHED)
        self.create_timer(0.1, self.tick)
        self.run_cmds([("idle", self.fsm.idle_mode)])
        self.get_logger().info(f"mode_manager: robot={self.robot}, idle={self.fsm.idle_mode}")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def on_markers(self, m: MarkerArray):
        self.markers = {x.id: Marker(id=x.id, label=x.label, slot=x.slot, x=x.position.x, y=x.position.y, z=x.position.z,
                                     facing=x.facing, cancel=x.cancel, frequency=x.frequency) for x in m.markers}
        self.fsm.set_markers(list(self.markers), [i for i, mk in self.markers.items() if mk.cancel])

    def on_selection(self, m: Selection):
        t_sel = time.time()
        self.event("selection", marker=m.marker_id, label=m.label)
        cmds = self.fsm.on_selection(m.marker_id, self.now())
        if any(c[0] == "send_goal" for c in cmds):   # only selections that start a goal (not queued ones) are timed
            self.lat["selection"] = t_sel
        self.run_cmds(cmds)

    # ------------------------------------------------------------------ latency instrumentation (Section 24.7)
    def event(self, name, **kw):
        self.pub_lat.publish(String(data=json.dumps({"event": name, "wall": time.time(), **kw})))

    def on_odom(self, m: Odometry):
        moving = abs(m.twist.twist.linear.x) + abs(m.twist.twist.angular.z) > 0.03
        if not self.lat["moving_reported"] and self.lat["accepted"] and moving:
            self.lat["moving_reported"] = True
            self.event("moving", dt_ms=1000 * (time.time() - self.lat["accepted"]))

    def on_ack(self, m: String):
        t0 = self.state_t.pop(m.data, None)
        if t0 is not None:
            self.event("state_visible", display=m.data, dt_ms=1000 * (time.time() - t0))

    # ------------------------------------------------------------------ localization consistency (24.5)
    def on_scan(self, s: LaserScan):
        """Fraction of LiDAR hits near an occupied /map cell at the current map pose. A teleported
        (kidnapped) robot keeps a confident but wrong AMCL pose; this catches it."""
        if self.clear is None:
            return
        try:
            t = self.tf.lookup_transform("map", s.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        r = np.asarray(s.ranges, float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, 8.0))
        if ok.sum() < 20:
            return
        x = t.transform.translation.x + r[ok] * np.cos(a[ok] + yaw)
        y = t.transform.translation.y + r[ok] * np.sin(a[ok] + yaw)
        ratio = float(np.mean(self.clear.sdf(x, y) < 0.25))
        now = self.now()
        if ratio < 0.4:
            self.loc_bad_since = self.loc_bad_since or now
            if self.loc_ok and now - self.loc_bad_since > 2.0:
                self.loc_ok = False
                self.get_logger().error(f"LOCALIZATION LOST: only {ratio * 100:.0f} % of the scan matches the map")
                self.run_cmds(self.fsm.on_localization(now, False))
        else:
            self.loc_bad_since = None
            if not self.loc_ok and ratio > 0.7:
                self.loc_ok = True
                self.get_logger().info(f"localization recovered ({ratio * 100:.0f} % scan match)")
                self.run_cmds(self.fsm.on_localization(now, True))

    def on_estop(self, m: Bool):
        if m.data:
            self.run_cmds(self.fsm.on_estop(self.now(), True))

    def on_reset(self, _):
        self.run_cmds(self.fsm.reset(self.now()))

    def on_amcl(self, m: PoseWithCovarianceStamped):
        c = m.pose.covariance
        if (c[0] + c[7]) > self.lost_cov and self.loc_ok:
            self.loc_ok = False
            self.run_cmds(self.fsm.on_localization(self.now(), False))

    def on_bci(self, m: BciStatus):
        ok = bool(m.signal_ok)
        if self.bci_ok is not None and ok != self.bci_ok:
            self.run_cmds(self.fsm.on_link(self.now(), "bci", ok))
        self.bci_ok = ok

    def on_xr(self, m: XrTiming):
        self.headset_seen = self.now()
        if self.headset_ok is False:
            self.run_cmds(self.fsm.on_link(self.now(), "headset", True))
        self.headset_ok = True

    # ------------------------------------------------------------------ FSM commands -> Nav2
    def run_cmds(self, cmds):
        for c in cmds:
            if c[0] == "send_goal":
                self.send_goal(c[1])
            elif c[0] in ("cancel_goal", "stop"):
                self.cancel_goal()
            elif c[0] == "idle":
                self.cancel_goal()
                self.pub_wander.publish(Bool(data=(c[1] == "WANDER")))
            elif c[0] == "arrived":
                self.get_logger().info(f"ARRIVED at marker {c[1]}")

    def goal_pose(self, mid):
        mk = self.markers[mid]
        approach = self.rcfg["nav"]["approach_distance"]
        fn = None
        if self.clear is not None:
            fn = lambda x, y, yaw: self.fp.clearance(x, y, yaw, self.clear.sdf)  # noqa: E731
        return marker_goal(mk, approach, fn, 0.15)

    def send_goal(self, mid):
        if mid not in self.markers:
            self.get_logger().warn(f"unknown marker {mid}")
            self.run_cmds(self.fsm.on_nav_failed(self.now(), "unknown marker"))
            return
        self.pub_wander.publish(Bool(data=False))
        x, y, yaw = self.goal_pose(mid)
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("navigate_to_pose action server unavailable")
            self.run_cmds(self.fsm.on_nav_failed(self.now(), "Nav2 unavailable"))
            return
        self.goal_seq += 1
        seq = self.goal_seq
        self.get_logger().info(f"NavigateToPose -> {self.markers[mid].label} ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)")
        fut = self.nav.send_goal_async(g)
        fut.add_done_callback(lambda f: self.on_goal_response(f, seq))

    def on_goal_response(self, fut, seq):
        gh = fut.result()
        if seq != self.goal_seq:
            return
        if not gh.accepted:
            self.run_cmds(self.fsm.on_nav_failed(self.now(), "goal rejected"))
            return
        self.goal_handle = gh
        if self.lat["selection"]:
            self.event("goal_accepted", dt_ms=1000 * (time.time() - self.lat["selection"]))
            self.lat["selection"] = None
        self.lat["accepted"], self.lat["moving_reported"] = time.time(), False
        gh.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, fut, seq):
        if seq != self.goal_seq:
            return     # result of a preempted / cancelled goal
        status = fut.result().status
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.run_cmds(self.fsm.on_nav_succeeded(self.now()))
        elif status == GoalStatus.STATUS_ABORTED:
            self.run_cmds(self.fsm.on_nav_failed(self.now(), "Nav2 aborted"))

    def cancel_goal(self):
        self.goal_seq += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    # ------------------------------------------------------------------ periodic
    def tick(self):
        t = self.now()
        if self.headset_seen is not None and t - self.headset_seen > self.headset_timeout and self.headset_ok:
            self.headset_ok = False
            self.run_cmds(self.fsm.on_link(t, "headset", False))
        self.run_cmds(self.fsm.tick(t))
        st = self.fsm.status()
        m = RobotStatus()
        m.stamp = self.get_clock().now().to_msg()
        m.robot, m.state, m.idle_mode = self.robot, st.state, st.idle_mode
        m.goal_id = int(st.goal) if st.goal is not None else -1
        m.goal_label = self.markers[st.goal].label if st.goal in self.markers else ""
        m.display = f"ENGAGED -> {m.goal_label}" if st.state == "ENGAGED" else st.display
        m.queue = [int(q) for q in st.queue]
        m.queue_labels = [self.markers[q].label if q in self.markers else str(q) for q in st.queue]
        m.mux_source, m.safety = self.mux_source, self.safety
        m.estop, m.deadman_locked, m.motion_locked = self.estop or st.state == "ESTOP", self.deadman, self.motion_locked
        m.reason, m.eeg_source = st.reason, self.eeg_source
        self.pub_status.publish(m)
        if m.display != self.last_display:
            self.last_display = m.display
            self.state_t[m.display] = time.time()
            self.event("state", display=m.display)


def main():
    rclpy.init()
    n = ModeManagerNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""health_monitor_node: /system/health (per-component status + reason) and the motion lock (Section 24.4).

/lock/health (twist_mux lock) stays TRUE until every required gate is green (gated startup, D21).
After unlock, a red 'lock_on_red' gate (MCU, LiDAR, TF, Nav2, safety) re-locks motion; BCI/headset
problems are handled by the FSM fallbacks of Section 24.5 instead (no lock), and are shown everywhere.
"""
from __future__ import annotations

import time

from eyesteer_behavior import _root  # noqa: F401

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from eyesteer_core import config

from eyesteer_interfaces.msg import BciStatus, ComponentHealth, MarkerArray, SystemHealth, XrTiming

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
NAV_NODES = ["bt_navigator", "controller_server", "planner_server", "behavior_server", "collision_monitor"]


class Gate:
    def __init__(self, name, required, lock_on_red):
        self.name, self.required, self.lock_on_red = name, required, lock_on_red
        self.level, self.msg, self.value = ComponentHealth.STALE, "no data yet", 0.0
        self.red_since = None

    def set(self, ok: bool, msg: str, value: float = 0.0, warn: bool = False):
        self.level = ComponentHealth.OK if ok else (ComponentHealth.WARN if warn else ComponentHealth.ERROR)
        self.msg, self.value = msg, float(value)


class HealthMonitor(Node):
    def __init__(self):
        super().__init__("health_monitor")
        p = self.declare_parameter
        self.mode = p("mode", "sim").value                    # sim | hil | real
        self.eeg = p("eeg", "synthetic").value
        self.robot = p("robot", "field_robot").value
        self.require_headset = p("require_headset", True).value
        self.slam = p("slam", False).value
        self.cov_max = p("localization_cov_max", 0.5).value
        self.refresh = float(config.hardware()["headset"]["refresh_hz"])
        self.table_hash = config.frequency_table()["hash"]
        hw_mcu = self.mode in ("hil", "real")
        self.g = {
            "muse_lsl": Gate("muse_lsl", True, False),
            "bci": Gate("bci", True, False),
            "headset": Gate("headset", self.require_headset, False),
            "frequency_table": Gate("frequency_table", True, False),
            "lidar": Gate("lidar", True, True),
            "tf": Gate("tf", True, True),
            "localization": Gate("localization", True, False),
            "nav2": Gate("nav2", True, True),
            "mcu": Gate("mcu", hw_mcu, True),
            "safety": Gate("safety", True, True),
        }
        self.bci = None
        self.bci_t = -1e9
        self.xr = None
        self.xr_t = -1e9
        self.marker_hash = None
        self.n_markers = 0
        self.scan_times = []
        self.scan_frame = ""
        self.amcl_cov = None
        self.mcu_alive, self.mcu_t, self.mcu_status = False, -1e9, ""
        self.grip_t = -1e9
        self.nav_states = {}
        self.unlocked = False
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(SystemHealth, "/system/health", 10)
        self.pub_lock = self.create_publisher(Bool, "/lock/health", 10)
        self.create_subscription(BciStatus, "/bci/status", self.on_bci, 10)
        self.create_subscription(XrTiming, "/xr/timing", self.on_xr, 10)
        self.create_subscription(MarkerArray, "/markers", self.on_markers, LATCHED)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: setattr(self, "amcl_cov", m.pose.covariance[0] + m.pose.covariance[7]), LATCHED)
        self.create_subscription(Bool, "/mcu/alive", self.on_mcu, 10)
        self.create_subscription(String, "/mcu/status", lambda m: setattr(self, "mcu_status", m.data), 10)
        self.create_subscription(Bool, "/grip", lambda m: setattr(self, "grip_t", self.wall()), 10)
        self.state_clients = {n: self.create_client(GetState, f"/{n}/get_state") for n in NAV_NODES}
        self.create_timer(0.5, self.evaluate)
        self.create_timer(0.1, self.publish_lock)
        self.create_timer(1.0, self.poll_nav)
        self.get_logger().info(f"health_monitor: mode={self.mode} eeg={self.eeg} robot={self.robot} "
                               f"require_headset={self.require_headset}: motion LOCKED until all required gates are green")

    @staticmethod
    def wall() -> float:
        return time.monotonic()

    def on_bci(self, m):
        self.bci, self.bci_t = m, self.wall()

    def on_xr(self, m):
        self.xr, self.xr_t = m, self.wall()

    def on_markers(self, m):
        self.marker_hash, self.n_markers = m.table_hash, len(m.markers)

    def on_scan(self, m):
        now = self.wall()
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.scan_times = [(w, st) for w, st in self.scan_times if now - w < 3.0] + [(now, stamp)]
        self.scan_frame = m.header.frame_id

    def on_mcu(self, m):
        self.mcu_alive, self.mcu_t = m.data, self.wall()

    def poll_nav(self):
        for n, c in self.state_clients.items():
            if c.service_is_ready():
                c.call_async(GetState.Request()).add_done_callback(
                    lambda f, n=n: self.nav_states.__setitem__(n, f.result().current_state.label if f.result() else "?"))

    def evaluate(self):
        now = self.wall()
        g = self.g
        # Muse / LSL
        if self.bci is None or now - self.bci_t > 2.0:
            g["muse_lsl"].set(False, "no /bci/status (ssvep_node down?)")
        else:
            r = self.bci.sample_rate
            ok_rate = abs(r - 256.0) <= 5.0
            g["muse_lsl"].set(ok_rate and self.bci.signal_ok,
                              f"{r:.0f} Hz, {self.bci.quality}" + ("" if ok_rate else " (expected 256 +/- 5 Hz)"), r)
            if self.bci.source == "REAL":
                g["bci"].set(self.bci.profile_loaded, "calibration profile loaded" if self.bci.profile_loaded
                             else "REAL EEG without a per-user calibration profile: run the calibration")
            else:
                g["bci"].set(True, f"{self.bci.source} EEG (labeled)")
        # headset
        if self.xr is None or now - self.xr_t > 3.0:
            g["headset"].set(False, "headset app not connected")
        else:
            x = self.xr
            probs = []
            if abs(x.refresh_hz - self.refresh) > 1.0:
                probs.append(f"refresh {x.refresh_hz:.0f} Hz != configured {self.refresh:.0f} Hz")
            if x.dropped_pct >= 1.0:
                probs.append(f"dropped frames {x.dropped_pct:.1f} %")
            if x.table_hash != self.table_hash:
                probs.append("frequency table mismatch")
            if self.mode == "real" and (not x.calibrated or x.calibration_residual > 0.05):
                probs.append("headset not calibrated to the map (residual > 5 cm)")
            if self.n_markers < 1:
                probs.append("no markers placed")
            if not x.consent:
                probs.append("photosensitivity consent pending")
            g["headset"].set(not probs, "; ".join(probs) or f"{x.refresh_hz:.0f} Hz, {x.frame_ms_mean:.1f} ms/frame, {self.n_markers} markers", x.refresh_hz)
        # frequency table hash agreement (decoder <-> served table <-> headset)
        hashes = {"decoder": self.bci.table_hash if self.bci else None, "markers": self.marker_hash,
                  "headset": self.xr.table_hash if (self.xr and now - self.xr_t <= 3.0) else None}
        known = {k: v for k, v in hashes.items() if v}
        bad = [k for k, v in known.items() if v != self.table_hash]
        g["frequency_table"].set(bool(known) and not bad, f"hash {self.table_hash}" + (f" MISMATCH: {bad}" if bad else ""))
        # LiDAR
        # (wall arrival, header stamp): liveness by wall time (a dead LiDAR sends nothing, so prune here too);
        # the RATE from the stamps, i.e. on the robot's clock. In sim that is sim time: a sim running slower
        # than real time under CPU load delivers fewer scans per wall second, which is not a LiDAR fault.
        self.scan_times = [(w, st) for w, st in self.scan_times if now - w < 3.0]
        n = len(self.scan_times)
        span = self.scan_times[-1][1] - self.scan_times[0][1] if n > 2 else 0.0
        rate = (n - 1) / span if span > 0 else 0.0
        alive = n > 2 and now - self.scan_times[-1][0] < 0.5
        g["lidar"].set(alive and abs(rate - 10.0) <= 2.0 and self.scan_frame == "laser_frame",
                       f"/scan {rate:.1f} Hz in '{self.scan_frame}'" + ("" if alive or n <= 2 else " (silent)"), rate)
        # TF
        try:
            for a, b in (("map", "odom"), ("odom", "base_footprint"), ("base_footprint", "laser_frame")):
                self.tf.lookup_transform(a, b, rclpy.time.Time())
            g["tf"].set(True, "map -> odom -> base_footprint -> laser_frame")
        except Exception as e:
            g["tf"].set(False, f"TF incomplete: {str(e)[:80]}")
        # localization
        if self.slam:
            g["localization"].set(True, "SLAM (mapping mode)")
        elif self.amcl_cov is None:
            g["localization"].set(False, "no /amcl_pose yet")
        else:
            g["localization"].set(self.amcl_cov < self.cov_max, f"AMCL cov {self.amcl_cov:.3f}", self.amcl_cov)
        # Nav2 lifecycle
        inactive = [n for n in NAV_NODES if self.nav_states.get(n) != "active"]
        g["nav2"].set(not inactive, "all lifecycle nodes active" if not inactive else f"not active: {inactive}")
        # MCU
        if g["mcu"].required:
            fresh = now - self.mcu_t < 1.0
            g["mcu"].set(fresh and self.mcu_alive, f"link {'alive' if fresh and self.mcu_alive else 'LOST'}; {self.mcu_status}")
        else:
            g["mcu"].set(True, "no MCU in this backend (mujoco)")
        # safety
        cm_ok = self.nav_states.get("collision_monitor") == "active"
        dm_ok = self.robot != "walker" or now - self.grip_t < 1.0
        g["safety"].set(cm_ok and dm_ok, ("collision monitor active" if cm_ok else "collision monitor NOT active")
                        + ("" if self.robot != "walker" else ("; dead-man sensor live" if dm_ok else "; NO grip data (dead-man)")))
        # lock logic
        req_ok = all(x.level == ComponentHealth.OK for x in g.values() if x.required)
        if not self.unlocked and not req_ok:
            red = [f"{x.name}: {x.msg}" for x in g.values() if x.required and x.level != ComponentHealth.OK]
            self.get_logger().info(f"waiting for gates: {red}", throttle_duration_sec=5.0)
        if req_ok and not self.unlocked:
            self.unlocked = True
            self.get_logger().info("all required health gates GREEN: motion unlocked")
        for x in g.values():
            if x.level == ComponentHealth.OK:
                x.red_since = None
            elif x.red_since is None:
                x.red_since = now
        if self.unlocked:
            red = [x.name for x in g.values() if x.required and x.lock_on_red and x.red_since and now - x.red_since > 0.5]
            if red:
                self.unlocked = False
                self.get_logger().error(f"health gate(s) RED {red}: motion LOCKED")
        msg = SystemHealth(all_required_ok=req_ok, motion_unlocked=self.unlocked, mode=self.mode)
        msg.stamp = self.get_clock().now().to_msg()
        for x in g.values():
            msg.components.append(ComponentHealth(name=x.name, level=x.level, required=x.required, message=x.msg, value=x.value))
        self.pub.publish(msg)

    def publish_lock(self):
        self.pub_lock.publish(Bool(data=not self.unlocked))


def main():
    rclpy.init()
    n = HealthMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

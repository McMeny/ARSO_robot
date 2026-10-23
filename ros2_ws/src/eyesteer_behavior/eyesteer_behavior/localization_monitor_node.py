"""localization_monitor_node: is the robot where AMCL thinks it is? (Section 24.5) -> /localization/ok (latched).

Two checks, either one declares the robot LOST (mode_manager's FSM stops navigation and accepts no goals):
  * AMCL covariance: sum of the x/y variances > localization_lost_cov.
  * Scan vs map (kidnap): a teleported robot keeps a confident but wrong AMCL pose. A wrong pose shows as LiDAR
    rays that pass THROUGH walls the map says are there; people and unmapped objects only ever SHORTEN rays, so
    they cannot fake it. Recovery: the scan agrees again (e.g. after "2D Pose Estimate" / /initialpose).

Its own process: the pose search costs 35-140 ms of numpy per evaluation. Inside mode_manager's single-threaded
executor that delayed Nav2 results and the 100 Hz /clock by seconds (ARRIVED reported 3.7 s late).
Not launched while mapping (slam_toolbox owns the pose and the map is still being built).
"""
from __future__ import annotations

import collections
import math
import time

from eyesteer_behavior import _root  # noqa: F401

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty as EmptySrv
from tf2_ros import Buffer, TransformListener

from eyesteer_behavior.mode_manager_node import LATCHED, MapClearance


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__("localization_monitor")
        self.lost_cov = self.declare_parameter("localization_lost_cov", 1.0).value
        self.clear = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.hist = collections.deque()
        self.bad_since = None
        self.n = 0
        self.ok = True
        self.cov_bad = False
        # AMCL only updates while the robot moves: when the scan disagrees, ask it to refine in place
        self.amcl_nomotion = self.create_client(EmptySrv, "/request_nomotion_update")
        self.nomotion_t = 0.0
        self.pub = self.create_publisher(Bool, "/localization/ok", LATCHED)
        self.pub.publish(Bool(data=True))
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "clear", MapClearance(m)), LATCHED)
        self.create_subscription(LaserScan, "/scan", self.on_scan, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, LATCHED)
        self.get_logger().info("localization_monitor: scan-vs-map (see-through rays) + AMCL covariance -> /localization/ok")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def set_ok(self, ok: bool, msg: str):
        if ok == self.ok:
            return
        self.ok = ok
        if ok:      # separate call sites: rclpy rejects one call site logging at two severities
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)
        self.pub.publish(Bool(data=ok))

    def on_amcl(self, m: PoseWithCovarianceStamped):
        c = m.pose.covariance
        self.cov_bad = (c[0] + c[7]) > self.lost_cov
        if self.cov_bad:
            self.set_ok(False, f"LOCALIZATION LOST: AMCL covariance {c[0] + c[7]:.2f} > {self.lost_cov}")

    def on_scan(self, s: LaserScan):
        if self.clear is None:
            return
        self.n += 1
        if self.n % 8:                          # 1.25 Hz is plenty (2 s mean), and keeps the CPU cost low
            return
        try:
            # the pose AT the scan's stamp: with the latest TF, a turning robot smears the scan across the map
            t = self.tf.lookup_transform("map", s.header.frame_id, rclpy.time.Time.from_msg(s.header.stamp))
        except Exception:
            t = None
        try:
            t = t or self.tf.lookup_transform("map", s.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"localization check: no TF map->{s.header.frame_id}: {e}", throttle_duration_sec=10.0)
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        r = np.asarray(s.ranges, float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        rmax = min(s.range_max, 8.0)
        ret = np.isfinite(r) & (r > s.range_min) & (r < rmax)
        if ret.sum() < 20:
            return
        r, a = r[ret][::4], a[ret][::4]
        # Sample each ray up to 0.3 m short of its return; a sample inside an occupied cell = see-through ray.
        # AMCL in the sparse field drifts by up to ~0.7 m / 0.2 rad while driving and corrects itself, so the
        # score is the BEST over nearby poses: an imprecise AMCL pose has a nearby pose that explains the scan,
        # a kidnapped robot has none. Offline (25 random poses): AMCL error 0.7 m / 0.2 rad -> <= 6 %;
        # kidnap -> >= 13 %, median 25 %.
        d = np.arange(0.2, rmax, 0.1)
        before = d[None, :] < (r[:, None] - 0.3)
        x0, y0 = t.transform.translation.x, t.transform.translation.y

        def best(step_xy, n_xy, step_yaw, n_yaw):
            off = step_xy * np.arange(-n_xy, n_xy + 1)
            dx, dy = (v.ravel()[:, None, None] for v in np.meshgrid(off, off))
            b = 1.0
            for dyaw in step_yaw * np.arange(-n_yaw, n_yaw + 1):
                ca, sa = np.cos(a + yaw + dyaw)[:, None] * d, np.sin(a + yaw + dyaw)[:, None] * d
                hit = self.clear.occupied(x0 + dx + ca, y0 + dy + sa) & before     # (poses, rays, samples)
                b = min(b, float(hit.any(axis=2).mean(axis=1).min()))
            return b

        near = best(0.15, 2, 0.05, 2)                     # +-0.3 m, +-0.1 rad: the usual case, cheap
        if near > 0.08 and time.monotonic() - self.nomotion_t > 0.5 and self.amcl_nomotion.service_is_ready():
            self.nomotion_t = time.monotonic()            # AMCL only updates while moving: refine in place
            self.amcl_nomotion.call_async(EmptySrv.Request())
        through = near if near <= 0.08 else min(near, best(0.15, 4, 0.075, 4))   # +-0.6 m, +-0.3 rad
        now = self.now()
        # mean over the last 2 s: single scans are noisy
        self.hist.append((now, through))
        while self.hist and now - self.hist[0][0] > 2.0:
            self.hist.popleft()
        through = float(np.mean([v for _, v in self.hist]))
        self.get_logger().debug(f"localization check: {through * 100:.0f} % of the rays pass through mapped walls")
        if through > 0.10:
            self.bad_since = self.bad_since or now
            if now - self.bad_since > 2.0:
                self.set_ok(False, f"LOCALIZATION LOST: {through * 100:.0f} % of the LiDAR rays pass through mapped walls")
        else:
            self.bad_since = None
            if through < 0.05 and not self.cov_bad:
                self.set_ok(True, f"localization recovered ({through * 100:.0f} % see-through rays)")


def main():
    rclpy.init()
    n = LocalizationMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Runs one tentacle-planner scenario against a running sim, as the "user": it scripts the grip
(on /walker/grip_keys - launch with grip:=keyboard) and the head direction (/head_pose), measures
what the walker does, and prints PASS / FAIL for each expected behaviour.

  terminal 1:  ros2 launch my_robot_bringup walker_tentacle.launch.xml scenario:=corridor
  terminal 2:  ros2 run my_robot_autonomy run_scenario corridor

Scenarios: open, corridor, tjunction, registered, unregistered, cluttered, deadend, grip_release.
(registered / unregistered use the 'obstacle' world; launch unregistered with eeg_valid:=false.)
A log of every tick is written to ~/eye2steering_ws/sessions/scenarios/<name>.json.
"""
import json
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

from my_robot_autonomy.world_model import load_world
from walker_interfaces.msg import GripState, HazardArray, SteeringStatus

WORLD_OF = {'registered': 'obstacle', 'unregistered': 'obstacle', 'grip_release': 'open', 'head_turn': 'open', 'grip_check': 'open'}
BODY = (-0.04, 0.46, 0.25)        # walker body: x from, x to, half width (base_footprint)
USER = (-0.79, -0.04, 0.3)        # where the user stands behind it


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def perimeter(x0, x1, hw, step=0.05):
    pts = []
    n = int((x1 - x0) / step)
    for i in range(n + 1):
        x = x0 + (x1 - x0) * i / n
        pts += [(x, hw), (x, -hw)]
    m = int(2 * hw / step)
    for j in range(m + 1):
        y = -hw + 2 * hw * j / m
        pts += [(x0, y), (x1, y)]
    return pts


class Runner(Node):
    def __init__(self, world_file):
        super().__init__('scenario_runner')
        self.objects = [o for o in load_world(world_file) if o.name != 'ground_plane']
        self.body = perimeter(*BODY)
        self.user = perimeter(*USER)
        self.grip = (0.0, 0.0)
        self.head = None              # absolute head yaw (map), or None = no head data
        self.pose = None
        self.v = 0.0
        self.status = None
        self.hazards = []
        self.mux = self.out = None
        self.log = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.grip_pub = self.create_publisher(GripState, 'walker/grip_keys', 10)
        self.head_pub = self.create_publisher(PoseStamped, 'head_pose', 10)
        self.create_subscription(Odometry, 'odometry/filtered', lambda m: setattr(self, 'v', m.twist.twist.linear.x), 10)
        self.create_subscription(SteeringStatus, 'walker/steering_status', lambda m: setattr(self, 'status', m), 10)
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)
        self.create_subscription(Twist, 'cmd_vel_mux', lambda m: setattr(self, 'mux', m), 10)
        self.create_subscription(Twist, 'cmd_vel_out', lambda m: setattr(self, 'out', m), 10)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def gap(self, pts):
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        return min(o.distance_to(x + c * px - s * py, y + s * px + c * py) for o in self.objects for px, py in pts)

    def tick(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        q = t.transform.rotation
        self.pose = (t.transform.translation.x, t.transform.translation.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        g = GripState()
        g.header.stamp = self.get_clock().now().to_msg()
        g.left, g.right = self.grip
        self.grip_pub.publish(g)
        x, y, th = self.pose
        if self.head is not None:
            ps = PoseStamped()
            ps.header.stamp = g.header.stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x - 0.5 * math.cos(th), y - 0.5 * math.sin(th), 1.6
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(self.head / 2), math.cos(self.head / 2)
            self.head_pub.publish(ps)
        st = self.status
        cm = (self.mux is not None and self.out is not None and abs(self.mux.linear.x) > 0.03
              and abs(self.out.linear.x) < 0.8 * abs(self.mux.linear.x))
        self.log.append({
            't': round(self.now(), 2), 'x': round(x, 3), 'y': round(y, 3), 'yaw': round(th, 3), 'v': round(self.v, 3),
            'grip': st.grip_state if st else None, 'pivot': st.turning_in_place if st else False,
            'blocked': st.blocked if st else False, 'caution': st.caution if st else False,
            'notice': st.notice if st else '', 'hint': st.hint if st else '',
            'authority': round(st.authority, 2) if st else None, 'desired': round(st.desired_yaw, 3) if st else None,
            'body_gap': round(self.gap(self.body), 3), 'user_gap': round(self.gap(self.user), 3), 'cm': cm,
            'hazards': {h.name: [h.state, h.registered, h.eeg_valid] for h in self.hazards}})

    # ---------- scripting ----------

    def wait_ready(self):
        start = time.time()
        while (self.pose is None or self.status is None) and time.time() - start < 120:
            time.sleep(0.2)
        if self.pose is None or self.status is None:
            raise RuntimeError('sim not running (no pose / steering status)')

    def run(self, seconds, grip, head='keep', until=None):
        """Hold a grip (WALK / STEER / STOP or (left, right)) and head (absolute map yaw, None, or 'keep')."""
        self.grip = {'STOP': (0.0, 0.0), 'WALK': (0.4, 0.4), 'STEER': (0.9, 0.9)}.get(grip, grip)
        if head != 'keep':
            self.head = head
        first = len(self.log)
        end = self.now() + seconds
        while self.now() < end:
            time.sleep(0.05)
            if until is not None and len(self.log) > first and until(self.log[-1]):
                break
        return self.log[first:]


# ---------- checks ----------

class Report:
    def __init__(self, name):
        self.name = name
        self.checks = []

    def check(self, label, ok, detail):
        self.checks.append({'check': label, 'pass': bool(ok), 'detail': detail})
        print(f'  [{"PASS" if ok else "FAIL"}] {label}: {detail}')


def common(rep, log):
    contacts = sum(1 for r in log if r['body_gap'] < 0.01)
    rep.check('no collisions', contacts == 0, f'min body gap {min(r["body_gap"] for r in log) * 100:.1f} cm, '
              f'min user gap {min(r["user_gap"] for r in log) * 100:.1f} cm, contact ticks {contacts}')
    # it must cut the command for 3 ticks in a row (one tick is just the output lagging while accelerating)
    cm = sum(1 for a, b, c in zip(log, log[1:], log[2:]) if a['cm'] and b['cm'] and c['cm'])
    rep.check('collision monitor never needed', cm == 0, f'{cm} ticks where it cut the command')


def heading_change(seg):
    return math.degrees(wrap(seg[-1]['yaw'] - seg[0]['yaw']))


def scenario(r, name, rep):
    yaw0 = r.pose[2]
    if name == 'open':
        a = r.run(8, 'WALK', None)
        drift = max(abs(row['y']) for row in a)
        rep.check('WALK goes straight', drift < 0.05 and abs(heading_change(a)) < 3,
                  f'{a[-1]["x"] - a[0]["x"]:.2f} m, sideways drift {drift * 100:.1f} cm, heading {heading_change(a):+.1f} deg')
        b = r.run(10, 'STEER', yaw0 + math.radians(45))
        rep.check('STEER + head 45 deg curves to it', abs(math.degrees(wrap(b[-1]['yaw'] - yaw0)) - 45) < 8,
                  f'heading {math.degrees(wrap(b[-1]["yaw"] - yaw0)):+.1f} deg')
        target = wrap(b[-1]['yaw'] + math.pi)
        c = r.run(15, 'STEER', target, until=lambda row: abs(wrap(row['yaw'] - target)) < math.radians(8) and not row['pivot'])
        moved = math.hypot(c[-1]['x'] - c[0]['x'], c[-1]['y'] - c[0]['y'])
        rep.check('STEER + head 180 deg pivots in place', any(row['pivot'] for row in c)
                  and abs(math.degrees(wrap(c[-1]['yaw'] - target))) < 12,
                  f'pivot seen {any(row["pivot"] for row in c)}, final error {math.degrees(wrap(c[-1]["yaw"] - target)):+.1f} deg, '
                  f'moved {moved:.2f} m while turning')
        r.run(2, 'STOP')
        return a + b + c
    if name == 'corridor':
        a = r.run(18, 'WALK', None)
        steady = a[len(a) // 4:]
        drift = max(abs(row['y']) for row in steady)
        yaw_sd = (sum(row['yaw'] ** 2 for row in steady) / len(steady)) ** 0.5
        rep.check('centred and steady', drift < 0.12 and math.degrees(yaw_sd) < 3,
                  f'{a[-1]["x"] - a[0]["x"]:.2f} m, max off-centre {drift * 100:.1f} cm, heading wobble (rms) {math.degrees(yaw_sd):.1f} deg')
        return a
    if name == 'tjunction':
        a = r.run(25, 'WALK', None, until=lambda row: row['blocked'] and row['v'] < 0.01)
        rep.check('WALK does not turn on its own at the T', a[-1]['blocked'] and abs(heading_change(a)) < 30,
                  f'stopped at x {a[-1]["x"]:.2f}, blocked {a[-1]["blocked"]}, heading {heading_change(a):+.1f} deg')
        b = r.run(20, 'STEER', yaw0 + math.pi / 2, until=lambda row: row['y'] > 1.6)
        rep.check('STEER turns toward the head (left branch)', b[-1]['y'] > 1.0 and abs(math.degrees(wrap(b[-1]['yaw'] - yaw0)) - 90) < 25,
                  f'reached y {b[-1]["y"]:.2f}, heading {math.degrees(wrap(b[-1]["yaw"] - yaw0)):+.1f} deg')
        r.run(2, 'STOP')
        return a + b
    if name in ('registered', 'unregistered'):
        a = r.run(22, 'WALK', None, until=lambda row: row['x'] > 6.0)
        near = [row for row in a if 1.5 < row['x'] < 4.0]
        v_near = min(row['v'] for row in near) if near else float('nan')
        caution = any(row['caution'] for row in a)
        names = list(a[-1]['hazards'])
        passing = [row['body_gap'] for row in a if 2.8 < row['x'] < 4.2]
        pass_gap = min(passing) if passing else float('nan')
        rep.check('avoided the obstacle', a[-1]['x'] > 5.0, f'reached x {a[-1]["x"]:.2f}, gap passing it {pass_gap * 100:.1f} cm')
        if name == 'registered':
            rep.check('normal speed (no caution)', not caution and v_near > 0.3,
                      f'caution {caution}, slowest near it {v_near:.2f} m/s, hazard {names}: {a[-1]["hazards"]}')
        else:
            rep.check('slowed to caution_scale', caution and v_near < 0.25,
                      f'caution {caution}, slowest near it {v_near:.2f} m/s, hazard {a[-1]["hazards"]}')
        rep.metrics = {'pass_gap': pass_gap, 'v_near': v_near}
        return a
    if name == 'cluttered':
        a = r.run(80, 'WALK', None, until=lambda row: row['x'] > 13.6)
        blocked = sum(1 for row in a if row['blocked']) * 0.05
        rep.check('gets through the clutter', a[-1]['x'] > 13.5, f'reached x {a[-1]["x"]:.2f} in {a[-1]["t"] - a[0]["t"]:.0f} s, '
                  f'blocked {blocked:.1f} s')
        return a
    if name == 'deadend':
        a = r.run(20, 'WALK', None, until=lambda row: row['notice'])
        blocked_at = next((row['t'] for row in a if row['blocked']), None)
        notice_at = next((row['t'] for row in a if row['notice']), None)
        rep.check('stops with blocked = true', blocked_at is not None and min(abs(row['v']) for row in a[-10:]) < 0.02,
                  f'stopped at x {a[-1]["x"]:.2f}, front {3.925 - 0.46 - a[-1]["x"]:.2f} m from the wall')
        rep.check('notice ~1 s after blocked', notice_at is not None and 0.9 <= notice_at - blocked_at <= 1.6,
                  f'"{a[-1]["notice"]}" after {(notice_at or 0) - (blocked_at or 0):.2f} s')
        r.run(1, 'WALK')
        target = wrap(yaw0 + math.pi)
        b = r.run(25, 'STEER', target, until=lambda row: abs(wrap(row['yaw'] - target)) < math.radians(8) and not row['pivot'])
        first_steer = next((i for i, row in enumerate(b) if row['grip'] == 2), None)
        cleared = first_steer is not None and not any(row['notice'] for row in b[first_steer:])
        rep.check('notice clears as soon as STEER starts', cleared,
                  f'STEER after {(first_steer or 0) * 0.05:.2f} s (0.5 s hold), notice cleared {cleared}')
        rep.check('STEER + turned head pivots it out', any(row['pivot'] for row in b) and abs(math.degrees(wrap(b[-1]['yaw'] - target))) < 12,
                  f'pivot {any(row["pivot"] for row in b)}, heading error {math.degrees(wrap(b[-1]["yaw"] - target)):+.1f} deg')
        c = r.run(6, 'WALK')
        rep.check('walks back out', c[-1]['x'] < c[0]['x'] - 0.8, f'x {c[0]["x"]:.2f} -> {c[-1]["x"]:.2f}')
        r.run(2, 'STOP')
        return a + b + c
    if name == 'grip_release':
        a = r.run(6, 'WALK', None)
        v0 = a[-1]['v']
        b = r.run(4, 'STOP')
        stop_i = next((i for i, row in enumerate(b) if abs(row['v']) < 0.01), None)
        dist = math.hypot(b[stop_i or -1]['x'] - b[0]['x'], b[stop_i or -1]['y'] - b[0]['y'])
        held = math.hypot(b[-1]['x'] - b[stop_i or -1]['x'], b[-1]['y'] - b[stop_i or -1]['y'])
        limit = v0 ** 2 / (2 * 0.5) + v0 * 0.35       # decel 0.5 m/s^2, plus grace + latency
        rep.check('stops within the deceleration limit, and holds', stop_i is not None and dist <= limit and held < 0.02,
                  f'from {v0:.2f} m/s: stopped in {(stop_i or 0) * 0.05:.2f} s, {dist:.2f} m (limit {limit:.2f}), drift after {held * 100:.1f} cm')
        r.run(5, 'WALK')
        c = r.run(3, (0.0, 0.4))
        stop_i = next((i for i, row in enumerate(c) if abs(row['v']) < 0.01), None)
        rep.check('one hand off counts as STOP', stop_i is not None and c[-1]['grip'] == 0,
                  f'grip {c[-1]["grip"]}, stopped after {(stop_i or 0) * 0.05:.2f} s')
        return a + b + c
    raise ValueError(f'unknown scenario {name}')


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'open'
    world = WORLD_OF.get(name, name)
    world_file = os.path.join(get_package_share_directory('my_robot_bringup'), 'worlds', 'scenarios', f'{world}.sdf')
    rclpy.init()
    r = Runner(world_file)
    ex = MultiThreadedExecutor()
    ex.add_node(r)
    threading.Thread(target=ex.spin, daemon=True).start()
    rep = Report(name)
    print(f'Scenario {name} (world {world})')
    try:
        r.wait_ready()
        r.run(2, 'STOP', None)
        log = scenario(r, name, rep)
        common(rep, log)
    finally:
        out = os.path.expanduser('~/eye2steering_ws/sessions/scenarios')
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, f'{name}.json'), 'w') as f:
            json.dump({'scenario': name, 'checks': rep.checks, 'metrics': getattr(rep, 'metrics', {}), 'log': r.log}, f)
        ok = all(c['pass'] for c in rep.checks)
        print(f'{name}: {"ALL PASS" if ok and rep.checks else "FAILED"} ({sum(c["pass"] for c in rep.checks)}/{len(rep.checks)})')
        ex.shutdown()
        r.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

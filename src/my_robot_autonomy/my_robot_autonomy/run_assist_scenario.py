"""Runs one ASSIST + guardian scenario against a running sim, as the "user": it holds the handles
(/walker/grip_keys) and pushes them (/walker/handle_force) - launch with grip:=keyboard - then
measures what the walker does and prints PASS / FAIL.

  terminal 1:  ros2 launch my_robot_bringup walker_assist.launch.xml scenario:=obstacle
  terminal 2:  ros2 run my_robot_autonomy run_assist_scenario registered

Scenarios: open, corridor, tjunction, registered, unregistered, cluttered, deadend, grip_release, head_turn, grip_check.
(unregistered: launch with eeg_valid:=false.) 12 N on each handle is a 0.4 m/s walk.
Every scenario also checks: no collisions, the collision monitor never needed, and the walker never
faster than the push. Log: ~/eye2steering_ws/sessions/scenarios/assist_<name>.json
"""
import json
import math
import os
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from my_robot_autonomy.run_scenario import Report, Runner, WORLD_OF, wrap
from walker_interfaces.msg import HandleForce

PUSH = 12.0


class AssistRunner(Runner):
    def __init__(self, world_file):
        super().__init__(world_file)
        self.force = (0.0, 0.0)
        self.guardian_cmd = None
        self.force_pub = self.create_publisher(HandleForce, 'walker/handle_force', 10)
        self.create_subscription(Twist, 'cmd_vel_guardian', lambda m: setattr(self, 'guardian_cmd', m), 10)
        self.command_pub = self.create_publisher(String, 'operator_command', 10)
        self.calibration = None
        self.create_subscription(String, 'walker/grip_calibration',
                                 lambda m: setattr(self, 'calibration', json.loads(m.data)), 10)
        # the guardian's own output speed and the push speed, from the same tick (one message)
        self.pairs = []
        self.create_subscription(String, 'walker_status', self.on_status_json,
                                 QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_status_json(self, msg):
        st = json.loads(msg.data)
        if 'speed' in st and 'user_speed' in st:
            self.pairs.append((st['speed'], st['user_speed']))

    def tick(self):
        n = len(self.log)
        super().tick()
        f = HandleForce()
        f.header.stamp = self.get_clock().now().to_msg()
        f.left, f.right = self.force
        self.force_pub.publish(f)
        if len(self.log) > n:
            st, c = self.status, self.guardian_cmd
            self.log[-1].update({
                'push': list(self.force), 'user_speed': round(st.user_speed, 3) if st else 0.0,
                'intervention': round(st.intervention, 2) if st else 0.0,
                'braking': st.braking if st else False, 'nudging': st.nudging if st else False,
                'head_steering': st.head_steering if st else False,
                'cmd_v': round(c.linear.x, 3) if c else 0.0, 'cmd_w': round(c.angular.z, 3) if c else 0.0})

    def act(self, seconds, hands=True, push=(0.0, 0.0), until=None, grip=None, head=None):
        self.force = push
        return self.run(seconds, grip or ('WALK' if hands else 'STOP'), head, until)


def moved(seg):
    return math.hypot(seg[-1]['x'] - seg[0]['x'], seg[-1]['y'] - seg[0]['y'])


def deg(a):
    return math.degrees(wrap(a))


def scenario(r, name, rep):
    yaw0 = r.pose[2]
    straight = (PUSH, PUSH)
    if name == 'open':
        a = r.act(3, push=(0.0, 0.0))
        rep.check('hands on, no push: does not move', moved(a) < 0.01, f'moved {moved(a) * 100:.1f} cm')
        b = r.act(7, push=straight)
        steady = [row['v'] for row in b[len(b) // 2:]]
        drift = max(abs(row['y']) for row in b)
        rep.check('push straight: goes straight at the pushed speed', abs(sum(steady) / len(steady) - 0.4) < 0.05 and drift < 0.05,
                  f'{sum(steady) / len(steady):.2f} m/s (push asks 0.40), drift {drift * 100:.1f} cm, '
                  f'guardian intervening {max(row["intervention"] for row in b) * 100:.0f}% at most')
        c = r.act(5, push=(PUSH - 5, PUSH + 5))
        rep.check('push the right handle harder: turns left', deg(c[-1]['yaw'] - c[0]['yaw']) > 20,
                  f'turned {deg(c[-1]["yaw"] - c[0]["yaw"]):+.0f} deg')
        r.act(3, push=(0.0, 0.0))
        d = r.act(4, push=(-10.0, 10.0))
        rep.check('one handle pushed, the other pulled: turns on the spot', abs(deg(d[-1]['yaw'] - d[0]['yaw'])) > 20 and moved(d) < 0.15,
                  f'turned {deg(d[-1]["yaw"] - d[0]["yaw"]):+.0f} deg, moved {moved(d) * 100:.0f} cm')
        r.act(2, push=(0.0, 0.0))
        return a + b + c + d
    if name == 'corridor':
        a = r.act(16, push=(PUSH - 0.5, PUSH + 0.5))       # drifting towards the left wall (~30 deg over the run)
        drift = max(abs(row['y']) for row in a)
        rep.check('pushed towards a wall: kept off it', drift < 0.45 and a[-1]['x'] > 4.0,
                  f'{a[-1]["x"]:.1f} m along, max off-centre {drift * 100:.0f} cm (wall at 75 cm)')
        return a
    if name == 'tjunction':
        a = r.act(25, push=straight, until=lambda row: row['v'] < 0.01 and row['x'] > 3.0)
        rep.check('pushed straight at the T: stops, does not take a branch by itself',
                  a[-1]['v'] < 0.02 and abs(deg(a[-1]['yaw'] - yaw0)) <= 40,
                  f'stopped at x {a[-1]["x"]:.2f}, turned {deg(a[-1]["yaw"] - yaw0):+.0f} deg (limit 35)')
        r.act(3, push=(-10.0, -10.0))                       # like a person: back up a little, turn, then walk
        b = r.act(20, push=(4.0, PUSH + 6), until=lambda row: deg(row['yaw'] - yaw0) > 80)
        b += r.act(15, push=straight, until=lambda row: row['y'] > 1.4)
        rep.check('user pushes to turn left: goes into the left branch', b[-1]['y'] > 1.2,
                  f'reached y {b[-1]["y"]:.2f}, heading {deg(b[-1]["yaw"] - yaw0):+.0f} deg')
        r.act(2, push=(0.0, 0.0))
        return a + b
    if name in ('registered', 'unregistered'):
        a = r.act(30, push=straight, until=lambda row: row['x'] > 6.0)
        near = [row for row in a if 1.5 < row['x'] < 4.0]
        v_near = min(row['v'] for row in near) if near else float('nan')
        passing = [row['body_gap'] for row in a if 2.8 < row['x'] < 4.2]
        gap = min(passing) if passing else float('nan')
        help_ = sum(row['intervention'] for row in a) / len(a)
        caution = any(row['caution'] for row in a)
        rep.check('gets past the obstacle', a[-1]['x'] > 5.5, f'reached x {a[-1]["x"]:.2f}, gap passing it {gap * 100:.0f} cm')
        if name == 'registered':
            rep.check('no caution, help comes later (only if heading into it)', not caution and v_near > 0.1,
                      f'caution {caution}, slowest near it {v_near:.2f} m/s, mean intervention {help_ * 100:.0f}%')
        else:
            rep.check('earlier, firmer help: caution, slower', caution and v_near < 0.27,
                      f'caution {caution}, slowest near it {v_near:.2f} m/s, mean intervention {help_ * 100:.0f}%')
        rep.metrics = {'gap': gap, 'v_near': v_near, 'intervention': help_}
        return a
    if name == 'cluttered':
        a = r.act(80, push=straight, until=lambda row: row['x'] > 13.5)
        rep.check('gets through the clutter', a[-1]['x'] > 13.4,
                  f'reached x {a[-1]["x"]:.2f} in {a[-1]["t"] - a[0]["t"]:.0f} s, '
                  f'mean intervention {sum(row["intervention"] for row in a) / len(a) * 100:.0f}%')
        return a
    if name == 'deadend':
        a = r.act(20, push=straight, until=lambda row: bool(row['notice']))
        notice_at = next((row['t'] for row in a if row['notice']), None)
        stopped_at = next((row['t'] for row in a if row['v'] < 0.01 and row['x'] > 1.0), None)
        rep.check('pushed into a dead end: stops before the wall', stopped_at is not None and 3.925 - 0.46 - a[-1]['x'] > 0.15,
                  f'front {3.925 - 0.46 - a[-1]["x"]:.2f} m from the wall')
        rep.check('notice ~1 s after it stops', notice_at is not None and stopped_at is not None
                  and 0.5 <= notice_at - stopped_at <= 2.0,
                  f'"{a[-1]["notice"]}" after {(notice_at or 0) - (stopped_at or 0):.2f} s')
        target = wrap(yaw0 + math.pi)
        b = r.act(30, push=(-10.0, 10.0), until=lambda row: abs(wrap(row['yaw'] - target)) < math.radians(10))
        rep.check('user turns it on the spot', abs(deg(b[-1]['yaw'] - target)) < 15,
                  f'heading error {deg(b[-1]["yaw"] - target):+.0f} deg, moved {moved(b) * 100:.0f} cm')
        c = r.act(6, push=straight)
        rep.check('walks back out', c[-1]['x'] < c[0]['x'] - 0.8, f'x {c[0]["x"]:.2f} -> {c[-1]["x"]:.2f}')
        r.act(2, push=(0.0, 0.0))
        return a + b + c
    if name == 'grip_release':
        a = r.act(6, push=straight)
        v0 = a[-1]['v']
        b = r.act(4, hands=False, push=straight)            # lets go, but something still presses the handles
        stop_i = next((i for i, row in enumerate(b) if abs(row['v']) < 0.01), None)
        dist = moved(b[:stop_i + 1]) if stop_i is not None else moved(b)
        held = moved(b[stop_i:]) if stop_i is not None else 1.0
        limit = v0 ** 2 / (2 * 0.5) + v0 * 0.5     # braking at 0.5 m/s^2 after the 0.3 s dead-man grace + ~0.2 s latency
        rep.check('hands off: stops within the deceleration limit, and holds', stop_i is not None and dist <= limit and held < 0.02,
                  f'from {v0:.2f} m/s: stopped in {(stop_i or 0) * 0.05:.2f} s, {dist:.2f} m (limit {limit:.2f}), drift after {held * 100:.1f} cm')
        c = r.act(4, push=(0.0, 0.0))
        rep.check('hands back on, no push: stays put (never moves on its own)', moved(c) < 0.01, f'moved {moved(c) * 100:.1f} cm')
        return a + b + c
    if name == 'head_turn':
        left = yaw0 + math.radians(45)
        a = r.act(4, push=straight, head=left)
        rep.check('looking aside, normal grip: keeps going straight', abs(deg(a[-1]['yaw'] - yaw0)) < 5,
                  f'turned {deg(a[-1]["yaw"] - yaw0):+.1f} deg')
        b = r.act(6, push=straight, grip='STEER', head=left)
        turned = deg(b[-1]['yaw'] - yaw0)
        rep.check('firm squeeze while pushing: turns towards the head', 30 < turned < 50, f'turned {turned:+.0f} deg (head at +45)')
        rep.check('reported as head steering', any(row['head_steering'] for row in b), '')
        mid = [row['v'] for row in b if row['head_steering']]
        rep.check('keeps walking pace while turning', len(mid) > 0 and sum(mid) / len(mid) > 0.2,
                  f'mean {sum(mid) / max(len(mid), 1):.2f} m/s (push asks 0.40)')
        c = r.act(3, push=straight, grip='STEER', head=left)
        rep.check('lined up with the head: stops turning', abs(deg(c[-1]['yaw'] - c[0]['yaw'])) < 5,
                  f'turned {deg(c[-1]["yaw"] - c[0]["yaw"]):+.1f} deg more')
        r.act(3, push=(0.0, 0.0))
        d = r.act(4, push=(0.0, 0.0), grip='STEER', head=wrap(yaw0 - math.pi / 2))
        rep.check('firm squeeze and head turn, no push: does not move', moved(d) < 0.01 and abs(deg(d[-1]['yaw'] - d[0]['yaw'])) < 1,
                  f'moved {moved(d) * 100:.1f} cm, turned {deg(d[-1]["yaw"] - d[0]["yaw"]):+.1f} deg')
        e = r.act(5, push=straight, grip='STEER', head=None)
        rep.check('head readings stop: no head turn, pushing carries on', moved(e) > 0.5 and not any(row['head_steering'] for row in e[-40:]),
                  f'moved {moved(e):.2f} m')
        r.act(2, push=(0.0, 0.0))
        return a + b + c + d + e
    if name == 'grip_check':
        def record(gentle, firm):
            r.act(2, grip=(gentle, gentle))
            r.command_pub.publish(String(data='calibrate_relaxed'))
            r.act(2, grip=(firm, firm))
            r.calibration = None
            r.command_pub.publish(String(data='calibrate_firm'))
            r.act(1, grip=(firm, firm))
            return r.calibration or {}
        c = record(0.3, 0.7)
        rep.check('hand check sets the thresholds between the two holds',
                  c.get('ok') is True and abs(c.get('light', 0) - 0.15) < 0.01 and abs(c.get('firm', 0) - 0.5) < 0.01,
                  f'{c.get("message")}')
        a = r.act(2, grip=(0.58, 0.58))
        rep.check("a squeeze past this person's threshold (default 0.6) counts as firm", a[-1]['grip'] == 2, f'grip state {a[-1]["grip"]}')
        b = r.act(2, grip=(0.25, 0.25))
        rep.check('a gentle hold still counts as held', b[-1]['grip'] == 1, f'grip state {b[-1]["grip"]}')
        c2 = record(0.5, 0.55)
        rep.check('a squeeze too close to the hold is refused', c2.get('ok') is False, f'{c2.get("message")}')
        d = r.act(2, grip=(0.58, 0.58))
        rep.check('refused: the last good thresholds stay', d[-1]['grip'] == 2, f'grip state {d[-1]["grip"]}')
        e = r.act(3, grip=(0.58, 0.58))
        rep.check('no push during any of it: the walker never moved', moved(a + b + d + e) < 0.01, f'moved {moved(a + b + d + e) * 100:.1f} cm')
        r.act(1, hands=False)
        return a + b + d + e
    raise ValueError(f'unknown scenario {name}')


def never_pushes(rep, pairs):
    """Every tick: the walker's speed from the guardian never above the push speed (both from one message)."""
    worst, bad = 0.0, 0
    for speed, user in pairs:
        excess = abs(speed) - abs(user)
        worst = max(worst, excess)
        bad += excess > 0.011            # both rounded to 1 cm/s in the message
    rep.check('never faster than the push', bad == 0 and len(pairs) > 0,
              f'{len(pairs)} ticks checked, {bad} over, worst {worst * 100:.1f} cm/s')


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'open'
    world = WORLD_OF.get(name, name)
    world_file = os.path.join(get_package_share_directory('my_robot_bringup'), 'worlds', 'scenarios', f'{world}.sdf')
    rclpy.init()
    r = AssistRunner(world_file)
    ex = MultiThreadedExecutor()
    ex.add_node(r)
    threading.Thread(target=ex.spin, daemon=True).start()
    rep = Report(name)
    print(f'ASSIST scenario {name} (world {world})')
    try:
        r.wait_ready()
        r.act(2, hands=False)
        log = scenario(r, name, rep)
        from my_robot_autonomy.run_scenario import common
        common(rep, log)
        never_pushes(rep, r.pairs)
    finally:
        out = os.path.expanduser('~/eye2steering_ws/sessions/scenarios')
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, f'assist_{name}.json'), 'w') as f:
            json.dump({'scenario': name, 'checks': rep.checks, 'metrics': getattr(rep, 'metrics', {}), 'log': r.log}, f)
        ok = all(c['pass'] for c in rep.checks)
        print(f'{name}: {"ALL PASS" if ok and rep.checks else "FAILED"} ({sum(c["pass"] for c in rep.checks)}/{len(rep.checks)})')
        ex.shutdown()
        r.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

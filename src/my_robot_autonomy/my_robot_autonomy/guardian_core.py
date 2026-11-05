"""ASSIST + guardian: the user moves the walker, the walker protects them. No ROS here, so it can
be unit tested.

  admittance  handle forces -> the user's intended (v, w), like pushing a heavy, well-damped cart.
              Ported from main's eyesteer_core/control/admittance.py (same model and defaults).
  dead-man    let go for longer than release_timeout -> locked at zero. Also ported from main.
  guardian    checks the user's own path with the tentacle machinery (swept walker+user footprint)
              and, only when needed, brakes and bends it - by holding a wheel back, never by
              driving one faster. Rules:
                * it never moves on its own: no push, no motion
                * each wheel is never faster (forward or backward) than the user's push makes it
                * so it can only slow the walker, or steer it by braking one side more
              The EEG sets how early and how firmly it steps in: a hazard the user registered gets a
              late, light correction; a missed one (or no reliable EEG) gets braking sooner, a firmer
              correction, a wider berth and a lower speed cap.
  head turn   a firm squeeze while pushing turns the walker towards where the head points, by holding the
              inner wheel back, so the wheels rule still holds. The result goes through the guardian
              like any other push. A head turn alone never moves the walker.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from my_robot_autonomy.tentacle_core import (
    Footprint, TentaclePlanner, Params, HazardLite, berth_cost, footprint_distances, rotation_poses,
    shape_poses, sweep, sweep_free, wrap)


# ---------- from main ----------

def deadband(f, db):
    if abs(f) <= db:
        return 0.0
    return f - math.copysign(db, f)


class Admittance:
    """Handle forces (N, + = pushing forward) -> velocity. Pushing the right handle harder turns
    left, like a rollator. Ported from main (eyesteer_core/control/admittance.py)."""

    def __init__(self, mass=25.0, damping=40.0, yaw_inertia=4.0, yaw_damping=8.0, force_deadband=4.0,
                 handle_y=0.25, v_max=0.5, w_max=0.8):
        self.m, self.b = mass, damping
        self.J, self.bw = yaw_inertia, yaw_damping
        self.db = force_deadband
        self.hy = handle_y
        self.v_max, self.w_max = v_max, w_max
        self.v = 0.0
        self.w = 0.0

    def reset(self):
        self.v = self.w = 0.0

    def step(self, f_left, f_right, dt):
        fl, fr = deadband(f_left, self.db), deadband(f_right, self.db)
        F = fl + fr
        tau = self.hy * (fr - fl)
        self.v += dt * (F - self.b * self.v) / self.m
        self.w += dt * (tau - self.bw * self.w) / self.J
        self.v = max(-self.v_max * 0.5, min(self.v_max, self.v))   # walking backwards is slower
        self.w = max(-self.w_max, min(self.w_max, self.w))
        return self.v, self.w

    def follow(self, v):
        """The walker was held back: the cart's speed is what it actually does (no wind-up). The turn
        rate is NOT copied back - it stays the user's own, so a guardian correction never becomes
        "what the user wanted"."""
        if abs(v) < abs(self.v):
            self.v = v


class DeadMan:
    """Walker grip: released for longer than `release_timeout` -> lock (zero). Ported from main."""

    def __init__(self, release_timeout=0.3):
        self.timeout = release_timeout
        self.released_since = None
        self.locked = True

    def update(self, gripped, t):
        if gripped:
            self.released_since = None
            self.locked = False
        else:
            if self.released_since is None:
                self.released_since = t
            self.locked = (t - self.released_since) >= self.timeout
        return self.locked


# ---------- the guardian ----------

def wheel_speeds(v, w, half_track):
    return v - w * half_track, v + w * half_track


def within_push(v, w, v_u, w_u, half_track, eps=1e-6):
    """True if neither wheel is faster than the user's push makes it (same direction, no more)."""
    for out, user in zip(wheel_speeds(v, w, half_track), wheel_speeds(v_u, w_u, half_track)):
        if abs(out) > abs(user) + eps or (abs(out) > eps and out * user < 0):
            return False
    return True


def limit_to_push(k, v_u, w_u, half_track):
    """Fastest (v, w) along curvature k with neither wheel faster than the user's push - so the
    walker can steer only by holding one side back. Returns (0, 0) if k needs a wheel to go the other way."""
    ul, ur = wheel_speeds(v_u, w_u, half_track)
    if v_u < 0:   # going backwards: mirror
        v, w = limit_to_push(-k, -v_u, w_u, half_track)
        return -v, w
    dl, dr = 1.0 - k * half_track, 1.0 + k * half_track     # wheel speed per unit v along k
    v = math.inf
    for d, u in ((dl, ul), (dr, ur)):
        if d > 1e-9:
            v = min(v, max(u, 0.0) / d)
        elif d < -1e-9:
            v = 0.0           # this wheel would have to run backwards
    v = 0.0 if not math.isfinite(v) else v
    return v, v * k


def head_steer(v_u, w_u, error, p):
    """The push (v_u, w_u) turned towards the head, `error` rad to the left of the walker's heading.
    Returns (v, w, steering). Forward pushes only, and neither wheel ends up faster than the push."""
    dead_zone = math.radians(p.head_dead_zone_deg)
    if v_u < p.min_push_speed or abs(error) <= dead_zone:
        return v_u, w_u, False
    turn = math.copysign(p.head_gain * (abs(error) - dead_zone), error)
    k = max(-p.head_max_curvature, min(p.head_max_curvature, (w_u + turn) / v_u))
    v, w = limit_to_push(k, v_u, w_u, p.half_track)
    if v < p.min_push_speed:      # the push is turning too hard the other way to bend it like this
        return v_u, w_u, False
    return v, w, True


@dataclass
class GuardParams:
    half_track: float = 0.22        # m, wheel to centre
    horizon_s: float = 5.0          # the user's path is checked this far ahead (at their speed)...
    L_min: float = 2.5              # ...but at least this far (m): bending round something has to start early
    L_max: float = 3.0
    brake_registered: float = 0.3   # m/s^2 the guardian plans to brake at - registered / nothing near
    brake_unregistered: float = 0.15  # a missed hazard: plans for gentler braking, so it starts sooner
    stop_margin: float = 0.42       # m short of the safety margin where it aims to stop: front ~0.55 m from a wall,
                                    # outside the collision monitor's slowdown zone and with room to turn on the spot
    comfort_registered: float = 2.3   # m of clear path below which it starts bending the path
    comfort_unregistered: float = 2.6
    nudge_registered: float = 1.0     # how much of the way to the safer path it bends (0..1)
    nudge_unregistered: float = 1.0
    min_commit: float = 0.5           # once it bends, at least this share of the way
    w_dev: float = 0.3              # alternatives: how much straying from the user's curvature costs
    w_free: float = 2.0             # ...and a shorter clear path
    w_hazard: float = 1.0           # ...and passing a hazard inside its berth
    berth_registered: float = 0.15
    berth_unregistered: float = 0.45
    caution_scale: float = 0.5      # speed cap (x v_max) with a missed / no-EEG hazard close ahead
    caution_radius: float = 2.0
    v_max: float = 0.5
    min_push_speed: float = 0.02    # m/s - below this the user isn't moving it
    spot_turn_curvature: float = 2.0  # 1/m - a push turning tighter than this counts as turning on the spot
    better_by: float = 0.4          # m more room an alternative needs to be worth bending the user's path to
    max_deviation_deg: float = 35.0  # the guardian never turns the walker further than this from the user's push
    return_gain: float = 0.6        # once clear, it undoes its own correction (1/m per rad of deviation)
    head_gain: float = 1.0          # head turn: rad/s of turn per rad the head is past the dead zone
    head_dead_zone_deg: float = 10.0  # the head this close to the walker's heading doesn't turn it
    head_max_curvature: float = 1.8  # 1/m - below spot_turn_curvature, so it stays a curve, never a pivot


@dataclass
class Decision:
    v: float = 0.0
    w: float = 0.0
    v_user: float = 0.0
    w_user: float = 0.0
    k_user: float = 0.0
    k_out: float = 0.0
    free: float = 0.0              # clear length along the user's own path (m)
    intervention: float = 0.0      # 0 = the user's push passes untouched, 1 = fully overridden (stopped)
    braking: bool = False
    nudging: bool = False
    blocked: bool = False          # the user is pushing but nothing in that direction is clear
    caution: bool = False
    turning_in_place: bool = False
    near_unregistered: bool = False
    head_steering: bool = False    # the head is turning the walker (firm squeeze, while pushing)


class Guardian:
    def __init__(self, params=None, footprint=None, tentacle_params=None):
        self.p = params or GuardParams()
        self.deviation = 0.0     # rad the guardian has turned the walker away from where the push alone would have
        self.fp = footprint or Footprint(margin=0.12)
        # the tentacle fan (arcs + lane changes) for finding a safer path near the user's
        self.fan = TentaclePlanner(tentacle_params or Params(), self.fp)

    def path(self, k, length, reverse=False):
        poses = shape_poses(k, 'arc', length, self.fan.p.step)
        if reverse:   # the same arc driven backwards
            poses = np.c_[-poses[:, 0], poses[:, 1], -poses[:, 2]]
        return poses

    def check(self, points, k, length, reverse=False, kind='arc'):
        """Clear length (m) along a path, and its clearance."""
        poses = self.path(k, length, reverse) if kind == 'arc' else shape_poses(k, kind, length, self.fan.p.step)
        n, clear = sweep_free(points, poses, self.fp)
        return n * self.fan.p.step, clear, poses

    def decide(self, points, v_u, w_u, hazards=(), held=True, dt=0.05, pushing=None):
        """The command for the user's push (v_u, w_u). points: lidar in base_footprint (self-filtered).
        pushing: is the user actually pushing on the handles (force past the deadband)? When braked hard
        the walker's speed - and so v_u - drops, but that is not the user letting go."""
        d = self._decide(points, v_u, w_u, hazards, held)
        if pushing is None:
            pushing = abs(v_u) >= self.p.min_push_speed
        if not pushing or not held:
            self.deviation = 0.0          # a new push starts from where the walker is
        else:
            self.deviation = wrap(self.deviation + (d.w - w_u) * dt)
        return d

    def _decide(self, points, v_u, w_u, hazards, held):
        p = self.p
        d = Decision(v_user=v_u, w_user=w_u)
        d.caution = self.fan.caution(hazards)
        if not held:
            d.intervention = 1.0 if abs(v_u) > p.min_push_speed or abs(w_u) > 0.05 else 0.0
            return d
        if abs(v_u) < p.min_push_speed or abs(w_u) > p.spot_turn_curvature * abs(v_u):
            if abs(w_u) < 0.02:
                return d                               # no push: no motion
            # turning on the spot (one handle pushed, the other pulled - or barely moving and turning hard):
            # passed through as pushed, only if the turning footprint is clear
            angle = math.copysign(math.radians(30), w_u)
            ok, _ = sweep(points, rotation_poses(angle), self.fp)
            d.turning_in_place = True
            if ok:
                d.v, d.w = v_u, w_u      # passed through as pushed
            else:
                d.blocked, d.intervention = True, 1.0
            return d

        reverse = v_u < 0
        k_u = w_u / v_u
        d.k_user = k_u
        L = max(p.L_min, min(p.L_max, abs(v_u) * p.horizon_s))
        free, _, poses = self.check(points, k_u, L, reverse)
        d.free = free
        unreg_near = any(h.unregistered and berth_cost(h, poses, self.fp, p.berth_registered, p.berth_unregistered) > 0
                         for h in hazards)
        d.near_unregistered = unreg_near
        missed = unreg_near or d.caution
        comfort = p.comfort_unregistered if missed else p.comfort_registered
        hazard_in_berth = any(berth_cost(h, poses[:int(round(free / self.fan.p.step)) + 1], self.fp,
                                         p.berth_registered, p.berth_unregistered) > 0 for h in hazards)

        # bend the path: find the clear tentacle closest to what the user is doing (forward only;
        # backwards it just brakes - the user is behind it, walking backwards slowly)
        k_out = k_u
        best_free, best_k = None, None
        cap_share = 1.0          # 1 = the 35 deg limit isn't holding the bend back
        if not reverse and (free < min(comfort, L) or hazard_in_berth):
            best, best_cost = None, math.inf
            limit = math.radians(p.max_deviation_deg)
            for k, kind in self.fan.tentacles:
                f, _, po = self.check(points, k, L, kind=kind)
                if f < min(self.fan.p.required_free, L):
                    continue
                # only ways *past* something: clearly better than the user's own path (better_by m more room, or
                # fully comfortable - a wall across a dead end offers neither), and ending pointed between straight on and the user's own turn, give or
                # take max_deviation - so it can't start turning the user into a junction; that's their decision.
                # Lane changes end parallel, so they always qualify.
                if f < min(comfort, L, free + p.better_by):
                    continue
                user_end = float(poses[-1, 2])
                if not (min(0.0, user_end) - limit <= float(po[-1, 2]) <= max(0.0, user_end) + limit):
                    continue
                c = (p.w_dev * abs(k - k_u) + p.w_free * (L - f) / L
                     + p.w_hazard * sum(berth_cost(h, po, self.fp, p.berth_registered, p.berth_unregistered)
                                        for h in hazards))
                if c < best_cost:
                    best, best_cost, best_free = k, c, f
            best_k = best
            if best is not None:
                risk = max(0.0, min(1.0, (comfort - free) / max(comfort - self.fan.p.required_free, 1e-3)))
                if hazard_in_berth:
                    risk = max(risk, 0.5)
                # once it decides to help it commits to at least min_commit of the bend: a too-gentle start
                # leaves it short of room beside the obstacle, and then it can only stop
                strength = (p.nudge_unregistered if missed else p.nudge_registered) * max(risk, p.min_commit)
                k_out = k_u + strength * (best - k_u)
                # never turn the walker more than max_deviation away from the user's push: past that,
                # it only brakes (a junction or a dead end is the user's decision, not the guardian's)
                limit = math.radians(p.max_deviation_deg)
                if (k_out - k_u) * self.deviation > 0:
                    # easing off over the last 15 deg, so the turn already under way doesn't carry it past
                    cap_share = max(0.0, min(1.0, (limit - abs(self.deviation)) / math.radians(15)))
                    k_out = k_u + (k_out - k_u) * cap_share
                d.nudging = abs(k_out - k_u) > 1e-3
        elif not reverse and abs(self.deviation) > 0.02:
            # clear again: undo its own correction, back to the heading the push alone would have
            k_back = k_u - p.return_gain * self.deviation
            if self.check(points, k_back, L)[0] >= min(comfort, L):
                k_out = k_back
                d.nudging = True
        d.k_out = k_out

        # speed: the user's push, held back by the wheels rule along the bent path...
        v, w = limit_to_push(k_out, v_u, w_u, p.half_track) if d.nudging else (v_u, w_u)
        # ...and by what lies ahead: slow enough to stop in the clear length (brake gentler -> sooner if missed)
        # how much room there is on the way the walker is being steered onto (the safer path it bends towards:
        # it is re-planned every tick, and slowing to a stop half way through the bend would leave it stuck)
        # - unless the 35 deg limit is stopping it from getting there: then only as far as it still can
        free_out = free
        if d.nudging:
            free_out = self.check(points, k_out, L)[0]
            if best_free is not None:
                free_out = max(free_out, free + cap_share * (best_free - free))
        brake = p.brake_unregistered if missed else p.brake_registered
        v_safe = math.sqrt(2 * brake * max(free_out - p.stop_margin, 0.0)) if free_out < L - 1e-6 else math.inf
        if d.caution:
            v_safe = min(v_safe, p.caution_scale * p.v_max)
        if abs(v) > v_safe:
            scale = v_safe / abs(v)
            v, w = v * scale, w * scale
            d.braking = True
        # blocked: the user is pushing, and the guardian is holding them at a standstill
        if not reverse and d.braking and abs(v) < p.min_push_speed:
            d.blocked = True
        d.v, d.w = v, w
        user = math.hypot(v_u, w_u * p.half_track)
        diff = math.hypot(v - v_u, (w - w_u) * p.half_track)
        d.intervention = max(0.0, min(1.0, diff / user)) if user > 1e-6 else 0.0
        return d

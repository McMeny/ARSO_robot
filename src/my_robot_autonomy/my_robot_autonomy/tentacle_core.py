"""The tentacle planner's maths, with no ROS in it so it can be unit tested.
(After von Hundelshausen et al., 2008, "Driving with Tentacles".)

Everything is in the walker's own frame (base_footprint: x forward, y left, origin on the rear
axle), except headings, which are in odom.

  tentacle    a constant-curvature arc from the walker's pose, sampled every 0.05 m as
              poses (x, y, heading)
  footprint   a rectangle covering the walker AND the user behind it, plus safety_margin
  free        how far along the tentacle the swept footprint stays further than safety_margin
              from every lidar point - except points it is already that close to and doesn't get
              closer to (so it can always drive away from something beside or behind it)
  valid       free >= required_free (braking distance plus a margin). The brief asked for the
              whole tentacle to be clear, but a constant-curvature arc that swerves round an
              obstacle in an aisle keeps curving into the aisle wall later on, so every swerve
              would be ruled out. As in the original tentacle method, a tentacle counts if it is
              clear far enough to stop, and a shorter clear length costs more; the choice is
              re-made 20 times a second anyway.
  clearance   the smallest distance from the footprint swept along the free part to any lidar point
"""
import math
from dataclasses import dataclass, field

import numpy as np

STOP, WALK, STEER = 0, 1, 2


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------- geometry ----------

@dataclass
class Footprint:
    length: float = 1.25      # m, front of the walker to the back of the user
    width: float = 0.6        # m
    offset_x: float = -0.165  # m, centre of the rectangle ahead of base_footprint
    margin: float = 0.08      # m, safety margin

    @property
    def front(self):
        return self.offset_x + self.length / 2

    def corners(self):
        hl, hw = self.length / 2, self.width / 2
        return np.array([[self.offset_x + hl, hw], [self.offset_x + hl, -hw],
                         [self.offset_x - hl, -hw], [self.offset_x - hl, hw]])


def tentacle_poses(k, length, step=0.05):
    """Poses (N, 3) along an arc of curvature k (1/m, + = left) from the origin, every `step` m."""
    s = np.linspace(0.0, length, max(int(round(length / step)), 1) + 1)
    if abs(k) < 1e-6:
        return np.c_[s, np.zeros_like(s), np.zeros_like(s)]
    return np.c_[np.sin(k * s) / k, (1 - np.cos(k * s)) / k, k * s]


def s_curve_poses(k, length, step=0.05):
    """A lane-change tentacle: curvature k for the first half, -k for the second, so it ends
    parallel to where it started, shifted sideways - for stepping round something and carrying on."""
    half = tentacle_poses(k, length / 2, step)
    x0, y0, th0 = half[-1]
    back = tentacle_poses(-k, length / 2, step)[1:]
    c, s = math.cos(th0), math.sin(th0)
    second = np.c_[x0 + c * back[:, 0] - s * back[:, 1], y0 + s * back[:, 0] + c * back[:, 1], th0 + back[:, 2]]
    return np.r_[half, second]


def shape_poses(k, kind, length, step=0.05):
    return s_curve_poses(k, length, step) if kind == 's' else tentacle_poses(k, length, step)


def end_yaw(k, length):
    """Heading change (rad) at the end of a tentacle."""
    return k * length


def rotation_poses(angle, step=math.radians(5)):
    """Poses turning on the spot from 0 to `angle` (rad)."""
    n = max(int(math.ceil(abs(angle) / step)), 1)
    th = np.linspace(0.0, angle, n + 1)
    return np.c_[np.zeros_like(th), np.zeros_like(th), th]


def footprint_distances(points, poses, fp):
    """(N poses, M points): distance from the footprint rectangle at each pose to each point,
    0 if the point is inside. points: (M, 2) in base_footprint."""
    if len(points) == 0:
        return np.full((len(poses), 0), np.inf)
    dx = points[None, :, 0] - poses[:, None, 0]
    dy = points[None, :, 1] - poses[:, None, 1]
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    lx, ly = c * dx + s * dy - fp.offset_x, -s * dx + c * dy
    ox = np.maximum(np.abs(lx) - fp.length / 2, 0.0)
    oy = np.maximum(np.abs(ly) - fp.width / 2, 0.0)
    return np.hypot(ox, oy)


def sweep(points, poses, fp):
    """(valid, clearance) for the footprint swept through `poses`."""
    if len(points) == 0:
        return True, math.inf
    d = footprint_distances(points, poses, fp)
    now = d[0]
    # too close, and closer than it already is
    worse = (d < fp.margin) & (d < now[None, :] - 1e-3)
    return (not bool(np.any(worse))), float(np.min(d[1:])) if len(d) > 1 else float(np.min(d))


def sweep_free(points, poses, fp):
    """(free poses, clearance): how many poses (after the first) the footprint can sweep through
    before getting too close to a point, and the clearance over those."""
    if len(points) == 0:
        return len(poses) - 1, math.inf
    d = footprint_distances(points, poses, fp)
    worse = np.any((d < fp.margin) & (d < d[0][None, :] - 1e-3), axis=1)
    n = int(np.argmax(worse)) - 1 if np.any(worse) else len(poses) - 1
    n = max(n, 0)
    return n, float(np.min(d[1:n + 1])) if n >= 1 else float(np.min(d[0]))


def free_length(points, k, fp, max_length, step=0.05, kind='arc'):
    """How far (m) the walker can go along a tentacle before the footprint (plus margin)
    would get closer than the margin to a point - the clearance ahead used for speed."""
    poses = shape_poses(k, kind, max_length, step)
    if len(points) == 0:
        return max_length
    d = footprint_distances(points, poses, fp)
    worse = np.any((d < fp.margin) & (d < d[0][None, :] - 1e-3), axis=1)
    hit = np.argmax(worse) if np.any(worse) else None
    return max_length if hit is None else float(hit * step)


def self_filter(points, fp):
    """Drop lidar points inside the bare footprint now - the walker itself, or the user's legs."""
    if len(points) == 0:
        return points
    d = footprint_distances(points, np.zeros((1, 3)), fp)[0]
    return points[d > 0.0]


def front_lines(poses, fp, half_width, length):
    """Centre, left and right floor lines (each (N, 2)) traced by the walker's front centre and
    front corners along the poses, `length` m long."""
    step = math.hypot(*(poses[1, :2] - poses[0, :2])) if len(poses) > 1 else 0.05
    n = min(len(poses), int(round(length / max(step, 1e-6))) + 1)
    p = poses[:n]
    c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
    fx, fy = p[:, 0] + fp.front * c, p[:, 1] + fp.front * s
    centre = np.c_[fx, fy]
    left = np.c_[fx - half_width * s, fy + half_width * c]
    right = np.c_[fx + half_width * s, fy - half_width * c]
    return centre, left, right


def outline(poses, fp, x_from=None, x_to=None, half_width=None):
    """Closed outline (5, 2) of a rectangle at the last pose. Defaults to the footprint."""
    x0 = fp.offset_x - fp.length / 2 if x_from is None else x_from
    x1 = fp.front if x_to is None else x_to
    hw = fp.width / 2 if half_width is None else half_width
    px, py, th = poses[-1]
    c, s = math.cos(th), math.sin(th)
    pts = [(x1, hw), (x1, -hw), (x0, -hw), (x0, hw), (x1, hw)]
    return np.array([[px + c * x - s * y, py + s * x + c * y] for x, y in pts])


# ---------- shared steering ----------

def authority(clearance, c_low, c_high):
    """How much say the head has (0 = machine, 1 = user), from the clearance around the walker."""
    if c_high <= c_low:
        return 1.0 if clearance >= c_high else 0.0
    return max(0.0, min(1.0, (clearance - c_low) / (c_high - c_low)))


@dataclass
class HazardLite:
    """A hazard in base_footprint, for the planner."""
    x: float
    y: float
    radius: float
    unregistered: bool   # not registered by the EEG, or no reliable EEG


def berth_cost(h, poses, fp, berth_registered, berth_unregistered):
    """The hazard's berth: 0 once the walker's side passes further than the berth from it, rising
    linearly, (berth - gap) / berth_registered, inside it. Wider when unregistered.
    (Not a plain Gaussian: with a wider sigma that is flatter close in, so it would push *less*.
    The hinge has the same slope for both berths but the wide one reaches further, so it is never
    weaker; and it is bounded, so it can't shove the walker into some other obstacle.)"""
    berth = berth_unregistered if h.unregistered else berth_registered
    gap = np.min(np.hypot(poses[:, 0] - h.x, poses[:, 1] - h.y)) - h.radius - fp.width / 2
    return float(max(berth - gap, 0.0) / berth_registered)


@dataclass
class Params:
    num_tentacles: int = 31
    k_max: float = 1.5
    horizon_s: float = 2.0
    L_min: float = 2.5               # brief: 1.0 - but at 0.4 m/s that leaves no room to swerve gently
    L_max: float = 2.5
    step: float = 0.05
    required_free: float = 0.7      # m a tentacle must be clear for to count as valid
    w_free: float = 1.5             # cost of a shorter clear length
    w_clear: float = 0.3
    clearance_cap: float = 1.0      # clearances above this all count the same
    w_head_max: float = 2.0
    c_low: float = 0.15
    c_high: float = 0.8
    w_hazard: float = 1.0
    berth_registered: float = 0.15
    berth_unregistered: float = 0.45
    w_hyst: float = 0.5
    w_curv: float = 0.3
    w_return: float = 0.8           # WALK: pull back to the heading WALK started with
    walk_max_turn: float = math.radians(45)   # WALK never turns further than this on its own
    v_max: float = 0.4
    omega_max: float = 0.8
    caution_scale: float = 0.5
    caution_radius: float = 2.0
    caution_half_width: float = 1.0
    stop_length: float = 0.1        # clearance ahead at which the speed reaches its minimum
    slow_length: float = 1.2        # ...and above which it is full speed
    min_speed_scale: float = 0.2
    lookahead_max: float = 3.0      # m checked ahead along the chosen tentacle for speed
    heading_near: float = 0.6       # m along a tentacle where its heading is judged...
    heading_far: float = 1.2        # ...and here
    lane_change: bool = True        # add S-shaped lane-change tentacles to the brief's arcs
    lane_change_k_max: float = 1.0


@dataclass
class Choice:
    index: int = -1
    k: float = 0.0              # curvature to drive now (the first part of an S-curve)
    kind: str = 'arc'           # 'arc' or 's' (lane change)
    length: float = 0.0
    valid: list = field(default_factory=list)
    costs: list = field(default_factory=list)
    clearances: list = field(default_factory=list)
    authority: float = 0.0
    caution: bool = False
    blocked: bool = False
    near_unregistered: bool = False


class TentaclePlanner:
    def __init__(self, params=None, footprint=None):
        self.p = params or Params()
        self.fp = footprint or Footprint()
        self.curvatures = np.linspace(self.p.k_max, -self.p.k_max, self.p.num_tentacles)  # left -> right
        # the brief's arcs, plus lane-change S-curves (not too tight: they'd fold back on themselves)
        self.tentacles = [(float(k), 'arc') for k in self.curvatures]
        if self.p.lane_change:
            self.tentacles += [(float(k), 's') for k in self.curvatures if 1e-6 < abs(k) <= self.p.lane_change_k_max]
        self._cache = {}

    def length(self, v_current):
        return max(self.p.L_min, min(self.p.L_max, abs(v_current) * self.p.horizon_s))

    def poses(self, k, length, kind='arc'):
        key = (round(float(k), 4), round(length, 2), kind)   # precomputed once per length bucket
        if key not in self._cache:
            self._cache[key] = shape_poses(k, kind, round(length, 2), self.p.step)
        return self._cache[key]

    def caution(self, hazards):
        for h in hazards:
            if not h.unregistered or h.x < -0.2:
                continue
            ahead = math.hypot(h.x - self.fp.front, h.y) - h.radius
            if ahead < self.p.caution_radius and abs(h.y) < self.p.caution_half_width + h.radius:
                return True
        return False

    def choose(self, points, v_current, heading, desired_yaw, grip_state, k_prev, hazards=(), ref_heading=None):
        """Pick the lowest-cost valid tentacle. points (M, 2) in base_footprint (self-filtered).
        heading / desired_yaw / ref_heading in odom (rad)."""
        p = self.p
        L = self.length(v_current)
        out = Choice(length=L)
        near = points[np.hypot(points[:, 0], points[:, 1]) < L + self.fp.length + 1.0] if len(points) else points
        _, around = sweep(near, np.zeros((1, 3)), self.fp)
        out.authority = authority(around, p.c_low, p.c_high) if grip_state == STEER else 0.0
        w_head = p.w_head_max * out.authority
        out.caution = self.caution(hazards)
        best = math.inf
        for i, (k, kind) in enumerate(self.tentacles):
            poses = self.poses(k, L, kind)
            n, clear = sweep_free(near, poses, self.fp)
            free = n * p.step
            ok = free >= min(p.required_free, L) - 1e-6
            turn = float(poses[-1, 2])
            near_h = float(poses[min(int(round(p.heading_near / p.step)), len(poses) - 1), 2])
            far_h = float(poses[min(int(round(p.heading_far / p.step)), len(poses) - 1), 2])
            # heading terms use where the tentacle points heading_near and heading_far m along: only its start is
            # ever driven before re-planning, so an S-curve that first swings away mustn't look free (judging by
            # its end), and a turn is judged over the next metre or so, not 2.5 m away (too sluggish)
            off = lambda target: 0.5 * (abs(wrap(heading + near_h - target)) + abs(wrap(heading + far_h - target)))
            if ok and grip_state == WALK and ref_heading is not None:   # never turns far on its own
                ok = float(np.max(np.abs(np.arctan2(np.sin(heading + poses[:, 2] - ref_heading),
                                                    np.cos(heading + poses[:, 2] - ref_heading))))) <= p.walk_max_turn
            out.valid.append(ok)
            out.clearances.append(clear)
            if not ok:
                out.costs.append(math.inf)
                continue
            c = min(clear, p.clearance_cap)
            J = p.w_clear * (1.0 / (c + 0.05) - 1.0 / (p.clearance_cap + 0.05))
            J += w_head * off(desired_yaw)
            if grip_state == WALK and ref_heading is not None:
                J += p.w_return * off(ref_heading)
            J += p.w_hazard * sum(berth_cost(h, poses[:n + 1], self.fp, p.berth_registered, p.berth_unregistered)
                                  for h in hazards if h.x > -0.5)
            J += p.w_free * (L - free) / L
            J += p.w_hyst * abs(k - k_prev)
            J += p.w_curv * abs(k)
            out.costs.append(J)
            if J < best:
                best, out.index, out.k, out.kind = J, i, float(k), kind
        out.blocked = out.index < 0
        if not out.blocked:
            poses = self.poses(out.k, L, out.kind)
            out.near_unregistered = any(
                h.unregistered and berth_cost(h, poses, self.fp, p.berth_registered, p.berth_unregistered) > 0.0
                for h in hazards)
        return out

    def speed(self, points, choice):
        """v and omega for the chosen tentacle."""
        p = self.p
        if choice.blocked:
            return 0.0, 0.0
        ahead = free_length(points, choice.k, self.fp, p.lookahead_max, 0.1, choice.kind)
        scale = (ahead - p.stop_length) / max(p.slow_length - p.stop_length, 1e-3)
        scale = max(p.min_speed_scale, min(1.0, scale))
        v = p.v_max * scale * (p.caution_scale if choice.caution else 1.0)
        w = max(-p.omega_max, min(p.omega_max, v * choice.k))
        return v, w

    def pivot_clear(self, points, angle):
        """Can the walker (and user) turn on the spot through `angle`?"""
        ok, _ = sweep(points, rotation_poses(angle), self.fp)
        return ok


# ---------- the hint and the blocked notice ----------

class HintLogic:
    """'Turn left? Squeeze to steer' when the user keeps looking away from the walker's heading in WALK.
    And 'Path blocked. Squeeze to steer out' when the walker has been blocked for a moment."""

    def __init__(self, hint_angle=math.radians(30), hint_dwell_s=3.0, hint_cooldown_s=30.0,
                 hint_max_successes=3, hint_show_s=5.0, blocked_notice_delay_s=1.0):
        self.angle = hint_angle
        self.dwell = hint_dwell_s
        self.cooldown = hint_cooldown_s
        self.max_successes = hint_max_successes
        self.show = hint_show_s
        self.notice_delay = blocked_notice_delay_s
        self.look_since = None
        self.last_hint = -math.inf
        self.hint = ''
        self.hint_until = -math.inf
        self.successes = 0
        self.blocked_since = None
        self.steer_start_heading = None

    def update(self, t, grip_state, look, caution, blocked, heading):
        """look: head yaw minus walker heading (rad), None if no head data.
        Returns (hint, notice)."""
        # successful STEER turns: a STEER that turned the walker by more than hint_angle
        if grip_state == STEER and self.steer_start_heading is None:
            self.steer_start_heading = heading
        elif grip_state != STEER and self.steer_start_heading is not None:
            if abs(wrap(heading - self.steer_start_heading)) > self.angle:
                self.successes += 1
            self.steer_start_heading = None

        # blocked notice: after a short debounce, until STEER starts or a way is free again
        if blocked and grip_state != STEER:
            if self.blocked_since is None:
                self.blocked_since = t
        else:
            self.blocked_since = None
        notice = 'Path blocked. Squeeze to steer out' if (
            self.blocked_since is not None and t - self.blocked_since >= self.notice_delay) else ''
        if notice:
            self.hint, self.look_since = '', None
            return '', notice

        # hint
        looking_away = grip_state == WALK and look is not None and abs(look) > self.angle and not caution
        if not looking_away:
            self.look_since = None
            if grip_state != WALK or (look is not None and abs(look) <= self.angle):
                self.hint = ''
        elif self.look_since is None:
            self.look_since = t
        if (looking_away and t - self.look_since >= self.dwell and not self.hint
                and t - self.last_hint >= self.cooldown and self.successes < self.max_successes):
            side = 'around' if abs(look) > math.radians(120) else ('left' if look > 0 else 'right')
            self.hint = f'Turn {side}? Squeeze to steer'
            self.last_hint = t
            self.hint_until = t + self.show
        if self.hint and t > self.hint_until:
            self.hint = ''
        return self.hint, ''

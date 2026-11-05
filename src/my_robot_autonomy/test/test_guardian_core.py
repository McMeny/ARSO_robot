import math

import numpy as np
import pytest

from my_robot_autonomy.guardian_core import (Admittance, DeadMan, Guardian, GuardParams, head_steer, limit_to_push,
                                             within_push, wheel_speeds)
from my_robot_autonomy.tentacle_core import HazardLite

NONE = np.empty((0, 2))


def wall_ahead(x):
    return np.c_[np.full(80, x), np.linspace(-2, 2, 80)]


# ---------- admittance and dead-man (ported from main) ----------

def test_admittance_push_forward_and_turn():
    a = Admittance()
    for _ in range(200):
        v, w = a.step(14.0, 14.0, 0.02)
    assert v == pytest.approx((28 - 8) / 40, rel=0.02)    # steady state F / damping, after the deadband
    a.reset()
    for _ in range(200):
        v, w = a.step(10.0, 20.0, 0.02)
    assert w > 0                                           # right handle harder -> turns left


def test_admittance_deadband_and_reverse_limit():
    a = Admittance()
    for _ in range(100):
        v, w = a.step(3.0, 3.0, 0.02)
    assert v == 0.0 and w == 0.0                           # resting hands don't move it
    for _ in range(500):
        v, _ = a.step(-60.0, -60.0, 0.02)
    assert v == pytest.approx(-0.25)                       # backwards at half speed at most


def test_deadman():
    d = DeadMan(0.3)
    assert d.update(True, 0.0) is False
    assert d.update(False, 0.1) is False
    assert d.update(False, 0.45) is True


# ---------- the never-pushes rule ----------

def test_limit_to_push_brakes_one_wheel():
    v_u, w_u = 0.4, 0.0
    v, w = limit_to_push(0.5, v_u, w_u, 0.22)              # bend left: hold the left wheel back
    l, r = wheel_speeds(v, w, 0.22)
    assert r == pytest.approx(0.4) and l < 0.4
    assert within_push(v, w, v_u, w_u, 0.22)


def test_no_push_no_motion():
    g = Guardian()
    d = g.decide(NONE, 0.0, 0.0)
    assert d.v == 0.0 and d.w == 0.0


def test_released_grip_is_zero_even_if_pushing():
    d = Guardian().decide(NONE, 0.4, 0.0, held=False)
    assert d.v == 0.0 and d.w == 0.0


def test_open_space_passes_the_push_untouched():
    d = Guardian().decide(NONE, 0.4, 0.1)
    assert d.v == pytest.approx(0.4) and d.w == pytest.approx(0.1) and d.intervention == 0.0


def test_never_faster_than_the_push_anywhere():
    g = Guardian()
    rng = np.random.default_rng(1)
    for _ in range(60):
        pts = rng.uniform(-3, 4, (60, 2))
        pts = pts[np.hypot(pts[:, 0] - 0.2, pts[:, 1]) > 0.9]
        v_u, w_u = rng.uniform(-0.25, 0.5), rng.uniform(-0.6, 0.6)
        d = g.decide(pts, v_u, w_u, [HazardLite(1.5, 0.1, 0.3, bool(rng.integers(2)))])
        assert within_push(d.v, d.w, v_u, w_u, g.p.half_track)


# ---------- braking and bending ----------

def test_brakes_for_a_wall_ahead():
    g = Guardian()
    far = g.decide(wall_ahead(3.5), 0.4, 0.0)
    near = g.decide(wall_ahead(1.0), 0.4, 0.0)
    assert far.v == pytest.approx(0.4)
    assert near.v < 0.35 and near.braking


def test_bends_round_an_obstacle_by_braking_one_side():
    g = Guardian()
    box = np.array([[x, y] for x in np.linspace(1.6, 2.1, 6) for y in np.linspace(-0.1, 0.3, 5)])
    d = g.decide(box, 0.4, 0.0)
    assert d.nudging and d.w < 0                           # obstacle left of centre -> bends right
    assert d.v < 0.4


def test_missed_hazard_gets_earlier_firmer_help():
    g = Guardian()
    box = np.array([[x, y] for x in np.linspace(2.4, 2.9, 6) for y in np.linspace(-0.1, 0.3, 5)])
    reg = g.decide(box, 0.4, 0.0, [HazardLite(2.65, 0.1, 0.32, False)])
    miss = g.decide(box, 0.4, 0.0, [HazardLite(2.65, 0.1, 0.32, True)])
    assert miss.intervention > reg.intervention
    assert miss.v < reg.v
    assert miss.caution and not reg.caution


def test_turn_on_the_spot_only_if_clear():
    g = Guardian()
    assert g.decide(NONE, 0.0, 0.5).w == pytest.approx(0.5)
    tight = np.array([[-0.6, 0.45], [-0.6, -0.45], [0.5, 0.4], [0.5, -0.4]])
    d = g.decide(tight, 0.0, 0.5)
    assert d.w == 0.0 and d.blocked


def test_does_not_bend_towards_a_wall_across_the_way():
    # a wall right across the way (a dead end): no way past it, so it only brakes - no turning the user
    g = Guardian()
    d = g.decide(wall_ahead(1.1), 0.4, 0.0)
    assert not d.nudging and d.w == 0.0 and d.v < 0.4


def test_tight_turning_push_counts_as_turning_on_the_spot():
    d = Guardian().decide(NONE, 0.03, 0.4)          # barely moving, turning hard
    assert d.turning_in_place and d.w == pytest.approx(0.4) and d.v == pytest.approx(0.03)


def test_never_turns_the_user_more_than_the_limit():
    g = Guardian()
    box = np.array([[x, y] for x in np.linspace(1.6, 2.1, 6) for y in np.linspace(-0.4, 0.4, 9)])
    for _ in range(400):                             # keep pushing straight at it, holding the scene still
        g.decide(box, 0.3, 0.0)
    assert abs(g.deviation) <= math.radians(g.p.max_deviation_deg) + 0.05


# ---------- head turn ----------

def test_head_turn_steers_towards_the_head_within_the_push():
    p = GuardParams()
    for error in (0.3, 1.0, 3.0, -0.3, -1.0, -3.0):
        v, w, steering = head_steer(0.4, 0.0, error, p)
        assert steering and v > 0.2
        assert math.copysign(1, w) == math.copysign(1, error)
        assert within_push(v, w, 0.4, 0.0, p.half_track)


def test_head_turn_stays_a_curve_never_a_pivot():
    p = GuardParams()
    v, w, _ = head_steer(0.4, 0.0, 3.0, p)
    assert w / v == pytest.approx(p.head_max_curvature)
    assert p.head_max_curvature < p.spot_turn_curvature


def test_head_turn_needs_a_forward_push_and_a_real_head_turn():
    p = GuardParams()
    assert head_steer(0.0, 0.0, 1.0, p) == (0.0, 0.0, False)        # no push: no motion
    assert head_steer(-0.2, 0.0, 1.0, p) == (-0.2, 0.0, False)      # backing up
    assert head_steer(0.4, 0.1, math.radians(9), p) == (0.4, 0.1, False)   # inside the dead zone


def test_head_turn_gives_up_rather_than_stop_a_hard_opposite_push():
    p = GuardParams()
    assert head_steer(0.05, -0.8, 1.0, p) == (0.05, -0.8, False)


def test_guardian_still_stops_a_head_turn_into_a_wall():
    g = Guardian()
    p = g.p
    pts = np.c_[np.linspace(0, 3, 80), np.full(80, 0.5)]            # a wall along the left
    v, w, _ = head_steer(0.4, 0.0, math.pi / 2, p)
    d = g.decide(pts, v, w, pushing=True)
    assert d.braking or d.nudging
    assert abs(d.v) < v or d.w < w

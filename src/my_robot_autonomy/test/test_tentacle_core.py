import math

import numpy as np
import pytest

from my_robot_autonomy.tentacle_core import (
    Footprint, Params, TentaclePlanner, HazardLite, HintLogic, STOP, WALK, STEER,
    tentacle_poses, end_yaw, footprint_distances, sweep, front_lines, authority, self_filter)


def wall(x0, x1, y, n=60):
    return np.c_[np.linspace(x0, x1, n), np.full(n, y)]


# ---------- geometry ----------

def test_straight_tentacle():
    p = tentacle_poses(0.0, 2.0)
    assert np.allclose(p[-1], [2.0, 0.0, 0.0])
    assert len(p) == 41


def test_max_curvature_tentacle_length_and_end_yaw():
    k, L = 1.5, 1.0
    p = tentacle_poses(k, L)
    steps = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))
    assert steps.sum() == pytest.approx(L, rel=1e-3)       # arc length ~ L
    assert p[-1, 2] == pytest.approx(end_yaw(k, L))
    assert p[-1, 0] == pytest.approx(math.sin(1.5) / 1.5)
    assert p[-1, 1] == pytest.approx((1 - math.cos(1.5)) / 1.5)


def test_footprint_collision_with_points():
    fp = Footprint()
    inside = np.array([[0.2, 0.0]])
    outside = np.array([[fp.front + 0.5, 0.0]])
    assert footprint_distances(inside, np.zeros((1, 3)), fp)[0, 0] == 0.0
    assert footprint_distances(outside, np.zeros((1, 3)), fp)[0, 0] == pytest.approx(0.5)
    # a straight tentacle through a point ahead is invalid; one curving away is fine
    point = np.array([[1.0, 0.0]])
    assert not sweep(point, tentacle_poses(0.0, 1.0), fp)[0]
    assert sweep(np.array([[1.0, -0.9]]), tentacle_poses(0.0, 1.0), fp)[0]


def test_can_drive_away_from_something_already_close():
    fp = Footprint()
    beside = wall(-0.8, 0.4, fp.width / 2 + 0.03)      # 3 cm off the side, inside the margin
    ok, _ = sweep(beside, tentacle_poses(0.0, 1.0), fp)
    assert ok
    ok, _ = sweep(beside, tentacle_poses(1.0, 1.0), fp)  # curving into it is not
    assert not ok


def test_self_filter_drops_points_inside_the_footprint():
    fp = Footprint()
    pts = np.array([[-0.5, 0.1], [2.0, 0.0]])          # the user's legs, and an obstacle
    assert np.allclose(self_filter(pts, fp), [[2.0, 0.0]])


def test_edge_lines_are_offset_perpendicular():
    fp = Footprint()
    for k in (0.0, 1.0):
        poses = tentacle_poses(k, 1.0)
        c, l, r = front_lines(poses, fp, 0.25, 1.0)
        assert np.allclose(np.hypot(*(l - c).T), 0.25)
        assert np.allclose(np.hypot(*(r - c).T), 0.25)
        heading = np.c_[np.cos(poses[:, 2]), np.sin(poses[:, 2])]
        assert np.allclose(np.sum((l - c) * heading, axis=1), 0.0, atol=1e-9)   # perpendicular
        assert np.all(np.cross(heading, l - c) > 0)                             # left is left


# ---------- costs ----------

def test_obstacle_on_the_left_makes_a_right_tentacle_win():
    planner = TentaclePlanner()
    obstacle = np.array([[1.8, y] for y in np.linspace(0.0, 0.5, 10)])
    c = planner.choose(obstacle, 0.4, 0.0, 0.0, WALK, 0.0, ref_heading=0.0)
    assert not c.blocked
    assert c.k < 0


def test_open_space_walk_goes_straight():
    c = TentaclePlanner().choose(np.empty((0, 2)), 0.3, 0.0, 0.0, WALK, 0.0, ref_heading=0.0)
    assert c.k == pytest.approx(0.0)


def test_steer_follows_the_head():
    c = TentaclePlanner().choose(np.empty((0, 2)), 0.3, 0.0, math.radians(45), STEER, 0.0)
    assert c.k >= 0.25


def test_walk_does_not_turn_far_on_its_own():
    planner = TentaclePlanner()
    end_wall = np.c_[np.full(80, 1.0), np.linspace(-2, 2, 80)]   # a wall 1 m ahead
    c = planner.choose(end_wall, 0.3, 0.0, 0.0, WALK, 0.0, ref_heading=0.0)
    assert c.blocked
    c = planner.choose(end_wall, 0.3, 0.0, math.radians(90), STEER, 0.0)
    assert c.blocked or c.k > 0


def test_unregistered_hazard_gets_a_wider_berth():
    planner = TentaclePlanner()
    pts = np.array([[1.6, 0.15]])
    reg = planner.choose(pts, 0.4, 0.0, 0.0, WALK, 0.0, [HazardLite(1.6, 0.15, 0.2, False)], 0.0)
    unreg = planner.choose(pts, 0.4, 0.0, 0.0, WALK, 0.0, [HazardLite(1.6, 0.15, 0.2, True)], 0.0)
    assert unreg.caution and not reg.caution
    assert abs(unreg.k) >= abs(reg.k)
    v_reg, _ = planner.speed(pts, reg)
    v_unreg, _ = planner.speed(pts, unreg)
    assert v_unreg < v_reg


def test_risk_scaled_authority():
    assert authority(0.1, 0.15, 0.8) == 0.0
    assert authority(0.475, 0.15, 0.8) == pytest.approx(0.5)
    assert authority(2.0, 0.15, 0.8) == 1.0
    planner = TentaclePlanner()
    assert planner.choose(np.empty((0, 2)), 0.3, 0.0, 0.0, WALK, 0.0).authority == 0.0
    assert planner.choose(np.empty((0, 2)), 0.3, 0.0, 0.0, STEER, 0.0).authority == 1.0
    tight = wall(-1, 1.5, 0.45)
    assert planner.choose(tight, 0.3, 0.0, 0.0, STEER, 0.0).authority < 1.0


def test_hysteresis_keeps_the_previous_tentacle():
    planner = TentaclePlanner()
    i = planner.tentacles.index(next(t for t in planner.tentacles if t[1] == 'arc' and abs(t[0] - 0.5) < 1e-6))
    near = planner.choose(np.empty((0, 2)), 0.3, 0.0, 0.3, STEER, 0.5)      # coming from k = 0.5
    far = planner.choose(np.empty((0, 2)), 0.3, 0.0, 0.3, STEER, -0.5)      # coming from k = -0.5
    assert far.costs[i] - near.costs[i] == pytest.approx(planner.p.w_hyst * 1.0)


def test_lane_change_tentacle_ends_parallel_and_shifted():
    from my_robot_autonomy.tentacle_core import s_curve_poses
    p = s_curve_poses(0.5, 2.5)
    assert p[-1, 2] == pytest.approx(0.0, abs=1e-9)
    assert p[-1, 1] > 0.3


def test_s_curve_does_not_hide_a_turn_away():
    # already past the wanted heading: a left S-curve must not beat straight on
    planner = TentaclePlanner()
    c = planner.choose(np.empty((0, 2)), 0.4, math.radians(50), math.radians(45), STEER, 0.7)
    assert c.k <= 0.1


# ---------- hint and notice ----------

def test_hint_needs_dwell_and_respects_cooldown():
    h = HintLogic()
    left = math.radians(50)
    assert h.update(0.0, WALK, left, False, False, 0.0) == ('', '')
    assert h.update(2.9, WALK, left, False, False, 0.0)[0] == ''
    assert h.update(3.1, WALK, left, False, False, 0.0)[0] == 'Turn left? Squeeze to steer'
    h.update(4.0, WALK, 0.0, False, False, 0.0)                    # looks back: hint clears
    assert h.update(4.1, WALK, 0.0, False, False, 0.0)[0] == ''
    h.update(5.0, WALK, -left, False, False, 0.0)
    assert h.update(9.0, WALK, -left, False, False, 0.0)[0] == ''   # within the 30 s cooldown
    h.update(40.0, WALK, math.radians(150), False, False, 0.0)
    assert h.update(44.0, WALK, math.radians(150), False, False, 0.0)[0] == 'Turn around? Squeeze to steer'


def test_no_hint_near_a_caution_hazard_or_after_successes():
    h = HintLogic()
    h.update(0.0, WALK, 1.0, True, False, 0.0)
    assert h.update(4.0, WALK, 1.0, True, False, 0.0)[0] == ''
    for i in range(3):                      # three successful STEER turns
        h.update(10 + i * 2, STEER, 1.0, False, False, 0.0)
        h.update(11 + i * 2, WALK, 0.0, False, False, 1.0 * (i + 1))
    h.update(100.0, WALK, 1.0, False, False, 3.0)
    assert h.update(105.0, WALK, 1.0, False, False, 3.0)[0] == ''


def test_blocked_notice_debounce_and_clear():
    h = HintLogic()
    assert h.update(0.0, WALK, 1.0, False, True, 0.0)[1] == ''
    assert h.update(0.9, WALK, 1.0, False, True, 0.0)[1] == ''
    hint, notice = h.update(1.1, WALK, 1.0, False, True, 0.0)
    assert notice == 'Path blocked. Squeeze to steer out' and hint == ''
    assert h.update(1.2, STEER, 1.0, False, True, 0.0)[1] == ''      # clears as soon as STEER starts
    h.update(2.0, WALK, 0.0, False, True, 0.0)
    assert h.update(3.1, WALK, 0.0, False, True, 0.0)[1] != ''       # shown again every time
    assert h.update(3.2, WALK, 0.0, False, False, 0.0)[1] == ''      # a way is free again

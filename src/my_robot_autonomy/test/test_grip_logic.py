import pytest

from my_robot_autonomy.grip_logic import GripLogic, STOP, WALK, STEER


def run(g, readings):
    return [g.update(l, r, t) for t, l, r in readings]


def test_walk_and_stop_with_grace():
    g = GripLogic()
    assert g.update(0.0, 0.0, 0.0) == STOP
    assert g.update(0.4, 0.4, 0.1) == WALK
    assert g.update(0.0, 0.4, 0.2) == WALK      # one hand slips: grace period
    assert g.update(0.0, 0.4, 0.35) == WALK
    assert g.update(0.0, 0.4, 0.45) == STOP     # > 0.2 s: one hand off counts as STOP


def test_brief_slip_does_not_stop():
    g = GripLogic()
    g.update(0.4, 0.4, 0.0)
    g.update(0.4, 0.0, 0.1)
    assert g.update(0.4, 0.4, 0.2) == WALK


def test_steer_needs_half_second_hold():
    g = GripLogic()
    g.update(0.4, 0.4, 0.0)
    assert g.update(0.9, 0.9, 0.1) == WALK
    assert g.update(0.9, 0.9, 0.5) == WALK
    assert g.update(0.9, 0.9, 0.61) == STEER
    assert g.update(0.4, 0.4, 0.7) == WALK


def test_startle_spike_ignored():
    g = GripLogic()
    g.update(0.4, 0.4, 0.0)
    states = run(g, [(0.1 + 0.05 * i, 1.0, 1.0) for i in range(6)] + [(0.45, 0.4, 0.4)])
    assert STEER not in states


def test_one_hand_firm_is_not_steer():
    g = GripLogic()
    g.update(0.4, 0.4, 0.0)
    assert run(g, [(0.1 * i, 0.9, 0.4) for i in range(1, 10)])[-1] == WALK


def test_hysteresis():
    g = GripLogic(light_threshold=0.2, hysteresis=0.05)
    assert g.update(0.22, 0.22, 0.0) == STOP     # below 0.25: not yet held
    assert g.update(0.26, 0.26, 0.1) == WALK
    assert g.update(0.17, 0.17, 0.2) == WALK     # still above 0.15
    g.update(0.14, 0.14, 0.3)
    assert g.update(0.14, 0.14, 0.6) == STOP


def test_one_handed():
    g = GripLogic(one_handed=True, hand='left')
    assert g.update(0.4, 0.0, 0.0) == WALK
    g.update(0.9, 0.0, 0.1)
    assert g.update(0.9, 0.0, 0.7) == STEER


def test_latched_safety_stop_needs_regrip():
    g = GripLogic()
    g.update(0.4, 0.4, 0.0)
    g.force_stop(latch=True)
    assert g.update(0.4, 0.4, 0.1) == STOP
    g.update(0.0, 0.0, 0.2)
    assert g.update(0.4, 0.4, 0.3) == WALK


def test_calibration():
    g = GripLogic()
    g.calibrate(relaxed=0.3, firm=0.9)
    assert g.light == pytest.approx(0.15)
    assert g.firm == pytest.approx(0.6)
    with pytest.raises(ValueError):
        g.calibrate(relaxed=0.5, firm=0.55)

"""M2: mux priorities, watchdogs, dead-man, admittance, pull-back, kinematics, footprint, collision monitor."""
import math

import numpy as np

from eyesteer_core import config
from eyesteer_core.control.admittance import Admittance, pullback_scale
from eyesteer_core.control.diffdrive_kinematics import EncoderOdometry, RateLimiter, twist_to_wheels, wheels_to_twist
from eyesteer_core.control.footprint import CollisionMonitor, Footprint, point_in_poly
from eyesteer_core.control.mux_rules import CmdVelWatchdog, DeadMan, Mux, Twist, twist_mux_yaml


def test_mux_priorities():
    m = Mux()
    m.input("navigation", Twist(0.3, 0), 0.0)
    assert m.output(0.1) == (Twist(0.3, 0), "NAVIGATION")
    m.input("teleop", Twist(0.1, 0.5), 0.1)
    assert m.output(0.2)[1] == "TELEOP"
    m.input("admittance", Twist(0.2, 0), 0.2)
    assert m.output(0.25)[1] == "ADMITTANCE"
    m.lock("deadman", True)
    assert m.output(0.25) == (Twist(), "LOCK:DEADMAN")
    m.lock("deadman", False)
    m.lock("estop", True)
    assert m.output(0.25) == (Twist(), "LOCK:ESTOP")
    m.lock("estop", False)
    # teleop times out (0.5 s) -> navigation resumes
    m.input("navigation", Twist(0.3, 0), 1.0)
    assert m.output(1.0)[1] == "NAVIGATION"
    assert m.output(5.0)[1] == "NONE"


def test_twist_mux_yaml_matches_rules():
    y = twist_mux_yaml()["twist_mux"]["ros__parameters"]
    assert y["locks"]["estop"]["priority"] > y["topics"]["admittance"]["priority"] > y["topics"]["teleop"]["priority"] > y["topics"]["navigation"]["priority"]


def test_cmd_vel_watchdog():
    w = CmdVelWatchdog(0.5)
    w.feed(Twist(0.4, 0), 0.0)
    assert w.output(0.3) == (Twist(0.4, 0), False)
    assert w.output(0.6) == (Twist(), True)


def test_deadman():
    d = DeadMan(0.3)
    assert d.update(True, 0.0) is False
    assert d.update(False, 1.0) is False
    assert d.update(False, 1.29) is False
    assert d.update(False, 1.31) is True
    assert d.update(True, 1.4) is False


def test_admittance_push_and_turn():
    a = Admittance(handle_y=0.25)
    for _ in range(300):
        v, w = a.step(15.0, 15.0, 0.01)
    assert v > 0.2 and abs(w) < 1e-6
    a.reset()
    for _ in range(300):
        v, w = a.step(5.0, 15.0, 0.01)
    assert w > 0.0  # right handle pushed harder -> turn left
    a.reset()
    assert a.step(2.0, 2.0, 0.01) == (0.0, 0.0)  # inside deadband


def test_pullback_scaling_user_wins():
    assert pullback_scale(0, 0) == 1.0
    assert pullback_scale(-10, -10) < 1.0
    assert pullback_scale(-20, -20) == 0.0


def test_kinematics_roundtrip_and_odometry():
    r, b = 0.1, 0.56
    wl, wr = twist_to_wheels(0.4, 0.3, r, b)
    v, w = wheels_to_twist(wl, wr, r, b)
    assert math.isclose(v, 0.4) and math.isclose(w, 0.3)
    odo = EncoderOdometry(r, b)
    al = ar = 0.0
    odo.update(0.0, 0.0, 0.0)
    for _ in range(1000):
        al += wl * 0.01
        ar += wr * 0.01
        odo.update(al, ar, 0.01)
    R = 0.4 / 0.3
    assert math.isclose(odo.x, R * math.sin(3.0), abs_tol=1e-3)


def test_rate_limiter():
    lim = RateLimiter(0.5, 0.8, 0.4, 1.0)
    v, _ = lim(1.0, 0, 0.1)
    assert math.isclose(v, 0.04)
    for _ in range(100):
        v, w = lim(1.0, 2.0, 0.1)
    assert v == 0.5 and w == 0.8


def test_walker_footprint_includes_user():
    fp = config.geometry("walker")["footprint"]
    f = Footprint(fp)
    rear = min(p[0] for p in fp["polygon"])
    handle_x = config.robot("walker")["handles"]["x"]
    assert rear <= handle_x - 0.5
    assert point_in_poly(np.array([[handle_x - 0.3, 0.0]]), f.poly)[0]


def test_collision_monitor_stops_before_contact():
    fp = config.geometry("walker")["footprint"]
    cm = CollisionMonitor(fp)
    front = max(p[0] for p in fp["polygon"])
    for dist, expect in ((1.0, "OK"), (0.3, "SLOWDOWN"), (0.05, "STOP")):
        pts = np.array([[front + dist, y] for y in np.linspace(-0.1, 0.1, 5)])
        assert cm(pts)[1] == expect
    # obstacle beside / behind does not deadlock
    pts = np.array([[-0.5, 0.45], [-0.6, -0.45]])
    assert cm(pts)[1] == "OK"

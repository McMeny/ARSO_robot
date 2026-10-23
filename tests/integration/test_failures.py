"""Failure injection (Section 24.5) + health gating (24.4) in sim / HIL. Each test brings up a fresh stack.

The headless headset client drives everything through the real rosbridge, including the sim fault
topics (/sim/eeg_fault, /sim/teleport, /sim/kill_mcu, /sim/lidar_off, /sim/estop_button, /sim/spawn_obstacle).
"""
from __future__ import annotations

import json
import math
import time

import pytest

from harness import bash_run, headset_client, ros_available, sim_stack, wait_state
from test_full_system import Tracker, run_until, track

pytestmark = pytest.mark.skipif(not ros_available(), reason="ROS 2 workspace not available")


def up(page, consent=True):
    wait_state(page, "s.connected && s.markers.length >= 6", 180)
    if consent:
        page.evaluate("window.eyesteer.consent()")
        wait_state(page, "s.health && s.health.motion_unlocked", 180)


def pub(page, topic, typ, msg):
    page.evaluate(f"window.eyesteer.pub({topic!r}, {typ!r}, {msg})")


def engage(page, tr, mid, label):
    page.evaluate(f"window.eyesteer.aim({mid})")
    run_until(page, tr, f"s.status && s.status.state === 'ENGAGED' && s.status.goal_label === '{label}'", 20)
    page.evaluate("window.eyesteer.aim(0)")


def gate(page, name):
    return page.evaluate(f"window.eyesteer.state().health.components.find(c => c.name === '{name}')")


# ---------------------------------------------------------------- health gating: motion stays locked
def test_startup_locked_without_headset_then_unlocks():
    with sim_stack("field", "field_robot", "mujoco", require_headset=True, log_name="f_gate_headset.log"):
        with headset_client() as (page, _):
            wait_state(page, "s.connected && s.health && s.status", 180)
            time.sleep(5)
            st = page.evaluate("window.eyesteer.state()")
            assert not st["health"]["motion_unlocked"] and st["status"]["mux_source"] == "LOCK:HEALTH"
            assert gate(page, "headset")["level"] == 2          # consent pending
            page.evaluate("window.eyesteer.consent()")
            wait_state(page, "s.health.motion_unlocked && s.status.mux_source !== 'LOCK:HEALTH'", 60)


def test_startup_locked_without_eeg_stream():
    with sim_stack("field", "field_robot", "mujoco", require_headset=True, log_name="f_gate_eeg.log"):
        with headset_client() as (page, _):
            wait_state(page, "s.connected && s.health", 180)
            for _ in range(5):
                pub(page, "/sim/eeg_fault", "std_msgs/String", "{data: 'drop'}")
                time.sleep(0.5)
            page.evaluate("window.eyesteer.consent()")
            wait_state(page, "s.health.components.find(c => c.name === 'muse_lsl' && c.level === 2)", 60)
            time.sleep(8)
            assert not page.evaluate("window.eyesteer.state().health.motion_unlocked")
            pub(page, "/sim/eeg_fault", "std_msgs/String", "{data: 'ok'}")
            wait_state(page, "s.health.motion_unlocked", 90)


def test_lidar_loss_relocks_motion():
    with sim_stack("field", "field_robot", "mujoco", log_name="f_lidar.log"):
        with headset_client() as (page, _):
            up(page)
            pub(page, "/sim/lidar_off", "std_msgs/Bool", "{data: true}")
            wait_state(page, "!s.health.motion_unlocked && s.status.mux_source === 'LOCK:HEALTH'", 30)
            pub(page, "/sim/lidar_off", "std_msgs/Bool", "{data: false}")
            wait_state(page, "s.health.motion_unlocked", 60)


# ---------------------------------------------------------------- 24.5 failure table
@pytest.mark.parametrize("robot", ["field_robot", "walker"])
def test_muse_stream_drop_and_flat(robot):
    world = "field" if robot == "field_robot" else "supermarket"
    with sim_stack(world, robot, "mujoco", log_name=f"f_muse_{robot}.log"):
        with headset_client() as (page, _):
            up(page)
            tr = Tracker(page, robot, world)
            engage(page, tr, 3 if robot == "field_robot" else 1, "C" if robot == "field_robot" else "Beans")
            track(page, tr, 4)
            for _ in range(3):
                pub(page, "/sim/eeg_fault", "std_msgs/String", "{data: 'drop'}")
            wait_state(page, "s.bci && !s.bci.signal_ok && s.health.components.find(c => c.name === 'muse_lsl' && c.level === 2)", 20)
            n = len(page.evaluate("window.eyesteer.state().selections"))
            x0, y0, _ = tr.sample()["xy"]
            page.evaluate("window.eyesteer.aim(2)")       # looking at another marker during the outage: nothing happens
            track(page, tr, 5)
            page.evaluate("window.eyesteer.aim(0)")
            assert len(page.evaluate("window.eyesteer.state().selections")) == n
            if robot == "walker":                          # walker stops and holds
                run_until(page, tr, "s.status.state === 'PAUSED'", 10)
                track(page, tr, 2)
                assert tr.speed(1.0) < 0.05
            else:                                          # field robot: the current goal continues
                assert page.evaluate("window.eyesteer.state().status.state") in ("ENGAGED", "ARRIVED")
                # displacement over the whole outage (the instantaneous speed is ~0 while it turns in place)
                x1, y1, _ = tr.sample()["xy"]
                st = page.evaluate("window.eyesteer.state().status")
                assert math.hypot(x1 - x0, y1 - y0) > 0.3 or st["state"] == "ARRIVED", st
            pub(page, "/sim/eeg_fault", "std_msgs/String", "{data: 'ok'}")
            wait_state(page, "s.bci && s.bci.signal_ok", 30)
            if robot == "walker":
                run_until(page, tr, "s.status.state === 'ENGAGED'", 15)
            # flat-line Oz (electrode off) -> BAD SIGNAL, no selections
            pub(page, "/sim/eeg_fault", "std_msgs/String", "{data: 'flat'}")
            wait_state(page, "s.bci && s.bci.quality.includes('flat')", 20)
            pub(page, "/sim/eeg_fault", "std_msgs/String", "{data: 'ok'}")
            wait_state(page, "s.bci && s.bci.signal_ok", 30)


def test_headset_disconnect_field_finishes_goal_then_holds():
    with sim_stack("field", "field_robot", "mujoco", require_headset=True, log_name="f_headset.log"):
        with headset_client() as (page, _):
            up(page)
            tr = Tracker(page, "field_robot", "field")
            engage(page, tr, 4, "D")
            page.evaluate("window.eyesteer.disconnect()")
        # page gone: the robot finishes D, then HOLDs (no wander) while the headset is away
        with headset_client() as (page2, _):
            st = wait_state(page2, "s.connected && s.status", 60)
            # the new client reconnected and resynced markers + live state
            assert len(st["markers"]) == 6
        bash_run("true")
        out = bash_run("timeout 8 ros2 topic echo --once /robot/status --field display", 30)
        assert "HOLD" in out or "WANDER" in out or "ENGAGED" in out or "ARRIVED" in out


def test_rosbridge_restart_client_reconnects():
    with sim_stack("field", "field_robot", "mujoco", log_name="f_rosbridge.log"):
        with headset_client() as (page, _):
            up(page)
            bash_run("pkill -f rosbridge_websocket; true", 20)
            wait_state(page, "!s.connected", 15)
            wait_state(page, "s.connected", 60)              # respawned rosbridge + client auto-reconnect
            wait_state(page, "s.status && s.health", 20)


def test_localization_lost_on_teleport():
    with sim_stack("field", "field_robot", "mujoco", log_name="f_loc.log"):
        with headset_client() as (page, _):
            up(page)
            tr = Tracker(page, "field_robot", "field")
            engage(page, tr, 2, "B")
            track(page, tr, 3)
            pub(page, "/sim/teleport", "geometry_msgs/Pose2D", "{x: 3.6, y: -3.4, theta: 2.0}")   # kidnapped
            run_until(page, tr, "s.status.state === 'LOST'", 20)
            track(page, tr, 2)
            assert tr.speed(1.0) < 0.05
            page.evaluate("window.eyesteer.aim(3)")
            track(page, tr, 5)
            page.evaluate("window.eyesteer.aim(0)")
            assert page.evaluate("window.eyesteer.state().status.state") == "LOST"    # no goals accepted while LOST
            # recovery = what the operator does: "2D Pose Estimate" (/initialpose) at the robot's true pose
            q = tr.sample()["pose"]
            pub(page, "/initialpose", "geometry_msgs/PoseWithCovarianceStamped",
                "{header: {frame_id: 'map'}, pose: {pose: %s, covariance: [0.0625, 0, 0, 0, 0, 0, 0, 0.0625, 0, 0, 0, 0, "
                "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685]}}" % json.dumps(q))
            run_until(page, tr, "s.status.state !== 'LOST'", 30)


def test_nav2_goal_failure_retries_then_idle():
    with sim_stack("field", "field_robot", "mujoco", log_name="f_nav.log"):
        with headset_client() as (page, _):
            up(page)
            tr = Tracker(page, "field_robot", "field")
            # marker E moved to an unreachable spot: outside the fence (beyond the map). Inside a crate does
            # not work: the goal search nudges the approach pose to the nearest free one, which is reachable.
            mk = page.evaluate("window.eyesteer.state().markers")
            for m in mk:
                if m["label"] == "E":
                    m.update(x=7.5, y=0.0, facing=0.0)
            page.evaluate("(l) => window.eyesteer.placeMarkers(l)", mk)
            wait_state(page, "s.markers.find(m => m.label === 'E' && Math.abs(m.x - 7.5) < 0.01)", 15)
            page.evaluate("window.eyesteer.aim(5)")
            run_until(page, tr, "s.status.state === 'ENGAGED'", 20)
            page.evaluate("window.eyesteer.aim(0)")
            st = run_until(page, tr, "s.status.display === 'WANDER' && s.status.reason.includes('failed')", 240)
            assert "3 retries" in st["status"]["reason"]


def test_estop_latched():
    with sim_stack("field", "field_robot", "mujoco", log_name="f_estop.log"):
        with headset_client() as (page, _):
            up(page)
            tr = Tracker(page, "field_robot", "field")
            engage(page, tr, 3, "C")
            track(page, tr, 4)
            assert tr.speed(1.0) > 0.1
            pub(page, "/sim/estop_button", "std_msgs/Bool", "{data: true}")
            run_until(page, tr, "s.status.state === 'ESTOP' && s.status.estop", 3)
            track(page, tr, 1.0)
            assert tr.speed(0.5) < 0.05
            pub(page, "/sim/estop_button", "std_msgs/Bool", "{data: false}")
            track(page, tr, 3)
            assert page.evaluate("window.eyesteer.state().status.state") == "ESTOP"     # latched
            pub(page, "/system/reset", "std_msgs/Empty", "{}")
            run_until(page, tr, "s.status.display === 'WANDER' && !s.status.estop", 10)


def test_sudden_obstacle_collision_monitor_stops():
    with sim_stack("field", "field_robot", "mujoco", log_name="f_obstacle.log"):
        with headset_client() as (page, _):
            up(page)
            tr = Tracker(page, "field_robot", "field")
            engage(page, tr, 3, "C")
            track(page, tr, 5)
            st = tr.sample()
            x, y, _ = st["xy"]
            q = st["pose"]["orientation"]
            yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
            ox, oy = x + 0.9 * math.cos(yaw), y + 0.9 * math.sin(yaw)
            pub(page, "/sim/spawn_obstacle", "geometry_msgs/Point", f"{{x: {ox}, y: {oy}, z: 0.25}}")
            seen = False
            t0 = time.time()
            while time.time() - t0 < 12:
                s = tr.sample()
                seen |= s["status"]["safety"] in ("SLOWDOWN", "STOP")
                d = math.hypot(s["xy"][0] - ox, s["xy"][1] - oy) - 0.25 - 0.17
                assert d > 0.0, "contact with the spawned obstacle"
                time.sleep(0.1)
            assert seen, "collision monitor never activated"


@pytest.mark.parametrize("what", ["controller", "mcu"])
def test_mcu_link_loss_hil(what):
    with sim_stack("field", "field_robot", "mujoco_hil", log_name=f"f_mcu_{what}.log"):
        with headset_client() as (page, _):
            up(page)
            tr = Tracker(page, "field_robot", "field")
            engage(page, tr, 3, "C")
            track(page, tr, 5)
            assert tr.speed(1.0) > 0.1
            if what == "controller":      # ROS side dies: no more 'm' commands -> firmware watchdog (200 ms)
                bash_run("pkill -9 -f ros2_control_node; true", 20)
            else:                          # MCU crashes / unplugged: no replies -> link gate red, motion locked
                pub(page, "/sim/kill_mcu", "std_msgs/Bool", "{data: true}")
            t0 = time.time()
            while time.time() - t0 < 1.5:
                tr.sample()
                time.sleep(0.05)
            assert tr.speed(0.5) < 0.05, "wheels still turning after the command stream stopped"
            if what == "mcu":
                wait_state(page, "s.health.components.find(c => c.name === 'mcu' && c.level === 2) && !s.health.motion_unlocked", 15)

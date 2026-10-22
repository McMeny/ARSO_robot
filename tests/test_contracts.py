"""Static contract tests: one per boundary in INTERFACES.md (no ROS needed)."""
import json
import math
import re
import subprocess
import time
from pathlib import Path

import pytest
import yaml

from eyesteer_core import config

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "ros2_ws/src"
NODE = None
for cand in [Path.home() / "AppData/Local/eyesteer_tools/node-v20.18.1-win-x64/node.exe", Path("/usr/bin/node"), Path("/usr/local/bin/node")]:
    if cand.exists():
        NODE = str(cand)


def msg_fields(name):
    p = SRC / "eyesteer_interfaces" / ("msg" if not name.endswith("Srv") else "srv") / f"{name}.msg"
    return {ln.split()[1] for ln in p.read_text(encoding="utf-8").splitlines() if ln.strip() and not ln.startswith("#") and "=" not in ln.split("#")[0]}


# 1. Muse -> LSL
def test_muse_emulator_stream_matches_contract():
    pylsl = pytest.importorskip("pylsl")
    import sys
    sys.path.insert(0, str(SRC / "eyesteer_bci"))
    from eyesteer_bci.muse_emulator import MuseEmulator
    hw = config.hardware()["eeg"]
    spec = hw["tools"][hw["source_tool"]]
    emu = MuseEmulator(freqs={1: 6.923})
    emu.start()
    try:
        import socket
        time.sleep(1.0)
        res = [r for r in pylsl.resolve_byprop("name", spec["stream_name"], 0, 3.0) if r.hostname() == socket.gethostname()]
        assert res, "emulator stream not found"
        info = res[0]
        assert info.type() == spec["stream_type"]
        assert info.channel_count() == len(spec["channels"]) == 5
        assert info.nominal_srate() == 256
        inlet = pylsl.StreamInlet(info)
        full = inlet.info()
        ch = full.desc().child("channels").child("channel")
        labels = []
        for _ in range(5):
            labels.append(ch.child_value("label"))
            ch = ch.next_sibling()
        assert labels == spec["channels"]
        assert hw["channel_map"]["Oz"] == 4 and labels[4].endswith("AUX")
        t0 = time.time()
        n = 0
        ts = []
        while time.time() - t0 < 3.0:
            c, t = inlet.pull_chunk(timeout=0.2)
            n += len(c)
            ts += t
        rate = (len(ts) - 1) / (ts[-1] - ts[0])
        assert abs(rate - 256) < 3
    finally:
        emu.running = False


# 3. headset <-> rosbridge
def test_webxr_topics_match_interfaces():
    js = (ROOT / "webxr/js/ros.js").read_text(encoding="utf-8") + (ROOT / "webxr/js/app.js").read_text(encoding="utf-8")
    doc = (ROOT / "INTERFACES.md").read_text(encoding="utf-8")
    used = set(re.findall(r'"(/[a-z_/]+)", "([a-z_]+/[A-Za-z]+)"', js))
    assert len(used) >= 12
    for topic, typ in used:
        assert f"`{topic}`" in doc, f"{topic} missing from INTERFACES.md"
        pkg, name = typ.split("/")
        if pkg == "eyesteer_interfaces":
            assert (SRC / "eyesteer_interfaces/msg" / f"{name}.msg").exists() or (SRC / "eyesteer_interfaces/srv" / f"{name}.srv").exists(), typ
    # fields the app reads from RobotStatus / BciStatus exist in the message definitions
    for f in ["display", "queue_labels", "mux_source", "safety", "goal_id", "goal_label", "motion_locked", "estop", "deadman_locked", "reason"]:
        assert f in msg_fields("RobotStatus")
    for f in ["labels", "rhos", "threshold", "quality", "sample_rate", "dwell", "no_control", "source", "table_hash"]:
        assert f in msg_fields("BciStatus")
    for f in ["refresh_hz", "frame_ms_mean", "dropped_pct", "table_hash", "calibrated", "calibration_residual", "markers_placed", "consent", "mode"]:
        assert f in msg_fields("XrTiming")
    from_js = re.search(r"export function xrTimingMsg\(t\) \{\s*return \{(.*?)\};", (ROOT / "webxr/js/messages.js").read_text(encoding="utf-8"), re.S).group(1)
    assert set(re.findall(r"(\w+):", from_js)) == msg_fields("XrTiming")


# 4. anchors -> map: Python and JS calibration math agree
@pytest.mark.skipif(NODE is None, reason="node not available")
def test_calibration_python_js_parity():
    from eyesteer_core.xr_calibration import XrCalibration, headset_to_map, map_to_headset, solve
    truth = XrCalibration(0.9, -1.2, 2.5, 0.03, 0.0)
    pose = (0.7, -0.4, 1.1)
    dec = config.robot("walker")["calibration_decals"]

    def dmap(d):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return (pose[0] + c * d["x"] - s * d["y"], pose[1] + s * d["x"] + c * d["y"], dec["plate_height"])
    hf, hr = map_to_headset(dmap(dec["front"]), truth), map_to_headset(dmap(dec["rear"]), truth)
    py = solve(hf, hr, pose, (dec["front"]["x"], dec["front"]["y"]), (dec["rear"]["x"], dec["rear"]["y"]), dec["plate_height"])
    script = (f"import {{ solve, headsetToMap }} from '{(ROOT / 'webxr/js/calib.js').as_uri()}';"
              f"const c = solve({list(hf)}, {list(hr)}, {list(pose)}, [{dec['front']['x']}, {dec['front']['y']}], "
              f"[{dec['rear']['x']}, {dec['rear']['y']}], {dec['plate_height']});"
              f"console.log(JSON.stringify({{...c, p: headsetToMap([0.3, 1.4, -2.0], c)}}));")
    out = subprocess.run([NODE, "--input-type=module", "-e", script], capture_output=True, text=True, timeout=30)
    js = json.loads(out.stdout)
    assert abs(js["yaw"] - py.yaw) < 1e-9 and abs(js["tx"] - py.tx) < 1e-9 and abs(js["ty"] - py.ty) < 1e-9
    assert abs(js["floorHeight"] - py.floor_height) < 1e-9
    pp = headset_to_map((0.3, 1.4, -2.0), py)
    assert all(abs(a - b) < 1e-9 for a, b in zip(js["p"], pp))
    assert abs(py.yaw - truth.yaw) < 1e-9 and py.residual < 1e-6


# 5. one frequency table everywhere
def test_single_frequency_table():
    import sys
    sys.path.insert(0, str(SRC / "eyesteer_behavior"))
    gen = json.loads((ROOT / "config/generated/frequencies.json").read_text(encoding="utf-8"))
    from compute_frequencies import compute, configured_refresh, load_rules
    fresh = compute(configured_refresh(), load_rules())
    assert gen["hash"] == fresh["hash"], "run `make generate`"
    assert config.frequency_table()["hash"] == gen["hash"]
    from eyesteer_core.world import load_markers
    for name in ("field_default", "supermarket_default"):
        mk = load_markers(name, gen)
        cancel = [m for m in mk if m.cancel]
        assert len(cancel) == 1 and cancel[0].frequency == max(m.frequency for m in mk)
    src = (SRC / "eyesteer_behavior/eyesteer_behavior/web_server.py").read_text(encoding="utf-8")
    assert "config.frequency_table()" in src     # the web server serves the same table


# 6. mode_manager -> Nav2
@pytest.mark.parametrize("robot", ["field_robot", "walker"])
@pytest.mark.parametrize("variant", ["sim", "real"])
def test_nav2_params_match_robot_yaml(robot, variant):
    p = yaml.safe_load((SRC / f"eyesteer_bringup/params/generated/{robot}_nav2_{variant}.yaml").read_text(encoding="utf-8"))
    r = config.robot(robot)
    gc = p["controller_server"]["ros__parameters"]["general_goal_checker"]
    assert gc["xy_goal_tolerance"] == r["nav"]["goal_xy_tolerance"]
    assert abs(gc["yaw_goal_tolerance"] - math.radians(r["nav"]["goal_yaw_tolerance_deg"])) < 1e-4
    lc = p["local_costmap"]["local_costmap"]["ros__parameters"]
    fp = config.geometry(robot)["footprint"]
    if fp["type"] == "polygon":
        assert json.loads(lc["footprint"]) == fp["polygon"]
    else:
        assert abs(lc["robot_radius"] - (fp["radius"] + fp["padding"])) < 1e-9
    v = p["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"]
    assert v <= r["limits"]["v_max"]
    assert p["bt_navigator"]["ros__parameters"]["global_frame"] == "map"


# 7. velocity chain
def test_velocity_chain_topics():
    tm = yaml.safe_load((SRC / "eyesteer_bringup/params/generated/twist_mux.yaml").read_text(encoding="utf-8"))
    from eyesteer_core.control.mux_rules import twist_mux_yaml
    assert tm == twist_mux_yaml()
    cm = yaml.safe_load((SRC / "eyesteer_bringup/params/generated/walker_nav2_sim.yaml").read_text(encoding="utf-8"))["collision_monitor"]["ros__parameters"]
    assert cm["cmd_vel_in_topic"] == "cmd_vel_mux" and cm["cmd_vel_out_topic"] == "cmd_vel_cm"
    launch = (SRC / "eyesteer_bringup/launch/stack.launch.py").read_text(encoding="utf-8")
    assert '("cmd_vel_out", "cmd_vel_mux")' in launch and '("cmd_vel", "cmd_vel_nav_raw")' in launch
    safety = (SRC / "eyesteer_behavior/eyesteer_behavior/safety_node.py").read_text(encoding="utf-8")
    for t in ['"/cmd_vel_nav_raw"', '"/cmd_vel_nav"', '"/cmd_vel_cm"', '"/cmd_vel"', '"/lock/estop"', '"/lock/deadman"']:
        assert t in safety
    bridge = (SRC / "eyesteer_mujoco/eyesteer_mujoco/mujoco_bridge.py").read_text(encoding="utf-8")
    assert '"/cmd_vel", self.on_cmd' in bridge
    hw = (SRC / "eyesteer_bringup/launch/hw_interface.py").read_text(encoding="utf-8")
    assert '("/diff_cont/cmd_vel_unstamped", "/cmd_vel")' in hw and '("/diff_cont/odom", "/odom")' in hw
    for lf in ("sim.launch.py", "real.launch.py"):
        assert "hw_interface_actions(" in (SRC / f"eyesteer_bringup/launch/{lf}").read_text(encoding="utf-8")


# 8. ros2_control <-> MCU
@pytest.mark.parametrize("robot", ["field_robot", "walker"])
def test_hw_interface_params_match_firmware(robot):
    urdf = (SRC / f"eyesteer_description/urdf/{robot}_real.urdf").read_text(encoding="utf-8")
    h = (ROOT / f"firmware/include/robot_constants_{robot}.h").read_text(encoding="utf-8")
    hdr = dict(re.findall(r"#define (\w+) (\S+)", h))
    param = dict(re.findall(r'<param name="(\w+)">([^<]+)</param>', urdf))
    assert param["loop_rate"] == hdr["PID_RATE_HZ"]
    assert param["baud_rate"] == hdr["BAUD_RATE"]
    assert param["enc_counts_per_rev"] == hdr["ENCODER_CPR"]
    assert int(hdr["WATCHDOG_MS"]) == 200
    for k in ("pid_p", "pid_d", "pid_i", "pid_o"):
        assert param[k] == hdr["PID_K" + k[-1].upper()]
    assert "diffdrive_arduino/DiffDriveArduinoHardware" in urdf


# 9. LiDAR
def test_lidar_contract():
    hw = config.hardware()["lidar"]
    assert hw["frame_id"] == "laser_frame" and hw["rate_hz"] == 10
    bridge = (SRC / "eyesteer_mujoco/eyesteer_mujoco/mujoco_bridge.py").read_text(encoding="utf-8")
    assert '"laser_frame"' in bridge and "range_min, sc.range_max = 0.15, 12.0" in bridge
    assert hw["range_min"] == 0.15 and hw["range_max"] == 12.0
    real = (SRC / "eyesteer_bringup/launch/real.launch.py").read_text(encoding="utf-8")
    assert '"frame_id": "laser_frame"' in real and '("scan", "scan_raw")' in real


# 11. clock
def test_clock_settings_in_launch_files():
    sim = (SRC / "eyesteer_bringup/launch/sim.launch.py").read_text(encoding="utf-8")
    real = (SRC / "eyesteer_bringup/launch/real.launch.py").read_text(encoding="utf-8")
    assert 'st = {"use_sim_time": True}' in sim and '"use_sim_time": "true"' in sim
    assert 'st = {"use_sim_time": False}' in real and '"use_sim_time": "false"' in real
    assert '"use_sim_time": "false"' not in sim and "True}" not in real.split("st =")[1].split("\n")[0]

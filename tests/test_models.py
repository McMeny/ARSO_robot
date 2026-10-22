"""M1: generated models are valid, consistent with each other, and drive forward."""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from generate_models import ROBOTS, build_urdf, load_hw, load_robot

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / "ros2_ws/src/eyesteer_description/urdf"


def urdf_tree(text):
    root = ET.fromstring(text)
    links = {l.get("name") for l in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}
    return root, links, joints


@pytest.mark.parametrize("name", ROBOTS)
def test_urdf_connected_rooted(name):
    _, links, joints = urdf_tree((DESC / f"{name}.urdf").read_text())
    children = {j.find("child").get("link") for j in joints.values()}
    parents = {j.find("parent").get("link") for j in joints.values()}
    roots = links - children
    assert roots == {"base_footprint"}
    assert parents | children <= links
    assert "laser_frame" in links and "imu_link" in links and "camera_optical_frame" in links
    assert "map" not in links and "odom" not in links
    if name == "walker":
        assert {"handle_left_ft", "handle_right_ft", "grip_left", "grip_right"} <= links


@pytest.mark.parametrize("name", ROBOTS)
def test_wheel_joints_continuous_and_match_controllers(name):
    _, _, joints = urdf_tree((DESC / f"{name}.urdf").read_text())
    ctrl = yaml.safe_load((ROOT / f"ros2_ws/src/eyesteer_bringup/config/generated/{name}_controllers.yaml").read_text())
    p = ctrl["diff_cont"]["ros__parameters"]
    for j in p["left_wheel_names"] + p["right_wheel_names"]:
        assert joints[j].get("type") == "continuous"
        assert joints[j].find("axis").get("xyz") == "0 1 0"
    r = load_robot(name)
    assert p["wheel_radius"] == r["wheels"]["radius"] and p["wheel_separation"] == r["wheels"]["separation"]


@pytest.mark.parametrize("name", ROBOTS)
def test_inertias_valid(name):
    root, _, _ = urdf_tree((DESC / f"{name}.urdf").read_text())
    for inert in root.iter("inertia"):
        ixx, iyy, izz = (float(inert.get(k)) for k in ("ixx", "iyy", "izz"))
        assert ixx > 0 and iyy > 0 and izz > 0
        assert ixx + iyy >= izz - 1e-12 and iyy + izz >= ixx - 1e-12 and ixx + izz >= iyy - 1e-12
    for m in root.iter("mass"):
        assert float(m.get("value")) > 0


@pytest.mark.parametrize("name", ROBOTS)
def test_real_urdf_has_diffdrive_arduino(name):
    text = (DESC / f"{name}_real.urdf").read_text()
    assert "diffdrive_arduino/DiffDriveArduinoHardware" in text
    hw = load_hw()
    assert f'<param name="loop_rate">{hw["mcu"]["pid_rate_hz"]}</param>' in text
    assert f'<param name="enc_counts_per_rev">{hw["encoders"]["counts_per_rev"]}</param>' in text
    assert "ros2_control" not in (DESC / f"{name}.urdf").read_text()
    xacro = (DESC / f"{name}.urdf.xacro").read_text()
    assert "$(arg backend)" in xacro


@pytest.mark.parametrize("name", ROBOTS)
def test_xacro_and_plain_agree(name):
    """Substituting the xacro properties by hand must reproduce the plain URDF numbers."""
    r, hw = load_robot(name), load_hw()
    x = build_urdf(r, hw, xacro=True)
    props = dict(re.findall(r'<xacro:property name="(\w+)" value="([^"]+)"/>', x))

    def ev(m):
        expr = m.group(1)
        for k, v in sorted(props.items(), key=lambda kv: -len(kv[0])):
            expr = re.sub(rf"\b{k}\b", v, expr)
        return f"{eval(expr):.6f}".rstrip("0").rstrip(".")
    sub = re.sub(r"\$\{([^}]+)\}", ev, x)
    nums_x = [float(v) for v in re.findall(r'(?:xyz|size|radius|length)="([-\d. ]+)"', sub) for v in v.split()]
    plain = build_urdf(r, hw, xacro=False)
    nums_p = [float(v) for v in re.findall(r'(?:xyz|size|radius|length)="([-\d. ]+)"', plain) for v in v.split()]
    assert np.allclose(nums_x, nums_p, atol=1e-6)


def _mj_urdf(name):
    text = (DESC / f"{name}.urdf").read_text()
    text = text.replace(f'<robot name="{name}">',
                        f'<robot name="{name}"><mujoco><compiler fusestatic="false" discardvisual="true" balanceinertia="true"/></mujoco>', 1)
    m = mujoco.MjModel.from_xml_string(text)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


@pytest.mark.parametrize("name", ROBOTS)
def test_urdf_mjcf_consistency(name):
    mu, du = _mj_urdf(name)
    mm = mujoco.MjModel.from_xml_path(str(ROOT / f"config/generated/mjcf/{name}.xml"))
    dm = mujoco.MjData(mm)
    mujoco.mj_forward(mm, dm)
    base_u = du.xpos[mu.body("base_footprint").id] if mujoco.mj_name2id(mu, mujoco.mjtObj.mjOBJ_BODY, "base_footprint") >= 0 else np.zeros(3)
    base_m = dm.xpos[mm.body("base_footprint").id]
    for body in ("base_link", "left_wheel_link", "right_wheel_link"):
        pu = du.xpos[mu.body(body).id] - base_u
        pm = dm.xpos[mm.body(body).id] - base_m
        assert np.allclose(pu, pm, atol=1e-3), (body, pu, pm)
    pu = du.xpos[mu.body("laser_frame").id] - base_u
    pm = dm.site_xpos[mm.site("laser_frame").id] - base_m
    assert np.allclose(pu, pm, atol=1e-3)
    pu = du.xpos[mu.body("imu_link").id] - base_u
    pm = dm.site_xpos[mm.site("imu_link").id] - base_m
    assert np.allclose(pu, pm, atol=1e-3)
    r = load_robot(name)["wheels"]["radius"]
    assert abs(mm.geom("left_wheel").size[0] - r) < 1e-3


@pytest.mark.parametrize("name", ROBOTS)
def test_equal_positive_wheel_commands_move_plus_x(name):
    """Flipped-axis guard (the #1 bug)."""
    m = mujoco.MjModel.from_xml_path(str(ROOT / f"config/generated/mjcf/{name}.xml"))
    d = mujoco.MjData(m)
    for _ in range(300):
        mujoco.mj_step(m, d)
    x0 = d.qpos[:2].copy()
    d.ctrl[:] = [3.0, 3.0]
    for _ in range(1000):
        mujoco.mj_step(m, d)
    dx, dy = d.qpos[:2] - x0
    assert dx > 0.1 and abs(dy) < 0.02 * dx


def test_firmware_header_generated():
    for n in ROBOTS:
        h = (ROOT / f"firmware/include/robot_constants_{n}.h").read_text()
        assert "#define WATCHDOG_MS 200" in h
        assert "#define PID_RATE_HZ" in h and "EYESTEER_BOARD_MEGA2560" in h

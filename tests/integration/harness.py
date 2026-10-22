"""Integration harness: start/stop the ROS sim stack (natively on Linux / the Jetson, through WSL on a
Windows dev machine) and drive the headless headset client (Playwright, desktop-preview mode).

The headless headset client is the real WebXR app served by the real web server, connected to the
real rosbridge; the only test hook is ?emulate_refresh=90 (a desktop browser runs at 60 Hz).
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ON_WINDOWS = sys.platform == "win32"
WSL_DISTRO = os.environ.get("EYESTEER_WSL_DISTRO", "Ubuntu-22.04")
CLEAN = ("pkill -9 -f 'eyesteer_|nav2_|rosbridge|twist_mux|ros2_control|controller_manager|robot_state_publisher|"
         "slam_toolbox|lifecycle_manager|amcl|map_server|rosapi|ros2 launch' 2>/dev/null; rm -f /tmp/eyesteer_mcu*; true")


def linux_root() -> str:
    if not ON_WINDOWS:
        return str(ROOT)
    drive, rest = str(ROOT).split(":", 1)
    return f"/mnt/{drive.lower()}" + rest.replace("\\", "/")


def bash(cmd: str, **kw) -> subprocess.Popen:
    full = f"cd {linux_root()} && source deploy/env.sh && {cmd}"
    if ON_WINDOWS:
        return subprocess.Popen(["wsl", "-d", WSL_DISTRO, "--", "bash", "-c", full], **kw)
    return subprocess.Popen(["bash", "-c", full], **kw)


def bash_run(cmd: str, timeout: float = 120) -> str:
    p = bash(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out, _ = p.communicate(timeout=timeout)
    return out


def ros_available() -> bool:
    if ON_WINDOWS:
        return shutil.which("wsl") is not None and "OK" in bash_run("ros2 pkg prefix eyesteer_bringup >/dev/null && echo OK", 60)
    return shutil.which("ros2") is not None or Path("/opt/ros").exists()


@contextmanager
def sim_stack(world="field", robot="field_robot", backend="mujoco", require_headset=True, eeg="synthetic",
              extra="", log_name=None):
    bash_run(CLEAN, 30)
    log = ROOT / "outputs" / "logs" / (log_name or f"it_{robot}_{backend}.log")
    log.parent.mkdir(parents=True, exist_ok=True)
    f = open(log, "w")
    cmd = (f"ros2 launch eyesteer_bringup sim.launch.py world:={world} robot:={robot} backend:={backend} eeg:={eeg} "
           f"require_headset:={'true' if require_headset else 'false'} {extra}")
    proc = bash(cmd, stdout=f, stderr=subprocess.STDOUT)
    try:
        yield proc, log
    finally:
        bash_run(CLEAN, 30)
        try:
            proc.kill()
        except Exception:
            pass
        f.close()


def ros_cmd(cmd: str, timeout: float = 30) -> str:
    """Run a one-shot ros2 CLI command (topic pub/echo, service call) against the running stack."""
    return bash_run(f"timeout {int(timeout)} {cmd}", timeout + 15)


@contextmanager
def headset_client(url_extra="", emulate_refresh=90, headless=True):
    from playwright.sync_api import sync_playwright
    cert = (ROOT / "deploy/certs/eyesteer.pem").exists()
    scheme, ws = ("https", "wss") if cert else ("http", "ws")
    url = f"{scheme}://localhost:8443/index.html?rosbridge={ws}://localhost:9090&emulate_refresh={emulate_refresh}{url_extra}"
    import ssl
    import urllib.request
    ctx = ssl._create_unverified_context()
    t0 = time.time()
    while True:                       # wait for the web server of the freshly launched stack
        try:
            urllib.request.urlopen(f"{scheme}://localhost:8443/config/frequencies.json", context=ctx, timeout=2).read()
            break
        except Exception:
            if time.time() - t0 > 180:
                raise TimeoutError("web server did not come up on :8443")
            time.sleep(1.0)
    with sync_playwright() as p:
        b = p.chromium.launch(headless=headless, args=["--ignore-certificate-errors"])
        ctx = b.new_context(ignore_https_errors=True, viewport={"width": 1400, "height": 900})
        page = ctx.new_page()
        logs = []
        page.on("console", lambda m: logs.append(m.text))
        page.goto(url)
        page.wait_for_function("window.eyesteer !== undefined", timeout=30000)
        try:
            yield page, logs
        finally:
            b.close()


def wait_state(page, js_predicate: str, timeout: float = 60.0, poll: float = 0.25):
    """Wait until predicate(state) is truthy; returns the state."""
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        last = page.evaluate(f"(() => {{ const s = window.eyesteer.state(); return ({js_predicate}) ? s : null; }})()")
        if last:
            return last
        time.sleep(poll)
    st = page.evaluate("window.eyesteer.state()")
    raise TimeoutError(f"timeout waiting for: {js_predicate}\nstate: status={st.get('status') and st['status'].get('display')} "
                       f"connected={st.get('connected')} markers={len(st.get('markers') or [])} "
                       f"health={st.get('health') and [(c['name'], c['level'], c['message']) for c in st['health']['components'] if c['level']]}")

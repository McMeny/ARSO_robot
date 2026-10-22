"""M4: the MCU emulator runs the firmware core behind a pty; protocol round trip as diffdrive_arduino
speaks it, watchdog, e-stop, and motor_test.py against the emulator. Linux only (pty, g++)."""
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(not hasattr(os, "openpty") or sys.platform == "win32", reason="needs a pty (Linux)")


@pytest.fixture
def emu(tmp_path):
    sys.path.insert(0, str(ROOT / "firmware/emulator"))
    from mcu_emulator import McuEmulator
    e = McuEmulator("field_robot", str(tmp_path / "mcu"))
    e.start()
    yield e
    e.stop()


def _tx(fd, cmd: str, timeout=1.0) -> str:
    os.write(fd, cmd.encode())
    buf = b""
    t = time.time()
    while not buf.endswith(b"\n") and time.time() - t < timeout:
        try:
            buf += os.read(fd, 64)
        except BlockingIOError:
            time.sleep(0.001)
    return buf.decode()


def _open(path):
    import termios
    import tty
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def test_diffdrive_arduino_protocol_roundtrip(emu):
    fd = _open(emu.link)
    assert _tx(fd, "\r") == "\r\n"                       # send_empty_msg
    assert _tx(fd, "u 20:12:0:50\r") == "OK\r\n"         # set_pid_values on activate
    assert _tx(fd, "e\r") == "0 0\r\n"
    for _ in range(40):                                   # 30 Hz write()/read() like ros2_control
        assert _tx(fd, "m 15 15\r") == "OK\r\n"
        time.sleep(1 / 30)
    l, r = map(int, _tx(fd, "e\r").split())
    assert l > 200 and r > 200 and abs(l - r) < 0.15 * l   # wheels turned forward together
    os.close(fd)


def test_watchdog_stops_motors(emu):
    fd = _open(emu.link)
    for _ in range(10):
        _tx(fd, "m 15 15\r")
        time.sleep(1 / 30)
    assert emu.core.pwm()[0] != 0
    time.sleep(0.35)                                       # > 200 ms without 'm'
    assert emu.core.pwm() == (0, 0)
    assert _tx(fd, "s\r").startswith("S 0 0 1")            # watchdog tripped
    os.close(fd)


def test_estop_latch(emu):
    fd = _open(emu.link)
    emu.estop_pressed = True
    time.sleep(0.1)
    _tx(fd, "m 15 15\r")
    time.sleep(0.1)
    assert emu.core.pwm() == (0, 0)
    assert _tx(fd, "x\r") == "ESTOP\r\n"
    emu.estop_pressed = False
    time.sleep(0.1)
    assert _tx(fd, "x\r") == "OK\r\n"
    os.close(fd)


def test_motor_test_tool_passes_against_emulator(emu):
    r = subprocess.run([sys.executable, str(ROOT / "tools/motor_test.py"), "--device", emu.link, "--secs", "0.8"],
                       capture_output=True, text=True, timeout=60)
    print(r.stdout)
    assert "RESULT: PASS" in r.stdout

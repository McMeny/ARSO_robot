"""mcu_bridge_node: serial proxy so ros2_control (diffdrive_arduino) and eyesteer share ONE MCU port.

  /dev/eyesteer_mcu (real) or the emulator pty (HIL)  <-->  mcu_bridge  <-->  pty for diffdrive_arduino
                                                             |  (default /tmp/eyesteer_mcu_ros)
                                                             +-> /mcu/alive, /mcu/status, /mcu/estop,
                                                                 walker: /handle_force/{left,right}, /grip

diffdrive_arduino's transactions ("e", "m", "u", "\\r") are forwarded unchanged, one line at a time,
with its reply returned verbatim. Between them, at 20 Hz, the bridge interleaves the eyesteer
extension queries "s" (status) and "h" (walker sensors). This is cleaner than forking the hardware
interface (DECISIONS.md). If the MCU stops answering for 0.5 s, /mcu/alive goes false (health gate
red -> motion lock); the MCU's own 200 ms watchdog has already stopped the motors.
"""
from __future__ import annotations

import os
import select
import threading
import time
import tty

from eyesteer_behavior import _root  # noqa: F401

import rclpy
import serial
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String


class McuBridge(Node):
    def __init__(self):
        super().__init__("mcu_bridge")
        p = self.declare_parameter
        self.device = p("device", "/dev/eyesteer_mcu").value
        self.baud = p("baud", 57600).value
        self.ros_link = p("ros_link", "/tmp/eyesteer_mcu_ros").value
        self.walker = p("walker", False).value
        self.reset_wait = p("reset_wait", 2.0).value      # AVR boards reboot when the port opens
        self.lock = threading.Lock()
        self.ser = None
        self.last_reply = 0.0
        self.alive = False
        self.pub_alive = self.create_publisher(Bool, "/mcu/alive", 10)
        self.pub_status = self.create_publisher(String, "/mcu/status", 10)
        self.pub_estop = self.create_publisher(Bool, "/mcu/estop", 10)
        if self.walker:
            self.pub_fl = self.create_publisher(WrenchStamped, "/handle_force/left", 10)
            self.pub_fr = self.create_publisher(WrenchStamped, "/handle_force/right", 10)
            self.pub_grip = self.create_publisher(Bool, "/grip", 10)
        self.create_subscription(Empty, "/system/reset", lambda _: self.transact(b"x\r"), 10)
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        tty.setraw(self.master)
        if os.path.islink(self.ros_link) or os.path.exists(self.ros_link):
            os.unlink(self.ros_link)
        os.symlink(os.ttyname(self.slave), self.ros_link)
        self.open_serial()
        threading.Thread(target=self.proxy_loop, daemon=True).start()
        self.create_timer(0.05, self.poll)
        self.get_logger().info(f"mcu_bridge: {self.device} @ {self.baud} <-> {self.ros_link} (diffdrive_arduino)")

    def open_serial(self):
        while rclpy.ok():
            try:
                self.ser = serial.Serial(self.device, self.baud, timeout=0.1, write_timeout=0.1)
                time.sleep(self.reset_wait)
                self.ser.reset_input_buffer()
                return
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f"cannot open {self.device}: {e}; retrying", throttle_duration_sec=5.0)
                time.sleep(1.0)

    def transact(self, line: bytes, timeout: float = 0.1) -> bytes:
        """One request/response with the MCU (serialized)."""
        with self.lock:
            try:
                self.ser.write(line)
                self.ser.timeout = timeout
                reply = self.ser.readline()
            except (serial.SerialException, OSError):
                reply = b""
        if reply.endswith(b"\n"):
            self.last_reply = time.monotonic()
        return reply

    def proxy_loop(self):
        buf = b""
        while rclpy.ok():
            r, _, _ = select.select([self.master], [], [], 0.05)
            if not r:
                continue
            try:
                buf += os.read(self.master, 256)
            except OSError:
                continue
            while b"\r" in buf:
                line, buf = buf.split(b"\r", 1)
                buf = buf.lstrip(b"\n")
                reply = self.transact(line + b"\r")
                if reply:
                    os.write(self.master, reply)

    def poll(self):
        st = self.transact(b"s\r", 0.05).decode(errors="replace").strip()
        if st.startswith("S "):
            f = st.split()
            self.pub_status.publish(String(data=f"estop_latched={f[1]} estop_input={f[2]} watchdog_tripped={f[3]} moving={f[4]}"))
            self.pub_estop.publish(Bool(data=f[1] == "1"))
        if self.walker:
            h = self.transact(b"h\r", 0.05).decode(errors="replace").strip().split()
            if len(h) == 4 and h[0] == "H":
                now = self.get_clock().now().to_msg()
                for pub, val, frame in ((self.pub_fl, h[1], "handle_left_ft"), (self.pub_fr, h[2], "handle_right_ft")):
                    w = WrenchStamped()
                    w.header.stamp, w.header.frame_id = now, frame
                    w.wrench.force.x = int(val) / 10.0
                    pub.publish(w)
                self.pub_grip.publish(Bool(data=h[3] == "1"))
        self.alive = time.monotonic() - self.last_reply < 0.5
        self.pub_alive.publish(Bool(data=self.alive))


def main():
    rclpy.init()
    n = McuBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

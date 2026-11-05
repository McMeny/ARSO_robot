"""The walker's grip state - STOP / WALK / STEER - from the two handles, at 50 Hz.

Sources (parameter `source`):
  quest     the Quest controllers' grip triggers (analog 0..1), sent by the headset through the
            web bridge on /walker/grip_raw. The sim stand-in for the handle FSRs.
  keyboard  /walker/grip_keys, from `ros2 run my_robot_autonomy walker_keys` in a terminal
            (or the headset page's keys in desktop mode, which also go to /walker/grip_raw).
  arduino   a serial line "left,right\\n" from the Arduino's two FSRs (raw ADC counts). The
            Arduino stops the motors itself in firmware; this node mirrors that state for ROS.
            With four values, "grip_left,grip_right,force_left,force_right\\n", the last two are the
            handle load cells in newtons, published on /walker/handle_force (ASSIST).
  operator  no handles: WALK while the operator page says Start walking (no STEER). The old
            "both hands always on the walker" mode.

Also: /walker/force_stop (Bool, from the headset - room edge reached, or the user too far from
the handles) and the operator's Stop walking force STOP. Raw readings older than
`raw_timeout` count as released, so a lost headset or cable stops the walker.

Calibration: services ~/calibrate_relaxed and ~/calibrate_firm (std_srvs/Trigger) each record
the last second of readings; once both are done the thresholds are set between them.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from my_robot_autonomy.grip_logic import GripLogic, NAMES, STOP, WALK
from walker_interfaces.msg import GripState, HandleForce


class GripStateNode(Node):
    def __init__(self):
        super().__init__('grip_state')
        p = self.declare_parameter
        self.source = p('source', 'quest').value
        self.logic = GripLogic(
            light_threshold=p('light_threshold', 0.2).value,
            firm_threshold=p('firm_threshold', 0.6).value,
            hysteresis=p('hysteresis', 0.05).value,
            release_grace_s=p('release_grace_s', 0.2).value,
            steer_hold_s=p('steer_hold_s', 0.5).value,
            one_handed=p('one_handed', False).value,
            hand=p('hand', 'right').value)
        self.raw_timeout = p('raw_timeout', 0.3).value
        self.adc_max = p('adc_max', 1023.0).value
        self.operator_enabled = p('start_enabled', False).value

        self.left = self.right = 0.0
        self.raw_time = -math.inf
        self.forced = False
        self.history = []          # (t, left, right) over the last second, for calibration
        self.calibration = {}
        self.serial = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(GripState, 'walker/grip_state', 10)
        self.force_pub = self.create_publisher(HandleForce, 'walker/handle_force', 10)
        if self.source == 'quest':
            self.create_subscription(GripState, 'walker/grip_raw', self.on_raw, 10)
        elif self.source == 'keyboard':
            self.create_subscription(GripState, 'walker/grip_keys', self.on_raw, 10)
        elif self.source == 'arduino':
            self.open_serial(p('serial_port', '/dev/ttyACM0').value, p('baud', 115200).value)
        elif self.source != 'operator':
            raise ValueError(f'unknown grip source "{self.source}" (quest, keyboard, arduino, operator)')
        self.create_subscription(Bool, 'walker/force_stop', self.on_force_stop, 10)
        self.create_subscription(Bool, 'walker_enabled', self.on_enabled, latched)
        self.create_service(Trigger, '~/calibrate_relaxed', lambda req, res: self.calibrate('relaxed', res))
        self.create_service(Trigger, '~/calibrate_firm', lambda req, res: self.calibrate('firm', res))
        self.create_timer(0.02, self.update)
        self.last_state = None
        self.get_logger().info(f'Grip source: {self.source}')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------- inputs ----------

    def on_raw(self, msg):
        self.left, self.right = float(msg.left), float(msg.right)
        self.raw_time = self.now()

    def on_force_stop(self, msg):
        if msg.data and not self.forced:
            self.get_logger().warn('Forced STOP (room edge, or the user is too far from the handles)')
        self.forced = msg.data

    def on_enabled(self, msg):
        self.operator_enabled = msg.data

    def open_serial(self, port, baud):
        try:
            import serial
            self.serial = serial.Serial(port, baud, timeout=0)
            self.serial_buffer = b''
            self.get_logger().info(f'Reading the handle FSRs from {port}')
        except Exception as e:  # no device: stay in STOP, keep retrying is the Arduino's job
            self.get_logger().error(f'Cannot open {port}: {e} - the walker stays in STOP')

    def read_serial(self):
        if self.serial is None:
            return
        try:
            self.serial_buffer += self.serial.read(256)
        except Exception as e:
            self.get_logger().error(f'Serial read failed: {e}')
            self.serial = None
            return
        *lines, self.serial_buffer = self.serial_buffer.split(b'\n')
        for line in lines[-1:]:
            try:
                values = [float(v) for v in line.decode().strip().split(',')]
            except ValueError:
                continue
            if len(values) not in (2, 4):
                continue
            l, r = values[:2]
            self.left, self.right = min(l / self.adc_max, 1.0), min(r / self.adc_max, 1.0)
            self.raw_time = self.now()
            if len(values) == 4:
                f = HandleForce()
                f.header.stamp = self.get_clock().now().to_msg()
                f.left, f.right = values[2], values[3]
                self.force_pub.publish(f)

    # ---------- calibration ----------

    def calibrate(self, kind, response):
        now = self.now()
        recent = [(l, r) for t, l, r in self.history if now - t <= 1.0]
        if len(recent) < 10:
            response.success, response.message = False, 'No grip readings in the last second'
            return response
        hand = self.logic.hand
        values = ([l for l, _ in recent] if hand == 'left' else [r for _, r in recent]) if self.logic.one_handed \
            else [min(l, r) for l, r in recent]
        self.calibration[kind] = sum(values) / len(values)
        response.success, response.message = True, f'{kind} grip recorded: {self.calibration[kind]:.2f}'
        if 'relaxed' in self.calibration and 'firm' in self.calibration:
            try:
                self.logic.calibrate(self.calibration['relaxed'], self.calibration['firm'])
                response.message += (f' - thresholds set: light {self.logic.light:.2f}, '
                                     f'firm {self.logic.firm:.2f}')
            except ValueError as e:
                response.success, response.message = False, str(e)
        self.get_logger().info(response.message)
        return response

    # ---------- main loop ----------

    def update(self):
        now = self.now()
        if self.source == 'arduino':
            self.read_serial()
        if self.source == 'operator':
            left = right = 0.4 if self.operator_enabled else 0.0
        elif now - self.raw_time > self.raw_timeout:
            left = right = 0.0     # no readings: count as released
        else:
            left, right = self.left, self.right
        self.history = [h for h in self.history if now - h[0] <= 1.0] + [(now, left, right)]

        state = self.logic.update(left, right, now)
        # the operator's Stop walking and the headset's safety stops beat the grip
        if self.forced or (self.source != 'operator' and not self.operator_enabled):
            self.logic.force_stop(latch=self.forced)
            state = STOP
        if self.source == 'operator' and state > WALK:
            state = WALK

        msg = GripState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.left, msg.right, msg.state = float(left), float(right), state
        self.pub.publish(msg)
        if state != self.last_state:
            self.get_logger().info(f'Grip: {NAMES[state]}')
            self.last_state = state


def main():
    rclpy.init()
    node = GripStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

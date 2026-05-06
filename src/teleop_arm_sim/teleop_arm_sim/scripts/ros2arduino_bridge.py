#!/usr/bin/env python3
"""
ROS2 → Arduino Serial Bridge
Subscribes to /finger_collision (JSON from collision_detector.py)
and forwards a single-byte command to the Arduino over Serial.

Command protocol (matches arduino.ino):
  '1' → index finger collision
  '2' → middle finger collision
  '3' → thumb collision
  '0' → all fingers (unused here, available for future use)
  'R' → release / collision ended  (Arduino ignores unknown chars gracefully)

Configuration via ROS2 parameters:
  serial_port   (default: '/dev/ttyUSB0')
  baud_rate     (default: 9600)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports
import json
import time


# ── Finger name → Arduino command char ───────────────────────────────────────
# Extend this when you add more fingers to collision_detector.py
FINGER_TO_CMD = {
    'index':  b'1',
    'middle': b'2',
    'thumb':  b'3',
    'ring':   b'4',   # reserved for future fingers
    'pinky':  b'5',
}
RELEASE_CMD = b'R'
# ─────────────────────────────────────────────────────────────────────────────


class Ros2ArduinoBridge(Node):
    def __init__(self):
        super().__init__('ros2arduino_bridge')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',   9600)
        self.declare_parameter('auto_detect', True)   # scan ports if main fails

        port      = self.get_parameter('serial_port').get_parameter_value().string_value
        baud      = self.get_parameter('baud_rate').get_parameter_value().integer_value
        auto_det  = self.get_parameter('auto_detect').get_parameter_value().bool_value

        # ── Open serial port ──────────────────────────────────────────────────
        self.ser = None
        self._open_serial(port, baud, auto_detect=auto_det)

        # ── ROS2 subscriber ───────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            String,
            '/finger_collision',
            self.collision_callback,
            10
        )

        # ── Cooldown: avoid flooding Arduino within same collision event ───────
        # Stores last-sent time per finger
        self._last_sent: dict[str, float] = {}
        self._cooldown_s = 0.4   # seconds — tune to match LED_ON_DURATION in .ino

        self.get_logger().info('ROS2 → Arduino bridge ready.')

    # ── Serial helpers ────────────────────────────────────────────────────────

    def _open_serial(self, port: str, baud: int, auto_detect: bool = True):
        """Try to open the given port; fall back to auto-detection if enabled."""
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)   # let Arduino reset after DTR toggle
            self.get_logger().info(f'Serial opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Could not open {port}: {e}')
            if auto_detect:
                self._auto_detect_arduino(baud)

    def _auto_detect_arduino(self, baud: int):
        """Scan available ports and try each one (looks for Arduino VID)."""
        ARDUINO_VIDS = {0x2341, 0x1A86, 0x0403, 0x10C4}   # Uno, CH340, FTDI, CP210x
        candidates = list(serial.tools.list_ports.comports())
        for p in candidates:
            vid = p.vid if p.vid else 0
            if vid in ARDUINO_VIDS or 'arduino' in (p.description or '').lower():
                try:
                    self.ser = serial.Serial(p.device, baud, timeout=1)
                    time.sleep(2)
                    self.get_logger().info(f'Auto-detected Arduino on {p.device}')
                    return
                except serial.SerialException:
                    continue
        # Last resort: just list what's available to help the user
        port_list = [p.device for p in candidates] or ['(none found)']
        self.get_logger().error(
            'Could not find Arduino. Available ports: ' + ', '.join(port_list) +
            '\nSet the correct port with: --ros-args -p serial_port:=/dev/ttyACM0'
        )

    def _send(self, cmd: bytes):
        """Write a byte to the Arduino with error recovery."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial not open — cannot send command.')
            return
        try:
            self.ser.write(cmd)
            self.ser.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self.ser = None   # mark as dead so we don't spam errors

    # ── Main callback ─────────────────────────────────────────────────────────

    def collision_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Bad JSON from /finger_collision: {msg.data}')
            return

        finger       = data.get('finger', '')
        in_collision = data.get('in_collision', False)
        penetration  = data.get('penetration', 0.0)

        if in_collision:
            cmd = FINGER_TO_CMD.get(finger)
            if cmd is None:
                self.get_logger().warn(f'Unknown finger name: "{finger}" — skipping.')
                return

            # Cooldown: only send if enough time has passed for this finger
            now = time.monotonic()
            last = self._last_sent.get(finger, 0.0)
            if now - last < self._cooldown_s:
                return

            self._last_sent[finger] = now
            self._send(cmd)
            self.get_logger().info(
                f'→ Arduino: finger={finger} cmd={cmd} penetration={penetration:.4f}m'
            )

        else:
            # Collision released — notify Arduino (it will just ignore 'R' for now,
            # but the Arduino sketch can be extended to handle it)
            self._send(RELEASE_CMD)
            self._last_sent.pop(finger, None)   # reset cooldown on release
            self.get_logger().info(f'→ Arduino: RELEASE {finger}')

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ros2ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Terminal keyboard teleop for the QCar.

Publishes a geometry_msgs/Vector3Stamped to /teleop/cmd (the manual input
of qcar_control's command_mux). Does NOT publish to /qcar/user_command
directly -- so it can never bypass the safety stops.

To make the mux actually use these commands, set its mode to "manual":

    ros2 topic pub --once /qcar/control_mode std_msgs/String '{data: manual}'
    # ... drive ...
    ros2 topic pub --once /qcar/control_mode std_msgs/String '{data: auto}'

Keys (current command shown in the terminal each frame):

    w / arrow up        throttle  +
    s / arrow down      throttle  -
    a / arrow left      steering  +  (left)
    d / arrow right     steering  -  (right)
    space               brake -- snap throttle + steering to 0
    e / E               toggle gear (forward only / reverse allowed)
    + / -               throttle step  larger / smaller
    [ / ]               steering step  larger / smaller
    q                   quit
    anything else       no-op

Deadman safety: if no key is pressed for `deadman_timeout` seconds, the
teleop stops publishing entirely. The mux's "no recent input -> (0, 0)"
behaviour then takes over.
"""

import sys
import os
import select
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped


# ANSI escape sequences for arrow keys (3-byte sequence: ESC [ A/B/C/D).
ARROW_UP    = '\x1b[A'
ARROW_DOWN  = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT  = '\x1b[D'


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('vector3_keyboard_teleop')

        self.declare_parameter('cmd_topic',        '/teleop/cmd')
        self.declare_parameter('throttle_step',    0.02)   # added per W press
        self.declare_parameter('steering_step',    0.10)   # added per A press
        self.declare_parameter('throttle_max',     0.20)
        self.declare_parameter('steering_max',     0.50)
        self.declare_parameter('publish_hz',       20.0)
        self.declare_parameter('deadman_timeout',  0.5)    # seconds
        self.declare_parameter('allow_reverse',    False)

        self.cmd_topic    = self.get_parameter('cmd_topic').value
        self.t_step       = float(self.get_parameter('throttle_step').value)
        self.s_step       = float(self.get_parameter('steering_step').value)
        self.t_max        = float(self.get_parameter('throttle_max').value)
        self.s_max        = float(self.get_parameter('steering_max').value)
        rate              = float(self.get_parameter('publish_hz').value)
        self.deadman_sec  = float(self.get_parameter('deadman_timeout').value)
        self.allow_rev    = bool(self.get_parameter('allow_reverse').value)

        self._throttle = 0.0
        self._steering = 0.0
        self._last_key_time = 0.0
        self._running = True

        self.pub = self.create_publisher(Vector3Stamped, self.cmd_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._publish_tick)

        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

        self.get_logger().info(self._banner())

    def _banner(self):
        return (
            '\n'
            '  ============================================================\n'
            f'   vector3_teleop ready -- publishing to {self.cmd_topic}\n'
            '  ------------------------------------------------------------\n'
            '   w / Up      throttle +        s / Down    throttle -\n'
            '   a / Left    steering +        d / Right   steering -\n'
            '   space       brake (0, 0)      q          quit\n'
            '   e           toggle reverse    + / -      throttle step\n'
            '   [ / ]       steering step\n'
            '  ------------------------------------------------------------\n'
            f'   throttle limits: ±{self.t_max:.2f}  step {self.t_step:.3f}\n'
            f'   steering limits: ±{self.s_max:.2f}  step {self.s_step:.3f}\n'
            f'   reverse enabled: {self.allow_rev}\n'
            f'   deadman timeout: {self.deadman_sec:.1f} s\n'
            '  ============================================================\n'
            '\n'
            '   NOTE: set the command_mux to "manual" so this teleop takes\n'
            '         over, e.g.:\n'
            '             ros2 topic pub --once /qcar/control_mode \\\n'
            '                 std_msgs/String "{data: manual}"\n'
        )

    # ── keyboard reader thread ────────────────────────────────────────────
    def _read_key(self, fd):
        """Read one key (possibly an arrow-key escape sequence). Returns ''
        if no key is available in the next 0.1 s."""
        if not select.select([fd], [], [], 0.1)[0]:
            return ''
        ch = os.read(fd, 1).decode(errors='ignore')
        if ch != '\x1b':
            return ch
        # Possible arrow key: ESC [ X
        rest = ''
        if select.select([fd], [], [], 0.01)[0]:
            rest += os.read(fd, 2).decode(errors='ignore')
        return ch + rest

    def _apply_key(self, ch):
        clamp_t = self.t_max
        clamp_s = self.s_max
        rev_floor = -self.t_max if self.allow_rev else 0.0

        if ch in ('w', ARROW_UP):
            self._throttle = min(clamp_t, self._throttle + self.t_step)
        elif ch in ('s', ARROW_DOWN):
            self._throttle = max(rev_floor, self._throttle - self.t_step)
        elif ch in ('a', ARROW_LEFT):
            self._steering = min(clamp_s, self._steering + self.s_step)
        elif ch in ('d', ARROW_RIGHT):
            self._steering = max(-clamp_s, self._steering - self.s_step)
        elif ch == ' ':
            self._throttle = 0.0
            self._steering = 0.0
        elif ch == 'e':
            self.allow_rev = not self.allow_rev
            if not self.allow_rev and self._throttle < 0:
                self._throttle = 0.0
            self.get_logger().info(f'reverse {"on" if self.allow_rev else "off"}')
        elif ch == '+':
            self.t_step = min(0.10, self.t_step * 1.25)
            self.get_logger().info(f'throttle step: {self.t_step:.3f}')
        elif ch == '-':
            self.t_step = max(0.005, self.t_step / 1.25)
            self.get_logger().info(f'throttle step: {self.t_step:.3f}')
        elif ch == '[':
            self.s_step = max(0.02, self.s_step / 1.25)
            self.get_logger().info(f'steering step: {self.s_step:.3f}')
        elif ch == ']':
            self.s_step = min(0.30, self.s_step * 1.25)
            self.get_logger().info(f'steering step: {self.s_step:.3f}')
        elif ch == 'q':
            self._running = False
            rclpy.shutdown()
            return

        # Print one-line live status.
        sys.stdout.write(
            f'\r  throttle {self._throttle:+.3f}  steering {self._steering:+.3f}'
            f'  ({"reverse on " if self.allow_rev else "reverse off"})    '
        )
        sys.stdout.flush()

    def _key_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._running:
                ch = self._read_key(fd)
                if not ch:
                    continue
                self._last_key_time = time.monotonic()
                self._apply_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ── publish loop ──────────────────────────────────────────────────────
    def _publish_tick(self):
        if not self._running:
            return
        # Deadman: stop publishing entirely if no key in deadman_sec.
        if (time.monotonic() - self._last_key_time) > self.deadman_sec:
            return
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'teleop'
        msg.vector.x = float(self._throttle)
        msg.vector.y = float(self._steering)
        msg.vector.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print()  # newline after the live-status carriage return


if __name__ == '__main__':
    main()

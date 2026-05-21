"""
Command multiplexer for the QCar.

Every node that wants to move the QCar publishes its desired
geometry_msgs/Vector3Stamped to this mux's input topics. The mux then
publishes the single authoritative /qcar/user_command, gated by safety
flags and an explicit mode.

Inputs:
  /lane_follower/raw_cmd       Vector3Stamped   the autonomous controller
  /teleop/cmd                  Vector3Stamped   the manual teleop
  /qcar/obstacle_detected      Bool             lidar safety stop
  /qcar/safe_stop_active       Bool             dashboard safe-stop button
  /qcar/control_mode           String           one of "auto" / "manual" / "off"

Output:
  /qcar/user_command           Vector3Stamped   the QCar hardware driver

Rules (highest priority first):
  1. obstacle_detected   -> publish (0, 0)
  2. safe_stop_active    -> publish (0, 0)
  3. mode == "off"       -> publish (0, 0)
  4. mode == "manual"    -> forward latest /teleop/cmd (or (0,0) if none yet)
  5. mode == "auto"      -> forward latest /lane_follower/raw_cmd (or (0,0))

The mode is sticky -- it only changes when a new message arrives on
/qcar/control_mode. Default at startup is "auto".

The output publish rate is independent of the input rates so the mux is
always responsive to safety flags.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, String


VALID_MODES = ('auto', 'manual', 'off')


class CommandMux(Node):
    def __init__(self):
        super().__init__('command_mux')

        self.declare_parameter('auto_in_topic',     '/lane_follower/raw_cmd')
        self.declare_parameter('manual_in_topic',   '/teleop/cmd')
        self.declare_parameter('cmd_out_topic',     '/qcar/user_command')
        self.declare_parameter('obstacle_topic',    '/qcar/obstacle_detected')
        self.declare_parameter('safe_stop_topic',   '/qcar/safe_stop_active')
        self.declare_parameter('mode_topic',        '/qcar/control_mode')
        self.declare_parameter('publish_hz',         30.0)
        self.declare_parameter('initial_mode',      'auto')

        auto_in   = self.get_parameter('auto_in_topic').value
        manual_in = self.get_parameter('manual_in_topic').value
        cmd_out   = self.get_parameter('cmd_out_topic').value
        obs_top   = self.get_parameter('obstacle_topic').value
        ss_top    = self.get_parameter('safe_stop_topic').value
        mode_top  = self.get_parameter('mode_topic').value
        rate_hz   = float(self.get_parameter('publish_hz').value)
        init_mode = str(self.get_parameter('initial_mode').value).lower()

        if init_mode not in VALID_MODES:
            self.get_logger().warn(
                f'invalid initial_mode "{init_mode}", falling back to "auto"'
            )
            init_mode = 'auto'
        self._mode = init_mode

        self._latest_auto_cmd = None
        self._latest_manual_cmd = None
        self._obstacle = False
        self._safe_stop = False

        self.create_subscription(Vector3Stamped, auto_in,   self._on_auto,   10)
        self.create_subscription(Vector3Stamped, manual_in, self._on_manual, 10)
        self.create_subscription(Bool,           obs_top,   self._on_obstacle, 10)
        self.create_subscription(Bool,           ss_top,    self._on_safe_stop, 10)
        self.create_subscription(String,         mode_top,  self._on_mode,   10)

        self.pub = self.create_publisher(Vector3Stamped, cmd_out, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'command_mux ready\n'
            f'  auto in : {auto_in}\n'
            f'  manual in: {manual_in}\n'
            f'  out     : {cmd_out} @ {rate_hz:.0f} Hz\n'
            f'  mode    : {self._mode}  (publish String to {mode_top} to change)\n'
        )

    # ── callbacks ──────────────────────────────────────────────────────────
    def _on_auto(self, msg: Vector3Stamped):
        self._latest_auto_cmd = msg

    def _on_manual(self, msg: Vector3Stamped):
        self._latest_manual_cmd = msg

    def _on_obstacle(self, msg: Bool):
        was = self._obstacle
        self._obstacle = bool(msg.data)
        if self._obstacle and not was:
            self.get_logger().warn('obstacle_detected ON -- forcing stop')
        elif not self._obstacle and was:
            self.get_logger().info('obstacle_detected OFF -- resuming')

    def _on_safe_stop(self, msg: Bool):
        was = self._safe_stop
        self._safe_stop = bool(msg.data)
        if self._safe_stop and not was:
            self.get_logger().warn('SAFE STOP engaged from dashboard')
        elif not self._safe_stop and was:
            self.get_logger().info('SAFE STOP released')

    def _on_mode(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode not in VALID_MODES:
            self.get_logger().warn(
                f'ignoring unknown control_mode "{msg.data}" '
                f'(expected one of {VALID_MODES})'
            )
            return
        if new_mode != self._mode:
            self.get_logger().info(f'control_mode: {self._mode} -> {new_mode}')
            self._mode = new_mode

    # ── publish loop ───────────────────────────────────────────────────────
    def _tick(self):
        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'command_mux'

        # 1. safety overrides
        if self._safe_stop or self._obstacle:
            out.vector.x = 0.0
            out.vector.y = 0.0
            out.vector.z = 0.0
            self.pub.publish(out)
            return

        # 2. explicit "off"
        if self._mode == 'off':
            out.vector.x = 0.0
            out.vector.y = 0.0
            out.vector.z = 0.0
            self.pub.publish(out)
            return

        # 3. mode-selected source
        if self._mode == 'manual':
            src = self._latest_manual_cmd
        else:  # 'auto'
            src = self._latest_auto_cmd

        if src is None:
            # No upstream command yet -- emit a zero so the QCar doesn't drift.
            out.vector.x = 0.0
            out.vector.y = 0.0
        else:
            out.vector.x = float(src.vector.x)
            out.vector.y = float(src.vector.y)
        out.vector.z = 0.0
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Command multiplexer for the QCar.

Every node that wants to move the QCar publishes its desired
geometry_msgs/Vector3Stamped to this mux's input topics. The mux then
publishes the single authoritative /qcar/user_command, gated by safety
flags and an explicit mode.

Inputs:
  /lane_follower/raw_cmd       Vector3Stamped   the lane-following controller
  /overtake/raw_cmd            Vector3Stamped   the overtake supervisor
  /qcar/obstacle_detected      Bool             lidar EMERGENCY stop
  /qcar/safe_stop_active       Bool             dashboard safe-stop button
  /qcar/control_mode           String           one of "auto" / "off"
  /qcar/control_source         String           one of "lane" / "overtake"

Output:
  /qcar/user_command           Vector3Stamped   the QCar hardware driver

Rules (highest priority first):
  1. obstacle_detected   -> publish (0, 0)
  2. safe_stop_active    -> publish (0, 0)
  3. mode == "off"       -> publish (0, 0)
  4. mode == "auto"      -> forward the command from the active source
                           (lane or overtake), with speed clamped to
                           +/- max_speed.

Both mode and source are sticky -- each changes only when a new message
arrives on its topic. Defaults at startup: mode "auto", source "lane"
(so behaviour is identical to the old mux until the supervisor takes over).

The speed clamp (|vector.x| <= max_speed) lives HERE so it is the single
source of truth: no upstream node can drive the car faster than the
hardware safety limit, regardless of which source is active.

The output publish rate is independent of the input rates so the mux is
always responsive to safety flags.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, String


VALID_MODES = ('auto', 'off')
VALID_SOURCES = ('lane', 'overtake')


STEER_GAIN_YPOS = 1.6  #  derecha
STEER_GAIN_YNEG = 1.0   # izquierda (reforzada)


class CommandMux(Node):
    def __init__(self):
        super().__init__('command_mux')

        self.declare_parameter('auto_in_topic',     '/lane_follower/raw_cmd')
        self.declare_parameter('overtake_in_topic', '/overtake/raw_cmd')
        self.declare_parameter('cmd_out_topic',     '/qcar/user_command')
        self.declare_parameter('obstacle_topic',    '/qcar/obstacle_detected')
        self.declare_parameter('safe_stop_topic',   '/qcar/safe_stop_active')
        self.declare_parameter('mode_topic',        '/qcar/control_mode')
        self.declare_parameter('source_topic',      '/qcar/control_source')
        self.declare_parameter('publish_hz',         30.0)
        self.declare_parameter('initial_mode',      'auto')
        self.declare_parameter('initial_source',    'lane')
        self.declare_parameter('max_speed',          0.1)   # m/s, hard cap
        # Si source='overtake' pero no llega comando fresco en este tiempo,
        # el mux reenvía el seguidor de línea (auto-recuperación si el
        # supervisor muere o se queda colgado). Evita que el mux se quede
        # "pegado" en overtake con un comando viejo.
        self.declare_parameter('overtake_timeout',   0.5)   # s

        auto_in     = self.get_parameter('auto_in_topic').value
        overtake_in = self.get_parameter('overtake_in_topic').value
        cmd_out     = self.get_parameter('cmd_out_topic').value
        obs_top     = self.get_parameter('obstacle_topic').value
        ss_top      = self.get_parameter('safe_stop_topic').value
        mode_top    = self.get_parameter('mode_topic').value
        source_top  = self.get_parameter('source_topic').value
        rate_hz     = float(self.get_parameter('publish_hz').value)
        init_mode   = str(self.get_parameter('initial_mode').value).lower()
        init_source = str(self.get_parameter('initial_source').value).lower()
        self._max_speed = abs(float(self.get_parameter('max_speed').value))
        self._overtake_to = float(self.get_parameter('overtake_timeout').value)

        if init_mode not in VALID_MODES:
            self.get_logger().warn(
                f'invalid initial_mode "{init_mode}", falling back to "auto"'
            )
            init_mode = 'auto'
        self._mode = init_mode

        if init_source not in VALID_SOURCES:
            self.get_logger().warn(
                f'invalid initial_source "{init_source}", falling back to "lane"'
            )
            init_source = 'lane'
        self._source = init_source

        self._latest_auto_cmd = None
        self._latest_overtake_cmd = None
        self._last_overtake_t = None
        self._overtake_stale_warned = False
        self._obstacle = False
        self._safe_stop = False

        self.create_subscription(Vector3Stamped, auto_in,     self._on_auto,     10)
        self.create_subscription(Vector3Stamped, overtake_in, self._on_overtake, 10)
        self.create_subscription(Bool,           obs_top,     self._on_obstacle, 10)
        self.create_subscription(Bool,           ss_top,      self._on_safe_stop, 10)
        self.create_subscription(String,         mode_top,    self._on_mode,     10)
        self.create_subscription(String,         source_top,  self._on_source,   10)

        self.pub = self.create_publisher(Vector3Stamped, cmd_out, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'command_mux ready\n'
            f'  lane in     : {auto_in}\n'
            f'  overtake in : {overtake_in}\n'
            f'  out         : {cmd_out} @ {rate_hz:.0f} Hz\n'
            f'  mode        : {self._mode}  (String -> {mode_top})\n'
            f'  source      : {self._source}  (String -> {source_top})\n'
            f'  max_speed   : {self._max_speed:.3f} m/s (hard cap on |x|)\n'
        )

    # ── callbacks ──────────────────────────────────────────────────────────
    def _on_auto(self, msg: Vector3Stamped):
        self._latest_auto_cmd = msg

    def _on_overtake(self, msg: Vector3Stamped):
        self._latest_overtake_cmd = msg
        self._last_overtake_t = self.get_clock().now()
        self._overtake_stale_warned = False

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

    def _on_source(self, msg: String):
        new_source = msg.data.strip().lower()
        if new_source not in VALID_SOURCES:
            self.get_logger().warn(
                f'ignoring unknown control_source "{msg.data}" '
                f'(expected one of {VALID_SOURCES})'
            )
            return
        if new_source != self._source:
            self.get_logger().info(f'control_source: {self._source} -> {new_source}')
            self._source = new_source

    def _clamp_speed(self, x: float) -> float:
        """Hard cap on forward/reverse speed -- single source of truth."""
        return max(min(x, self._max_speed), -self._max_speed)

    # ── publish loop ───────────────────────────────────────────────────────
    def _tick(self):
        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'command_mux'

        # 1. safety overrides.
        #    El paro de emergencia del lidar (obstacle) se SUPRIME mientras la
        #    fuente activa es 'overtake': en ese modo la maniobra de evasión ES
        #    la respuesta al obstáculo, así que frenar en seco la congelaría.
        #    El safe_stop del dashboard SIEMPRE para.
        emergency = self._obstacle and self._source != 'overtake'
        if self._safe_stop or emergency:
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

        # 3. forward the active source's latest command, speed-clamped.
        #    Watchdog: if 'overtake' is selected but no fresh overtake command
        #    arrived recently (supervisor dead/stalled), fall back to the lane
        #    follower so control isn't stuck on a stale overtake command.
        if self._source == 'overtake':
            fresh = (self._last_overtake_t is not None and
                     (self.get_clock().now() - self._last_overtake_t).nanoseconds * 1e-9
                     <= self._overtake_to)
            if fresh:
                src = self._latest_overtake_cmd
            else:
                src = self._latest_auto_cmd
                if not self._overtake_stale_warned:
                    self.get_logger().warn(
                        'overtake source stale -- falling back to lane follower')
                    self._overtake_stale_warned = True
        else:
            src = self._latest_auto_cmd

        if src is None:
            # No upstream command yet -- emit a zero so the QCar doesn't drift.
            out.vector.x = 0.0
            out.vector.y = 0.0
        else:
            out.vector.x = self._clamp_speed(float(src.vector.x))
            y = float(src.vector.y)
            y *= STEER_GAIN_YPOS if y > 0.0 else STEER_GAIN_YNEG
            out.vector.y = y
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

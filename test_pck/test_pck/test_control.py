#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Vector3Stamped


class test_control(Node):
    def __init__(self):
        super().__init__('test_control')

        self.declare_parameter('is_sim',        False)
        self.declare_parameter('mode',          'circle')  # 'circle' | 'square'

        # Parámetros círculo
        self.declare_parameter('throttle',      0.06)
        self.declare_parameter('steering',      0.25)

        # Parámetros cuadrado
        self.declare_parameter('side_duration',    10.0)   # s por lado recto
        self.declare_parameter('turn_duration',     7.3)   # s por giro 90°
        self.declare_parameter('turn_steering',     0.45)  # rad en las esquinas
        self.declare_parameter('straight_steering', 0.0)   # trim para corregir deriva en rectos
        # num_segments: cuántos segmentos ejecutar antes de parar.
        #   8 = cuadrado completo (4 rectos + 4 giros)
        #   3 = L             (recto → giro → recto)
        #   6 = U / C         (recto → giro → recto → giro → recto → giro)
        self.declare_parameter('num_segments',      8)

        is_sim = self.get_parameter('is_sim').get_parameter_value().bool_value
        self._mode       = self.get_parameter('mode').value
        self._throttle   = self.get_parameter('throttle').get_parameter_value().double_value
        self._steering   = self.get_parameter('steering').get_parameter_value().double_value
        self._side_dur       = self.get_parameter('side_duration').get_parameter_value().double_value
        self._turn_dur       = self.get_parameter('turn_duration').get_parameter_value().double_value
        self._turn_steer     = self.get_parameter('turn_steering').get_parameter_value().double_value
        self._straight_steer = self.get_parameter('straight_steering').get_parameter_value().double_value
        self._num_segments   = self.get_parameter('num_segments').get_parameter_value().integer_value

        topic = '/qcar_sim/user_command' if is_sim else '/qcar/user_command'

        # Máquina de estados cuadrado:
        # segmentos 0,2,4,6 = rectos | 1,3,5,7 = giros | 8 = done
        self._segment   = 0
        self._seg_start = self.get_clock().now()

        self.pub   = self.create_publisher(Vector3Stamped, topic, 10)
        self.timer = self.create_timer(0.1, self._tick)

        if self._mode == 'circle':
            self.get_logger().info(
                f'Modo CIRCULO → {topic}\n'
                f'  throttle = {self._throttle:.3f}  '
                f'steering = {self._steering:.3f} rad '
                f'({"izquierda" if self._steering >= 0 else "derecha"})'
            )
        else:
            shape = {3: 'L', 6: 'U/C', 8: 'CUADRADO'}.get(self._num_segments,
                     f'{self._num_segments} segmentos')
            self.get_logger().info(
                f'Modo {shape} → {topic}\n'
                f'  throttle        = {self._throttle:.3f}\n'
                f'  lado            = {self._side_dur:.1f} s  '
                f'(~{self._throttle * self._side_dur:.2f} m)\n'
                f'  giro            = {self._turn_dur:.1f} s  (~90°)\n'
                f'  turn_steer      = {self._turn_steer:.3f} rad\n'
                f'  straight_steer  = {self._straight_steer:+.3f} rad (trim)\n'
                f'  num_segments    = {self._num_segments}'
            )

    # ── tick principal ─────────────────────────────────────────────────────────

    def _tick(self):
        if self._mode == 'circle':
            self._publish(self._throttle, self._steering)
            return

        # --- modo cuadrado ---
        if self._segment >= self._num_segments:
            self._publish(0.0, 0.0)
            return

        elapsed  = (self.get_clock().now() - self._seg_start).nanoseconds * 1e-9
        is_straight = (self._segment % 2 == 0)
        duration    = self._side_dur if is_straight else self._turn_dur

        if elapsed >= duration:
            self._segment  += 1
            self._seg_start = self.get_clock().now()
            if self._segment >= self._num_segments:
                self.get_logger().info('Cuadrado completo — deteniendo')
            elif self._segment % 2 == 0:
                self.get_logger().info(f'→ Lado {self._segment // 2 + 1}/4')
            else:
                self.get_logger().info(f'→ Giro {self._segment // 2 + 1}/4')

        if self._segment >= self._num_segments:
            self._publish(0.0, 0.0)
        elif self._segment % 2 == 0:
            self._publish(self._throttle, self._straight_steer)  # recto + trim
        else:
            self._publish(self._throttle, self._turn_steer)      # giro

    # ── helpers ────────────────────────────────────────────────────────────────

    def _publish(self, throttle: float, steering: float):
        msg = Vector3Stamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command_input'
        msg.vector.x = throttle
        msg.vector.y = steering
        msg.vector.z = 0.0
        self.pub.publish(msg)

    def publish_stop(self):
        for _ in range(10):
            self._publish(0.0, 0.0)
            time.sleep(0.02)
        self.get_logger().info('shutdown: published zero velocity/direction')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = test_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.publish_stop()
            except Exception as e:
                node.get_logger().error(f'failed to publish stop: {e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

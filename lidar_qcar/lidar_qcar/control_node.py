#!/usr/bin/env python3
"""
control_node.py  —  Rebase reactivo simple
==========================================

Comportamiento:
  1. Siempre avanza (x = FWD_SPEED).
  2. Si hay obstáculo enfrente → gira a la derecha (y = STEER_RIGHT)
     hasta que el obstáculo ya no esté en la zona frontal.
  3. Una vez despejado el frente → sigue recto hasta que el obstáculo
     salga completamente del rango lateral derecho del LiDAR.
  4. Regresa al carril girando a la izquierda durante RETURN_TIME segundos.
  5. Vuelve a marcha recta normal.

Uso:
  ros2 run lidar_qcar control_node
  ros2 run lidar_qcar control_node --ros-args -p is_sim:=true
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from enum import Enum, auto


class State(Enum):
    STRAIGHT     = auto()   # marcha recta normal
    DODGE        = auto()   # girando a la derecha — obstáculo enfrente
    PASS         = auto()   # frente despejado, avanzando hasta dejar el obstáculo atrás
    RETURN       = auto()   # regresando al carril (giro izquierda por tiempo fijo)


# ── Parámetros por defecto ────────────────────────────────────────────────────
FWD_SPEED    = 0.05    # m/s
STEER_RIGHT  = -0.30   # vector.y negativo → gira derecha
STEER_LEFT   =  0.30   # vector.y positivo → gira izquierda
RETURN_TIME  =  1.5    # segundos girando izquierda para regresar al carril
PASS_TIME    =  2.0    # segundos avanzando recto antes de regresar al carril

# Sector "lateral derecho" para detectar que el obstáculo quedó atrás.
# 0° = frente, ángulos en sentido antihorario.
# Derecha del robot ≈ 270° (o −90°), ventana ±40°.
RIGHT_SECTOR_MIN_DEG = 230.0
RIGHT_SECTOR_MAX_DEG = 310.0
RIGHT_CLEAR_DIST     = 0.70   # metros — más lejos de esto → carril libre


class OvertakeController(Node):

    def __init__(self):
        super().__init__('overtake_controller')

        # ── Parámetros ROS ────────────────────────────────────────────────────
        self.declare_parameter('is_sim',       False)
        self.declare_parameter('fwd_speed',    FWD_SPEED)
        self.declare_parameter('steer_right',  STEER_RIGHT)
        self.declare_parameter('steer_left',   STEER_LEFT)
        self.declare_parameter('return_time',  RETURN_TIME)
        self.declare_parameter('pass_time',    PASS_TIME)
        self.declare_parameter('right_clear_dist', RIGHT_CLEAR_DIST)

        is_sim          = self.get_parameter('is_sim').get_parameter_value().bool_value
        self._fwd       = self.get_parameter('fwd_speed').value
        self._sr        = self.get_parameter('steer_right').value
        self._sl        = self.get_parameter('steer_left').value
        self._ret_time  = self.get_parameter('return_time').value
        self._pass_time = self.get_parameter('pass_time').value
        self._clr_dist  = self.get_parameter('right_clear_dist').value

        cmd_topic = '/qcar_sim/user_command' if is_sim else '/qcar/user_command'

        # ── QoS BEST_EFFORT para el scan ─────────────────────────────────────
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self._pub = self.create_publisher(Vector3Stamped, cmd_topic, 10)
        self.create_subscription(Bool,      '/qcar/obstacle_detected',
                                 self._cb_obstacle, 10)
        self.create_subscription(LaserScan, '/qcar/scan_republished',
                                 self._cb_scan,     qos_be)

        # ── Estado interno ────────────────────────────────────────────────────
        self._state          = State.STRAIGHT
        self._obstacle_front = False
        self._right_clear    = True
        self._phase_start    = self.get_clock().now()

        self.create_timer(0.1, self._loop)   # 10 Hz

        self.get_logger().info(
            f'\n'
            f'  ╔══════════════════════════════════════╗\n'
            f'  ║   OvertakeController  iniciado       ║\n'
            f'  ║  topic : {cmd_topic:<27}║\n'
            f'  ╚══════════════════════════════════════╝'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_obstacle(self, msg: Bool):
        self._obstacle_front = msg.data

    def _cb_scan(self, msg: LaserScan):
        pass  # reservado para uso futuro

    # ── Loop de control ───────────────────────────────────────────────────────

    def _loop(self):
        now = self.get_clock().now()

        # ── Transiciones de estado ────────────────────────────────────────────
        if self._state == State.STRAIGHT:
            if self._obstacle_front:
                self._go(State.DODGE, now)

        elif self._state == State.DODGE:
            if not self._obstacle_front:
                self._go(State.PASS, now)

        elif self._state == State.PASS:
            elapsed = (now - self._phase_start).nanoseconds * 1e-9
            if elapsed >= self._pass_time and not self._obstacle_front:
                self._go(State.RETURN, now)

        elif self._state == State.RETURN:
            elapsed = (now - self._phase_start).nanoseconds * 1e-9
            if elapsed >= self._ret_time:
                self._go(State.STRAIGHT, now)

        # ── Salida de velocidad según estado ACTUAL (ya actualizado) ──────────
        vel_x = self._fwd   # siempre avanza

        if self._state == State.STRAIGHT:
            vel_y = 0.0
        elif self._state == State.DODGE:
            vel_y = self._sr        # gira derecha
        elif self._state == State.PASS:
            vel_y = 0.0
        elif self._state == State.RETURN:
            vel_y = self._sl        # gira izquierda
        else:
            vel_y = 0.0

        self._publish(vel_x, vel_y)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _go(self, new_state: State, now):
        self.get_logger().info(f'  [{self._state.name}] → [{new_state.name}]')
        self._state       = new_state
        self._phase_start = now

    def _publish(self, vel_x: float, vel_y: float):
        msg = Vector3Stamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command_input'
        msg.vector.x = vel_x
        msg.vector.y = vel_y
        msg.vector.z = 0.0
        self._pub.publish(msg)
        self.get_logger().debug(
            f'[{self._state.name:<10}]  x={vel_x:+.3f}  y={vel_y:+.3f}  '
            f'obs={self._obstacle_front}  right_clear={self._right_clear}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OvertakeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
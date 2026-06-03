#!/usr/bin/env python3
"""
lidar_node.py
=============
Nodo ROS 2 para el sensor LiDAR del QCar.

- Suscribe /qcar/scan  (sensor_msgs/msg/LaserScan)
- Republica en /qcar/scan_republished
- Publica estadísticas JSON en /qcar/lidar_stats
- Publica bandera de obstáculo en /qcar/obstacle_detected  (std_msgs/Bool)
- Diagnóstico automático si no llegan mensajes tras N segundos

Detección de obstáculo
----------------------
Se evalúa SÓLO la zona frontal del robot, con forma de TRAPEZOIDE:
ancho cerca del QCar (base ancha) y estrecho al frente (lejos).
Para cada rayo i con ángulo θ_i:
  - distancia longitudinal  x = r · cos(θ_i)
  - distancia lateral        y = r · sin(θ_i)
El semiancho permitido se interpola linealmente con x:
  half_width(x) = (OBS_WIDTH_NEAR + (x/OBS_DEPTH)·(OBS_WIDTH_FAR − OBS_WIDTH_NEAR)) / 2
Si  0 < x ≤ OBS_DEPTH  y  |y| ≤ half_width(x)  → obstáculo detectado.
La zona muerta central se mantiene igual y se ignora.
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import math
import json


# ── Dimensiones de la zona de seguridad frontal (trapezoide) ─────────────────
OBS_DEPTH      = 0.16 * 2   # metros hacia el frente
OBS_WIDTH_NEAR = 0.10 * 2   # ancho cerca del QCar (base ancha del trapezoide)
OBS_WIDTH_FAR  = 0.05 * 2   # ancho al frente, lejos (parte estrecha)
DEAD_ZONE_DEPTH = 0.1 * 2
DEAD_ZONE_WIDTH = 0.18 * 2

# Dirección "enfrente" del QCar dentro del frame del LiDAR (grados).
# El RPLidar está montado con su 0° hacia el costado, así que el frente real
# está a -90°. Mismo valor que 'lidar_yaw_offset_deg' de los dashboards.
FRONT_ANGLE_DEG = -90.0
_COS_F = math.cos(math.radians(FRONT_ANGLE_DEG))
_SIN_F = math.sin(math.radians(FRONT_ANGLE_DEG))


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        # ── Parámetros ───────────────────────────────────────────────────────
        self.declare_parameter('input_topic',   '/qcar/scan')
        self.declare_parameter('output_topic',  '/qcar/scan_republished')
        self.declare_parameter('stats_topic',   '/qcar/lidar_stats')
        self.declare_parameter('obstacle_topic','/qcar/obstacle_detected')
        self.declare_parameter('log_interval',  1.0)
        self.declare_parameter('diag_timeout',  5.0)   # seg sin msgs → warning
        self.publisher_marker = self.create_publisher(Marker, '/qcar/safety_zone', 10)

        input_topic       = self.get_parameter('input_topic').value
        output_topic      = self.get_parameter('output_topic').value
        stats_topic       = self.get_parameter('stats_topic').value
        obstacle_topic    = self.get_parameter('obstacle_topic').value
        self.log_interval = self.get_parameter('log_interval').value
        diag_timeout      = self.get_parameter('diag_timeout').value

        # ── QoS BEST_EFFORT (igual que lidar_qos del QCar) ──────────────────
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # ── Suscriptor ───────────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, qos_be
        )

        # ── Publicadores ─────────────────────────────────────────────────────
        self.publisher_scan     = self.create_publisher(LaserScan, output_topic, qos_be)
        self.publisher_stats    = self.create_publisher(String, stats_topic, 10)
        self.publisher_obstacle = self.create_publisher(Bool, obstacle_topic, 10)

        # ── Dimensiones de la zona (latched): los dashboards las leen ────────
        # QoS TRANSIENT_LOCAL para que cualquier dashboard que se conecte
        # después reciba el último valor sin necesidad de republicar.
        qos_latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.publisher_dims = self.create_publisher(
            Float32MultiArray, '/qcar/safety_zone_dims', qos_latched)
        self._publish_zone_dims()

        # ── Estado ───────────────────────────────────────────────────────────
        self._msg_count  = 0
        self._last_log_t = self.get_clock().now()
        self._first_msg  = False

        # ── Timer de diagnóstico ─────────────────────────────────────────────
        self._diag_timer = self.create_timer(diag_timeout, self._diagnostics)

        # ── Info de arranque ─────────────────────────────────────────────────
        domain_id = os.environ.get('ROS_DOMAIN_ID', '0 (default)')
        self.get_logger().info(
            f'\n'
            f'  ╔══════════════════════════════════════╗\n'
            f'  ║          LidarNode iniciado          ║\n'
            f'  ╠══════════════════════════════════════╣\n'
            f'  ║  Suscrito a  : {input_topic:<22}║\n'
            f'  ║  Republica en: {output_topic:<22}║\n'
            f'  ║  Obstáculo en: {obstacle_topic:<22}║\n'
            f'  ║  ROS_DOMAIN_ID: {domain_id:<21}║\n'
            f'  ╚══════════════════════════════════════╝'
        )

    # ── Diagnóstico automático ────────────────────────────────────────────────
    def _diagnostics(self):
        """Se ejecuta cada diag_timeout segundos. Si no llegó ningún mensaje,
        imprime un aviso con posibles causas."""
        if self._msg_count == 0:
            domain_id = os.environ.get('ROS_DOMAIN_ID', '0 (default)')
            self.get_logger().warn(
                f'\n'
                f'  ⚠  No se han recibido mensajes en /qcar/scan\n'
                f'  Posibles causas:\n'
                f'  1. ROS_DOMAIN_ID incorrecto → actual: {domain_id}\n'
                f'     QCar Blue=114 | Green=115 | Red=116\n'
                f'     Solución: export ROS_DOMAIN_ID=<id_del_qcar>\n'
                f'  2. El nodo lidar_qos no está corriendo en el QCar\n'
                f'     Verificar: ros2 topic list | grep scan\n'
                f'  3. El QCar no está en la misma red\n'
                f'     Verificar: ping <ip_del_qcar>'
            )
        else:
            # Ya recibimos mensajes → cancelar timer
            self._diag_timer.cancel()

    # ── Detección de obstáculo ────────────────────────────────────────────────
    def _obstacle_in_rect(self, msg: LaserScan) -> bool:
        """Devuelve True si algún punto válido cae dentro del trapezoide
        frontal: sólo enfrente del QCar (x > 0), ancho cerca y estrecho lejos."""
        angle = msg.angle_min
        for r in msg.ranges:

            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # Proyección al frame del QCar: fwd = hacia enfrente, lat = lateral
                fwd =  x * _COS_F + y * _SIN_F
                lat = -x * _SIN_F + y * _COS_F

                # Sólo se evalúa la zona frontal (fwd > 0) hasta OBS_DEPTH
                if 0.0 < fwd <= OBS_DEPTH:

                    # Zona muerta central: se ignora (igual que el original,
                    # alineada al frame del LiDAR, SIN rotar)
                    inside_dead_zone = (
                        abs(x) <= DEAD_ZONE_DEPTH / 2.0 and
                        abs(y) <= DEAD_ZONE_WIDTH / 2.0
                    )

                    if not inside_dead_zone:
                        # Semiancho interpolado: ancho cerca (fwd≈0) → estrecho lejos (fwd≈OBS_DEPTH)
                        t = fwd / OBS_DEPTH
                        half_width = (OBS_WIDTH_NEAR +
                                      t * (OBS_WIDTH_FAR - OBS_WIDTH_NEAR)) / 2.0
                        if abs(lat) <= half_width:
                            return True
            angle += msg.angle_increment
        return False

    # ── Callback principal ────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        if not self._first_msg:
            self._first_msg = True
            self.get_logger().info(
                f'✓ Primer mensaje recibido — frame_id: "{msg.header.frame_id}" | '
                f'{len(msg.ranges)} lecturas'
            )
            self._diag_timer.cancel()

        self._msg_count += 1

        valid_ranges = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]

        if valid_ranges:
            r_min  = min(valid_ranges)
            r_max  = max(valid_ranges)
            r_mean = sum(valid_ranges) / len(valid_ranges)
        else:
            r_min = r_max = r_mean = float('nan')

        n_total = len(msg.ranges)
        n_valid = len(valid_ranges)

        # ── Detección de obstáculo y publicación del flag ────────────────────
        obstacle = self._obstacle_in_rect(msg)
        obs_msg = Bool()
        obs_msg.data = obstacle
        self.publisher_obstacle.publish(obs_msg)
        if obstacle:
            self.get_logger().warn(
                f'⛔ Obstáculo detectado en zona frontal (trapezoide '
                f'{OBS_DEPTH*100:.0f} cm prof., {OBS_WIDTH_NEAR*100:.0f}→'
                f'{OBS_WIDTH_FAR*100:.0f} cm ancho)'
            )

        # Modificar frame_id
        msg.header.frame_id = 'lidar_corrected'

        # Republicar
        self.publisher_scan.publish(msg)

        # Estadísticas
        stats = {
            'msg_count'    : self._msg_count,
            'frame_id'     : msg.header.frame_id,
            'n_total'      : n_total,
            'n_valid'      : n_valid,
            'range_min_m'  : round(r_min,  3),
            'range_max_m'  : round(r_max,  3),
            'range_mean_m' : round(r_mean, 3),
            'angle_min_deg': round(math.degrees(msg.angle_min), 2),
            'angle_max_deg': round(math.degrees(msg.angle_max), 2),
            'obstacle'     : obstacle,
        }
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.publisher_stats.publish(stats_msg)
        self._publish_safety_marker(msg.header.stamp, obstacle)

        # Log periódico
        now     = self.get_clock().now()
        elapsed = (now - self._last_log_t).nanoseconds * 1e-9
        if elapsed >= self.log_interval:
            self._last_log_t = now
            self.get_logger().info(
                f'[Scan #{self._msg_count:>6}] '
                f'{n_valid}/{n_total} válidas | '
                f'min={r_min:.3f} m  max={r_max:.3f} m  media={r_mean:.3f} m'
            )

    def _publish_zone_dims(self):
        """Publica (latched) las dimensiones de la zona para que los dashboards
        dibujen exactamente lo que el nodo detecta. Orden del array:
        [OBS_DEPTH, OBS_WIDTH_NEAR, OBS_WIDTH_FAR, DEAD_ZONE_DEPTH, DEAD_ZONE_WIDTH]"""
        dims = Float32MultiArray()
        dims.data = [
            float(OBS_DEPTH), float(OBS_WIDTH_NEAR), float(OBS_WIDTH_FAR),
            float(DEAD_ZONE_DEPTH), float(DEAD_ZONE_WIDTH),
        ]
        self.publisher_dims.publish(dims)
        self.get_logger().info(
            f'Dimensiones publicadas en /qcar/safety_zone_dims → '
            f'trapezoide {OBS_DEPTH:.2f}m prof, {OBS_WIDTH_NEAR:.2f}→{OBS_WIDTH_FAR:.2f}m ancho | '
            f'zona muerta {DEAD_ZONE_DEPTH:.2f}×{DEAD_ZONE_WIDTH:.2f}m'
        )

    def _publish_safety_marker(self, stamp, obstacle: bool):
        # Trapezoide frontal definido en el frame del QCar (fwd, lat):
        # ancho cerca (fwd=0) y estrecho al frente (fwd=OBS_DEPTH).
        near_half = OBS_WIDTH_NEAR / 2.0
        far_half  = OBS_WIDTH_FAR / 2.0
        corners_fwd = [
            (0.0,        near_half),   # A  base ancha, izquierda
            (0.0,       -near_half),   # B  base ancha, derecha
            (OBS_DEPTH, -far_half),    # C  frente estrecho, derecha
            (OBS_DEPTH,  far_half),    # D  frente estrecho, izquierda
        ]
        # Rotar (fwd, lat) → (x, y) del LiDAR según FRONT_ANGLE_DEG.
        corners = [
            (f * _COS_F - l * _SIN_F, f * _SIN_F + l * _COS_F)
            for (f, l) in corners_fwd
        ]

        def _pt(xy):
            p = Point()
            p.x = float(xy[0])
            p.y = float(xy[1])
            p.z = 0.0
            return p

        m = Marker()
        m.header.frame_id = 'lidar_corrected'
        m.header.stamp = stamp
        m.ns = 'safety_zone'
        m.id = 0
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD

        m.pose.orientation.w = 1.0
        m.scale.x = 1.0          # TRIANGLE_LIST usa escala 1 (puntos en metros)
        m.scale.y = 1.0
        m.scale.z = 1.0

        # Dos triángulos para rellenar el trapezoide: (A,B,C) y (A,C,D)
        for idx in (0, 1, 2, 0, 2, 3):
            m.points.append(_pt(corners[idx]))

        # Rojo si hay obstáculo, verde si está libre — alfa 0.35
        m.color.r = 1.0 if obstacle else 0.0
        m.color.g = 0.0 if obstacle else 1.0
        m.color.b = 0.0
        m.color.a = 0.35

        m.lifetime.sec = 0       # 0 = persiste hasta nueva publicación
        self.publisher_marker.publish(m)

        m_dead = Marker()
        m_dead.header.frame_id = 'lidar_corrected'
        m_dead.header.stamp = stamp
        m_dead.ns = 'dead_zone'
        m_dead.id = 1
        m_dead.type = Marker.CUBE
        m_dead.action = Marker.ADD

        m_dead.scale.x = DEAD_ZONE_DEPTH
        m_dead.scale.y = DEAD_ZONE_WIDTH
        m_dead.scale.z = 0.05
        m_dead.pose.orientation.w = 1.0

        m_dead.color.r = 0.0 
        m_dead.color.g = 0.0
        m_dead.color.b = 1.0
        m_dead.color.a = 0.35

        m_dead.lifetime.sec = 0 
        self.publisher_marker.publish(m_dead)


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

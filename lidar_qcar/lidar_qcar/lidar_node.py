#!/usr/bin/env python3
"""
lidar_node.py
=============
Nodo ROS 2 para el sensor LiDAR del QCar.

- Suscribe /qcar/scan  (sensor_msgs/msg/LaserScan)
- Republica en /qcar/scan_republished
- Publica estadísticas JSON en /qcar/lidar_stats
- Diagnóstico automático si no llegan mensajes tras N segundos
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import math
import json


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        # ── Parámetros ───────────────────────────────────────────────────────
        self.declare_parameter('input_topic',   '/qcar/scan')
        self.declare_parameter('output_topic',  '/qcar/scan_republished')
        self.declare_parameter('stats_topic',   '/qcar/lidar_stats')
        self.declare_parameter('log_interval',  1.0)
        self.declare_parameter('diag_timeout',  5.0)   # seg sin msgs → warning

        input_topic       = self.get_parameter('input_topic').value
        output_topic      = self.get_parameter('output_topic').value
        stats_topic       = self.get_parameter('stats_topic').value
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
        self.publisher_scan  = self.create_publisher(LaserScan, output_topic, qos_be)
        self.publisher_stats = self.create_publisher(String, stats_topic, 10)

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
        }
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.publisher_stats.publish(stats_msg)

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

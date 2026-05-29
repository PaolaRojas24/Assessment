#!/usr/bin/env python3
"""
Suscribe /odom (encoder) y el IMU del QCar y guarda ambos en un CSV.

Modos de guardado (parámetro 'mode'):
  'continuous'  – graba cada vez que el carro se mueve >= min_dist metros
  'timed'       – graba a tasa fija (log_rate_hz)

Columnas del CSV:
  run, timestamp_s,
  x_m, y_m, yaw_rad, yaw_deg,   <- odometría / encoder
  vx_enc, wz_enc,                <- velocidad lineal y yaw-rate del encoder
  ax, ay, az,                    <- acelerómetro IMU  (m/s²)
  wx_imu, wy_imu, wz_imu        <- giroscopio IMU    (rad/s)

Para calibrar: bias_gyro_z ≈ mean(wz_imu - wz_enc) en línea recta.
En círculo:    ay ≈ vx² / radio  (aceleración centrípeta).
"""
import math
import csv
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def _yaw_from_quaternion(qx, qy, qz, qw) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')

        self.declare_parameter('output_file',
                               os.path.expanduser('~/qcar_odom_log.csv'))
        self.declare_parameter('mode', 'continuous')   # 'continuous' | 'timed'
        self.declare_parameter('log_rate_hz', 50.0)    # usado en modo 'timed'
        self.declare_parameter('min_dist', 0.02)       # m, usado en modo 'continuous'
        self.declare_parameter('run_label', '')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic',  '/imu/data')

        self._output_file = self.get_parameter('output_file').value
        self._mode        = self.get_parameter('mode').value
        self._min_dist    = self.get_parameter('min_dist').get_parameter_value().double_value
        self._run_label   = self.get_parameter('run_label').value or self._next_run_label()
        self._odom_topic  = self.get_parameter('odom_topic').value
        self._imu_topic   = self.get_parameter('imu_topic').value

        self._last_xy   = None
        self._row_count = 0
        self._latest_odom: Odometry | None = None
        self._latest_imu:  Imu | None      = None

        self._file_existed = os.path.exists(self._output_file)
        self._csv_file = open(self._output_file, 'a', newline='')
        self._writer   = csv.writer(self._csv_file)
        if not self._file_existed:
            self._writer.writerow([
                'run', 'timestamp_s',
                'x_m', 'y_m', 'yaw_rad', 'yaw_deg',
                'vx_enc', 'wz_enc',
                'ax', 'ay', 'az',
                'wx_imu', 'wy_imu', 'wz_imu',
            ])
            self._csv_file.flush()

        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)
        self.create_subscription(Imu, self._imu_topic, self._imu_cb, be_qos)

        if self._mode == 'timed':
            hz = float(self.get_parameter('log_rate_hz').value)
            self.create_timer(1.0 / hz, self._timer_cb)

        # Watchdog: avisa cada 5 s si no llegan datos
        self._watchdog_rows = 0
        self.create_timer(5.0, self._watchdog_cb)

        self.get_logger().info(
            f'odom_logger listo\n'
            f'  modo    : {self._mode}\n'
            f'  corrida : {self._run_label}\n'
            f'  odom    : {self._odom_topic}\n'
            f'  imu     : {self._imu_topic}\n'
            f'  archivo : {self._output_file}'
        )

    # ── helpers ────────────────────────────────────────────────────────────────

    def _next_run_label(self) -> str:
        if not os.path.exists(self._output_file):
            return 'run_1'
        with open(self._output_file, 'r') as f:
            existing = {row[0] for i, row in enumerate(csv.reader(f)) if i > 0 and row}
        n = len(existing) + 1
        return f'run_{n}'

    def _imu_fields(self):
        if self._latest_imu is None:
            return ['', '', '', '', '', '']
        la = self._latest_imu.linear_acceleration
        av = self._latest_imu.angular_velocity
        return [
            round(la.x, 6), round(la.y, 6), round(la.z, 6),
            round(av.x, 6), round(av.y, 6), round(av.z, 6),
        ]

    def _write_row(self, msg: Odometry):
        p   = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        tw  = msg.twist.twist
        yaw = _yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w)
        t   = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self._writer.writerow([
            self._run_label,
            round(t,   6),
            round(p.x, 6),
            round(p.y, 6),
            round(yaw, 6),
            round(math.degrees(yaw), 4),
            round(tw.linear.x,  6),
            round(tw.angular.z, 6),
            *self._imu_fields(),
        ])
        self._csv_file.flush()
        self._row_count += 1

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg

        if self._mode != 'continuous':
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last_xy is not None:
            dx = x - self._last_xy[0]
            dy = y - self._last_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self._min_dist:
                return

        self._last_xy = (x, y)
        self._write_row(msg)

    def _imu_cb(self, msg: Imu):
        self._latest_imu = msg

    def _timer_cb(self):
        if self._latest_odom is not None:
            self._write_row(self._latest_odom)

    def _watchdog_cb(self):
        if self._latest_odom is None:
            self.get_logger().warn(
                f'Sin datos de /odom — verifica que odom_kalman corre en el QCar '
                f'(topic: {self._odom_topic})'
            )
        else:
            new = self._row_count - self._watchdog_rows
            self.get_logger().info(f'{new} filas guardadas en los últimos 5 s '
                                   f'(total: {self._row_count})')
            self._watchdog_rows = self._row_count

    # ── shutdown ───────────────────────────────────────────────────────────────

    def close(self):
        self._csv_file.flush()
        self._csv_file.close()
        self.get_logger().info(
            f'odom_logger cerrado — {self._row_count} filas guardadas en '
            f'{self._output_file}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

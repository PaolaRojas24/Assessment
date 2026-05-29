#!/usr/bin/env python3
"""
EKF 2D Odometria — fusiona encoder (/qcar/velocity) + IMU (/imu/data)

Estado:  x = [px, py, theta]
Control: u = [v (encoder m/s), omega (giroscopio z rad/s, ya corregido por imu_publisher)]

Modelo de movimiento (bicicleta simplificado):
    px_new    = px    + v * cos(theta) * dt
    py_new    = py    + v * sin(theta) * dt
    theta_new = theta + omega * dt

Sin medicion de actualizacion (no hay GPS/landmarks),
el filtro hace dead-reckoning con propagacion de covarianza.
El bias del giroscopio Z se corrige en imu_publisher, no aqui.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
import math


# ---------------------------------------------------------------------------
# EKF
# ---------------------------------------------------------------------------

class EKF2D:
    def __init__(self, q_pos: float, q_yaw: float):
        self.x = np.zeros(3)               # [px, py, theta]
        self.P = np.diag([0.1, 0.1, 0.05])

        # Ruido de proceso calibrado:
        #   q_pos → incertidumbre en posicion (m²/s)
        #   q_yaw → std_gyro_z² = 0.087² ≈ 0.0076 (rad²/s)
        self.q_pos = q_pos
        self.q_yaw = q_yaw

    def predict(self, v: float, omega: float, dt: float):
        if dt <= 0.0 or dt > 0.5:
            return

        th = self.x[2]

        # --- propagacion del estado ---
        self.x[0] += v * math.cos(th) * dt
        self.x[1] += v * math.sin(th) * dt
        self.x[2] += omega * dt
        self.x[2]  = math.atan2(math.sin(self.x[2]), math.cos(self.x[2]))

        # --- Jacobiano del modelo ---
        F = np.array([
            [1.0, 0.0, -v * math.sin(th) * dt],
            [0.0, 1.0,  v * math.cos(th) * dt],
            [0.0, 0.0,  1.0],
        ])

        # --- ruido de proceso discreto ---
        Q = np.diag([self.q_pos, self.q_pos, self.q_yaw]) * dt

        # --- propagacion de covarianza ---
        self.P = F @ self.P @ F.T + Q

    def pose(self):
        return self.x.copy(), self.P.copy()


# ---------------------------------------------------------------------------
# Nodo ROS2
# ---------------------------------------------------------------------------

class OdomKalman(Node):
    def __init__(self):
        super().__init__('odom_kalman')

        # Ruido de proceso — q_yaw viene de std_gyro_z² medido en calibración
        self.declare_parameter('q_pos', 0.01)
        self.declare_parameter('q_yaw', 0.0076)   # 0.087² rad²/s

        q_pos = self.get_parameter('q_pos').get_parameter_value().double_value
        q_yaw = self.get_parameter('q_yaw').get_parameter_value().double_value

        self.ekf = EKF2D(q_pos, q_yaw)
        self.last_imu_stamp: Time | None = None
        self.latest_v     = 0.0
        self.latest_omega = 0.0

        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(Vector3Stamped, '/qcar/velocity', self._vel_cb, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info(
            f'odom_kalman listo — esperando /imu/data y /qcar/velocity\n'
            f'  q_pos = {q_pos}  q_yaw = {q_yaw}'
        )

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _vel_cb(self, msg: Vector3Stamped):
        self.latest_v = msg.vector.x

    def _imu_cb(self, msg: Imu):
        # Usar el stamp del mensaje para dt, no el reloj del sistema
        stamp = Time.from_msg(msg.header.stamp)

        if self.last_imu_stamp is None:
            self.last_imu_stamp = stamp
            return

        dt = (stamp - self.last_imu_stamp).nanoseconds * 1e-9
        self.last_imu_stamp = stamp

        # El bias ya fue corregido en imu_publisher — usar directo
        self.latest_omega = msg.angular_velocity.z

        self.ekf.predict(self.latest_v, self.latest_omega, dt)
        self._publish(msg.header.stamp)

    # -----------------------------------------------------------------------
    # Publicar Odometry + TF
    # -----------------------------------------------------------------------

    def _publish(self, stamp):
        x, P = self.ekf.pose()
        px, py, th = x

        qz = math.sin(th * 0.5)
        qw = math.cos(th * 0.5)

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = px
        odom.pose.pose.position.y    = py
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0]  = P[0, 0]
        cov[1]  = P[0, 1]
        cov[6]  = P[1, 0]
        cov[7]  = P[1, 1]
        cov[35] = P[2, 2]
        odom.pose.covariance = cov

        odom.twist.twist.linear.x  = self.latest_v
        odom.twist.twist.angular.z = self.latest_omega

        tcov = [0.0] * 36
        tcov[0]  = self.ekf.q_pos
        tcov[35] = self.ekf.q_yaw
        odom.twist.covariance = tcov

        self.pub_odom.publish(odom)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = OdomKalman()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Launcher de odometria del QCar.

Lanza:
  - imu_publisher  -> lee la ESP32 por /dev/ttyUSB0 y publica /imu/data
  - odom_kalman    -> EKF 2D que fusiona /imu/data + /qcar/velocity -> /odom
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package='qcar_odom',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen',
    )

    odom_node = Node(
        package='qcar_odom',
        executable='odom_kalman',
        name='odom_kalman',
        output='screen',
    )

    return LaunchDescription([
        imu_node,
        odom_node,
    ])

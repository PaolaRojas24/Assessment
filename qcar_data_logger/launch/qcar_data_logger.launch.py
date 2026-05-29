"""
Lanza el nodo odom_logger del QCar.

Graba odometría + IMU en CSV mientras el robot esté en movimiento.
Úsalo junto con teleop (u otro nodo de control) en una terminal separada.

Parámetros:
  output_file   ruta del CSV                         default: ~/qcar_odom_log.csv
  mode          'continuous' | 'timed'               default: continuous
  log_rate_hz   Hz de grabación (modo timed)         default: 50.0
  min_dist      distancia mínima entre registros (m) default: 0.02
  run_label     etiqueta de corrida                  default: auto (run_1, run_2, …)
  imu_topic     topic del IMU                        default: /imu/data

Ejemplos:
  ros2 launch qcar_data_logger qcar_data_logger.launch.py
  ros2 launch qcar_data_logger qcar_data_logger.launch.py output_file:=/tmp/teleop.csv
  ros2 launch qcar_data_logger qcar_data_logger.launch.py mode:=timed log_rate_hz:=50.0
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('output_file',
                              default_value=os.path.expanduser('~/qcar_odom_log.csv')),
        DeclareLaunchArgument('mode',        default_value='continuous'),
        DeclareLaunchArgument('log_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('min_dist',    default_value='0.02'),
        DeclareLaunchArgument('run_label',   default_value=''),
        DeclareLaunchArgument('imu_topic',   default_value='/imu/data'),
    ]

    logger_node = Node(
        package='qcar_data_logger',
        executable='odom_logger',
        name='odom_logger',
        output='screen',
        parameters=[{
            'output_file':  LaunchConfiguration('output_file'),
            'mode':         LaunchConfiguration('mode'),
            'log_rate_hz':  LaunchConfiguration('log_rate_hz'),
            'min_dist':     LaunchConfiguration('min_dist'),
            'run_label':    LaunchConfiguration('run_label'),
            'imu_topic':    LaunchConfiguration('imu_topic'),
        }],
    )

    return LaunchDescription(args + [logger_node])

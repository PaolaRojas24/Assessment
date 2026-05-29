"""
Lanza el QCar en círculo o cuadrado y registra la odometría simultáneamente.

Parámetros de trayectoria (test_userCommand):
  drive_mode     'circle' | 'square'                    default: circle
  is_sim         False (real) | True (simulador)         default: False
  throttle       velocidad hacia adelante                default: 0.06
  steering       ángulo de giro fijo — solo círculo      default: 0.25
  side_duration  segundos por lado recto — solo cuadrado default: 10.0
  turn_duration  segundos por giro 90° — solo cuadrado   default: 7.3
  turn_steering  steering en las esquinas — solo cuadrado default: 0.45

Parámetros de logger (odom_logger):
  output_file    ruta del CSV                            default: ~/qcar_circle_log.csv
  log_mode       'continuous' | 'timed'                  default: timed
  log_rate_hz    Hz de grabación (solo modo timed)       default: 50.0
  imu_topic      topic del IMU                           default: /imu/data

Ejemplos:
  ros2 launch test_pck circle_log.launch.py
  ros2 launch test_pck circle_log.launch.py drive_mode:=square output_file:=/tmp/cuadrado.csv
  ros2 launch test_pck circle_log.launch.py steering:=-0.25 output_file:=/tmp/derecha.csv
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # trayectoria
        DeclareLaunchArgument('drive_mode',     default_value='square'),
        DeclareLaunchArgument('is_sim',         default_value='false'),
        DeclareLaunchArgument('throttle',       default_value='0.06'),
        DeclareLaunchArgument('steering',       default_value='0.25'),
        DeclareLaunchArgument('side_duration',      default_value='3.0'),
        DeclareLaunchArgument('turn_duration',      default_value='9.5'),
        DeclareLaunchArgument('turn_steering',      default_value='0.5'),
        DeclareLaunchArgument('straight_steering',  default_value='-0.06'),
        DeclareLaunchArgument('num_segments',       default_value='3'),
        # logger
        DeclareLaunchArgument('output_file',
                              default_value=os.path.expanduser('~/qcar_circle_log.csv')),
        DeclareLaunchArgument('log_mode',       default_value='timed'),
        DeclareLaunchArgument('log_rate_hz',    default_value='50.0'),
        DeclareLaunchArgument('imu_topic',      default_value='/imu/data'),
    ]

    drive_node = Node(
        package='test_pck',
        executable='test_userCommand',
        name='drive_control',
        output='screen',
        parameters=[{
            'is_sim':           LaunchConfiguration('is_sim'),
            'mode':             LaunchConfiguration('drive_mode'),
            'throttle':         LaunchConfiguration('throttle'),
            'steering':         LaunchConfiguration('steering'),
            'side_duration':    LaunchConfiguration('side_duration'),
            'turn_duration':    LaunchConfiguration('turn_duration'),
            'turn_steering':    LaunchConfiguration('turn_steering'),
            'straight_steering': LaunchConfiguration('straight_steering'),
            'num_segments':      LaunchConfiguration('num_segments'),
        }],
    )

    logger_node = Node(
        package='qcar_data_logger',
        executable='odom_logger',
        name='odom_logger',
        output='screen',
        parameters=[{
            'output_file':  LaunchConfiguration('output_file'),
            'mode':         LaunchConfiguration('log_mode'),
            'log_rate_hz':  LaunchConfiguration('log_rate_hz'),
            'imu_topic':    LaunchConfiguration('imu_topic'),
            'run_label':    '',
        }],
    )

    return LaunchDescription(args + [drive_node, logger_node])

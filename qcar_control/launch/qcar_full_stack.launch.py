"""
Full-stack launcher for the QCar.

Brings up everything in dependency order:
  - lidar_qcar/lidar_node            -- /qcar/scan -> /qcar/obstacle_detected
  - qcar_lane_perception             -- /qcar/csi_front -> overlay/BEV/mask + target
  - qcar_control                     -- lane_follower + command_mux
  - keepalives for /qcar/scan, /qcar/stateBattery, /qcar/obstacle_detected, /qcar/safe_stop_active
  - qcar_opencv_dashboard (optional, default on)

Required on the QCar side (separate machine):
    ros2 launch ROSes_pkg qcar_red_lf.launch.py nodes:='csi_lf,qcar,lidar_qos'

Run on the laptop:
    ros2 launch qcar_control qcar_full_stack.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    perception_pkg = get_package_share_directory('qcar_lane_perception')
    control_pkg    = get_package_share_directory('qcar_control')
    opencv_pkg     = get_package_share_directory('qcar_opencv_dashboard')

    dashboard_arg = DeclareLaunchArgument(
        'dashboard', default_value='true',
        description='Launch the OpenCV dashboard window.',
    )
    initial_mode_arg = DeclareLaunchArgument(
        'initial_mode', default_value='auto',
        description='command_mux initial mode (auto / manual / off).',
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_pkg, 'launch', 'lane_perception.launch.py')
        ),
    )
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, 'launch', 'qcar_control.launch.py')
        ),
        launch_arguments={
            'initial_mode': LaunchConfiguration('initial_mode'),
        }.items(),
    )
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(opencv_pkg, 'launch', 'dashboard.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    lidar_node = Node(
        package='lidar_qcar',
        executable='lidar_node',
        name='lidar_obstacle_node',
        output='screen',
    )

    # Keepalives for the topics that aren't in the perception launch.
    keepalives = [
        ExecuteProcess(
            cmd=['ros2', 'topic', 'hz', topic],
            name=f'{name}_hz_keepalive',
            output='log',
        )
        for name, topic in [
            ('scan',       '/qcar/scan'),
            ('battery',    '/qcar/stateBattery'),
            ('obstacle',   '/qcar/obstacle_detected'),
            ('safe_stop',  '/qcar/safe_stop_active'),
        ]
    ]

    return LaunchDescription([
        dashboard_arg,
        initial_mode_arg,
        perception,
        lidar_node,
        control,
        *keepalives,
        dashboard,
    ])

"""
Full QCar driving + visualization stack.

Same control pipeline as line_follower_real_fast.launch.py
  CompressedImage  -> image_converter_fast (depth=1)
                   -> lane_detector_fast   (publishes target + overlay + BEV + mask)
                   -> lane_follower_q      (pure-pursuit, publishes /qcar/user_command)

Plus the unified dashboard, including the LiDAR tile.

Required on the QCar side:
    ros2 launch ROSes_pkg qcar_red_lf.launch.py nodes:='csi_lf,qcar,rgbd,lidar_qos'
                                                                       ^^^^^^^^^
                                       <-- add lidar_qos so /qcar/scan is published
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('line_follower_real')

    default_detector_params = os.path.join(
        pkg_dir, 'config', 'lane_detector_fast_params.yaml'
    )
    default_follower_params = os.path.join(
        pkg_dir, 'config', 'lane_follower_real_params.yaml'
    )

    detector_params_arg = DeclareLaunchArgument(
        'detector_params_file', default_value=default_detector_params,
        description='Lane detector parameter file.',
    )
    follower_params_arg = DeclareLaunchArgument(
        'follower_params_file', default_value=default_follower_params,
        description='Lane follower parameter file.',
    )
    dashboard_arg = DeclareLaunchArgument(
        'dashboard', default_value='true',
        description='Launch the unified QCar dashboard window.',
    )
    lidar_yaw_arg = DeclareLaunchArgument(
        'lidar_yaw_offset_deg', default_value='-90.0',
        description=('Yaw rotation (deg) applied to the LiDAR plot so the '
                     'car forward axis points UP. Try -90, 0, 90, or 180 '
                     "if the front of the scan isn't where you expect."),
    )
    lidar_range_arg = DeclareLaunchArgument(
        'lidar_range_m', default_value='2.0',
        description='Max range (m) drawn on the LiDAR tile.',
    )

    img_converter_node = Node(
        package='line_follower_real',
        executable='image_converter_fast',
        name='image_converter_fast',
        output='screen',
        parameters=[
            {'subscribe_topic': '/qcar/csi_front'},
            {'publish_topic': '/qcar/decompressed/csi_front'},
        ],
    )

    lane_detector_node = Node(
        package='line_follower_real',
        executable='lane_detector_fast',
        name='lane_detector_fast',
        output='screen',
        parameters=[LaunchConfiguration('detector_params_file')],
    )

    lane_follower_node = Node(
        package='control_helpers_pkg',
        executable='lane_follower_q',
        name='lane_follower_q',
        output='screen',
        parameters=[LaunchConfiguration('follower_params_file')],
    )

    # `ros2 topic hz` keepalives. Each one keeps the DDS local-discovery state
    # warm for one topic; without them the dashboard tile occasionally freezes
    # even though the publisher is firing.
    keepalives = [
        ExecuteProcess(
            cmd=['ros2', 'topic', 'hz', topic],
            name=f'{name}_hz_keepalive',
            output='log',
        )
        for name, topic in [
            ('csi',     '/qcar/csi_front'),
            ('overlay', '/qcar/line_follower/overlay'),
            ('bev',     '/qcar/line_follower/bev'),
            ('mask',    '/qcar/line_follower/mask'),
            ('scan',    '/qcar/scan'),
        ]
    ]

    dashboard_node = Node(
        package='line_follower_real',
        executable='dashboard',
        name='qcar_dashboard',
        output='screen',
        parameters=[{
            'lidar_yaw_offset_deg': LaunchConfiguration('lidar_yaw_offset_deg'),
            'lidar_range_m': LaunchConfiguration('lidar_range_m'),
        }],
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    return LaunchDescription([
        detector_params_arg,
        follower_params_arg,
        dashboard_arg,
        lidar_yaw_arg,
        lidar_range_arg,
        img_converter_node,
        lane_detector_node,
        lane_follower_node,
        *keepalives,
        dashboard_node,
    ])

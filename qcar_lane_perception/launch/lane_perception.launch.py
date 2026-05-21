"""
Perception-only launch: CompressedImage decode + lane detector + the
two `ros2 topic hz` keepalives that keep the laptop's local DDS pipe warm.

Outputs:
  /qcar/decompressed/csi_front          sensor_msgs/Image
  /qcar/line_follower/overlay           sensor_msgs/Image
  /qcar/line_follower/bev               sensor_msgs/Image
  /qcar/line_follower/mask              sensor_msgs/Image
  /lane_target_point_m                  std_msgs/Float32MultiArray

This launch only handles the perception side. The actual driving command
comes out of qcar_control (lane_follower_q + command_mux).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('qcar_lane_perception')

    default_detector_params = os.path.join(
        pkg_dir, 'config', 'lane_detector_fast_params.yaml'
    )

    detector_params_arg = DeclareLaunchArgument(
        'detector_params_file', default_value=default_detector_params,
        description='Lane detector parameter file.',
    )

    img_converter_node = Node(
        package='qcar_lane_perception',
        executable='image_converter_fast',
        name='image_converter_fast',
        output='screen',
        parameters=[
            {'subscribe_topic': '/qcar/csi_front'},
            {'publish_topic': '/qcar/decompressed/csi_front'},
        ],
    )

    lane_detector_node = Node(
        package='qcar_lane_perception',
        executable='lane_detector_fast',
        name='lane_detector_fast',
        output='screen',
        parameters=[LaunchConfiguration('detector_params_file')],
    )

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
        ]
    ]

    return LaunchDescription([
        detector_params_arg,
        img_converter_node,
        lane_detector_node,
        *keepalives,
    ])

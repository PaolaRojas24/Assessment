"""
Perception-only launch: CompressedImage decode + lane detector + the
two `ros2 topic hz` keepalives that keep the laptop's local DDS pipe warm.

All node parameters are hardcoded inside the Python modules themselves
(no YAML overrides), so this launch only spawns the processes.

Outputs:
  /qcar/decompressed/csi_front          sensor_msgs/Image
  /qcar/line_follower/overlay           sensor_msgs/Image
  /qcar/line_follower/bev               sensor_msgs/Image
  /qcar/line_follower/mask              sensor_msgs/Image
  /lane_target_point_m                  std_msgs/Float32MultiArray

This launch only handles the perception side. The actual driving command
comes out of qcar_control (lane_follower_q + command_mux).
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    img_converter_node = Node(
        package='qcar_lane_perception',
        executable='image_converter',
        name='image_converter',
        output='screen',
    )

    lane_detector_node = Node(
        package='qcar_lane_perception',
        executable='lane_detector',
        name='lane_detector',
        output='screen',
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
        img_converter_node,
        lane_detector_node,
        *keepalives,
    ])

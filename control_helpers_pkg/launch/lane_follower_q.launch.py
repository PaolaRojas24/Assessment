import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('control_helpers_pkg'),
        'config',
        'lane_follower_q_params.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to lane follower parameter YAML file.',
    )

    lane_follower_node = Node(
        package='control_helpers_pkg',
        executable='lane_follower_q',
        name='lane_follower_q',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        lane_follower_node,
    ])

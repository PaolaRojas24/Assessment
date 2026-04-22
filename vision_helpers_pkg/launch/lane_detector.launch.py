from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('vision_helpers_pkg'),
        'config',
        'lane_detector_params.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to lane detector ROS parameters YAML file.'
    )

    lane_detector_node = Node(
        package='vision_helpers_pkg',
        executable='lane_detector',
        name='lane_detector',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        lane_detector_node,
    ])

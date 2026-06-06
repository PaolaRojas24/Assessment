"""
Control-only launch: lane_follower_q (autonomous) + command_mux.

Both source nodes that drive the QCar live here. The autonomous
lane_follower_q reads /lane_target_point_m and publishes
/lane_follower/raw_cmd; the mux then arbitrates between that and the
safety flags to produce /qcar/user_command.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('qcar_control')
    default_follower_params = os.path.join(
        pkg_dir, 'config', 'lane_follower_real_params.yaml'
    )

    follower_params_arg = DeclareLaunchArgument(
        'follower_params_file', default_value=default_follower_params,
        description='Lane follower parameter file.',
    )
    initial_mode_arg = DeclareLaunchArgument(
        'initial_mode', default_value='auto',
        description='Initial command_mux mode: auto / off.',
    )
    initial_source_arg = DeclareLaunchArgument(
        'initial_source', default_value='lane',
        description='Initial command_mux source: lane / overtake.',
    )
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='0.1',
        description='Hard cap on |throttle| (m/s) enforced by the mux.',
    )

    lane_follower_node = Node(
        package='control_helpers_pkg',
        executable='lane_follower_q',
        name='lane_follower_q',
        output='screen',
        parameters=[LaunchConfiguration('follower_params_file')],
    )

    command_mux_node = Node(
        package='qcar_control',
        executable='command_mux',
        name='command_mux',
        output='screen',
        parameters=[{
            'initial_mode': LaunchConfiguration('initial_mode'),
            'initial_source': LaunchConfiguration('initial_source'),
            'max_speed': ParameterValue(
                LaunchConfiguration('max_speed'), value_type=float),
        }],
    )

    return LaunchDescription([
        follower_params_arg,
        initial_mode_arg,
        initial_source_arg,
        max_speed_arg,
        lane_follower_node,
        command_mux_node,
    ])

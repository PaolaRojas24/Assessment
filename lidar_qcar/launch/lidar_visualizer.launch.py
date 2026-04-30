import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_path('qcar_description'),
        'urdf', 'qcar_system.urdf.xacro'
    )
    rviz_config_path = os.path.join(
        get_package_share_path('lidar_qcar'),
        'config', 'lidar_view.rviz'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' use_sim:=false']),
        value_type=str
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    lidar_node = Node(
        package='lidar_qcar',
        executable='lidar_node',
        output='screen',
        parameters=[{
            'input_topic' : '/qcar/scan',
            'log_interval': 1.0,
            'diag_timeout': 5.0,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        rsp_node,
        jsp_node,
        lidar_node,
        rviz_node,
    ])

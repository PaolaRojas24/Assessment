from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():
    gazebo_launch_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')

    gazebo_world_path = os.path.join(get_package_share_path('prius_bringup'),
                                     'worlds', 'prius_world.sdf')
    
    gazebo_config_path = os.path.join(get_package_share_path('prius_bringup'),
                                     'config', 'gz_bridge.yaml')
    
    models_path = os.path.join(get_package_share_path('prius_bringup'), 'models')
    
    display_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_launch_path,
            '/gz_sim.launch.py'
        ]), launch_arguments={'gz_args':f'{gazebo_world_path} -r'}.items()
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': gazebo_config_path}]
    )

    return LaunchDescription([
        # Set the Gazebo resource path to include your custom models
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[models_path, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        ),
        display_robot_gazebo,
        bridge_node
    ])

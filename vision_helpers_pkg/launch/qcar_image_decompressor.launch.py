from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def nodes_to_execute(context, *args, **kwargs):

    platform = str(LaunchConfiguration('platform').perform(context))

    if platform == 'red':
        nodes_list = ['ic_front', 'ic_back', 'ic_right', 'ic_rgbd']
    else:
        nodes_list = ['ic_front', 'ic_back', 'ic_right', 'ic_left', 'ic_rgbd']

    parameters = {
        "ic_front": [
            {'subscribe_topic': '/qcar/csi_front'},
            {'publish_topic': '/qcar/decompressed/csi_front'}
        ],
        "ic_back": [
            {'subscribe_topic': '/qcar/csi_back'},
            {'publish_topic': '/qcar/decompressed/csi_back'}
        ],
        "ic_right": [
            {'subscribe_topic': '/qcar/csi_right'},
            {'publish_topic': '/qcar/decompressed/csi_right'}
        ],
        "ic_left": [
            {'subscribe_topic': '/qcar/csi_left'},
            {'publish_topic': '/qcar/decompressed/csi_left'}
        ],
        "ic_rgbd": [
            {'subscribe_topic': '/qcar/rgbd_color'},
            {'publish_topic': '/qcar/decompressed/rgbd_color'}
        ],
    }
    
    launch_nodes = [
        Node(
            package="vision_helpers_pkg",
            executable="img_converter",
            name=node_name,
            output="screen",
            parameters=parameters.get(node_name, [])
        ) for node_name in nodes_list
    ]
    return launch_nodes

def generate_launch_description():
    # Declare launch arguments for selecting the qcar platform
    platform_arg = DeclareLaunchArgument(
        'platform', 
        default_value= 'blue',
        description='Selct one of the 3 qcar platforms: red, blue, or green. The red platform has only 4 cameras (front, back, right, and rgbd), while the blue and green platforms have 5 cameras (front, back, right, left, and rgbd).'
    )

    return LaunchDescription([
        platform_arg
    ] + [OpaqueFunction(function=nodes_to_execute)])
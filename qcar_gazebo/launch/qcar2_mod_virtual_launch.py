# This is the launch file that starts up the basic QCar2 nodes

import subprocess

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, TimerAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import (OnProcessExit, OnProcessStart)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def nodes_to_execute(context, *args, **kwargs):
    lidar_node = Node(
            package='qcar2_nodes',
            executable='lidar',
            name='Lidar',
            parameters=[{"device_type":"virtual"}]
        )
    
    realsense_camera_node = Node(
            package='qcar2_nodes',
            executable='rgbd',
            name='RealsenseCamera',
            parameters=[{"device_type":"virtual"},
                        {"frame_width_rgb":640},
                        {"frame_height_rgb":480},
                        {"frame_width_depth":640},
                        {"frame_height_depth":480}]
        )
    
    csi_num_param = LaunchConfiguration('csi_num')

    csi_num = int(csi_num_param.perform(context))

    csi_camera_node = Node(
            package='qcar2_nodes',
            executable='csi',
            name='csi_camera',
            parameters=[{"device_type":"virtual"},
                        {"frame_width":410},
                        {"frame_height":205},
                        {"frame_rate":15.0},
                        {"camera_num":csi_num}]
        )
    
    qcar2_hardware = Node(
            package='qcar2_nodes',
            executable='qcar2_hardware',
            name='qcar2_hardware',
            parameters=[{"device_type":"virtual"}]

        )
    
    return [
        lidar_node,
        realsense_camera_node,
        csi_camera_node,
        qcar2_hardware,
        ]

def generate_launch_description():
        
    # Declare launch arguments for Xacro parameters
    csi_arg = DeclareLaunchArgument(
        'csi_num', 
        default_value= '3',
        description='Parameter select the csi camera. right:0, rear:1, left:2, front:3'
    )
     
    return LaunchDescription([
        csi_arg
        ] + [OpaqueFunction(function=nodes_to_execute)])

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(package="joy", executable="joy_node")
    
    joystick_controller_node = Node(package='control_helpers_pkg',
                                    executable='holonomic_joy_control',
                                    output='screen')

    return LaunchDescription([joy_node,
                              joystick_controller_node])
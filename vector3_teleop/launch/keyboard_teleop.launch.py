"""
Launches the terminal keyboard teleop. Make sure to run this in an
interactive terminal (Terminator, gnome-terminal) -- it reads stdin
directly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_topic_arg = DeclareLaunchArgument(
        'cmd_topic', default_value='/teleop/cmd',
        description='Topic to publish manual commands on. Must match '
                    'command_mux.manual_in_topic.',
    )
    allow_reverse_arg = DeclareLaunchArgument(
        'allow_reverse', default_value='false',
        description='Set true to allow negative throttle (reverse).',
    )

    teleop_node = Node(
        package='vector3_teleop',
        executable='keyboard_teleop',
        name='vector3_keyboard_teleop',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'cmd_topic':     LaunchConfiguration('cmd_topic'),
            'allow_reverse': LaunchConfiguration('allow_reverse'),
        }],
    )

    return LaunchDescription([
        cmd_topic_arg,
        allow_reverse_arg,
        teleop_node,
    ])

"""
Launches the OpenCV dashboard alone (no perception, no control).

Run it on top of qcar_control / qcar_lane_perception (or use
qcar_full_stack.launch.py from qcar_control to bring everything up at once).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dashboard_node = Node(
        package='qcar_opencv_dashboard',
        executable='dashboard',
        name='qcar_opencv_dashboard',
        output='screen',
    )
    return LaunchDescription([dashboard_node])

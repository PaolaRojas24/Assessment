"""
Launches the PyQt5 dashboard alone (no line-follower stack).

Run the QCar + lane-follower side via qcar_interface.launch.py with
dashboard:=false, then this launcher on top:

    ros2 launch line_follower_real qcar_interface.launch.py dashboard:=false
    ros2 launch qcar_pyqt_dashboard qcar_pyqt_dashboard.launch.py

Both this dashboard and the OpenCV one publish /qcar/safe_stop_active when
their button is pressed. safety_mux just consumes the most-recent Bool.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    yaw_arg = DeclareLaunchArgument(
        'lidar_yaw_offset_deg', default_value='-90.0',
        description=('Yaw rotation (deg) applied to the LiDAR plot so the '
                     "car's forward axis points UP. Try -90 / 0 / 90 / 180."),
    )
    range_arg = DeclareLaunchArgument(
        'lidar_range_m', default_value='2.0',
        description='Max range (m) drawn on the LiDAR + Lidar-detection tiles.',
    )
    team_arg = DeclareLaunchArgument(
        'team_name', default_value='guns-n-ROSes',
        description='Team name shown in the title bar.',
    )

    dashboard_node = Node(
        package='qcar_pyqt_dashboard',
        executable='dashboard',
        name='qcar_pyqt_dashboard',
        output='screen',
        parameters=[{
            'lidar_yaw_offset_deg': LaunchConfiguration('lidar_yaw_offset_deg'),
            'lidar_range_m':        LaunchConfiguration('lidar_range_m'),
            'team_name':            LaunchConfiguration('team_name'),
        }],
    )

    return LaunchDescription([
        yaw_arg,
        range_arg,
        team_arg,
        dashboard_node,
    ])

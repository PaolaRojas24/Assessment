"""
Launcher de visualizacion de odometria (lado laptop).

Suscribe /odom (publicado por qcar_odom en el carro) y:
  - publica nav_msgs/Path con la traza en /odom_path
  - abre RViz2 con un layout que muestra Odometry + Path + TF
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar_odom_viz')
    rviz_cfg  = os.path.join(pkg_share, 'config', 'odom_view.rviz')

    odom_path_node = Node(
        package='qcar_odom_viz',
        executable='odom_path',
        name='odom_path',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_odom',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([
        odom_path_node,
        rviz_node,
    ])

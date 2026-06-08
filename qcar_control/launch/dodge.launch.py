"""
Esquive por trayectoria en S (obstacle_dodge) + RViz.

Arranca:
  - obstacle_dodge (esquive sigmoide, sin lógica de curva),
  - static_transform_publisher map->lidar_corrected (Fixed Frame para RViz),
  - RViz con overtake.rviz (reusa LaserScan + MarkerArray /overtake/markers).

NO arranca el lidar ni el seguidor: el lidar_node debe correr aparte
(/qcar/scan_republished), y el seguidor + mux desde qcar_full_stack.

Uso:
  ros2 launch qcar_control dodge.launch.py
  ros2 launch qcar_control dodge.launch.py pass_side:=-1     # esquivar por la derecha
  ros2 launch qcar_control dodge.launch.py rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('qcar_control')
    rviz_cfg = os.path.join(pkg_dir, 'rviz', 'overtake.rviz')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                     description='Lanzar RViz.')
    static_tf_arg = DeclareLaunchArgument(
        'static_tf', default_value='true',
        description='Publicar map->lidar_corrected (false si el QCar ya lo da).')
    pass_side_arg = DeclareLaunchArgument(
        'pass_side', default_value='1',
        description='+1 esquivar por izquierda, -1 por derecha.')

    dodge = Node(
        package='qcar_control', executable='obstacle_dodge', name='obstacle_dodge',
        output='screen',
        parameters=[{
            'pass_side': ParameterValue(LaunchConfiguration('pass_side'),
                                        value_type=int),
            'publish_markers': True,
        }],
    )

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lidar_corrected_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lidar_corrected'],
        condition=IfCondition(LaunchConfiguration('static_tf')),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg, static_tf_arg, pass_side_arg,
        dodge, static_tf, rviz,
    ])

"""
Visualización del supervisor de rebase en RViz.

Arranca:
  - overtake_supervisor (dry_run por defecto: NO toca la conducción),
  - un static_transform_publisher para que el frame 'lidar_corrected'
    exista en TF (sin esto RViz no dibuja nada),
  - RViz con el config overtake.rviz (LaserScan + MarkerArray ya cargados).

Este launch NO arranca el lidar ni el seguidor de línea: esos corren
aparte (lidar_node republica el scan y publica /qcar/scan_republished).

Uso:
  ros2 launch qcar_control overtake_viz.launch.py
  # armado real (ejecuta el rebase):
  ros2 launch qcar_control overtake_viz.launch.py dry_run:=false
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

    dry_run_arg = DeclareLaunchArgument(
        'dry_run', default_value='true',
        description='true = solo percepción/markers, no toca la conducción.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Lanzar RViz con el config de rebase.',
    )
    static_tf_arg = DeclareLaunchArgument(
        'static_tf', default_value='true',
        description=('Publicar map->lidar_corrected. Ponlo en false si el '
                     'QCar ya publica ese frame en TF (evita conflicto).'),
    )
    check_curve_arg = DeclareLaunchArgument(
        'check_curve', default_value='true',
        description='false = ignora el veto de curva (probar rebase en recta).',
    )

    supervisor = Node(
        package='qcar_control',
        executable='overtake_supervisor',
        name='overtake_supervisor',
        output='screen',
        parameters=[{
            'dry_run': ParameterValue(LaunchConfiguration('dry_run'),
                                      value_type=bool),
            'check_curve': ParameterValue(LaunchConfiguration('check_curve'),
                                          value_type=bool),
            'publish_markers': True,
        }],
    )

    # 'lidar_corrected' es el frame_id que pone lidar_node en el scan
    # republicado. Publicamos identidad respecto a 'map' para que RViz
    # tenga un Fixed Frame válido aunque no haya árbol TF del robot.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_corrected_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lidar_corrected'],
        condition=IfCondition(LaunchConfiguration('static_tf')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        dry_run_arg,
        rviz_arg,
        static_tf_arg,
        check_curve_arg,
        supervisor,
        static_tf,
        rviz,
    ])

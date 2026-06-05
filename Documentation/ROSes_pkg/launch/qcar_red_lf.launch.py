"""
Line-follower variant of qcar_red.launch.py.

Mirrors the original red-QCar launcher (qcar/launch/qcar_red.launch.py) but
swaps the heavy `csi_redpatch` CSI node for the light-weight `csi_lf` from
ROSes_pkg (csi_front only, 320x240, 15 Hz, JPEG q=60).

Original command:
    ros2 launch qcar qcar_red.launch.py nodes:='qcar,rgbd,csi_redpatch'

Equivalent with line-follower CSI:
    ros2 launch ROSes_pkg qcar_red_lf.launch.py nodes:='qcar,rgbd,csi_lf'

The `qcar` and `rgbd` nodes still come from the stock `qcar` package; only
`csi_lf` comes from ROSes_pkg. All other supported names ('command', 'csi',
'csi_redpatch', 'rgbd_synchro', 'lidar_qos', 'imu_external') still dispatch
to the qcar package, so this file is a drop-in replacement.
"""

import os
import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Nodes that live in the stock `qcar` package.
_QCAR_NODES = {
    "command", "qcar", "csi", "csi_redpatch",
    "rgbd", "rgbd_synchro", "lidar_qos",
}

# Nodes that live in this package (ROSes_pkg).
# `imu_external` was moved here so it can be edited without touching the
# stock qcar package (the BNO055 JSON-over-serial reader lives here now).
# `odom_kalman` runs on the QCar so dead-reckoning survives a laptop
# disconnect (EKF fuses /qcar/velocity + /imu/data -> /odom).
_ROSES_NODES = {
    "csi_lf", "imu_external", "odom_kalman",
}


def launch_setup(context, *args, **kwargs):
    _csi_nodes = ["csi_right", "csi_front", "csi_back"]
    _rgbd_nodes = ["rgbd_color", "rgbd_depth"]

    nodes_to_launch = LaunchConfiguration('nodes').perform(context)
    nodes_list = [n.strip() for n in nodes_to_launch.split(',') if n.strip()]

    csi_nodes = LaunchConfiguration('csi_cameras').perform(context)
    csi_list = [n.strip() for n in csi_nodes.split(',')
                if n.strip() and n.strip() in _csi_nodes]

    rgbd_nodes = LaunchConfiguration('rgbd_cameras').perform(context)
    rgbd_list = [n.strip() for n in rgbd_nodes.split(',')
                 if n.strip() and n.strip() in _rgbd_nodes]

    calibration_file = LaunchConfiguration('rgbd_calibration_file').perform(context)
    if not calibration_file:
        calibration_file = os.path.join(
            get_package_share_directory('qcar'),
            'config',
            'red_qcar_camera_calibration_complete.yaml',
        )

    print('CSI:', csi_list, ' RGBD:', rgbd_list)

    parameters = {
        "command": [],
        "qcar": [
            {'publishers': ['imu', 'battery', 'velocity']},
            {'imu_publish_frequency': 100},
            {'battery_publish_frequency': 1},
            {'velocity_publish_frequency': 10},
        ],
        "csi": [
            {'publishers': csi_list},
            {'csi_right_resolution': [820, 410]},
            {'csi_front_resolution': [820, 410]},
            {'csi_back_resolution': [820, 410]},
            {'csi_right_freq': 120},
            {'csi_front_freq': 120},
            {'csi_back_freq': 120},
        ],
        # Tuning for the line-follower CSI publisher (ROSes_pkg/csi_lf).
        # Capture and publish both at 820x410 (the QCar's native CSI mode --
        # the only one Camera2D will open). Software downscale becomes a
        # no-op. JPEG quality 60 keeps the wire payload small enough.
        "csi_lf": [
            {'capture_width': 820},
            {'capture_height': 410},
            {'capture_freq': 120},
            {'output_width': 820},
            {'output_height': 410},
            {'publish_freq': 15.0},
            {'jpeg_quality': 60},
            {'topic': '/qcar/csi_front'},
            # Red QCar maps csi_front to device id "2" (see csinode_redpatch).
            {'camera_id': '2'},
        ],
        "rgbd": [
            {'publishers': rgbd_list},
            {'rgbd_color_resolution': [640, 480]},
            {'rgbd_depth_resolution': [640, 480]},
            {'rgbd_color_freq': 60},
            {'rgbd_depth_freq': 60},
        ],
        "rgbd_synchro": [
            {'publishers': ['rgbd_color', 'rgbd_depth',
                            'rgbd_color_camera_info', 'rgbd_depth_camera_info']},
            {'rgbd_color_resolution': [640, 480]},
            {'rgbd_depth_resolution': [640, 480]},
            {'sync_freq': 60},
        ],
        "lidar_qos": [
            {'lidar_Number_Samples': 100},
        ],
        # Puerto del ESP32/BNO055 está hardcoded dentro del nodo (by-id).
        "imu_external": [],
        # EKF de odometría: lee /qcar/velocity + /imu/data, publica /odom.
        # Corre en el carro para sobrevivir caídas de conexión con la laptop.
        "odom_kalman": [],
        "csi_redpatch": [],
    }

    launch_nodes = []
    for n in nodes_list:
        if n in _ROSES_NODES:
            pkg = 'ROSes_pkg'
        elif n in _QCAR_NODES:
            pkg = 'qcar'
        else:
            print(f"[qcar_red_lf] skipping unknown node: {n}")
            continue

        node_parameters = parameters.get(n, [])
        if n == 'rgbd_synchro':
            node_parameters = [calibration_file] + node_parameters

        launch_nodes.append(
            Node(
                package=pkg,
                node_executable=n,
                node_name=n,
                prefix=['stdbuf -o L'],
                output='screen',
                parameters=node_parameters,
            )
        )

    return launch_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'nodes',
            default_value='qcar,rgbd,csi_lf,imu_external,odom_kalman,lidar_qos',
            description=('Comma-separated nodes to launch. '
                         'Available: command, qcar, csi, csi_redpatch, csi_lf, '
                         'rgbd, rgbd_synchro, lidar_qos, imu_external, '
                         'odom_kalman. `csi_lf`, `imu_external` and '
                         '`odom_kalman` come from ROSes_pkg; everything else '
                         'comes from the qcar package.'),
        ),
        DeclareLaunchArgument(
            'csi_cameras',
            default_value='csi_right,csi_front,csi_back',
            description=('Subset of CSI cameras for the stock `csi` / '
                         '`csi_redpatch` nodes. Ignored when using csi_lf '
                         '(which is csi_front only).'),
        ),
        DeclareLaunchArgument(
            'rgbd_cameras',
            default_value='rgbd_color,rgbd_depth',
            description='Subset of RGBD cameras for the stock rgbd node.',
        ),
        DeclareLaunchArgument(
            'rgbd_calibration_file',
            default_value='',
            description=('Optional RGBD calibration YAML for rgbd_synchro. '
                         'Empty -> qcar/config/red_qcar_camera_calibration_complete.yaml.'),
        ),
        OpaqueFunction(function=launch_setup),
    ])

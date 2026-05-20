"""
Optimized line-following pipeline for the physical QCar.

Differences vs. line_follower_real.launch.py:
  - Uses image_converter_fast + lane_detector_fast (both in this package).
  - image_relay is dropped from the data path: the new detector subscribes
    BEST_EFFORT+depth=1 directly to the decompressed topic, so there is no
    QoS bridge to do.
  - Expects the QCar to be running `ros2 run ROSes_pkg csi_lf` (320x240,
    15 Hz, JPEG q=60, csi_front only). ROSes_pkg is the standalone package
    in /home/xyg/Desktop/ROSes_pkg meant to be deployed to the car.

Pipeline:
  /qcar/csi_front                 (CompressedImage, RELIABLE+TRANSIENT_LOCAL)
      [image_converter_fast]
  /qcar/decompressed/csi_front    (Image, BEST_EFFORT+depth=1)
      [lane_detector_fast]
      |
      |--> /lane_target_point_m   (Float32MultiArray)
      |        [lane_follower_q]
      |        --> /qcar/user_command (Vector3Stamped) or /qcar2_motor_speed_cmd
      |
      \\--> /qcar/line_follower/overlay (Image @ 5 Hz, view with rqt_image_view)

Before running, set the correct ROS domain for your QCar colour:
  Blue  -> export ROS_DOMAIN_ID=114
  Green -> export ROS_DOMAIN_ID=115
  Red   -> export ROS_DOMAIN_ID=116
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('line_follower_real')

    default_detector_params = os.path.join(
        pkg_dir, 'config', 'lane_detector_fast_params.yaml'
    )
    default_follower_params = os.path.join(
        pkg_dir, 'config', 'lane_follower_real_params.yaml'
    )

    detector_params_arg = DeclareLaunchArgument(
        'detector_params_file',
        default_value=default_detector_params,
        description='Lane detector parameter file (tuned for 320x240).',
    )
    follower_params_arg = DeclareLaunchArgument(
        'follower_params_file',
        default_value=default_follower_params,
        description='Lane follower parameter file.',
    )
    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description=('Launch the unified line-follower dashboard '
                     '(raw + detections + BEV + command status). '
                     'Set dashboard:=false to skip it (e.g. headless runs).'),
    )

    # Step 1 - Decompress CompressedImage -> Image (depth=1 throughout)
    img_converter_node = Node(
        package='line_follower_real',
        executable='image_converter_fast',
        name='image_converter_fast',
        output='screen',
        parameters=[
            {'subscribe_topic': '/qcar/csi_front'},
            {'publish_topic': '/qcar/decompressed/csi_front'},
        ],
    )

    # Step 2 - Detect lane and publish target point + low-rate overlay
    lane_detector_node = Node(
        package='line_follower_real',
        executable='lane_detector_fast',
        name='lane_detector_fast',
        output='screen',
        parameters=[LaunchConfiguration('detector_params_file')],
    )

    # Step 3 - Pure Pursuit controller
    lane_follower_node = Node(
        package='control_helpers_pkg',
        executable='lane_follower_q',
        name='lane_follower_q',
        output='screen',
        parameters=[LaunchConfiguration('follower_params_file')],
    )

    # Step 4 - Discovery keepalive.
    # The QCar publisher uses RELIABLE+TRANSIENT_LOCAL from a stripped-down
    # DDS stack and discovery against ROS 2 FastDDS is fragile. Running
    # `ros2 topic hz` triggers an aggressive participant probe that surfaces
    # the bare-DDS publisher; without it, no frames arrive. Output is sent
    # to `log` so it doesn't spam the terminal. This is a workaround that
    # belongs on the QCar side; until then, keep it.
    topic_hz_keepalive = ExecuteProcess(
        cmd=['ros2', 'topic', 'hz', '/qcar/csi_front'],
        name='topic_hz_keepalive',
        output='log',
    )

    # Same trick applied to the local overlay / BEV topics. Without this,
    # the dashboard tile occasionally freezes after the first few seconds
    # even though the publisher is still firing -- the symptom of a
    # subscription-side DDS hiccup. A continuously-active `ros2 topic hz`
    # subscriber keeps the local discovery state alive.
    overlay_hz_keepalive = ExecuteProcess(
        cmd=['ros2', 'topic', 'hz', '/qcar/line_follower/overlay'],
        name='overlay_hz_keepalive',
        output='log',
    )
    bev_hz_keepalive = ExecuteProcess(
        cmd=['ros2', 'topic', 'hz', '/qcar/line_follower/bev'],
        name='bev_hz_keepalive',
        output='log',
    )
    mask_hz_keepalive = ExecuteProcess(
        cmd=['ros2', 'topic', 'hz', '/qcar/line_follower/mask'],
        name='mask_hz_keepalive',
        output='log',
    )
    scan_hz_keepalive = ExecuteProcess(
        cmd=['ros2', 'topic', 'hz', '/qcar/scan'],
        name='scan_hz_keepalive',
        output='log',
    )

    # Step 5 - Unified dashboard (raw + detections + BEV + command status).
    # Toggleable so headless deploys can skip it.
    dashboard_node = Node(
        package='line_follower_real',
        executable='dashboard',
        name='line_follower_dashboard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    return LaunchDescription([
        detector_params_arg,
        follower_params_arg,
        dashboard_arg,
        img_converter_node,
        lane_detector_node,
        lane_follower_node,
        topic_hz_keepalive,
        overlay_hz_keepalive,
        bev_hz_keepalive,
        mask_hz_keepalive,
        scan_hz_keepalive,
        dashboard_node,
    ])

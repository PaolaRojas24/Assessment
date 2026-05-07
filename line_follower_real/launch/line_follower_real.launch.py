"""
Line-following pipeline for the physical QCar.

Pipeline:
  /qcar/csi_front  (CompressedImage, BEST_EFFORT QoS)
      → [img_converter]   vision_helpers_pkg
      → /qcar/decompressed/csi_front  (Image, BEST_EFFORT)
      → [image_relay]     line_follower_real  (QoS bridge)
      → /qcar/reliable/csi_front  (Image, RELIABLE)
      → [lane_detector]   vision_helpers_pkg
      → /lane_target_point_m  (Float32MultiArray [x_lateral_m, y_forward_m])
      → [lane_follower_q] control_helpers_pkg
      → /qcar/user_command  (Vector3Stamped  x=throttle, y=steering)
      → QCar hardware driver

Before running, set the correct ROS domain for your QCar colour:
  Blue  → export ROS_DOMAIN_ID=114
  Green → export ROS_DOMAIN_ID=115
  Red   → export ROS_DOMAIN_ID=116
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('line_follower_real')

    default_detector_params = os.path.join(
        pkg_dir, 'config', 'lane_detector_real_params.yaml'
    )
    default_follower_params = os.path.join(
        pkg_dir, 'config', 'lane_follower_real_params.yaml'
    )

    detector_params_arg = DeclareLaunchArgument(
        'detector_params_file',
        default_value=default_detector_params,
        description='Lane detector parameter file.',
    )
    follower_params_arg = DeclareLaunchArgument(
        'follower_params_file',
        default_value=default_follower_params,
        description='Lane follower parameter file.',
    )

    # Step 1 — Decompress /qcar/csi_front (CompressedImage, BEST_EFFORT) →
    #           /qcar/decompressed/csi_front (Image, BEST_EFFORT)
    img_converter_node = Node(
        package='vision_helpers_pkg',
        executable='img_converter',
        name='img_converter_front',
        output='screen',
        parameters=[
            {'subscribe_topic': '/qcar/csi_front'},
            {'publish_topic': '/qcar/decompressed/csi_front'},
        ],
    )

    # Step 2 — QoS bridge: BEST_EFFORT → RELIABLE
    #           img_converter publishes BEST_EFFORT; lane_detector requires RELIABLE.
    #           ROS2 DDS drops all messages when these policies mismatch.
    image_relay_node = Node(
        package='line_follower_real',
        executable='image_relay',
        name='image_relay',
        output='screen',
        parameters=[
            {'input_topic':  '/qcar/decompressed/csi_front'},
            {'output_topic': '/qcar/reliable/csi_front'},
            # Drop stale frames and limit to 15 Hz so lane_detector always sees
            # a recent frame instead of processing a growing backlog.
            {'throttle_hz':  15.0},
        ],
    )

    # Step 3 — Detect lane and publish target point
    lane_detector_node = Node(
        package='vision_helpers_pkg',
        executable='lane_detector',
        name='lane_detector',
        output='screen',
        parameters=[LaunchConfiguration('detector_params_file')],
    )

    # Step 4 — Pure Pursuit controller → /qcar/user_command
    lane_follower_node = Node(
        package='control_helpers_pkg',
        executable='lane_follower_q',
        name='lane_follower_q',
        output='screen',
        parameters=[LaunchConfiguration('follower_params_file')],
    )

    # Step 5 — Discovery probe for the QCar's bare-DDS publisher.
    #          The QCar publishes /qcar/csi_front from a non-ROS2 DDS app
    #          (shows up as "_CREATED_BY_BARE_DDS_APP_" in `ros2 topic info`).
    #          Until `ros2 topic hz` runs, the publisher is invisible to our
    #          subscribers — discovery never completes properly.  Running
    #          `ros2 topic hz` triggers the aggressive discovery probe that
    #          surfaces the bare-DDS publisher; once visible, all subscribers
    #          start receiving frames.  Output is hidden ('log') so it doesn't
    #          spam the terminal.  This is a workaround, not a fix; the real
    #          fix is on the QCar side (use a proper ROS2 publisher).
    topic_hz_keepalive = ExecuteProcess(
        cmd=['ros2', 'topic', 'hz', '/qcar/csi_front'],
        name='topic_hz_keepalive',
        output='log',
    )

    return LaunchDescription([
        detector_params_arg,
        follower_params_arg,
        img_converter_node,
        image_relay_node,
        lane_detector_node,
        lane_follower_node,
        topic_hz_keepalive,
    ])

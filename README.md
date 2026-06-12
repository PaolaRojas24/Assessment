# Assessment - Guns-n-ROSes

ROS 2 project for autonomous QCar lane following and obstacle avoidance. The car uses its front CSI camera to detect lane markings, compute a target point in vehicle coordinates, and follow it using a Pure Pursuit controller.

---

## Overview

```
Camera image
    │
    ▼
[lane_detector] ─── /lane_target_point_m ──► [lane_follower_q] ──► /qcar/user_command
    │                                                                      │
    └── /lane_lines (debug)                                          QCar motors
```

| Node | Package | Role |
|------|---------|------|
| `lane_detector` | `qcar_lane_perception` | Detects lane lines from camera and publishes a target waypoint in meters (rear-axle frame) |
| `lane_follower_q` | `control_helpers_pkg` | Pure Pursuit controller that converts the target point into throttle + steering commands |
| `obstacle_dodge` | `qcar_control` | Executes lateral dodge maneuver when an obstacle is detected ahead |

---

## Launch Reference

The project is started with **three launch commands** — two run on the laptop and one on the car.

---

### 1. Full Stack — Laptop

```bash
ros2 launch qcar_control qcar_full_stack.launch.py
```

```
qcar_full_stack.launch.py
├─ include  lane_perception.launch.py
│   ├─ node  image_converter        (qcar_lane_perception)   image_converter.py
│   ├─ node  lane_detector          (qcar_lane_perception)   lane_detector.py
│   └─ keepalives hz
├─ node     lidar_node              (lidar_qcar)
├─ include  qcar_control.launch.py
│   ├─ node  lane_follower_q        (control_helpers_pkg)    ← lane_follower_real_params.yaml
│   └─ node  command_mux            (qcar_control)
├─ include  qcar_odom_viz.launch.py
│   ├─ node  odom_path              (qcar_odom_viz)          odom_path.py
│   └─ node  rviz2                                           ← odom_view.rviz
└─ include  dashboard.launch.py     [dashboard:=true]
    └─ node  dashboard              (qcar_opencv_dashboard)  dashboard.py
```

Keepalives monitored: `/qcar/scan`, `stateBattery`, `obstacle_detected`, `safe_stop_active`

---

### 2. Obstacle Dodge — Laptop

```bash
ros2 launch qcar_control dodge.launch.py
```

```
dodge.launch.py
├─ node  obstacle_dodge             (qcar_control)
├─ node  static_transform_publisher (tf2_ros)        map → lidar_corrected   [static_tf:=true]
└─ node  rviz2                                       ← overtake.rviz          [rviz:=true]
```

---

### 3. Car Nodes — QCar (on-board)

```bash
ros2 launch ROSes_pkg qcar_red_lf.launch.py
```

```
qcar_red_lf.launch.py
│
├─ [ROSes_pkg]  csi_lf        → csinode_lf.py
├─ [ROSes_pkg]  imu_external  → imu_external.py
├─ [ROSes_pkg]  odom_kalman   → odom_kalman.py
│
├─ [qcar]  qcar        → qcarnode.py
├─ [qcar]  rgbd        → rgbdnode.py
├─ [qcar]  lidar_qos   → lidarnode_qos.py
│   └─ optional nodes: command, csi, csi_redpatch, rgbd_synchro
│       └─ rgbd_synchro uses: red_qcar_camera_calibration_complete.yaml
│
└─ Dependencies (external to qcar package)
    ├─ qcar_interface   custom ROS msgs/srvs   (copy alongside qcar — ≠ qcar2_interfaces)
    └─ pal (Quanser SDK) hardware library in Python   (lives on the car, do NOT copy)
```

> **ROSes_pkg** is the package deployed on the robot. The full source is available locally under [Documentation/ROSes_pkg/](Documentation/ROSes_pkg/) and as a zip at [Documentation/ROSes_pkg.zip](Documentation/ROSes_pkg.zip).

---

## Packages

### `qcar_lane_perception`
Camera-based lane detection pipeline:
1. `image_converter.py` — converts the raw CSI image to a format suitable for processing.
2. `lane_detector.py` — segments white/yellow markings, applies ROI mask, runs Probabilistic Hough Transform, averages lines into left/right lanes, projects target pixel through a Bird's-Eye View homography to metric coordinates.

**Published topics:**
- `/lane_lines` (`Float32MultiArray`) — raw left + right line endpoints (debug)
- `/lane_target_point_m` (`Float32MultiArray`) — `[x_lateral_m, y_forward_m]` in rear-axle frame

**Subscribed topics:**
- `/qcar/csi_front/image_raw` (configurable)

### `control_helpers_pkg`
Pure Pursuit lane follower:
- Computes dynamic lookahead distance based on current speed.
- Calculates steering angle using `atan2(2·L·x, Ld²)`.
- Reduces speed automatically when in a curve (`|δ| > curve_threshold`).
- Supports both `qcar` (`Vector3Stamped`) and `qcar2` (`MotorCommands`) platforms.

**Subscribed topics:** `/lane_target_point_m`

**Published topics:** `/qcar/user_command` (platform-dependent)

### `qcar_control`
Higher-level control logic:
- `command_mux` — multiplexes lane follower and dodge commands by priority.
- `obstacle_dodge` — executes a lateral dodge maneuver on obstacle detection.

### `lidar_qcar`
Publishes filtered LiDAR scan data from the QCar's RPLiDAR and exposes `/qcar/scan`.

### `qcar_odom_viz`
Odometry visualization: accumulates pose history (`odom_path.py`) and renders it in RViz2.

### `qcar_opencv_dashboard`
OpenCV overlay dashboard (`dashboard.py`) showing live camera feed, lane lines, speed, and steering.

### `ROSes_pkg` *(on-board package)*
Runs directly on the QCar. Contains:
- `csinode_lf.py` — CSI camera publisher (left-front).
- `imu_external.py` — IMU data bridge.
- `odom_kalman.py` — Kalman-filtered odometry estimator.

Source: [Documentation/ROSes_pkg/](Documentation/ROSes_pkg/)

---

## Key Parameters

### Lane Follower (`control_helpers_pkg/config/lane_follower_real_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `platform` | `qcar` | Target platform: `qcar` or `qcar2` |
| `wheelbase` | `0.256 m` | QCar wheelbase |
| `lookahead_base` | `0.20 m` | Base lookahead distance |
| `lookahead_min / max` | `0.16 / 0.34 m` | Lookahead clamp range |
| `speed_straight` | `0.15` | Throttle on straight sections |
| `speed_curve` | `0.0775` | Throttle on curves |
| `max_steering_angle` | `0.50 rad` | Steering saturation limit |
| `curve_threshold` | `0.20 rad` | Steering angle above which curve speed applies |
| `steering_sign` | `-1.0` | Invert if steering is mirrored |

---

## Dependencies

### Laptop
- ROS 2 Humble or later
- `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`
- `cv_bridge`, `opencv-python`, `numpy`
- `tf2_ros`
- `rviz2`

### QCar (on-board)
- `ROSes_pkg` — see [Documentation/ROSes_pkg/](Documentation/ROSes_pkg/)
- `qcar` package (qcarnode, rgbdnode, lidarnode_qos) — copy to the car
- `qcar_interface` — custom msgs/srvs, copy alongside `qcar`
- `pal` (Quanser SDK) — Python hardware library, already present on the car

---

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

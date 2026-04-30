# Assessment - Guns-n-ROSes

ROS 2 project for autonomous QCar lane following and obstacle avoidance. The car uses its front CSI camera to detect lane markings, compute a target point in vehicle coordinates, and follow it using a Pure Pursuit controller.

---

## Overview

```
Camera image
    │
    ▼
[lane_detector] ─── /lane_target_point_m ──► [lane_follower_q] ──► /qcar_sim/user_command
    │                                                                      │
    └── /lane_lines (debug)                                          QCar motors
```

| Node | Package | Role |
|------|---------|------|
| `lane_detector` | `vision_helpers_pkg` | Detects lane lines from camera and publishes a target waypoint in meters (rear-axle frame) |
| `lane_follower_q` | `control_helpers_pkg` | Pure Pursuit controller that converts the target point into throttle + steering commands |
| `vector3_publisher` | `vector3_teleop` | Manual teleoperation publisher (sinusoidal test drive) |

---

## Packages

### `vision_helpers_pkg`
Camera-based lane detection pipeline:
1. Converts camera image to HLS color space.
2. Segments white and yellow lane markings.
3. Applies a trapezoidal ROI mask.
4. Runs Probabilistic Hough Transform to extract line segments.
5. Averages lines into left/right lanes; computes center line.
6. Projects the target pixel through a Bird's-Eye View homography to get metric coordinates relative to the rear axle.

**Published topics:**
- `/lane_lines` (`Float32MultiArray`) — raw left + right line endpoints (debug)
- `/lane_target_point_m` (`Float32MultiArray`) — `[x_lateral_m, y_forward_m]` in rear-axle frame

**Subscribed topics:**
- `/qcar_sim/csi_front/image_raw` (default, configurable)

### `control_helpers_pkg`
Pure Pursuit lane follower:
- Computes dynamic lookahead distance based on current speed.
- Calculates steering angle using `atan2(2·L·x, Ld²)`.
- Reduces speed automatically when in a curve (`|δ| > curve_threshold`).
- Supports both `qcar` (`Vector3Stamped`) and `qcar2` (`MotorCommands`) platforms.

**Subscribed topics:**
- `/lane_target_point_m`

**Published topics:**
- `/qcar_sim/user_command` or `/qcar/user_command` (platform-dependent)

### `vector3_teleop`
Simple teleop node for manual testing. Publishes a sinusoidal steering pattern at 10 Hz.

```bash
# Real QCar
ros2 run vector3_teleop publisher

# Simulated QCar
ros2 run vector3_teleop publisher --ros-args -p is_sim:=true
```

---

## Simulation

The simulation uses **Gazebo** via the `qcar_gazebo` package with a custom QCar track.

```bash
# Launch simulator (default world: test_world.sdf)
ros2 launch qcar_gazebo gz_sim.launch.py

# Launch with the assessment world
ros2 launch qcar_gazebo gz_sim.launch.py world:=assessment_world.sdf
```

Available worlds (in `qcar_gazebo/worlds/`):
- `test_world.sdf`
- `assessment_world.sdf`
- `stop_sign_cross_world.sdf`
- `stop_test_world.sdf`

---

## Running Lane Following (Simulation)

**Terminal 1 — Simulator:**
```bash
ros2 launch qcar_gazebo gz_sim.launch.py world:=assessment_world.sdf
```

**Terminal 2 — Lane Detector:**
```bash
ros2 launch vision_helpers_pkg lane_detector.launch.py
```

**Terminal 3 — Lane Follower:**
```bash
ros2 launch control_helpers_pkg lane_follower_q.launch.py
```

---

## Key Parameters

### Lane Detector (`vision_helpers_pkg/config/lane_detector_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `subscribe_topic` | `/qcar_sim/csi_front/image_raw` | Input camera topic |
| `bev_pixels_per_meter` | `3000.0` | BEV resolution |
| `camera_to_rear_axle_forward_m` | `0.323` | Camera offset from rear axle |
| `lane_lateral_bias` | `0.25` | Lateral bias within lane (0 = center, +1 = left) |
| `lane_half_width_px` | `50.0` | Initial estimate of half-lane width in pixels |

### Lane Follower (`control_helpers_pkg/config/lane_follower_q_params.yaml`)

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

- ROS 2 (Humble or later)
- `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`
- `cv_bridge`, `opencv-python`
- `numpy`
- `qcar2_interfaces` (custom messages: `MotorCommands`, `BooleanLeds`)
- `ros_gz_sim`, `ros_gz_bridge` (for simulation)
- `robot_localization` (EKF odometry in simulation)

---

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

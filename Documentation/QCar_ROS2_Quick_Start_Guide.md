# QCar ROS 2 Interface Quick Start

This document serves as a quick start guide for developing autonomous navigation algorithms on the Quanser QCar platform using ROS 2.

## Remote Connection to the QCar
To access the onboard computer, **SSH (Secure Shell)** is the recommended tool.

```bash
ssh -X nvidia@qcar_ip_address
```

* **Default Password:** `nvidia`
* **IP Address:** Located on the QCar's LCD display. 
* **Network:** All QCars are connected to the `Mocap_UVS` network by default.
* **Note:** It is highly recommended to avoid providing an internet connection to the router, as it is optimized for LAN usage.

## Running ROS 2 Nodes on the QCar

To successfully run the installed nodes, you must enter **Super User** mode:

```bash
sudo -s
```

Once acting as `root`, you can run any of the following nodes (unless specified as hardware-dependent or platform-specific):

### Common Nodes
* **`csi`**: Opens all monocular cameras and publishes to `/qcar/csi_{camera}` (left, right, front, back). Topics use `sensor_msgs/msg/CompressedImage`.
* **`imageviewer`**: A visualization tool for all camera topics (`csi` and `rgbd`).
* **`lidar_qos`**: The updated LiDAR node (replaces the deprecated `lidar` node). Publishes to the `/qcar/scan` topic.
* **`qcar`**: The central node. Enables motors, internal IMU, battery state, and motor encoders. 
    * **Publishes:** `/qcar/imu`, `/qcar/velocity`, `/qcar/batteryState`
    * **Subscribes:** `/qcar/user_command`
* **`rgbd`**: Reads the Intel RealSense (stereoscopic) camera.
    * **Publishes:** `/qcar/rgbd_color` (`CompressedImage`) and `/qcar/rgbd_depth` (`Image`).

### Hardware-Dependent Nodes
* **`command`**: Converts joystick inputs to the `/qcar/user_command` topic.
* **`imu_external`**: Integrates the **BNO055 9-axis IMU** via a microcontroller (RS232 serial). Publishes `/imu/data` and `/imu/accel_raw`.

### Platform-Specific Patch (Red QCar)
* **`csi_redpatch`**: Specifically for the **Red QCar**, which has a faulty `csi_left` camera. This node remaps camera IDs to ignore the missing hardware while publishing to the remaining `right`, `front`, and `back` topics.

---

## Node Summary Tables

### Internal Common Nodes
| Node | Publishes to | Subscribes to | Comments |
| :--- | :--- | :--- | :--- |
| `/qcar` | `/qcar/imu`, `/qcar/stateBattery`, `/qcar/velocity` | `/qcar/user_command` | in `/qcar/user_command`, `x` = drive, `y` = steering |
| `/csi` | `/qcar/csi_{back, front, left, right}` | — | All are compressed images |
| `/lidar_qos` | `/qcar/scan` | — | Improved QoS over legacy node |
| `/rgbd` | `/qcar/rgbd_color`, `/qcar/rgbd_depth` | — | `color` is compressed |
| `/imageviewer` | — | Camera topics | Debugging/visualization tool |

### Hardware-Dependent & Specific Nodes
| Node | Publishes to | Subscribes to | Comments |
| :--- | :--- | :--- | :--- |
| `/command` | `/qcar/user_command` | — | Use with manufacturer joystick |
| `/imu_external` | `/imu/data`, `/imu/accel_raw` | — | Requires BNO055 via USB |
| `/csi_redpatch`| `/qcar/csi_{back, front, right}` | — | Patch for Red QCar (ignores `left`) |

---

## Using Launch Files

Each QCar has a specific launch file to initialize its unique hardware configuration.

| QCar Platform | Launch File Name | Default `nodes` Argument |
| :---: | :--- | :--- |
| **Blue** | `qcar_blue.launch.py` | `'qcar,csi,rgbd,lidar_qos,imu_external'` |
| **Green** | `qcar_green.launch.py` | `'qcar,csi,rgbd,lidar_qos'` |
| **Red** | `qcar_red.launch.py` | `'qcar,csi_redpatch,rgbd,lidar_qos'` |

### Launch Commands
To view all available arguments for a specific platform:
```bash
ros2 launch -s qcar qcar_{platform_color}.launch.py
```

To launch with **Remote Control (Joystick)** enabled:
```bash
ros2 launch qcar qcar_{platform_color}.launch.py nodes:='command,qcar,csi,rgbd,lidar_qos'
```

---

## Multi-Robot Coordination (`ROS_DOMAIN_ID`)
To prevent network interference when operating multiple QCars simultaneously, each platform is assigned a unique `ROS_DOMAIN_ID`:

| QCar Platform | ROS_DOMAIN_ID |
| :---: | :---: |
| **Blue** | `114` |
| **Green** | `115` |
| **Red** | `116` |

---

## Topic Reference List

| Topic | ROS Message Type |
| :--- | :--- |
| `/imu/accel_raw` | `geometry_msgs/msg/TwistStamped` |
| `/imu/data` | `sensor_msgs/msg/Imu` |
| `/qcar/csi_{back, front, left, right}` | `sensor_msgs/msg/CompressedImage` |
| `/qcar/imu` | `sensor_msgs/msg/Imu` |
| `/qcar/rgbd_color` | `sensor_msgs/msg/CompressedImage` |
| `/qcar/rgbd_depth` | `sensor_msgs/msg/Image` |
| `/qcar/scan` | `sensor_msgs/msg/LaserScan` |
| `/qcar/stateBattery` | `sensor_msgs/msg/BatteryState` |
| `/qcar/user_command` | `geometry_msgs/msg/Vector3Stamped` |
| `/qcar/velocity` | `geometry_msgs/msg/Vector3Stamped` |
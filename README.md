# Smart Mobility (Movilidad Inteligente)

This repository contains essential ROS 2 packages designed for the **Autonomous Mobile Robots** course. These packages provide the core functionalities required to simulate various platforms in a **Gazebo** environment, including:
* **Differential Drive Robots**
* **Toyota Prius**
* **Quanser QCar**

Additionally, this repository hosts packages developed throughout the **Smart Mobility** course.

---

## 🚀 Getting Started

### Prerequisites
To use these packages, you must have one of the following ROS 2 distributions installed on your system:
* **ROS 2 Humble**
* **ROS 2 Jazzy**

### Dependencies

> [!NOTE]
> Ensure you have sourced your ROS 2 installation (e.g., `source /opt/ros/jazzy/setup.bash`) so that the `${ROS_DISTRO}` variable is correctly identified by your shell.

Run the following command to install the required dependencies:

```bash
sudo apt update && \
sudo apt install -y ros-${ROS_DISTRO}-ros2-control \
                    ros-${ROS_DISTRO}-ros2-controllers \
                    ros-${ROS_DISTRO}-gz-ros2-control \
                    ros-${ROS_DISTRO}-ros-gz \
                    ros-${ROS_DISTRO}-ros-gz-bridge \
                    ros-${ROS_DISTRO}-joint-state-publisher \
                    ros-${ROS_DISTRO}-robot-state-publisher \
                    ros-${ROS_DISTRO}-xacro \
                    ros-${ROS_DISTRO}-joy
```

### Compilation

Follow these steps to clone and build the packages within your ROS 2 workspace:

1.  **Navigate** to your workspace's source directory (e.g., `~/ros2_ws/src`):
    ```bash
    cd ~/ros2_ws/src
    ```
2.  **Clone** this repository:
    ```bash
    git clone https://github.com/dsosa114/movilidad_inteligente.git
    ```
3.  **Return** to the root of your workspace:
    ```bash
    cd ~/ros2_ws/
    ```
4.  **Build** the packages using `colcon`:
    ```bash
    colcon build
    ```

> [!TIP]
> This command builds only the necessary packages, ensuring a quick and clean compilation. Once the build is successful, remember to source your workspace (`source install/setup.bash`) to begin using the nodes.

---
## 🎮 Usage

Once the packages are built and your workspace is sourced, you can run the simulations using the following commands.

### Simulation Launch

Each vehicle project includes a launch file named `gz_sim.launch.py` within its respective package.

#### Quanser QCar
The QCar simulation supports both **ROS 2 Humble** and **Jazzy** distributions via the `is_ign` parameter.
* **For Jazzy (Gazebo Harmonic):**
    ```bash
    ros2 launch qcar_gazebo gz_sim.launch.py 
    ```
* **For Humble (Gazebo Fortress/Ignition):**
    ```bash
    ros2 launch qcar_gazebo gz_sim.launch.py is_ign:=true
    ```

#### Toyota Prius
To start the Toyota Prius simulation:
```bash
ros2 launch prius_bringup gz_sim.launch.py
```

---

### 🕹️ Teleoperation

To control the simulation platforms manually, you can use a joystick or gamepad.

#### QCar Joystick Control
To enable joystick control for the QCar, run this command in a new terminal:
```bash
ros2 launch qcar_gazebo joystick.launch.py
```

> [!TIP]
> Ensure your controller is connected and recognized by the system before running the joystick launch file. You can check this by running `ls /dev/input/js*`.


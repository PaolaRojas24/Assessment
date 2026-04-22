# Ackermann Vehicle Description Package (ROS 2)

This repository serves as the **description package** for an **Ackermann steering vehicle** model, designed specifically for use within the **Robot Operating System 2 (ROS 2)** framework.

It contains all the necessary assets (URDF, configuration, and launch files) to define, visualize, and eventually control the vehicle in a simulation environment (like Gazebo or Rviz 2).

---

## 🏗️ Project Structure

The package is structured according to ROS 2 conventions, focusing on the Ackermann vehicle definition:

| Directory/File | Description |
| :--- | :--- |
| `urdf/` | Contains the **Unified Robot Description Format (URDF)** file(s) that define the kinematic, visual, and inertial properties of the Ackermann vehicle model. | 
| `config/` | Holds configuration files (e.g., YAML) for controllers, robot parameters, or simulation environments. | 
| `launch/` | Includes **ROS 2 launch files** (`.py`) for starting up the vehicle model in a simulator (like Gazebo) or initializing the robot state publisher and Rviz 2. |
| `CMakeLists.txt` | Standard build file for ROS 2 packages using the `ament_cmake` build system. | 
| `package.xml` | Package manifest defining dependencies, maintainer information, and the build type (`ament_cmake`). | 
| `LICENSE` | Licensing information for the project. |

---

## 🚀 Getting Started (ROS 2)

### Prerequisites

This project requires a working installation of **ROS 2** (e.g., Foxy, Galactic, Humble, etc.) set up on your system.

### Build Instructions

1.  **Clone the Repository:**
    ```bash
    cd <your_ros2_workspace>/src
    git clone https://github.com/dsosa114/movilidad_inteligente.git
    ```
2.  **Build the Package:**
    Use `colcon` to build the workspace:
    ```bash
    cd <your_ros2_workspace>
    colcon build --packages-select ackermann_vehicle_description
    ```
3.  **Source the Workspace:**
    ```bash
    source install/setup.bash
    ```

### Running the Vehicle Description

The launch files are used to bring up the vehicle in visualization tools. A common use case for a description package is launching the **robot state publisher** and **Rviz 2**.

To launch the model:

```bash
ros2 launch movilidad_inteligente display.launch.py
```

This command will typically:

* Publish the joint and link positions defined in the URDF.
* Open Rviz 2 for viewing the vehicle model and its coordinate frames.

## ⚙️ Ackermann Steering Geometry

This package implements the specialized joint configuration required for Ackermann steering, which mimics the turning mechanism of a car (where the inner wheel turns more sharply than the outer wheel).

The URDF files meticulously define the joints, links, and transmission interfaces needed for both accurate visualization and eventual kinematic control.

## ✅ Current Status

The project is a work-in-progress.

## 🤝 Contributing

Contributions are welcome! Please feel free to open an issue or submit a pull request for improvements, bug fixes, or new features.

## 📄 License

This project is licensed under the terms defined in the LICENSE file.

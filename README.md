# ROS2 Control Example for RB‑1

![ROS2](https://img.shields.io/badge/ros2-humble-blue) ![Gazebo](https://img.shields.io/badge/gazebo-latest-green)

## Introduction

This repository demonstrates how to use the ROS 2 Control framework to drive and manipulate the differential‑drive robot RB‑1. You will learn how to:

* Launch a Gazebo simulation of RB‑1
* Activate and command the differential‑drive controller
* Command an elevator (lifting) controller

## Features

* **Differential Drive Control:** Interface with RB‑1’s base using standard `geometry_msgs/msg/Twist` messages.
* **Elevator Manipulation:** Move the lifting unit up and down via a simple Float64MultiArray command.
* **Out‑of‑the‑Box Simulation:** Ready‑to‑run Gazebo launch file for seamless setup.

## Prerequisites

Before you begin, ensure you have the following installed on Ubuntu 22.04:

1. **ROS 2 Humble:** Follow the official Quickstart guide [here](https://docs.ros.org/en/humble/Installation.html).
2. **Gazebo:** The version bundled with ROS 2 Humble or later.
3. **colcon build tool:** Installs with `sudo apt install python3-colcon-common-extensions`.

## Installation

1. Clone the repository into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/mathrosas/ros2_control.git
   ```
2. Install any missing dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep update && rosdep install --from-paths src --ignore-src -r -y
   ```

## Building the Package

Compile and source your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_control
source install/setup.bash
```

## Running the Simulation

Launch RB‑1 in Gazebo:

```bash
ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py
```

Verify that controllers and hardware interfaces are active:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

## Differential Drive Controller

You can drive RB‑1 in two ways:

1. **Publish velocity commands manually:**

   ```bash
   ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped \
     geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.2}}"
   ```
2. **Use teleop\_twist\_keyboard:**

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
     --ros-args --remap cmd_vel:=/rb1_base_controller/cmd_vel_unstamped
   ```

## Elevator (Lifting Unit) Controller

Control the lift with Float64MultiArray commands:

* **Move Up:**

  ```bash
  ros2 topic pub /elevator_controller/commands std_msgs/msg/Float64MultiArray \
    "data: [0.1]" -1
  ```
* **Move Down:**

  ```bash
  ros2 topic pub /elevator_controller/commands std_msgs/msg/Float64MultiArray \
    "data: [0.0]" -1
  ```

## Contributing

Contributions, bug reports, and feature requests are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature-name`)
3. Commit your changes (`git commit -m "Add feature"`)
4. Push to your fork (`git push origin feature-name`)
5. Open a Pull Request

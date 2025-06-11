# ros2_control

## Introduction:
This repository is an example of ROS2 control framework.
This package is able to control the differential drive robot RB-1.
It can drive the robot around as well as manipulate its elevator.

## Installation instructions:
Be sure to install ROS 2 Humble and Gazebo. Then clone this repository: https://github.com/mathrosas/ros2_control.git

## Getting started:
Once the package is installed, the next step is to get started with the simulation.
Then, execute the following commands:
Go to ros2_ws: cd ~/ros2_ws/
Compile the package: colcon build
Source: source install/setup.bash
Finally, launch the simulation: ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py
Check the controllers loaded successfully: ros2 control list_hardware_interfaces; ros2 control list_controllers

## Differential drive controller activation:
Once the simulation is running, you can move the robot with one of the following commands:
1. ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.2}}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rb1_base_controller/cmd_vel_unstamped
2. ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rb1_base_controller/cmd_vel_unstamped

## Elevator controller activation:
In order to move the lift up and down, you can use the following commands:
1. UP: ros2 topic pub /elevator_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1]" -1
2. DOWN: ros2 topic pub /elevator_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]" -1
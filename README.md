# Overview
A Simple implementiation of ROS2 on a Gazebo world for a 4x4 car simulation, the world file and ROS contents are packaged inside the code itself

# Setting up
## ROS2 Humble
The Code requires ROS2 Humble to be installed in your system, install it using [this link to official documentation](https://docs.ros.org/en/humble/index.html)

## Gazebo
Gazebo Should also be installed alongside ROS2 Humble, [Link](https://gazebosim.org/docs/latest/install/)

## Setup Gazebo with ROS2
By Default, Gazebo and ROS2 are not configured to work alongside, hence we have to install dependancies for it
The official guide for dependancing can be found on [Official Documentation for Gazebo](https://gazebosim.org/docs/latest/ros_installation/)

### Before running the simulation, setup the bash file (required for every terminal booted)
> source /opt/ros/humble/setup.bash
> source <path_to_workspace>/ROS_Task_InterIIT_Tech/install/setup.bash

This sets up required runtimes for shell

### Launch
> ros2 launch motor_controller world_launch

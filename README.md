# Simple ROS2 Car Simulation in Gazebo

This project sets up a simple car (robot) in Gazebo and controls it using ROS 2. The car is simulated in a Gazebo environment, and its movement is controlled using the teleop_twist_keyboard package. The car’s actuators are controlled through the ros2_control framework, and a custom controller node (operator node) listens to cmd_vel messages and publishes corresponding control messages.

# Setting up
Requirements

-  ROS 2 Humble or later
-  Gazebo (compatible with ROS 2)
-  Python 3 (for ROS 2 nodes and launch files)
-  Install ROS 2 Control and Dependencies

### Before running the simulation, setup the bash file (required for every terminal booted)
```
source /opt/ros/humble/setup.bash
source <path_to_workspace>/ROS_Task_InterIIT_Tech/install/setup.bash
```

### ROS_Control
A set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces. We use this in order to comminicate with the hardware of our robot, in this case the virtual robot we create in Gazebo
```
sudo apt-get install ros-humble-ros-control ros-humble-ros-controllers
rosdep install --from-paths src --ignore-src -r -y  
```
use this command for installation

This sets up required runtimes for shell

# Launch
```ros2 launch motor_controller world_launch```

## Custom Controller Node (operatorNode.py)

The operatorNode.py is a custom node that listens to the cmd_vel topic and re-publishes control messages to drive the robot. This is useful for adding additional logic for controlling the robot beyond basic teleoperation. The purpose of this node was to showcase knowledge regarding creation and usage of publishers and subscribers

# ABOUT THE CODE

The code as of now is partially functional, so it won't run using the launch command because I haven't been able to setup the controller and launch script for the system, the URDF for the model is correct and the main part of logic that is **Nodes and ROS Communication Methods** is fully functional and can be tried by manually initialising the nodes in different terminals

### Make sure setup.bash is sourced in your terminal
`source install/setup.bash`

Then run the following commands in different terminals
```
cd src/motor_controller/motor_controller
./operatorNode # For operator

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

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
Start the Teleops Keyboard service for inputs
```ros2 run teleop_twist_keyboard teleop_twist_keyboard```

Launch the device
```ros2 launch motor_controller launch.py```

# ABOUT THE CODE

The code is currently fully functional and launched directly, it will load the gazebo model and start the required nodes, however the teleops_keyboard has to be started on its own, the URDF for the model is correct and the main part of logic that is **Nodes and ROS Communication Methods** is fully functional and can be tried by manually initialising the nodes in different terminals, the motion of car is stuck to one axis due to not implementing a steering system

### Make sure setup.bash is sourced in your terminal
`source install/setup.bash`

Then run the following commands in different terminals
```
cd src/motor_controller/motor_controller
./operatorNode # For operator

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Functioning
The simulator uses multiple components

## ROS2
Used as a framework, consisting of nodes which can commincate between each other using publishers/subscribers or two way client-server communications using Services and Actions.

## Custom Controller Node (operatorNode.py)

The operatorNode.py is a custom node that listens to the cmd_vel topic and re-publishes control messages to drive the robot. This is useful for adding additional logic for controlling the robot beyond basic teleoperation. The purpose of this node was to showcase knowledge regarding creation and usage of publishers and subscribers.

## Teleops_twist_keyboard 

The teleops_twist_keyboard is a built-in package of ROS2 utilized to take inputs from terminal for controlling our robot, the teleops_twist-keyboard creates a publisher on topic `cmd_vel` using the datatype as `twist` which, in this code is passed on to our Custom Controller Node, which forwards it to our model, it is an unnessary step but added to showcase working of the Nodes.

## URDF Model
,
The URDF Model is a XML format file in which we describe our model's design in terms of `links`, and `joints`, each `link` acts as a element in the design (Eg: a cube and wheels) amd `joint` acts as connction between two `links`, we have to se parameters like `joint-type`(universal joint, revolute joint, Axis),

## ROS2_Control

The Framework used as a communication between ROS 2 system and actual hardware, here we add the Gazebo plugin for `dif_drive_controller` in the URDF file and set its configuration in the `configuration.yaml` file, and load it into launch file

## Launch File

The all in one file used to launch everything together, we initilize `robot_state_publisher`, spawn the bot by loading the URDF file, load the `controller_manager` for ROS2_Control and launch our Operator Node.


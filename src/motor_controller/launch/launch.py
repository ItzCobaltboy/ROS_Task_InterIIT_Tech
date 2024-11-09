import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo world
        Node(
            package='gazebo_ros',
            executable='gzserver',
            output='screen',
            arguments=['-s', 'libgazebo_ros_init.so', '$(find motor_controller)/gazebo_world/testing_world.world']
        ),
        
        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            arguments=[
                '-entity', 'simple_robot',
                '-file', '$(find motor_controller)/gazebo_models/carModelURDF.urdf'
            ],
            output='screen'
        ),

        # Start the operator node (controller)
        Node(
            package='motor_controller',
            executable='operatorNode',
            output='screen'
        ),

        # Start the teleop twist keyboard node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/cmd_vel', '/cmd_vel')]  # Remapping cmd_vel to match topic name in URDF and controller
        ),
    ])

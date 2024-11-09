import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='my_robot', description='Robot model name'),
        
        # Gazebo Launch
        IncludeLaunchDescription(
            launch_description=launch.substitutions.LaunchConfiguration('robot_model')
        ),
        
        # Teleop Twist Keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Custom controller node
        Node(
            package='motor_controller',
            executable='operatorNode',
            name='Operator',
            output='screen'
        ),
        
        # Controller Manager (ros2_control)
        Node(
            package='controller_manager',
            executable='spawner.py',
            name='controller_spawner',
            arguments=['--controller-manager', '/controller_manager', 'diff_drive_controller'],
            output='screen'
        ),
    ])

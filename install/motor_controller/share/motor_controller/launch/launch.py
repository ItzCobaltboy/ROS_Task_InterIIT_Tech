import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Get the package directory path
    motor_controller_share = get_package_share_directory('motor_controller')

    # Path to the URDF file
    urdf_file = os.path.join(motor_controller_share, 'gazebo_models', 'carModelURDF.urdf')


    return LaunchDescription([

        # Declare the argument for use_sim_time (simulation time vs. wall time)
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        # 1. Start robot_state_publisher to publish robot transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_file, 'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # 2. Spawn the robot URDF into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=['-file', urdf_file, '-entity', 'simple_4wd_robot'],
            # Optional: Specify the initial position (x, y, z, roll, pitch, yaw)
            # arguments=['-file', urdf_file, '-entity', 'simple_4wd_robot', '-x', '0', '-y', '0', '-z', '0.1']
        ),

        Node(
            package='controller_manager',  # ROS package that manages controllers
            executable='spawner',  # Controller manager spawner
            name='diff_drive_controller_spawner',
            arguments=[
                'diff_drive_controller',  # Name of the controller as defined in your config
                '--controller-manager', '/controller_manager'
            ],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],  # Spawn the differential drive controller
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
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),

    ])

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package directory path
    motor_controller_share = get_package_share_directory('motor_controller')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Path to the XACRO file
    xacro_file = os.path.join(motor_controller_share, 'gazebo_models', 'carModelURDF.urdf.xacro')
    
    # Get URDF via xacro
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}
    
    # Path to the controller config file
    controller_params_file = os.path.join(motor_controller_share,'gazebo_models','controller.yaml')

    return LaunchDescription([
        # Declare the argument for use_sim_time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': ''}.items()
        ),

        # Start robot_state_publisher first
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        # Start controller_manager with robot description from topic
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'controller_config_file': 'controller_params_file'},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/robot_description', '/robot_state_publisher/robot_description')
            ],
            output='screen'
        ),

        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'four_wheeled_robot'
            ],
            output='screen'
        ),

        # Start the operator node
        Node(
            package='motor_controller',
            executable='operatorNode',
            output='screen'
        ),

        # Load joint_state_broadcaster
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                         'joint_state_broadcaster'],
                    output='screen'
                )
            ]
        ),

        # Load diff_drive_controller
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                         'diff_drive_controller'],
                    output='screen'
                )
            ]
        )
    ])
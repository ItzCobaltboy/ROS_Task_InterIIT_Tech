import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    # Get the package directory path
    motor_controller_share = get_package_share_directory('motor_controller')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Path to the URDF file
    urdf_file = os.path.join(motor_controller_share, 'gazebo_models', 'carModelURDF.urdf')

    # Get URDF via xacro
    robot_description = {'robot_description': ParameterValue(urdf_file, value_type=str)}

    controller_params_file = os.path.join(motor_controller_share, 'gazebo_models', 'controllers.yaml')

    return LaunchDescription([

        # Declare the argument for use_sim_time (simulation time vs. wall time)
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        # Launch Gazebo
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': ''}.items()
        ),

        # 1. Start robot_state_publisher to publish robot transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
        output='screen'
        ),
        
        # 2. Spawn the robot URDF into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'four_wheeled_robot',
                '-x', '0',
                '-y', '0',
                '-z', '0.15'
        ],
        output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_params_file],
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

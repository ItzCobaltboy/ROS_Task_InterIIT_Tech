import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction


def generate_launch_description():

    # Get the package directory path
    motor_controller_share = get_package_share_directory('motor_controller')

    # Path to the URDF file
    urdf_file = os.path.join(motor_controller_share, 'gazebo_models', 'carModelURDF.xacro')
    
    # Gazebo path
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': ''}.items()  # Empty world
    )

    # Get URDF via xacro
    robot_description_content = Command(
        ['xacro ', urdf_file]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Load controllers configuration
    controller_params_file = os.path.join(motor_controller_share, 'config', 'controllers.yaml')

    return LaunchDescription([
        gazebo, 
        # Declare the argument for use_sim_time (simulation time vs. wall time)
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        # Start robot_state_publisher to publish robot transforms
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
        ),
        
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

        # Controller spawner for diff_drive_controller
        Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params_file],
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
        ),


        # Start the operator node (controller)
        Node(
            package='motor_controller',
            executable='operatorNode',
            output='screen'
        ),
    ])

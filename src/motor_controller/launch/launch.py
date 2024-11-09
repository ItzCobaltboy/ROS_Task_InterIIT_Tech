import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory for the motor_controller package (your robot package)
    motor_controller_pkg = get_package_share_directory('motor_controller')
    gazebo_models_pkg = os.path.join(motor_controller_pkg, 'gazebo_models')  # Path to the gazebo_models folder
    gazebo_world_pkg = os.path.join(motor_controller_pkg, 'gazebo_world')  # Path to the gazebo_world folder

    # Path to the world file (if needed for Gazebo)
    world_file = LaunchConfiguration('world', default='testing_world.world')

    # Path to the robot model (URDF/XACRO file)
    robot_model_path = PathJoinSubstitution([gazebo_models_pkg, 'carModelURDF.urdf.xacro'])

    return LaunchDescription([
        # Declare the 'robot_model' argument
        DeclareLaunchArgument('robot_model', default_value='carModelURDF',
                              description='Robot model name (e.g., URDF or XACRO file)'),
        
        # Declare the 'world' argument (if you want to pass a custom Gazebo world)
        DeclareLaunchArgument('world', default_value='testing_world.world',
                              description='Gazebo world file to load'),

        # Log the robot model selected (for debugging)
        LogInfo(
            condition=None,
            msg="Loading robot model: " + LaunchConfiguration('robot_model')
        ),

        # Launch Gazebo with the specified world
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo',
            output='screen',
            arguments=['--verbose', '--world', PathJoinSubstitution([gazebo_world_pkg, world_file])],
            parameters=[{'use_sim_time': 'true'}]
        ),

        # Launch the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': 'true'}],
            arguments=[robot_model_path]
        ),

        # Spawn the robot in Gazebo using the selected URDF/XACRO model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-file', robot_model_path,
                '-entity', LaunchConfiguration('robot_model'),
                '-x', '0', '-y', '0', '-z', '0.1',  # Adjust spawn position as needed
                '-R', '0', '-P', '0', '-Y', '0'
            ]
        ),

        # Launch the teleop_twist_keyboard node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            parameters=[{'use_sim_time': 'true'}]
        ),

        # Custom controller node for motor control
        Node(
            package='motor_controller',
            executable='operatorNode',
            name='Operator',
            output='screen'
        ),

        # Spawn the controller using controller_manager (ros2_control)
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            arguments=['--controller-manager', '/controller_manager', 'diff_drive_controller'],
        ),
    ])

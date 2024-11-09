from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(
            condition=None,
            msg="Launching Gazebo with ROS2...",
        ),
        
        Node(
            package='gazebo_ros',
            executable='gazebo',
            output='screen',
            arguments=['--verbose', '--world', '$(find my_package)/worlds/my_world.world']
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=['-topic', '/robot_description', '-entity', 'my_robot']
        ),
    ])

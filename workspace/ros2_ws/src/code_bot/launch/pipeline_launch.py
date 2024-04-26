# combined_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='panda_env',  # Package name containing GymNode
            executable='GymNode',  # Executable name for GymNode
            name='GymNode',  # Node name
            output='screen'  # Output behavior
        ),
        Node(
            package='robot_api_layer',  # Package name containing ros_api_node
            executable='robot_api_node',  # Executable name for ros_api_node
            name='robot_api_node',  # Node name
            output='screen'  # Output behavior
        ),
        Node(
            package='code_bot',  # Package name containing GymNode
            executable='bot_node',  # Executable name for GymNode
            name='bot_node',  # Node name
            output='screen'  # Output behavior
        )
    ])

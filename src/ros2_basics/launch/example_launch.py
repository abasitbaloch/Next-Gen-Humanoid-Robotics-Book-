from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch example nodes for the Physical AI & Humanoid Robotics book."""

    return LaunchDescription([
        Node(
            package='ros2_basics',
            executable='talker',
            name='talker',
            output='screen',
            parameters=[
                # Add parameters here if needed
            ]
        ),
        Node(
            package='ros2_basics',
            executable='listener',
            name='listener',
            output='screen',
        ),
    ])
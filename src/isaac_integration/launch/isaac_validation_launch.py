import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    validation_timeout = LaunchConfiguration('validation_timeout', default='30.0')
    output_dir = LaunchConfiguration('output_directory', default='/tmp/isaac_validation_results')

    # Isaac component validator node
    isaac_validator = Node(
        package='isaac_integration',
        executable='isaac_component_validator',
        name='isaac_component_validator',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'validation_timeout': validation_timeout},
            {'validation_output_dir': output_dir}
        ],
        output='screen'
    )

    # Isaac perception node (simulated)
    perception_node = Node(
        package='isaac_integration',
        executable='perception_node',
        name='perception_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Isaac navigation node (simulated)
    navigation_node = Node(
        package='isaac_integration',
        executable='navigation_node',
        name='navigation_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Isaac manipulation node (simulated)
    manipulation_node = Node(
        package='isaac_integration',
        executable='manipulation_node',
        name='manipulation_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))

    ld.add_action(DeclareLaunchArgument(
        'validation_timeout',
        default_value='30.0',
        description='Timeout for validation process in seconds'
    ))

    ld.add_action(DeclareLaunchArgument(
        'output_directory',
        default_value='/tmp/isaac_validation_results',
        description='Directory to save validation results'
    ))

    # Add nodes to launch description
    ld.add_action(perception_node)
    ld.add_action(navigation_node)
    ld.add_action(manipulation_node)
    ld.add_action(isaac_validator)

    return ld
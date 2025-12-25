import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    validation_threshold = LaunchConfiguration('validation_threshold', default='0.1')
    output_dir = LaunchConfiguration('output_directory', default='/tmp/validation_results')

    # Sim-to-real validator node
    sim_to_real_validator = Node(
        package='isaac_integration',
        executable='sim_to_real_validator',
        name='sim_to_real_validator',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'validation_threshold': validation_threshold},
            {'validation_output_dir': output_dir},
            {'validation_metrics': ['position', 'orientation', 'velocity']}
        ],
        output='screen'
    )

    # Isaac Sim bridge node (simulated)
    isaac_sim_bridge = Node(
        package='isaac_integration',
        executable='isaac_sim_bridge',
        name='isaac_sim_bridge',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Real robot bridge node (simulated)
    real_robot_bridge = Node(
        package='isaac_integration',
        executable='real_robot_bridge',
        name='real_robot_bridge',
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
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    ))

    ld.add_action(DeclareLaunchArgument(
        'validation_threshold',
        default_value='0.1',
        description='Threshold for validation acceptance'
    ))

    ld.add_action(DeclareLaunchArgument(
        'output_directory',
        default_value='/tmp/validation_results',
        description='Directory to save validation results'
    ))

    # Add nodes to launch description
    ld.add_action(sim_to_real_validator)
    ld.add_action(isaac_sim_bridge)
    ld.add_action(real_robot_bridge)

    return ld
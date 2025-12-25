import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    output_dir = LaunchConfiguration('output_directory', default='/tmp/synthetic_data')
    num_samples = LaunchConfiguration('num_samples', default='100')

    # Synthetic data generator node
    synthetic_data_generator = Node(
        package='isaac_integration',
        executable='synthetic_data_generator',
        name='synthetic_data_generator',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'output_directory': output_dir},
            {'num_samples': num_samples},
            {'image_width': 640},
            {'image_height': 480},
            {'data_types': ['rgb', 'depth', 'segmentation']}
        ],
        output='screen'
    )

    # Isaac Sim data collection node (simulated)
    isaac_sim_bridge = Node(
        package='isaac_integration',
        executable='isaac_sim_bridge',
        name='isaac_sim_bridge',
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
        'output_directory',
        default_value='/tmp/synthetic_data',
        description='Directory to save synthetic data'
    ))

    ld.add_action(DeclareLaunchArgument(
        'num_samples',
        default_value='100',
        description='Number of synthetic samples to generate'
    ))

    # Add nodes to launch description
    ld.add_action(synthetic_data_generator)
    ld.add_action(isaac_sim_bridge)

    return ld
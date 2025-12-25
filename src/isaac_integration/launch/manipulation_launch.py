import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Manipulation controller node
    manipulation_controller = Node(
        package='isaac_integration',
        executable='manipulation_controller',
        name='manipulation_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'arm_joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']},
            {'gripper_joints': ['gripper_joint']}
        ],
        output='screen'
    )

    # Joint trajectory controller for arm
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # Joint trajectory controller for gripper
    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # Register event handler to start controllers after controller manager is ready
    delay_arm_controller_after_spawner_started = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=manipulation_controller,
            on_start=[arm_controller],
        )
    )

    delay_gripper_controller_after_arm_started = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=arm_controller,
            on_start=[gripper_controller],
        )
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))

    # Add nodes to launch description
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))
    ld.add_action(manipulation_controller)
    ld.add_action(delay_arm_controller_after_spawner_started)
    ld.add_action(delay_gripper_controller_after_arm_started)

    return ld
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for a basic ROS 2 system with robot state publisher and joint state publisher.

    This launch file demonstrates:
    - Loading a robot description from URDF
    - Starting robot state publisher
    - Starting joint state publisher (for visualization)
    - Parameter configuration
    - Conditional launching
    """

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_description_path = DeclareLaunchArgument(
        'robot_description_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_basics'),
            'urdf',
            'basic_humanoid.urdf'
        ]),
        description='Path to robot URDF file'
    )

    # Get launch configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    robot_description_path_config = LaunchConfiguration('robot_description_path')

    # Robot State Publisher node
    # This node reads the robot description parameter and publishes the static transforms
    # between all links in the URDF to the tf2 tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'robot_description':
                PathJoinSubstitution([
                    FindPackageShare('ros2_basics'),
                    'urdf',
                    'basic_humanoid.urdf'
                ])
            }
        ],
        output='screen'
    )

    # Joint State Publisher node
    # This node publishes joint states for visualization in RViz
    # In a real robot, this would be replaced by actual joint state publisher
    # that reads from encoders
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        output='screen'
    )

    # Joint State Publisher GUI (optional)
    # This provides a GUI to manually control joint positions for visualization
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        output='screen'
    )

    # RViz2 node for visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('ros2_basics'),
                'rviz',
                'basic_humanoid.rviz'
            ])
        ],
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        output='screen'
    )

    # Example of a simple controller node (placeholder)
    # In a real system, this would be replaced with actual controllers
    controller_node = Node(
        package='ros2_basics',
        executable='simple_controller',  # This would be a real controller executable
        name='simple_controller',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'control_frequency': 50.0},
            {'max_velocity': 1.0}
        ],
        output='screen'
    )

    # Example of delayed startup using TimerAction
    # This could be useful for starting nodes in a specific order
    delayed_controller = TimerAction(
        period=5.0,  # Wait 5 seconds before starting
        actions=[
            Node(
                package='ros2_basics',
                executable='delayed_controller',  # This would be a real controller executable
                name='delayed_controller',
                parameters=[
                    {'use_sim_time': use_sim_time_config}
                ],
                output='screen'
            )
        ]
    )

    # Example of conditional launching
    # Launch the GUI only if a parameter is set to true
    launch_gui_arg = DeclareLaunchArgument(
        'launch_gui',
        default_value='false',
        description='Launch joint state publisher GUI'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    launch_gui_config = LaunchConfiguration('launch_gui')
    launch_rviz_config = LaunchConfiguration('launch_rviz')

    # Conditional launch for GUI
    from launch.conditions import IfCondition

    conditional_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui_conditional',
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        condition=IfCondition(launch_gui_config),
        output='screen'
    )

    conditional_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_conditional',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('ros2_basics'),
                'rviz',
                'basic_humanoid.rviz'
            ])
        ],
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        condition=IfCondition(launch_rviz_config),
        output='screen'
    )

    # Return the complete launch description
    # The order of nodes in the list doesn't determine execution order
    # Use TimerAction or EventHandlers for specific timing
    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        robot_description_path,
        launch_gui_arg,
        launch_rviz_arg,

        # Core nodes
        robot_state_publisher,
        joint_state_publisher,

        # Conditional nodes
        conditional_gui,
        conditional_rviz,

        # Delayed nodes
        delayed_controller,

        # Controller node
        controller_node,
    ])


def generate_basic_launch_description():
    """
    A simplified version of the launch file for basic usage.
    """
    return LaunchDescription([
        # Simple launch with default settings
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description':
                        PathJoinSubstitution([
                            FindPackageShare('ros2_basics'),
                            'urdf',
                            'basic_humanoid.urdf'
                        ])
                }
            ],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    ])


def generate_simulation_launch_description():
    """
    Launch file specifically for simulation with Gazebo.
    """
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'basic_humanoid',
            '-file', PathJoinSubstitution([
                FindPackageShare('ros2_basics'),
                'urdf',
                'basic_humanoid.urdf'
            ]),
            '-x', '0', '-y', '0', '-z', '1.0'  # Spawn at height 1.0
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot
    ])


if __name__ == '__main__':
    # This section is for testing the launch file syntax
    # In practice, launch files are called by ros2 launch command
    print("This is a ROS 2 launch file. Run with: ros2 launch ros2_basics basic_ros2_system.launch.py")
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for basic navigation simulation example with humanoid robot.
    This includes Gazebo simulation, navigation stack, and visualization tools.
    """

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    world_file = DeclareLaunchArgument(
        'world',
        default_value='maze_world.world',
        description='World file to load in Gazebo'
    )

    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot'
    )

    # Get launch configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    world_file_config = LaunchConfiguration('world')
    robot_name_config = LaunchConfiguration('robot_name')

    # Include Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('ros2_basics'),
                'worlds',
                world_file_config
            ]),
            'use_sim_time': use_sim_time_config
        }.items()
    )

    # Robot state publisher
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
                    'humanoid_gazebo.urdf.xacro'
                ])
            }
        ],
        output='screen'
    )

    # Joint state publisher (for simulation)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name_config,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',  # Start slightly above ground
            '-R', '0.0',  # Roll
            '-P', '0.0',  # Pitch
            '-Y', '0.0'   # Yaw
        ],
        output='screen'
    )

    # Navigation container
    navigation_container = ComposableNodeContainer(
        name='navigation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Map server
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'yaml_filename': 'package://ros2_basics/maps/empty_map.yaml'}
                ],
                remappings=[
                    ('/map', 'map'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
            ),

            # Local costmap
            ComposableNode(
                package='nav2_costmap_2d',
                plugin='nav2_costmap_2d::LocalCostmap',
                name='local_costmap',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'global_frame': 'odom'},
                    {'robot_base_frame': 'base_link'},
                    {'update_frequency': 5.0},
                    {'publish_frequency': 2.0},
                    {'width': 3.0},
                    {'height': 3.0},
                    {'resolution': 0.05},
                    {'origin_x': -1.5},
                    {'origin_y': -1.5},
                    {'footprint': '[[0.3, 0.3], [-0.3, 0.3], [-0.3, -0.3], [0.3, -0.3]]'},
                    {'plugins': ['voxel_layer', 'inflation_layer']},
                    {'inflation_layer.inflation_radius': 0.55}
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/local_costmap/costmap_raw', 'local_costmap/costmap_raw'),
                    ('/local_costmap/costmap', 'local_costmap/costmap'),
                    ('/local_costmap/clearing_endpoints', 'local_costmap/clearing_endpoints'),
                    ('/marker', 'local_costmap/marker')
                ]
            ),

            # Global costmap
            ComposableNode(
                package='nav2_costmap_2d',
                plugin='nav2_costmap_2d::GlobalCostmap',
                name='global_costmap',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'global_frame': 'map'},
                    {'robot_base_frame': 'base_link'},
                    {'update_frequency': 1.0},
                    {'publish_frequency': 0.5},
                    {'width': 20.0},
                    {'height': 20.0},
                    {'resolution': 0.05},
                    {'origin_x': -10.0},
                    {'origin_y': -10.0},
                    {'footprint': '[[0.3, 0.3], [-0.3, 0.3], [-0.3, -0.3], [0.3, -0.3]]'},
                    {'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer']},
                    {'inflation_layer.inflation_radius': 0.55}
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/global_costmap/costmap_raw', 'global_costmap/costmap_raw'),
                    ('/global_costmap/costmap', 'global_costmap/costmap'),
                    ('/global_costmap/clearing_endpoints', 'global_costmap/clearing_endpoints'),
                    ('/marker', 'global_costmap/marker')
                ]
            ),

            # Planner server
            ComposableNode(
                package='nav2_navfn_planner',
                plugin='nav2_navfn_planner::NavfnPlanner',
                name='navfn_planner',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'allow_unknown': True},
                    {'tolerance': 0.5},
                    {'use_astar': False},
                    {'planner_frequency': 1.0},
                    {'planner_thread_num': 1}
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/global_costmap/costmap_raw', 'global_costmap/costmap_raw'),
                    ('/global_costmap/costmap', 'global_costmap/costmap'),
                    ('/global_costmap/clearing_endpoints', 'global_costmap/clearing_endpoints'),
                    ('/marker', 'global_costmap/marker')
                ]
            ),

            # Controller server
            ComposableNode(
                package='nav2_simple_commander',
                plugin='nav2_simple_commander::SimpleCommander',
                name='simple_commander',
                parameters=[
                    {'use_sim_time': use_sim_time_config}
                ]
            ),

            # Behavior tree navigator
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'global_frame': 'map'},
                    {'robot_base_frame': 'base_link'},
                    {'odom_topic': '/odom'},
                    {'bt_xml_filename': 'package://nav2_bt_navigator/xml/navigate_w_replanning_and_recovery.xml'},
                    {'default_server_timeout': 20},
                    {'goal_checker.move_base.duration': 0.0},
                    {'goal_checker.move_base.dist_tol': 0.25},
                    {'goal_checker.move_base.angle_tol': 0.25}
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
            ),

            # Lifecycle manager for navigation
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'autostart': True},
                    {'node_names': [
                        'map_server',
                        'local_costmap',
                        'global_costmap',
                        'navfn_planner',
                        'simple_commander',
                        'bt_navigator'
                    ]}
                ]
            )
        ],
        output='screen'
    )

    # AMCL (Localization)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'set_initial_pose': True},
            {'initial_pose.x': 0.0},
            {'initial_pose.y': 0.0},
            {'initial_pose.z': 0.0},
            {'initial_pose.yaw': 0.0}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', 'scan')
        ],
        output='screen'
    )

    # Velocity smoother for humanoid locomotion
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'speed_lim_v': 0.5},  # Lower speed for humanoid balance
            {'speed_lim_w': 1.0},
            {'accel_lim_v': 0.5},
            {'accel_lim_w': 1.0},
            {'decel_lim_v': -0.5},
            {'decel_lim_w': -1.0}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_smoothed'),
            ('smoothed_cmd_vel', 'cmd_vel_out')
        ],
        output='screen'
    )

    # Waypoint follower for humanoid-specific navigation
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'loop_rate': 20},
            {'stop_on_failure': False},
            {' waypoint_task_executor_queue_size': 1}
        ],
        remappings=[
            ('/follow_waypoints', 'follow_waypoints'),
            ('/completed_waypoints', 'completed_waypoints')
        ],
        output='screen'
    )

    # RViz2 for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ros2_basics'),
        'rviz',
        'navigation_simulation.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        output='screen'
    )

    # Delay RViz2 launch until other nodes are ready
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz]
    )

    # Simple navigation tester node
    navigation_tester = Node(
        package='ros2_basics',
        executable='navigation_tester_node',
        name='navigation_tester',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'test_waypoints': [
                {'x': 2.0, 'y': 2.0, 'theta': 0.0},
                {'x': -2.0, 'y': 2.0, 'theta': 1.57},
                {'x': -2.0, 'y': -2.0, 'theta': 3.14},
                {'x': 2.0, 'y': -2.0, 'theta': -1.57}
            ]},
            {'waypoint_tolerance': 0.3},
            {'heading_tolerance': 0.2}
        ],
        output='screen'
    )

    # Delay navigation tester until after RViz2
    delayed_navigation_tester = TimerAction(
        period=10.0,
        actions=[navigation_tester]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        world_file,
        robot_name,

        # Gazebo simulation
        gazebo,

        # Robot state publishing
        robot_state_publisher,
        joint_state_publisher,

        # Spawn robot in Gazebo
        spawn_robot,

        # Navigation stack
        navigation_container,
        amcl,
        velocity_smoother,
        waypoint_follower,

        # Visualization
        delayed_rviz,

        # Test nodes
        delayed_navigation_tester
    ])


def create_simple_navigation_demo():
    """
    Alternative launch description for a simplified navigation demo.
    This version focuses on basic navigation without the full Nav2 stack.
    """
    return LaunchDescription([
        # Launch Gazebo with simple world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('ros2_basics'),
                    'worlds',
                    'simple_navigation_test.world'
                ])
            }.items()
        ),

        # Simple robot controller for basic navigation
        Node(
            package='ros2_basics',
            executable='simple_navigation_controller',
            name='simple_navigation_controller',
            parameters=[
                {'use_sim_time': True},
                {'linear_velocity': 0.3},
                {'angular_velocity': 0.5},
                {'min_obstacle_distance': 0.5}
            ],
            output='screen'
        ),

        # Obstacle detection node
        Node(
            package='ros2_basics',
            executable='obstacle_detector',
            name='obstacle_detector',
            parameters=[
                {'use_sim_time': True},
                {'scan_topic': '/scan'},
                {'min_distance': 0.6}
            ],
            output='screen'
        )
    ])
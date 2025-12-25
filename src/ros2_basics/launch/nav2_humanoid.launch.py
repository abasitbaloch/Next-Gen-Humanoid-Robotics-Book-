from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for Nav2 navigation stack configured specifically for humanoid robots.
    Includes all necessary nodes with humanoid-specific parameters and configurations.
    """

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_basics'),
            'config',
            'nav2_params_humanoid.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_basics'),
            'maps',
            'humanoid_lab.yaml'
        ]),
        description='Full path to map file to load'
    )

    use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed bringup if true'
    )

    use_respawn = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn if a node crashes'
    )

    # Get launch configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    params_file_config = LaunchConfiguration('params_file')
    autostart_config = LaunchConfiguration('autostart')
    map_yaml_file_config = LaunchConfiguration('map')
    use_composition_config = LaunchConfiguration('use_composition')
    use_respawn_config = LaunchConfiguration('use_respawn')

    # Set environment variables
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    # Remappings
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', 'cmd_vel_nav'),
        ('/odom', 'odom'),
        ('/scan', 'scan'),
        ('/camera/depth/image_raw', 'head_camera/depth/image_raw'),
        ('/camera/rgb/image_raw', 'head_camera/rgb/image_raw')
    ]

    # Launch the main Nav2 nodes
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time_config,
            'params_file': params_file_config,
            'autostart': autostart_config,
            'map': map_yaml_file_config,
            'use_composition': use_composition_config,
            'use_respawn': use_respawn_config
        }.items()
    )

    # Humanoid-specific navigation nodes
    humanoid_navigation_nodes = [
        # Humanoid-specific path planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='humanoid_planner_server',
            parameters=[params_file_config],
            remappings=remappings,
            output='screen'
        ),

        # Humanoid-specific controller with balance considerations
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='humanoid_controller_server',
            parameters=[params_file_config],
            remappings=remappings,
            output='screen'
        ),

        # Behavior server with humanoid-specific recovery behaviors
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='humanoid_bt_navigator',
            parameters=[params_file_config],
            remappings=remappings,
            output='screen'
        ),

        # Waypoint follower optimized for humanoid locomotion
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='humanoid_waypoint_follower',
            parameters=[params_file_config],
            remappings=remappings,
            output='screen'
        ),

        # Velocity smoother with humanoid-specific constraints
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='humanoid_velocity_smoother',
            parameters=[params_file_config],
            remappings=remappings,
            output='screen'
        ),

        # Lifecycle manager for navigation nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='humanoid_lifecycle_manager',
            parameters=[
                {'use_sim_time': use_sim_time_config},
                {'autostart': autostart_config},
                {'node_names': lifecycle_nodes}
            ],
            output='screen'
        )
    ]

    # Isaac ROS integration nodes
    isaac_ros_integration = [
        # Isaac ROS visual SLAM integration
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='isaac_visual_slam',
            parameters=[
                {'use_sim_time': use_sim_time_config},
                {'enable_rectification': True},
                {'enable_debug_mode': False},
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_frame': 'base_link'},
                {'publish_odom_tf': True}
            ],
            remappings=[
                ('/visual_slam/image_raw', '/head_camera/rgb/image_raw'),
                ('/visual_slam/camera_info', '/head_camera/rgb/camera_info'),
                ('/visual_slam/imu', '/imu/data')
            ],
            output='screen'
        ),

        # Isaac ROS perception integration
        Node(
            package='isaac_ros_perceptor',
            executable='perceptor_node',
            name='isaac_perceptor',
            parameters=[
                {'use_sim_time': use_sim_time_config},
                {'detection_topic': '/detections'},
                {'tracking_topic': '/tracks'},
                {'enable_dnn': True},
                {'enable_tracking': True}
            ],
            remappings=[
                ('/rgb', '/head_camera/rgb/image_raw'),
                ('/depth', '/head_camera/depth/image_raw')
            ],
            output='screen'
        ),

        # Isaac ROS stereo image processing
        Node(
            package='isaac_ros_stereo_image_proc',
            executable='disparity_node',
            name='isaac_disparity',
            parameters=[
                {'use_sim_time': use_sim_time_config},
                {'approximate_sync': True}
            ],
            remappings=[
                ('left/image_rect', '/head_camera/rgb/image_raw'),
                ('right/image_rect', '/head_camera/depth/image_raw'),
                ('left/camera_info', '/head_camera/rgb/camera_info'),
                ('right/camera_info', '/head_camera/depth/camera_info')
            ],
            output='screen'
        )
    ]

    # Navigation UI tools
    navigation_ui_nodes = [
        # Navigation 2D goal publisher
        Node(
            package='nav2_msgs',
            executable='nav2_2d_goal_publisher',
            name='nav2_2d_goal_publisher',
            parameters=[{'use_sim_time': use_sim_time_config}],
            output='screen'
        ),

        # Navigation lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_config,
                'autostart': autostart_config,
                'node_names': lifecycle_nodes
            }]
        )
    ]

    # Humanoid-specific navigation monitoring
    navigation_monitor = Node(
        package='ros2_basics',
        executable='humanoid_navigation_monitor',
        name='navigation_monitor',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'balance_threshold': 0.3},  # Radians
            {'stability_window': 1.0},   # Seconds
            {'navigation_frequency': 10.0}
        ],
        output='screen'
    )

    # Humanoid-specific path execution validator
    path_validator = Node(
        package='ros2_basics',
        executable='path_execution_validator',
        name='path_validator',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'step_size_limit': 0.1},    # Meters
            {'turn_rate_limit': 0.2},    # Radians/second
            {'balance_check_frequency': 20.0}
        ],
        output='screen'
    )

    # Combine all nodes
    all_nodes = [
        use_sim_time,
        params_file,
        autostart,
        map_yaml_file,
        use_composition,
        use_respawn,

        nav2_bringup_launch
    ]

    all_nodes.extend(humanoid_navigation_nodes)
    all_nodes.extend(isaac_ros_integration)
    all_nodes.extend(navigation_ui_nodes)
    all_nodes.extend([navigation_monitor, path_validator])

    return LaunchDescription(all_nodes)


def generate_humanoid_localization_launch_description():
    """
    Alternative launch file for localization-only setup for humanoid robots.
    """
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_basics'),
            'maps',
            'humanoid_lab.yaml'
        ]),
        description='Full path to map file to load'
    )

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_basics'),
            'config',
            'nav2_params_humanoid.yaml'
        ]),
        description='Full path to param file to load'
    )

    use_sim_time_config = LaunchConfiguration('use_sim_time')
    map_yaml_file_config = LaunchConfiguration('map')
    params_file_config = LaunchConfiguration('params_file')

    localization_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='humanoid_map_server',
            parameters=[{'use_sim_time': use_sim_time_config},
                       {'yaml_filename': map_yaml_file_config}],
            output='screen'
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='humanoid_amcl',
            parameters=[params_file_config],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{'use_sim_time': use_sim_time_config},
                       {'autostart': True},
                       {'node_names': ['map_server', 'amcl']}],
            output='screen'
        )
    ]

    return LaunchDescription([
        use_sim_time,
        map_yaml_file,
        params_file
    ] + localization_nodes)


def generate_humanoid_planning_only_launch_description():
    """
    Launch file for planning-only setup for humanoid robots.
    """
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_basics'),
            'config',
            'nav2_params_humanoid.yaml'
        ]),
        description='Full path to param file to load'
    )

    use_sim_time_config = LaunchConfiguration('use_sim_time')
    params_file_config = LaunchConfiguration('params_file')

    planning_nodes = [
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='humanoid_planner_server',
            parameters=[params_file_config],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            parameters=[{'use_sim_time': use_sim_time_config},
                       {'autostart': True},
                       {'node_names': ['humanoid_planner_server']}],
            output='screen'
        )
    ]

    return LaunchDescription([
        use_sim_time,
        params_file
    ] + planning_nodes)


def generate_humanoid_localization_launch_description():
    """
    Alternative launch file for localization-only setup for humanoid robots.
    """
    return LaunchDescription([
        # Localization-specific nodes
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='humanoid_map_server',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time', default='true')},
                {'yaml_filename': LaunchConfiguration('map', default='humanoid_lab.yaml')}
            ],
            output='screen'
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='humanoid_amcl',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('ros2_basics'),
                    'config',
                    'amcl_params_humanoid.yaml'
                ])
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/scan', 'scan'),
                ('/initialpose', 'initialpose'),
                ('/amcl_pose', 'amcl_pose')
            ],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time', default='true')},
                {'autostart': LaunchConfiguration('autostart', default='true')},
                {'node_names': ['map_server', 'amcl']}
            ],
            output='screen'
        )
    ])


def generate_humanoid_planning_only_launch_description():
    """
    Launch file for planning-only setup for humanoid robots.
    """
    return LaunchDescription([
        # Planning-specific nodes
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='humanoid_planner_server',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('ros2_basics'),
                    'config',
                    'planner_params_humanoid.yaml'
                ])
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/global_costmap/costmap', 'global_costmap/costmap'),
                ('/global_costmap/costmap_updates', 'global_costmap/costmap_updates')
            ],
            output='screen'
        ),

        # Controller for path following
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='humanoid_controller_server',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('ros2_basics'),
                    'config',
                    'controller_params_humanoid.yaml'
                ])
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/local_costmap/costmap', 'local_costmap/costmap'),
                ('/local_costmap/costmap_updates', 'local_costmap/costmap_updates')
            ],
            output='screen'
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time', default='true')},
                {'autostart': LaunchConfiguration('autostart', default='true')},
                {'node_names': ['humanoid_planner_server', 'humanoid_controller_server']}
            ],
            output='screen'
        )
    ])


def generate_humanoid_navigation_test_launch():
    """
    Launch file for testing humanoid navigation capabilities.
    """
    return LaunchDescription([
        # Launch the full navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros2_basics'),
                    'launch',
                    'nav2_humanoid.launch.py'
                ])
            ])
        ),

        # Test scenario nodes
        Node(
            package='ros2_basics',
            executable='navigation_test_scenario',
            name='navigation_test_scenario',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time', default='true')},
                {'test_scenarios': [
                    'simple_navigation',
                    'obstacle_avoidance',
                    'narrow_passage',
                    'dynamic_obstacles'
                ]},
                {'evaluation_criteria': {
                    'success_threshold': 0.8,
                    'time_limit': 60.0,
                    'collision_threshold': 0.1
                }}
            ],
            output='screen'
        ),

        # Performance monitoring
        Node(
            package='ros2_basics',
            executable='navigation_performance_monitor',
            name='navigation_performance_monitor',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time', default='true')},
                {'metrics_collection_frequency': 1.0}
            ],
            output='screen'
        )
    ])
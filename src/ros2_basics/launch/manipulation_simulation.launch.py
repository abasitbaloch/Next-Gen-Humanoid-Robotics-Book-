from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for manipulation simulation example with humanoid robot.
    This includes Gazebo simulation, manipulation stack, and visualization tools.
    """

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot'
    )

    world_file = DeclareLaunchArgument(
        'world',
        default_value='manipulation_lab.world',
        description='World file to load in Gazebo'
    )

    # Get launch configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    robot_name_config = LaunchConfiguration('robot_name')
    world_file_config = LaunchConfiguration('world')

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
                    'humanoid_manipulation.urdf.xacro'
                ])
            }
        ],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        output='screen'
    )

    # Joint state publisher GUI (for debugging)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        condition=launch.conditions.IfCondition(
            LaunchConfiguration('gui', default='false')
        )
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
            '-z', '0.5'
        ],
        output='screen'
    )

    # Manipulation stack container
    manipulation_container = launch_ros.actions.ComposableNodeContainer(
        name='manipulation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # MoveIt2 components
            launch_ros.descriptions.ComposableNode(
                package='moveit_ros_move_group',
                plugin='moveit::move_group::MoveGroupExe',
                name='move_group',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'publish_monitored_planning_scene': True},
                    {'capabilities': ' '.join([
                        'move_group/MoveGroupCartesianPathService',
                        'move_group/MoveGroupExecuteTrajectoryAction',
                        'move_group/MoveGroupGetPlanningSceneService',
                        'move_group/MoveGroupKinematicsService',
                        'move_group/MoveGroupMoveAction',
                        'move_group/MoveGroupPickPlaceAction',
                        'move_group/MoveGroupPlanService',
                        'move_group/MoveGroupQueryPlannersService',
                        'move_group/MoveGroupStateValidationService',
                        'move_group/MoveGroupWaypointPlanningAction'
                    ])},
                    {'max_safe_path_cost': 1},
                    {'jiggle_fraction': 0.05},
                    {'capabilities': ''},
                    {'disable_capabilities': ''}
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/move_group/monitored_planning_scene', '/monitored_planning_scene'),
                    ('/move_group/plan_execution/attempt', '/plan_execution/attempt'),
                    ('/move_group/plan_execution/fail', '/plan_execution/fail'),
                    ('/move_group/plan_execution/success', '/plan_execution/success'),
                    ('/move_group/execute_trajectory/cancel', '/execute_trajectory/cancel'),
                    ('/move_group/execute_trajectory/feedback', '/execute_trajectory/feedback'),
                    ('/move_group/execute_trajectory/goal', '/execute_trajectory/goal'),
                    ('/move_group/execute_trajectory/result', '/execute_trajectory/result')
                ]
            ),

            # Planning scene monitor
            launch_ros.descriptions.ComposableNode(
                package='moveit_ros_planning',
                plugin='planning_scene_monitor::PlanningSceneMonitor',
                name='planning_scene_monitor',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'planning_scene_monitor/publish_planning_scene': True},
                    {'planning_scene_monitor/publish_geometry_updates': True},
                    {'planning_scene_monitor/publish_state_updates': True},
                    {'planning_scene_monitor/publish_transforms_updates': True}
                ],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/attached_collision_object', 'attached_collision_object'),
                    ('/collision_object', 'collision_object'),
                    ('/planning_scene', 'planning_scene'),
                    ('/planning_scene_world', 'planning_scene_world')
                ]
            ),

            # Trajectory execution manager
            launch_ros.descriptions.ComposableNode(
                package='moveit_ros_planning',
                plugin='moveit::planning_interface::TrajectoryExecutionManager',
                name='trajectory_execution_manager',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'moveit_manage_controllers': True},
                    {'trajectory_execution.allowed_execution_duration_scaling': 1.2},
                    {'trajectory_execution.allowed_goal_duration_margin': 0.5},
                    {'trajectory_execution.allowed_start_tolerance': 0.01}
                ]
            ),

            # Controller manager
            launch_ros.descriptions.ComposableNode(
                package='controller_manager',
                plugin='controller_manager::ControllerManager',
                name='controller_manager',
                parameters=[
                    {'use_sim_time': use_sim_time_config}
                ]
            )
        ],
        output='screen'
    )

    # Joint trajectory controller
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '-c', '/controller_manager'
        ],
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ]
    )

    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/controller_manager'
        ],
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ]
    )

    # RViz2 for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ros2_basics'),
        'rviz',
        'manipulation_simulation.rviz'
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

    # MoveIt2 setup assistant (optional)
    moveit_setup_assistant = Node(
        package='moveit_setup_assistant',
        executable='moveit_setup_assistant',
        name='moveit_setup_assistant',
        arguments=[
            '--config_pkg', 'ros2_basics_moveit_config',
            '--urdf_path', PathJoinSubstitution([
                FindPackageShare('ros2_basics'),
                'urdf',
                'humanoid_manipulation.urdf.xacro'
            ])
        ],
        condition=launch.conditions.IfCondition(
            LaunchConfiguration('setup_assistant', default='false')
        )
    )

    # Object spawner for manipulation tasks
    object_spawner = Node(
        package='ros2_basics',
        executable='object_spawner_node',
        name='object_spawner',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'objects': [
                {'name': 'block_1', 'type': 'cube', 'size': [0.05, 0.05, 0.05], 'pose': [0.5, 0.2, 0.8, 0, 0, 0]},
                {'name': 'block_2', 'type': 'cylinder', 'radius': 0.03, 'height': 0.06, 'pose': [0.6, -0.1, 0.8, 0, 0, 0]},
                {'name': 'sphere_1', 'type': 'sphere', 'radius': 0.04, 'pose': [0.4, 0.0, 0.8, 0, 0, 0]}
            ]},
            {'spawn_frequency': 10.0}
        ],
        output='screen'
    )

    # Gripper controller (if robot has grippers)
    gripper_controller = Node(
        package='ros2_basics',
        executable='gripper_controller_node',
        name='gripper_controller',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'gripper_joint_names': ['left_gripper_joint', 'right_gripper_joint']},
            {'gripper_open_position': 0.05},
            {'gripper_closed_position': 0.01},
            {'gripper_speed': 0.01}
        ],
        output='screen'
    )

    # Simple manipulation controller
    manipulation_controller = Node(
        package='ros2_basics',
        executable='simple_manipulation_controller',
        name='manipulation_controller',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'planning_group': 'left_arm'},
            {'end_effector_link': 'left_hand'},
            {'tcp_offset': [0.0, 0.0, 0.1]},  # Offset to tip of gripper
            {'approach_distance': 0.1},
            {'lift_distance': 0.1}
        ],
        output='screen'
    )

    # Delay RViz launch until other nodes are ready
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz]
    )

    # Delay controller launch until RViz is up
    delayed_controller = TimerAction(
        period=10.0,
        actions=[manipulation_controller]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        robot_name,
        world_file,

        # Gazebo simulation
        gazebo,

        # Robot state publishing
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,

        # Spawn robot
        spawn_robot,

        # Controllers
        joint_state_broadcaster,
        joint_trajectory_controller,

        # Manipulation stack
        manipulation_container,

        # Object spawner
        object_spawner,

        # Gripper controller
        gripper_controller,

        # Visualization
        delayed_rviz,

        # Manipulation controller
        delayed_controller
    ])


def create_simple_manipulation_demo():
    """
    Alternative launch description for a simplified manipulation demo.
    This version focuses on basic manipulation without the full MoveIt2 stack.
    """
    return LaunchDescription([
        # Launch Gazebo with manipulation world
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
                    'simple_manipulation_test.world'
                ]),
                'use_sim_time': LaunchConfiguration('use_sim_time', default='true')
            }.items()
        ),

        # Robot with simple manipulation controller
        Node(
            package='ros2_basics',
            executable='simple_manipulation_controller',
            name='simple_manipulator',
            parameters=[
                {'use_sim_time': True},
                {'control_rate': 50.0},
                {'position_tolerance': 0.01},
                {'orientation_tolerance': 0.1}
            ],
            output='screen'
        ),

        # Object interaction detector
        Node(
            package='ros2_basics',
            executable='object_interaction_detector',
            name='object_detector',
            parameters=[
                {'use_sim_time': True},
                {'contact_threshold': 5.0},  # Force threshold for contact detection
                {'object_topic': '/gazebo/model_states'}
            ],
            output='screen'
        )
    ])


def create_pick_place_demo():
    """
    Launch description for pick-and-place demonstration.
    """
    return LaunchDescription([
        # Launch Gazebo with pick-and-place environment
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
                    'pick_place_demo.world'
                ]),
                'use_sim_time': LaunchConfiguration('use_sim_time', default='true')
            }.items()
        ),

        # Pick and place controller
        Node(
            package='ros2_basics',
            executable='pick_place_controller',
            name='pick_place_controller',
            parameters=[
                {'use_sim_time': True},
                {'approach_height': 0.1},
                {'grasp_height': 0.02},
                {'lift_height': 0.2},
                {'place_height': 0.05},
                {'gripper_grasp_force': 10.0}
            ],
            output='screen'
        ),

        # Task scheduler
        Node(
            package='ros2_basics',
            executable='task_scheduler',
            name='task_scheduler',
            parameters=[
                {'use_sim_time': True},
                {'task_sequence': [
                    {'action': 'detect_objects', 'params': {}},
                    {'action': 'select_object', 'params': {'priority': 'closest'}},
                    {'action': 'approach_object', 'params': {}},
                    {'action': 'grasp_object', 'params': {}},
                    {'action': 'lift_object', 'params': {}},
                    {'action': 'transport_object', 'params': {'destination': 'box_1'}},
                    {'action': 'place_object', 'params': {}},
                    {'action': 'return_home', 'params': {}}
                ]}
            ],
            output='screen'
        )
    ])
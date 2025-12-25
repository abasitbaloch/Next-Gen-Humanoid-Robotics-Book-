from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for simulation validation system.
    This launch file starts the validation nodes and runs comprehensive validation tests.
    """

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    validation_duration = DeclareLaunchArgument(
        'validation_duration',
        default_value='60.0',
        description='Duration of validation in seconds'
    )

    # Get launch configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    validation_duration_config = LaunchConfiguration('validation_duration')

    # Simulation validator node
    simulation_validator = Node(
        package='ros2_basics',
        executable='simulation_validator',
        name='simulation_validator',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'validation_duration': validation_duration_config},
            {'min_update_rate': 20.0},
            {'max_position_drift': 0.1},
            {'imu_orientation_threshold': 0.1},
            {'sensor_data_timeout': 5.0}
        ],
        output='screen',
        respawn=False
    )

    # Physics validator node
    physics_validator = Node(
        package='ros2_basics',
        executable='physics_validator',
        name='physics_validator',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'mass_tolerance': 0.1},
            {'inertia_tolerance': 0.01},
            {'friction_tolerance': 0.1},
            {'contact_stiffness_range': [1000000.0, 50000000.0]}
        ],
        output='screen',
        respawn=False
    )

    # Sensor feedback validator node
    sensor_validator = Node(
        package='ros2_basics',
        executable='sensor_feedback_validator',
        name='sensor_feedback_validator',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'data_consistency_threshold': 0.95},
            {'timing_accuracy_ms': 100.0},
            {'sensor_correlation_threshold': 0.7}
        ],
        output='screen',
        respawn=False
    )

    # Validation result publisher node
    validation_publisher = Node(
        package='ros2_basics',
        executable='validation_result_publisher',
        name='validation_result_publisher',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'validation_topic': '/simulation_validation/status'},
            {'result_file': '/tmp/validation_results.json'}
        ],
        output='screen',
        respawn=False
    )

    # Performance monitor node
    performance_monitor = Node(
        package='ros2_basics',
        executable='performance_monitor',
        name='performance_monitor',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'monitor_frequency': 1.0},
            {'cpu_threshold_percent': 80.0},
            {'memory_threshold_mb': 4096.0}
        ],
        output='screen',
        respawn=False
    )

    # Create a delayed action to run validation tests
    delayed_validation = TimerAction(
        period=5.0,  # Wait 5 seconds after other nodes start
        actions=[
            Node(
                package='ros2_basics',
                executable='validation_test_runner',
                name='validation_test_runner',
                parameters=[
                    {'use_sim_time': use_sim_time_config},
                    {'test_scenarios': [
                        'basic_functionality',
                        'navigation_test',
                        'manipulation_test',
                        'balance_test',
                        'sensor_validation'
                    ]}
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        validation_duration,

        # Validation nodes
        simulation_validator,
        physics_validator,
        sensor_validator,
        validation_publisher,
        performance_monitor,

        # Delayed validation runner
        delayed_validation
    ])


def create_basic_validation_nodes():
    """
    Create a minimal set of validation nodes for quick validation.
    """
    return LaunchDescription([
        Node(
            package='ros2_basics',
            executable='simulation_validator',
            name='basic_simulation_validator',
            parameters=[
                {'use_sim_time': True},
                {'validation_duration': 30.0},
                {'min_update_rate': 10.0}  # Lower requirement for basic validation
            ],
            output='screen'
        ),

        Node(
            package='ros2_basics',
            executable='validation_publisher',
            name='basic_validation_publisher',
            parameters=[
                {'use_sim_time': True},
                {'validation_topic': '/basic_validation/status'}
            ],
            output='screen'
        )
    ])


def create_comprehensive_validation_nodes():
    """
    Create all validation nodes for comprehensive validation.
    """
    return LaunchDescription([
        # All validation nodes as defined in main function
        Node(
            package='ros2_basics',
            executable='simulation_validator',
            name='comprehensive_simulation_validator',
            parameters=[
                {'use_sim_time': True},
                {'validation_duration': 120.0},  # Longer validation
                {'min_update_rate': 20.0},
                {'max_position_drift': 0.05},    # Stricter requirements
                {'imu_orientation_threshold': 0.05}
            ],
            output='screen'
        ),

        Node(
            package='ros2_basics',
            executable='physics_validator',
            name='comprehensive_physics_validator',
            parameters=[
                {'use_sim_time': True},
                {'mass_tolerance': 0.05},      # Stricter mass tolerance
                {'inertia_tolerance': 0.005},  # Stricter inertia tolerance
                {'friction_tolerance': 0.05},  # Stricter friction tolerance
            ],
            output='screen'
        ),

        Node(
            package='ros2_basics',
            executable='sensor_feedback_validator',
            name='comprehensive_sensor_validator',
            parameters=[
                {'use_sim_time': True},
                {'data_consistency_threshold': 0.98},  # Higher consistency requirement
                {'timing_accuracy_ms': 50.0},          # Stricter timing
                {'sensor_correlation_threshold': 0.8}  # Higher correlation requirement
            ],
            output='screen'
        ),

        Node(
            package='ros2_basics',
            executable='validation_result_publisher',
            name='comprehensive_validation_publisher',
            parameters=[
                {'use_sim_time': True},
                {'validation_topic': '/comprehensive_validation/status'},
                {'result_file': '/tmp/comprehensive_validation_results.json'},
                {'detailed_reporting': True}
            ],
            output='screen'
        ),

        # Additional validation tools
        Node(
            package='ros2_basics',
            executable='environment_validator',
            name='environment_validator',
            parameters=[
                {'use_sim_time': True},
                {'environment_checklist': [
                    'physics_engine_running',
                    'ground_plane_exists',
                    'lighting_proper',
                    'models_loaded_correctly',
                    'sensors_functioning'
                ]}
            ],
            output='screen'
        ),

        Node(
            package='ros2_basics',
            executable='robot_model_validator',
            name='robot_model_validator',
            parameters=[
                {'use_sim_time': True},
                {'model_path': 'package://ros2_basics/urdf/humanoid_model.urdf.xacro'},
                {'check_kinematics': True},
                {'check_dynamics': True},
                {'check_sensors': True}
            ],
            output='screen'
        ),

        Node(
            package='ros2_basics',
            executable='real_time_factor_monitor',
            name='rtf_monitor',
            parameters=[
                {'use_sim_time': True},
                {'target_rtf': 1.0},
                {'rtf_tolerance': 0.1},
                {'monitor_interval': 1.0}
            ],
            output='screen'
        )
    ])


def create_custom_validation_scenario(scenario_name):
    """
    Create validation nodes for specific scenarios.

    Args:
        scenario_name (str): Name of the validation scenario

    Returns:
        LaunchDescription: Launch description for the scenario
    """
    scenario_nodes = []

    if scenario_name == "balance_validation":
        scenario_nodes.extend([
            Node(
                package='ros2_basics',
                executable='balance_validator',
                name='balance_validation_node',
                parameters=[
                    {'use_sim_time': True},
                    {'com_threshold': 0.1},  # Center of mass threshold
                    {'balance_time_threshold': 10.0},  # Time to maintain balance
                    {'perturbation_tests': True}
                ],
                output='screen'
            )
        ])

    elif scenario_name == "navigation_validation":
        scenario_nodes.extend([
            Node(
                package='ros2_basics',
                executable='navigation_validator',
                name='navigation_validation_node',
                parameters=[
                    {'use_sim_time': True},
                    {'success_threshold': 0.8},  # 80% success rate
                    {'path_efficiency_threshold': 1.2},  # Path should be within 1.2x optimal
                    {'collision_threshold': 0.1}  # Max 10% collision rate
                ],
                output='screen'
            )
        ])

    elif scenario_name == "manipulation_validation":
        scenario_nodes.extend([
            Node(
                package='ros2_basics',
                executable='manipulation_validator',
                name='manipulation_validation_node',
                parameters=[
                    {'use_sim_time': True},
                    {'grasp_success_threshold': 0.7},  # 70% grasp success
                    {'placement_accuracy_threshold': 0.05},  # 5cm placement accuracy
                    {'task_completion_threshold': 0.8}  # 80% task completion
                ],
                output='screen'
            )
        ])

    elif scenario_name == "sensor_validation":
        scenario_nodes.extend([
            Node(
                package='ros2_basics',
                executable='sensor_validator',
                name='sensor_validation_node',
                parameters=[
                    {'use_sim_time': True},
                    {'camera_fps_threshold': 25.0},
                    {'lidar_accuracy_threshold': 0.02},  # 2cm accuracy
                    {'imu_drift_threshold': 0.1}  # 0.1 rad/s drift
                ],
                output='screen'
            )
        ])

    return LaunchDescription(scenario_nodes)


def get_validation_scenarios():
    """
    Return a list of available validation scenarios.

    Returns:
        list: List of validation scenario names
    """
    return [
        "balance_validation",
        "navigation_validation",
        "manipulation_validation",
        "sensor_validation",
        "basic_functionality",
        "comprehensive_validation",
        "environment_validation",
        "performance_validation"
    ]


def run_validation_workflow():
    """
    Run the complete validation workflow including setup, execution, and reporting.
    """
    # This would typically orchestrate the validation process
    # For now, we'll return the main launch description
    return generate_launch_description()


if __name__ == '__main__':
    # This would normally be called through ros2 launch
    # The function returns the launch description that would be used by the launch system
    ld = run_validation_workflow()
    print("Validation launch file created successfully.")
    print("Run with: ros2 launch ros2_basics simulation_validation.launch.py")
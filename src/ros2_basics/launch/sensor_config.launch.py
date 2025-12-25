from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for configuring sensors in simulation environment.
    Sets up camera, LIDAR, and IMU sensors for the humanoid robot.
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

    # Get launch configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    robot_name_config = LaunchConfiguration('robot_name')

    # Camera sensor node
    camera_node = Node(
        package='gazebo_ros',
        executable='camera',
        name='camera_driver',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'update_rate': 30.0},
            {'camera_info_url': 'package://ros2_basics/config/camera_info.yaml'},
            {'frame_name': 'camera_link'}
        ],
        remappings=[
            ('/camera/image_raw', '/camera/rgb/image_raw'),
            ('/camera/camera_info', '/camera/rgb/camera_info')
        ],
        output='screen'
    )

    # LIDAR sensor node
    lidar_node = Node(
        package='gazebo_ros',
        executable='laserscan',
        name='lidar_driver',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'update_rate': 10.0},
            {'frame_name': 'lidar_link'}
        ],
        remappings=[
            ('/scan', '/laser_scan')
        ],
        output='screen'
    )

    # IMU sensor node
    imu_node = Node(
        package='gazebo_ros',
        executable='imu',
        name='imu_driver',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'frame_name': 'imu_link'},
            {'update_rate': 100.0}
        ],
        remappings=[
            ('/imu/data', '/imu/data_raw'),
            ('/imu/mag', '/imu/magnetic_field')
        ],
        output='screen'
    )

    # Robot state publisher with sensor configuration
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

    # Gazebo simulation with configured robot
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
                'humanoid_test_world.world'
            ]),
            'use_sim_time': use_sim_time_config
        }.items()
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
            '-z', '1.0'
        ],
        output='screen'
    )

    # Sensor processing nodes
    # RGB-D processing node
    rgbd_processor = Node(
        package='image_proc',
        executable='image_proc',
        name='rgbd_processor',
        parameters=[
            {'use_sim_time': use_sim_time_config}
        ],
        output='screen'
    )

    # Point cloud processing
    pointcloud_processor = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[
            {'use_sim_time': use_sim_time_config},
            {'target_frame': 'base_link'},
            {'transform_tolerance': 0.01},
            {'min_height': 0.0},
            {'max_height': 1.0},
            {'angle_min': -1.5708},  # -M_PI/2
            {'angle_max': 1.5708},   # M_PI/2
            {'angle_increment': 0.0087},  # M_PI/360.0
            {'scan_time': 0.3333},
            {'range_min': 0.45},
            {'range_max': 4.0},
            {'use_inf': True},
            {'inf_epsilon': 1.0}
        ],
        remappings=[
            ('cloud_in', '/camera/depth/points'),
            ('scan', '/camera/depth/scan')
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        robot_name,

        # Launch Gazebo first
        gazebo,

        # Robot description nodes
        robot_state_publisher,
        joint_state_publisher,

        # Sensor drivers
        camera_node,
        lidar_node,
        imu_node,

        # Spawn robot after Gazebo is ready
        spawn_robot,

        # Sensor processing
        rgbd_processor,
        pointcloud_processor
    ])


def generate_sensor_configuration_nodes():
    """
    Alternative function to create standalone sensor configuration nodes.
    """
    return LaunchDescription([
        # Camera configuration node
        Node(
            package='sensor_msgs_py',
            executable='camera_publisher',
            name='sim_camera_config',
            parameters=[
                {'width': 640},
                {'height': 480},
                {'fps': 30},
                {'format': 'rgb8'}
            ],
            output='screen'
        ),

        # LIDAR configuration node
        Node(
            package='sensor_msgs_py',
            executable='lidar_publisher',
            name='sim_lidar_config',
            parameters=[
                {'range_min': 0.1},
                {'range_max': 10.0},
                {'angle_min': -3.14159},
                {'angle_max': 3.14159},
                {'angle_increment': 0.0174533},  # 1 degree
                {'scan_time': 0.1}
            ],
            output='screen'
        ),

        # IMU configuration node
        Node(
            package='sensor_msgs_py',
            executable='imu_publisher',
            name='sim_imu_config',
            parameters=[
                {'linear_acceleration_stddev': 0.017},
                {'angular_velocity_stddev': 0.001},
                {'orientation_stddev': 0.001}
            ],
            output='screen'
        )
    ])
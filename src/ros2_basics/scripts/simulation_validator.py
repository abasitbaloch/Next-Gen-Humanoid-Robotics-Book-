#!/usr/bin/env python3
"""
Simulation Validator for Humanoid Robot in Gazebo

This script validates that the simulation environment is running correctly with
proper robot physics and sensor feedback. It checks various aspects of the simulation
to ensure realistic behavior and data quality.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu, LaserScan, Image, CameraInfo
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
import numpy as np
import time
import threading
from collections import deque
import statistics


class SimulationValidator(Node):
    """
    Comprehensive validator for humanoid robot simulation in Gazebo.
    Validates physics, sensor data quality, and system stability.
    """

    def __init__(self):
        super().__init__('simulation_validator')

        # Validation parameters
        self.declare_parameter('validation_duration', 30.0)  # seconds
        self.declare_parameter('min_update_rate', 20.0)      # Hz
        self.declare_parameter('max_position_drift', 0.1)   # meters
        self.declare_parameter('imu_orientation_threshold', 0.1)  # radians
        self.declare_parameter('sensor_data_timeout', 5.0)  # seconds

        self.validation_duration = self.get_parameter('validation_duration').value
        self.min_update_rate = self.get_parameter('min_update_rate').value
        self.max_position_drift = self.get_parameter('max_position_drift').value
        self.imu_orientation_threshold = self.get_parameter('imu_orientation_threshold').value
        self.sensor_data_timeout = self.get_parameter('sensor_data_timeout').value

        # Data collection
        self.joint_states = []
        self.imu_data = []
        self.odom_data = []
        self.scan_data = []
        self.camera_data = []

        # Timestamp tracking for rate calculation
        self.joint_timestamps = deque(maxlen=100)
        self.imu_timestamps = deque(maxlen=100)
        self.odom_timestamps = deque(maxlen=100)
        self.scan_timestamps = deque(maxlen=100)
        self.camera_timestamps = deque(maxlen=100)

        # Subscriptions with appropriate QoS
        sensor_qos = qos_profile_sensor_data
        default_qos = QoSProfile(depth=10)

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, sensor_qos
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, default_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, sensor_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, sensor_qos
        )

        # Publishers for validation results
        self.validation_status_pub = self.create_publisher(Bool, '/simulation_validation/status', 10)
        self.validation_metrics_pub = self.create_publisher(Float32, '/simulation_validation/metrics', 10)

        # Validation timer
        self.validation_timer = self.create_timer(1.0, self.check_validation_status)

        # Start validation process
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.validation_results = {
            'physics_valid': False,
            'sensor_feedback_valid': False,
            'performance_acceptable': False,
            'stability_good': False
        }

        self.get_logger().info('Simulation Validator initialized and ready to validate')

    def joint_callback(self, msg):
        """Process joint state messages"""
        self.joint_states.append(msg)
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.joint_timestamps.append(current_time)

        # Validate joint positions are within reasonable bounds
        for pos in msg.position:
            if abs(pos) > 100:  # Unreasonable joint position
                self.get_logger().warn(f'Unreasonable joint position: {pos}')

    def imu_callback(self, msg):
        """Process IMU messages"""
        self.imu_data.append(msg)
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.imu_timestamps.append(current_time)

        # Validate quaternion normalization
        quat_norm = np.sqrt(
            msg.orientation.x**2 +
            msg.orientation.y**2 +
            msg.orientation.z**2 +
            msg.orientation.w**2
        )

        if abs(quat_norm - 1.0) > 0.01:
            self.get_logger().warn(f'IMU quaternion not normalized: {quat_norm}')

        # Validate linear acceleration magnitude (should be ~9.81 m/s² when stationary)
        acc_magnitude = np.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )

        if acc_magnitude > 20.0:  # Too high for normal operation
            self.get_logger().warn(f'High IMU acceleration: {acc_magnitude} m/s²')

    def odom_callback(self, msg):
        """Process odometry messages"""
        self.odom_data.append(msg)
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.odom_timestamps.append(current_time)

        # Check if position is drifting unexpectedly
        pos = msg.pose.pose.position
        position_magnitude = np.sqrt(pos.x**2 + pos.y**2 + pos.z**2)

        if position_magnitude > 100.0:  # Robot shouldn't be this far away
            self.get_logger().warn(f'Unexpectedly large position: {position_magnitude} m')

    def scan_callback(self, msg):
        """Process LIDAR scan messages"""
        self.scan_data.append(msg)
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.scan_timestamps.append(current_time)

        # Validate scan ranges
        for i, range_val in enumerate(msg.ranges):
            if not (msg.range_min <= range_val <= msg.range_max or
                    np.isnan(range_val) or np.isinf(range_val)):
                self.get_logger().warn(f'Invalid LIDAR range at index {i}: {range_val}')

    def camera_callback(self, msg):
        """Process camera image messages"""
        self.camera_data.append(msg)
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.camera_timestamps.append(current_time)

        # Validate image data
        expected_size = msg.width * msg.height * 3  # Assuming RGB
        if len(msg.data) != expected_size:
            self.get_logger().warn(f'Camera image size mismatch: {len(msg.data)} vs {expected_size}')

    def camera_info_callback(self, msg):
        """Process camera info messages"""
        # Validate camera parameters
        if msg.width <= 0 or msg.height <= 0:
            self.get_logger().warn(f'Invalid camera dimensions: {msg.width}x{msg.height}')

    def check_validation_status(self):
        """Periodic validation checks"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time

        # Calculate update rates
        joint_rate = self.calculate_update_rate(self.joint_timestamps)
        imu_rate = self.calculate_update_rate(self.imu_timestamps)
        odom_rate = self.calculate_update_rate(self.odom_timestamps)
        scan_rate = self.calculate_update_rate(self.scan_timestamps)
        camera_rate = self.calculate_update_rate(self.camera_timestamps)

        # Validate update rates
        rates_valid = all([
            joint_rate >= self.min_update_rate * 0.8,  # Allow some tolerance
            imu_rate >= self.min_update_rate * 0.8,
            odom_rate >= self.min_update_rate * 0.8,
            scan_rate >= self.min_update_rate * 0.5,  # LIDAR can be slower
            camera_rate >= self.min_update_rate * 0.5  # Camera can be slower
        ])

        # Validate sensor data quality
        sensor_data_valid = self.validate_sensor_data_quality()

        # Validate physics stability
        physics_stable = self.validate_physics_stability()

        # Update validation results
        self.validation_results = {
            'physics_valid': physics_stable,
            'sensor_feedback_valid': sensor_data_valid,
            'performance_acceptable': rates_valid,
            'stability_good': physics_stable and sensor_data_valid
        }

        # Log validation status
        status_msg = Bool()
        status_msg.data = all(self.validation_results.values())
        self.validation_status_pub.publish(status_msg)

        # Calculate overall validation metric
        metric_msg = Float32()
        metric_msg.data = sum(self.validation_results.values()) / len(self.validation_results)
        self.validation_metrics_pub.publish(metric_msg)

        # Print status summary
        self.get_logger().info(f'Validation Status (Elapsed: {elapsed_time:.1f}s):')
        self.get_logger().info(f'  Joint Rate: {joint_rate:.1f}Hz (req: {self.min_update_rate}Hz)')
        self.get_logger().info(f'  IMU Rate: {imu_rate:.1f}Hz')
        self.get_logger().info(f'  Odom Rate: {odom_rate:.1f}Hz')
        self.get_logger().info(f'  Physics Valid: {physics_stable}')
        self.get_logger().info(f'  Sensors Valid: {sensor_data_valid}')
        self.get_logger().info(f'  Overall Status: {"VALID" if status_msg.data else "INVALID"}')

    def calculate_update_rate(self, timestamps):
        """Calculate update rate from timestamp deque"""
        if len(timestamps) < 2:
            return 0.0

        time_diffs = [timestamps[i] - timestamps[i-1] for i in range(1, len(timestamps))]
        avg_dt = sum(time_diffs) / len(time_diffs) if time_diffs else 1.0

        return 1.0 / avg_dt if avg_dt > 0 else 0.0

    def validate_sensor_data_quality(self):
        """Validate the quality of sensor data"""
        if not self.joint_states or not self.imu_data:
            return False

        # Check for reasonable joint positions
        latest_joint_state = self.joint_states[-1]
        for pos in latest_joint_state.position:
            if abs(pos) > 10:  # Very large joint positions are suspicious
                return False

        # Check IMU data quality
        latest_imu = self.imu_data[-1]
        quat_norm = np.sqrt(
            latest_imu.orientation.x**2 +
            latest_imu.orientation.y**2 +
            latest_imu.orientation.z**2 +
            latest_imu.orientation.w**2
        )

        if abs(quat_norm - 1.0) > 0.05:  # More tolerant for data quality check
            return False

        # Check for reasonable accelerations
        acc_mag = np.sqrt(
            latest_imu.linear_acceleration.x**2 +
            latest_imu.linear_acceleration.y**2 +
            latest_imu.linear_acceleration.z**2
        )

        if acc_mag > 50.0:  # Very high acceleration indicates issues
            return False

        # If we have odometry data, check for reasonable velocities
        if self.odom_data:
            latest_odom = self.odom_data[-1]
            vel_mag = np.sqrt(
                latest_odom.twist.twist.linear.x**2 +
                latest_odom.twist.twist.linear.y**2 +
                latest_odom.twist.twist.linear.z**2
            )

            if vel_mag > 10.0:  # Unreasonably fast for humanoid
                return False

        return True

    def validate_physics_stability(self):
        """Validate physics simulation stability"""
        if not self.odom_data or len(self.odom_data) < 10:
            return True  # Not enough data to validate

        # Get recent positions
        recent_positions = []
        for odom_msg in self.odom_data[-10:]:
            pos = odom_msg.pose.pose.position
            recent_positions.append([pos.x, pos.y, pos.z])

        # Calculate position variance to check for stability
        if len(recent_positions) > 1:
            pos_array = np.array(recent_positions)
            pos_variance = np.var(pos_array, axis=0)
            total_variance = np.sum(pos_variance)

            # If variance is too high, physics might be unstable
            if total_variance > 0.1:  # Threshold for stability
                self.get_logger().warn(f'High position variance indicating potential instability: {total_variance}')
                return False

        # Check for extremely high velocities that might indicate instability
        recent_velocities = []
        for odom_msg in self.odom_data[-10:]:
            twist = odom_msg.twist.twist
            vel = np.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
            recent_velocities.append(vel)

        if recent_velocities:
            max_vel = max(recent_velocities)
            if max_vel > 5.0:  # Very high velocity indicates instability
                return False

        return True

    def run_comprehensive_validation(self):
        """Run comprehensive validation for the specified duration"""
        self.get_logger().info(f'Starting comprehensive validation for {self.validation_duration} seconds...')

        start_time = self.get_clock().now().nanoseconds / 1e9
        end_time = start_time + self.validation_duration

        validation_thread = threading.Thread(target=self._validation_worker, args=(end_time,))
        validation_thread.start()

        # Wait for validation to complete
        while self.get_clock().now().nanoseconds / 1e9 < end_time:
            time.sleep(0.1)

        validation_thread.join()

        # Generate final report
        self.generate_validation_report()

    def _validation_worker(self, end_time):
        """Worker thread for validation"""
        while self.get_clock().now().nanoseconds / 1e9 < end_time:
            time.sleep(1.0)

    def generate_validation_report(self):
        """Generate a comprehensive validation report"""
        report = "\n" + "="*60 + "\n"
        report += "SIMULATION VALIDATION REPORT\n"
        report += "="*60 + "\n"

        # Calculate final metrics
        joint_rate = self.calculate_update_rate(self.joint_timestamps)
        imu_rate = self.calculate_update_rate(self.imu_timestamps)
        odom_rate = self.calculate_update_rate(self.odom_timestamps)
        scan_rate = self.calculate_update_rate(self.scan_timestamps)
        camera_rate = self.calculate_update_rate(self.camera_timestamps)

        report += f"Update Rates:\n"
        report += f"  Joint States: {joint_rate:.2f} Hz (min req: {self.min_update_rate})\n"
        report += f"  IMU: {imu_rate:.2f} Hz\n"
        report += f"  Odometry: {odom_rate:.2f} Hz\n"
        report += f"  LIDAR: {scan_rate:.2f} Hz\n"
        report += f"  Camera: {camera_rate:.2f} Hz\n\n"

        report += f"Validation Results:\n"
        for key, value in self.validation_results.items():
            status = "PASS" if value else "FAIL"
            report += f"  {key.replace('_', ' ').title()}: {status}\n"

        # Calculate overall score
        passed_checks = sum(self.validation_results.values())
        total_checks = len(self.validation_results)
        overall_score = (passed_checks / total_checks) * 100

        report += f"\nOverall Score: {overall_score:.1f}% ({passed_checks}/{total_checks} checks passed)\n"

        if overall_score >= 80:
            report += "Status: SIMULATION VALIDATION PASSED\n"
        elif overall_score >= 60:
            report += "Status: SIMULATION VALIDATION CONDITIONAL (Manual review needed)\n"
        else:
            report += "Status: SIMULATION VALIDATION FAILED\n"

        report += "="*60 + "\n"

        self.get_logger().info(report)

        # Save report to file
        with open('/tmp/simulation_validation_report.txt', 'w') as f:
            f.write(report)

        self.get_logger().info('Validation report saved to /tmp/simulation_validation_report.txt')


def main(args=None):
    """Main function to run the simulation validator"""
    rclpy.init(args=args)

    validator = SimulationValidator()

    try:
        # Run comprehensive validation
        validator.run_comprehensive_validation()

        # Continue running to monitor ongoing validation
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Simulation validation interrupted by user')
        validator.generate_validation_report()
    finally:
        validator.destroy_node()
        rclpy.shutdown()


class PhysicsValidator(Node):
    """
    Specialized validator for physics parameters in humanoid simulation.
    """
    def __init__(self):
        super().__init__('physics_validator')

        # Physics-specific validation parameters
        self.declare_parameter('mass_tolerance', 0.1)  # kg
        self.declare_parameter('inertia_tolerance', 0.01)  # kg*m²
        self.declare_parameter('friction_tolerance', 0.1)
        self.declare_parameter('contact_stiffness_range', [1000000.0, 50000000.0])

        self.mass_tolerance = self.get_parameter('mass_tolerance').value
        self.inertia_tolerance = self.get_parameter('inertia_tolerance').value
        self.friction_tolerance = self.get_parameter('friction_tolerance').value
        self.contact_stiffness_range = self.get_parameter('contact_stiffness_range').value

        # Publishers and subscribers for physics validation
        self.physics_param_sub = self.create_subscription(
            Float32, '/physics_parameters', self.physics_param_callback, 10
        )

        self.physics_validation_pub = self.create_publisher(
            Bool, '/physics_validation/status', 10
        )

        self.get_logger().info('Physics Validator initialized')

    def physics_param_callback(self, msg):
        """Receive physics parameter updates for validation"""
        # This would typically receive parameter updates from simulation
        # For now, we'll just log the validation approach
        self.get_logger().info(f'Received physics parameter: {msg.data}')
        self.validate_physics_parameters(msg.data)

    def validate_physics_parameters(self, param_value):
        """Validate specific physics parameters"""
        # Example validation for contact stiffness
        if self.contact_stiffness_range[0] <= param_value <= self.contact_stiffness_range[1]:
            is_valid = True
            self.get_logger().info(f'Physics parameter {param_value} is within valid range')
        else:
            is_valid = False
            self.get_logger().warn(f'Physics parameter {param_value} is OUTSIDE valid range')

        # Publish validation result
        status_msg = Bool()
        status_msg.data = is_valid
        self.physics_validation_pub.publish(status_msg)

    def validate_robot_mass_distribution(self, robot_description):
        """Validate that robot mass distribution is physically plausible"""
        # This would parse URDF/SDF and validate mass properties
        # For now, we'll just outline the validation approach
        total_mass = 0.0
        center_of_mass = np.array([0.0, 0.0, 0.0])

        # In a real implementation, this would iterate through all links
        # and calculate total mass and center of mass
        # For humanoid robots, total mass should be reasonable (e.g., 30-100kg)
        # and center of mass should be within the torso for stability

        return total_mass > 20.0 and total_mass < 150.0  # Reasonable humanoid mass range

    def validate_joint_dynamics(self, joint_states, joint_commands):
        """Validate joint dynamics behavior"""
        # Check for reasonable joint velocities and accelerations
        # Verify joint limits are respected
        # Check for excessive joint forces that might indicate physics issues
        pass


class SensorFeedbackValidator(Node):
    """
    Specialized validator for sensor feedback quality and consistency.
    """
    def __init__(self):
        super().__init__('sensor_feedback_validator')

        # Sensor feedback validation parameters
        self.declare_parameter('data_consistency_threshold', 0.95)
        self.declare_parameter('timing_accuracy_ms', 100.0)
        self.declare_parameter('sensor_correlation_threshold', 0.7)

        self.consistency_threshold = self.get_parameter('data_consistency_threshold').value
        self.timing_accuracy_ms = self.get_parameter('timing_accuracy_ms').value
        self.correlation_threshold = self.get_parameter('sensor_correlation_threshold').value

        # Multiple sensor subscriptions for correlation analysis
        self.multi_sensor_sub = self.create_subscription(
            JointState, '/joint_states', self.multi_sensor_callback, 10
        )

        self.get_logger().info('Sensor Feedback Validator initialized')

    def multi_sensor_callback(self, msg):
        """Analyze multiple sensor correlations"""
        # This would correlate data from multiple sensors
        # to validate consistency and accuracy
        pass


if __name__ == '__main__':
    main()
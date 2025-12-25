#!/usr/bin/env python3

"""
Sim-to-Real Transfer Validation for Isaac Integration

This script validates the transfer of models and behaviors from simulation to real hardware.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float64
import numpy as np
import cv2
from cv_bridge import CvBridge
import message_filters
from scipy.spatial.transform import Rotation as R
import json
import os


class SimToRealValidator(Node):
    def __init__(self):
        super().__init__('sim_to_real_validator')

        # Declare parameters
        self.declare_parameter('validation_threshold', 0.1)
        self.declare_parameter('validation_output_dir', '/tmp/validation_results')
        self.declare_parameter('validation_metrics', ['position', 'orientation', 'velocity'])

        # Get parameters
        self.validation_threshold = self.get_parameter('validation_threshold').value
        self.validation_output_dir = self.get_parameter('validation_output_dir').value
        self.validation_metrics = self.get_parameter('validation_metrics').value

        # Create output directory
        os.makedirs(self.validation_output_dir, exist_ok=True)

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # Subscribers for simulation and real data
        self.sim_pose_sub = message_filters.Subscriber(self, PoseStamped, '/sim/pose')
        self.real_pose_sub = message_filters.Subscriber(self, PoseStamped, '/real/pose')

        # Synchronize sim and real pose messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sim_pose_sub, self.real_pose_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.pose_validation_callback)

        # Subscribers for other sensor data
        self.sim_image_sub = self.create_subscription(
            Image,
            '/sim/camera/image_raw',
            self.sim_image_callback,
            10
        )

        self.real_image_sub = self.create_subscription(
            Image,
            '/real/camera/image_raw',
            self.real_image_callback,
            10
        )

        self.sim_scan_sub = self.create_subscription(
            LaserScan,
            '/sim/scan',
            self.sim_scan_callback,
            10
        )

        self.real_scan_sub = self.create_subscription(
            LaserScan,
            '/real/scan',
            self.real_scan_callback,
            10
        )

        # Publishers for validation results
        self.validation_result_pub = self.create_publisher(String, '/validation/result', 10)
        self.validation_score_pub = self.create_publisher(Float64, '/validation/score', 10)

        # Storage for validation data
        self.validation_results = []
        self.sim_images = []
        self.real_images = []
        self.sim_scans = []
        self.real_scans = []

        self.get_logger().info('Sim-to-Real Validator initialized')

    def pose_validation_callback(self, sim_pose_msg, real_pose_msg):
        """Validate pose similarity between simulation and real data"""
        # Extract pose data
        sim_pos = np.array([sim_pose_msg.pose.position.x,
                           sim_pose_msg.pose.position.y,
                           sim_pose_msg.pose.position.z])
        sim_ori = np.array([sim_pose_msg.pose.orientation.x,
                           sim_pose_msg.pose.orientation.y,
                           sim_pose_msg.pose.orientation.z,
                           sim_pose_msg.pose.orientation.w])

        real_pos = np.array([real_pose_msg.pose.position.x,
                            real_pose_msg.pose.position.y,
                            real_pose_msg.pose.position.z])
        real_ori = np.array([real_pose_msg.pose.orientation.x,
                            real_pose_msg.pose.orientation.y,
                            real_pose_msg.pose.orientation.z,
                            real_pose_msg.pose.orientation.w])

        # Calculate position error
        pos_error = np.linalg.norm(sim_pos - real_pos)

        # Calculate orientation error
        # Convert quaternions to rotation matrices and compare
        sim_rot = R.from_quat(sim_ori).as_matrix()
        real_rot = R.from_quat(real_ori).as_matrix()

        # Calculate rotation error using Frobenius norm
        rot_error = np.linalg.norm(sim_rot - real_rot, 'fro')

        # Check if errors are within threshold
        pos_valid = pos_error <= self.validation_threshold
        rot_valid = rot_error <= self.validation_threshold

        # Calculate overall validation score
        score = 1.0 - min(1.0, (pos_error + rot_error) / 2.0)

        # Store validation result
        result = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'position_error': float(pos_error),
            'orientation_error': float(rot_error),
            'position_valid': bool(pos_valid),
            'orientation_valid': bool(rot_valid),
            'overall_score': float(score)
        }

        self.validation_results.append(result)

        # Publish validation result
        result_msg = String()
        result_msg.data = json.dumps(result)
        self.validation_result_pub.publish(result_msg)

        score_msg = Float64()
        score_msg.data = score
        self.validation_score_pub.publish(score_msg)

        # Log validation status
        status = "PASS" if (pos_valid and rot_valid) else "FAIL"
        self.get_logger().info(f'Pose validation: {status} - Position error: {pos_error:.3f}, Orientation error: {rot_error:.3f}, Score: {score:.3f}')

    def sim_image_callback(self, msg):
        """Process simulation image data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.sim_images.append(cv_image)
            self.get_logger().debug(f'Received sim image: {msg.width}x{msg.height}')
        except Exception as e:
            self.get_logger().error(f'Error processing sim image: {e}')

    def real_image_callback(self, msg):
        """Process real image data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.real_images.append(cv_image)
            self.get_logger().debug(f'Received real image: {msg.width}x{msg.height}')
        except Exception as e:
            self.get_logger().error(f'Error processing real image: {e}')

    def sim_scan_callback(self, msg):
        """Process simulation laser scan data"""
        self.sim_scans.append(msg)
        self.get_logger().debug(f'Received sim scan with {len(msg.ranges)} ranges')

    def real_scan_callback(self, msg):
        """Process real laser scan data"""
        self.real_scans.append(msg)
        self.get_logger().debug(f'Received real scan with {len(msg.ranges)} ranges')

    def validate_image_similarity(self):
        """Validate similarity between sim and real images"""
        if len(self.sim_images) == 0 or len(self.real_images) == 0:
            return 0.0

        # Calculate image similarity using SSIM (simplified)
        # In a real implementation, you would use a proper image similarity metric
        sim_img = self.sim_images[-1]
        real_img = self.real_images[-1]

        # Resize images to same size if needed
        if sim_img.shape != real_img.shape:
            real_img = cv2.resize(real_img, (sim_img.shape[1], sim_img.shape[0]))

        # Calculate MSE (Mean Squared Error)
        mse = np.mean((sim_img - real_img) ** 2)

        # Convert to similarity score (0-1, where 1 is most similar)
        max_pixel_value = 255.0
        similarity = 1.0 - (mse / (max_pixel_value ** 2))

        return max(0.0, similarity)

    def validate_scan_similarity(self):
        """Validate similarity between sim and real laser scans"""
        if len(self.sim_scans) == 0 or len(self.real_scans) == 0:
            return 0.0

        sim_scan = self.sim_scans[-1]
        real_scan = self.real_scans[-1]

        # Calculate scan similarity (simplified)
        # In a real implementation, you would use proper scan matching algorithms
        if len(sim_scan.ranges) != len(real_scan.ranges):
            min_len = min(len(sim_scan.ranges), len(real_scan.ranges))
            sim_ranges = np.array(sim_scan.ranges[:min_len])
            real_ranges = np.array(real_scan.ranges[:min_len])
        else:
            sim_ranges = np.array(sim_scan.ranges)
            real_ranges = np.array(real_scan.ranges)

        # Calculate mean absolute error between scans
        mae = np.mean(np.abs(sim_ranges - real_ranges))

        # Convert to similarity score (0-1, where 1 is most similar)
        max_range = max(np.max(sim_ranges), np.max(real_ranges))
        if max_range > 0:
            similarity = 1.0 - (mae / max_range)
        else:
            similarity = 1.0

        return max(0.0, similarity)

    def save_validation_report(self):
        """Save validation results to file"""
        report = {
            'validation_metrics': self.validation_metrics,
            'total_results': len(self.validation_results),
            'average_score': np.mean([r['overall_score'] for r in self.validation_results]) if self.validation_results else 0.0,
            'image_similarity': self.validate_image_similarity(),
            'scan_similarity': self.validate_scan_similarity(),
            'results': self.validation_results
        }

        report_path = os.path.join(self.validation_output_dir, 'validation_report.json')
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)

        self.get_logger().info(f'Validation report saved to: {report_path}')


def main(args=None):
    rclpy.init(args=args)

    validator = SimToRealValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.save_validation_report()
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
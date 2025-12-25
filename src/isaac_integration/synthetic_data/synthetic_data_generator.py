#!/usr/bin/env python3

"""
Synthetic Data Generation Pipeline for Isaac Integration

This script generates synthetic data for training and validation
of perception systems in Isaac Sim environment.
"""

import os
import json
import yaml
import cv2
import numpy as np
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import random


class SyntheticDataGenerator(Node):
    def __init__(self):
        super().__init__('synthetic_data_generator')

        # Declare parameters
        self.declare_parameter('output_directory', '/tmp/synthetic_data')
        self.declare_parameter('num_samples', 100)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('data_types', ['rgb', 'depth', 'segmentation'])

        # Get parameters
        self.output_dir = self.get_parameter('output_directory').value
        self.num_samples = self.get_parameter('num_samples').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.data_types = self.get_parameter('data_types').value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'images'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'labels'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'metadata'), exist_ok=True)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Publishers for synthetic data
        self.rgb_pub = self.create_publisher(Image, '/synthetic_rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/synthetic_depth', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)

        # Timer for data generation
        self.timer = self.create_timer(0.1, self.generate_data_callback)

        # Sample counter
        self.sample_counter = 0

        self.get_logger().info(f'Synthetic Data Generator initialized. Output directory: {self.output_dir}')

    def generate_scene(self):
        """Generate a synthetic scene with random objects and lighting"""
        # Create a random RGB image
        rgb_image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        # Add random background color
        bg_color = [random.randint(50, 200) for _ in range(3)]
        rgb_image[:, :] = bg_color

        # Add random objects
        num_objects = random.randint(1, 5)
        for _ in range(num_objects):
            # Random object properties
            obj_type = random.choice(['circle', 'rectangle', 'triangle'])
            color = [random.randint(0, 255) for _ in range(3)]
            x = random.randint(50, self.image_width - 50)
            y = random.randint(50, self.image_height - 50)
            size = random.randint(20, 80)

            if obj_type == 'circle':
                cv2.circle(rgb_image, (x, y), size, color, -1)
            elif obj_type == 'rectangle':
                pt1 = (x - size//2, y - size//2)
                pt2 = (x + size//2, y + size//2)
                cv2.rectangle(rgb_image, pt1, pt2, color, -1)
            elif obj_type == 'triangle':
                points = np.array([
                    [x, y - size//2],
                    [x - size//2, y + size//2],
                    [x + size//2, y + size//2]
                ], np.int32)
                cv2.fillPoly(rgb_image, [points], color)

        return rgb_image

    def generate_depth_map(self, rgb_image):
        """Generate corresponding depth map for the RGB image"""
        # Create a simple depth map based on object positions
        depth_map = np.zeros((self.image_height, self.image_width), dtype=np.float32)

        # Create depth variations based on image content
        for y in range(self.image_height):
            for x in range(self.image_width):
                # Simple depth based on position (farther as we go down/right)
                depth = 1.0 + (y / self.image_height) * 9.0  # 1-10 meters
                depth_map[y, x] = depth

        return depth_map

    def generate_segmentation_mask(self, rgb_image):
        """Generate segmentation mask for the RGB image"""
        # Create a segmentation mask where each object has a unique ID
        segmentation_mask = np.zeros((self.image_height, self.image_width), dtype=np.uint8)

        # For simplicity, we'll create a basic mask
        # In a real implementation, this would be more sophisticated
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        _, segmentation_mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        return segmentation_mask

    def generate_data_callback(self):
        """Generate synthetic data samples"""
        if self.sample_counter >= self.num_samples:
            self.get_logger().info('Completed generating synthetic data samples')
            self.timer.cancel()
            return

        # Generate synthetic scene
        rgb_image = self.generate_scene()
        depth_map = self.generate_depth_map(rgb_image)
        segmentation_mask = self.generate_segmentation_mask(rgb_image)

        # Create ROS messages
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.header.frame_id = 'camera_rgb_optical_frame'

        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='32FC1')
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = 'camera_depth_optical_frame'

        # Publish synthetic data
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)

        # Save data to disk
        self.save_sample(rgb_image, depth_map, segmentation_mask)

        self.sample_counter += 1
        self.get_logger().info(f'Generated synthetic sample {self.sample_counter}/{self.num_samples}')

    def save_sample(self, rgb_image, depth_map, segmentation_mask):
        """Save synthetic data sample to disk"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

        # Save RGB image
        rgb_path = os.path.join(self.output_dir, 'images', f'rgb_{timestamp}.png')
        cv2.imwrite(rgb_path, rgb_image)

        # Save depth map
        depth_path = os.path.join(self.output_dir, 'images', f'depth_{timestamp}.png')
        cv2.imwrite(depth_path, (depth_map * 255).astype(np.uint8))

        # Save segmentation mask
        seg_path = os.path.join(self.output_dir, 'labels', f'segmentation_{timestamp}.png')
        cv2.imwrite(seg_path, segmentation_mask)

        # Save metadata
        metadata = {
            'timestamp': timestamp,
            'sample_id': self.sample_counter,
            'image_width': self.image_width,
            'image_height': self.image_height,
            'rgb_path': f'images/rgb_{timestamp}.png',
            'depth_path': f'images/depth_{timestamp}.png',
            'segmentation_path': f'labels/segmentation_{timestamp}.png',
            'objects_count': random.randint(1, 5),
            'scene_description': 'Synthetic scene with random objects'
        }

        metadata_path = os.path.join(self.output_dir, 'metadata', f'metadata_{timestamp}.json')
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)

    def generate_camera_info(self):
        """Generate camera info for synthetic camera"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = 'camera_rgb_optical_frame'
        camera_info.width = self.image_width
        camera_info.height = self.image_height

        # Default camera matrix (adjust based on your camera parameters)
        camera_info.k = [615.179443, 0.000000, 320.500000,
                         0.000000, 615.179443, 240.500000,
                         0.000000, 0.000000, 1.000000]

        # Default distortion coefficients
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Rectification matrix
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix
        camera_info.p = [615.179443, 0.000000, 320.500000, 0.000000,
                         0.000000, 615.179443, 240.500000, 0.000000,
                         0.000000, 0.000000, 1.000000, 0.000000]

        return camera_info


def main(args=None):
    rclpy.init(args=args)

    synthetic_data_generator = SyntheticDataGenerator()

    try:
        rclpy.spin(synthetic_data_generator)
    except KeyboardInterrupt:
        pass
    finally:
        synthetic_data_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
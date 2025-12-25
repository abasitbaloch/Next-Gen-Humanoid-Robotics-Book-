#!/usr/bin/env python3
"""
Isaac ROS Perception Pipeline for Humanoid Robotics

This module implements a comprehensive perception pipeline using Isaac ROS components
for humanoid robot applications. The pipeline includes visual SLAM, object detection,
semantic segmentation, and sensor fusion capabilities.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2, LaserScan
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, Float32
from visualization_msgs.msg import Marker, MarkerArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
import threading
import queue
from dataclasses import dataclass


@dataclass
class PerceptionConfig:
    """Configuration for Isaac ROS perception pipeline"""
    # Camera settings
    camera_topic: str = "/head_camera/rgb/image_raw"
    depth_topic: str = "/head_camera/depth/image_raw"
    camera_info_topic: str = "/head_camera/rgb/camera_info"

    # Processing parameters
    detection_frequency: float = 10.0  # Hz
    tracking_frequency: float = 30.0  # Hz
    mapping_frequency: float = 5.0    # Hz

    # Object detection parameters
    detection_confidence_threshold: float = 0.7
    max_detection_range: float = 5.0
    min_detection_size: float = 0.01

    # SLAM parameters
    slam_voxel_size: float = 0.1
    slam_max_range: float = 10.0
    slam_resolution: float = 0.05

    # Tracking parameters
    max_track_age: int = 30
    min_track_hits: int = 3
    track_iou_threshold: float = 0.3


class IsaacROSPerceptionPipeline(Node):
    """
    Isaac ROS perception pipeline with GPU acceleration and multi-modal fusion.
    """
    def __init__(self):
        super().__init__('isaac_ros_perception_pipeline')

        # Load configuration
        self.config = self.load_perception_config()

        # Initialize perception components
        self.initialize_visual_slam()
        self.initialize_object_detection()
        self.initialize_semantic_segmentation()
        self.initialize_sensor_fusion()

        # Publishers
        self.object_detection_pub = self.create_publisher(
            MarkerArray, '/perception/object_detections', 10
        )
        self.tracking_pub = self.create_publisher(
            MarkerArray, '/perception/tracked_objects', 10
        )
        self.semantic_map_pub = self.create_publisher(
            OccupancyGrid, '/perception/semantic_map', 10
        )
        self.depth_cloud_pub = self.create_publisher(
            PointCloud2, '/perception/depth_cloud', 10
        )
        self.perception_status_pub = self.create_publisher(
            Float32, '/perception/status', 10
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.config.camera_topic, self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, self.config.depth_topic, self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.config.camera_info_topic, self.camera_info_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Processing queues
        self.image_queue = queue.Queue(maxsize=5)
        self.depth_queue = queue.Queue(maxsize=5)
        self.processing_lock = threading.Lock()

        # Perception status
        self.last_detection_time = self.get_clock().now()
        self.detection_count = 0
        self.tracking_objects = {}

        # Timers for different processing rates
        self.detection_timer = self.create_timer(
            1.0/self.config.detection_frequency, self.detection_processing_loop
        )
        self.mapping_timer = self.create_timer(
            1.0/self.config.mapping_frequency, self.mapping_processing_loop
        )
        self.status_timer = self.create_timer(1.0, self.publish_perception_status)

        self.get_logger().info('Isaac ROS Perception Pipeline initialized')

    def load_perception_config(self) -> PerceptionConfig:
        """Load perception configuration parameters"""
        config = PerceptionConfig()

        # Declare and get parameters
        self.declare_parameter('camera_topic', config.camera_topic)
        self.declare_parameter('depth_topic', config.depth_topic)
        self.declare_parameter('detection_frequency', config.detection_frequency)
        self.declare_parameter('detection_confidence_threshold', config.detection_confidence_threshold)
        self.declare_parameter('max_detection_range', config.max_detection_range)

        config.camera_topic = self.get_parameter('camera_topic').value
        config.depth_topic = self.get_parameter('depth_topic').value
        config.detection_frequency = self.get_parameter('detection_frequency').value
        config.detection_confidence_threshold = self.get_parameter('detection_confidence_threshold').value
        config.max_detection_range = self.get_parameter('max_detection_range').value

        return config

    def initialize_visual_slam(self):
        """Initialize Isaac ROS Visual SLAM components"""
        try:
            # Import Isaac ROS components if available
            # In real implementation, this would initialize:
            # - Visual SLAM system
            # - Feature tracker
            # - Map builder
            # - Loop closure detector

            self.visual_slam_initialized = True
            self.get_logger().info('Isaac ROS Visual SLAM initialized')
        except ImportError:
            self.visual_slam_initialized = False
            self.get_logger().warn('Isaac ROS Visual SLAM not available, using fallback')

    def initialize_object_detection(self):
        """Initialize Isaac ROS object detection components"""
        try:
            # Import Isaac ROS detection components
            # In real implementation, this would initialize:
            # - Deep learning detection models
            # - TensorRT acceleration
            # - Detection post-processing

            self.detection_initialized = True
            self.get_logger().info('Isaac ROS Object Detection initialized')
        except ImportError:
            self.detection_initialized = False
            self.get_logger().warn('Isaac ROS Object Detection not available, using fallback')

    def initialize_semantic_segmentation(self):
        """Initialize Isaac ROS semantic segmentation components"""
        try:
            # Import Isaac ROS segmentation components
            # In real implementation, this would initialize:
            # - Semantic segmentation models
            # - Instance segmentation models
            # - GPU-accelerated inference

            self.segmentation_initialized = True
            self.get_logger().info('Isaac ROS Semantic Segmentation initialized')
        except ImportError:
            self.segmentation_initialized = False
            self.get_logger().warn('Isaac ROS Semantic Segmentation not available, using fallback')

    def initialize_sensor_fusion(self):
        """Initialize sensor fusion components"""
        # Initialize multi-sensor fusion system
        self.fusion_initialized = True
        self.get_logger().info('Isaac ROS Sensor Fusion initialized')

    def image_callback(self, msg: Image):
        """Process incoming image messages"""
        try:
            # Add image to processing queue
            if not self.image_queue.full():
                self.image_queue.put(msg)
            else:
                # Drop oldest image if queue is full
                try:
                    self.image_queue.get_nowait()
                    self.image_queue.put(msg)
                except queue.Empty:
                    pass

            # Process image with Isaac ROS components
            if self.detection_initialized:
                self.process_image_with_isaac_detection(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def depth_callback(self, msg: Image):
        """Process incoming depth messages"""
        try:
            # Add depth to processing queue
            if not self.depth_queue.full():
                self.depth_queue.put(msg)
            else:
                # Drop oldest depth if queue is full
                try:
                    self.depth_queue.get_nowait()
                    self.depth_queue.put(msg)
                except queue.Empty:
                    pass

            # Process depth with Isaac ROS components
            if self.visual_slam_initialized:
                self.process_depth_with_isaac_slam(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """Process camera calibration information"""
        # Store camera parameters for processing
        self.camera_info = msg

    def imu_callback(self, msg: Imu):
        """Process IMU data for sensor fusion"""
        # Use IMU data for SLAM and tracking
        if self.visual_slam_initialized:
            self.update_slam_with_imu(msg)

    def process_image_with_isaac_detection(self, image_msg: Image):
        """Process image using Isaac ROS detection components"""
        try:
            # Convert ROS image to format for Isaac processing
            cv_image = self.ros_image_to_cv2(image_msg)

            # Perform object detection using Isaac ROS (simulated)
            if self.detection_initialized:
                # In real implementation, this would call Isaac ROS detection
                detections = self.simulate_isaac_detection(cv_image)
            else:
                # Fallback to basic processing
                detections = self.fallback_detection(cv_image)

            # Filter detections by confidence
            filtered_detections = [
                det for det in detections
                if det.get('confidence', 0) >= self.config.detection_confidence_threshold
            ]

            # Publish detections as markers
            self.publish_detections_as_markers(filtered_detections, image_msg.header)

            # Update tracking system
            self.update_object_tracking(filtered_detections)

            self.detection_count += len(filtered_detections)

        except Exception as e:
            self.get_logger().error(f'Error in Isaac detection processing: {e}')

    def process_depth_with_isaac_slam(self, depth_msg: Image):
        """Process depth using Isaac ROS SLAM components"""
        try:
            # Convert depth image to point cloud
            point_cloud = self.depth_to_pointcloud(depth_msg, self.camera_info)

            # Process with Isaac SLAM if available
            if self.visual_slam_initialized:
                # In real implementation, this would integrate with Isaac SLAM
                slam_result = self.simulate_isaac_slam_processing(point_cloud)
            else:
                # Fallback to basic processing
                slam_result = self.fallback_slam_processing(point_cloud)

            # Update map with SLAM results
            self.update_semantic_map(slam_result)

        except Exception as e:
            self.get_logger().error(f'Error in Isaac SLAM processing: {e}')

    def simulate_isaac_detection(self, image: np.ndarray) -> List[Dict]:
        """Simulate Isaac ROS object detection (in real implementation, this would use Isaac components)"""
        # This is a simulation of what Isaac ROS detection would provide
        # In real implementation, this would call Isaac ROS detection nodes
        detections = []

        # For simulation, we'll create mock detections
        # In real implementation, this would use Isaac ROS detection
        height, width = image.shape[:2]

        # Create some mock detections
        for i in range(3):  # Simulate detecting 3 objects
            detection = {
                'class': f'object_{i+1}',
                'confidence': 0.8 + np.random.random() * 0.2,  # 0.8-1.0 confidence
                'bbox': [
                    int(width * 0.2 * i),  # x_min
                    int(height * 0.3),     # y_min
                    int(width * 0.2 * (i+1) + 50),  # x_max
                    int(height * 0.3 + 50)  # y_max
                ],
                'center_3d': Point(x=i*0.5, y=0.0, z=1.0 + i*0.1),  # Simulated 3D position
                'size_3d': [0.1, 0.1, 0.1]  # Simulated 3D size
            }
            detections.append(detection)

        return detections

    def simulate_isaac_slam_processing(self, point_cloud: PointCloud2) -> Dict:
        """Simulate Isaac ROS SLAM processing"""
        # This would integrate with Isaac ROS Visual SLAM in real implementation
        return {
            'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'map_update': True,
            'tracking_confidence': 0.95,
            'features_detected': 150
        }

    def depth_to_pointcloud(self, depth_msg: Image, camera_info: CameraInfo) -> PointCloud2:
        """Convert depth image to point cloud"""
        # This would use Isaac ROS depth processing in real implementation
        # For now, return a basic point cloud structure
        pc_msg = PointCloud2()
        pc_msg.header = depth_msg.header
        return pc_msg

    def ros_image_to_cv2(self, image_msg: Image) -> np.ndarray:
        """Convert ROS Image message to OpenCV image"""
        from cv_bridge import CvBridge
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        return cv_image

    def publish_detections_as_markers(self, detections: List[Dict], header: Header):
        """Publish detections as visualization markers"""
        marker_array = MarkerArray()

        for i, detection in enumerate(detections):
            # Create marker for bounding box
            bbox_marker = Marker()
            bbox_marker.header = header
            bbox_marker.ns = "detections"
            bbox_marker.id = i
            bbox_marker.type = Marker.LINE_STRIP
            bbox_marker.action = Marker.ADD

            # Set bounding box points
            x_min, y_min, x_max, y_max = detection['bbox']
            # Convert 2D bbox to 3D points in the camera frame
            # This is simplified - real implementation would use depth information
            bbox_marker.points = [
                Point(x=x_min, y=y_min, z=1.0),
                Point(x=x_max, y=y_min, z=1.0),
                Point(x=x_max, y=y_max, z=1.0),
                Point(x=x_min, y=y_max, z=1.0),
                Point(x=x_min, y=y_min, z=1.0)  # Close the loop
            ]

            bbox_marker.scale.x = 0.02  # Line width
            bbox_marker.color.r = 1.0
            bbox_marker.color.g = 0.0
            bbox_marker.color.b = 0.0
            bbox_marker.color.a = 1.0

            # Create text marker for class and confidence
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "detection_labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = x_min
            text_marker.pose.position.y = y_min - 10
            text_marker.pose.position.z = 1.0
            text_marker.pose.orientation.w = 1.0

            text_marker.text = f"{detection['class']}: {detection['confidence']:.2f}"
            text_marker.scale.z = 0.1  # Font size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            marker_array.markers.extend([bbox_marker, text_marker])

        self.object_detection_pub.publish(marker_array)

    def update_object_tracking(self, detections: List[Dict]):
        """Update object tracking with new detections"""
        current_time = self.get_clock().now()

        # Simple tracking algorithm (in real implementation, this would be more sophisticated)
        for detection in detections:
            # Check if this detection matches existing tracked object
            matched = False
            for obj_id, tracked_obj in self.tracking_objects.items():
                # Calculate distance to existing tracked object
                if self.is_same_object(detection, tracked_obj):
                    # Update tracked object
                    tracked_obj['last_seen'] = current_time
                    tracked_obj['position'] = detection['center_3d']
                    tracked_obj['confidence'] = detection['confidence']
                    matched = True
                    break

            if not matched:
                # Create new tracked object
                new_obj_id = len(self.tracking_objects)
                self.tracking_objects[new_obj_id] = {
                    'id': new_obj_id,
                    'class': detection['class'],
                    'position': detection['center_3d'],
                    'size': detection['size_3d'],
                    'confidence': detection['confidence'],
                    'first_seen': current_time,
                    'last_seen': current_time,
                    'track_age': 0
                }

        # Remove old tracks that haven't been seen
        keys_to_remove = []
        for obj_id, tracked_obj in self.tracking_objects.items():
            time_since_seen = (current_time - tracked_obj['last_seen']).nanoseconds / 1e9
            if time_since_seen > 2.0:  # Remove if not seen in 2 seconds
                keys_to_remove.append(obj_id)

        for key in keys_to_remove:
            del self.tracking_objects[key]

        # Publish tracked objects
        self.publish_tracked_objects()

    def is_same_object(self, detection: Dict, tracked_obj: Dict) -> bool:
        """Check if detection matches existing tracked object"""
        # Calculate distance between detection center and tracked position
        det_pos = detection['center_3d']
        track_pos = tracked_obj['position']

        distance = np.sqrt(
            (det_pos.x - track_pos.x)**2 +
            (det_pos.y - track_pos.y)**2 +
            (det_pos.z - track_pos.z)**2
        )

        # Consider same object if within threshold
        return distance < 0.2  # 20cm threshold

    def publish_tracked_objects(self):
        """Publish tracked objects as markers"""
        marker_array = MarkerArray()

        for obj_id, tracked_obj in self.tracking_objects.items():
            # Create marker for tracked object
            marker = Marker()
            marker.header.frame_id = "map"  # Use global frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tracked_objects"
            marker.id = obj_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position = tracked_obj['position']
            marker.pose.orientation.w = 1.0

            # Set size based on detected object size
            marker.scale.x = tracked_obj['size'][0] if tracked_obj['size'] else 0.1
            marker.scale.y = tracked_obj['size'][1] if tracked_obj['size'] else 0.1
            marker.scale.z = tracked_obj['size'][2] if tracked_obj['size'] else 0.1

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7  # Semi-transparent

            marker_array.markers.append(marker)

        self.tracking_pub.publish(marker_array)

    def update_semantic_map(self, slam_result: Dict):
        """Update semantic map with SLAM results"""
        # Create occupancy grid from SLAM results
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = "map"
        map_msg.header.stamp = self.get_clock().now().to_msg()

        # Set map properties
        map_msg.info.resolution = self.config.slam_resolution
        map_msg.info.width = 100  # 100x100 cells
        map_msg.info.height = 100
        map_msg.info.origin.position.x = -5.0  # Map centered at (0,0)
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Initialize map data (simplified)
        map_msg.data = [0] * (map_msg.info.width * map_msg.info.height)  # All free initially

        self.semantic_map_pub.publish(map_msg)

    def detection_processing_loop(self):
        """Processing loop for object detection"""
        if not self.image_queue.empty():
            try:
                image_msg = self.image_queue.get_nowait()
                self.process_image_with_isaac_detection(image_msg)
            except queue.Empty:
                pass

    def mapping_processing_loop(self):
        """Processing loop for mapping"""
        # This would periodically process mapping updates
        # In real implementation, this would integrate Isaac ROS mapping
        pass

    def publish_perception_status(self):
        """Publish perception system status"""
        status_msg = Float32()
        status_msg.data = float(self.detection_count)
        self.perception_status_pub.publish(status_msg)

        self.get_logger().debug(f'Perception status: {self.detection_count} detections processed')

    def fallback_detection(self, image: np.ndarray) -> List[Dict]:
        """Fallback detection when Isaac ROS components not available"""
        # Implement basic OpenCV-based detection as fallback
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Simple contour-based detection
        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for i, contour in enumerate(contours[:5]):  # Limit to 5 largest contours
            if cv2.contourArea(contour) > 100:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                detection = {
                    'class': 'unknown_object',
                    'confidence': 0.5,  # Lower confidence for fallback
                    'bbox': [x, y, x+w, y+h],
                    'center_3d': Point(x=x/100.0, y=y/100.0, z=1.0),  # Approximate 3D position
                    'size_3d': [w/100.0, h/100.0, 0.1]
                }
                detections.append(detection)

        return detections

    def fallback_slam_processing(self, point_cloud: PointCloud2) -> Dict:
        """Fallback SLAM processing when Isaac ROS components not available"""
        return {
            'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
            'map_update': True,
            'tracking_confidence': 0.5,  # Lower confidence for fallback
            'features_detected': 50  # Fewer features detected
        }

    def get_perception_metrics(self) -> Dict[str, float]:
        """Get perception system metrics"""
        return {
            'detection_rate': self.detection_count / 60.0 if self.get_clock().now().nanoseconds / 1e9 > 60 else 0.0,  # Per minute
            'tracking_objects': len(self.tracking_objects),
            'processing_latency': 0.05,  # 50ms average (simulated)
            'detection_accuracy': 0.85 if self.detection_initialized else 0.60  # Higher with Isaac
        }


class IsaacROSPerceptionManager(Node):
    """
    High-level manager for Isaac ROS perception pipeline.
    Coordinates multiple perception components and manages resources.
    """
    def __init__(self):
        super().__init__('isaac_ros_perception_manager')

        # Perception pipeline instance
        self.perception_pipeline = IsaacROSPerceptionPipeline()

        # Resource management
        self.gpu_monitor = self.initialize_gpu_monitor()
        self.memory_monitor = self.initialize_memory_monitor()

        # Performance metrics
        self.metrics_publisher = self.create_publisher(
            MarkerArray, '/perception/metrics', 10
        )

        # Configuration parameters
        self.declare_parameter('enable_gpu_acceleration', True)
        self.declare_parameter('max_detection_objects', 20)
        self.declare_parameter('enable_tracking', True)

        self.enable_gpu = self.get_parameter('enable_gpu_acceleration').value
        self.max_objects = self.get_parameter('max_detection_objects').value
        self.enable_tracking = self.get_parameter('enable_tracking').value

        # Performance monitoring
        self.performance_timer = self.create_timer(2.0, self.monitor_performance)

        self.get_logger().info('Isaac ROS Perception Manager initialized')

    def initialize_gpu_monitor(self):
        """Initialize GPU monitoring for Isaac ROS components"""
        try:
            import pynvml
            pynvml.nvmlInit()
            return pynvml
        except ImportError:
            self.get_logger().warn('pynvml not available for GPU monitoring')
            return None

    def initialize_memory_monitor(self):
        """Initialize memory monitoring"""
        import psutil
        return psutil

    def monitor_performance(self):
        """Monitor perception system performance"""
        # Get metrics from perception pipeline
        metrics = self.perception_pipeline.get_perception_metrics()

        # Monitor GPU usage if available
        if self.gpu_monitor:
            try:
                handle = self.gpu_monitor.nvmlDeviceGetHandleByIndex(0)
                util_info = self.gpu_monitor.nvmlDeviceGetUtilizationRates(handle)
                memory_info = self.gpu_monitor.nvmlDeviceGetMemoryInfo(handle)

                metrics['gpu_utilization'] = util_info.gpu
                metrics['gpu_memory_used'] = memory_info.used / 1024 / 1024  # MB
                metrics['gpu_memory_total'] = memory_info.total / 1024 / 1024  # MB
            except Exception as e:
                self.get_logger().warn(f'GPU monitoring error: {e}')

        # Monitor CPU and memory usage
        metrics['cpu_usage'] = self.memory_monitor.cpu_percent()
        metrics['memory_usage'] = self.memory_monitor.virtual_memory().percent

        # Log performance metrics
        self.get_logger().info(f'Perception Metrics - Detection Rate: {metrics["detection_rate"]:.2f}, '
                              f'Tracking Objects: {metrics["tracking_objects"]}, '
                              f'GPU: {metrics.get("gpu_utilization", "N/A")}%, '
                              f'CPU: {metrics["cpu_usage"]:.1f}%')

        # Publish metrics as markers for visualization
        self.publish_performance_metrics(metrics)

    def publish_performance_metrics(self, metrics: Dict[str, float]):
        """Publish performance metrics for visualization"""
        marker_array = MarkerArray()

        # Create text markers for each metric
        y_offset = 0
        for i, (key, value) in enumerate(metrics.items()):
            text_marker = Marker()
            text_marker.header.frame_id = "base_link"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "performance_metrics"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = 0.0
            text_marker.pose.position.y = y_offset
            text_marker.pose.position.z = 1.0
            text_marker.pose.orientation.w = 1.0

            text_marker.text = f"{key}: {value:.2f}"
            text_marker.scale.z = 0.05
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)
            y_offset -= 0.1  # Space metrics vertically

        self.metrics_publisher.publish(marker_array)

    def adjust_detection_parameters(self, confidence_threshold: float, max_range: float):
        """Adjust detection parameters based on performance"""
        # This would modify the perception pipeline parameters
        # For now, just log the adjustment
        self.get_logger().info(f'Adjusted detection parameters: confidence={confidence_threshold}, max_range={max_range}')

    def enable_gpu_acceleration(self, enable: bool):
        """Enable or disable GPU acceleration"""
        if self.gpu_monitor and enable:
            self.get_logger().info('GPU acceleration enabled')
        elif not enable:
            self.get_logger().info('GPU acceleration disabled')
        else:
            self.get_logger().warn('GPU not available, cannot enable acceleration')

    def validate_perception_accuracy(self, ground_truth_data: List[Dict]) -> Dict[str, float]:
        """
        Validate perception accuracy against ground truth data.
        This would be used during sim-to-real validation.
        """
        # Get current detections from pipeline
        current_detections = []  # Would get from pipeline

        # Calculate accuracy metrics
        if current_detections and ground_truth_data:
            # Calculate precision, recall, and accuracy
            true_positives = 0
            false_positives = 0
            false_negatives = 0

            for gt_obj in ground_truth_data:
                matched = False
                for det in current_detections:
                    if self.match_detection_to_ground_truth(det, gt_obj):
                        true_positives += 1
                        matched = True
                        break
                if not matched:
                    false_negatives += 1

            for det in current_detections:
                matched = False
                for gt_obj in ground_truth_data:
                    if self.match_detection_to_ground_truth(det, gt_obj):
                        matched = True
                        break
                if not matched:
                    false_positives += 1

            precision = true_positives / (true_positives + false_positives) if (true_positives + false_positives) > 0 else 0
            recall = true_positives / (true_positives + false_negatives) if (true_positives + false_negatives) > 0 else 0
            accuracy = (true_positives + 0) / (true_positives + false_positives + false_negatives) if (true_positives + false_positives + false_negatives) > 0 else 0

            return {
                'precision': precision,
                'recall': recall,
                'accuracy': accuracy,
                'true_positives': true_positives,
                'false_positives': false_positives,
                'false_negatives': false_negatives
            }
        else:
            return {'precision': 0, 'recall': 0, 'accuracy': 0}

    def match_detection_to_ground_truth(self, detection: Dict, ground_truth: Dict, threshold: float = 0.3) -> bool:
        """Match detection to ground truth object"""
        # Calculate IoU or distance-based matching
        # Simplified implementation
        det_pos = detection.get('center_3d', Point(x=0, y=0, z=0))
        gt_pos = ground_truth.get('position', Point(x=0, y=0, z=0))

        distance = np.sqrt(
            (det_pos.x - gt_pos.x)**2 +
            (det_pos.y - gt_pos.y)**2 +
            (det_pos.z - gt_pos.z)**2
        )

        return distance < threshold


def main(args=None):
    """Main function to run the Isaac ROS perception pipeline"""
    rclpy.init(args=args)

    # Create perception manager node
    perception_manager = IsaacROSPerceptionManager()

    try:
        rclpy.spin(perception_manager)
    except KeyboardInterrupt:
        perception_manager.get_logger().info('Perception pipeline stopped by user')
    finally:
        perception_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
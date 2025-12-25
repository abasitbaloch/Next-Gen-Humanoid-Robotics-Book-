#!/usr/bin/env python3
"""
Basic Navigation Controller for Humanoid Robot Simulation

This node implements a simple navigation controller for humanoid robots in simulation.
It demonstrates basic path following and obstacle avoidance behaviors.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import numpy as np
from typing import List, Tuple, Optional


class BasicNavigationController(Node):
    """
    A simple navigation controller for humanoid robots in simulation.
    Implements basic path following and obstacle avoidance.
    """

    def __init__(self):
        super().__init__('basic_navigation_controller')

        # Declare parameters
        self.declare_parameter('linear_velocity', 0.3)
        self.declare_parameter('angular_velocity', 0.5)
        self.declare_parameter('min_obstacle_distance', 0.5)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('heading_tolerance', 0.2)
        self.declare_parameter('control_frequency', 10.0)

        # Get parameters
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.control_frequency = self.get_parameter('control_frequency').value

        # Robot state
        self.current_pose = None
        self.current_twist = None
        self.laser_data = None
        self.current_goal = None
        self.navigation_active = False
        self.avoiding_obstacle = False

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)

        # Navigation status publisher
        self.nav_status_pub = self.create_publisher(Bool, 'navigation_active', 10)

        # Timer for control loop
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)

        self.get_logger().info('Basic Navigation Controller initialized')

    def odom_callback(self, msg: Odometry):
        """Callback for odometry data"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def scan_callback(self, msg: LaserScan):
        """Callback for laser scan data"""
        self.laser_data = msg

    def goal_callback(self, msg: PoseStamped):
        """Callback for goal pose"""
        self.current_goal = msg.pose
        self.navigation_active = True
        self.avoiding_obstacle = False
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def control_loop(self):
        """Main control loop"""
        if not self.navigation_active:
            return

        if self.current_pose is None or self.current_goal is None:
            return

        if self.laser_data is None:
            # If no laser data, stop the robot
            self.stop_robot()
            return

        # Check for obstacles
        obstacle_detected = self.check_for_obstacles()

        if obstacle_detected:
            self.avoiding_obstacle = True
            cmd_vel = self.obstacle_avoidance_behavior()
        else:
            self.avoiding_obstacle = False
            cmd_vel = self.navigate_to_goal()

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        # Check if goal reached
        if self.is_goal_reached():
            self.navigation_active = False
            self.get_logger().info('Goal reached!')

        # Publish navigation status
        status_msg = Bool()
        status_msg.data = self.navigation_active
        self.nav_status_pub.publish(status_msg)

    def check_for_obstacles(self) -> bool:
        """Check if obstacles are within minimum distance"""
        if self.laser_data is None:
            return False

        # Get valid range readings
        valid_ranges = [r for r in self.laser_data.ranges
                       if not (math.isnan(r) or math.isinf(r))]

        if not valid_ranges:
            return False

        # Check if any range is below minimum distance
        min_range = min(valid_ranges) if valid_ranges else float('inf')
        return min_range < self.min_obstacle_distance

    def obstacle_avoidance_behavior(self) -> Twist:
        """Simple obstacle avoidance behavior"""
        cmd_vel = Twist()

        # Turn away from obstacles
        if self.laser_data:
            # Analyze laser data to find clear direction
            ranges = self.laser_data.ranges
            angles = np.linspace(
                self.laser_data.angle_min,
                self.laser_data.angle_max,
                len(ranges)
            )

            # Find the direction with maximum clearance
            max_clearance_idx = np.argmax(ranges)
            max_clearance_angle = angles[max_clearance_idx]

            # Simple proportional control for turning
            cmd_vel.angular.z = np.clip(max_clearance_angle * 0.5, -self.angular_velocity, self.angular_velocity)
            cmd_vel.linear.x = self.linear_velocity * 0.3  # Move slowly while avoiding

        return cmd_vel

    def navigate_to_goal(self) -> Twist:
        """Navigate towards the goal using simple proportional control"""
        cmd_vel = Twist()

        if self.current_pose is None or self.current_goal is None:
            return cmd_vel

        # Calculate desired heading
        dx = self.current_goal.position.x - self.current_pose.position.x
        dy = self.current_goal.position.y - self.current_pose.position.y
        desired_heading = math.atan2(dy, dx)

        # Get current heading from orientation
        current_heading = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # Calculate heading error
        heading_error = self.normalize_angle(desired_heading - current_heading)

        # Simple proportional control for rotation
        cmd_vel.angular.z = np.clip(heading_error * 2.0, -self.angular_velocity, self.angular_velocity)

        # Move forward if roughly aligned with goal
        if abs(heading_error) < self.heading_tolerance:
            cmd_vel.linear.x = self.linear_velocity
        else:
            cmd_vel.linear.x = 0.0  # Wait until properly oriented

        return cmd_vel

    def get_yaw_from_quaternion(self, quaternion) -> float:
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def is_goal_reached(self) -> bool:
        """Check if robot has reached the goal"""
        if self.current_pose is None or self.current_goal is None:
            return False

        # Calculate distance to goal
        dx = self.current_goal.position.x - self.current_pose.position.x
        dy = self.current_goal.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        return distance < self.goal_tolerance

    def stop_robot(self):
        """Stop the robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)


class WaypointNavigator(Node):
    """
    Navigate through a sequence of waypoints.
    """

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('waypoints', [])
        self.declare_parameter('waypoint_tolerance', 0.5)

        # Current navigation state
        self.waypoints = []
        self.current_waypoint_index = 0
        self.navigation_active = False
        self.all_waypoints_reached = False

        # Subscriptions and publications
        self.goal_reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_callback, 10)
        self.nav_status_sub = self.create_subscription(Bool, 'navigation_active', self.nav_status_callback, 10)

        self.next_waypoint_pub = self.create_publisher(PoseStamped, 'next_waypoint', 10)
        self.all_goals_reached_pub = self.create_publisher(Bool, 'all_goals_reached', 10)

        # Timer for waypoint management
        self.waypoint_timer = self.create_timer(1.0, self.waypoint_management_loop)

        self.get_logger().info('Waypoint Navigator initialized')

    def load_waypoints_from_params(self):
        """Load waypoints from parameters"""
        waypoints_param = self.get_parameter('waypoints').value
        if waypoints_param:
            self.waypoints = []
            for wp_data in waypoints_param:
                pose = PoseStamped()
                pose.pose.position.x = wp_data.get('x', 0.0)
                pose.pose.position.y = wp_data.get('y', 0.0)
                pose.pose.position.z = wp_data.get('z', 0.0)
                pose.pose.orientation.w = 1.0  # Default orientation
                self.waypoints.append(pose.pose)

            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')

    def goal_reached_callback(self, msg: Bool):
        """Callback when a goal is reached"""
        if msg.data and self.navigation_active and not self.all_waypoints_reached:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached')

            # Move to next waypoint
            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.waypoints):
                # All waypoints reached
                self.all_waypoints_reached = True
                self.navigation_active = False

                # Publish completion message
                completion_msg = Bool()
                completion_msg.data = True
                self.all_goals_reached_pub.publish(completion_msg)

                self.get_logger().info('All waypoints reached successfully!')
            else:
                # Publish next waypoint
                self.publish_next_waypoint()

    def nav_status_callback(self, msg: Bool):
        """Update navigation status"""
        # This could be used to coordinate with the basic controller
        pass

    def waypoint_management_loop(self):
        """Manage waypoint sequence"""
        if not self.navigation_active and not self.all_waypoints_reached and len(self.waypoints) > 0:
            # Start navigation to first waypoint
            self.navigation_active = True
            self.current_waypoint_index = 0
            self.all_waypoints_reached = False
            self.publish_next_waypoint()

    def publish_next_waypoint(self):
        """Publish the next waypoint as a goal"""
        if self.current_waypoint_index < len(self.waypoints):
            goal_msg = PoseStamped()
            goal_msg.pose = self.waypoints[self.current_waypoint_index]
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'

            # Publish goal to navigation controller
            # In a real system, this would publish to /goal_pose or similar topic
            self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index}: '
                                 f'({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})')


def main(args=None):
    """Main function to run the navigation controller"""
    rclpy.init(args=args)

    # Create navigation controller
    navigation_controller = BasicNavigationController()

    # Optionally create waypoint navigator if needed
    # waypoint_navigator = WaypointNavigator()

    try:
        rclpy.spin(navigation_controller)
    except KeyboardInterrupt:
        navigation_controller.get_logger().info('Navigation controller stopped by user')
    finally:
        # Stop the robot before shutting down
        navigation_controller.stop_robot()
        navigation_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
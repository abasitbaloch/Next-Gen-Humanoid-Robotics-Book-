#!/usr/bin/env python3
"""
Basic Navigation Controller for Humanoid Robot Simulation

This node implements a simple navigation controller for humanoid robots in simulation.
It demonstrates basic path following and obstacle avoidance behaviors.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
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
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Navigation status publisher
        self.nav_status_pub = self.create_publisher(Bool, '/navigation_active', 10)

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
            # If navigation is not active, stop the robot
            if self.avoiding_obstacle:
                self.stop_robot()
                self.avoiding_obstacle = False
            return

        if self.current_pose is None or self.current_goal is None:
            self.stop_robot()
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
            self.stop_robot()
            self.get_logger().info('Goal reached!')

        # Publish navigation status
        status_msg = Bool()
        status_msg.data = self.navigation_active
        self.nav_status_pub.publish(status_msg)

    def check_for_obstacles(self) -> bool:
        """Check if obstacles are within minimum distance"""
        if self.laser_data is None:
            return False

        # Get valid range readings (ignore infinities and NaNs)
        valid_ranges = [r for r in self.laser_data.ranges
                       if not (math.isnan(r) or math.isinf(r)) and r < self.laser_data.range_max]

        if not valid_ranges:
            return False

        # Check if any range is below minimum distance
        min_range = min(valid_ranges) if valid_ranges else float('inf')
        return min_range < self.min_obstacle_distance

    def obstacle_avoidance_behavior(self) -> Twist:
        """Simple obstacle avoidance behavior"""
        cmd_vel = Twist()

        if self.laser_data:
            # Get ranges in front of the robot (±30 degrees)
            front_start = int(len(self.laser_data.ranges) / 2 - len(self.laser_data.ranges) * 30 / 360)
            front_end = int(len(self.laser_data.ranges) / 2 + len(self.laser_data.ranges) * 30 / 360)

            if front_start < 0:
                front_start = 0
            if front_end > len(self.laser_data.ranges):
                front_end = len(self.laser_data.ranges)

            front_ranges = self.laser_data.ranges[front_start:front_end]
            front_valid = [r for r in front_ranges if not (math.isnan(r) or math.isinf(r)) and r < self.laser_data.range_max]

            if front_valid and min(front_valid) < self.min_obstacle_distance:
                # Obstacle detected in front, turn away
                left_ranges = self.laser_data.ranges[:len(self.laser_data.ranges)//2]
                right_ranges = self.laser_data.ranges[len(self.laser_data.ranges)//2:]

                left_avg = np.mean([r for r in left_ranges if not (math.isnan(r) or math.isinf(r)) and r < self.laser_data.range_max])
                right_avg = np.mean([r for r in right_ranges if not (math.isnan(r) or math.isinf(r)) and r < self.laser_data.range_max])

                # Turn toward the clearer side
                if left_avg > right_avg:
                    cmd_vel.angular.z = self.angular_velocity  # Turn left
                else:
                    cmd_vel.angular.z = -self.angular_velocity  # Turn right

                cmd_vel.linear.x = 0.0  # Stop forward motion while turning
            else:
                # No immediate obstacle, continue toward goal
                cmd_vel = self.navigate_to_goal()

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

        # Get current heading from orientation (assuming z-axis rotation)
        current_heading = self.get_yaw_from_quaternion(self.current_pose.orientation)

        # Calculate heading error
        heading_error = self.normalize_angle(desired_heading - current_heading)

        # Simple proportional control for rotation
        cmd_vel.angular.z = max(min(heading_error * 2.0, self.angular_velocity), -self.angular_velocity)

        # Move forward if roughly aligned with goal
        if abs(heading_error) < self.heading_tolerance:
            cmd_vel.linear.x = self.linear_velocity
        else:
            cmd_vel.linear.x = 0.0  # Wait until properly oriented

        return cmd_vel

    def get_yaw_from_quaternion(self, quaternion) -> float:
        """Extract yaw angle from quaternion"""
        # Simplified calculation for z-axis rotation
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


def main(args=None):
    """Main function to run the navigation controller"""
    rclpy.init(args=args)

    # Create navigation controller
    navigation_controller = BasicNavigationController()

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
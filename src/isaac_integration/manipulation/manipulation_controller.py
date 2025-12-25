#!/usr/bin/env python3

"""
Manipulation Control Node for Humanoid Robot

This node handles manipulation tasks for the humanoid robot,
including arm control, grasping, and object manipulation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import math
from builtin_interfaces.msg import Duration


class ManipulationController(Node):
    def __init__(self):
        super().__init__('manipulation_controller')

        # Declare parameters
        self.declare_parameter('robot_description', '')
        self.declare_parameter('arm_joints', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('gripper_joints', ['gripper_joint'])

        # Get parameters
        self.arm_joints = self.get_parameter('arm_joints').value
        self.gripper_joints = self.get_parameter('gripper_joints').value

        # Publishers
        self.arm_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.gripper_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.manipulation_cmd_sub = self.create_subscription(
            String,
            '/manipulation_command',
            self.manipulation_command_callback,
            10
        )

        # Current joint states
        self.current_joint_states = JointState()

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Manipulation Controller initialized')

    def joint_state_callback(self, msg):
        """Callback to receive joint states"""
        self.current_joint_states = msg

    def manipulation_command_callback(self, msg):
        """Callback to handle manipulation commands"""
        command = msg.data
        self.get_logger().info(f'Received manipulation command: {command}')

        if command == 'grasp':
            self.execute_grasp()
        elif command == 'release':
            self.execute_release()
        elif command == 'pick':
            self.execute_pick()
        elif command == 'place':
            self.execute_place()
        elif command.startswith('move_to:'):
            # Extract target position from command like "move_to:x,y,z"
            try:
                coords = command.split(':')[1].split(',')
                target_x, target_y, target_z = float(coords[0]), float(coords[1]), float(coords[2])
                self.move_arm_to_position(target_x, target_y, target_z)
            except:
                self.get_logger().error(f'Invalid move_to command format: {command}')
        else:
            self.get_logger().warn(f'Unknown manipulation command: {command}')

    def execute_grasp(self):
        """Execute grasping motion"""
        self.get_logger().info('Executing grasp motion')

        # Create trajectory for gripper to close
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joints

        point = JointTrajectoryPoint()
        point.positions = [0.8]  # Close gripper
        point.time_from_start = Duration(sec=1, nanosec=0)

        trajectory.points.append(point)

        self.gripper_trajectory_pub.publish(trajectory)

    def execute_release(self):
        """Execute release motion"""
        self.get_logger().info('Executing release motion')

        # Create trajectory for gripper to open
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joints

        point = JointTrajectoryPoint()
        point.positions = [0.0]  # Open gripper
        point.time_from_start = Duration(sec=1, nanosec=0)

        trajectory.points.append(point)

        self.gripper_trajectory_pub.publish(trajectory)

    def execute_pick(self):
        """Execute pick motion - approach, grasp, lift"""
        self.get_logger().info('Executing pick motion')

        # This would be a more complex sequence in a real implementation
        # For now, just execute grasp
        self.execute_grasp()

    def execute_place(self):
        """Execute place motion - approach, release, lift"""
        self.get_logger().info('Executing place motion')

        # This would be a more complex sequence in a real implementation
        # For now, just execute release
        self.execute_release()

    def move_arm_to_position(self, x, y, z):
        """Move arm to specified position (simplified inverse kinematics)"""
        self.get_logger().info(f'Moving arm to position: ({x}, {y}, {z})')

        # This is a simplified approach - in a real implementation,
        # you would use a proper inverse kinematics solver
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joints

        point = JointTrajectoryPoint()

        # For demonstration, use simple joint angles based on position
        # In reality, you'd use an IK solver
        joint_angles = self.calculate_joint_angles(x, y, z)
        point.positions = joint_angles
        point.time_from_start = Duration(sec=2, nanosec=0)

        trajectory.points.append(point)

        self.arm_trajectory_pub.publish(trajectory)

    def calculate_joint_angles(self, x, y, z):
        """Calculate joint angles for target position (simplified)"""
        # This is a very simplified IK calculation for demonstration
        # In a real implementation, you would use a proper IK library

        # Assuming a simple 6-DOF arm
        # Calculate angles based on position (this is not a real IK solution)
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        theta2 = math.atan2(z, r)
        theta3 = 0.0  # Simplified
        theta4 = 0.0  # Simplified
        theta5 = 0.0  # Simplified
        theta6 = 0.0  # Simplified

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def control_loop(self):
        """Main control loop"""
        # This would contain the main control logic
        # For now, just log the current joint states
        if len(self.current_joint_states.name) > 0:
            self.get_logger().debug(f'Current joint states: {self.current_joint_states.name}')


def main(args=None):
    rclpy.init(args=args)

    manipulation_controller = ManipulationController()

    try:
        rclpy.spin(manipulation_controller)
    except KeyboardInterrupt:
        pass
    finally:
        manipulation_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
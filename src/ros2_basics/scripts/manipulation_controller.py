#!/usr/bin/env python3
"""
Manipulation Controller for Humanoid Robot Simulation

This node implements basic manipulation capabilities for humanoid robots in simulation.
It demonstrates pick-and-place operations, grasping, and object interaction.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveGroupActionResult, PickupActionResult, PlaceActionResult
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import math
import numpy as np
from typing import List, Tuple, Optional
import time


class ManipulationController(Node):
    """
    A manipulation controller for humanoid robots in simulation.
    Implements basic pick-and-place operations and grasping behaviors.
    """

    def __init__(self):
        super().__init__('manipulation_controller')

        # Declare parameters
        self.declare_parameter('planning_group', 'left_arm')
        self.declare_parameter('end_effector_link', 'left_hand')
        self.declare_parameter('gripper_joint_names', ['left_gripper_joint'])
        self.declare_parameter('gripper_open_position', 0.05)
        self.declare_parameter('gripper_close_position', 0.01)
        self.declare_parameter('approach_distance', 0.1)
        self.declare_parameter('lift_distance', 0.1)
        self.declare_parameter('control_frequency', 50.0)

        # Get parameters
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.gripper_joints = self.get_parameter('gripper_joint_names').value
        self.gripper_open_pos = self.get_parameter('gripper_open_position').value
        self.gripper_close_pos = self.get_parameter('gripper_close_position').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.lift_distance = self.get_parameter('lift_distance').value
        self.control_frequency = self.get_parameter('control_frequency').value

        # Robot state
        self.joint_states = None
        self.object_positions = {}
        self.manipulation_active = False
        self.current_task = None

        # Publishers and subscribers
        self.joint_traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.gripper_cmd_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.object_sub = self.create_subscription(String, '/detected_objects', self.object_callback, 10)

        # Manipulation command subscriber
        self.manip_cmd_sub = self.create_subscription(String, '/manipulation_command', self.manipulation_command_callback, 10)

        # Task status publisher
        self.task_status_pub = self.create_publisher(String, '/manipulation_status', 10)

        # Timer for control loop
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)

        self.get_logger().info('Manipulation Controller initialized')

    def joint_state_callback(self, msg: JointState):
        """Callback for joint state updates"""
        self.joint_states = msg

    def object_callback(self, msg: String):
        """Callback for detected objects"""
        try:
            # Parse object information from string message
            # Format: "object_name:x,y,z,roll,pitch,yaw"
            parts = msg.data.split(':')
            if len(parts) == 2:
                obj_name = parts[0]
                pose_str = parts[1]
                coords = [float(x.strip()) for x in pose_str.split(',')]

                if len(coords) == 6:  # x, y, z, roll, pitch, yaw
                    self.object_positions[obj_name] = {
                        'position': Point(x=coords[0], y=coords[1], z=coords[2]),
                        'orientation': self.euler_to_quaternion(coords[3], coords[4], coords[5])
                    }
        except ValueError:
            self.get_logger().warn(f'Could not parse object data: {msg.data}')

    def manipulation_command_callback(self, msg: String):
        """Callback for manipulation commands"""
        command = msg.data.lower().strip()

        if command.startswith('pick '):
            obj_name = command.split(' ', 1)[1]
            self.start_pick_operation(obj_name)
        elif command.startswith('place '):
            obj_name = command.split(' ', 1)[1]
            destination = self.parse_destination(command)
            self.start_place_operation(obj_name, destination)
        elif command == 'open_gripper':
            self.open_gripper()
        elif command == 'close_gripper':
            self.close_gripper()
        elif command == 'home_position':
            self.move_to_home_position()
        else:
            self.get_logger().warn(f'Unknown manipulation command: {command}')

    def start_pick_operation(self, object_name: str):
        """Start a pick operation for the specified object"""
        if object_name not in self.object_positions:
            self.get_logger().error(f'Object {object_name} not found in detected objects')
            return

        self.manipulation_active = True
        self.current_task = 'pick'
        self.current_object = object_name
        self.current_object_pose = self.object_positions[object_name]

        self.get_logger().info(f'Starting pick operation for {object_name}')

        # Execute pick sequence
        future = rclpy.task.Future()
        self.execute_pick_sequence(future)

    def start_place_operation(self, object_name: str, destination: Point):
        """Start a place operation for the specified object"""
        self.manipulation_active = True
        self.current_task = 'place'
        self.current_object = object_name
        self.destination_pose = destination

        self.get_logger().info(f'Starting place operation for {object_name} at {destination}')

        # Execute place sequence
        future = rclpy.task.Future()
        self.execute_place_sequence(future)

    def execute_pick_sequence(self, future):
        """Execute the complete pick sequence"""
        try:
            # 1. Move to approach position (above object)
            approach_pose = self.calculate_approach_pose(self.current_object_pose)
            self.move_to_pose(approach_pose)

            # 2. Lower to grasp position
            grasp_pose = self.current_object_pose.copy()
            grasp_pose.position.z += 0.02  # Slightly above object to account for gripper approach
            self.move_to_pose(grasp_pose)

            # 3. Close gripper
            self.close_gripper()

            # 4. Lift object
            lift_pose = self.current_object_pose.copy()
            lift_pose.position.z += self.lift_distance
            self.move_to_pose(lift_pose)

            # 5. Move away from object location
            retreat_pose = self.calculate_retreat_pose(self.current_object_pose)
            self.move_to_pose(retreat_pose)

            self.get_logger().info(f'Pick operation completed for {self.current_object}')
            self.publish_task_status('pick_completed')

        except Exception as e:
            self.get_logger().error(f'Pick operation failed: {e}')
            self.publish_task_status('pick_failed')

        finally:
            self.manipulation_active = False
            self.current_task = None

    def execute_place_sequence(self, future):
        """Execute the complete place sequence"""
        try:
            # 1. Move to approach position above destination
            approach_pose = self.calculate_approach_pose_at_destination(self.destination_pose)
            self.move_to_pose(approach_pose)

            # 2. Lower to place position
            place_pose = self.destination_pose.copy()
            place_pose.position.z += 0.02  # Slightly above destination
            self.move_to_pose(place_pose)

            # 3. Open gripper
            self.open_gripper()

            # 4. Lift gripper
            lift_pose = self.destination_pose.copy()
            lift_pose.position.z += self.lift_distance
            self.move_to_pose(lift_pose)

            # 5. Move away from placement location
            retreat_pose = self.calculate_retreat_pose_at_destination(self.destination_pose)
            self.move_to_pose(retreat_pose)

            self.get_logger().info(f'Place operation completed for {self.current_object}')
            self.publish_task_status('place_completed')

        except Exception as e:
            self.get_logger().error(f'Place operation failed: {e}')
            self.publish_task_status('place_failed')

        finally:
            self.manipulation_active = False
            self.current_task = None

    def calculate_approach_pose(self, object_pose) -> Pose:
        """Calculate approach pose above the object"""
        approach_pose = Pose()
        approach_pose.position = Point(
            x=object_pose.position.x,
            y=object_pose.position.y,
            z=object_pose.position.z + self.approach_distance
        )
        approach_pose.orientation = object_pose.orientation
        return approach_pose

    def calculate_retreat_pose(self, object_pose) -> Pose:
        """Calculate retreat pose to move away from object location"""
        retreat_pose = Pose()
        retreat_pose.position = Point(
            x=object_pose.position.x,
            y=object_pose.position.y,
            z=object_pose.position.z + self.approach_distance + self.lift_distance
        )
        retreat_pose.orientation = object_pose.orientation
        return retreat_pose

    def calculate_approach_pose_at_destination(self, destination_pose) -> Pose:
        """Calculate approach pose above the destination"""
        approach_pose = Pose()
        approach_pose.position = Point(
            x=destination_pose.x,
            y=destination_pose.y,
            z=destination_pose.z + self.approach_distance
        )
        # Keep current orientation or use default
        approach_pose.orientation = self.get_current_orientation()
        return approach_pose

    def calculate_retreat_pose_at_destination(self, destination_pose) -> Pose:
        """Calculate retreat pose to move away from destination"""
        retreat_pose = Pose()
        retreat_pose.position = Point(
            x=destination_pose.x,
            y=destination_pose.y,
            z=destination_pose.z + self.approach_distance + self.lift_distance
        )
        retreat_pose.orientation = self.get_current_orientation()
        return retreat_pose

    def get_current_orientation(self) -> Quaternion:
        """Get current end-effector orientation"""
        if self.joint_states is None:
            # Return default orientation
            return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

        # Calculate current orientation based on joint angles
        # This would require forward kinematics which is simplified here
        return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

    def move_to_pose(self, target_pose: Pose):
        """Move end-effector to target pose using inverse kinematics"""
        # This would normally call MoveIt2 or a custom IK solver
        # For simulation, we'll use a simplified approach

        self.get_logger().info(f'Moving to pose: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})')

        # In a real implementation, this would:
        # 1. Call inverse kinematics to find joint angles
        # 2. Generate trajectory to reach those angles
        # 3. Execute the trajectory

        # Simulate movement time
        time.sleep(1.0)  # Simulate movement in simulation

    def open_gripper(self):
        """Open the gripper"""
        self.move_gripper_joints(self.gripper_open_pos)
        self.get_logger().info('Opening gripper')
        time.sleep(0.5)  # Simulate gripper movement

    def close_gripper(self):
        """Close the gripper"""
        self.move_gripper_joints(self.gripper_close_pos)
        self.get_logger().info('Closing gripper')
        time.sleep(0.5)  # Simulate gripper movement

    def move_gripper_joints(self, position):
        """Move gripper joints to specified position"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.gripper_joints
        traj_msg.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()
        point.positions = [position] * len(self.gripper_joints)
        point.time_from_start.sec = 1  # 1 second to reach position
        traj_msg.points = [point]

        self.gripper_cmd_pub.publish(traj_msg)

    def move_to_home_position(self):
        """Move manipulator to home position"""
        home_pose = Pose()
        home_pose.position = Point(x=0.3, y=0.0, z=1.0)  # Home position in front of robot
        home_pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

        self.move_to_pose(home_pose)
        self.get_logger().info('Moved to home position')

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def parse_destination(self, command: str) -> Point:
        """Parse destination from command string"""
        # Simple parsing - in real implementation this would be more sophisticated
        # Format: "place object_name at x,y,z"
        try:
            coords_part = command.split('at')[1].strip()
            coords = [float(x.strip()) for x in coords_part.split(',')]
            if len(coords) == 3:
                return Point(x=coords[0], y=coords[1], z=coords[2])
        except:
            # Default destination if parsing fails
            return Point(x=0.5, y=0.5, z=0.8)

        return Point(x=0.5, y=0.5, z=0.8)  # Default destination

    def control_loop(self):
        """Main control loop"""
        if not self.manipulation_active:
            # Publish current status
            self.publish_task_status('idle')
        else:
            # Publish active task status
            if self.current_task:
                self.publish_task_status(f'{self.current_task}_in_progress')

    def publish_task_status(self, status: str):
        """Publish task status"""
        status_msg = String()
        status_msg.data = status
        self.task_status_pub.publish(status_msg)


class SimpleManipulationDemo(Node):
    """
    A simple manipulation demo that demonstrates basic manipulation behaviors.
    """

    def __init__(self):
        super().__init__('simple_manipulation_demo')

        # Publishers for demonstration
        self.manip_cmd_pub = self.create_publisher(String, '/manipulation_command', 10)
        self.demo_status_pub = self.create_publisher(String, '/demo_status', 10)

        # Timer for demo sequence
        self.demo_timer = self.create_timer(5.0, self.demo_sequence)
        self.demo_step = 0

        self.get_logger().info('Simple Manipulation Demo initialized')

    def demo_sequence(self):
        """Execute demonstration sequence"""
        demo_commands = [
            'open_gripper',
            'pick block_1',
            'place block_1 at 0.6,0.3,0.8',
            'home_position',
            'open_gripper',
            'pick cylinder_1',
            'place cylinder_1 at 0.7,-0.2,0.8',
            'home_position'
        ]

        if self.demo_step < len(demo_commands):
            command = demo_commands[self.demo_step]
            cmd_msg = String()
            cmd_msg.data = command

            self.manip_cmd_pub.publish(cmd_msg)
            self.get_logger().info(f'Executing demo command: {command}')

            status_msg = String()
            status_msg.data = f'Demo step {self.demo_step + 1}: {command}'
            self.demo_status_pub.publish(status_msg)

            self.demo_step += 1
        else:
            # Reset demo
            self.demo_step = 0
            status_msg = String()
            status_msg.data = 'Demo completed - restarting'
            self.demo_status_pub.publish(status_msg)


def main(args=None):
    """Main function to run the manipulation controller"""
    rclpy.init(args=args)

    # Create manipulation controller
    manipulation_controller = ManipulationController()

    # Optionally create demo node
    try:
        manipulation_demo = SimpleManipulationDemo()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(manipulation_controller)
        executor.add_node(manipulation_demo)

        try:
            executor.spin()
        except KeyboardInterrupt:
            manipulation_controller.get_logger().info('Manipulation controller stopped by user')
        finally:
            manipulation_controller.destroy_node()
            manipulation_demo.destroy_node()
            rclpy.shutdown()
    except:
        try:
            rclpy.spin(manipulation_controller)
        except KeyboardInterrupt:
            manipulation_controller.get_logger().info('Manipulation controller stopped by user')
        finally:
            manipulation_controller.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
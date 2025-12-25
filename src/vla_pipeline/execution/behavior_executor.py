#!/usr/bin/env python3

"""
Behavior Executor for Vision-Language-Action Pipeline

This node executes autonomous behaviors based on planned tasks.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import json
import time
import threading
from typing import Dict, Any, Optional
from enum import Enum


class BehaviorState(Enum):
    IDLE = "idle"
    EXECUTING = "executing"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"


class BehaviorExecutor(Node):
    def __init__(self):
        super().__init__('behavior_executor')

        # Publishers
        self.behavior_status_pub = self.create_publisher(String, '/behavior_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.subtask_queue_sub = self.create_subscription(
            String,
            '/subtask_queue',
            self.subtask_queue_callback,
            10
        )

        self.perception_result_sub = self.create_subscription(
            String,
            '/perception/result',
            self.perception_result_callback,
            10
        )

        # Initialize
        self.current_behavior = None
        self.behavior_state = BehaviorState.IDLE
        self.perception_data = {}
        self.execution_queue = []
        self.execution_lock = threading.Lock()

        # Timer for behavior execution
        self.execution_timer = self.create_timer(0.1, self.execute_behavior)

        self.get_logger().info('Behavior Executor initialized')

    def subtask_queue_callback(self, msg):
        """Callback for incoming subtasks"""
        try:
            subtask = json.loads(msg.data)
            self.get_logger().info(f'Queued subtask: {subtask["id"]}')

            with self.execution_lock:
                self.execution_queue.append(subtask)

            # Update behavior status
            self.publish_behavior_status({
                'state': self.behavior_state.value,
                'queued_tasks': len(self.execution_queue),
                'current_task': self.current_behavior['id'] if self.current_behavior else None,
                'timestamp': time.time()
            })

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in subtask: {msg.data}')

    def perception_result_callback(self, msg):
        """Callback for perception results"""
        try:
            perception_data = json.loads(msg.data)
            self.perception_data.update(perception_data)
            self.get_logger().debug(f'Updated perception data: {list(perception_data.keys())}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in perception result: {msg.data}')

    def execute_behavior(self):
        """Main behavior execution loop"""
        if self.behavior_state != BehaviorState.EXECUTING and len(self.execution_queue) > 0:
            # Start executing the next task if available
            self.start_next_behavior()

        if self.current_behavior and self.behavior_state == BehaviorState.EXECUTING:
            # Execute the current behavior step
            self.execute_current_behavior()

    def start_next_behavior(self):
        """Start executing the next behavior from the queue"""
        with self.execution_lock:
            if len(self.execution_queue) > 0:
                self.current_behavior = self.execution_queue.pop(0)
                self.behavior_state = BehaviorState.EXECUTING

                self.get_logger().info(f'Starting behavior: {self.current_behavior["id"]}')

                # Publish status update
                self.publish_behavior_status({
                    'state': self.behavior_state.value,
                    'current_task': self.current_behavior['id'],
                    'action': self.current_behavior['action'],
                    'queued_tasks': len(self.execution_queue),
                    'timestamp': time.time()
                })

    def execute_current_behavior(self):
        """Execute the current behavior based on its action type"""
        if not self.current_behavior:
            return

        action = self.current_behavior['action']
        params = self.current_behavior.get('parameters', {})

        self.get_logger().debug(f'Executing behavior: {self.current_behavior["id"]} - {action}')

        success = False

        if action == 'navigate':
            success = self.execute_navigation(params)
        elif action == 'grasp':
            success = self.execute_grasp(params)
        elif action == 'place':
            success = self.execute_place(params)
        elif action == 'find_object':
            success = self.execute_find_object(params)
        elif action == 'look_at':
            success = self.execute_look_at(params)
        elif action == 'stop':
            success = self.execute_stop(params)
        elif action == 'move_arm':
            success = self.execute_arm_movement(params)
        else:
            self.get_logger().warn(f'Unknown action: {action}')
            success = True  # Mark as successful to continue

        if success:
            # Mark current behavior as completed
            self.complete_current_behavior()
        # If not successful, we'll try again in the next cycle

    def execute_navigation(self, params: Dict[str, Any]) -> bool:
        """Execute navigation behavior"""
        target = params.get('target', 'unknown')

        # In a real implementation, this would interface with the navigation system
        # For now, we'll simulate navigation by publishing velocity commands
        twist = Twist()

        # Simple navigation simulation - move forward for a bit
        # In reality, this would use the navigation stack
        twist.linear.x = 0.2  # Move forward at 0.2 m/s
        twist.angular.z = 0.0  # No rotation

        self.cmd_vel_pub.publish(twist)

        # Simulate navigation taking some time
        # In a real implementation, we would wait for navigation feedback
        self.get_logger().debug(f'Navigating to: {target}')

        # For simulation, return True after some time to indicate completion
        # In a real implementation, check actual navigation status
        return True

    def execute_grasp(self, params: Dict[str, Any]) -> bool:
        """Execute grasp behavior"""
        object_name = params.get('object_name', 'unknown')

        # In a real implementation, this would interface with the manipulation system
        self.get_logger().info(f'Attempting to grasp: {object_name}')

        # Publish manipulation command
        manip_cmd_pub = self.create_publisher(String, '/manipulation_command', 10)
        cmd_msg = String()
        cmd_msg.data = f'grasp'
        manip_cmd_pub.publish(cmd_msg)

        return True

    def execute_place(self, params: Dict[str, Any]) -> bool:
        """Execute place behavior"""
        location = params.get('location', 'default')

        # In a real implementation, this would interface with the manipulation system
        self.get_logger().info(f'Attempting to place at: {location}')

        # Publish manipulation command
        manip_cmd_pub = self.create_publisher(String, '/manipulation_command', 10)
        cmd_msg = String()
        cmd_msg.data = f'place'
        manip_cmd_pub.publish(cmd_msg)

        return True

    def execute_find_object(self, params: Dict[str, Any]) -> bool:
        """Execute object finding behavior"""
        object_name = params.get('object_name', 'unknown')

        self.get_logger().info(f'Searching for object: {object_name}')

        # Request perception system to find the object
        perception_request_pub = self.create_publisher(String, '/perception/request', 10)
        request_msg = String()
        request_msg.data = json.dumps({
            'request_type': 'find_object',
            'object_name': object_name,
            'timestamp': time.time()
        })
        perception_request_pub.publish(request_msg)

        # For now, assume object is found (in real implementation, wait for perception result)
        return True

    def execute_look_at(self, params: Dict[str, Any]) -> bool:
        """Execute look at behavior"""
        target = params.get('target', 'forward')

        self.get_logger().info(f'Looking at: {target}')

        # In a real implementation, control the head/pan-tilt to look at the target
        return True

    def execute_stop(self, params: Dict[str, Any]) -> bool:
        """Execute stop behavior"""
        self.get_logger().info('Stopping all motion')

        # Stop all movement
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

        return True

    def execute_arm_movement(self, params: Dict[str, Any]) -> bool:
        """Execute arm movement behavior"""
        pose = params.get('pose', {})

        self.get_logger().info(f'Moving arm to pose: {pose}')

        # Publish arm command
        arm_cmd_pub = self.create_publisher(String, '/arm_command', 10)
        cmd_msg = String()
        cmd_msg.data = json.dumps({
            'action': 'move_to_pose',
            'pose': pose
        })
        arm_cmd_pub.publish(cmd_msg)

        return True

    def complete_current_behavior(self):
        """Mark the current behavior as completed"""
        if not self.current_behavior:
            return

        behavior_id = self.current_behavior['id']
        self.get_logger().info(f'Completed behavior: {behavior_id}')

        # Publish completion status
        status_msg = String()
        status_msg.data = json.dumps({
            'subtask_id': behavior_id,
            'task_id': self.current_behavior.get('task_id'),
            'status': 'completed',
            'timestamp': time.time()
        })

        status_pub = self.create_publisher(String, '/subtask_status', 10)
        status_pub.publish(status_msg)

        # Reset current behavior
        self.current_behavior = None
        self.behavior_state = BehaviorState.IDLE

        # Publish updated status
        self.publish_behavior_status({
            'state': self.behavior_state.value,
            'completed_task': behavior_id,
            'queued_tasks': len(self.execution_queue),
            'timestamp': time.time()
        })

    def publish_behavior_status(self, status_data: Dict[str, Any]):
        """Publish behavior execution status"""
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.behavior_status_pub.publish(status_msg)


class HumanRobotInteraction(Node):
    """Node for handling human-robot interaction aspects of VLA"""
    def __init__(self):
        super().__init__('human_robot_interaction')

        # Publishers
        self.speech_output_pub = self.create_publisher(String, '/speech_output', 10)
        self.behavior_request_pub = self.create_publisher(String, '/behavior_request', 10)

        # Subscribers
        self.interaction_event_sub = self.create_subscription(
            String,
            '/interaction_event',
            self.interaction_event_callback,
            10
        )

        self.task_status_sub = self.create_subscription(
            String,
            '/task_status',
            self.task_status_callback,
            10
        )

        self.get_logger().info('Human-Robot Interaction handler initialized')

    def interaction_event_callback(self, msg):
        """Handle interaction events (greetings, attention, etc.)"""
        try:
            event_data = json.loads(msg.data)
            event_type = event_data.get('type')
            source = event_data.get('source', 'unknown')

            self.get_logger().info(f'Interaction event: {event_type} from {source}')

            # Generate appropriate response based on event
            if event_type == 'greeting':
                self.respond_to_greeting(source)
            elif event_type == 'attention':
                self.respond_to_attention(source)
            elif event_type == 'request_help':
                self.respond_to_help_request(source)
            elif event_type == 'task_completed':
                self.acknowledge_task_completion(event_data)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in interaction event: {msg.data}')

    def task_status_callback(self, msg):
        """Handle task status updates for interaction"""
        try:
            status_data = json.loads(msg.data)
            task_id = status_data.get('task_id')
            status = status_data.get('status')

            if status == 'completed':
                self.announce_task_completion(task_id)
            elif status == 'failed':
                self.announce_task_failure(task_id)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in task status: {msg.data}')

    def respond_to_greeting(self, source):
        """Respond to a greeting"""
        responses = [
            "Hello! How can I assist you today?",
            "Greetings! I'm ready to help.",
            "Hi there! What would you like me to do?"
        ]

        import random
        response = random.choice(responses)

        self.speak(response)
        self.get_logger().info(f'Responded to greeting from {source}: {response}')

    def respond_to_attention(self, source):
        """Respond when attention is drawn"""
        response = "Yes, I'm here. How can I help you?"
        self.speak(response)
        self.get_logger().info(f'Responded to attention from {source}')

    def respond_to_help_request(self, source):
        """Respond to a help request"""
        response = "I'm here to assist you. You can ask me to perform tasks like 'move to the kitchen' or 'pick up the red cup'."
        self.speak(response)
        self.get_logger().info(f'Responded to help request from {source}')

    def acknowledge_task_completion(self, event_data):
        """Acknowledge that a task was completed"""
        task_id = event_data.get('task_id', 'unknown')
        response = f"I've completed the task {task_id}. Is there anything else I can help with?"
        self.speak(response)
        self.get_logger().info(f'Acknowledged task completion: {task_id}')

    def announce_task_completion(self, task_id):
        """Announce that a task has been completed"""
        response = f"Task {task_id} has been completed successfully."
        self.speak(response)
        self.get_logger().info(f'Announced task completion: {task_id}')

    def announce_task_failure(self, task_id):
        """Announce that a task has failed"""
        response = f"I'm sorry, but I couldn't complete task {task_id}. Would you like me to try again?"
        self.speak(response)
        self.get_logger().info(f'Announced task failure: {task_id}')

    def speak(self, text):
        """Publish speech output"""
        speech_msg = String()
        speech_msg.data = text
        self.speech_output_pub.publish(speech_msg)


def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    behavior_executor = BehaviorExecutor()
    hri_handler = HumanRobotInteraction()

    # Use MultiThreadedExecutor to run both nodes
    executor = MultiThreadedExecutor()
    executor.add_node(behavior_executor)
    executor.add_node(hri_handler)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        behavior_executor.destroy_node()
        hri_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
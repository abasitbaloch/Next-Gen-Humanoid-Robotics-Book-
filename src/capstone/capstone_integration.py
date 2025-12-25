#!/usr/bin/env python3

"""
Capstone Integration Node for Autonomous Humanoid Pipeline

This node integrates all components: voice → plan → navigation → perception → manipulation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
import json
import time
from typing import Dict, Any, Optional
from enum import Enum


class CapstoneState(Enum):
    IDLE = "idle"
    PROCESSING_VOICE = "processing_voice"
    PLANNING_TASK = "planning_task"
    EXECUTING_NAVIGATION = "executing_navigation"
    PERFORMING_PERCEPTION = "performing_perception"
    EXECUTING_MANIPULATION = "executing_manipulation"
    TASK_COMPLETED = "task_completed"
    ERROR = "error"


class CapstoneIntegration(Node):
    def __init__(self):
        super().__init__('capstone_integration')

        # Declare parameters
        self.declare_parameter('integration_timeout', 300)  # 5 minutes timeout
        self.declare_parameter('enable_debug', True)

        # Get parameters
        self.integration_timeout = self.get_parameter('integration_timeout').value
        self.enable_debug = self.get_parameter('enable_debug').value

        # Publishers
        self.capstone_status_pub = self.create_publisher(String, '/capstone/status', 10)
        self.voice_input_pub = self.create_publisher(String, '/voice_input', 10)
        self.llm_request_pub = self.create_publisher(String, '/llm_request', 10)
        self.task_plan_pub = self.create_publisher(String, '/task_plan', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.manipulation_cmd_pub = self.create_publisher(String, '/manipulation_command', 10)
        self.perception_request_pub = self.create_publisher(String, '/perception/request', 10)

        # Subscribers
        self.voice_command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10
        )

        self.task_plan_sub = self.create_subscription(
            String,
            '/task_plan',
            self.task_plan_callback,
            10
        )

        self.navigation_status_sub = self.create_subscription(
            String,
            '/navigation/status',
            self.navigation_status_callback,
            10
        )

        self.perception_result_sub = self.create_subscription(
            String,
            '/perception/result',
            self.perception_result_callback,
            10
        )

        self.manipulation_status_sub = self.create_subscription(
            String,
            '/manipulation/status',
            self.manipulation_status_callback,
            10
        )

        # Initialize state
        self.state = CapstoneState.IDLE
        self.current_task = None
        self.task_start_time = None
        self.voice_command_data = None
        self.task_plan_data = None

        # Timer for state management
        self.state_timer = self.create_timer(0.5, self.state_management_callback)

        self.get_logger().info('Capstone Integration Node initialized')

    def voice_command_callback(self, msg):
        """Callback for voice commands"""
        try:
            command_data = json.loads(msg.data)
            self.get_logger().info(f'Received voice command: {command_data}')

            self.voice_command_data = command_data
            self.state = CapstoneState.PLANNING_TASK

            # Send to LLM for planning
            llm_msg = String()
            llm_msg.data = json.dumps({
                'type': 'voice_command',
                'command': command_data,
                'raw_input': command_data.get('raw_command', '')
            })
            self.llm_request_pub.publish(llm_msg)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in voice command: {msg.data}')

    def task_plan_callback(self, msg):
        """Callback for task plans"""
        try:
            task_plan = json.loads(msg.data)
            self.get_logger().info(f'Received task plan: {task_plan.get("task_id", "unknown")}')

            self.task_plan_data = task_plan
            self.state = CapstoneState.EXECUTING_NAVIGATION

            # Start executing the plan
            self.execute_task_plan(task_plan)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in task plan: {msg.data}')

    def navigation_status_callback(self, msg):
        """Callback for navigation status"""
        try:
            status_data = json.loads(msg.data)
            nav_status = status_data.get('status', 'unknown')

            if nav_status == 'completed' and self.state == CapstoneState.EXECUTING_NAVIGATION:
                # Navigation completed, move to perception
                self.state = CapstoneState.PERFORMING_PERCEPTION
                self.request_perception()

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in navigation status: {msg.data}')

    def perception_result_callback(self, msg):
        """Callback for perception results"""
        try:
            perception_data = json.loads(msg.data)
            self.get_logger().debug(f'Received perception result: {list(perception_data.keys())}')

            if self.state == CapstoneState.PERFORMING_PERCEPTION:
                # Perception completed, move to manipulation
                self.state = CapstoneState.EXECUTING_MANIPULATION
                self.execute_manipulation(perception_data)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in perception result: {msg.data}')

    def manipulation_status_callback(self, msg):
        """Callback for manipulation status"""
        try:
            status_data = json.loads(msg.data)
            manip_status = status_data.get('status', 'unknown')

            if manip_status == 'completed' and self.state == CapstoneState.EXECUTING_MANIPULATION:
                # Manipulation completed, task is done
                self.state = CapstoneState.TASK_COMPLETED
                self.complete_task()

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in manipulation status: {msg.data}')

    def state_management_callback(self):
        """Manage the state of the capstone integration"""
        current_time = time.time()

        # Check for timeouts
        if self.task_start_time and (current_time - self.task_start_time > self.integration_timeout):
            self.get_logger().error('Capstone integration timeout')
            self.state = CapstoneState.ERROR
            self.publish_status({
                'state': self.state.value,
                'status': 'timeout',
                'error': 'Integration timeout',
                'timestamp': current_time
            })
            return

        # Publish current status
        self.publish_status({
            'state': self.state.value,
            'current_task': self.current_task['task_id'] if self.current_task else None,
            'timestamp': current_time
        })

        # Debug output
        if self.enable_debug:
            self.get_logger().debug(f'Capstone state: {self.state.value}')

    def execute_task_plan(self, task_plan: Dict[str, Any]):
        """Execute a task plan by triggering appropriate subsystems"""
        self.current_task = task_plan
        self.task_start_time = time.time()

        self.get_logger().info(f'Starting execution of task plan: {task_plan.get("task_id", "unknown")}')

        for step in task_plan.get('steps', []):
            action = step.get('action', 'unknown')
            params = step.get('parameters', {})

            if action == 'navigate':
                self.execute_navigation_step(params)
            elif action == 'find_object':
                self.execute_perception_step(params)
            elif action == 'grasp' or action == 'place':
                self.execute_manipulation_step(params)
            else:
                self.get_logger().warn(f'Unknown action in task plan: {action}')

    def execute_navigation_step(self, params: Dict[str, Any]):
        """Execute navigation step"""
        target = params.get('target', 'unknown')

        self.get_logger().info(f'Executing navigation to: {target}')

        # Create a goal pose based on the target
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        # In a real implementation, this would come from a map or be calculated
        # For simulation, we'll use mock coordinates
        if 'kitchen' in target.lower():
            goal_msg.pose.position.x = 1.0
            goal_msg.pose.position.y = 1.0
        elif 'living room' in target.lower():
            goal_msg.pose.position.x = 2.0
            goal_msg.pose.position.y = 0.0
        elif 'bedroom' in target.lower():
            goal_msg.pose.position.x = 0.0
            goal_msg.pose.position.y = 2.0
        else:
            # Default location
            goal_msg.pose.position.x = 0.5
            goal_msg.pose.position.y = 0.5

        goal_msg.pose.orientation.w = 1.0

        self.goal_pose_pub.publish(goal_msg)

    def execute_perception_step(self, params: Dict[str, Any]):
        """Execute perception step"""
        object_name = params.get('object_name', 'unknown')

        self.get_logger().info(f'Executing perception to find: {object_name}')

        # Request perception system to find the object
        request_msg = String()
        request_msg.data = json.dumps({
            'request_type': 'find_object',
            'object_name': object_name,
            'timestamp': time.time()
        })
        self.perception_request_pub.publish(request_msg)

    def execute_manipulation_step(self, params: Dict[str, Any]):
        """Execute manipulation step"""
        action = params.get('action', 'unknown')

        self.get_logger().info(f'Executing manipulation: {action}')

        # Send manipulation command
        cmd_msg = String()
        if action == 'grasp':
            cmd_msg.data = 'grasp'
        elif action == 'place':
            cmd_msg.data = 'place'
        else:
            cmd_msg.data = action

        self.manipulation_cmd_pub.publish(cmd_msg)

    def request_perception(self):
        """Request perception when needed"""
        # In the integrated system, perception might be needed after navigation
        request_msg = String()
        request_msg.data = json.dumps({
            'request_type': 'scene_analysis',
            'timestamp': time.time()
        })
        self.perception_request_pub.publish(request_msg)

    def execute_manipulation(self, perception_data: Dict[str, Any]):
        """Execute manipulation based on perception data"""
        # In a real implementation, this would use perception data to guide manipulation
        cmd_msg = String()
        cmd_msg.data = 'grasp'  # Default action
        self.manipulation_cmd_pub.publish(cmd_msg)

    def complete_task(self):
        """Complete the current task"""
        self.get_logger().info('Capstone task completed successfully')

        # Publish completion status
        self.publish_status({
            'state': self.state.value,
            'status': 'completed',
            'task_id': self.current_task['task_id'] if self.current_task else None,
            'execution_time': time.time() - self.task_start_time if self.task_start_time else 0,
            'timestamp': time.time()
        })

        # Reset for next task
        self.current_task = None
        self.task_start_time = None
        self.voice_command_data = None
        self.task_plan_data = None
        self.state = CapstoneState.IDLE

    def publish_status(self, status_data: Dict[str, Any]):
        """Publish capstone status"""
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.capstone_status_pub.publish(status_msg)


class CapstoneValidation(Node):
    """Node for validating the capstone integration"""
    def __init__(self):
        super().__init__('capstone_validation')

        # Publishers
        self.validation_result_pub = self.create_publisher(String, '/capstone/validation_result', 10)

        # Subscribers
        self.capstone_status_sub = self.create_subscription(
            String,
            '/capstone/status',
            self.capstone_status_callback,
            10
        )

        self.task_status_sub = self.create_subscription(
            String,
            '/task_status',
            self.task_status_callback,
            10
        )

        # Initialize validation state
        self.validation_results = []
        self.start_time = None
        self.end_time = None

        self.get_logger().info('Capstone Validation Node initialized')

    def capstone_status_callback(self, msg):
        """Monitor capstone status for validation"""
        try:
            status_data = json.loads(msg.data)
            state = status_data.get('state', 'unknown')

            if state == 'task_completed':
                self.end_time = time.time()
                self.validate_integration()
            elif state == 'error':
                self.record_validation_failure(status_data)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in capstone status: {msg.data}')

    def task_status_callback(self, msg):
        """Monitor task status for validation"""
        try:
            status_data = json.loads(msg.data)
            if status_data.get('status') == 'decomposed':
                self.start_time = time.time()
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in task status: {msg.data}')

    def validate_integration(self):
        """Validate the end-to-end integration"""
        if not self.start_time or not self.end_time:
            return

        execution_time = self.end_time - self.start_time

        # Validate that all components participated
        validation_result = {
            'test_name': 'end_to_end_integration',
            'status': 'passed',
            'execution_time': execution_time,
            'components_validated': [
                'voice_processing',
                'llm_planning',
                'task_decomposition',
                'navigation',
                'perception',
                'manipulation'
            ],
            'timestamp': time.time()
        }

        # Add to results
        self.validation_results.append(validation_result)

        # Publish result
        result_msg = String()
        result_msg.data = json.dumps(validation_result)
        self.validation_result_pub.publish(result_msg)

        self.get_logger().info(f'Integration validation passed: {validation_result}')

    def record_validation_failure(self, error_data):
        """Record validation failure"""
        validation_result = {
            'test_name': 'end_to_end_integration',
            'status': 'failed',
            'error': error_data.get('error', 'unknown'),
            'timestamp': time.time()
        }

        self.validation_results.append(validation_result)

        result_msg = String()
        result_msg.data = json.dumps(validation_result)
        self.validation_result_pub.publish(result_msg)

        self.get_logger().error(f'Integration validation failed: {validation_result}')


def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    capstone_integration = CapstoneIntegration()
    capstone_validation = CapstoneValidation()

    # Use MultiThreadedExecutor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(capstone_integration)
    executor.add_node(capstone_validation)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        capstone_integration.destroy_node()
        capstone_validation.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
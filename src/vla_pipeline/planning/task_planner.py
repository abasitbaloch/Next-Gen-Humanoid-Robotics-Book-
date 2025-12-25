#!/usr/bin/env python3

"""
Task Planning Node for Vision-Language-Action Pipeline

This node integrates with LLMs for task planning and decomposition.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from action_msgs.msg import GoalStatus
import json
import time
import requests
import openai
from typing import Dict, List, Any


class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')

        # Declare parameters
        self.declare_parameter('llm_model', 'gpt-3.5-turbo')
        self.declare_parameter('llm_api_key', '')
        self.declare_parameter('llm_base_url', 'https://api.openai.com/v1')
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('timeout', 30)

        # Get parameters
        self.llm_model = self.get_parameter('llm_model').value
        self.llm_api_key = self.get_parameter('llm_api_key').value
        self.llm_base_url = self.get_parameter('llm_base_url').value
        self.max_retries = self.get_parameter('max_retries').value
        self.timeout = self.get_parameter('timeout').value

        # Publishers
        self.task_plan_pub = self.create_publisher(String, '/task_plan', 10)
        self.navigation_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.manipulation_cmd_pub = self.create_publisher(String, '/manipulation_command', 10)

        # Subscribers
        self.llm_request_sub = self.create_subscription(
            String,
            '/llm_request',
            self.llm_request_callback,
            10
        )

        self.perception_result_sub = self.create_subscription(
            String,
            '/perception/result',
            self.perception_result_callback,
            10
        )

        # Initialize OpenAI client
        if self.llm_api_key:
            openai.api_key = self.llm_api_key
        else:
            self.get_logger().warn('No LLM API key provided. Using mock responses.')

        # Task planning state
        self.current_task = None
        self.task_queue = []
        self.perception_data = {}

        self.get_logger().info('Task Planner initialized')

    def llm_request_callback(self, msg):
        """Callback for LLM requests (typically from voice processor)"""
        try:
            request_data = json.loads(msg.data)
            self.get_logger().info(f'Processing LLM request: {request_data}')

            # Plan the task using LLM
            task_plan = self.plan_task_with_llm(request_data)

            if task_plan:
                # Publish the task plan
                plan_msg = String()
                plan_msg.data = json.dumps(task_plan)
                self.task_plan_pub.publish(plan_msg)

                # Execute the first step of the plan
                self.execute_task_plan(task_plan)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in LLM request: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing LLM request: {e}')

    def perception_result_callback(self, msg):
        """Callback for perception results to update planning context"""
        try:
            perception_data = json.loads(msg.data)
            self.perception_data.update(perception_data)
            self.get_logger().debug(f'Updated perception data: {list(perception_data.keys())}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in perception result: {msg.data}')

    def plan_task_with_llm(self, request_data: Dict[str, Any]) -> Dict[str, Any]:
        """Plan a task using LLM"""
        if request_data['type'] == 'voice_command':
            command = request_data['command']
            raw_input = request_data['raw_input']

            # Create a prompt for the LLM
            prompt = self.create_planning_prompt(command, raw_input)

            if self.llm_api_key:  # Use real LLM
                try:
                    response = openai.ChatCompletion.create(
                        model=self.llm_model,
                        messages=[
                            {"role": "system", "content": "You are a helpful assistant that converts natural language commands into robot action plans. Respond with a JSON object containing the task plan."},
                            {"role": "user", "content": prompt}
                        ],
                        temperature=0.1,
                        timeout=self.timeout
                    )

                    plan_text = response.choices[0].message.content
                    self.get_logger().debug(f'LLM response: {plan_text}')

                    # Parse the LLM response
                    plan = self.parse_llm_response(plan_text)
                    return plan

                except Exception as e:
                    self.get_logger().error(f'LLM call failed: {e}')
                    # Fall back to mock planning
                    return self.mock_task_planning(command)

            else:  # Use mock planning
                return self.mock_task_planning(command)

        return None

    def create_planning_prompt(self, command: Dict[str, Any], raw_input: str) -> str:
        """Create a prompt for task planning"""
        prompt = f"""
        Convert the following natural language command into a structured task plan for a humanoid robot.

        Command: {raw_input}
        Action: {command.get('action', 'unknown')}
        Target: {command.get('target', 'unknown')}

        The robot has the following capabilities:
        - Navigation: move to specific locations
        - Manipulation: grasp objects, place objects
        - Perception: detect and recognize objects
        - Interaction: respond to commands

        Respond with a JSON object containing:
        - task_id: unique identifier for the task
        - command: the original command
        - steps: array of action steps to execute
        - dependencies: any dependencies between steps
        - success_criteria: how to determine if the task was completed successfully

        Each step should have:
        - id: step identifier
        - action: the action to perform
        - parameters: any parameters needed for the action
        - description: human-readable description of the step
        """

        return prompt

    def parse_llm_response(self, response_text: str) -> Dict[str, Any]:
        """Parse the LLM response into a task plan"""
        try:
            # Extract JSON from response if it's wrapped in markdown
            if '```json' in response_text:
                start_idx = response_text.find('```json') + 7
                end_idx = response_text.find('```', start_idx)
                json_str = response_text[start_idx:end_idx].strip()
            elif '{' in response_text:
                start_idx = response_text.find('{')
                end_idx = response_text.rfind('}') + 1
                json_str = response_text[start_idx:end_idx].strip()
            else:
                json_str = response_text.strip()

            plan = json.loads(json_str)
            self.get_logger().info(f'Parsed task plan: {plan}')
            return plan

        except json.JSONDecodeError:
            self.get_logger().error(f'Could not parse LLM response as JSON: {response_text}')
            # Return a mock plan
            return self.create_mock_plan(response_text)

    def mock_task_planning(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Create a mock task plan when LLM is not available"""
        action = command.get('action', 'unknown')
        target = command.get('target', 'unknown')

        # Create mock plan based on action
        if action == 'navigate':
            plan = {
                'task_id': f'navigate_{int(time.time())}',
                'command': command,
                'steps': [
                    {
                        'id': 'find_location',
                        'action': 'find_object',
                        'parameters': {'object_name': target},
                        'description': f'Locate the {target}'
                    },
                    {
                        'id': 'navigate_to_location',
                        'action': 'navigate',
                        'parameters': {'target': target},
                        'description': f'Navigate to the {target}'
                    }
                ],
                'dependencies': [],
                'success_criteria': f'Robot is at the {target}'
            }
        elif action == 'grasp':
            plan = {
                'task_id': f'grasp_{int(time.time())}',
                'command': command,
                'steps': [
                    {
                        'id': 'find_object',
                        'action': 'find_object',
                        'parameters': {'object_name': target},
                        'description': f'Locate the {target}'
                    },
                    {
                        'id': 'navigate_to_object',
                        'action': 'navigate',
                        'parameters': {'target': target},
                        'description': f'Navigate close to the {target}'
                    },
                    {
                        'id': 'grasp_object',
                        'action': 'grasp',
                        'parameters': {'object_name': target},
                        'description': f'Grasp the {target}'
                    }
                ],
                'dependencies': [
                    {'from': 'find_object', 'to': 'navigate_to_object'},
                    {'from': 'navigate_to_object', 'to': 'grasp_object'}
                ],
                'success_criteria': f'Robot has grasped the {target}'
            }
        elif action == 'place':
            plan = {
                'task_id': f'place_{int(time.time())}',
                'command': command,
                'steps': [
                    {
                        'id': 'find_place_location',
                        'action': 'find_object',
                        'parameters': {'object_name': target},
                        'description': f'Locate the place location: {target}'
                    },
                    {
                        'id': 'navigate_to_location',
                        'action': 'navigate',
                        'parameters': {'target': target},
                        'description': f'Navigate to the {target} location'
                    },
                    {
                        'id': 'place_object',
                        'action': 'place',
                        'parameters': {'location': target},
                        'description': f'Place the object at {target}'
                    }
                ],
                'dependencies': [
                    {'from': 'find_place_location', 'to': 'navigate_to_location'},
                    {'from': 'navigate_to_location', 'to': 'place_object'}
                ],
                'success_criteria': f'Object has been placed at {target}'
            }
        else:
            plan = {
                'task_id': f'generic_{int(time.time())}',
                'command': command,
                'steps': [
                    {
                        'id': 'understand_command',
                        'action': 'unknown',
                        'parameters': {'command': command},
                        'description': f'Command not understood: {command}'
                    }
                ],
                'dependencies': [],
                'success_criteria': 'Command acknowledged'
            }

        self.get_logger().info(f'Created mock plan: {plan}')
        return plan

    def create_mock_plan(self, response_text: str) -> Dict[str, Any]:
        """Create a mock plan from LLM response text"""
        return {
            'task_id': f'mock_{int(time.time())}',
            'command': {'action': 'unknown', 'raw_command': response_text},
            'steps': [
                {
                    'id': 'mock_step',
                    'action': 'unknown',
                    'parameters': {'response': response_text},
                    'description': f'Mock step for: {response_text}'
                }
            ],
            'dependencies': [],
            'success_criteria': 'Mock task completed'
        }

    def execute_task_plan(self, task_plan: Dict[str, Any]):
        """Execute a task plan"""
        self.get_logger().info(f'Executing task plan: {task_plan["task_id"]}')

        for step in task_plan['steps']:
            self.execute_task_step(step)

    def execute_task_step(self, step: Dict[str, Any]):
        """Execute a single task step"""
        action = step['action']
        params = step.get('parameters', {})

        self.get_logger().info(f'Executing step: {step["id"]} - {action}')

        if action == 'navigate':
            target = params.get('target', 'unknown')
            self.send_navigation_goal(target)
        elif action == 'grasp':
            object_name = params.get('object_name', 'unknown')
            self.send_manipulation_command(f'grasp_object:{object_name}')
        elif action == 'place':
            location = params.get('location', 'unknown')
            self.send_manipulation_command(f'place_at:{location}')
        elif action == 'find_object':
            object_name = params.get('object_name', 'unknown')
            self.send_perception_request(object_name)
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def send_navigation_goal(self, target_location: str):
        """Send a navigation goal to the navigation system"""
        # In a real implementation, this would convert the target location
        # to a specific pose based on a map or object recognition
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        # For demonstration, use mock coordinates based on location name
        # In reality, this would come from a map or object recognition
        if 'kitchen' in target_location.lower():
            goal_msg.pose.position.x = 1.0
            goal_msg.pose.position.y = 1.0
        elif 'living room' in target_location.lower():
            goal_msg.pose.position.x = 2.0
            goal_msg.pose.position.y = 0.0
        elif 'bedroom' in target_location.lower():
            goal_msg.pose.position.x = 0.0
            goal_msg.pose.position.y = 2.0
        else:
            # Default location
            goal_msg.pose.position.x = 0.0
            goal_msg.pose.position.y = 0.0

        goal_msg.pose.orientation.w = 1.0

        self.navigation_goal_pub.publish(goal_msg)
        self.get_logger().info(f'Sent navigation goal to {target_location}: {goal_msg.pose.position}')

    def send_manipulation_command(self, command: str):
        """Send a manipulation command"""
        cmd_msg = String()
        cmd_msg.data = command
        self.manipulation_cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Sent manipulation command: {command}')

    def send_perception_request(self, object_name: str):
        """Send a perception request to find an object"""
        request_msg = String()
        request_msg.data = json.dumps({
            'request_type': 'find_object',
            'object_name': object_name,
            'timestamp': time.time()
        })
        # Publish to perception system (topic would be defined in perception system)
        # For now, we'll use a mock topic
        perception_request_pub = self.create_publisher(String, '/perception/request', 10)
        perception_request_pub.publish(request_msg)
        self.get_logger().info(f'Sent perception request to find: {object_name}')


def main(args=None):
    rclpy.init(args=args)

    task_planner = TaskPlanner()

    try:
        rclpy.spin(task_planner)
    except KeyboardInterrupt:
        pass
    finally:
        task_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
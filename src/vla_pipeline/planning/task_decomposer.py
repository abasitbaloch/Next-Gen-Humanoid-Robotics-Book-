#!/usr/bin/env python3

"""
Task Decomposition System for Vision-Language-Action Pipeline

This node decomposes high-level tasks into executable subtasks.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import json
import time
from typing import Dict, List, Any, Optional


class TaskDecomposer(Node):
    def __init__(self):
        super().__init__('task_decomposer')

        # Publishers
        self.subtask_pub = self.create_publisher(String, '/subtask_queue', 10)
        self.task_status_pub = self.create_publisher(String, '/task_status', 10)

        # Subscribers
        self.task_plan_sub = self.create_subscription(
            String,
            '/task_plan',
            self.task_plan_callback,
            10
        )

        self.subtask_status_sub = self.create_subscription(
            String,
            '/subtask_status',
            self.subtask_status_callback,
            10
        )

        # Task management
        self.active_tasks = {}
        self.subtask_queues = {}  # Maps task_id to list of subtasks
        self.subtask_status = {}  # Maps subtask_id to status

        self.get_logger().info('Task Decomposer initialized')

    def task_plan_callback(self, msg):
        """Callback for incoming task plans"""
        try:
            task_plan = json.loads(msg.data)
            task_id = task_plan.get('task_id', f'task_{int(time.time())}')

            self.get_logger().info(f'Received task plan: {task_id}')

            # Decompose the task plan into subtasks
            subtasks = self.decompose_task_plan(task_plan)

            # Store the subtasks
            self.subtask_queues[task_id] = subtasks
            self.active_tasks[task_id] = task_plan

            # Publish subtasks to the execution queue
            for subtask in subtasks:
                self.publish_subtask(subtask, task_id)

            # Publish task status
            status_msg = String()
            status_msg.data = json.dumps({
                'task_id': task_id,
                'status': 'decomposed',
                'subtasks_count': len(subtasks),
                'subtasks': [s['id'] for s in subtasks],
                'timestamp': time.time()
            })
            self.task_status_pub.publish(status_msg)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in task plan: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing task plan: {e}')

    def subtask_status_callback(self, msg):
        """Callback for subtask status updates"""
        try:
            status_data = json.loads(msg.data)
            subtask_id = status_data.get('subtask_id')
            status = status_data.get('status')
            task_id = status_data.get('task_id')

            if subtask_id:
                self.subtask_status[subtask_id] = status_data
                self.get_logger().debug(f'Updated subtask status: {subtask_id} -> {status}')

                # Check if all subtasks for a task are complete
                if task_id and task_id in self.subtask_queues:
                    self.check_task_completion(task_id)

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in subtask status: {msg.data}')

    def decompose_task_plan(self, task_plan: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Decompose a task plan into executable subtasks"""
        subtasks = []

        for i, step in enumerate(task_plan.get('steps', [])):
            subtask = {
                'id': f"{task_plan['task_id']}_{step['id']}_{i}",
                'task_id': task_plan['task_id'],
                'step_id': step['id'],
                'action': step['action'],
                'parameters': step.get('parameters', {}),
                'description': step['description'],
                'dependencies': self.get_step_dependencies(step, i, task_plan),
                'priority': i,  # Execute in order
                'status': 'pending',
                'timestamp': time.time()
            }

            subtasks.append(subtask)

        # Add dependency information
        for subtask in subtasks:
            self.resolve_dependencies(subtask, subtasks)

        return subtasks

    def get_step_dependencies(self, step: Dict[str, Any], step_index: int, task_plan: Dict[str, Any]) -> List[str]:
        """Get dependencies for a step based on the task plan"""
        dependencies = []

        # Check explicit dependencies in the task plan
        plan_dependencies = task_plan.get('dependencies', [])
        for dep in plan_dependencies:
            if dep['to'] == step['id']:
                dependencies.append(dep['from'])

        # Add sequential dependencies (each step depends on the previous one)
        if step_index > 0:
            prev_step = task_plan['steps'][step_index - 1]
            dependencies.append(prev_step['id'])

        return dependencies

    def resolve_dependencies(self, subtask: Dict[str, Any], all_subtasks: List[Dict[str, Any]]):
        """Resolve dependencies to actual subtask IDs"""
        resolved_deps = []
        for dep_step_id in subtask['dependencies']:
            # Find the corresponding subtask
            for other_subtask in all_subtasks:
                if other_subtask['step_id'] == dep_step_id and other_subtask['task_id'] == subtask['task_id']:
                    resolved_deps.append(other_subtask['id'])
                    break
        subtask['resolved_dependencies'] = resolved_deps

    def publish_subtask(self, subtask: Dict[str, Any], task_id: str):
        """Publish a subtask to the execution queue"""
        subtask_msg = String()
        subtask_msg.data = json.dumps(subtask)
        self.subtask_pub.publish(subtask_msg)

        self.get_logger().info(f'Published subtask: {subtask["id"]} for task: {task_id}')

    def check_task_completion(self, task_id: str):
        """Check if all subtasks for a task are completed"""
        if task_id not in self.subtask_queues:
            return

        subtasks = self.subtask_queues[task_id]
        all_completed = True

        for subtask in subtasks:
            subtask_id = subtask['id']
            status_data = self.subtask_status.get(subtask_id, {})
            status = status_data.get('status', 'pending')

            if status not in ['completed', 'skipped', 'failed']:
                all_completed = False
                break

        if all_completed:
            # Publish task completion status
            completion_msg = String()
            completion_msg.data = json.dumps({
                'task_id': task_id,
                'status': 'completed',
                'completion_time': time.time(),
                'subtasks_completed': len(subtasks)
            })
            self.task_status_pub.publish(completion_msg)

            self.get_logger().info(f'Task completed: {task_id}')

            # Clean up completed task data
            del self.subtask_queues[task_id]
            if task_id in self.active_tasks:
                del self.active_tasks[task_id]

            # Clean up subtask statuses
            for subtask in subtasks:
                subtask_id = subtask['id']
                if subtask_id in self.subtask_status:
                    del self.subtask_status[subtask_id]


class TaskExecutionMonitor(Node):
    """Monitor for task execution status"""
    def __init__(self):
        super().__init__('task_execution_monitor')

        # Subscribers
        self.subtask_status_sub = self.create_subscription(
            String,
            '/subtask_status',
            self.subtask_status_callback,
            10
        )

        self.task_status_sub = self.create_subscription(
            String,
            '/task_status',
            self.task_status_callback,
            10
        )

        # Publishers
        self.execution_status_pub = self.create_publisher(String, '/execution_status', 10)

        # Track execution status
        self.execution_status = {
            'active_tasks': 0,
            'completed_tasks': 0,
            'failed_tasks': 0,
            'active_subtasks': 0,
            'completed_subtasks': 0,
            'failed_subtasks': 0
        }

        self.get_logger().info('Task Execution Monitor initialized')

    def subtask_status_callback(self, msg):
        """Update execution status based on subtask status"""
        try:
            status_data = json.loads(msg.data)
            status = status_data.get('status')

            if status == 'executing':
                self.execution_status['active_subtasks'] += 1
            elif status == 'completed':
                self.execution_status['active_subtasks'] = max(0, self.execution_status['active_subtasks'] - 1)
                self.execution_status['completed_subtasks'] += 1
            elif status == 'failed':
                self.execution_status['active_subtasks'] = max(0, self.execution_status['active_subtasks'] - 1)
                self.execution_status['failed_subtasks'] += 1

            self.publish_execution_status()

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in subtask status: {msg.data}')

    def task_status_callback(self, msg):
        """Update execution status based on task status"""
        try:
            status_data = json.loads(msg.data)
            status = status_data.get('status')

            if status == 'decomposed':
                self.execution_status['active_tasks'] += 1
            elif status == 'completed':
                self.execution_status['active_tasks'] = max(0, self.execution_status['active_tasks'] - 1)
                self.execution_status['completed_tasks'] += 1
            elif status == 'failed':
                self.execution_status['active_tasks'] = max(0, self.execution_status['active_tasks'] - 1)
                self.execution_status['failed_tasks'] += 1

            self.publish_execution_status()

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in task status: {msg.data}')

    def publish_execution_status(self):
        """Publish overall execution status"""
        status_msg = String()
        status_msg.data = json.dumps(self.execution_status)
        self.execution_status_pub.publish(status_msg)

        self.get_logger().debug(f'Execution status: {self.execution_status}')


def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    task_decomposer = TaskDecomposer()
    task_monitor = TaskExecutionMonitor()

    # Use MultiThreadedExecutor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(task_decomposer)
    executor.add_node(task_monitor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        task_decomposer.destroy_node()
        task_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
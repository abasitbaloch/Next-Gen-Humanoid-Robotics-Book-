---
sidebar_position: 5
---

# Autonomous Behaviors

## Overview

This section covers autonomous behavior execution systems that enable humanoid robots to perform complex tasks without continuous human supervision. Autonomous behaviors are the culmination of perception, planning, and control systems working together to achieve high-level goals.

## Behavior Architecture

### Behavior Trees

Behavior trees provide a structured approach to autonomous behavior:

```python
from enum import Enum
from typing import List, Dict, Any, Optional
import time

class BehaviorStatus(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"
    ERROR = "error"

class BehaviorNode:
    def __init__(self, name: str):
        self.name = name
        self.status = BehaviorStatus.RUNNING
        self.children: List['BehaviorNode'] = []

    def tick(self) -> BehaviorStatus:
        """Execute one cycle of the behavior"""
        raise NotImplementedError

    def reset(self):
        """Reset behavior state"""
        self.status = BehaviorStatus.RUNNING

class CompositeNode(BehaviorNode):
    def __init__(self, name: str):
        super().__init__(name)

    def add_child(self, child: 'BehaviorNode'):
        self.children.append(child)

class SequenceNode(CompositeNode):
    """Execute children in sequence until one fails"""
    def __init__(self, name: str):
        super().__init__(name)
        self.current_child_index = 0

    def tick(self) -> BehaviorStatus:
        while self.current_child_index < len(self.children):
            child = self.children[self.current_child_index]
            child_status = child.tick()

            if child_status == BehaviorStatus.FAILURE:
                self.current_child_index = 0
                return BehaviorStatus.FAILURE
            elif child_status == BehaviorStatus.RUNNING:
                return BehaviorStatus.RUNNING
            elif child_status == BehaviorStatus.SUCCESS:
                self.current_child_index += 1

        # All children succeeded
        self.current_child_index = 0
        return BehaviorStatus.SUCCESS

class SelectorNode(CompositeNode):
    """Execute children until one succeeds"""
    def __init__(self, name: str):
        super().__init__(name)
        self.current_child_index = 0

    def tick(self) -> BehaviorStatus:
        while self.current_child_index < len(self.children):
            child = self.children[self.current_child_index]
            child_status = child.tick()

            if child_status == BehaviorStatus.SUCCESS:
                self.current_child_index = 0
                return BehaviorStatus.SUCCESS
            elif child_status == BehaviorStatus.RUNNING:
                return BehaviorStatus.RUNNING
            elif child_status == BehaviorStatus.FAILURE:
                self.current_child_index += 1

        # All children failed
        self.current_child_index = 0
        return BehaviorStatus.FAILURE
```

### Leaf Nodes

Implement basic behavior actions:

```python
class ActionNode(BehaviorNode):
    """Leaf node that performs an action"""
    def __init__(self, name: str, action_func):
        super().__init__(name)
        self.action_func = action_func
        self.start_time = None

    def tick(self) -> BehaviorStatus:
        if self.status == BehaviorStatus.RUNNING:
            result = self.action_func()
            if result is True:
                return BehaviorStatus.SUCCESS
            elif result is False:
                return BehaviorStatus.FAILURE
            else:
                return BehaviorStatus.RUNNING
        return self.status

class ConditionNode(BehaviorNode):
    """Leaf node that checks a condition"""
    def __init__(self, name: str, condition_func):
        super().__init__(name)
        self.condition_func = condition_func

    def tick(self) -> BehaviorStatus:
        result = self.condition_func()
        return BehaviorStatus.SUCCESS if result else BehaviorStatus.FAILURE
```

## Navigation Behaviors

### Waypoint Navigation

Implement autonomous navigation behaviors:

```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class NavigationBehavior:
    def __init__(self, node):
        self.node = node
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.status_publisher = node.create_publisher(String, 'behavior_status', 10)

    def navigate_to_waypoint(self, x: float, y: float, theta: float = 0.0) -> bool:
        """Navigate to specified waypoint"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = theta

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)

        # Wait for result with timeout
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)

        if future.result() is not None:
            return future.result().result
        return False

class NavigationBehaviorNode(ActionNode):
    def __init__(self, node, target_pose: Dict[str, float]):
        self.nav_behavior = NavigationBehavior(node)
        self.target_pose = target_pose
        super().__init__(f"navigate_to_{target_pose.get('name', 'unknown')}", self._execute)

    def _execute(self) -> bool:
        """Execute navigation action"""
        success = self.nav_behavior.navigate_to_waypoint(
            self.target_pose['x'],
            self.target_pose['y'],
            self.target_pose.get('theta', 0.0)
        )
        return success
```

### Obstacle Avoidance

Implement dynamic obstacle avoidance:

```python
class ObstacleAvoidanceBehavior:
    def __init__(self, node):
        self.node = node
        self.laser_subscriber = node.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_vel_publisher = node.create_publisher(Twist, 'cmd_vel', 10)
        self.obstacle_threshold = 0.5  # meters
        self.avoidance_active = False

    def scan_callback(self, msg: LaserScan):
        """Process laser scan for obstacle detection"""
        min_distance = min(msg.ranges)
        if min_distance < self.obstacle_threshold:
            self.trigger_avoidance()

    def trigger_avoidance(self):
        """Trigger obstacle avoidance behavior"""
        if not self.avoidance_active:
            self.avoidance_active = True
            self.execute_avoidance()

    def execute_avoidance(self):
        """Execute obstacle avoidance maneuver"""
        # Simple avoidance: turn away from obstacles
        twist = Twist()
        twist.angular.z = 0.5  # Turn to avoid
        self.cmd_vel_publisher.publish(twist)

class AvoidanceBehaviorNode(ActionNode):
    def __init__(self, node):
        self.avoidance_behavior = ObstacleAvoidanceBehavior(node)
        super().__init__("obstacle_avoidance", self._execute)

    def _execute(self) -> bool:
        """Execute obstacle avoidance"""
        # This would typically run as a background behavior
        return True  # Always succeeds as it runs continuously
```

## Manipulation Behaviors

### Grasping Behaviors

Implement autonomous manipulation:

```python
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ManipulationBehavior:
    def __init__(self, node):
        self.node = node
        self.arm_controller = ActionClient(
            node, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        self.gripper_controller = ActionClient(
            node, FollowJointTrajectory, 'gripper_controller/follow_joint_trajectory')

    def grasp_object(self, object_pose: Dict[str, Any]) -> bool:
        """Execute grasping behavior"""
        # Move arm to pre-grasp position
        pre_grasp_success = self.move_arm_to_pre_grasp(object_pose)
        if not pre_grasp_success:
            return False

        # Execute grasp
        grasp_success = self.execute_grasp(object_pose)
        if not grasp_success:
            return False

        # Lift object
        lift_success = self.lift_object()
        return lift_success

    def move_arm_to_pre_grasp(self, object_pose: Dict[str, Any]) -> bool:
        """Move arm to position before grasp"""
        # Calculate pre-grasp pose based on object pose
        pre_grasp_pose = self.calculate_pre_grasp_pose(object_pose)

        # Generate trajectory to pre-grasp pose
        trajectory = self.generate_arm_trajectory(pre_grasp_pose)

        return self.execute_arm_trajectory(trajectory)

    def execute_grasp(self, object_pose: Dict[str, Any]) -> bool:
        """Execute grasp action"""
        # Move to grasp pose
        grasp_pose = self.calculate_grasp_pose(object_pose)
        trajectory = self.generate_arm_trajectory(grasp_pose)

        if not self.execute_arm_trajectory(trajectory):
            return False

        # Close gripper
        return self.close_gripper()

class GraspingBehaviorNode(ActionNode):
    def __init__(self, node, object_info: Dict[str, Any]):
        self.manipulation_behavior = ManipulationBehavior(node)
        self.object_info = object_info
        super().__init__(f"grasp_{object_info.get('name', 'unknown')}", self._execute)

    def _execute(self) -> bool:
        """Execute grasping behavior"""
        return self.manipulation_behavior.grasp_object(self.object_info)
```

## Perception-Action Behaviors

### Look and Act

Combine perception and action:

```python
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionActionBehavior:
    def __init__(self, node):
        self.node = node
        self.cv_bridge = CvBridge()

        # Subscribers
        self.image_subscriber = node.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.pointcloud_subscriber = node.create_subscription(
            PointCloud2, 'pointcloud', self.pointcloud_callback, 10)

        # Data storage
        self.current_image = None
        self.current_pointcloud = None
        self.last_detection_time = 0

    def image_callback(self, msg: Image):
        """Process incoming images"""
        self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud"""
        self.current_pointcloud = msg

    def detect_and_approach_object(self, object_type: str) -> bool:
        """Detect object and approach it"""
        # Look for object
        object_pose = self.look_for_object(object_type)

        if object_pose is None:
            self.node.get_logger().warn(f"Could not find {object_type}")
            return False

        # Navigate to object
        approach_success = self.approach_object(object_pose)
        return approach_success

    def look_for_object(self, object_type: str) -> Optional[Dict[str, Any]]:
        """Look for object in current view"""
        if self.current_image is None:
            return None

        # Simple color-based detection (in practice, use more sophisticated methods)
        if object_type == "red_cup":
            return self.detect_red_object(self.current_image)
        elif object_type == "green_bottle":
            return self.detect_green_object(self.current_image)
        else:
            return self.detect_generic_object(self.current_image, object_type)

    def detect_red_object(self, image: np.ndarray) -> Optional[Dict[str, Any]]:
        """Detect red objects in image"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        # Red can also be at the high end of hue spectrum
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get largest contour (closest object)
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Minimum size threshold
                # Calculate center of object
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    return {
                        'x': cx,
                        'y': cy,
                        'contour': largest_contour
                    }

        return None

class PerceptionActionBehaviorNode(ActionNode):
    def __init__(self, node, object_type: str):
        self.perception_action = PerceptionActionBehavior(node)
        self.object_type = object_type
        super().__init__(f"detect_and_approach_{object_type}", self._execute)

    def _execute(self) -> bool:
        """Execute perception-action behavior"""
        return self.perception_action.detect_and_approach_object(self.object_type)
```

## State Management

### Behavior State Machine

Manage behavior states and transitions:

```python
class BehaviorStateMachine:
    def __init__(self):
        self.current_state = "IDLE"
        self.previous_state = None
        self.state_start_time = time.time()
        self.state_transitions = {}

    def transition_to(self, new_state: str):
        """Transition to new state"""
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_start_time = time.time()

        self.log_state_transition()

    def log_state_transition(self):
        """Log state transition for debugging"""
        print(f"State transition: {self.previous_state} -> {self.current_state}")

    def get_state_duration(self) -> float:
        """Get time spent in current state"""
        return time.time() - self.state_start_time

    def update_state(self, sensor_data: Dict[str, Any]) -> str:
        """Update state based on sensor data"""
        # Define state transition logic
        if self.current_state == "IDLE":
            if sensor_data.get('command_received'):
                return "PLANNING"
        elif self.current_state == "PLANNING":
            if sensor_data.get('plan_ready'):
                return "EXECUTING"
        elif self.current_state == "EXECUTING":
            if sensor_data.get('execution_complete'):
                return "COMPLETED"
            elif sensor_data.get('error_detected'):
                return "ERROR_HANDLING"
        elif self.current_state == "ERROR_HANDLING":
            if sensor_data.get('recovery_complete'):
                return "IDLE"

        return self.current_state  # Stay in current state
```

## Safety and Monitoring

### Behavior Safety

Implement safety checks for autonomous behaviors:

```python
class SafeBehaviorExecutor:
    def __init__(self, node):
        self.node = node
        self.safety_limits = {
            'max_velocity': 0.5,
            'max_force': 50.0,
            'max_time': 300,  # 5 minutes
            'max_current': 10.0  # for actuators
        }
        self.emergency_stop_active = False
        self.last_safe_check = time.time()

    def execute_behavior_safely(self, behavior_tree: BehaviorNode) -> BehaviorStatus:
        """Execute behavior with safety monitoring"""
        start_time = time.time()

        while not self.emergency_stop_active:
            # Check safety constraints
            if not self._check_safety_constraints():
                self._trigger_emergency_stop()
                return BehaviorStatus.ERROR

            # Execute behavior tick
            status = behavior_tree.tick()

            # Check execution time
            if time.time() - start_time > self.safety_limits['max_time']:
                self.node.get_logger().warn("Behavior exceeded maximum execution time")
                return BehaviorStatus.FAILURE

            if status != BehaviorStatus.RUNNING:
                return status

            # Small delay to prevent excessive CPU usage
            time.sleep(0.05)

        return BehaviorStatus.ERROR

    def _check_safety_constraints(self) -> bool:
        """Check all safety constraints"""
        checks = [
            self._check_velocity_limits(),
            self._check_force_limits(),
            self._check_current_limits(),
            self._check_collision_avoidance()
        ]

        return all(checks)

    def _check_velocity_limits(self) -> bool:
        """Check velocity limits"""
        # In practice, read current velocities from robot state
        current_velocity = self._get_current_velocity()
        return abs(current_velocity) <= self.safety_limits['max_velocity']

    def _check_force_limits(self) -> bool:
        """Check force/torque limits"""
        # In practice, read force/torque sensors
        current_force = self._get_current_force()
        return abs(current_force) <= self.safety_limits['max_force']

    def _trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.emergency_stop_active = True
        self._publish_emergency_stop()

    def _publish_emergency_stop(self):
        """Publish emergency stop command"""
        # Publish zero velocity commands to all controllers
        pass
```

## Learning and Adaptation

### Behavior Learning

Implement learning mechanisms for autonomous behaviors:

```python
class AdaptiveBehaviorLearner:
    def __init__(self):
        self.behavior_performance_history = {}
        self.adaptation_rules = {}
        self.learning_enabled = True

    def record_behavior_outcome(self, behavior_name: str, outcome: Dict[str, Any]):
        """Record outcome of behavior execution"""
        if behavior_name not in self.behavior_performance_history:
            self.behavior_performance_history[behavior_name] = []

        self.behavior_performance_history[behavior_name].append(outcome)

        # Check if adaptation is needed
        if self._should_adapt(behavior_name):
            self._adapt_behavior(behavior_name)

    def _should_adapt(self, behavior_name: str) -> bool:
        """Check if behavior needs adaptation"""
        if behavior_name not in self.behavior_performance_history:
            return False

        recent_outcomes = self.behavior_performance_history[behavior_name][-5:]  # Last 5 attempts
        failure_rate = sum(1 for outcome in recent_outcomes if not outcome.get('success', False)) / len(recent_outcomes)

        return failure_rate > 0.6  # Adapt if failure rate > 60%

    def _adapt_behavior(self, behavior_name: str):
        """Adapt behavior based on performance"""
        # Implement adaptation logic
        # This could involve:
        # - Adjusting parameters
        # - Changing strategy
        # - Adding safety margins
        # - Using different algorithms
        pass

    def get_adapted_behavior_params(self, behavior_name: str) -> Dict[str, Any]:
        """Get adapted parameters for behavior"""
        if behavior_name in self.adaptation_rules:
            return self.adaptation_rules[behavior_name]
        return {}
```

## Integration with VLA Pipeline

### Complete Integration

Integrate autonomous behaviors with the VLA pipeline:

```python
class VLAExecutionIntegrator:
    def __init__(self, node, task_decomposer, behavior_executor):
        self.node = node
        self.task_decomposer = task_decomposer
        self.behavior_executor = behavior_executor
        self.adaptive_learner = AdaptiveBehaviorLearner()

        # Publishers and subscribers
        self.execution_status_publisher = node.create_publisher(
            String, 'execution_status', 10)
        self.execution_feedback_subscriber = node.create_subscription(
            String, 'execution_feedback', self.feedback_callback, 10)

        self.current_execution_id = None
        self.execution_active = False

    def execute_decomposed_task(self, decomposition: List[Dict]) -> Dict[str, Any]:
        """Execute decomposed task using autonomous behaviors"""
        execution_start_time = time.time()
        execution_results = []

        for step in decomposition:
            if not self.execution_active:
                break

            # Create behavior tree for this step
            behavior_tree = self._create_behavior_tree(step)

            # Execute behavior with safety monitoring
            status = self.behavior_executor.execute_behavior_safely(behavior_tree)

            # Record outcome for learning
            outcome = {
                'step_id': step['id'],
                'status': status.value,
                'success': status == BehaviorStatus.SUCCESS,
                'execution_time': time.time() - execution_start_time
            }
            execution_results.append(outcome)

            self.adaptive_learner.record_behavior_outcome(step['action'], outcome)

            # Publish execution status
            self._publish_execution_status(outcome)

            if status == BehaviorStatus.FAILURE:
                # Handle failure - maybe try alternative or stop
                if not self._handle_step_failure(step, execution_results):
                    break

        execution_time = time.time() - execution_start_time

        return {
            'execution_id': self.current_execution_id,
            'results': execution_results,
            'overall_success': all(r['success'] for r in execution_results),
            'execution_time': execution_time,
            'steps_completed': len(execution_results)
        }

    def _create_behavior_tree(self, step: Dict[str, Any]) -> BehaviorNode:
        """Create behavior tree for a single step"""
        action_type = step.get('action', 'unknown')
        parameters = step.get('parameters', {})

        if action_type == 'navigate':
            return self._create_navigation_behavior(parameters)
        elif action_type == 'grasp':
            return self._create_grasping_behavior(parameters)
        elif action_type == 'find_object':
            return self._create_perception_behavior(parameters)
        else:
            return self._create_generic_behavior(step)

    def feedback_callback(self, msg: String):
        """Handle execution feedback"""
        try:
            feedback_data = json.loads(msg.data)
            self._process_execution_feedback(feedback_data)
        except json.JSONDecodeError:
            self.node.get_logger().error(f"Invalid feedback JSON: {msg.data}")

    def _process_execution_feedback(self, feedback: Dict[str, Any]):
        """Process execution feedback for adaptation"""
        if 'error' in feedback:
            self._handle_execution_error(feedback)
        elif 'success' in feedback:
            self._record_success(feedback)
```

## Performance Monitoring

### Execution Monitoring

Monitor behavior execution performance:

```python
class BehaviorPerformanceMonitor:
    def __init__(self):
        self.metrics = {
            'execution_success_rate': [],
            'execution_time': [],
            'resource_usage': [],
            'safety_incidents': []
        }
        self.start_times = {}

    def start_monitoring(self, execution_id: str):
        """Start monitoring an execution"""
        self.start_times[execution_id] = time.time()

    def record_execution_result(self, execution_id: str, result: Dict[str, Any]):
        """Record execution result for analysis"""
        execution_time = time.time() - self.start_times.get(execution_id, time.time())

        self.metrics['execution_success_rate'].append(result.get('overall_success', False))
        self.metrics['execution_time'].append(execution_time)

        # Calculate success rate
        success_rate = sum(self.metrics['execution_success_rate']) / len(self.metrics['execution_success_rate'])

        return {
            'success_rate': success_rate,
            'avg_execution_time': sum(self.metrics['execution_time']) / len(self.metrics['execution_time']),
            'total_executions': len(self.metrics['execution_success_rate'])
        }

    def get_performance_report(self) -> Dict[str, Any]:
        """Generate performance report"""
        if not self.metrics['execution_success_rate']:
            return {'message': 'No execution data available'}

        return {
            'success_rate': sum(self.metrics['execution_success_rate']) / len(self.metrics['execution_success_rate']),
            'average_execution_time': sum(self.metrics['execution_time']) / len(self.metrics['execution_time']),
            'total_executions': len(self.metrics['execution_success_rate']),
            'safety_incidents': len(self.metrics['safety_incidents'])
        }
```

## Troubleshooting

### Common Issues

- **Behavior oscillation**: Robot repeatedly switching between behaviors
- **Resource conflicts**: Multiple behaviors competing for resources
- **Timing issues**: Behaviors not executing in proper sequence
- **Safety violations**: Behaviors bypassing safety checks

### Solutions

- **Priority systems**: Implement behavior priorities
- **Resource management**: Coordinate resource usage
- **Synchronization**: Properly sequence behavior execution
- **Safety layers**: Multiple safety validation layers

## Next Steps

Continue to the next section to learn about human-robot interaction systems that enable natural communication and collaboration between humans and humanoid robots.
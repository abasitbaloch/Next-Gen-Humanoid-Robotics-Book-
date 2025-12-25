# Vision-Language-Action (VLA) Exercises

## Exercise 1: Voice Command Processing

### Objective
Test the voice processing pipeline and command understanding.

### Steps
1. Launch the VLA system:
   ```bash
   # Launch voice processing
   ros2 launch vla_pipeline voice_processing_launch.py
   ```

2. Send a voice command (simulated):
   ```bash
   ros2 topic pub /voice_input std_msgs/String "data: 'Move to the kitchen'"
   ```

3. Monitor the command processing:
   ```bash
   ros2 topic echo /voice_command
   ros2 topic echo /llm_request
   ```

### Validation
- Check that voice commands are properly parsed
- Verify structured commands are generated
- Confirm LLM requests are sent correctly

### Expected Outcome
The system should convert the voice command "Move to the kitchen" into a structured command with action "navigate" and target "kitchen".

---

## Exercise 2: LLM-Based Task Planning

### Objective
Test the integration of LLM for task planning and decomposition.

### Steps
1. Launch the task planning system:
   ```bash
   ros2 launch vla_pipeline task_planning_launch.py
   ```

2. Send a complex command:
   ```bash
   ros2 topic pub /llm_request std_msgs/String "data: '{\"type\": \"voice_command\", \"command\": {\"action\": \"grasp\", \"target\": \"red cup\"}, \"raw_input\": \"Pick up the red cup\"}'"
   ```

3. Monitor the task plan generation:
   ```bash
   ros2 topic echo /task_plan
   ```

### Validation
- Verify that complex commands are decomposed into simple steps
- Check that navigation and manipulation steps are properly planned
- Confirm task dependencies are established

### Expected Outcome
The system should generate a task plan with steps: locate object → navigate → grasp object.

---

## Exercise 3: Task Decomposition and Execution

### Objective
Test the task decomposition system and behavior execution.

### Steps
1. Launch the complete VLA pipeline:
   ```bash
   ros2 launch vla_pipeline vla_complete_launch.py
   ```

2. Send a task plan for execution:
   ```bash
   ros2 topic pub /task_plan std_msgs/String "data: '{\"task_id\": \"test_task_1\", \"steps\": [{\"id\": \"step_1\", \"action\": \"navigate\", \"parameters\": {\"target\": \"kitchen\"}, \"description\": \"Go to kitchen\"}, {\"id\": \"step_2\", \"action\": \"find_object\", \"parameters\": {\"object_name\": \"cup\"}, \"description\": \"Find cup\"}]}'"
   ```

3. Monitor execution:
   ```bash
   ros2 topic echo /subtask_queue
   ros2 topic echo /behavior_status
   ros2 topic echo /subtask_status
   ```

### Validation
- Check that tasks are properly decomposed into subtasks
- Verify subtasks are executed in correct order
- Confirm completion status is reported

### Expected Outcome
The system should decompose the task into subtasks and execute them sequentially, reporting status for each.

---

## Exercise 4: Autonomous Behavior Execution

### Objective
Test the execution of autonomous behaviors based on planned tasks.

### Steps
1. Launch the behavior execution system:
   ```bash
   ros2 launch vla_pipeline behavior_execution_launch.py
   ```

2. Send subtasks for execution:
   ```bash
   ros2 topic pub /subtask_queue std_msgs/String "data: '{\"id\": \"behavior_1\", \"task_id\": \"task_1\", \"action\": \"navigate\", \"parameters\": {\"target\": \"living_room\"}, \"description\": \"Go to living room\"}'"
   ```

3. Monitor behavior execution:
   ```bash
   ros2 topic echo /behavior_status
   ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}"
   ```

### Validation
- Verify behaviors are executed as planned
- Check that robot moves according to commands
- Confirm behavior status updates are published

### Expected Outcome
The robot should execute navigation behavior and move toward the specified target.

---

## Exercise 5: Human-Robot Interaction

### Objective
Test the human-robot interaction capabilities of the VLA system.

### Steps
1. Launch the complete VLA system:
   ```bash
   ros2 launch vla_pipeline vla_system_launch.py
   ```

2. Simulate human interaction:
   ```bash
   # Greeting interaction
   ros2 topic pub /interaction_event std_msgs/String "data: '{\"type\": \"greeting\", \"source\": \"human\"}'"

   # Task request
   ros2 topic pub /voice_input std_msgs/String "data: 'Please bring me the book from the table'"
   ```

3. Monitor interaction responses:
   ```bash
   ros2 topic echo /speech_output
   ros2 topic echo /task_status
   ```

### Validation
- Check that the robot responds appropriately to greetings
- Verify task requests are understood and executed
- Confirm completion is acknowledged

### Expected Outcome
The robot should greet the human, understand the request to bring a book, execute the task, and acknowledge completion.

---

## Exercise 6: End-to-End VLA Pipeline

### Objective
Test the complete VLA pipeline from voice command to physical action.

### Steps
1. Launch all VLA components:
   ```bash
   # Launch perception system
   ros2 launch perception_system perception_launch.py
   # Launch navigation system
   ros2 launch nav2_system nav2_launch.py
   # Launch manipulation system
   ros2 launch manipulation_system manipulation_launch.py
   # Launch VLA pipeline
   ros2 launch vla_pipeline complete_vla_launch.py
   ```

2. Execute a complete command:
   - Voice: "Robot, please go to the kitchen and bring me the red apple"
   - Simulated: Publish the command through the system

3. Monitor the complete pipeline:
   - Voice processing → LLM planning → Task decomposition → Behavior execution
   - Navigation to kitchen → Object detection → Grasping the apple → Return

### Validation
- All VLA components work together seamlessly
- End-to-end task execution completes successfully
- Each component provides appropriate feedback

### Expected Outcome
A complete execution from voice command to physical action, demonstrating the full VLA pipeline capability.
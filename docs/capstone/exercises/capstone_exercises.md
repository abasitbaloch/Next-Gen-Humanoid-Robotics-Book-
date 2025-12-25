# Capstone Integration Exercises

## Exercise 1: Voice-to-Action Pipeline

### Objective
Test the complete pipeline from voice command to physical action execution.

### Steps
1. Launch the complete system:
   ```bash
   # Launch all required subsystems
   ros2 launch isaac_integration nav2_humanoid_launch.py
   ros2 launch manipulation_launch.py
   ros2 launch perception_system perception_launch.py
   ros2 launch vla_pipeline complete_vla_launch.py
   ros2 launch capstone capstone_system_launch.py
   ```

2. Send a voice command:
   ```bash
   ros2 topic pub /voice_input std_msgs/String "data: 'Robot, please go to the kitchen and bring me the red cup'"
   ```

3. Monitor the complete pipeline execution:
   ```bash
   # Monitor each stage
   ros2 topic echo /voice_command
   ros2 topic echo /task_plan
   ros2 topic echo /capstone/status
   ros2 topic echo /navigation/status
   ros2 topic echo /perception/result
   ros2 topic echo /manipulation/status
   ```

### Validation
- Voice command is processed correctly
- Task plan is generated with appropriate steps
- Each subsystem executes its part
- Physical action is completed

### Expected Outcome
The robot navigates to the kitchen, locates a red cup, grasps it, and returns to the user.

---

## Exercise 2: Multi-Modal Integration Test

### Objective
Test the integration of voice, navigation, perception, and manipulation systems.

### Steps
1. Set up the environment with known objects:
   ```bash
   # Launch simulation environment with known objects
   ros2 launch simulation kitchen_world.launch.py
   ```

2. Launch all subsystems:
   ```bash
   ros2 launch complete_robot_system.launch.py
   ```

3. Execute a complex command:
   ```bash
   ros2 topic pub /voice_input std_msgs/String "data: 'Find the green bottle in the living room, pick it up, and place it on the table in the dining room'"
   ```

4. Monitor integration points:
   ```bash
   # Monitor handoffs between systems
   ros2 topic echo /llm_request
   ros2 topic echo /subtask_queue
   ros2 topic echo /perception/request
   ros2 topic echo /behavior_status
   ```

### Validation
- All subsystems participate in the task
- Information flows correctly between systems
- Task completion criteria are met
- Error handling works if any subsystem fails

### Expected Outcome
Complete execution of the multi-step task with proper coordination between all systems.

---

## Exercise 3: Error Recovery and Robustness

### Objective
Test the system's ability to handle errors and recover gracefully.

### Steps
1. Launch the capstone system:
   ```bash
   ros2 launch capstone capstone_system_launch.py
   ```

2. Introduce simulated failures:
   ```bash
   # Stop navigation temporarily
   ros2 service call /navigation/stop std_srvs/Trigger

   # Simulate perception failure
   ros2 topic pub /perception/simulated_failure std_msgs/Bool "data: True"
   ```

3. Monitor error handling:
   ```bash
   ros2 topic echo /capstone/status
   ros2 topic echo /error_status
   ros2 topic echo /recovery_status
   ```

4. Resume normal operation:
   ```bash
   ros2 service call /navigation/resume std_srvs/Trigger
   ```

### Validation
- System detects failures appropriately
- Error recovery mechanisms activate
- Task can be resumed or alternative plans generated
- User is informed of issues

### Expected Outcome
The system should handle failures gracefully, attempt recovery, and either complete the task or inform the user of limitations.

---

## Exercise 4: Performance and Timing Validation

### Objective
Validate that the complete pipeline meets timing and performance requirements.

### Steps
1. Launch the system with performance monitoring:
   ```bash
   ros2 launch capstone capstone_system_with_monitoring.launch.py
   ```

2. Execute timed tasks:
   ```bash
   # Execute a simple navigation task
   ros2 topic pub /voice_input std_msgs/String "data: 'Go to the bedroom'"
   ```

3. Monitor performance metrics:
   ```bash
   # Monitor timing and performance
   ros2 run topic_tools relay /performance_metrics /performance_log &
   ros2 topic echo /timing_analysis
   ```

4. Execute multiple sequential tasks:
   ```bash
   # Execute several tasks in sequence
   ros2 topic pub /voice_input std_msgs/String "data: 'Go to the kitchen'"
   sleep 5
   ros2 topic pub /voice_input std_msgs/String "data: 'Go to the living room'"
   sleep 5
   ros2 topic pub /voice_input std_msgs/String "data: 'Go to the bedroom'"
   ```

### Validation
- Tasks complete within acceptable time limits
- System maintains responsiveness during execution
- Performance metrics meet requirements
- No degradation over multiple tasks

### Expected Outcome
Consistent performance across multiple tasks with acceptable response times.

---

## Exercise 5: End-to-End Autonomous Task

### Objective
Execute a complete autonomous task demonstrating all capstone capabilities.

### Steps
1. Set up the complete environment:
   ```bash
   # Launch full simulation
   ros2 launch full_environment.launch.py

   # Launch all subsystems
   ros2 launch full_robot_system.launch.py
   ```

2. Execute a complex autonomous task:
   ```bash
   ros2 topic pub /voice_input std_msgs/String "data: 'Robot, I need your help. Please go to the office, find my glasses on the desk, pick them up, bring them to me in the living room, and then return to your charging station'"
   ```

3. Monitor complete execution:
   ```bash
   # Monitor all aspects of execution
   ros2 topic echo /capstone/status
   ros2 topic echo /task_status
   ros2 topic echo /execution_status
   ros2 topic echo /system_health
   ```

### Validation
- All components work together seamlessly
- Complex task is decomposed and executed properly
- System maintains awareness throughout execution
- Task completes successfully

### Expected Outcome
Complete autonomous execution of the complex multi-step task from start to finish.

---

## Exercise 6: Human-Robot Interaction Validation

### Objective
Validate the human-robot interaction aspects of the capstone system.

### Steps
1. Launch the system with HRI components:
   ```bash
   ros2 launch capstone capstone_with_hri.launch.py
   ```

2. Test various interaction patterns:
   ```bash
   # Test interruption
   ros2 topic pub /voice_input std_msgs/String "data: 'Go to the kitchen'"
   sleep 2
   ros2 topic pub /voice_input std_msgs/String "data: 'Stop, go to the bedroom instead'"

   # Test clarification requests
   ros2 topic pub /voice_input std_msgs/String "data: 'Get the thing'"
   # System should ask for clarification
   ```

3. Monitor interaction handling:
   ```bash
   ros2 topic echo /interaction_event
   ros2 topic echo /speech_output
   ros2 topic echo /behavior_status
   ```

### Validation
- System handles interruptions appropriately
- Clarification requests are made when needed
- Natural interaction patterns are supported
- User intent is preserved during interactions

### Expected Outcome
Natural and effective human-robot interaction with appropriate responses to various interaction patterns.
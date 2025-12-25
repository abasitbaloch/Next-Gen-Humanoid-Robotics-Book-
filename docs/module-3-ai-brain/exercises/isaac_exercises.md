# Isaac Integration Exercises

## Exercise 1: Perception Pipeline Setup

### Objective
Configure and test the Isaac perception pipeline with synthetic data.

### Steps
1. Launch the synthetic data generation pipeline:
   ```bash
   ros2 launch isaac_integration synthetic_data_launch.py
   ```

2. Monitor the generated data streams:
   ```bash
   ros2 topic echo /synthetic_rgb
   ros2 topic echo /synthetic_depth
   ```

3. Verify that RGB and depth images are being published correctly.

### Validation
- Check that images are published at the expected rate (10 Hz)
- Verify image dimensions match configuration (640x480)
- Confirm depth values are within expected range

### Expected Outcome
You should see continuous RGB and depth image streams being published.

---

## Exercise 2: Navigation System Configuration

### Objective
Configure and test the Nav2 navigation stack for the humanoid robot.

### Steps
1. Launch the Nav2 system:
   ```bash
   ros2 launch isaac_integration nav2_humanoid_launch.py
   ```

2. Send a navigation goal using RViz or command line:
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"
   ```

3. Monitor the navigation execution:
   ```bash
   ros2 topic echo /local_costmap/costmap
   ros2 topic echo /global_costmap/costmap
   ```

### Validation
- Check that the global and local costmaps are updated
- Verify the robot follows the planned path
- Confirm obstacle avoidance behavior

### Expected Outcome
The robot should navigate to the specified goal while avoiding obstacles.

---

## Exercise 3: Manipulation Control

### Objective
Execute basic manipulation tasks using the manipulation controller.

### Steps
1. Launch the manipulation system:
   ```bash
   ros2 launch isaac_integration manipulation_launch.py
   ```

2. Send manipulation commands:
   ```bash
   ros2 topic pub /manipulation_command std_msgs/String "data: 'grasp'"
   ros2 topic pub /manipulation_command std_msgs/String "data: 'move_to:0.5,0.2,0.3'"
   ```

3. Monitor the arm trajectory execution:
   ```bash
   ros2 topic echo /arm_controller/joint_trajectory
   ros2 topic echo /gripper_controller/joint_trajectory
   ```

### Validation
- Verify joint trajectories are sent correctly
- Check that the gripper opens and closes as commanded
- Confirm arm moves to specified positions

### Expected Outcome
The robotic arm should execute the commanded manipulation tasks.

---

## Exercise 4: Sim-to-Real Validation

### Objective
Validate the sim-to-real transfer using the validation framework.

### Steps
1. Launch the validation system:
   ```bash
   ros2 launch isaac_integration sim_to_real_validation_launch.py
   ```

2. Send simulated and real data for comparison:
   ```bash
   # This would involve running both simulation and real robot
   # and comparing their outputs
   ros2 topic echo /validation/result
   ros2 topic echo /validation/score
   ```

3. Analyze the validation results:
   ```bash
   # Check validation report file
   cat /tmp/validation_results/validation_report.json
   ```

### Validation
- Check that validation scores are above the threshold
- Verify position and orientation errors are within acceptable limits
- Confirm image and scan similarity metrics

### Expected Outcome
The validation score should be above the configured threshold (0.9), indicating successful sim-to-real transfer.

---

## Exercise 5: Integrated Perception-Navigation-Manipulation

### Objective
Execute a complete task integrating perception, navigation, and manipulation.

### Steps
1. Launch all required systems:
   ```bash
   # Launch navigation
   ros2 launch isaac_integration nav2_humanoid_launch.py
   # Launch manipulation
   ros2 launch isaac_integration manipulation_launch.py
   # Launch perception (simulated)
   ros2 launch isaac_integration perception_launch.py
   ```

2. Execute a pick-and-place task:
   - Navigate to pickup location
   - Use perception to locate object
   - Manipulate to grasp object
   - Navigate to drop-off location
   - Release object

### Validation
- All subsystems should work together seamlessly
- Task should complete without errors
- Each phase should validate successfully

### Expected Outcome
A complete pick-and-place operation from navigation to manipulation execution.
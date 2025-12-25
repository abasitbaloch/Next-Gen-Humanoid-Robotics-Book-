# Troubleshooting Guide

This guide provides solutions to common issues encountered when working with the Physical AI & Humanoid Robotics system.

## ROS 2 Common Issues

### 1. ROS 2 Environment Not Set Up
**Problem**: Commands like `ros2 run` or `ros2 launch` return "command not found"

**Solution**:
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to your ~/.bashrc to make it permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. Permission Denied for Device Access
**Problem**: Cannot access serial devices or hardware interfaces

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in for changes to take effect
```

### 3. Package Not Found
**Problem**: `ros2 run package_name executable` fails with "Package not found"

**Solution**:
```bash
# Make sure you're in the workspace root
cd ~/ros2_ws

# Build the workspace
colcon build --packages-select package_name

# Source the setup file
source install/setup.bash
```

## Simulation Issues

### 1. Gazebo Not Starting
**Problem**: Gazebo simulation fails to start or crashes

**Solution**:
1. Check GPU drivers:
   ```bash
   nvidia-smi  # For NVIDIA GPUs
   glxinfo | grep "OpenGL renderer"  # Check OpenGL support
   ```

2. Set environment variables:
   ```bash
   export GAZEBO_RENDERING_LIBRARY=ogre
   export MESA_GL_VERSION_OVERRIDE=3.3
   ```

### 2. Robot Not Spawning in Gazebo
**Problem**: Robot model fails to appear in simulation

**Solution**:
1. Check URDF model validity:
   ```bash
   check_urdf /path/to/robot.urdf
   ```

2. Verify robot state publisher:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'
   ```

### 3. Physics Issues in Simulation
**Problem**: Robot behaves unrealistically in simulation

**Solution**:
1. Check physics parameters in URDF (mass, inertia, friction)
2. Adjust Gazebo physics parameters in `.sdf` or `.world` files
3. Verify joint limits and safety controllers

## Navigation Issues

### 1. Navigation Fails to Initialize
**Problem**: Nav2 stack doesn't start properly

**Solution**:
1. Check parameters file:
   ```bash
   # Verify the parameters file is correctly formatted
   ros2 param list
   ```

2. Ensure map server is running:
   ```bash
   ros2 run nav2_map_server map_server
   ```

3. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### 2. Robot Gets Stuck or Oscillates
**Problem**: Robot fails to navigate properly or oscillates in place

**Solution**:
1. Adjust costmap parameters (inflation, resolution)
2. Tune controller parameters (velocity limits, acceleration)
3. Verify sensor data quality and frequency

## Manipulation Issues

### 1. Arm Doesn't Move as Expected
**Problem**: Manipulation commands don't result in expected motion

**Solution**:
1. Check joint limits in URDF
2. Verify joint trajectory controller is running:
   ```bash
   ros2 control list_controllers
   ```

3. Ensure proper inverse kinematics solver is configured

### 2. Grasping Fails
**Problem**: Robot fails to grasp objects successfully

**Solution**:
1. Calibrate gripper positions and forces
2. Verify object detection and pose estimation accuracy
3. Check approach trajectory and grasp planning

## Isaac Integration Issues

### 1. Isaac Components Not Responding
**Problem**: Isaac ROS nodes don't respond to commands

**Solution**:
1. Verify Isaac Sim is properly installed:
   ```bash
   # Check Isaac Sim installation
   python -c "import omni; print('Isaac Sim available')"
   ```

2. Ensure correct GPU acceleration:
   ```bash
   # Check CUDA availability
   nvidia-smi
   ```

### 2. Perception Pipeline Issues
**Problem**: Object detection or scene understanding fails

**Solution**:
1. Verify sensor configuration in URDF
2. Check camera/image topics are publishing:
   ```bash
   ros2 topic echo /camera/image_raw --field data --field header.stamp
   ```

3. Ensure perception nodes are subscribed to correct topics

## VLA Pipeline Issues

### 1. Voice Commands Not Recognized
**Problem**: Voice processing doesn't understand commands

**Solution**:
1. Check audio input device:
   ```bash
   arecord -l  # List audio devices
   ```

2. Verify microphone permissions and settings
3. Test with pre-recorded audio files

### 2. LLM Integration Fails
**Problem**: Task planning with LLM doesn't work

**Solution**:
1. Verify API key is set:
   ```bash
   echo $OPENAI_API_KEY  # Should not be empty
   ```

2. Check network connectivity
3. Verify LLM service is accessible

## Capstone Integration Issues

### 1. End-to-End Pipeline Fails
**Problem**: Complete voice-to-action pipeline doesn't execute

**Solution**:
1. Enable debug output:
   ```bash
   export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
   ```

2. Check each subsystem individually before integration
3. Verify message passing between components

### 2. Timing Issues
**Problem**: Components don't synchronize properly

**Solution**:
1. Use appropriate time synchronization:
   ```bash
   # In simulation
   export GAZEBO_USE_SIM_TIME=true
   ```

2. Adjust timeout parameters in configuration files
3. Check system performance and resource usage

## Build and Dependency Issues

### 1. Package Build Fails
**Problem**: `colcon build` fails with compilation errors

**Solution**:
1. Clean build directory:
   ```bash
   rm -rf build/ install/ log/
   ```

2. Install missing dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Check ROS 2 distribution compatibility

### 2. Python Dependencies Missing
**Problem**: Python modules not found

**Solution**:
1. Install Python dependencies:
   ```bash
   pip3 install -r requirements.txt
   # Or for ROS 2 packages
   sudo apt update && sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator
   ```

## Performance Issues

### 1. Slow Simulation
**Problem**: Simulation runs slower than real-time

**Solution**:
1. Reduce physics update rate in world file
2. Simplify collision meshes
3. Check GPU performance and drivers

### 2. High CPU Usage
**Problem**: System runs slowly or becomes unresponsive

**Solution**:
1. Reduce sensor update rates
2. Limit the number of active nodes
3. Check for infinite loops in callbacks

## Debugging Tips

### Enable Detailed Logging
```bash
# Set logging level
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Or for specific nodes
ros2 run package node --ros-args --log-level DEBUG
```

### Monitor System Resources
```bash
# Monitor CPU and memory
htop

# Monitor ROS 2 topics
ros2 topic list
ros2 topic hz /topic_name  # Check frequency

# Monitor TF tree
ros2 run tf2_tools view_frames
```

### Common Debugging Commands
```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic_name

# Check service availability
ros2 service list

# List parameters
ros2 param list
```

## Getting Help

If you encounter issues not covered in this guide:

1. Check the ROS 2 documentation: https://docs.ros.org/
2. Search ROS Answers: https://answers.ros.org/
3. Check the project's GitHub issues
4. Review the specific module documentation in the book
5. Verify your hardware meets the requirements

For hardware-specific issues, consult the documentation for your specific robot platform and NVIDIA Isaac tools.
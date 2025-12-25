# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 50GB free space
- **GPU**: NVIDIA RTX series (for Isaac Sim and accelerated simulation)
- **Processor**: Multi-core processor (Intel i7 or AMD Ryzen equivalent)

### Software Dependencies
1. **ROS 2 Humble Hawksbill**
   - Installation: Follow official ROS 2 Humble installation guide
   - Verify: `ros2 --version`

2. **Gazebo Harmonic**
   - Installation: `sudo apt install ros-humble-gazebo-*`
   - Verify: `gz sim --version`

3. **Python 3.10 or 3.11**
   - Verify: `python3 --version`

4. **Git and Version Control**
   - Verify: `git --version`

5. **Docker** (optional, for isolated environments)
   - Verify: `docker --version`

## Setting Up the Development Environment

### 1. Install ROS 2 Humble
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. Install Gazebo
```bash
# Install Gazebo Harmonic
sudo apt install ros-humble-gazebo-*
```

### 3. Create Workspace
```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

### 4. Clone Book Repository
```bash
cd ~/ros2_ws/src
git clone [repository-url]
cd ~/ros2_ws
colcon build --packages-select [book-packages]
source install/setup.bash
```

## Running Your First Example

### 1. Basic Publisher/Subscriber
```bash
# Terminal 1: Run the publisher
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2: Run the subscriber
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_cpp listener
```

### 2. Launch File Example
```bash
# Run a launch file
source ~/ros2_ws/install/setup.bash
ros2 launch [package_name] [launch_file].py
```

### 3. Simulation Example
```bash
# Start Gazebo simulation
source ~/ros2_ws/install/setup.bash
ros2 launch [simulation_package] [simulation_launch_file].py

# In another terminal, send commands to the simulated robot
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

## Book Structure Navigation

### Module 1: ROS 2 (Robotic Nervous System)
- **Chapter 1**: ROS 2 Fundamentals - Start here if new to ROS 2
- **Chapter 2**: Nodes and Communication - Core communication patterns
- **Chapter 3**: URDF and Robot Description - Defining robot models

### Module 2: Digital Twin (Gazebo & Unity)
- **Chapter 1**: Simulation Fundamentals - Setting up Gazebo environments
- **Chapter 2**: Robot Models in Simulation - Importing URDF to simulation

### Module 3: AI-Robot Brain (NVIDIA Isaac)
- **Chapter 1**: Isaac Sim Introduction - Setting up Isaac Sim environment
- **Chapter 2**: Perception Systems - Working with sensors and data

### Module 4: Vision-Language-Action (VLA)
- **Chapter 1**: VLA Concepts - Understanding AI integration
- **Chapter 2**: Voice Processing - Implementing speech-to-text

## Capstone: Autonomous Humanoid

### Running the Complete Pipeline
```bash
# 1. Start the simulation environment
source ~/ros2_ws/install/setup.bash
ros2 launch capstone_bringup simulation.launch.py

# 2. Start the perception system
ros2 launch capstone_perception perception.launch.py

# 3. Start the navigation system
ros2 launch capstone_navigation navigation.launch.py

# 4. Start the voice processing system
ros2 run capstone_voice voice_processor.py

# 5. Send a voice command (or use the test interface)
ros2 topic pub /voice_command std_msgs/String "data: 'Move to the kitchen and pick up the red cup'"
```

## Troubleshooting Common Issues

### ROS 2 Environment Not Sourced
- **Issue**: Command not found errors
- **Solution**: Run `source ~/ros2_ws/install/setup.bash` or add to `~/.bashrc`

### Gazebo Not Starting
- **Issue**: Graphics driver conflicts
- **Solution**: Ensure NVIDIA drivers are properly installed and run with proper environment variables

### Package Not Found
- **Issue**: `ros2 run` command fails
- **Solution**: Run `colcon build` in workspace and source the setup file

## Development Workflow

### 1. Create New Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_package
# or for C++
ros2 pkg create --build-type ament_cmake my_robot_package_cpp
```

### 2. Build and Test
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash
# Test your package
ros2 run my_robot_package my_node
```

### 3. Launch and Integrate
```bash
# Create launch files for easy testing
# Test integration with other components
# Validate against book exercises
```

## Validation Checklist

- [ ] ROS 2 Humble installed and working
- [ ] Gazebo simulation environment running
- [ ] Basic publisher/subscriber example working
- [ ] Workspace created and building successfully
- [ ] All dependencies installed
- [ ] GPU acceleration working (if applicable)
- [ ] Book code examples running on target platform
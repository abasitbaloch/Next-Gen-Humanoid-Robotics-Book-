# Development Environment Setup

This guide will help you set up the development environment for the Physical AI & Humanoid Robotics book.

## Hardware Requirements

### Minimum Requirements
- **Processor**: Intel i7 or AMD Ryzen equivalent (4+ cores)
- **RAM**: 16GB
- **Storage**: 50GB free space
- **GPU**: NVIDIA RTX 2060 or equivalent for accelerated simulation
- **OS**: Ubuntu 22.04 LTS

### Recommended Requirements
- **Processor**: Intel i9 or AMD Ryzen 9 (8+ cores)
- **RAM**: 32GB or more
- **Storage**: 100GB SSD
- **GPU**: NVIDIA RTX 3080 or higher for Isaac Sim
- **Network**: Reliable internet connection for package downloads

### Specialized Hardware (Optional)
- **Robot Platform**: Unitree G1 humanoid robot (or Go2 quadruped for simplified examples)
- **Sensors**: Intel RealSense depth camera, LIDAR (e.g., Ouster or Velodyne)
- **Development Kit**: NVIDIA Jetson Orin Nano/NX for edge deployment
- **Additional**: Robot gripper, force/torque sensors

## Software Requirements

### Operating System
- **Primary**: Ubuntu 22.04 LTS (recommended for ROS 2 Humble compatibility)
- **Alternative**: Windows 10/11 with WSL2 (Ubuntu 22.04) or macOS with Docker

### Essential Software Stack
- **ROS 2**: Humble Hawksbill (LTS) - Long Term Support version
- **Simulation**: Gazebo Harmonic (primary), Isaac Sim 4.x (optional)
- **Development**: Python 3.10/3.11, C++17 compiler
- **Visualization**: RViz2, Gazebo GUI

## ROS 2 Humble Installation

1. **Set up sources**:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
   ```

2. **Install ROS 2 packages**:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop
   sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

3. **Initialize rosdep**:
   ```bash
   sudo rosdep init
   rosdep update
   ```

4. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

## Gazebo Installation

Install Gazebo Harmonic:
```bash
sudo apt install ros-humble-gazebo-*
```

## Isaac Sim Setup

For NVIDIA Isaac Sim, follow the official installation guide:
1. Ensure NVIDIA GPU drivers are properly installed
2. Download Isaac Sim from NVIDIA Developer website
3. Follow the installation instructions for your platform

## Python Environment

Create a virtual environment for Python development:
```bash
python3 -m venv ~/ros2_env
source ~/ros2_env/bin/activate
pip install --upgrade pip
```

## Workspace Setup

Create your ROS 2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Verification

Test your installation:
```bash
ros2 --version
gz --version
```

## Troubleshooting

### Common Issues

1. **Package not found**: Make sure you've sourced the ROS 2 environment
2. **Permission errors**: Check that you have proper permissions for the workspace
3. **GPU acceleration not working**: Verify NVIDIA drivers are properly installed

### Environment Variables

Add these to your `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/your_robot_models
```
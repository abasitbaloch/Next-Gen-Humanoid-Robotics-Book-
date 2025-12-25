# Physical AI & Humanoid Robotics Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-09

## Active Technologies

- ROS 2 Humble Hawksbill (LTS)
- Gazebo Harmonic (Simulation)
- NVIDIA Isaac Sim and Isaac ROS
- Python 3.10/3.11
- Ubuntu 22.04 LTS
- Docusaurus v3 (Documentation)
- Vision-Language-Action (VLA) systems
- Nav2 Navigation Stack
- Jetson Orin Nano/NX (Deployment)

## Project Structure

```text
next-gen-humanoid-robotics-book/
├── docs/                    # Docusaurus documentation site
├── specs/1-physical-ai-humanoid/  # Feature specifications
│   ├── spec.md             # Feature specification
│   ├── plan.md             # Implementation plan
│   ├── research.md         # Research findings
│   ├── data-model.md       # Data models
│   ├── quickstart.md       # Quickstart guide
│   ├── contracts/          # API contracts
│   └── tasks.md            # Implementation tasks
├── src/                    # Source code packages
│   ├── ros2_basics/        # ROS 2 fundamentals
│   ├── simulation/         # Gazebo/Unity simulation
│   ├── isaac_integration/  # NVIDIA Isaac components
│   └── vla_pipeline/       # Vision-Language-Action systems
├── tests/                  # Test suites
└── .specify/               # SpecKit Plus configuration
```

## Commands

### ROS 2 Commands
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# Run ROS 2 packages
ros2 run <package_name> <executable_name>

# Launch files
ros2 launch <package_name> <launch_file>.py

# Check ROS 2 status
ros2 topic list
ros2 node list
```

### Simulation Commands
```bash
# Start Gazebo simulation
gz sim

# Launch robot simulation
ros2 launch <simulation_package> <launch_file>.py
```

### Docusaurus Commands
```bash
# Install dependencies
npm install

# Start local server
npm start

# Build static site
npm run build

# Deploy to GitHub Pages
npm run deploy
```

## Code Style

### Python (ROS 2)
- Follow PEP 8 style guide
- Use rclpy for ROS 2 Python nodes
- Node class structure with proper initialization
- Publisher/subscriber patterns with appropriate queue sizes
- Proper logging using `self.get_logger().info()`

### Launch Files
- Use Python launch files for complex configurations
- YAML launch files for simple parameter loading
- Parameter files in separate YAML files
- Use launch arguments for configurable parameters

### URDF/XACRO
- Use XACRO for parameterized robot descriptions
- Include proper joint limits and safety controllers
- Separate visual and collision models appropriately
- Use standard joint and link naming conventions

## Recent Changes

- Feature 1-physical-ai-humanoid: Created comprehensive Physical AI & Humanoid Robotics book
  - 4 modules: ROS 2, Digital Twin, AI-Robot Brain, Vision-Language-Action
  - Capstone project: Autonomous Humanoid (voice → plan → nav → perception → manipulation)
  - Complete development workflow with simulation and deployment

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
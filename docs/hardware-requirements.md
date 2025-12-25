# Hardware Requirements Appendix

This document outlines the hardware requirements for developing, simulating, and deploying the Physical AI & Humanoid Robotics system.

## Development Workstation Requirements

### Minimum Specifications
- **CPU**: Intel i7-8700K or AMD Ryzen 7 3700X (6 cores, 12 threads)
- **RAM**: 16 GB DDR4
- **GPU**: NVIDIA GTX 1060 (6GB) or equivalent
- **Storage**: 500 GB SSD
- **OS**: Ubuntu 22.04 LTS
- **Network**: Gigabit Ethernet

### Recommended Specifications
- **CPU**: Intel i9-12900K or AMD Ryzen 9 5900X (16+ cores)
- **RAM**: 32-64 GB DDR4 (3200MHz+)
- **GPU**: NVIDIA RTX 3080/4080 or RTX A4000/A5000 series
- **Storage**: 1 TB+ NVMe SSD
- **Additional**: Multiple monitors for development

### GPU Requirements for Isaac Sim
- **Minimum**: NVIDIA RTX 2060 (8GB VRAM)
- **Recommended**: NVIDIA RTX 3080/4080 or RTX A4000/A5000
- **VRAM**: Minimum 8GB, Recommended 12GB+
- **CUDA**: Compute Capability 6.0+
- **Driver**: Latest NVIDIA driver supporting CUDA 11.8+

## Simulation Requirements

### Physics Simulation
- **CPU**: Multi-core processor with high single-core performance
- **RAM**: 16GB+ for complex scenes
- **GPU**: Dedicated graphics card with good OpenGL support
- **VRAM**: 8GB+ for high-fidelity rendering

### Real-time Performance
To achieve real-time or faster simulation:
- **CPU**: 8+ cores for parallel physics computation
- **GPU**: Modern GPU with good compute capabilities
- **Memory**: 32GB+ for large environments

## Robot Hardware Platforms

### Reference Platform: Unitree G1
- **Actuators**: 20+ high-torque servos
- **Sensors**: IMU, cameras, force/torque sensors
- **Compute**: Onboard Jetson Orin AGX (optional)
- **Power**: 48V battery system
- **Communication**: Ethernet, WiFi, CAN bus

### Alternative Platforms
- **Boston Dynamics**: Spot, Atlas (research access required)
- **PAL Robotics**: REEM-C, TIAGo
- **ROBOTIS**: OP3, OP2
- **Custom**: ROS-compatible humanoid platforms

## NVIDIA Jetson Deployment

### Jetson Orin Series
- **Jetson Orin AGX**: 64GB RAM, 2048 CUDA cores
- **Jetson Orin NX**: 8GB RAM, 1024 CUDA cores
- **Jetson Orin Nano**: 4GB RAM, 512 CUDA cores

### Requirements for Deployment
- **OS**: JetPack 5.1+ (based on Ubuntu 20.04/22.04)
- **ROS 2**: Humble Hawksbill
- **Memory**: 8GB+ recommended for full functionality
- **Thermal**: Adequate cooling for sustained compute loads
- **Power**: 60W+ power supply for Orin AGX

## Sensor Requirements

### Vision Systems
- **RGB Cameras**: 720p/1080p at 30fps minimum
- **Depth Sensors**: L515, D435i, or equivalent
- **Wide-angle**: 120+ degree FOV for navigation
- **Mounting**: Stable mounting with known calibration

### Inertial Measurement
- **IMU**: 9-axis (accelerometer, gyroscope, magnetometer)
- **Accuracy**: Consumer-grade or better
- **Rate**: 100Hz+ for dynamic applications
- **Integration**: ROS driver support required

### Force/Torque Sensors
- **Joints**: At critical manipulation joints
- **Gripper**: Force feedback for grasp control
- **Rate**: 100Hz+ for responsive control
- **Accuracy**: Appropriate for task requirements

## Network Requirements

### Real-time Communication
- **Latency**: <10ms for control loops
- **Bandwidth**: 100Mbps+ for sensor data
- **Reliability**: Deterministic communication for safety

### Development Network
- **LAN**: Gigabit Ethernet for development
- **WiFi**: 802.11ac for mobile access
- **Access Points**: Redundant coverage for mobile robots

## Safety Equipment

### Physical Safety
- **Emergency Stop**: Accessible hardware button
- **Safety Cage**: For testing with moving parts
- **Light Curtains**: Perimeter safety for operation
- **Padding**: On robot and environment for collision safety

### Electrical Safety
- **Grounding**: Proper electrical grounding
- **Fuses**: Appropriate fusing for power systems
- **Breakers**: Individual circuit protection
- **UPS**: Uninterruptible power for critical systems

## Development Tools

### Debugging Hardware
- **Oscilloscope**: For electrical debugging
- **Multimeter**: Basic electrical measurements
- **Logic Analyzer**: Digital signal analysis
- **Power Analyzer**: Power consumption measurement

### Calibration Equipment
- **Calibration Boards**: Chessboard patterns for cameras
- **Measurement Tools**: Calipers, rulers for physical calibration
- **Reference Objects**: Known objects for perception validation
- **Laser Distance**: Accurate distance measurement

## Environmental Requirements

### Operating Conditions
- **Temperature**: 10-35°C for electronics
- **Humidity**: 20-80% non-condensing
- **Ventilation**: Adequate cooling for compute hardware
- **Vibration**: Stable mounting for sensors

### Laboratory Setup
- **Space**: Minimum 3x3 meters for basic navigation
- **Flooring**: Smooth, non-slip surface
- **Lighting**: Consistent, controllable lighting
- **Obstacles**: Standard objects for testing

## Performance Benchmarks

### Simulation Performance
- **Physics Rate**: Real-time or faster (1x+)
- **Render Rate**: 30fps+ for visual debugging
- **Sensor Simulation**: Realistic noise and latency models

### Real-time Control
- **Control Rate**: 50Hz+ for stable control
- **Communication**: Deterministic timing
- **Response Time**: <50ms for safety-critical functions

## Upgrade Paths

### GPU Scaling
- **Entry Level**: RTX 3060 → RTX 4060
- **Mid Range**: RTX 3070 → RTX 4070
- **High End**: RTX 3080 → RTX 4080/4090
- **Professional**: RTX A4000 → RTX A5000/A6000

### Compute Scaling
- **Development**: Workstation → Render Farm
- **Simulation**: Single GPU → Multi-GPU SLI
- **Training**: Local → Cloud GPU instances
- **Deployment**: Jetson → Edge server

## Budget Considerations

### Research Lab Setup (Estimate)
- **Workstations**: $5,000 - $15,000 each
- **Robot Platform**: $50,000 - $200,000
- **Sensors**: $5,000 - $15,000
- **Infrastructure**: $10,000 - $20,000
- **Safety Equipment**: $2,000 - $5,000

### Individual Developer Setup (Estimate)
- **Workstation**: $2,000 - $5,000
- **Simulation Only**: $1,000 - $3,000
- **Sensors**: $500 - $2,000
- **Total**: $3,500 - $10,000

## Compatibility Matrix

| Component | Minimum | Recommended | Professional |
|-----------|---------|-------------|--------------|
| GPU VRAM | 6GB | 12GB | 24GB+ |
| CPU Cores | 6 | 16 | 32+ |
| System RAM | 16GB | 32GB | 64GB+ |
| Storage | 500GB | 1TB SSD | 2TB+ NVMe |
| Network | 1Gbps | 10Gbps | 10Gbps+ |

## Procurement Guidelines

### Where to Purchase
- **GPUs**: NVIDIA, authorized resellers
- **Robot Platforms**: Direct from manufacturers
- **Sensors**: ROS-compatible vendors
- **Development**: Standard computer retailers

### Lead Times
- **GPUs**: 2-8 weeks depending on availability
- **Robot Platforms**: 4-16 weeks for custom builds
- **Sensors**: 1-4 weeks for standard models
- **Custom Parts**: 2-12 weeks depending on complexity

This hardware requirements document should be reviewed and updated regularly as technology evolves and new platforms become available. Always verify compatibility with your specific software stack before procurement.
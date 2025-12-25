---
sidebar_position: 3
---

# Sensors and Physics Configuration

## Overview

This section covers configuring sensors and physics parameters in your simulation environment to match real-world behavior as closely as possible.

## Physics Configuration

### Gravity and World Settings

Configure your simulation world with appropriate physics parameters:

```xml
<!-- In your .world file -->
<sdf version='1.7'>
  <world name='default'>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
  </world>
</sdf>
```

### Collision Properties

Configure collision properties for realistic interaction:

- **Friction coefficients**: Static and dynamic friction for different surfaces
- **Bounce**: Restitution coefficients for elastic collisions
- **Contact surfaces**: Parameters for stable contact resolution

### Inertial Properties

Ensure your robot's inertial properties are realistic:

- Mass values should reflect real hardware
- Inertia tensors should be calculated properly
- Center of mass should be accurate for stable simulation

## Sensor Configuration

### Camera Sensors

Configure RGB cameras with realistic parameters:

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Sensors

Configure depth sensors for 3D perception:

- Point cloud generation parameters
- Depth accuracy and noise modeling
- Field of view settings

### LIDAR Sensors

Set up LIDAR sensors for navigation:

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="head_rplidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>5.5</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_head_rplidar_controller" filename="libgazebo_ros_laser.so">
      <topic_name>scan</topic_name>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensors

Configure IMU sensors for state estimation:

- Accelerometer and gyroscope parameters
- Noise characteristics
- Update rates

## Physics Tuning

### Real-time Performance

Balance accuracy with performance:

- Step size: Smaller for accuracy, larger for performance
- Update rate: Match real-world sensor rates
- Real-time factor: 1.0 for real-time simulation

### Stability

Ensure simulation stability:

- Proper mass and inertia values
- Appropriate solver parameters
- Adequate update rates for control systems

## Sensor Calibration

### Intrinsic Calibration

Simulate camera intrinsic parameters that match real hardware:

- Focal length and principal point
- Distortion coefficients
- Image format and resolution

### Extrinsic Calibration

Configure sensor positions and orientations relative to the robot:

- Transform relationships between sensors and robot frames
- Sensor mounting positions and angles

## Validation

### Physics Validation

- Compare simulation behavior with real robot (when available)
- Verify that basic movements are stable and realistic
- Test contact behavior and friction

### Sensor Validation

- Compare sensor outputs with real hardware characteristics
- Validate noise models and accuracy
- Check update rates and data quality

## Common Issues

- **Simulation instability**: Check mass/inertia values and solver parameters
- **Sensor noise**: Adjust noise parameters to match real hardware
- **Performance issues**: Reduce physics complexity or increase step size
- **Drift**: Verify IMU and odometry parameters

## Next Steps

Continue to the next section to learn about creating simulation environments and worlds.
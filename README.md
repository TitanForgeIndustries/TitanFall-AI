# TITAN-FALL-AI — C++ / ROS 2 / Gazebo Scaffold

**Author:** Bryce K. Courtney | Titan Forge Industries  
**Version:** 1.0.0  
**License:** MIT

## Overview

Titan Fall AI is an advanced exosuit control system built on ROS 2 and Gazebo. This project provides a comprehensive scaffold for developing intelligent robotic exosuits with advanced AI capabilities including sensor fusion, intent recognition, torque control, energy management, fault detection, and thermal management.

## Features

### Core Systems
- **Sensor Fusion**: Multi-sensor data fusion using Kalman and complementary filters
- **Intent Model**: AI-powered user intent recognition and prediction
- **Torque Controller**: Advanced PID-based torque control with safety limits
- **Energy Manager**: Intelligent power management and battery optimization
- **Fault Detection**: Real-time fault detection and system health monitoring
- **Thermal Manager**: Thermal monitoring and cooling system control

### Key Capabilities
- Real-time sensor data processing
- Predictive intent recognition
- Adaptive torque control
- Dynamic power optimization
- Comprehensive fault detection
- Thermal protection systems
- Gazebo simulation support
- ROS 2 integration

## Prerequisites

### System Requirements
- Ubuntu 20.04+ or macOS 10.15+
- ROS 2 Humble or later
- Gazebo Classic or Ignition Gazebo
- C++17 compatible compiler
- CMake 3.8+

### Dependencies
```bash
# ROS 2 packages
sudo apt install ros-humble-rclcpp ros-humble-std-msgs ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-tf2
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros-control ros-humble-controller-manager
sudo apt install ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller
sudo apt install ros-humble-position-controllers ros-humble-velocity-controllers
sudo apt install ros-humble-effort-controllers

# Additional tools
sudo apt install python3-colcon-common-extensions python3-rosdep
```

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/titanforge/titanfall_ai.git
cd titanfall_ai
```

### 2. Setup Environment
```bash
# Make setup script executable
chmod +x setup_env.sh

# Run setup script
./setup_env.sh
```

### 3. Build the Package
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build with colcon
colcon build --packages-select titanfall_ai

# Source the workspace
source install/setup.bash
```

## Usage

### Running the Simulation
```bash
# Launch the complete simulation
ros2 launch titanfall_ai titanfall_sim.launch.py

# Launch only Gazebo world
ros2 launch titanfall_ai gazebo_world.launch.py

# Run the main AI node
ros2 run titanfall_ai titanfall_ai_node
```

### Configuration
The system uses YAML configuration files located in the `config/` directory:

- `ros_topics.yaml`: ROS topic names and QoS settings
- `pid_params.yaml`: PID controller parameters and joint limits
- `system_thresholds.yaml`: System safety thresholds and limits
- `power_limits.yaml`: Power management and battery configuration

### Customization
You can customize the system by modifying:
- Configuration files in `config/`
- Gazebo world file in `worlds/test_environment.world`
- Robot model in `models/exosuit_model.sdf`
- Launch parameters in `launch/`

## Architecture

### System Components
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensor Fusion │    │   Intent Model  │    │ Torque Controller│
│                 │    │                 │    │                 │
│ • IMU Processing│    │ • Pattern Recog │    │ • PID Control   │
│ • Data Fusion   │    │ • Intent Predict │    │ • Safety Limits │
│ • State Est.    │    │ • Learning      │    │ • Joint Control │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │  Main Controller │
                    │                 │
                    │ • System Coord.  │
                    │ • State Mgmt.    │
                    │ • Error Handling │
                    └─────────────────┘
                                 │
         ┌───────────────────────┼───────────────────────┐
         │                       │                       │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Energy Manager  │    │ Fault Detection │    │ Thermal Manager │
│                 │    │                 │    │                 │
│ • Power Mgmt.   │    │ • Fault Monitor │    │ • Temp Monitor  │
│ • Battery Opt.  │    │ • Health Check  │    │ • Cooling Ctrl  │
│ • Efficiency    │    │ • Error Report  │    │ • Thermal Prot. │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Data Flow
1. **Sensors** → Raw sensor data (IMU, joint states, temperature, power)
2. **Sensor Fusion** → Fused state estimation and odometry
3. **Intent Model** → User intent recognition and prediction
4. **Torque Controller** → Joint torque commands based on intent
5. **Energy Manager** → Power optimization and battery management
6. **Fault Detection** → System health monitoring and fault detection
7. **Thermal Manager** → Temperature monitoring and cooling control

## API Reference

### Main Controller
```cpp
// Initialize the system
main_controller.initialize();

// Start the system
main_controller.start();

// Run the main loop
main_controller.run();

// Get system status
SystemState state = main_controller.getCurrentState();
bool healthy = main_controller.isSystemHealthy();
```

### Sensor Fusion
```cpp
// Get fused state
nav_msgs::msg::Odometry odom = sensor_fusion.getFusedOdometry();
geometry_msgs::msg::Pose pose = sensor_fusion.getCurrentPose();

// Configure fusion parameters
sensor_fusion.setFusionParameters(0.98, 0.02, 0.01);
```

### Intent Model
```cpp
// Get intent prediction
IntentCommand intent = intent_model.predictNextIntent();

// Configure model
intent_model.setPredictionHorizon(2.0);
intent_model.setConfidenceThreshold(0.7);
```

## Testing

### Unit Tests
```bash
# Run unit tests
colcon test --packages-select titanfall_ai

# View test results
colcon test-result --all --verbose
```

### Integration Tests
```bash
# Run simulation tests
ros2 launch titanfall_ai titanfall_sim.launch.py test_mode:=true
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For support and questions:
- **Email**: bryce@titanforge.com
- **Issues**: [GitHub Issues](https://github.com/titanforge/titanfall_ai/issues)
- **Documentation**: [Wiki](https://github.com/titanforge/titanfall_ai/wiki)

## Acknowledgments

- ROS 2 Community
- Gazebo Development Team
- Titan Forge Industries Team

---

**Titan Fall AI** - Advancing the future of intelligent robotics

# TITAN-FALL-AI — C++ / ROS 2 / Gazebo Scaffold

**Author:** Bryce K. Courtney | Titan Forge Industries  
**Version:** 1.0.0  
**License:** MIT

## Overview

Titan Fall AI is an advanced exosuit control system built on ROS 2 and Gazebo. This project provides a comprehensive scaffold for developing intelligent robotic exosuits with advanced AI capabilities including sensor fusion, intent recognition, torque control, energy management, fault detection, and thermal management.

## Features

### Core Systems
- **Sensor Fusion**: Multi-sensor data fusion using Kalman and complementary filters
- **Intent Model**: AI-powered user intent recognition and prediction with TensorFlow Lite
- **Torque Controller**: Advanced PID-based torque control with safety limits
- **Energy Manager**: Intelligent power management and battery optimization
- **Fault Detection**: Real-time fault detection and system health monitoring
- **Thermal Manager**: Thermal monitoring and cooling system control

### AI & Machine Learning Features
- **TensorFlow Lite Integration**: Neural network inference in C++ for real-time intent prediction
- **Advanced Feature Extraction**: Temporal convolutions and attention mechanisms for sensor data
- **Online Learning**: Q-learning based reinforcement learning for continuous improvement
- **Multiple Model Architectures**: Support for MLP, LSTM, and CNN models
- **Python Training Pipeline**: Complete workflow for data collection, training, and deployment
- **Experience Replay**: Stores and learns from past experiences (10,000 sample buffer)

### Key Capabilities
- Real-time sensor data processing with advanced feature extraction
- Predictive intent recognition using deep learning
- Adaptive torque control with learned policies
- Dynamic power optimization
- Comprehensive fault detection
- Thermal protection systems
- Gazebo simulation support
- ROS 2 integration
- Continuous learning from real-world experience

## Prerequisites

### System Requirements
- Ubuntu 20.04+ or macOS 10.15+
- ROS 2 Humble or later
- Gazebo Classic or Ignition Gazebo
- C++17 compatible compiler
- CMake 3.8+

### Dependencies

#### ROS 2 Packages
```bash
sudo apt install ros-humble-rclcpp ros-humble-std-msgs ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-tf2
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros-control ros-humble-controller-manager
sudo apt install ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller
sudo apt install ros-humble-position-controllers ros-humble-velocity-controllers
sudo apt install ros-humble-effort-controllers
```

#### TensorFlow Lite (Optional - for ML inference)
```bash
# Install TensorFlow Lite C++ library
# Option 1: Build from source (recommended)
git clone https://github.com/tensorflow/tensorflow.git
cd tensorflow
./tensorflow/lite/tools/make/download_dependencies.sh
./tensorflow/lite/tools/make/build_lib.sh

# Option 2: Use pre-built package (if available)
# The system will fall back to rule-based classification if TFLite is not available
```

#### Python Dependencies (for training)
```bash
cd scripts
pip install -r requirements.txt
```

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

### 🚀 Quick Start (5 Minutes)

```bash
# 1. Build and source
colcon build --packages-select titanfall_ai
source install/setup.bash

# 2. Run your first example
ros2 run titanfall_ai intent_recognition_example 1

# 3. Train your first AI model
cd src/TitanFall-AI
python3 scripts/collect_training_data.py --demo
python3 scripts/train_intent_model.py --synthetic --epochs 20
```

**🎉 Success!** You've just trained your first AI model!

### Running the Simulation

```bash
# Launch the complete simulation
ros2 launch titanfall_ai titanfall_sim.launch.py

# Launch only Gazebo world
ros2 launch titanfall_ai gazebo_world.launch.py

# Run the main AI node
ros2 run titanfall_ai titanfall_ai_node

# Run intent recognition examples
ros2 run titanfall_ai intent_recognition_example 1  # Basic demo
ros2 run titanfall_ai intent_recognition_example 2  # Advanced callbacks
ros2 run titanfall_ai intent_recognition_example 3  # Batch training
ros2 run titanfall_ai intent_recognition_example 4  # Real-time recognition
```

### Training AI Models

```bash
# Collect training data from ROS bags
python3 scripts/collect_training_data.py --bag_dir ./rosbags --output_dir ./data/training

# Train with different architectures
python3 scripts/train_intent_model.py --architecture mlp --epochs 100
python3 scripts/train_intent_model.py --architecture lstm --epochs 150
python3 scripts/train_intent_model.py --architecture cnn --epochs 100

# Test trained model
python3 scripts/test_intent_model.py --model_path ./models/intent_model.tflite
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

### Intent Model (AI-Powered)

The intent recognition system uses advanced machine learning with TensorFlow Lite and online learning capabilities.

```cpp
// Load pre-trained TensorFlow Lite model
intent_model.loadModel("/path/to/intent_model.tflite");

// Get intent prediction
IntentCommand current_intent = intent_model.recognizeCurrentIntent();
IntentCommand predicted_intent = intent_model.predictNextIntent();

// Configure model
intent_model.setPredictionHorizon(2.0);      // Predict 2 seconds ahead
intent_model.setConfidenceThreshold(0.7);    // 70% confidence threshold

// Enable/disable specific intents
intent_model.enableIntentType(IntentType::JUMP, true);

// Online learning - provide feedback
bool execution_success = executeIntent(predicted_intent);
intent_model.updateModel(predicted_intent, execution_success);

// Save learned model
intent_model.saveModel("/path/to/learned_model.dat");
```

**Features:**
- **TensorFlow Lite Integration**: Neural network inference in C++
- **Advanced Feature Extraction**: Temporal convolutions and attention mechanisms
- **Online Learning**: Q-learning for continuous improvement
- **Multiple Architectures**: MLP, LSTM, and CNN support

**Training Your Own Model:**

```bash
# 1. Collect training data from ROS bags
python3 scripts/collect_training_data.py --bag_dir ./rosbags --output_dir ./data/training

# 2. Train the model
python3 scripts/train_intent_model.py --architecture mlp --epochs 100 --output_dir ./models

# 3. Load in C++
intent_model.loadModel("./models/intent_model_YYYYMMDD_HHMMSS.tflite");
```

See [Training Pipeline Documentation](scripts/README_TRAINING.md) and [Online Learning Guide](docs/ONLINE_LEARNING.md) for details.

## 📚 Documentation

### Getting Started
- **[Getting Started Guide](docs/GETTING_STARTED.md)** - Your first steps with TitanFall AI
- **[Installation Guide](docs/INSTALLATION_GUIDE.md)** - Complete installation instructions
- **[Quick Reference](docs/QUICK_REFERENCE.md)** - API and command reference

### AI & Machine Learning
- **[AI Features Summary](docs/AI_FEATURES_SUMMARY.md)** - Complete overview of AI capabilities
- **[Online Learning Guide](docs/ONLINE_LEARNING.md)** - Deep dive into Q-learning and RL
- **[Training Pipeline](scripts/README_TRAINING.md)** - Data collection and model training

### Examples & Tutorials
- **[Intent Recognition Examples](examples/intent_recognition_example.cpp)** - Four complete usage examples
- **Training Scripts**: `scripts/train_intent_model.py`, `scripts/collect_training_data.py`
- **Testing Script**: `scripts/test_intent_model.py`

### Architecture Diagrams
The documentation includes interactive Mermaid diagrams showing:
- Intent recognition architecture
- Training pipeline workflow
- System component interactions

## Testing

### Run Examples

```bash
# Basic intent recognition demo
ros2 run titanfall_ai intent_recognition_example 1

# Advanced usage with callbacks
ros2 run titanfall_ai intent_recognition_example 2

# Batch training workflow
ros2 run titanfall_ai intent_recognition_example 3

# Real-time intent recognition
ros2 run titanfall_ai intent_recognition_example 4
```

### Test Trained Models

```bash
# Test a trained model
python3 scripts/test_intent_model.py \
    --model_path ./models/intent_model.tflite \
    --data_dir ./data/training \
    --output_dir ./test_results

# View results
# - test_results/confusion_matrix.png
# - test_results/confidence_distribution.png
# - test_results/per_class_accuracy.png
```

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

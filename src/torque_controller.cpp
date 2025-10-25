#include "titanfall_ai/torque_controller.hpp"

namespace titanfall_ai {

TorqueController::TorqueController(rclcpp::Node::SharedPtr node)
    : node_(node), control_mode_("position"), safety_limits_enabled_(true), 
      emergency_stop_(false), control_rate_(100.0) {
    initialize();
}

void TorqueController::initialize() {
    // Initialize publishers
    torque_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("joint_torques", 10);
    command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);
    
    // Initialize subscribers
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&TorqueController::jointStateCallback, this, std::placeholders::_1));
    
    target_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "target_velocity", 10,
        std::bind(&TorqueController::targetCallback, this, std::placeholders::_1));
    
    // Initialize joint names (example joints for exosuit)
    joint_names_ = {"hip_left", "knee_left", "ankle_left", 
                    "hip_right", "knee_right", "ankle_right"};
    
    // Initialize vectors
    current_positions_.resize(joint_names_.size(), 0.0);
    current_velocities_.resize(joint_names_.size(), 0.0);
    target_positions_.resize(joint_names_.size(), 0.0);
    target_velocities_.resize(joint_names_.size(), 0.0);
    output_torques_.resize(joint_names_.size(), 0.0);
    
    // Initialize PID parameters
    for (const auto& joint : joint_names_) {
        PIDParams params;
        params.kp = 100.0;
        params.ki = 10.0;
        params.kd = 5.0;
        params.integral_limit = 50.0;
        params.output_limit = 200.0;
        params.deadband = 0.01;
        pid_params_[joint] = params;
        
        integral_errors_[joint] = 0.0;
        previous_errors_[joint] = 0.0;
        
        // Set joint limits
        JointLimits limits;
        limits.min_position = -3.14;
        limits.max_position = 3.14;
        limits.min_velocity = -10.0;
        limits.max_velocity = 10.0;
        limits.min_torque = -200.0;
        limits.max_torque = 200.0;
        joint_limits_[joint] = limits;
    }
    
    // Initialize timing
    last_update_time_ = node_->now();
    
    loadConfiguration();
    RCLCPP_INFO(node_->get_logger(), "Torque Controller initialized");
}

void TorqueController::update() {
    auto current_time = node_->now();
    auto dt = (current_time - last_update_time_).seconds();
    
    if (dt >= 1.0 / control_rate_) {
        if (!emergency_stop_) {
            updatePIDControllers();
            applySafetyLimits();
        } else {
            // Emergency stop - set all torques to zero
            std::fill(output_torques_.begin(), output_torques_.end(), 0.0);
        }
        
        publishTorqueCommands();
        last_update_time_ = current_time;
    }
}

void TorqueController::setTargetPositions(const std::vector<double>& positions) {
    if (positions.size() == joint_names_.size()) {
        target_positions_ = positions;
    }
}

void TorqueController::setTargetVelocities(const std::vector<double>& velocities) {
    if (velocities.size() == joint_names_.size()) {
        target_velocities_ = velocities;
    }
}

void TorqueController::setTargetTorques(const std::vector<double>& torques) {
    if (torques.size() == joint_names_.size()) {
        output_torques_ = torques;
    }
}

void TorqueController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Update current joint states
    for (size_t i = 0; i < msg->name.size() && i < joint_names_.size(); ++i) {
        if (msg->name[i] == joint_names_[i]) {
            current_positions_[i] = msg->position[i];
            current_velocities_[i] = msg->velocity[i];
        }
    }
}

void TorqueController::targetCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Convert twist to joint targets (simplified)
    // In a real implementation, this would use inverse kinematics
    target_velocities_[0] = msg->linear.x;  // hip_left
    target_velocities_[1] = msg->linear.y; // knee_left
    target_velocities_[2] = msg->linear.z; // ankle_left
    target_velocities_[3] = msg->angular.x; // hip_right
    target_velocities_[4] = msg->angular.y; // knee_right
    target_velocities_[5] = msg->angular.z; // ankle_right
}

std::vector<double> TorqueController::getCurrentTorques() const {
    return output_torques_;
}

std::vector<double> TorqueController::getCurrentPositions() const {
    return current_positions_;
}

std::vector<double> TorqueController::getCurrentVelocities() const {
    return current_velocities_;
}

void TorqueController::setPIDParams(const std::string& joint_name, const PIDParams& params) {
    if (pid_params_.find(joint_name) != pid_params_.end()) {
        pid_params_[joint_name] = params;
    }
}

void TorqueController::setJointLimits(const std::string& joint_name, const JointLimits& limits) {
    if (joint_limits_.find(joint_name) != joint_limits_.end()) {
        joint_limits_[joint_name] = limits;
    }
}

void TorqueController::setControlMode(const std::string& mode) {
    control_mode_ = mode;
}

void TorqueController::enableSafetyLimits(bool enable) {
    safety_limits_enabled_ = enable;
}

void TorqueController::setEmergencyStop(bool stop) {
    emergency_stop_ = stop;
}

bool TorqueController::isEmergencyStop() const {
    return emergency_stop_;
}

void TorqueController::updatePIDControllers() {
    auto current_time = node_->now();
    auto dt = (current_time - last_update_time_).seconds();
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string& joint_name = joint_names_[i];
        double error = 0.0;
        
        if (control_mode_ == "position") {
            error = target_positions_[i] - current_positions_[i];
        } else if (control_mode_ == "velocity") {
            error = target_velocities_[i] - current_velocities_[i];
        }
        
        double torque = calculatePID(joint_name, error, dt);
        output_torques_[i] = torque;
    }
}

double TorqueController::calculatePID(const std::string& joint_name, double error, double dt) {
    const auto& params = pid_params_[joint_name];
    
    // Proportional term
    double p_term = params.kp * error;
    
    // Integral term
    integral_errors_[joint_name] += error * dt;
    integral_errors_[joint_name] = std::clamp(integral_errors_[joint_name], 
                                             -params.integral_limit, params.integral_limit);
    double i_term = params.ki * integral_errors_[joint_name];
    
    // Derivative term
    double d_term = params.kd * (error - previous_errors_[joint_name]) / dt;
    previous_errors_[joint_name] = error;
    
    // Calculate output
    double output = p_term + i_term + d_term;
    
    // Apply deadband
    if (std::abs(output) < params.deadband) {
        output = 0.0;
    }
    
    // Apply output limits
    output = std::clamp(output, -params.output_limit, params.output_limit);
    
    return output;
}

void TorqueController::applySafetyLimits() {
    if (!safety_limits_enabled_) return;
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string& joint_name = joint_names_[i];
        const auto& limits = joint_limits_[joint_name];
        
        // Limit torque output
        output_torques_[i] = std::clamp(output_torques_[i], 
                                      limits.min_torque, limits.max_torque);
        
        // Check position limits
        if (current_positions_[i] < limits.min_position || 
            current_positions_[i] > limits.max_position) {
            RCLCPP_WARN(node_->get_logger(), "Joint %s position limit exceeded", joint_name.c_str());
            output_torques_[i] = 0.0;
        }
        
        // Check velocity limits
        if (std::abs(current_velocities_[i]) > limits.max_velocity) {
            RCLCPP_WARN(node_->get_logger(), "Joint %s velocity limit exceeded", joint_name.c_str());
            output_torques_[i] *= 0.5; // Reduce torque
        }
    }
}

void TorqueController::publishTorqueCommands() {
    // Publish torque commands
    std_msgs::msg::Float64MultiArray torque_msg;
    torque_msg.data = output_torques_;
    torque_pub_->publish(torque_msg);
    
    // Publish joint commands
    sensor_msgs::msg::JointState command_msg;
    command_msg.header.stamp = node_->now();
    command_msg.name = joint_names_;
    command_msg.position = target_positions_;
    command_msg.velocity = target_velocities_;
    command_msg.effort = output_torques_;
    command_pub_->publish(command_msg);
}

void TorqueController::loadConfiguration() {
    // Load configuration from parameters
    // This would typically load from YAML files
    RCLCPP_INFO(node_->get_logger(), "Loading torque controller configuration");
}

} // namespace titanfall_ai

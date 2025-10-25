#ifndef TITANFALL_AI_TORQUE_CONTROLLER_HPP
#define TITANFALL_AI_TORQUE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>

namespace titanfall_ai {

struct PIDParams {
    double kp, ki, kd;
    double integral_limit;
    double output_limit;
    double deadband;
};

struct JointLimits {
    double min_position, max_position;
    double min_velocity, max_velocity;
    double min_torque, max_torque;
};

class TorqueController {
public:
    TorqueController(rclcpp::Node::SharedPtr node);
    ~TorqueController() = default;

    // Initialize torque control system
    void initialize();
    
    // Main control loop
    void update();
    
    // Set target positions/velocities
    void setTargetPositions(const std::vector<double>& positions);
    void setTargetVelocities(const std::vector<double>& velocities);
    void setTargetTorques(const std::vector<double>& torques);
    
    // Input callbacks
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void targetCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    // Get current outputs
    std::vector<double> getCurrentTorques() const;
    std::vector<double> getCurrentPositions() const;
    std::vector<double> getCurrentVelocities() const;
    
    // Configuration
    void setPIDParams(const std::string& joint_name, const PIDParams& params);
    void setJointLimits(const std::string& joint_name, const JointLimits& limits);
    void setControlMode(const std::string& mode); // "position", "velocity", "torque"
    
    // Safety and limits
    void enableSafetyLimits(bool enable);
    void setEmergencyStop(bool stop);
    bool isEmergencyStop() const;

private:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_sub_;
    
    // Control state
    std::vector<std::string> joint_names_;
    std::vector<double> current_positions_;
    std::vector<double> current_velocities_;
    std::vector<double> target_positions_;
    std::vector<double> target_velocities_;
    std::vector<double> output_torques_;
    
    // PID controllers
    std::map<std::string, PIDParams> pid_params_;
    std::map<std::string, double> integral_errors_;
    std::map<std::string, double> previous_errors_;
    std::map<std::string, JointLimits> joint_limits_;
    
    // Control mode
    std::string control_mode_;
    bool safety_limits_enabled_;
    bool emergency_stop_;
    
    // Timing
    rclcpp::Time last_update_time_;
    double control_rate_;
    
    // Internal methods
    void updatePIDControllers();
    double calculatePID(const std::string& joint_name, double error, double dt);
    void applySafetyLimits();
    void publishTorqueCommands();
    void loadConfiguration();
};

} // namespace titanfall_ai

#endif // TITANFALL_AI_TORQUE_CONTROLLER_HPP

#ifndef TITANFALL_AI_ENERGY_MANAGER_HPP
#define TITANFALL_AI_ENERGY_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>

namespace titanfall_ai {

enum class PowerMode {
    ECO_MODE,
    BALANCED_MODE,
    PERFORMANCE_MODE,
    EMERGENCY_MODE
};

struct BatteryStatus {
    double voltage;
    double current;
    double temperature;
    double charge_percentage;
    double health_percentage;
    double time_remaining; // seconds
    bool is_charging;
    bool is_critical;
};

struct PowerConsumption {
    std::string component_name;
    double current_draw;    // Amperes
    double power_usage;     // Watts
    double efficiency;      // 0.0 to 1.0
    bool is_active;
};

class EnergyManager {
public:
    EnergyManager(rclcpp::Node::SharedPtr node);
    ~EnergyManager() = default;

    // Initialize energy management system
    void initialize();
    
    // Main energy management loop
    void update();
    
    // Power management
    void setPowerMode(PowerMode mode);
    PowerMode getCurrentPowerMode() const;
    void setComponentPower(const std::string& component, bool enable);
    void setComponentPowerLimit(const std::string& component, double limit_watts);
    
    // Battery management
    BatteryStatus getBatteryStatus() const;
    void setBatteryThreshold(double threshold_percent);
    void enableBatteryProtection(bool enable);
    
    // Input callbacks
    void batteryCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // Energy monitoring
    std::vector<PowerConsumption> getPowerConsumption() const;
    double getTotalPowerUsage() const;
    double getEstimatedRuntime() const;
    
    // Configuration
    void setPowerLimits(const std::map<std::string, double>& limits);
    void setEfficiencyTargets(const std::map<std::string, double>& targets);
    void enablePowerOptimization(bool enable);

private:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr power_usage_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr low_battery_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr power_mode_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr battery_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    
    // State variables
    BatteryStatus battery_status_;
    PowerMode current_power_mode_;
    std::map<std::string, PowerConsumption> component_power_;
    std::map<std::string, double> power_limits_;
    std::map<std::string, double> efficiency_targets_;
    
    // Power management
    bool power_optimization_enabled_;
    bool battery_protection_enabled_;
    double battery_threshold_;
    double total_power_usage_;
    
    // Timing
    rclcpp::Time last_update_time_;
    double update_rate_;
    
    // Internal methods
    void updatePowerConsumption();
    void updateBatteryStatus();
    void optimizePowerUsage();
    void enforcePowerLimits();
    void calculateRuntime();
    void publishPowerStatus();
    void loadPowerConfiguration();
};

} // namespace titanfall_ai

#endif // TITANFALL_AI_ENERGY_MANAGER_HPP

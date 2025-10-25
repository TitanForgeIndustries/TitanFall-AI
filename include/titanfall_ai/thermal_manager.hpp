#ifndef TITANFALL_AI_THERMAL_MANAGER_HPP
#define TITANFALL_AI_THERMAL_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>

namespace titanfall_ai {

enum class CoolingMode {
    PASSIVE_COOLING,
    ACTIVE_COOLING,
    EMERGENCY_COOLING,
    THERMAL_SHUTDOWN
};

struct ThermalZone {
    std::string name;
    double current_temperature;
    double target_temperature;
    double max_temperature;
    double min_temperature;
    double thermal_capacity;
    double thermal_resistance;
    bool is_critical;
    bool cooling_active;
};

struct CoolingSystem {
    std::string name;
    bool is_active;
    double cooling_capacity;
    double power_consumption;
    double efficiency;
    std::vector<std::string> controlled_zones;
};

class ThermalManager {
public:
    ThermalManager(rclcpp::Node::SharedPtr node);
    ~ThermalManager() = default;

    // Initialize thermal management system
    void initialize();
    
    // Main thermal management loop
    void update();
    
    // Temperature monitoring
    void updateTemperature(const std::string& zone, double temperature);
    double getZoneTemperature(const std::string& zone) const;
    std::vector<ThermalZone> getAllThermalZones() const;
    
    // Cooling control
    void setCoolingMode(CoolingMode mode);
    CoolingMode getCurrentCoolingMode() const;
    void setZoneTargetTemperature(const std::string& zone, double temperature);
    void enableCoolingSystem(const std::string& system, bool enable);
    
    // Input callbacks
    void temperatureCallback(const sensor_msgs::msg::Temperature::SharedPtr msg);
    void powerCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    // Thermal protection
    bool isThermalProtectionActive() const;
    void setThermalLimits(const std::string& zone, double min_temp, double max_temp);
    void enableThermalProtection(bool enable);
    
    // Configuration
    void setThermalParameters(const std::string& zone, double capacity, double resistance);
    void setCoolingParameters(const std::string& system, double capacity, double efficiency);

private:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr avg_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr thermal_warning_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cooling_mode_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr power_sub_;
    
    // Thermal state
    std::map<std::string, ThermalZone> thermal_zones_;
    std::map<std::string, CoolingSystem> cooling_systems_;
    CoolingMode current_cooling_mode_;
    
    // Thermal management
    bool thermal_protection_enabled_;
    double global_max_temperature_;
    double thermal_warning_threshold_;
    double thermal_critical_threshold_;
    
    // Timing
    rclcpp::Time last_update_time_;
    double update_rate_;
    
    // Internal methods
    void updateThermalModel();
    void updateCoolingSystems();
    void checkThermalLimits();
    void activateThermalProtection();
    void publishThermalStatus();
    void loadThermalConfiguration();
};

} // namespace titanfall_ai

#endif // TITANFALL_AI_THERMAL_MANAGER_HPP

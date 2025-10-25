#include "titanfall_ai/energy_manager.hpp"

namespace titanfall_ai {

EnergyManager::EnergyManager(rclcpp::Node::SharedPtr node)
    : node_(node), current_power_mode_(PowerMode::BALANCED_MODE), 
      power_optimization_enabled_(true), battery_protection_enabled_(true),
      battery_threshold_(20.0), total_power_usage_(0.0), update_rate_(10.0) {
    initialize();
}

void EnergyManager::initialize() {
    // Initialize publishers
    power_usage_pub_ = node_->create_publisher<std_msgs::msg::Float64>("total_power_usage", 10);
    low_battery_pub_ = node_->create_publisher<std_msgs::msg::Bool>("low_battery_warning", 10);
    power_mode_pub_ = node_->create_publisher<std_msgs::msg::String>("power_mode", 10);
    
    // Initialize subscribers
    battery_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "battery_voltage", 10,
        std::bind(&EnergyManager::batteryCallback, this, std::placeholders::_1));
    
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&EnergyManager::jointStateCallback, this, std::placeholders::_1));
    
    // Initialize battery status
    battery_status_.voltage = 24.0;
    battery_status_.current = 0.0;
    battery_status_.temperature = 25.0;
    battery_status_.charge_percentage = 100.0;
    battery_status_.health_percentage = 100.0;
    battery_status_.time_remaining = 3600.0; // 1 hour
    battery_status_.is_charging = false;
    battery_status_.is_critical = false;
    
    // Initialize component power consumption
    std::vector<std::string> components = {
        "sensor_fusion", "intent_model", "torque_controller", 
        "energy_manager", "fault_detection", "thermal_manager",
        "actuators", "sensors", "computing"
    };
    
    for (const auto& component : components) {
        PowerConsumption consumption;
        consumption.component_name = component;
        consumption.current_draw = 0.0;
        consumption.power_usage = 0.0;
        consumption.efficiency = 0.8;
        consumption.is_active = true;
        component_power_[component] = consumption;
        
        power_limits_[component] = 50.0; // 50W default limit
        efficiency_targets_[component] = 0.8; // 80% efficiency target
    }
    
    // Initialize timing
    last_update_time_ = node_->now();
    
    loadPowerConfiguration();
    RCLCPP_INFO(node_->get_logger(), "Energy Manager initialized");
}

void EnergyManager::update() {
    auto current_time = node_->now();
    auto dt = (current_time - last_update_time_).seconds();
    
    if (dt >= 1.0 / update_rate_) {
        updatePowerConsumption();
        updateBatteryStatus();
        
        if (power_optimization_enabled_) {
            optimizePowerUsage();
        }
        
        enforcePowerLimits();
        calculateRuntime();
        publishPowerStatus();
        
        last_update_time_ = current_time;
    }
}

void EnergyManager::setPowerMode(PowerMode mode) {
    current_power_mode_ = mode;
    
    // Adjust power limits based on mode
    switch (mode) {
        case PowerMode::ECO_MODE:
            for (auto& limit : power_limits_) {
                limit.second *= 0.5; // Reduce power limits by 50%
            }
            break;
        case PowerMode::BALANCED_MODE:
            // Use default limits
            break;
        case PowerMode::PERFORMANCE_MODE:
            for (auto& limit : power_limits_) {
                limit.second *= 1.5; // Increase power limits by 50%
            }
            break;
        case PowerMode::EMERGENCY_MODE:
            for (auto& limit : power_limits_) {
                limit.second *= 0.2; // Minimal power usage
            }
            break;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Power mode changed to: %d", static_cast<int>(mode));
}

PowerMode EnergyManager::getCurrentPowerMode() const {
    return current_power_mode_;
}

void EnergyManager::setComponentPower(const std::string& component, bool enable) {
    if (component_power_.find(component) != component_power_.end()) {
        component_power_[component].is_active = enable;
    }
}

void EnergyManager::setComponentPowerLimit(const std::string& component, double limit_watts) {
    power_limits_[component] = limit_watts;
}

BatteryStatus EnergyManager::getBatteryStatus() const {
    return battery_status_;
}

void EnergyManager::setBatteryThreshold(double threshold_percent) {
    battery_threshold_ = threshold_percent;
}

void EnergyManager::enableBatteryProtection(bool enable) {
    battery_protection_enabled_ = enable;
}

void EnergyManager::batteryCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    battery_status_.voltage = msg->data;
    
    // Estimate current based on power usage
    battery_status_.current = total_power_usage_ / battery_status_.voltage;
    
    // Update charge percentage (simplified calculation)
    battery_status_.charge_percentage = (battery_status_.voltage / 24.0) * 100.0;
    battery_status_.charge_percentage = std::clamp(battery_status_.charge_percentage, 0.0, 100.0);
    
    // Check if battery is critical
    battery_status_.is_critical = battery_status_.charge_percentage < battery_threshold_;
}

void EnergyManager::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Calculate actuator power consumption based on joint effort
    double actuator_power = 0.0;
    for (const auto& effort : msg->effort) {
        actuator_power += std::abs(effort) * 0.1; // Simplified power calculation
    }
    
    if (component_power_.find("actuators") != component_power_.end()) {
        component_power_["actuators"].power_usage = actuator_power;
        component_power_["actuators"].current_draw = actuator_power / battery_status_.voltage;
    }
}

std::vector<PowerConsumption> EnergyManager::getPowerConsumption() const {
    std::vector<PowerConsumption> consumption;
    for (const auto& pair : component_power_) {
        consumption.push_back(pair.second);
    }
    return consumption;
}

double EnergyManager::getTotalPowerUsage() const {
    return total_power_usage_;
}

double EnergyManager::getEstimatedRuntime() const {
    return battery_status_.time_remaining;
}

void EnergyManager::setPowerLimits(const std::map<std::string, double>& limits) {
    power_limits_ = limits;
}

void EnergyManager::setEfficiencyTargets(const std::map<std::string, double>& targets) {
    efficiency_targets_ = targets;
}

void EnergyManager::enablePowerOptimization(bool enable) {
    power_optimization_enabled_ = enable;
}

void EnergyManager::updatePowerConsumption() {
    total_power_usage_ = 0.0;
    
    for (auto& pair : component_power_) {
        PowerConsumption& consumption = pair.second;
        
        if (consumption.is_active) {
            // Update power usage based on component activity
            consumption.power_usage = consumption.current_draw * battery_status_.voltage;
            total_power_usage_ += consumption.power_usage;
        }
    }
}

void EnergyManager::updateBatteryStatus() {
    // Update battery health and remaining time
    battery_status_.health_percentage = std::max(0.0, battery_status_.health_percentage - 0.001);
    
    if (total_power_usage_ > 0.0) {
        double remaining_capacity = (battery_status_.charge_percentage / 100.0) * 100.0; // Wh
        battery_status_.time_remaining = (remaining_capacity / total_power_usage_) * 3600.0; // seconds
    } else {
        battery_status_.time_remaining = 999999.0; // Infinite if no power usage
    }
}

void EnergyManager::optimizePowerUsage() {
    // Optimize power usage based on current mode and battery status
    if (battery_status_.is_critical) {
        // Reduce power usage of non-critical components
        setComponentPower("intent_model", false);
        setComponentPower("fault_detection", false);
    }
    
    // Adjust efficiency targets based on power mode
    double efficiency_multiplier = 1.0;
    switch (current_power_mode_) {
        case PowerMode::ECO_MODE:
            efficiency_multiplier = 1.2;
            break;
        case PowerMode::PERFORMANCE_MODE:
            efficiency_multiplier = 0.8;
            break;
        default:
            efficiency_multiplier = 1.0;
            break;
    }
    
    for (auto& pair : efficiency_targets_) {
        pair.second *= efficiency_multiplier;
        pair.second = std::clamp(pair.second, 0.5, 1.0);
    }
}

void EnergyManager::enforcePowerLimits() {
    for (auto& pair : component_power_) {
        const std::string& component = pair.first;
        PowerConsumption& consumption = pair.second;
        
        if (consumption.power_usage > power_limits_[component]) {
            RCLCPP_WARN(node_->get_logger(), "Component %s exceeding power limit", component.c_str());
            consumption.is_active = false; // Disable component
        }
    }
}

void EnergyManager::calculateRuntime() {
    // Runtime calculation is done in updateBatteryStatus()
}

void EnergyManager::publishPowerStatus() {
    // Publish total power usage
    std_msgs::msg::Float64 power_msg;
    power_msg.data = total_power_usage_;
    power_usage_pub_->publish(power_msg);
    
    // Publish low battery warning
    std_msgs::msg::Bool battery_msg;
    battery_msg.data = battery_status_.is_critical;
    low_battery_pub_->publish(battery_msg);
    
    // Publish power mode
    std_msgs::msg::String mode_msg;
    mode_msg.data = std::to_string(static_cast<int>(current_power_mode_));
    power_mode_pub_->publish(mode_msg);
}

void EnergyManager::loadPowerConfiguration() {
    RCLCPP_INFO(node_->get_logger(), "Loading power configuration");
}

} // namespace titanfall_ai

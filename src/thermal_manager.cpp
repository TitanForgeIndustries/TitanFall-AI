#include "titanfall_ai/thermal_manager.hpp"

namespace titanfall_ai {

ThermalManager::ThermalManager(rclcpp::Node::SharedPtr node)
    : node_(node), current_cooling_mode_(CoolingMode::PASSIVE_COOLING),
      thermal_protection_enabled_(true), global_max_temperature_(80.0),
      thermal_warning_threshold_(60.0), thermal_critical_threshold_(75.0),
      update_rate_(5.0) {
    initialize();
}

void ThermalManager::initialize() {
    // Initialize publishers
    avg_temp_pub_ = node_->create_publisher<std_msgs::msg::Float64>("average_temperature", 10);
    thermal_warning_pub_ = node_->create_publisher<std_msgs::msg::Bool>("thermal_warning", 10);
    cooling_mode_pub_ = node_->create_publisher<std_msgs::msg::String>("cooling_mode", 10);
    
    // Initialize subscribers
    temp_sub_ = node_->create_subscription<sensor_msgs::msg::Temperature>(
        "temperature_data", 10,
        std::bind(&ThermalManager::temperatureCallback, this, std::placeholders::_1));
    
    power_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "power_usage", 10,
        std::bind(&ThermalManager::powerCallback, this, std::placeholders::_1));
    
    // Initialize thermal zones
    std::vector<std::string> zones = {
        "cpu", "actuators", "battery", "sensors", "motors", "electronics"
    };
    
    for (const auto& zone : zones) {
        ThermalZone thermal_zone;
        thermal_zone.name = zone;
        thermal_zone.current_temperature = 25.0; // Room temperature
        thermal_zone.target_temperature = 30.0;
        thermal_zone.max_temperature = 70.0;
        thermal_zone.min_temperature = -10.0;
        thermal_zone.thermal_capacity = 100.0; // J/K
        thermal_zone.thermal_resistance = 0.1; // K/W
        thermal_zone.is_critical = false;
        thermal_zone.cooling_active = false;
        thermal_zones_[zone] = thermal_zone;
    }
    
    // Initialize cooling systems
    std::vector<std::string> cooling_systems = {
        "fan_system", "liquid_cooling", "heat_sink"
    };
    
    for (const auto& system : cooling_systems) {
        CoolingSystem cooling_system;
        cooling_system.name = system;
        cooling_system.is_active = false;
        cooling_system.cooling_capacity = 50.0; // W/K
        cooling_system.power_consumption = 10.0; // W
        cooling_system.efficiency = 0.8;
        
        if (system == "fan_system") {
            cooling_system.controlled_zones = {"cpu", "electronics"};
        } else if (system == "liquid_cooling") {
            cooling_system.controlled_zones = {"actuators", "motors"};
        } else if (system == "heat_sink") {
            cooling_system.controlled_zones = {"battery", "sensors"};
        }
        
        cooling_systems_[system] = cooling_system;
    }
    
    // Initialize timing
    last_update_time_ = node_->now();
    
    loadThermalConfiguration();
    RCLCPP_INFO(node_->get_logger(), "Thermal Manager initialized");
}

void ThermalManager::update() {
    auto current_time = node_->now();
    auto dt = (current_time - last_update_time_).seconds();
    
    if (dt >= 1.0 / update_rate_) {
        updateThermalModel();
        updateCoolingSystems();
        checkThermalLimits();
        
        if (thermal_protection_enabled_) {
            activateThermalProtection();
        }
        
        publishThermalStatus();
        last_update_time_ = current_time;
    }
}

void ThermalManager::updateTemperature(const std::string& zone, double temperature) {
    if (thermal_zones_.find(zone) != thermal_zones_.end()) {
        thermal_zones_[zone].current_temperature = temperature;
    }
}

double ThermalManager::getZoneTemperature(const std::string& zone) const {
    if (thermal_zones_.find(zone) != thermal_zones_.end()) {
        return thermal_zones_.at(zone).current_temperature;
    }
    return 25.0; // Default room temperature
}

std::vector<ThermalZone> ThermalManager::getAllThermalZones() const {
    std::vector<ThermalZone> zones;
    for (const auto& pair : thermal_zones_) {
        zones.push_back(pair.second);
    }
    return zones;
}

void ThermalManager::setCoolingMode(CoolingMode mode) {
    current_cooling_mode_ = mode;
    
    // Adjust cooling systems based on mode
    switch (mode) {
        case CoolingMode::PASSIVE_COOLING:
            for (auto& pair : cooling_systems_) {
                pair.second.is_active = false;
            }
            break;
        case CoolingMode::ACTIVE_COOLING:
            for (auto& pair : cooling_systems_) {
                if (pair.first == "fan_system") {
                    pair.second.is_active = true;
                }
            }
            break;
        case CoolingMode::EMERGENCY_COOLING:
            for (auto& pair : cooling_systems_) {
                pair.second.is_active = true;
            }
            break;
        case CoolingMode::THERMAL_SHUTDOWN:
            // Shutdown cooling systems
            for (auto& pair : cooling_systems_) {
                pair.second.is_active = false;
            }
            break;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Cooling mode changed to: %d", static_cast<int>(mode));
}

CoolingMode ThermalManager::getCurrentCoolingMode() const {
    return current_cooling_mode_;
}

void ThermalManager::setZoneTargetTemperature(const std::string& zone, double temperature) {
    if (thermal_zones_.find(zone) != thermal_zones_.end()) {
        thermal_zones_[zone].target_temperature = temperature;
    }
}

void ThermalManager::enableCoolingSystem(const std::string& system, bool enable) {
    if (cooling_systems_.find(system) != cooling_systems_.end()) {
        cooling_systems_[system].is_active = enable;
    }
}

void ThermalManager::temperatureCallback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    // Update temperature for the zone specified in the message
    std::string zone = msg->header.frame_id;
    if (zone.empty()) {
        zone = "cpu"; // Default zone
    }
    
    updateTemperature(zone, msg->temperature);
}

void ThermalManager::powerCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    // Update thermal model based on power consumption
    // Higher power consumption leads to higher heat generation
    double heat_generation = msg->data * 0.1; // Simplified heat generation model
    
    // Distribute heat to different zones based on power consumption
    for (auto& pair : thermal_zones_) {
        ThermalZone& zone = pair.second;
        zone.current_temperature += heat_generation / zone.thermal_capacity;
    }
}

bool ThermalManager::isThermalProtectionActive() const {
    return current_cooling_mode_ == CoolingMode::EMERGENCY_COOLING ||
           current_cooling_mode_ == CoolingMode::THERMAL_SHUTDOWN;
}

void ThermalManager::setThermalLimits(const std::string& zone, double min_temp, double max_temp) {
    if (thermal_zones_.find(zone) != thermal_zones_.end()) {
        thermal_zones_[zone].min_temperature = min_temp;
        thermal_zones_[zone].max_temperature = max_temp;
    }
}

void ThermalManager::enableThermalProtection(bool enable) {
    thermal_protection_enabled_ = enable;
}

void ThermalManager::setThermalParameters(const std::string& zone, double capacity, double resistance) {
    if (thermal_zones_.find(zone) != thermal_zones_.end()) {
        thermal_zones_[zone].thermal_capacity = capacity;
        thermal_zones_[zone].thermal_resistance = resistance;
    }
}

void ThermalManager::setCoolingParameters(const std::string& system, double capacity, double efficiency) {
    if (cooling_systems_.find(system) != cooling_systems_.end()) {
        cooling_systems_[system].cooling_capacity = capacity;
        cooling_systems_[system].efficiency = efficiency;
    }
}

void ThermalManager::updateThermalModel() {
    // Update thermal model for each zone
    for (auto& pair : thermal_zones_) {
        ThermalZone& zone = pair.second;
        
        // Calculate heat dissipation
        double heat_dissipation = 0.0;
        for (const auto& cooling_pair : cooling_systems_) {
            const CoolingSystem& cooling = cooling_pair.second;
            
            // Check if this cooling system controls this zone
            if (std::find(cooling.controlled_zones.begin(), cooling.controlled_zones.end(), zone.name) 
                != cooling.controlled_zones.end()) {
                
                if (cooling.is_active) {
                    double temp_diff = zone.current_temperature - zone.target_temperature;
                    heat_dissipation += cooling.cooling_capacity * cooling.efficiency * temp_diff;
                }
            }
        }
        
        // Update temperature based on heat dissipation
        zone.current_temperature -= heat_dissipation / zone.thermal_capacity;
        
        // Apply thermal resistance
        zone.current_temperature = zone.target_temperature + 
            (zone.current_temperature - zone.target_temperature) * zone.thermal_resistance;
    }
}

void ThermalManager::updateCoolingSystems() {
    // Update cooling system states based on zone temperatures
    for (auto& pair : cooling_systems_) {
        CoolingSystem& cooling = pair.second;
        
        bool should_activate = false;
        for (const std::string& zone_name : cooling.controlled_zones) {
            if (thermal_zones_.find(zone_name) != thermal_zones_.end()) {
                const ThermalZone& zone = thermal_zones_[zone_name];
                if (zone.current_temperature > zone.target_temperature + 5.0) {
                    should_activate = true;
                    break;
                }
            }
        }
        
        cooling.is_active = should_activate;
    }
}

void ThermalManager::checkThermalLimits() {
    bool thermal_warning = false;
    bool thermal_critical = false;
    
    for (auto& pair : thermal_zones_) {
        ThermalZone& zone = pair.second;
        
        if (zone.current_temperature > thermal_warning_threshold_) {
            thermal_warning = true;
        }
        
        if (zone.current_temperature > thermal_critical_threshold_) {
            thermal_critical = true;
            zone.is_critical = true;
        } else {
            zone.is_critical = false;
        }
        
        // Check zone-specific limits
        if (zone.current_temperature > zone.max_temperature) {
            RCLCPP_ERROR(node_->get_logger(), "Zone %s exceeded maximum temperature: %.2f°C", 
                        zone.name.c_str(), zone.current_temperature);
        }
        
        if (zone.current_temperature < zone.min_temperature) {
            RCLCPP_WARN(node_->get_logger(), "Zone %s below minimum temperature: %.2f°C", 
                       zone.name.c_str(), zone.current_temperature);
        }
    }
    
    // Update cooling mode based on thermal conditions
    if (thermal_critical) {
        setCoolingMode(CoolingMode::EMERGENCY_COOLING);
    } else if (thermal_warning) {
        setCoolingMode(CoolingMode::ACTIVE_COOLING);
    } else {
        setCoolingMode(CoolingMode::PASSIVE_COOLING);
    }
}

void ThermalManager::activateThermalProtection() {
    if (isThermalProtectionActive()) {
        RCLCPP_WARN(node_->get_logger(), "Thermal protection activated");
        
        // In a real system, this would trigger safety measures like:
        // - Reducing power consumption
        // - Shutting down non-critical systems
        // - Alerting operators
    }
}

void ThermalManager::publishThermalStatus() {
    // Calculate average temperature
    double total_temp = 0.0;
    int zone_count = 0;
    
    for (const auto& pair : thermal_zones_) {
        total_temp += pair.second.current_temperature;
        zone_count++;
    }
    
    double avg_temp = zone_count > 0 ? total_temp / zone_count : 25.0;
    
    // Publish average temperature
    std_msgs::msg::Float64 temp_msg;
    temp_msg.data = avg_temp;
    avg_temp_pub_->publish(temp_msg);
    
    // Publish thermal warning
    std_msgs::msg::Bool warning_msg;
    warning_msg.data = avg_temp > thermal_warning_threshold_;
    thermal_warning_pub_->publish(warning_msg);
    
    // Publish cooling mode
    std_msgs::msg::String mode_msg;
    mode_msg.data = std::to_string(static_cast<int>(current_cooling_mode_));
    cooling_mode_pub_->publish(mode_msg);
}

void ThermalManager::loadThermalConfiguration() {
    RCLCPP_INFO(node_->get_logger(), "Loading thermal configuration");
}

} // namespace titanfall_ai

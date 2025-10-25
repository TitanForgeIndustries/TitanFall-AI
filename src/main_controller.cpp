#include "titanfall_ai/main_controller.hpp"

namespace titanfall_ai {

MainController::MainController()
    : current_state_(SystemState::INITIALIZING), current_control_mode_(ControlMode::MANUAL),
      system_initialized_(false), system_running_(false), main_loop_rate_(50.0) {
    // Initialize ROS node
    node_ = rclcpp::Node::make_shared("titanfall_ai_main_controller");
}

void MainController::initialize() {
    RCLCPP_INFO(node_->get_logger(), "Initializing Titan Fall AI System...");
    
    // Load configuration
    loadDefaultConfiguration();
    
    // Initialize subsystems
    initializeSubsystems();
    
    // Validate configuration
    validateConfiguration();
    
    system_initialized_ = true;
    current_state_ = SystemState::READY;
    
    RCLCPP_INFO(node_->get_logger(), "Titan Fall AI System initialized successfully");
}

void MainController::run() {
    if (!system_initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "System not initialized. Call initialize() first.");
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Starting Titan Fall AI System main loop...");
    
    rclcpp::Rate rate(main_loop_rate_);
    
    while (rclcpp::ok() && system_running_) {
        updateSystemState();
        checkSystemHealth();
        handleSystemErrors();
        publishSystemStatus();
        
        rclcpp::spin_some(node_);
        rate.sleep();
    }
    
    RCLCPP_INFO(node_->get_logger(), "Titan Fall AI System main loop stopped");
}

void MainController::start() {
    if (!system_initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "System not initialized. Call initialize() first.");
        return;
    }
    
    system_running_ = true;
    current_state_ = SystemState::RUNNING;
    
    RCLCPP_INFO(node_->get_logger(), "Titan Fall AI System started");
}

void MainController::stop() {
    system_running_ = false;
    current_state_ = SystemState::READY;
    
    RCLCPP_INFO(node_->get_logger(), "Titan Fall AI System stopped");
}

void MainController::pause() {
    if (system_running_) {
        system_running_ = false;
        current_state_ = SystemState::PAUSED;
        RCLCPP_INFO(node_->get_logger(), "Titan Fall AI System paused");
    }
}

void MainController::resume() {
    if (current_state_ == SystemState::PAUSED) {
        system_running_ = true;
        current_state_ = SystemState::RUNNING;
        RCLCPP_INFO(node_->get_logger(), "Titan Fall AI System resumed");
    }
}

void MainController::emergencyStop() {
    system_running_ = false;
    current_state_ = SystemState::EMERGENCY_STOP;
    
    // Stop all subsystems
    if (torque_controller_) {
        torque_controller_->setEmergencyStop(true);
    }
    
    RCLCPP_ERROR(node_->get_logger(), "EMERGENCY STOP ACTIVATED");
}

void MainController::shutdown() {
    system_running_ = false;
    current_state_ = SystemState::SHUTDOWN;
    
    RCLCPP_INFO(node_->get_logger(), "Titan Fall AI System shutdown");
}

SystemState MainController::getCurrentState() const {
    return current_state_;
}

ControlMode MainController::getCurrentControlMode() const {
    return current_control_mode_;
}

void MainController::setControlMode(ControlMode mode) {
    current_control_mode_ = mode;
    RCLCPP_INFO(node_->get_logger(), "Control mode changed to: %d", static_cast<int>(mode));
}

std::shared_ptr<SensorFusion> MainController::getSensorFusion() {
    return sensor_fusion_;
}

std::shared_ptr<IntentModel> MainController::getIntentModel() {
    return intent_model_;
}

std::shared_ptr<TorqueController> MainController::getTorqueController() {
    return torque_controller_;
}

std::shared_ptr<EnergyManager> MainController::getEnergyManager() {
    return energy_manager_;
}

std::shared_ptr<FaultDetection> MainController::getFaultDetection() {
    return fault_detection_;
}

std::shared_ptr<ThermalManager> MainController::getThermalManager() {
    return thermal_manager_;
}

bool MainController::isSystemHealthy() const {
    // Check if all subsystems are healthy
    if (fault_detection_) {
        return !fault_detection_->hasCriticalFaults();
    }
    return true;
}

std::vector<std::string> MainController::getActiveWarnings() const {
    std::vector<std::string> warnings;
    
    if (fault_detection_) {
        auto faults = fault_detection_->getActiveFaults();
        for (const auto& fault : faults) {
            if (fault.severity == FaultSeverity::WARNING) {
                warnings.push_back(fault.component + ": " + fault.description);
            }
        }
    }
    
    return warnings;
}

std::vector<std::string> MainController::getActiveErrors() const {
    std::vector<std::string> errors;
    
    if (fault_detection_) {
        auto faults = fault_detection_->getActiveFaults();
        for (const auto& fault : faults) {
            if (fault.severity == FaultSeverity::ERROR || 
                fault.severity == FaultSeverity::CRITICAL ||
                fault.severity == FaultSeverity::FATAL) {
                errors.push_back(fault.component + ": " + fault.description);
            }
        }
    }
    
    return errors;
}

void MainController::loadConfiguration(const std::string& config_path) {
    RCLCPP_INFO(node_->get_logger(), "Loading configuration from: %s", config_path.c_str());
    // Implement configuration loading from file
}

void MainController::saveConfiguration(const std::string& config_path) {
    RCLCPP_INFO(node_->get_logger(), "Saving configuration to: %s", config_path.c_str());
    // Implement configuration saving to file
}

void MainController::setParameter(const std::string& subsystem, const std::string& parameter, const std::string& value) {
    configuration_[subsystem][parameter] = value;
    RCLCPP_INFO(node_->get_logger(), "Set parameter %s.%s = %s", 
               subsystem.c_str(), parameter.c_str(), value.c_str());
}

void MainController::initializeSubsystems() {
    RCLCPP_INFO(node_->get_logger(), "Initializing subsystems...");
    
    // Initialize sensor fusion
    sensor_fusion_ = std::make_shared<SensorFusion>(node_);
    sensor_fusion_->initialize();
    
    // Initialize intent model
    intent_model_ = std::make_shared<IntentModel>(node_);
    intent_model_->initialize();
    
    // Initialize torque controller
    torque_controller_ = std::make_shared<TorqueController>(node_);
    torque_controller_->initialize();
    
    // Initialize energy manager
    energy_manager_ = std::make_shared<EnergyManager>(node_);
    energy_manager_->initialize();
    
    // Initialize fault detection
    fault_detection_ = std::make_shared<FaultDetection>(node_);
    fault_detection_->initialize();
    
    // Initialize thermal manager
    thermal_manager_ = std::make_shared<ThermalManager>(node_);
    thermal_manager_->initialize();
    
    RCLCPP_INFO(node_->get_logger(), "All subsystems initialized");
}

void MainController::updateSystemState() {
    // Update all subsystems
    if (sensor_fusion_) {
        sensor_fusion_->process();
    }
    
    if (intent_model_) {
        intent_model_->process();
    }
    
    if (torque_controller_) {
        torque_controller_->update();
    }
    
    if (energy_manager_) {
        energy_manager_->update();
    }
    
    if (fault_detection_) {
        fault_detection_->update();
    }
    
    if (thermal_manager_) {
        thermal_manager_->update();
    }
}

void MainController::checkSystemHealth() {
    if (!isSystemHealthy()) {
        current_state_ = SystemState::ERROR;
        RCLCPP_ERROR(node_->get_logger(), "System health check failed");
    }
}

void MainController::handleSystemErrors() {
    if (current_state_ == SystemState::ERROR) {
        auto errors = getActiveErrors();
        for (const auto& error : errors) {
            RCLCPP_ERROR(node_->get_logger(), "System error: %s", error.c_str());
        }
        
        // Implement error handling logic
        // For now, just log the errors
    }
}

void MainController::publishSystemStatus() {
    // Publish system status information
    // This would typically publish to ROS topics
    static int status_counter = 0;
    if (++status_counter % 100 == 0) { // Every 2 seconds at 50Hz
        RCLCPP_INFO(node_->get_logger(), "System Status - State: %d, Mode: %d, Healthy: %s",
                   static_cast<int>(current_state_), static_cast<int>(current_control_mode_),
                   isSystemHealthy() ? "Yes" : "No");
    }
}

void MainController::loadDefaultConfiguration() {
    RCLCPP_INFO(node_->get_logger(), "Loading default configuration");
    
    // Set default parameters for each subsystem
    configuration_["sensor_fusion"]["update_rate"] = "100.0";
    configuration_["sensor_fusion"]["alpha"] = "0.98";
    configuration_["sensor_fusion"]["beta"] = "0.02";
    
    configuration_["intent_model"]["prediction_horizon"] = "2.0";
    configuration_["intent_model"]["confidence_threshold"] = "0.7";
    
    configuration_["torque_controller"]["control_rate"] = "100.0";
    configuration_["torque_controller"]["control_mode"] = "position";
    
    configuration_["energy_manager"]["update_rate"] = "10.0";
    configuration_["energy_manager"]["battery_threshold"] = "20.0";
    
    configuration_["fault_detection"]["detection_rate"] = "10.0";
    configuration_["fault_detection"]["sensitivity"] = "1.0";
    
    configuration_["thermal_manager"]["update_rate"] = "5.0";
    configuration_["thermal_manager"]["max_temperature"] = "80.0";
}

void MainController::validateConfiguration() {
    RCLCPP_INFO(node_->get_logger(), "Validating configuration...");
    
    // Validate that all required parameters are set
    std::vector<std::string> required_subsystems = {
        "sensor_fusion", "intent_model", "torque_controller",
        "energy_manager", "fault_detection", "thermal_manager"
    };
    
    for (const auto& subsystem : required_subsystems) {
        if (configuration_.find(subsystem) == configuration_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Missing configuration for subsystem: %s", subsystem.c_str());
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Configuration validation complete");
}

} // namespace titanfall_ai
